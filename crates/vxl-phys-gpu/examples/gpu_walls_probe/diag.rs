//! diag：`gpu_walls_probe` 的**诊断共件**（子档，2026-09-24 拆出——主档触到 800 行阈值）。
//!
//! 装的是"两侧 A/B 都用到的那套东西"：卡上链（`Sim` / `chain_sim` / `chain_sim_dens`）、
//! 判据聚合（底层均高 / 分箱）与 **②b 的 2b 容器对照**。主档保留场景构建（`tank`/`water`/
//! `make_cfg`/`flat`）、① 密度对拍、`main` 与 `facade_path`，以及 ②c 之后的各条诊断。

use super::{flat, make_cfg, water};
use vxl_phys::{Quat, Shape, Vec3};
use vxl_phys_core::interop::ProviderColliders;
use vxl_phys_fluid::FluidSystem;
use vxl_phys_gpu::pipeline::{Packet, PacketCfg, WallSide, WallStage};

/// **底层粒子平均高度**（按 y 取最低 25%）——静置仿真里"站没站住"的判据。
pub fn bottom_mean_y(ps: &[Vec3], n_fluid: usize) -> f32 {
    let mut ys: Vec<f32> = ps[..n_fluid.min(ps.len())].iter().map(|p| p.y).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let k = (ys.len() / 4).max(1);
    ys[..k].iter().sum::<f32>() / k as f32
}

/// **按"到最近壁面的水平距离"分箱的平均高度**（腔体半宽 0.25 ⇒ 距离 = 0.25 − max(|x|,|z|)）。
/// 用途：把"水位差 2.8 mm"定位到**近壁带**还是**内部**——近壁为主 ⇒ 投影/镜像的局部效应；
/// 全箱均匀 ⇒ 鬼影的**全局补偿**偏差（它加的质量整体抬高压力）。
/// ⚠️ **这几箱是混沌量**（§15 补记七：CPU 给自己加 1e-8 就给出同量级同签名的差）⇒ 只当参照。
pub fn band_profile(ps: &[Vec3], n: usize) -> [(f32, usize); 3] {
    let mut acc = [(0.0f32, 0usize); 3];
    for p in ps.iter().take(n) {
        let d = 0.25 - p.x.abs().max(p.z.abs());
        let k = if d < 0.01 {
            0
        } else if d < 0.05 {
            1
        } else {
            2
        };
        acc[k].0 += p.y;
        acc[k].1 += 1;
    }
    for a in acc.iter_mut() {
        if a.1 > 0 {
            a.0 /= a.1 as f32;
        }
    }
    acc
}

/// **②b 对照：同一场景换 2b 容器**（静态盒体当墙 ⇒ **无 provider** ⇒ 壁面档不参与）。
/// 判据用途：它在 150 tick 上是 **6.79e-6 m**（已实测）⇒ "口径 B 的宏观噪声底"被证否，
/// provider 容器确实多出 2.8 mm 级差（那条差的归属见 §15 补记七/九/十）。
pub fn sim_2b(adapter: usize, substeps: usize, h: f32, ticks: usize) {
    let t = 0.05f32;
    let half = 0.25f32;
    let pose = |p: Vec3| vxl_phys_fluid::BodyPose {
        pos: p,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    };
    let bodies = vec![
        (
            0u32,
            Shape::Box {
                half: Vec3::new(0.6, t, 0.6),
            },
            pose(Vec3::new(0.0, 1.0 - t, 0.0)),
        ),
        (
            1,
            Shape::Box {
                half: Vec3::new(t, 0.6, 0.6),
            },
            pose(Vec3::new(half + t, 1.2, 0.0)),
        ),
        (
            2,
            Shape::Box {
                half: Vec3::new(t, 0.6, 0.6),
            },
            pose(Vec3::new(-(half + t), 1.2, 0.0)),
        ),
        (
            3,
            Shape::Box {
                half: Vec3::new(0.6, 0.6, t),
            },
            pose(Vec3::new(0.0, 1.2, half + t)),
        ),
        (
            4,
            Shape::Box {
                half: Vec3::new(0.6, 0.6, t),
            },
            pose(Vec3::new(0.0, 1.2, -(half + t))),
        ),
    ];
    let mut cpu = water();
    cpu.set_boundary_particles(&bodies);
    // ⚠️ 本场景的 `cfg` 必须**按本场景自己算**（`n`/`n_fluid` 要含 2b 边界粒子——
    // 借用 provider 场景那份会把边界粒子在建包时丢掉 ⇒ 水没人托、直接穿地，实测 −18 m ✗）。
    let cfg = make_cfg(&cpu);
    let init = flat(&cpu);
    let n = cpu.len();
    for _ in 0..ticks {
        cpu.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let a = cpu.positions().to_vec();
    let mk = Sim {
        init: &init,
        cfg,
        substeps,
        ticks,
        ghost: false,
        project: false,
        ids: Vec::new(),
        h,
        adapter,
    };
    let Some(b) = chain_sim(&mk, &vxl_phys_core::interop::NoProviders) else {
        println!("  ── ②b 2b 容器对照：没起 GPU 管线（跳过）");
        return;
    };
    let (ya, yb) = (bottom_mean_y(&a, n), bottom_mean_y(&b, n));
    println!("  ── ②b **对照：同一场景换 2b 容器**（静态盒体当墙、无 provider，{ticks} tick）──");
    println!(
        "     底层平均 y：CPU {ya:.5} vs 卡上 {yb:.5}（差 **{:.2e} m**）⇒ 与 provider 容器的 2.77e-3 比",
        (ya - yb).abs()
    );
}

/// 静置仿真的参数（避免长参数表）。
#[derive(Clone)]
pub struct Sim<'a> {
    pub init: &'a (Vec<f32>, Vec<f32>, Vec<f32>),
    pub cfg: PacketCfg,
    pub substeps: usize,
    pub ticks: usize,
    /// 喂平面表并跑**鬼影**（密度侧）。
    pub ghost: bool,
    /// 跑**投影**（穿透侧）；只有 `ghost` 打开才有意义（两侧**各有各的表**：口径不同，见 §15 补记六）。
    pub project: bool,
    pub ids: Vec<u32>,
    pub h: f32,
    pub adapter: usize,
}

/// 跑一条**卡上仿真链**：每 tick 收平面表 → 上传 → 推进一个 tick（可带鬼影/投影）。
pub fn chain_sim(s: &Sim, prov: &dyn ProviderColliders) -> Option<Vec<Vec3>> {
    chain_sim_dens(s, prov).map(|(p, _)| p)
}

/// 与 [`chain_sim`] 同体，只是**多回读一次卡上密度**（分相定位用：密度同而位置不同 ⇒ 差在力/积分相；
/// 密度就不同 ⇒ 密度/镜像相）。多出的那次回读只在链末发生一次 ⇒ 对读数无影响。
pub fn chain_sim_dens(s: &Sim, prov: &dyn ProviderColliders) -> Option<(Vec<Vec3>, Vec<f32>)> {
    let (init_pos, init_vel, init_mass) = s.init;
    let (mut pk, mut walls) =
        Packet::new_with_walls(s.adapter, s.cfg, init_pos, init_vel, init_mass).ok()?;
    walls.set_project(s.project);
    for _ in 0..s.ticks {
        let mut w_arg: Option<&WallStage> = None;
        if s.ghost {
            let (gp, _) = pk.read_state();
            let ps: Vec<Vec3> = (0..gp.len() / 3)
                .map(|k| Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]))
                .collect();
            let (i2, st, pl) = FluidSystem::gather_wall_contacts(&s.ids, s.h, &ps, prov);
            walls.upload(&pk, WallSide::Mirror, &i2, &st, &pl);
            // **投影侧另有一张表**（口径不同：`contacts_point_boundary` + 含穿透）。
            let (i3, st3, pl3) = FluidSystem::gather_wall_project_contacts(&s.ids, s.h, &ps, prov);
            walls.upload(&pk, WallSide::Project, &i3, &st3, &pl3);
            w_arg = Some(&walls);
        }
        pk.run_stages_with(&s.cfg, 0b111_1111, 1, s.substeps, false, w_arg);
    }
    let (gp, _) = pk.read_state();
    let n = gp.len() / 3;
    let pos = (0..n)
        .map(|k| Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]))
        .collect();
    // 卡上密度 = **最后一个子步**密度相位的产物（与 CPU `densities()` 同一时点口径）。
    let dens = walls.read_dens(&pk, s.cfg.n_fluid as usize);
    Some((pos, dens))
}
