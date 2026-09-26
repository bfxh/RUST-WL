//! **【定位探针】2b 不确定性的第一刀：`dens` 是不是在一个子步内就已经差？**
//!
//! 背景：`tests/sorted_copies_boundary.rs` 把现象钉成了"混合场景（流体 + 2b 边界粒子）不是 run-to-run
//! 可复现的，且种子是**一个舍入级、间歇性**的事件"（`PLAN-gpu.md` §24/§24.1）。本探针回答下一条：
//! **那一刀切在哪**——密度阶段（或更上游的格表）就已经差，还是密度一致、差异只在 eos/力/积分里长出来？
//!
//! ## ✅ 结论（2026-09-26 实测）
//!
//! ⚠️ **一条已试过的空转判别（勿重犯）**：把晶格间距设成 = `h`（想造"每格 1 粒 ⇒ 格内序无自由度"）
//! 会让最近邻**恰好落在 `r = h` 上** ⇒ 核函数 `t = h²−r² = 0` ⇒ 邻居贡献**恒为 0** ⇒ 场景冻结 ⇒
//! 当然"确定"。**那不是机理的证据**。要查"格内序到底有没有被规范化"，得走**直接读表**那条路
//! （对拍 GPU 的 `items` 与 CPU 的 —— 见 §24.3 的下一步）。
//!
//! **一个子步内 `dens` 就已经差**（子步 1：dens 差 **6** 个分量、vel 差 **8** 个），之后两者同步被混沌
//! 放大（8 子步后 dens 541 / vel 2159）⇒ **种子在「密度阶段或更上游的格表」**，而 **eos / 力 / 积分
//! 被排除**（它们只是 `dens` 的下游，不可能自己造出种子）。
//!
//! ⚠️ 这条读数有一个**必须自证的仪器前提**：`read_dens` 的读回本身要是确定的。自证方法（下次开工先跑）：
//! 在同一步上连读两次（不推子步）比对——若两次就读出不同的 `dens`，那是**仪器**的错，本切法作废。
//!
//! 手法：把同一条路径按**子步**切成快照（`dens` 用现成的 `WallStage::read_dens`，速度用 `snapshot`），
//! 跑两遍，逐子步比对，打印每一者第一次出现不符的子步号。
//!
//! 为什么用 `new_with_walls` + **空镜面表**：镜面表条目为 0 ⇒ 鬼影阶段是**空操作**（物理不变），
//! 但这样才拿得到 `dens` 的读回口——不必为了诊断给 `Packet` 新开 API。
//!
//! CI 无适配器 ⇒ 与其它 GPU 探针同口径跳过（不构成 CI 门禁）。

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg, WallSide, WallStage};

const N: usize = 16;
const SPACING: f32 = 0.05;
/// 记多少个**子步**的快照（每子步 `dt = 1/60`，与 `TICKS=1,SUB=1` 同口径）。
const STEPS: usize = 8;

fn scene() -> FluidSystem {
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.5,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    let half = N as f32 * SPACING * 0.5 + 4.0 * h;
    let bodies = vec![(
        0u32,
        Shape::Box {
            half: Vec3::new(half, 2.0 * SPACING, half),
        },
        BodyPose {
            pos: Vec3::new(0.0, -2.0 * h, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    )];
    let nb = f.set_boundary_particles(&bodies);
    assert!(nb > 0, "地板没造出边界粒子 ⇒ 场景退化");
    f
}

fn cfg_for(f: &FluidSystem, follow_box: bool) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    let (apos, _, _, n_fluid) = f.raw_particles();
    let np = apos.len();
    PacketCfg {
        n: np as u32,
        n_fluid: n_fluid as u32,
        total: gdims.0 * gdims.1 * gdims.2,
        gmin: [gmin.x, gmin.y, gmin.z],
        inv: ginv,
        dims: [gdims.0, gdims.1, gdims.2],
        cap: 512,
        h,
        h2: h * h,
        k6: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)),
        w0: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)) * h.powi(6),
        ks: 45.0 / (std::f32::consts::PI * h.powi(6)),
        mass: f.particle_mass(),
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        gravity: [0.0, 0.0, 0.0],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        // 【判别】`follow_box` = 每子步在卡上按**全量粒子**重算包围盒（= 覆盖盒）——管线里现成的档。
        recompute_box: follow_box,
        grid_bins_cap: if follow_box {
            1 << 20
        } else {
            gdims.0 * gdims.1 * gdims.2
        },
    }
}

fn flatten_all(f: &FluidSystem) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let (apos, avel, apmass, _) = f.raw_particles();
    let mut pos: Vec<f32> = Vec::with_capacity(apos.len() * 3);
    let mut vel: Vec<f32> = Vec::with_capacity(avel.len() * 3);
    for k in 0..apos.len() {
        pos.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        vel.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    (pos, vel, apmass.to_vec())
}

/// 逐子步快照：`(dens, vel)`。
fn per_substep(
    pk: &mut Packet,
    walls: &WallStage,
    pc: &PacketCfg,
    np: usize,
) -> Vec<(Vec<f32>, Vec<f32>)> {
    let mut out = Vec::with_capacity(STEPS);
    for _ in 0..STEPS {
        pk.run_substep(pc, 1.0 / 60.0, 0b111_1111, Some(walls));
        // 【仪器自证】同一步连读两次（不推子步）必须逐位一致——否则 `read_dens` 本身不定，
        // 本探针的一切结论作废（§24.2 写死的第一步）。
        let d0 = walls.read_dens(pk, np);
        let d1 = walls.read_dens(pk, np);
        assert_eq!(
            n_diff(&d0, &d1),
            0,
            "❌ 仪器自证失败：`read_dens` 同一步读两次就不一致 ⇒ 本探针结论不可用"
        );
        out.push((d1, pk.snapshot().1));
    }
    out
}

fn n_diff(a: &[f32], b: &[f32]) -> usize {
    a.iter()
        .zip(b.iter())
        .filter(|(x, y)| x.to_bits() != y.to_bits())
        .count()
}

#[test]
fn dens_or_vel_first_substep_that_diverges() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    let f = scene();
    let pc = cfg_for(&f, false);
    let np = pc.n as usize;
    let (pos, vel, pmass) = flatten_all(&f);
    // 【修法判别】同一场景跑两档盒：`follow_box=false`（配**固定盒**，管线默认口径，盒抄引擎那张旧表）
    // 与 `true`（每子步在卡上按**全量粒子**重算 = **覆盖盒**）。若后者把不确定性消掉 ⇒ 修法就是"用覆盖盒"。
    for follow in [false, true] {
        let pc = cfg_for(&f, follow);
        let (mut pk, walls) = Packet::new_with_walls(0, pc, &pos, &vel, &pmass).expect("建包失败");
        // 空镜面表 ⇒ 鬼影是空操作（物理不变），只为拿到 `dens` 读回口。
        let mut walls = walls;
        assert_eq!(
            walls.upload(&pk, WallSide::Mirror, &[], &[], &[]),
            0,
            "空表应当无事发生"
        );
        // 两遍：同一设备、同一初值（`restore` 回初值）、同一子步切法。
        pk.restore(&pos, &vel);
        let a = per_substep(&mut pk, &walls, &pc, np);
        pk.restore(&pos, &vel);
        let b = per_substep(&mut pk, &walls, &pc, np);
        let tag = if follow {
            "覆盖盒(follow)"
        } else {
            "固定盒(默认)"
        };
        let (dd, dv) = (
            n_diff(&a[STEPS - 1].0, &b[STEPS - 1].0),
            n_diff(&a[STEPS - 1].1, &b[STEPS - 1].1),
        );
        println!(
            "== [{tag}] 逐子步（{STEPS} 子步；子步 1 的 dens/vel 与末子步）==\n  · 子步 1：dens {} | vel {}\n  · 子步 {STEPS}：dens {dd} | vel {dv}",
            n_diff(&a[0].0, &b[0].0),
            n_diff(&a[0].1, &b[0].1)
        );
    }
    let _ = (pos, vel);
}
