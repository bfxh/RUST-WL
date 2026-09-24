//! **提供者壁面的镜像鬼影**（GPU 侧）验收（`PLAN-gpu.md` §13.10）——**在同一状态上比密度场**。
//!
//! 为什么比密度而不是比"水会不会被托住"：镜像鬼影只补**密度**（`0 < sdf < h` 的那层）；
//! 挡住穿透的是 CPU 侧**每子步**的壁面投影（`boundary_pass`），而投影是主机侧机制
//! （SDF 查询在主机）⇒ **本片没上卡**。所以"让水静置在体素地上"这种场景缺了投影必然穿地
//! （首版实栽：三档全掉到 −18 m）。本探针因此把判据改成**单点密度对拍**：
//!
//! - 同一个初始状态（体素水槽 + 铸装水块，底面落在静置线）：
//!   1. **CPU**：`FluidSystem::step(0.0, providers)`（零步长 ⇒ 位置不动，只跑网格+密度）
//!      ⇒ `densities()`——CPU 的密度轮**自带**镜像鬼影；
//!   2. **GPU + 鬼影**：同一初值建包 + 主机收集平面表（**同一个** `wall_planes_in`）上传
//!      ⇒ `run_stages_with(显卡 0b000_1111 = 网格+密度, …, Some(&walls))` ⇒ 读回密度；
//!   3. **GPU 不带鬼影**（**金丝雀**）：同上去掉鬼影 ⇒ 应复现"贴壁核质量亏损"。
//!
//! 判据：① `max|ρ_gpu − ρ_cpu|/ρ0`（带鬼影，应只差求和序）；② 同一量在**不带**鬼影时的大小
//! （金丝雀：应明显更大 ⇒ 证明鬼影真在补质量）；③ 参与粒子的比例（近壁粒子数）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_walls_probe -- [--adapter K]`

use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_core::interop::ProviderColliders;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{gather_wall_contacts, Packet, PacketCfg, WallStage};

const SPACING: f32 = 0.05;

/// 体素水槽：5×5 格（外沿 2.5 m）地板 + 中心围堰 ⇒ 内腔 0.5×0.5 m（与 `tests/fluid_boundary.rs` 同款）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// 铸装水块：`8³@0.05`，底面落在**静置线** sdf = +skin（与 CPU 口径一致）。
fn water() -> FluidSystem {
    FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.2, 0.99, -0.2),
        [8, 8, 8],
        SPACING,
    )
}

/// 按流体现状建 `PacketCfg`（与其它探针同一套映射）。
fn make_cfg(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let gd = f.neighbor_grid();
    PacketCfg {
        n: f.raw_particles().0.len() as u32,
        n_fluid: f.len() as u32,
        total: gd.dims.0 * gd.dims.1 * gd.dims.2,
        gmin: [gd.min.x, gd.min.y, gd.min.z],
        inv: gd.inv,
        dims: [gd.dims.0, gd.dims.1, gd.dims.2],
        cap: 512,
        h,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        ks,
        mass: f.particle_mass(),
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        gravity: [
            f.config().gravity.x,
            f.config().gravity.y,
            f.config().gravity.z,
        ],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box: true,
        grid_bins_cap: 1 << 20,
    }
}

/// 稀疏平面表：逐粒调 CPU 的 `wall_planes_in`（≤8 面）⇒ `(ids, start, planes)`。
/// 抽别名：clippy 的 `type_complexity` 拦长元组。
type PlaneTable = (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>);
/// 跑一趟"网格 + 密度"（**不推进状态**）：可选带鬼影 ⇒ 返回逐粒密度。
fn dens_on_gpu(
    init: &(Vec<f32>, Vec<f32>, Vec<f32>),
    cfg: PacketCfg,
    planes: Option<&PlaneTable>,
    adapter: usize,
) -> Option<Vec<f32>> {
    let (pos_flat, vel_flat, pmass) = init;
    let (mut pk, mut walls) =
        Packet::new_with_walls(adapter, cfg, pos_flat, vel_flat, pmass).ok()?;
    let mut w_arg: Option<&WallStage> = None;
    if let Some((i2, st, pl)) = planes {
        walls.upload(&pk, i2, st, pl);
        w_arg = Some(&walls);
    }
    // 位掩码 0b000_1111 = 分箱 / 扫描+占位 / 规范化 / **密度**（不跑 EOS、力、积分 ⇒ 位置不动）。
    pk.run_stages_with(&cfg, 0b000_1111, 1, 1, false, w_arg);
    Some(walls.read_dens(&pk, cfg.n_fluid as usize))
}

/// 扁平化（全粒子 `pos`/`vel`/`pmass`；建包用）。
fn flat(f: &FluidSystem) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let (apos, avel, apmass, _) = f.raw_particles();
    let mut p = Vec::with_capacity(apos.len() * 3);
    let mut v = Vec::with_capacity(apos.len() * 3);
    for k in 0..apos.len() {
        p.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        v.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    (p, v, apmass.to_vec())
}

/// **底层粒子平均高度**（按 y 取最低 25%）——静置仿真里"站没站住"的判据。
fn bottom_mean_y(ps: &[Vec3], n_fluid: usize) -> f32 {
    let mut ys: Vec<f32> = ps[..n_fluid.min(ps.len())].iter().map(|p| p.y).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let k = (ys.len() / 4).max(1);
    ys[..k].iter().sum::<f32>() / k as f32
}

/// 静置仿真的参数（避免长参数表）。
struct Sim<'a> {
    init: &'a (Vec<f32>, Vec<f32>, Vec<f32>),
    cfg: PacketCfg,
    substeps: usize,
    ticks: usize,
    /// 喂平面表并跑**鬼影**（密度侧）。
    ghost: bool,
    /// 跑**投影**（穿透侧）；只有 `ghost` 打开才有意义（两者共用同一张表）。
    project: bool,
    ids: Vec<u32>,
    h: f32,
    adapter: usize,
}

/// 跑一条**卡上仿真链**：每 tick 收平面表 → 上传 → 推进一个 tick（可带鬼影/投影）。
fn chain_sim(s: &Sim, prov: &dyn ProviderColliders) -> Option<Vec<Vec3>> {
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
            let (i2, st, pl) = gather_wall_contacts(&s.ids, s.h, &ps, prov);
            walls.upload(&pk, &i2, &st, &pl);
            w_arg = Some(&walls);
        }
        pk.run_stages_with(&s.cfg, 0b111_1111, 1, s.substeps, false, w_arg);
    }
    let (gp, _) = pk.read_state();
    let n = gp.len() / 3;
    Some(
        (0..n)
            .map(|k| Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]))
            .collect(),
    )
}

fn main() {
    let mut it = std::env::args().skip(1);
    let rest: Vec<String> = it.by_ref().collect();
    let mut adapter = 0usize;
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    // 场景：体素水槽（World 只用来给 provider 壁面）+ 铸装水块（探针自持流体，便于零步长取密度）。
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let mut sys = water();
    // ⚠️ 必须把**壁面 provider id 表**交给流体：CPU 的密度轮走 `self.boundaries` 才收集壁面平面
    // （首版漏了这一步 ⇒ CPU 根本没跑镜像 ⇒ 与"无鬼影"档一致，把结论带歪）。
    sys.set_boundaries(&[v]);
    let init = flat(&sys);
    // ① CPU：**零步长**推进（位置不动、只跑网格 + 密度）⇒ 密度轮自带镜像鬼影。
    //    ⚠️ 前提：初值在静置线上（无穿透）⇒ 四个子步的投影都不触发 ⇒ 各子步密度相同。
    sys.step(0.0, w.providers());
    let cpu_dens: Vec<f32> = sys.densities().to_vec();
    let rho0 = sys.config().rest_density;
    let n = sys.len();
    let h = sys.config().smoothing_radius;
    let cfg = make_cfg(&sys);
    let ps: Vec<Vec3> = sys.positions().to_vec();
    let prov = w.providers();
    let (i2, st, pl) = gather_wall_contacts(&[v], h, &ps, prov);
    let Some(gpu_with) = dens_on_gpu(
        &init,
        cfg,
        Some(&(i2.clone(), st.clone(), pl.clone())),
        adapter,
    ) else {
        println!("⚠️ 没起 GPU 管线——本探针需要适配器");
        return;
    };
    let Some(gpu_without) = dens_on_gpu(&init, cfg, None, adapter) else {
        return;
    };
    let dev = |a: &[f32], b: &[f32]| -> (f32, f32) {
        let mut mx = 0.0f32;
        let mut mean = 0.0f32;
        for (x, y) in a.iter().zip(b.iter()) {
            let d = (x - y).abs();
            mx = mx.max(d);
            mean += d;
        }
        (mx, mean / a.len() as f32)
    };
    let (mx_w, mn_w) = dev(&cpu_dens, &gpu_with);
    let (mx_wo, mn_wo) = dev(&cpu_dens, &gpu_without);
    println!("== 提供者壁面的镜像鬼影（GPU 侧）vs CPU 档：同一状态比密度场 ==");
    println!(
        "  体素水槽 + 8³@0.05 水块（{n} 粒）| 近壁粒子（有条目）{} / {} 面 | 零步长 ⇒ 位置不动",
        i2.len(),
        pl.len()
    );
    println!(
        "  ① 带鬼影：max |Δρ| = {mx_w:.3e} kg/m³（{:.2e} ρ0）/ mean {mn_w:.3e}",
        mx_w / rho0
    );
    println!(
        "  ② **金丝雀**（不带鬼影）：max |Δρ| = {mx_wo:.3e}（{:.2e} ρ0）/ mean {mn_wo:.3e} ⇒ 鬼影补的质量就是这一块",
        mx_wo / rho0
    );
    println!(
        "  ③ 鬼影把最大偏差压小了 **{:.1}×**",
        (mx_wo / mx_w.max(1e-30)).max(1.0)
    );
    // ── ② 静置仿真（真刀真枪）：同一初值三条链 ──
    // A CPU 档（自带镜像 + 逐子步投影）；B 卡上（鬼影 + **投影**）；C 卡上（**只鬼影** = 金丝雀）。
    let ticks = 150usize;
    let mut cpu = water();
    cpu.set_boundaries(&[v]);
    for _ in 0..ticks {
        cpu.step(1.0 / 60.0, w.providers());
    }
    let a = cpu.positions().to_vec();
    let substeps = cpu.config().substeps.max(1) as usize;
    let mk = |ghost: bool, project: bool| Sim {
        init: &init,
        cfg,
        substeps,
        ticks,
        ghost,
        project,
        ids: vec![v],
        h,
        adapter,
    };
    let Some(b) = chain_sim(&mk(true, true), prov) else {
        return;
    };
    let Some(c) = chain_sim(&mk(true, false), prov) else {
        return;
    };
    let (ya, yb, yc) = (
        bottom_mean_y(&a, n),
        bottom_mean_y(&b, n),
        bottom_mean_y(&c, n),
    );
    let mut mx = 0.0f32;
    for (p, q) in a.iter().zip(b.iter()) {
        mx = mx.max((*p - *q).length());
    }
    println!("  ── 静置仿真（{ticks} tick、重力开、同一初值三条链）──");
    println!("     A CPU（镜像 + 逐子步投影）  底层平均 y = {ya:.5} m");
    println!(
        "     B 卡上（鬼影 + **投影**）    底层平均 y = {yb:.5} m（与 A 差 {:.2e} m）| max|Δpos| {mx:.2e} m",
        (ya - yb).abs()
    );
    println!(
        "     C 卡上（**只鬼影**，金丝雀）底层平均 y = {yc:.5} m ⇒ {}",
        if yc < 0.0 {
            "穿地（证明投影不可省）"
        } else {
            "也站住了（异常：投影该是必需的）"
        }
    );
}
