//! **混合场景（流体 + 2b 边界粒子）的可复现性**探针——也是格序副本档在耦合路径上的准入前置条件。
//!
//! ## 实测结论（2026-09-26；细节与排除过程见 `PLAN-gpu.md` §24/§24.1）
//!
//! **这条路径今天不是 run-to-run 可复现的**（同一个 `Packet`、**同一设备**、同一输入，`restore` 回初值再跑
//! 一遍 ⇒ 流体段速度 ~30–340 个分量逐位不同、次数随机；位置与边界段全同）。已实测排除格表溢出、未初始化读、
//! 跨设备差异与回读（后两条由控制组否掉）；定位见 §24.1：种子是**一个舍入级、且间歇性**的事件。
//!
//! ## 本文件两半
//!
//! - 控制组 [`pure_fluid_same_device_rerun_is_bitwise_identical`]（**判红**）：纯流体同设备两遍必须逐位
//!   相同——它把上面那条钉成"混合场景**特有**"，而不是"回读或计时器坏了"；
//! - 报告 [`mixed_scene_same_device_rerun_report`]（**只打印**）：如实报出不可复现的规模。
//!
//! 格序档的准入判据也在这里的理由：`new_sorted` 在混合场景下**今天会静默退回平铺档** ⇒ "两档逐位相同"
//! 退化成"同一条路跑两遍"，而那条路本身不可复现 ⇒ 判据写不出来（会随机红）⇒ **§23.4 必须先于 §23.3**。
//!
//! CI 无适配器 ⇒ 与其它 GPU 探针同口径跳过（不构成 CI 门禁；跑它的是本地全量门禁的 `cargo test`）。

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

const N: usize = 16;
const SPACING: f32 = 0.05;
/// **多子步是刻意的**：单子步下这条判据**不可靠**——实测同一场景单子步有时差 1 个分量、有时差 0
/// （**间歇性**、概率不高）；跑到 2 tick × 4 子步就差 300+（**流体的混沌把种子放大**）⇒ 常驻报告要
/// 的是"可靠地看见"，不是"看见种子"。种子那条读数（单子步 1 或 0、舍入级）记在 `PLAN-gpu.md` §24。
const TICKS: usize = 2;
const SUB: usize = 4;

/// 小场景：晶格 + 5 趟静置 + 剪切初速；`floor` 为真时再加一块地板（提供 2b 边界粒子）。
fn scene(floor: bool) -> FluidSystem {
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
    if floor {
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
    }
    f
}

/// 与探针同口径的 `PacketCfg`。
fn cfg_for(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    // ⚠️ `f.len()` 给的是 **n_fluid**（不是总数）——`Packet` 吃**全量**视图，必须走 `raw_particles`。
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
        recompute_box: false,
        grid_bins_cap: gdims.0 * gdims.1 * gdims.2,
    }
}

/// **全量**粒子（含边界）的位置/速度/逐粒质量——`Packet` 吃的是全量视图。
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

/// 一格（同一个 `Packet`、同一个设备）跑两遍的比对结果：`(流体段不符数, 边界段不符数, 总分量数)`。
fn rerun_diff(floor: bool) -> (usize, usize, usize) {
    let f = scene(floor);
    let pc = cfg_for(&f);
    let (pos, vel, pmass) = flatten_all(&f);
    let mut pk = Packet::new(0, pc, &pos, &vel, &pmass).expect("建包失败");
    pk.run(&pc, TICKS, SUB, false);
    let (_, v1) = pk.snapshot();
    pk.restore(&pos, &vel);
    pk.run(&pc, TICKS, SUB, false);
    let (_, v2) = pk.snapshot();
    let nf = pc.n_fluid as usize;
    let (mut df, mut db) = (0usize, 0usize);
    for k in 0..v1.len().min(v2.len()) {
        if v1[k].to_bits() != v2[k].to_bits() {
            if k / 3 < nf {
                df += 1;
            } else {
                db += 1;
            }
        }
    }
    (df, db, v1.len())
}

fn have_adapter() -> bool {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return false;
    }
    true
}

/// **控制组**（判红）：纯流体的"同设备两遍"必须逐位相同——它把 `mixed` 那条结论钉成场景特有。
#[test]
fn pure_fluid_same_device_rerun_is_bitwise_identical() {
    if !have_adapter() {
        return;
    }
    let (df, db, tot) = rerun_diff(false);
    assert_eq!(
        (df, db),
        (0, 0),
        "纯流体在同一设备上跑两遍必须逐位相同（实得 流体段 {df} / 边界段 {db}，共 {tot} 个分量）——\
         若这里红了，说明**回读或计时器坏了**，那么混合场景那条观察也要重新解释"
    );
}

/// **报告**（不判红）：混合场景的不可复现规模——如实打印，等 `PLAN-gpu.md` §23.4。
#[test]
fn mixed_scene_same_device_rerun_report() {
    if !have_adapter() {
        return;
    }
    let (df, db, tot) = rerun_diff(true);
    println!(
        "混合场景（流体 + 2b 边界）同设备两遍（{TICKS} tick × {SUB} 子步）：流体段 {df} / 边界段 {db} \
         个速度分量逐位不同（共 {tot}）"
    );
    // ⚠️ 用 `n_fluid = n` 去"二分"是**混杂的、不要用**（实测过）：它把边界粒子从"运动学冻结"变成
    // **可动** ⇒ 物理本身变了（差异从个位数飙到上万）⇒ 读数不是定位信号。负面结果记在 §24.1。
    println!(
        "⇒ 已知未修（`PLAN-gpu.md` §24）：这条路径今天不是 run-to-run 可复现的 ⇒ 不能拿它当格序档的逐位判据。"
    );
}
