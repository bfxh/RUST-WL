//! **耦合回路验收**（`PLAN-gpu.md` §13.5）：**每 tick 重建边界粒子**的动体场景。
//!
//! 场景：静态盒体（地板 + 四壁）里，**地板做脚本化垂向振荡**（`y(t) = A·sin(2πft)`，
//! 体速度 = `A·2πf·cos(2πft)`），流体零重力、**关 XSPH**（它是速度平滑，不严格守恒动量，
//! 会在大账里留一项噪声）。两条链每 tick 做同一件事：
//! ① CPU `FluidSystem::set_boundary_particles` 按**新姿态**重建边界粒子（位置 + 体面速度 `v+ω×r`）；
//! ② GPU `Packet::upload_boundary_segment` 只写**边界段**（流体状态留在卡上，不回读）；
//! ③ 两侧各推进一个 tick；④ GPU 卡上聚合每体 `(F, τ)`（`ReactionStage` / `reduce.wgsl`）。
//!
//! 三类判据（读数与判读都写在 §13.5）：
//! ① **逐体对拍**（CPU `boundary_reactions()` vs GPU 卡上聚合）——口径 B，取**逐 tick**相对量；
//! ② **动量大账**（**与位置无关**的判据）：零重力 + 无提供者 + 关 XSPH ⇒ 流体动量之变只来自
//!    边界反作用，`Σ_体 ∫F dt = −ΔP_流体`。⚠️ 逐 tick 采样用的是**末子步**的力（CPU/GPU 都只给
//!    末子步）⇒ 绝对值含一项 O(`dt_tick`·dF/dt) 的采样误差；**两侧之差**没有这一项，是锋利的尺。
//! ③ **时间序列**（漂移与相对差同点取样）：分辨"接线错"（第 1 个 tick 就大）与"口径 B 混沌"
//!    （随漂移一起长）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_coupling_probe -- [n] [ticks] [选项]`
//! 选项：`--amp=<m>`（地板振幅，0 = 静态对照）、`--freq=<Hz>`、`--no-reup`（跳过每 tick 重传，
//! 静态下应与默认路径**逐位相同**）、`--adapter <K>`。

use std::f32::consts::PI;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg, ReactionStage};

/// 地板振荡振幅（米）：≈ 0.2h；每 tick 位移 ≈ 0.16 晶格间距——够产生真实动量交换，又不挤穿。
const AMP: f32 = 0.02;
/// 振荡频率（Hz）。
const FREQ: f32 = 1.5;

/// 盒体（地板 + 四壁）：与 `gpu_tick_probe --tank` 同形（示例各自是独立目标 ⇒ 不能跨档复用）。
/// 体 0 = 地板（本探针要让它动），体 1..4 = 四壁。
fn tank_bodies(n: usize, spacing: f32, h: f32) -> Vec<(u32, Shape, BodyPose)> {
    let half = n as f32 * spacing * 0.5;
    let t = 2.0 * spacing;
    let hgt = (n as f32 * spacing) + 2.0 * h;
    let tip = 0.5 + hgt * 0.5;
    let pose = |pos: Vec3| BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    };
    let span = half + 2.0 * t;
    let wall_x = |x: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(t * 0.5, hgt * 0.5, span),
            },
            pose(Vec3::new(x, tip, 0.0)),
        )
    };
    let wall_z = |z: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(span, hgt * 0.5, t * 0.5),
            },
            pose(Vec3::new(0.0, tip, z)),
        )
    };
    vec![
        (
            0u32,
            Shape::Box {
                half: Vec3::new(span, t * 0.5, span),
            },
            pose(Vec3::new(0.0, 0.5 - t * 0.5, 0.0)),
        ),
        wall_x(half + t * 0.5, 1),
        wall_x(-(half + t * 0.5), 2),
        wall_z(half + t * 0.5, 3),
        wall_z(-(half + t * 0.5), 4),
    ]
}

/// 命令行参数。
struct Args {
    n: usize,
    ticks: usize,
    adapter: usize,
    amp: f32,
    freq: f32,
    /// `false` = 跳过每 tick 的重建 + 上传（静态场景的 A/B：重传路径不许改任何东西）。
    reupload: bool,
}

fn parse_args() -> Args {
    let mut it = std::env::args().skip(1);
    let n = it.next().and_then(|s| s.parse().ok()).unwrap_or(20);
    let ticks = it.next().and_then(|s| s.parse().ok()).unwrap_or(240);
    let rest: Vec<String> = it.collect();
    let mut a = Args {
        n,
        ticks,
        adapter: 0,
        amp: AMP,
        freq: FREQ,
        reupload: !rest.iter().any(|x| x == "--no-reup"),
    };
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        a.adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    for x in &rest {
        if let Some(v) = x.strip_prefix("--amp=") {
            a.amp = v.parse().unwrap_or(AMP);
        }
        if let Some(v) = x.strip_prefix("--freq=") {
            a.freq = v.parse().unwrap_or(FREQ);
        }
    }
    a
}

/// 装置：两侧的引擎 + 初始状态 + 不变的标量（质量、粒子数、时间步）。
struct Rig {
    f: FluidSystem,
    pk: Packet,
    stage: ReactionStage,
    pc: PacketCfg,
    base: Vec<(u32, Shape, BodyPose)>,
    nb: usize,
    n_fluid: usize,
    mass: f32,
    dt_tick: f32,
    substeps: usize,
}

/// 起两条链（**同一初值**）：CPU 引擎 + GPU 常驻管线（全量上传一次：流体 + 边界 + 逐粒 `pmass`）。
/// `None` = 没起 GPU 管线（无适配器）。
fn build(a: &Args) -> Option<Rig> {
    let n = a.n;
    let spacing = 0.05f32;
    let cfg0 = FluidConfig {
        gravity: Vec3::ZERO,
        // 关 XSPH：速度平滑不是严格的动量守恒项（开着会在"动量大账"里留一笔无物理意义的噪声）。
        xsph_viscosity: 0.0,
        ..FluidConfig::default()
    };
    let h = cfg0.smoothing_radius;
    let substeps = cfg0.substeps.max(1) as usize;
    let dt_tick = 1.0 / 60.0;
    let mut f = FluidSystem::new(
        cfg0,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    // 剪切初速：零重力下完美晶格 ρ ≡ ρ0 ⇒ p ≡ 0 ⇒ 全程静止（容差口径压不到，动量大账也恒 0）。
    {
        let mut vs = f.velocities().to_vec();
        for (i, v) in vs.iter_mut().enumerate() {
            let p = f.positions()[i];
            v.x += 0.6 * (p.y * 12.0).sin();
            v.z += 0.4 * (p.y * 8.0).cos();
        }
        f.set_velocities(&vs);
    }
    let base = tank_bodies(n, spacing, h);
    let nb = f.set_boundary_particles(&base);
    let (apos, avel, apmass, n_fluid) = f.raw_particles();
    let np = apos.len();
    let mass = f.particle_mass();
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        pos_flat.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        vel_flat.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    let pmass: Vec<f32> = apmass.to_vec();
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    let pc = PacketCfg {
        n: np as u32,
        n_fluid: n_fluid as u32,
        total: gdims.0 * gdims.1 * gdims.2,
        gmin: [gmin.x, gmin.y, gmin.z],
        inv: ginv,
        dims: [gdims.0, gdims.1, gdims.2],
        cap: 512,
        h,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        ks,
        mass,
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        gravity: [0.0, 0.0, 0.0],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box: true,
        grid_bins_cap: 1 << 20,
    };
    let pk = Packet::new(a.adapter, pc, &pos_flat, &vel_flat, &pmass).ok()?;
    let stage = ReactionStage::new(&pk);
    Some(Rig {
        f,
        pk,
        stage,
        pc,
        base,
        nb,
        n_fluid,
        mass,
        dt_tick,
        substeps,
    })
}

/// 流体动量的**分段和**（`Σ m·v`，只算流体前缀）：边界粒子是运动学脚手架，不进账。
fn fluid_momentum(f: &FluidSystem) -> Vec3 {
    let mut p = Vec3::ZERO;
    let m = f.particle_mass();
    for v in f.velocities() {
        p += *v * m;
    }
    p
}

/// 驱动参数（`amp = 0` ⇒ **静态对照**：用来分辨"漂移来自驱动"还是"来自接线"）。
#[derive(Clone, Copy)]
struct Drive {
    amp: f32,
    freq: f32,
    time: f32,
    /// `false` ⇒ 跳过每 tick 的重建 + 上传（静态场景的 A/B）。
    reupload: bool,
}

/// 一个 tick 的对拍读数（GPU 侧 = 卡上聚合，CPU 侧 = `boundary_reactions()`）。
struct TickCmp {
    max_df: f32,
    max_dt: f32,
    sum_f_cpu: Vec3,
    sum_f_gpu: Vec3,
    scale_f: f32,
    scale_t: f32,
}

/// **协议**：宿主重建边界 → 只上传边界段 → 两侧各推进一个 tick。
fn step_rig(
    f: &mut FluidSystem,
    pk: &mut Packet,
    pc: &PacketCfg,
    base: &[(u32, Shape, BodyPose)],
    dr: Drive,
    dt_tick: f32,
    substeps: usize,
) {
    // ① 宿主侧重建：地板姿态 + 体面速度（其余四壁不动）。
    if dr.reupload {
        let w = 2.0 * PI * dr.freq;
        let mut bodies = base.to_vec();
        bodies[0].2.pos.y += dr.amp * (w * dr.time).sin();
        bodies[0].2.linvel = Vec3::new(0.0, dr.amp * w * (w * dr.time).cos(), 0.0);
        f.set_boundary_particles(&bodies);
        // ② 只把边界段写回卡上（流体状态不动）。
        let (pos, vel, _, nf) = f.raw_particles();
        pk.upload_boundary_segment(nf as u32, &pos[nf..], &vel[nf..]);
    }
    // ③ 两侧各推进一个 tick。
    f.step(dt_tick, &NoProviders);
    pk.run(pc, 1, substeps, false);
}

/// **验收**：卡上聚合（段表每次随调用上传 ⇒ 体原点跟着体走）并与 CPU 逐体对拍。
fn compare(f: &FluidSystem, pk: &Packet, stage: &mut ReactionStage) -> TickCmp {
    let gb = stage.aggregate(pk, f.boundary_spans());
    let mut out = TickCmp {
        max_df: 0.0,
        max_dt: 0.0,
        sum_f_cpu: Vec3::ZERO,
        sum_f_gpu: Vec3::ZERO,
        scale_f: 0.0,
        scale_t: 0.0,
    };
    for (i, &(_, fc, tc)) in f.boundary_reactions().iter().enumerate() {
        out.sum_f_cpu += fc;
        out.scale_f += fc.length();
        out.scale_t += tc.length();
        if let Some(&(_, fg, tg)) = gb.get(i) {
            out.sum_f_gpu += fg;
            out.max_df = out.max_df.max((fc - fg).length());
            out.max_dt = out.max_dt.max((tc - tg).length());
        }
    }
    out
}

/// 时间序列样本：漂移与相对差**同点取样** —— 用来分辨"接线错"（第 1 个 tick 就大）
/// 与"口径 B 混沌"（随漂移一起长）。
struct Sample {
    t: usize,
    max_pos: f32,
    mean_pos: f32,
    rel_f: f32,
    rel_t: f32,
}

/// 整段跑完的读数。
struct Run {
    worst_f: f32,
    worst_t: f32,
    rel_f: f32,
    rel_t: f32,
    scale_f: f32,
    scale_t: f32,
    acc_cpu: Vec3,
    acc_gpu: Vec3,
    dp_cpu: Vec3,
    dp_gpu: Vec3,
    bad: usize,
    n_fluid: usize,
    nb: usize,
    series: Vec<Sample>,
}

/// 跑 `ticks` 个 tick，返回所有读数（不打印；打印在 `report`）。
fn run_loop(rig: &mut Rig, a: &Args) -> Run {
    let p_start = fluid_momentum(&rig.f);
    let (mut worst_f, mut worst_t) = (0.0f32, 0.0f32);
    // **逐 tick 的最坏相对量**（对当 tick 的 Σ|F_cpu|、Σ|τ_cpu|）——最锋利的那把尺：
    // 按整段求和标定会把分母撑大 ticks 倍，把"每 tick 的真实相对差"掩盖掉。
    let (mut rel_f, mut rel_t) = (0.0f32, 0.0f32);
    let (mut scale_f, mut scale_t) = (0.0f32, 0.0f32);
    let (mut acc_cpu, mut acc_gpu) = (Vec3::ZERO, Vec3::ZERO);
    let mut series: Vec<Sample> = Vec::new();
    for t in 0..a.ticks {
        let dr = Drive {
            amp: a.amp,
            freq: a.freq,
            time: (t as f32 + 1.0) * rig.dt_tick,
            reupload: a.reupload,
        };
        step_rig(
            &mut rig.f,
            &mut rig.pk,
            &rig.pc,
            &rig.base,
            dr,
            rig.dt_tick,
            rig.substeps,
        );
        let c = compare(&rig.f, &rig.pk, &mut rig.stage);
        worst_f = worst_f.max(c.max_df);
        worst_t = worst_t.max(c.max_dt);
        rel_f = rel_f.max(c.max_df / c.scale_f.max(1e-30));
        rel_t = rel_t.max(c.max_dt / c.scale_t.max(1e-30));
        scale_f += c.scale_f;
        scale_t += c.scale_t;
        acc_cpu += c.sum_f_cpu * rig.dt_tick;
        acc_gpu += c.sum_f_gpu * rig.dt_tick;
        let tk = t + 1;
        if matches!(tk, 1 | 2 | 5 | 10 | 20) || tk % 40 == 0 || tk == a.ticks {
            let (gp, _) = rig.pk.read_state();
            let (mut mx, mut mean) = (0.0f32, 0.0f32);
            let cp = rig.f.positions();
            for k in 0..cp.len() {
                let g = Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]);
                let d = (cp[k] - g).length();
                mx = mx.max(d);
                mean += d;
            }
            series.push(Sample {
                t: tk,
                max_pos: mx,
                mean_pos: mean / cp.len() as f32,
                rel_f: c.max_df / c.scale_f.max(1e-30),
                rel_t: c.max_dt / c.scale_t.max(1e-30),
            });
        }
    }
    let dp_cpu = fluid_momentum(&rig.f) - p_start;
    let (_, gv) = rig.pk.read_state();
    let mut p1_gpu = Vec3::ZERO;
    let mut bad = 0usize;
    for k in 0..rig.n_fluid {
        let v = Vec3::new(gv[k * 3], gv[k * 3 + 1], gv[k * 3 + 2]);
        if !v.is_finite() {
            bad += 1;
        }
        p1_gpu += v * rig.mass;
    }
    Run {
        worst_f,
        worst_t,
        rel_f,
        rel_t,
        scale_f,
        scale_t,
        acc_cpu,
        acc_gpu,
        dp_cpu,
        dp_gpu: p1_gpu - p_start,
        bad,
        n_fluid: rig.n_fluid,
        nb: rig.nb,
        series,
    }
}

fn report(a: &Args, r: &Run) {
    let tag = if a.amp == 0.0 {
        "（**静态对照**）"
    } else {
        ""
    };
    let reup_tag = if a.reupload {
        ""
    } else {
        "／**不重传**（A/B 对照）"
    };
    println!("== 耦合回路（每 tick 重建边界段 + 卡上聚合）==");
    println!(
        "  {}³ 流体 {} 粒 + **{} 边界**（地板 A = {} m / f = {} Hz{tag}{reup_tag}）× {} tick",
        a.n, r.n_fluid, r.nb, a.amp, a.freq, a.ticks
    );
    println!(
        "  ① 逐体对拍：max |ΔF| = {:.3e} N / max |Δτ| = {:.3e}｜**逐 tick 最坏相对量** {:.2e}（力）/ {:.2e}（力矩）",
        r.worst_f, r.worst_t, r.rel_f, r.rel_t
    );
    println!(
        "     （整段标定 Σ_t Σ|F_cpu| = {:.3e} N、Σ_t Σ|τ_cpu| = {:.3e}）",
        r.scale_f, r.scale_t
    );
    println!(
        "  ② 动量大账（Σ_体 ∫F dt 应 = −ΔP_流体；逐 tick 取末子步的力 ⇒ 绝对值含 O(dt) 采样项）："
    );
    let dpt = r.dp_cpu.length().max(1e-30);
    println!(
        "     CPU 残差 {:.3e} N·s（{:.2e} 相对 |ΔP|）/ GPU 残差 {:.3e}（{:.2e}）/ **两侧之差 {:.3e}（{:.2e}）**",
        (r.acc_cpu + r.dp_cpu).length(),
        (r.acc_cpu + r.dp_cpu).length() / dpt,
        (r.acc_gpu + r.dp_gpu).length(),
        (r.acc_gpu + r.dp_gpu).length() / dpt,
        (r.acc_cpu - r.acc_gpu).length(),
        (r.acc_cpu - r.acc_gpu).length() / dpt
    );
    println!(
        "     |ΔP|：CPU {:.4e} N·s vs GPU {:.4e} N·s（差 {:.2e}）| 非有限速度 {} 粒",
        r.dp_cpu.length(),
        r.dp_gpu.length(),
        (r.dp_cpu - r.dp_gpu).length(),
        r.bad
    );
    println!(
        "  ③ 时间序列（漂移与相对差**同点取样**：第 1 个 tick 就大 ⇒ 接线错；随漂移一起长 ⇒ 口径 B 混沌）："
    );
    for s in &r.series {
        println!(
            "     t={:>4}：|Δpos|max {:.5} m / mean {:.5}｜relF {:.2e}｜relT {:.2e}",
            s.t, s.max_pos, s.mean_pos, s.rel_f, s.rel_t
        );
    }
}

fn main() {
    let a = parse_args();
    let Some(mut rig) = build(&a) else {
        println!("⚠️ 没起 GPU 管线——本探针需要适配器");
        return;
    };
    let r = run_loop(&mut rig, &a);
    report(&a, &r);
}
