//! **闭环验收**（`PLAN-gpu.md` §13.6）：**GPU 的反作用真的推得动刚体**。
//!
//! 与 `gpu_coupling_probe` 的区别：那个探针的体是**脚本驱动**的（只验力算得对不对）；本探针是
//! **闭环**——反作用 → 体运动 → 新姿态 → 重建边界粒子 → 再算力。两条链各跑一条闭环，互相对拍。
//!
//! 场景（**开放域、无箱壁**：于是"流体 + 挡板"是**闭合系统**，动量账没有第三方向量）：
//! - 12³ 流体块（1728 粒）带 **+x 初速** `V0` 朝前飞；
//! - 前方一块**挡板**（盒体，比水重 ⇒ 被推走后不会回撞水）：只解**平动**，转动锁死
//!   （ω ≡ 0；不对称加载会产生力矩，本片不验转轴，但力矩仍逐 tick 与 CPU 对拍）；
//! - 零重力、关 XSPH（速度平滑不严格守恒动量）⇒ 闭合系统的动量账是**恒等式**：
//!   `m_体·v_体 + ΔP_流体 = 0`（力 → 运动的**符号/量纲错会立刻破这条**）。
//!
//! 两条链每 tick 都做：① 宿主按**当前**体姿态重建边界粒子（`set_boundary_particles`）；
//! ② GPU 侧只把**边界段**上传（`upload_boundary_segment`）；③ 各推进一个 tick；
//! ④ 取反作用（CPU `boundary_reactions()` / GPU `ReactionStage::aggregate`）→ 积分体运动。
//! ⇒ 与 facade 的时序同形（`World::fluid_pass` 先重建再步进；反作用在体子步里施加）。
//!
//! 判据：① 体轨迹（`x`/`vx`，CPU vs GPU，**口径 B**）；② **闭合系统动量账**两侧各一条；
//! ③ 逐 tick 反作用相对差；④ 非有限值计数。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_body_probe -- [n] [ticks] [--adapter K]`

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg, ReactionStage};

/// 固定 tick 步长（与引擎同：60 Hz）。
const DT: f32 = 1.0 / 60.0;
/// 流体初速（+x，米/秒）：远低于 CFL 上限（`max_speed_frac·h/dt_sub` ≈ 9.6）。
const V0: f32 = 0.8;
/// 挡板密度（kg/m³）：比水重 ⇒ 推走不回头。
const BODY_RHO: f32 = 4000.0;
/// 挡板与流体块前缘的初始间距（米）：留出"起飞—撞上"的观察窗。
const GAP: f32 = 0.35;
/// **有载判据**（牛顿）：`Σ|F_cpu|` 低于它算空载（撞击前 / 推走之后，力只有浮点噪声）。
const LOAD_MIN: f32 = 1e-3;

/// 挡板形状：厚度 0.25 m = **2.5h**（本仓 2b 口径：体薄于 2h 反作用失准）。
fn body_shape() -> Shape {
    Shape::Box {
        half: Vec3::new(0.125, 0.2, 0.2),
    }
}

/// 平动体（转动锁死）。`Copy`：两条链各持一份同值初值。
#[derive(Clone, Copy)]
struct Body {
    pos: Vec3,
    vel: Vec3,
    mass: f32,
}

impl Body {
    fn new(pos: Vec3) -> Self {
        let Shape::Box { half } = body_shape() else {
            unreachable!("挡板形状就是盒体")
        };
        let vol = 8.0 * half.x * half.y * half.z;
        Self {
            pos,
            vel: Vec3::ZERO,
            mass: BODY_RHO * vol,
        }
    }

    fn pose(&self) -> BodyPose {
        BodyPose {
            pos: self.pos,
            rot: Quat::IDENTITY,
            linvel: self.vel,
            angvel: Vec3::ZERO,
        }
    }

    /// 半隐式欧拉一步（与 facade 同口径：力 = 反作用，一 tick 施加一次）。
    fn integrate(&mut self, f: Vec3, dt: f32) {
        self.vel += f * (dt / self.mass);
        self.pos += self.vel * dt;
    }
}

/// 命令行参数。
struct Args {
    n: usize,
    ticks: usize,
    adapter: usize,
}

fn parse_args() -> Args {
    let mut it = std::env::args().skip(1);
    let n = it.next().and_then(|s| s.parse().ok()).unwrap_or(12);
    let ticks = it.next().and_then(|s| s.parse().ok()).unwrap_or(240);
    let rest: Vec<String> = it.collect();
    let mut adapter = 0usize;
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    Args { n, ticks, adapter }
}

fn make_fluid(n: usize) -> FluidSystem {
    let spacing = 0.05f32;
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        // 关 XSPH：速度平滑不是严格的动量守恒项（开着会在"闭合系统动量账"里留噪声）。
        xsph_viscosity: 0.0,
        ..FluidConfig::default()
    };
    FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.0,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    )
}

/// 流体动量的分段和（只算流体前缀；边界粒子是脚手架，不进账）。
fn fluid_momentum(f: &FluidSystem) -> Vec3 {
    let mut p = Vec3::ZERO;
    let m = f.particle_mass();
    for v in f.velocities() {
        p += *v * m;
    }
    p
}

/// GPU 侧流体动量：**从卡上回读**（流体状态活在卡上；脚手架 `FluidSystem` 的流体段停在初值）。
fn gpu_momentum(pk: &Packet, n_fluid: usize, mass: f32) -> (Vec3, usize) {
    let (_, gv) = pk.read_state();
    let mut p = Vec3::ZERO;
    let mut bad = 0usize;
    for k in 0..n_fluid {
        let v = Vec3::new(gv[k * 3], gv[k * 3 + 1], gv[k * 3 + 2]);
        if !v.is_finite() {
            bad += 1;
        }
        p += v * mass;
    }
    (p, bad)
}

/// 按流体现状建 `PacketCfg`（与 `gpu_coupling_probe` 同一套映射）。
fn make_cfg(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let gd = f.neighbor_grid();
    let (apos, ..) = f.raw_particles();
    PacketCfg {
        n: apos.len() as u32,
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
    }
}

/// 两条链 + 各自的挡板（**同一份初值**）。
struct Rig {
    cpu: FluidSystem,
    gpu: FluidSystem,
    pk: Packet,
    stage: ReactionStage,
    pc: PacketCfg,
    b_cpu: Body,
    b_gpu: Body,
    substeps: usize,
    n_fluid: usize,
    mass: f32,
    p0: Vec3,
}

fn build(a: &Args) -> Option<Rig> {
    let spacing = 0.05f32;
    let front = a.n as f32 * spacing * 0.5;
    let mut cpu = make_fluid(a.n);
    let mut gpu = make_fluid(a.n);
    let b0 = Body::new(Vec3::new(front + GAP + 0.125, 0.0, 0.0));
    // 初速：+x（两条链同一份）。
    let mut vs = cpu.velocities().to_vec();
    for v in vs.iter_mut() {
        v.x = V0;
    }
    cpu.set_velocities(&vs);
    gpu.set_velocities(&vs);
    // 边界粒子：只有挡板（开放域 ⇒ 没有箱壁）。
    let (sh, pose0) = (body_shape(), b0.pose());
    cpu.set_boundary_particles(&[(0, sh, pose0)]);
    gpu.set_boundary_particles(&[(0, sh, pose0)]);
    let n_fluid = cpu.len();
    let mass = cpu.particle_mass();
    let substeps = cpu.config().substeps.max(1) as usize;
    let pc = make_cfg(&gpu);
    let (pos_flat, vel_flat, pmass) = {
        let (apos, avel, apmass, _) = gpu.raw_particles();
        let mut p = Vec::with_capacity(apos.len() * 3);
        let mut v = Vec::with_capacity(apos.len() * 3);
        for k in 0..apos.len() {
            p.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
            v.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
        }
        (p, v, apmass.to_vec())
    };
    let pk = Packet::new(a.adapter, pc, &pos_flat, &vel_flat, &pmass).ok()?;
    let stage = ReactionStage::new(&pk);
    Some(Rig {
        p0: fluid_momentum(&cpu),
        cpu,
        gpu,
        pk,
        stage,
        pc,
        b_cpu: b0,
        b_gpu: b0,
        substeps,
        n_fluid,
        mass,
    })
}

/// 一个 tick（两侧同形）：宿主重建边界 → （GPU 只上传边界段）→ 各自步进 → 取反作用 → 积分体。
/// 返回本 tick 的**反作用相对差**（逐粒最大，GPU 卡上聚合 vs CPU 自带聚合）。
fn tick(r: &mut Rig, dt: f32) -> f32 {
    // ① CPU 链：重建（用**当前**姿态）→ 步进 → 取反作用 → 积分体。
    let pose_cpu = r.b_cpu.pose();
    r.cpu.set_boundary_particles(&[(0, body_shape(), pose_cpu)]);
    r.cpu.step(dt, &NoProviders);
    let fc = r
        .cpu
        .boundary_reactions()
        .first()
        .map(|x| x.1)
        .unwrap_or(Vec3::ZERO);
    r.b_cpu.integrate(fc, dt);
    // ② GPU 链：宿主侧同样重建（脚手架），但只把**边界段**写回卡上。
    let pose_gpu = r.b_gpu.pose();
    r.gpu.set_boundary_particles(&[(0, body_shape(), pose_gpu)]);
    {
        let (pos, vel, _, nf) = r.gpu.raw_particles();
        r.pk.upload_boundary_segment(nf as u32, &pos[nf..], &vel[nf..]);
    }
    r.pk.run(&r.pc, 1, r.substeps, false);
    let gb = r.stage.aggregate(&r.pk, r.gpu.boundary_spans());
    let mut fg = Vec3::ZERO;
    let mut df = 0.0f32;
    let mut scale = 0.0f32;
    for (i, &(_, fcpu, _)) in r.cpu.boundary_reactions().iter().enumerate() {
        scale += fcpu.length();
        if let Some(&(_, g, _)) = gb.get(i) {
            fg += g;
            df = df.max((fcpu - g).length());
        }
    }
    r.b_gpu.integrate(fg, dt);
    // 相对量按**当 tick 的 Σ|F_cpu|** 归一（逐粒归一会让小力处爆掉）。**空载 tick**（撞上之前 /
    // 推走之后）里力全是浮点噪声 ⇒ 比值没有意义 ⇒ 记 0，只由调用方统计**有载** tick。
    if scale < LOAD_MIN {
        return 0.0;
    }
    df / scale
}

/// 时间序列样本：体轨迹 + 本 tick 的逐粒反作用相对差（**同点取样**：第 1 个 tick 就大 ⇒ 接线错；
/// 随漂移一起长 ⇒ 口径 B 混沌）。
struct Sample {
    t: usize,
    x_cpu: f32,
    x_gpu: f32,
    v_cpu: f32,
    v_gpu: f32,
    /// `None` = 本 tick **空载**（力只有浮点噪声，比值无意义）。
    rel: Option<f32>,
}

/// 整段跑完的读数。
struct Run {
    worst_rel: f32,
    loaded: usize,
    trace: Vec<Sample>,
    dp_cpu: Vec3,
    dp_gpu: Vec3,
    bad: usize,
}

fn run_loop(r: &mut Rig, a: &Args) -> Run {
    let mut worst_rel = 0.0f32;
    let mut loaded = 0usize;
    let mut trace: Vec<Sample> = Vec::new();
    for t in 0..a.ticks {
        let rel = tick(r, DT);
        // 相对量只在**有载** tick 上取：空载时力是浮点噪声，比值无意义（踩过——首版把撞击前的
        // 噪声记成了 12% 的"最大相对差"）。
        let loaded_now = r
            .cpu
            .boundary_reactions()
            .iter()
            .map(|x| x.1.length())
            .sum::<f32>()
            > LOAD_MIN;
        if loaded_now {
            worst_rel = worst_rel.max(rel);
            loaded += 1;
        }
        let tk = t + 1;
        if matches!(tk, 1 | 5 | 20) || tk % 40 == 0 || tk == a.ticks {
            trace.push(Sample {
                t: tk,
                x_cpu: r.b_cpu.pos.x,
                x_gpu: r.b_gpu.pos.x,
                v_cpu: r.b_cpu.vel.x,
                v_gpu: r.b_gpu.vel.x,
                rel: if loaded_now { Some(rel) } else { None },
            });
        }
    }
    let dp_cpu = fluid_momentum(&r.cpu) - r.p0;
    let (p_gpu, bad) = gpu_momentum(&r.pk, r.n_fluid, r.mass);
    Run {
        worst_rel,
        loaded,
        trace,
        dp_cpu,
        dp_gpu: p_gpu - r.p0,
        bad,
    }
}

fn report(a: &Args, r: &Rig, o: &Run) {
    println!("== 闭环：GPU 反作用推动刚体（开放域 ⇒ 流体 + 挡板 = **闭合系统**）==");
    println!(
        "  {}³ 流体 {} 粒 + 挡板（厚 0.25 m，{:.0} kg）| 流体初速 {V0} m/s | {} tick",
        a.n, r.n_fluid, r.b_cpu.mass, a.ticks
    );
    println!(
        "  ① 逐 tick 反作用相对差（按当 tick 的 Σ|F_cpu| 归一；只在 {} 个有载 tick 上取）：最坏 {:.2e}",
        o.loaded, o.worst_rel
    );
    println!(
        "  ② 轨迹 + 同点相对差（**第 1 个 tick 就大 ⇒ 接线错**；随漂移一起长 ⇒ 口径 B 混沌）："
    );
    for s in &o.trace {
        let rel = match s.rel {
            Some(v) => format!("{v:.2e}"),
            None => "—（空载）".to_string(),
        };
        println!(
            "     t={:>4}：x {:.5} vs {:.5}（差 {:.2e}）| vx {:.5} vs {:.5}（差 {:.2e}）| relF {rel}",
            s.t,
            s.x_cpu,
            s.x_gpu,
            (s.x_cpu - s.x_gpu).abs(),
            s.v_cpu,
            s.v_gpu,
            (s.v_cpu - s.v_gpu).abs()
        );
    }
    let ib = r.b_cpu.vel * r.b_cpu.mass;
    let ig = r.b_gpu.vel * r.b_gpu.mass;
    let rl = |v: Vec3, s: Vec3| v.length() / s.length().max(1e-30);
    println!("  ③ 闭合系统动量账（`m_体·v_体 + ΔP_流体 ≈ 0`；零重力 / 无提供者 / 关 XSPH）：");
    println!(
        "     **两链之差 {:.3e} N·s（相对体冲量 {:.2e}）** ← 锋利的那把尺（采样项两链共享 ⇒ 相减抵消）",
        (ib - ig).length(),
        rl(ib - ig, ib)
    );
    println!(
        "     绝对残差：CPU {:.3e} / GPU {:.3e}（相对 {:.2e} / {:.2e}）——含**末子步采样项**：逐 tick 只拿得到",
        (ib + o.dp_cpu).length(),
        (ig + o.dp_gpu).length(),
        rl(ib + o.dp_cpu, ib),
        rl(ig + o.dp_gpu, ig)
    );
    println!(
        "     末子步的力，而流体动量的变化是**全部子步**的力积出来的（撞击段力是尖的 ⇒ 这项能到几十个百分点）。体冲量 {:.4e} N·s。",
        ib.length()
    );
    println!(
        "  ④ 非有限速度 {} 粒 | 末速 vx：CPU {:.5} / GPU {:.5}（相对差 {:.2e}）",
        o.bad,
        r.b_cpu.vel.x,
        r.b_gpu.vel.x,
        (r.b_cpu.vel.x - r.b_gpu.vel.x).abs() / r.b_cpu.vel.x.abs().max(1e-30)
    );
    // ⑤ 卡上报的**紧凑包围盒** vs 主机按卡上状态算的 AABB —— 门面的近域过滤用的就是它。
    // ⚠️ **差不为 0 是对的**：卡上盒子是**上一子步起点**的（`box_setup` 在每子步开头跑），
    // 而这里比的是末态 ⇒ 期望差 ≈ 每子步行距 `|v|·dt_sub`（本场景 0.8 m/s × 1/240 ≈ 3.3 mm ✓ 实测 3–4 mm）。
    // 近域过滤自带 `pad = h = 0.1 m` 的余量 ⇒ 这点滞后盖得住（要零差就在 `step` 末再刷一次盒子）。
    let (gp, _) = r.pk.read_state();
    let nb = r.gpu.raw_particles().0.len() - r.n_fluid;
    let (mut lo, mut hi) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
    for k in 0..(r.n_fluid + nb) {
        let p = Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]);
        lo = lo.min(p);
        hi = hi.max(p);
    }
    let d = match r.pk.read_box() {
        Some((l, h)) => format!(
            "min 差 {:.2e} m / max 差 {:.2e} m",
            (lo - Vec3::new(l[0], l[1], l[2])).length(),
            (hi - Vec3::new(h[0], h[1], h[2])).length()
        ),
        None => "（未取到）".to_string(),
    };
    println!("  ⑤ 卡上紧凑盒 vs 主机算的 AABB：{d}");
}

fn main() {
    let a = parse_args();
    let Some(mut rig) = build(&a) else {
        println!("⚠️ 没起 GPU 管线——本探针需要适配器");
        return;
    };
    let out = run_loop(&mut rig, &a);
    report(&a, &rig, &out);
}
