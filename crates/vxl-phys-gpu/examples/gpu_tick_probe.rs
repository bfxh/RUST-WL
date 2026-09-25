//! **GPU 常驻管线：整 tick 读数 + 与 CPU 引擎的状态对照**（`docs/PLAN-gpu.md` §9.4 ③ / §12）。
//!
//! 做的事：同一初始状态、同一 tick 数，两条路各跑一遍——
//! ① **CPU 引擎**（`FluidSystem::step`，相位计时给出 ms/tick）；
//! ② **GPU 常驻管线**（`pipeline::Packet`：网格 → 密度 → EOS → 力 → 积分，缓冲只建一次）。
//! 产出：GPU 整 tick ms（**无逐 tick 回读**）、"含耦合回读"的预估、以及两侧末态的对照
//! （NaN 计数、|Δpos| / |Δv| 的极值与均值、动能比）——**口径 B 的轨迹口径**：浮点相位不逐位，
//! 所以多 tick 后位置会漂（混沌放大），这里只声明"同一物理、无发散"，不声明位级一致。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_tick_probe -- [n] [ticks] [--adapter K]`

use std::time::Instant;

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg, ReactionStage};

/// `--tank` 的静态盒体（地板 + 四壁）：按"表面采样 + 两层内移"变成 2b 边界粒子
/// （壁厚取 2×晶格间距 ⇒ 采成实心；四壁贴着流体块的 x/z 边界，地板顶面 = 流体底面）。
fn tank_bodies(n: usize, spacing: f32, h: f32) -> Vec<(u32, Shape, vxl_phys_fluid::BodyPose)> {
    let half = n as f32 * spacing * 0.5;
    let t = 2.0 * spacing;
    let hgt = (n as f32 * spacing) + 2.0 * h;
    let tip = 0.5 + hgt * 0.5;
    let pose = |pos: Vec3| vxl_phys_fluid::BodyPose {
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

/// 命令行参数（位置参数 + `--adapter K` / `--box=follow` / `--gravity` / `--tank`）。
struct Args {
    n: usize,
    ticks: usize,
    adapter_index: usize,
    /// `--box=follow`：每子步从位置重算箱子（与 CPU 同频）；默认 `fixed` = §12.1 的读数口径。
    follow_box: bool,
    /// `--gravity`：开重力（自由落体；配合 `--box=follow` 才能不跑出箱子）。
    gravity_on: bool,
    /// `--tank`：给 2b **边界粒子**（地板 + 四壁的静态盒体）——用来验"边界粒子在 GPU 侧也跑得对"：
    /// 密度/力核里的 `sum_b`/`m_j = pmass[j]` 是 2b 的数学，积分则必须**只跑流体前缀**。
    tank: bool,
    /// `--cpu-threads K`：**CPU 对照档**的线程数（默认 1 = 与历史读数同口径）。>1 走并行档——
    /// 常驻测试 `parallel_equals_serial_bitwise` 已证两者**逐位相同** ⇒ 判据不受影响，
    /// 但千万粒档的 CPU 对照从"十几分钟"降到"一两分钟"（大档验收的前提）。
    cpu_threads: usize,
}

fn parse_args() -> Args {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
    let rest: Vec<String> = args.collect();
    let mut adapter_index = 0usize;
    if let Some(k) = rest.iter().position(|a| a == "--adapter") {
        adapter_index = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    Args {
        n,
        ticks,
        adapter_index,
        follow_box: rest.iter().any(|a| a == "--box=follow"),
        gravity_on: rest.iter().any(|a| a == "--gravity"),
        tank: rest.iter().any(|a| a == "--tank"),
        cpu_threads: rest
            .iter()
            .position(|a| a == "--cpu-threads")
            .and_then(|k| rest.get(k + 1))
            .and_then(|v| v.parse().ok())
            .unwrap_or(1),
    }
}

/// 场景常数 + 建好的 CPU 侧流体。
struct FluidSetup {
    f: FluidSystem,
    h: f32,
    substeps: usize,
}

/// 建 CPU 侧流体：**零重力场景**（`pipeline::Packet` 的箱子是**固定**的，见其头注，自由落体会在
/// 几十个 tick 后把粒子全部钳进边缘格（段长撞 `cap`、计时失真）；关掉重力 ⇒ 流体留在箱内，
/// 且压 wave 让场景比自由落体更混沌 ⇒ 是**更好的**容差口径测试。重力项是每粒一次加法，不影响
/// 相位成本量级）+ 可选 2b 边界粒子 + 剪切初速。
fn build_fluid(a: &Args) -> FluidSetup {
    let spacing = 0.05f32;
    // CPU 对照档可走并行（`--cpu-threads K`）：与串行**逐位相同**（常驻测试守），只是快 K×。
    if a.cpu_threads > 1 {
        println!(
            "⚠️ CPU 对照档 threads={}（并行口径；与串行逐位相同）⇒ 下面的 CPU 读数与加速比按并行读",
            a.cpu_threads
        );
    }
    let cfg0 = FluidConfig {
        gravity: if a.gravity_on {
            Vec3::new(0.0, -9.81 * 0.1, 0.0)
        } else {
            Vec3::ZERO
        },
        threads: a.cpu_threads,
        ..FluidConfig::default()
    };
    let h = cfg0.smoothing_radius;
    let substeps = cfg0.substeps.max(1) as usize;
    let dt_tick = 1.0 / 60.0;
    let mut f = FluidSystem::new(
        cfg0,
        Vec3::new(
            -(a.n as f32) * spacing * 0.5,
            0.5,
            -(a.n as f32) * spacing * 0.5,
        ),
        [a.n, a.n, a.n],
        spacing,
    );
    for _ in 0..5 {
        f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
    }
    if a.tank {
        let bodies = tank_bodies(a.n, spacing, h);
        let nb = f.set_boundary_particles(&bodies);
        println!("  2b 边界粒子：{nb} 个（地板 + 四壁；流体 {} 个）", f.len());
    }
    // **给一个剪切初速**：零重力下完美晶格的 ρ ≡ ρ0 ⇒ p ≡ 0 ⇒ 全程静止（那样容差口径根本没被压到）。
    // 加一层 shear ⇒ 有真动力学，又是**受限**的（流体留在箱内，不撞固定箱子的边缘格）。
    {
        let mut vs = f.velocities().to_vec();
        for (i, v) in vs.iter_mut().enumerate() {
            let p = f.positions()[i];
            v.x += 0.6 * (p.y * 12.0).sin();
            v.z += 0.4 * (p.y * 8.0).cos();
        }
        f.set_velocities(&vs);
    }
    FluidSetup { f, h, substeps }
}

/// 建立 GPU 常驻管线（`Packet`）：CPU 侧的**全量**粒子视图（含 2b 边界粒子）+ 网格打包进 `PacketCfg`。
/// 起点 = 这一刻；箱子取这一刻引擎算出来的那套（**本片固定不重算**，见 `pipeline.rs` 头注）。
fn build_packet(
    f: &FluidSystem,
    h: f32,
    adapter_index: usize,
    follow_box: bool,
) -> Option<(Packet, PacketCfg)> {
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    // **全部粒子**（含 2b 边界粒子）：`raw_particles` 给后端用的全量视图（`positions()` 只给流体前缀）。
    let (apos, avel, apmass, n_fluid) = f.raw_particles();
    let pos0: Vec<Vec3> = apos.to_vec();
    let vel0: Vec<Vec3> = avel.to_vec();
    let np = pos0.len();
    let total = gdims.0 * gdims.1 * gdims.2;

    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        pos_flat.extend_from_slice(&[pos0[k].x, pos0[k].y, pos0[k].z]);
        vel_flat.extend_from_slice(&[vel0[k].x, vel0[k].y, vel0[k].z]);
    }
    // **逐粒质量**（流体 = 粒子质量；边界 = 2b 的面密度质量 ⇒ 那个 `sum_b`/`m_j` 用的就是它）。
    let pmass: Vec<f32> = apmass.to_vec();
    let mass = f.particle_mass();
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let pc = PacketCfg {
        n: np as u32,
        // 流体前缀（积分只跑它）；纯流体场景时 = n。
        n_fluid: n_fluid as u32,
        total,
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
        recompute_box: follow_box,
        // 跟随时给足额度（= CPU 侧同一个预算上限）：粒子散开后箱子变大，格表要装得下。
        grid_bins_cap: if follow_box { 1 << 20 } else { total },
    };
    match Packet::new(adapter_index, pc, &pos_flat, &vel_flat, &pmass) {
        Ok(p) => Some((p, pc)),
        Err(e) => {
            println!("GPU 路径不可用：{e}");
            None
        }
    }
}

/// 一行漂移读数：`(tick, |Δpos|max, |Δpos|mean, |Δv|max, 动能比)`。
type DriftRow = (usize, f32, f64, f32, f64);

/// ③ 逐 tick 漂移形态表：CPU 与 GPU **每 tick 各推进一步**再比位置
///    ⇒ 一次分清"接线错"（线性增长）与"混沌放大"（随机游走/指数）。
fn run_drift(
    f: &mut FluidSystem,
    pk: &mut Packet,
    pc: &PacketCfg,
    ticks: usize,
    substeps: usize,
    dt_tick: f32,
) -> (Vec<DriftRow>, usize) {
    let n_fluid = f.raw_particles().3;
    let mass = f.particle_mass();
    let probe_at = [1usize, 2, 3, 5, 8, 13, 21, 30];
    let mut drift: Vec<DriftRow> = Vec::new();
    let mut nan_cnt = 0usize;
    let cmp = |pos_cpu: &[Vec3], vel_cpu: &[Vec3], pos_gpu: &[f32], vel_gpu: &[f32]| {
        let mut mx = 0.0f32;
        let mut sum = 0.0f64;
        let mut dv = 0.0f32;
        let mut ke_c = 0.0f64;
        let mut ke_g = 0.0f64;
        let mut bad = 0usize;
        // **只比流体前缀**：CPU 侧的 `positions()/velocities()` 就是流体，边界粒子是运动学冻结的
        // （比它没意义，且下标会越界）。
        let nf = n_fluid;
        for i in 0..nf {
            let (px, py, pz) = (pos_gpu[i * 3], pos_gpu[i * 3 + 1], pos_gpu[i * 3 + 2]);
            if !px.is_finite() || !py.is_finite() || !pz.is_finite() {
                bad += 1;
                continue;
            }
            let d = Vec3::new(px - pos_cpu[i].x, py - pos_cpu[i].y, pz - pos_cpu[i].z);
            let dl = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
            if dl > mx {
                mx = dl;
            }
            sum += dl as f64;
            let dvv = Vec3::new(
                vel_gpu[i * 3] - vel_cpu[i].x,
                vel_gpu[i * 3 + 1] - vel_cpu[i].y,
                vel_gpu[i * 3 + 2] - vel_cpu[i].z,
            );
            let dvl = (dvv.x * dvv.x + dvv.y * dvv.y + dvv.z * dvv.z).sqrt();
            if dvl > dv {
                dv = dvl;
            }
            let vc = vel_cpu[i];
            let vg = Vec3::new(vel_gpu[i * 3], vel_gpu[i * 3 + 1], vel_gpu[i * 3 + 2]);
            ke_c += 0.5 * f64::from(mass) * f64::from(vc.length_squared());
            ke_g += 0.5 * f64::from(mass) * f64::from(vg.length_squared());
        }
        (bad, mx, sum / nf.max(1) as f64, dv, ke_g / ke_c.max(1e-30))
    };
    for t in 1..=ticks {
        f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
        pk.run(pc, 1, substeps, false);
        let (pg, vg) = pk.read_state();
        let (bad, mx, mean, dv, ker) = cmp(f.positions(), f.velocities(), &pg, &vg);
        nan_cnt = nan_cnt.max(bad);
        if probe_at.contains(&t) || t == ticks {
            drift.push((t, mx, mean, dv, ker));
        }
    }
    (drift, nan_cnt)
}

/// ④ CPU 引擎的相位计时（**放在漂移表之后**：它会把状态推进，放在前面会让对照组错位一个 tick
///    ——踩过：那样 GPU 全程滞后一 tick，漂移表里表现为"刚性平移 + 精确 g·Δt"的假信号；
///    同一类坑还有一次：`Packet::run` 内部曾自带"预热一个 tick" ⇒ 每次调用都多推一 tick）。
fn time_cpu(f: &mut FluidSystem, ticks: usize, dt_tick: f32) -> (f64, f64) {
    f.reset_phase_us();
    let t_cpu = Instant::now();
    for _ in 0..ticks {
        f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
    }
    let cpu_wall_ms = t_cpu.elapsed().as_secs_f64() * 1e3;
    let cpu_phase_ms: f64 = f.phase_us().iter().sum::<u64>() as f64 / 1e3;
    (cpu_phase_ms, cpu_wall_ms)
}

/// ⑤ 稳态计时读数（无逐 tick 回读）。
struct GpuTiming {
    gpu_ticks: usize,
    reps: [f32; 3],
    gpu_ms: f32,
    box_ms: f32,
    stage_ms: Vec<(&'static str, f32)>,
    rb_ms: f32,
    overflow: u32,
}

/// ⑤ 稳态计时：先跑一次**丢弃计时**的热身（摊掉首轮编译/首触访存）——漂移表已经跑完，这里再推进
///    状态不影响任何对照。两条实测教训：
///    ① 短窗受 **GPU 时钟爬升**支配（同配置能差 2×）⇒ 3 次 × 100 tick 取最小，原值一并打印；
///       **下限按规模让路**：`n > 2M` 档降到 20 趟（长趟本身已把时钟爬升平均掉，而 100 趟
///       在大档是分钟级：10M 一趟 ≈230 ms ⇒ 100×3 = 70 s 起步，再加回读与快照更多）；
///    ② **每次计时前必须 `restore` 到同一状态**——计时本身会推进流体（剪切压实 ⇒ 邻域候选
///       变多），不还原就会把"状态演化"混进"相位成本"（实测：同一全链先后差 3×）。
fn time_gpu(pk: &mut Packet, pc: &PacketCfg, ticks: usize, substeps: usize) -> GpuTiming {
    let _ = pk.run(pc, 2, substeps, false);
    // 下限按规模让路（见函数注①）：大档 20 趟，小档维持 100 趟（口径按实际趟数打印）。
    let gpu_ticks = ticks.max(if pc.n > 2_000_000 { 20 } else { 100 });
    let (snap_pos, snap_vel) = pk.snapshot();
    let mut reps = [0.0f32; 3];
    let mut box_ms = 0.0f32;
    for r in reps.iter_mut() {
        pk.restore(&snap_pos, &snap_vel);
        let t = pk.run(pc, gpu_ticks, substeps, false);
        *r = t.per_tick;
        // 记最后一次的"每子步箱子"累计（读数含在 `per_tick` 里，这里只把它单列出来）。
        box_ms = t.box_ms / gpu_ticks.max(1) as f32;
    }
    let gpu_ms = reps.iter().cloned().fold(f32::INFINITY, f32::min);
    // **相位消融**（位：1 分箱 / 2 扫描+占位 / 4 规范化 / 8 密度 / 16 EOS / 32 力 / 64 积分）
    // 单项单次测量有 ±2 ms 噪声 ⇒ 每项 2 次取最小（总量那一行才是可信的主读数）。
    let mut stage_ms = Vec::new();
    for (name, mask) in [
        ("分箱", 0b000_0001u32),
        ("+扫描/占位", 0b000_0011),
        ("+规范化", 0b000_0111),
        ("+密度", 0b000_1111),
        ("+EOS", 0b001_1111),
        ("+力", 0b011_1111),
        ("+积分=全链", 0b111_1111),
    ] {
        let mut best = f32::INFINITY;
        for _ in 0..2 {
            pk.restore(&snap_pos, &snap_vel);
            let ms = pk.run_stages(pc, mask, gpu_ticks, substeps, false).per_tick;
            best = best.min(ms);
        }
        stage_ms.push((name, best));
    }
    pk.restore(&snap_pos, &snap_vel);
    let rb_ms = pk.measure_readback_ms(u64::MAX);
    // §19.1 ① 判别：**同一次全量拷贝**、只映射 1 MB ⇒ 与全量映射比。线性降 ⇒ 驱动按映射区间拷贝
    // （生产上可只映射需要的部分）；不降 ⇒ 整块 flush（驱动行为，只能换回读路径）。直接打印、不进 Report。
    let rb_1m_ms = pk.measure_readback_ms(1 << 20);
    println!(
        "  回读口径判别（PLAN-gpu §19.1①）：全量映射 {rb_ms:.2} ms vs 只映射 1 MB {rb_1m_ms:.2} ms ⇒ {}",
        if rb_1m_ms < rb_ms * 0.5 {
            "**按映射区间拷贝**（可只映射真需要的部分）"
        } else {
            "**整块 flush**（驱动行为：只能换回读路径）"
        }
    );
    GpuTiming {
        gpu_ticks,
        reps,
        gpu_ms,
        box_ms,
        stage_ms,
        rb_ms,
        overflow: pk.read_overflow(),
    }
}

/// 报表所需的全部读数（`report` 单一入参，避免长签名）。
struct Report<'a> {
    np: usize,
    ticks: usize,
    substeps: usize,
    gdims: (u32, u32, u32),
    cpu_phase_ms: f64,
    cpu_wall_ms: f64,
    timing: &'a GpuTiming,
    drift: &'a [DriftRow],
    nan_cnt: usize,
    vmax_cpu: f32,
    vsum_cpu: f64,
    recompute_box: bool,
}

/// 结果报表：CPU/GPU 对照 + 相位消融 + 末态对照（漂移形态表）+ 速度标定 + 网格护栏。
fn report(r: &Report) {
    let np = r.np;
    let ticks = r.ticks;
    let substeps = r.substeps;
    let gdims = r.gdims;
    let t = r.timing;
    let gpu_ms = t.gpu_ms;
    let reps = t.reps;
    println!(
        "== GPU 常驻管线 vs CPU 引擎（{np} 粒；{ticks} tick × {substeps} 子步；箱子 {gdims:?}）=="
    );
    println!(
        "  CPU 引擎：相位合计 **{:.1} ms**（{:.2} ms/tick）| 壁钟 {:.0} ms",
        r.cpu_phase_ms,
        r.cpu_phase_ms / ticks as f64,
        r.cpu_wall_ms
    );
    let gpu_ticks = t.gpu_ticks;
    println!(
        "  GPU 常驻：**{gpu_ms:.2} ms/tick**（{gpu_ticks} tick；3 次原值 {:?}，取最小）",
        reps.map(|v| (v * 100.0).round() / 100.0)
    );
    let rb_ms = t.rb_ms;
    println!(
        "  ⇒ 整 tick **{:.1}×**（对 CPU 相位口径）；单独量的一次状态回读+同步 = {:.2} ms ⇒ 含耦合回读 {:.2} ms/tick（{:.1}×）",
        r.cpu_phase_ms / ticks as f64 / f64::from(gpu_ms.max(1e-9)),
        rb_ms,
        f64::from(gpu_ms + rb_ms),
        r.cpu_phase_ms / ticks as f64 / f64::from((gpu_ms + rb_ms).max(1e-9))
    );
    if r.recompute_box {
        // 诚实记账：`--box=follow` 的"每子步一次归约 + 24 B 回读往返"占整 tick 多少（已含在上面）。
        println!(
            "  ├ 其中**每子步箱子**：{:.2} ms/tick（{:.0}%，{substeps} 子步 × 归约+回读+写 uniform）",
            t.box_ms,
            100.0 * t.box_ms / gpu_ms.max(1e-9)
        );
    }
    {
        let mut line = String::from("  相位消融（ms/tick）");
        let mut prev = 0.0f32;
        for (name, ms) in &t.stage_ms {
            line.push_str(&format!(" | {name} {ms:.2}（+{:.2}）", ms - prev));
            prev = *ms;
        }
        println!("{line}");
    }
    println!(
        "  末态对照（**口径 B**：浮点相位不逐位 ⇒ 多 tick 后位置会漂，这里只声明同一物理、无发散）；"
    );
    let nan_cnt = r.nan_cnt;
    println!("    NaN 粒 {nan_cnt} | 漂移形态（tick: |Δpos|max / mean / |Δv|max / 动能比）：");
    for (tick, mx, mean, dv, ker) in r.drift {
        println!("      t={tick:>3}：{mx:.5} m / {mean:.5} m / {dv:.5} m/s / {ker:.4}");
    }
    // 速度量级（读动能比要用它标定：本探针零重力 ⇒ |v| 极小，动能比对绝对差极敏感）
    println!(
        "    标定：CPU 侧 |v| max {:.4} m/s / mean {:.4} m/s（零重力 ⇒ 极小 ⇒ 动能比只当「发散与否」看）",
        r.vmax_cpu,
        r.vsum_cpu / np as f64
    );
    println!(
        "    ⚠️ 判读：**线性增长 ⇒ 接线/口径错**；随机游走或指数 ⇒ 混沌放大（口径 B 的必然）。"
    );
    let overflow = t.overflow;
    println!(
        "    网格护栏 overflow（段长超 cap 的格数，应 0）：{overflow}{}",
        if overflow == 0 {
            ""
        } else {
            " ⚠️ 非 0 ⇒ 表未规范化（箱内粒子过挤，换箱或调 cap）"
        }
    );
}

/// `--tank` 的③b：反作用验收——GPU 逐粒回读 + **卡上每体聚合**，与 CPU 的
/// `boundary_forces()` / `boundary_reactions()` 对拍（判据与读数口径见 `PLAN-gpu.md` §13.3）。
fn report_reactions(pk: &Packet, f: &FluidSystem, n_fluid: u32) {
    let mut stage = ReactionStage::new(pk);
    let text = stage.report(
        pk,
        n_fluid,
        f.boundary_forces(),
        f.boundary_reactions(),
        f.boundary_spans(),
    );
    print!("{text}");
}

fn main() {
    let a = parse_args();
    // **分段计时**：千万粒档要能直接看出"是哪一段在吃时间"（`--cpu-threads` 只管 CPU 对照档的
    // 效率，而探针自己的建场/建卡/对拍三段在大档上是分钟级 ⇒ 没有这个读数就只能猜）。
    let mut t_stage = std::time::Instant::now();
    let mut stage = |m: &str| {
        println!("⏱ {m}：{:.1} s", t_stage.elapsed().as_secs_f64());
        t_stage = std::time::Instant::now();
    };
    let s = build_fluid(&a);
    stage("build_fluid（CPU 建场 + 5 趟静置）");
    let mut f = s.f;
    let dt_tick = 1.0 / 60.0;
    let gdims = {
        let gd = f.neighbor_grid();
        (gd.dims.0, gd.dims.1, gd.dims.2)
    };
    let np = f.raw_particles().0.len();
    let n_fluid = f.raw_particles().3;
    let Some((mut pk, pc)) = build_packet(&f, s.h, a.adapter_index, a.follow_box) else {
        return;
    };
    stage("build_packet（建卡上管线/缓冲 + 首次上传）");

    let (drift, nan_cnt) = run_drift(&mut f, &mut pk, &pc, a.ticks, s.substeps, dt_tick);
    stage("run_drift（CPU/GPU 各推 + 逐 tick 对拍）");
    // ③b 反作用回读 + 聚合验收（`--tank`）：GPU 逐粒 / 每体（卡上聚合）vs CPU 的
    // `boundary_forces()` / `boundary_reactions()`——同一末态、同一子步（两边都停在
    // 末子步边界）⇒ 只该差浮点累加序（`PLAN-gpu.md` §13.2 / §13.3）。
    if a.tank {
        report_reactions(&pk, &f, n_fluid as u32);
    }
    stage("report_reactions（--tank 未开则 ≈0）");
    let (cpu_phase_ms, cpu_wall_ms) = time_cpu(&mut f, a.ticks, dt_tick);
    stage("time_cpu（CPU 计时档）");
    let timing = time_gpu(&mut pk, &pc, a.ticks, s.substeps);
    stage("time_gpu（GPU 计时档）");

    // 速度量级（读动能比要用它标定：本探针零重力 ⇒ |v| 极小，动能比对绝对差极敏感）
    let mut vmax_cpu = 0.0f32;
    let mut vsum = 0.0f64;
    for v in f.velocities() {
        let s = v.length_squared().sqrt();
        if s > vmax_cpu {
            vmax_cpu = s;
        }
        vsum += s as f64;
    }

    report(&Report {
        np,
        ticks: a.ticks,
        substeps: s.substeps,
        gdims,
        cpu_phase_ms,
        cpu_wall_ms,
        timing: &timing,
        drift: &drift,
        nan_cnt,
        vmax_cpu,
        vsum_cpu: vsum,
        recompute_box: pc.recompute_box,
    });
}
