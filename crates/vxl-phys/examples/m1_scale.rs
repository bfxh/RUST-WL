//! M1 规模场景（§3 最低通过档）：10 万动态 + 10 万静态，CPU headless。
//! 出口门槛：≥ 30 FPS（每 tick < 33.3 ms）。
//! 运行：cargo run --release -p vxl-phys --example m1_scale -- [threads] [static] [dynamic] [ticks] [iters]
//! 默认：threads=8, 静态 102400（320×320 瓦片）, 动态 100000, 300 tick, 迭代 16。

use std::time::Instant;

use vxl_phys::{Manifold, PhysConfig, Quat, Shape, Vec3, World};

/// 命令行参数：[threads] [static] [dynamic] [ticks] [iters]。
struct Args {
    threads: usize,
    n_static: usize,
    n_dynamic: usize,
    ticks: u32,
    iters: u32,
}

fn parse_args() -> Args {
    let mut args = std::env::args().skip(1);
    Args {
        threads: args.next().and_then(|s| s.parse().ok()).unwrap_or(8),
        n_static: args.next().and_then(|s| s.parse().ok()).unwrap_or(102_400),
        n_dynamic: args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000),
        ticks: args.next().and_then(|s| s.parse().ok()).unwrap_or(300),
        iters: args.next().and_then(|s| s.parse().ok()).unwrap_or(16),
    }
}

/// 建规模场景：静态瓦片地面 + 动态盒堆。
/// 注：地面 = 静态瓦片（不含高度场——双层地面会让每个盒子多算一次
/// 无接触的高度场对，规模档里是 10 万次/帧的白算）。
fn build_scaled_world(cfg: PhysConfig, n_static: usize, n_dynamic: usize) -> World {
    let mut w = World::new(cfg);
    let side = (n_static as f64).sqrt() as usize;
    for k in 0..n_static {
        let x = (k % side) as f32 - side as f32 * 0.5;
        let z = (k / side) as f32 - side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    let d_side = (n_dynamic as f64).sqrt() as usize + 1;
    for k in 0..n_dynamic {
        let x = (k % d_side) as f32 * 1.0 - d_side as f32 * 0.5;
        let z = (k / d_side) as f32 * 1.0 - d_side as f32 * 0.5;
        let y = 3.0 + ((k * 29) % 71) as f32 / 71.0 * 8.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

/// 逐 tick 累计量 + 抖动审计状态。
struct Acc {
    total: f64,
    worst: f64,
    /// 相位时间累计（供末尾的工作集/达成吞吐读数； 每 tick 被清零 ⇒ 这里自己累加）。
    tb_sum: f64,
    tn_sum: f64,
    ts_sum: f64,
    // ⚠️ **必须跨 tick 取峰值**：末帧快照会失真——盒子落定并入睡后求解器被清空
    // （）⇒ 8B 场景末帧读到 warm 0 条、流形 0.0 MB（实测踩过）。
    mfp_max: usize,
    warm_max: usize,
    pts_max: u64,
    cands_max: usize,
    active_ticks: u32,
    active_ms: f64,
    // **抖动审计（SPEC §3：休眠体被重复唤醒 < 1 次/秒/体）**——逐体「睡→醒」翻转计数。
    // 口径与 `m0_gates` 的那份一致（同一判据不许两处各写各的）；只在**有清醒体**的 tick 计数
    // （全睡之后不可能再有翻转），且整段在 `ms` 计时区**之外** ⇒ 不污染逐 tick 读数。
    dyn_ids: Vec<usize>,
    flips_per_body: Vec<u32>,
    prev_awake: Vec<bool>,
    max_flips: u32,
}

impl Acc {
    fn new(w: &World) -> Self {
        Acc {
            total: 0.0,
            worst: 0.0,
            tb_sum: 0.0,
            tn_sum: 0.0,
            ts_sum: 0.0,
            mfp_max: 0,
            warm_max: 0,
            pts_max: 0,
            cands_max: 0,
            active_ticks: 0,
            active_ms: 0.0,
            dyn_ids: (0..w.bodies.len())
                .filter(|&i| w.bodies.is_dynamic(i))
                .collect(),
            flips_per_body: vec![0; w.bodies.len()],
            prev_awake: (0..w.bodies.len()).map(|i| w.bodies.awake[i]).collect(),
            max_flips: 0,
        }
    }
}

/// 推进 `ticks` 个 tick，逐 tick 采样 + 打诊断行（相位行 / 求解细分 / 每 100 tick 健康读数）。
fn run_ticks(w: &mut World, ticks: u32, acc: &mut Acc) {
    for t in 1..=ticks {
        // `PhaseTimings` 是**累计值**（见 docs/M1-PLAN.md 读数陷阱）⇒ 每 tick 清零，
        // 否则「窄相/求解」两列读到的是「自首帧起的累计」——按它调参会调错方向。
        w.reset_timings();
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        acc.total += ms;
        acc.worst = acc.worst.max(ms);
        // 每 tick 一行 stderr（无缓冲，长跑可观测）。
        let tim = w.timings();
        acc.tb_sum += (tim.broadphase_us) as f64;
        acc.tn_sum += (tim.narrowphase_us) as f64;
        acc.ts_sum += (tim.solve_us) as f64;
        acc.mfp_max = acc.mfp_max.max(w.manifolds().len());
        acc.warm_max = acc.warm_max.max(w.solver.island_diag.warm_count as usize);
        acc.pts_max = acc.pts_max.max(w.solver.island_diag.points as u64);
        acc.cands_max = acc.cands_max.max(w.broad.cand_total());
        let hh = w.health();
        if hh.awake_bodies > 0 {
            acc.active_ticks += 1;
            acc.active_ms += ms;
            // 抖动审计（见上方注释）：只在活跃 tick 扫，翻转峰从这一批里取。
            for &i in &acc.dyn_ids {
                if !acc.prev_awake[i] && w.bodies.awake[i] {
                    acc.flips_per_body[i] += 1;
                    acc.max_flips = acc.max_flips.max(acc.flips_per_body[i]);
                }
                acc.prev_awake[i] = w.bodies.awake[i];
            }
        }
        print_tick_diag(w, t, ms);
        if t % 100 == 0 {
            let h = w.health();
            println!(
                "tick {t:3}: 近100均值 {:6.2} ms | contacts {} awake {} NaN {} deep {}",
                acc.total / t as f64,
                h.contacts,
                h.awake_bodies,
                h.nan_bodies,
                h.deep_penetrations,
            );
        }
    }
}

/// 一个 tick 的两段诊断（相位行 + 求解细分，均为**末子步快照**口径见下注）。
fn print_tick_diag(w: &World, t: u32, ms: f64) {
    let tim = w.timings();
    let bd = w.broad.breakdown_us();
    let th = w.broad.tree_height();
    // ⚠️ 相位覆盖口径（2026-09-22 修）：`PhaseTimings` **每子步 `+=`、harness 每 tick 清零**
    // ⇒ 下面这一行是**整 tick 的 7 相全覆盖**（`合计` 应≈左边 ms；差额＝子步循环外的簿记）。
    // 而 `w.solver.last_phase_us` 是**每子步覆写** ⇒ 那三个数只是**末子步快照**，故单独标为
    // `末子步[…]`——**别再把它们当整 tick 的解算时间**（本行曾因此少算一半，见 OPEN-PROBLEMS P3）。
    eprintln!(
        "tick {t:3}: {ms:7.2} ms | 场 {:5.2} 速积 {:5.2} 宽相 {:6.2} (AABB {:5.2} 树 {:6.2} 查询 {:6.2}) tree_h {th:3} 候选 {:7} | 窄相 {:6.2} | 解算 {:6.2} | 位积 {:6.2} | CCD {:5.2} | 合计 {:7.2} | 岛 {:5} 末子步[岛 {:6.2} 解算 {:6.2} 休眠 {:6.2}]",
        tim.fields_us as f64 / 1000.0,
        tim.integrate_vel_us as f64 / 1000.0,
        (bd.0 + bd.1 + bd.2 + bd.3) as f64 / 1000.0,
        bd.0 as f64 / 1000.0,
        bd.1 as f64 / 1000.0,
        bd.2 as f64 / 1000.0,
        w.broad.cand_total(),
        tim.narrowphase_us as f64 / 1000.0,
        tim.solve_us as f64 / 1000.0,
        tim.integrate_pos_us as f64 / 1000.0,
        tim.ccd_us as f64 / 1000.0,
        tim.total_us() as f64 / 1000.0,
        w.solver.island_count,
        w.solver.last_phase_us.0 as f64 / 1000.0,
        w.solver.last_phase_us.1 as f64 / 1000.0,
        w.solver.last_phase_us.2 as f64 / 1000.0,
    );
    // 求解细分（**同样是末子步快照**，见上注）：`gather`（= 建岛 + 按组 fill）与
    // `scope`+`scatter` 构成一次 `solve()` 调用内部；`build/warm/iter` 是**各组之和**
    // （CPU，可超墙钟）⇒ 判占比可靠、判绝对量须串行跑同字段比（`IslandDiag` 注）。
    {
        let d = &w.solver.island_diag;
        // 组墙钟（末子步、各组）：`d_solve - Σ组墙钟` ≈ 散回/回写等**组外**开销；
        // `组和 build/warm/iter` 是组**内**三段（CPU 之和），两者合起来可判"每岛开销"在哪一半。
        let gmax = d.group_us.iter().copied().max().unwrap_or(0) as f64 / 1000.0;
        let gsum: f64 = d.group_us.iter().map(|&x| x as f64).sum::<f64>() / 1000.0;
        eprintln!(
            "    ↳ 求解细分(末子步): 建岛 {:5.2} fill {:5.2} scope {:6.2} scatter {:6.2} | 组和 build {:6.2} warm {:5.2} iter {:6.2} | 组数 {:3} 组墙钟 和 {:6.2} 峰 {:6.2} | 流形 {:6} 点 {:7}",
            d.island_build_us as f64 / 1000.0,
            d.fill_us as f64 / 1000.0,
            d.scope_us as f64 / 1000.0,
            d.scatter_us as f64 / 1000.0,
            d.build_us as f64 / 1000.0,
            d.warm_us as f64 / 1000.0,
            d.iter_us as f64 / 1000.0,
            d.g_count,
            gsum,
            gmax,
            d.manifolds,
            d.points,
        );
    }
}

/// 末行总览：均值/最差 + 健康。
fn print_summary(w: &World, avg: f64, worst: f64) {
    let h = w.health();
    println!(
        "平均 {avg:.2} ms/tick → {:.1} FPS | 最差 {worst:.2} ms | NaN {} deep {}",
        1000.0 / avg,
        h.nan_bodies,
        h.deep_penetrations
    );
}

/// ── **工作集与达成吞吐**（2026-09-21 加；数据布局工程的第一步：把"访存受限"定量）──
///
/// 为什么看这两个数：P2/P3/P4/T4 的共同真因是**访存延迟**（不是带宽饱和——把四个相位的
/// 达成吞吐算出来只有 1–2 GB/s，远低于单核 10–20 GB/s）。本段把"每 tick 要触碰多少字节"
/// 与"实际达到多少 GB/s"打出来，作为后续一切数据布局改动的**基准读数**。
///
/// ⚠️ 口径：**字节数是模型**（按各相位的访问模式估），不是实测计数器——
///   热组 32B/体（位姿）+ 32B/体（速度）见 `body.rs` 文档；冷组按字段 size_of 求和；
///   求解按 200 B/接触点（约束写入约 120 + 体读取约 80）；宽相按 24B/体（叶盒）+ 32B/候选。
///   要更准需要硬件计数器（本机没有 perf），故此处的用途是**比较不同规模/不同改动的相对值**。
fn report_working_set(w: &World, ticks: u32, acc: &Acc) {
    let n = w.bodies.len() as f64;
    let act = acc.active_ticks.max(1) as f64;
    println!(
        "活跃期（awake>0 的 {} / {ticks} tick）：均 {:.2} ms/tick → {:.1} FPS | 峰值（跨 tick）流形 {} ｜ warm 槽 {} ｜ 接触点 {} ｜ 候选 {}",
        acc.active_ticks,
        acc.active_ms / act,
        1000.0 / (acc.active_ms / act).max(0.001),
        acc.mfp_max,
        acc.warm_max,
        acc.pts_max,
        acc.cands_max
    );
    // 抖动审计（SPEC §3：休眠体被重复唤醒 < 1 次/秒/体）。**分母用活跃秒**（更严：
    // 全睡之后不可能再翻转，用全期秒数会稀释）⇒ 报的是保守上界。判据进门见
    // `scripts/gate_scale.sh`（确定性量：翻转峰是场景与码的函数）。
    // ⚠️ 行尾**必须保留 ASCII 机读标签** `wake_flips=` / `wake_rate_per_s=`：门脚本按它解析
    // （照 `determinism` 打 `FINAL_HASH=0x…` 的先例）——**别在门里切中文**：
    // 实测 `sed 's/[^0-9]*([0-9]+)次…/'` 在中文前缀上匹配不上、把整行漏给判据（踩过）。
    println!(
        "抖动：单体贴最大睡醒翻转 {} 次（活跃 {:.1} s ⇒ {:.2} 次/秒/体；SPEC §3 阈值 <1 ⇒ {}）｜ wake_flips={} wake_rate_per_s={:.2}",
        acc.max_flips,
        act / 60.0,
        acc.max_flips as f64 / (act / 60.0),
        if (acc.max_flips as f64) < act / 60.0 {
            "过"
        } else {
            "**不过**"
        },
        acc.max_flips,
        acc.max_flips as f64 / (act / 60.0),
    );
    let hot = n * (32.0 + 32.0);
    let cold = n
        * (std::mem::size_of::<Shape>() as f64
            + std::mem::size_of::<vxl_phys_core::BodyType>() as f64
            + 4.0  // inv_mass
            + 12.0 // local_inv_inertia
            + 12.0 // force
            + 12.0 // torque
            + 1.0  // awake
            + 4.0  // sleep_timer
            + 4.0); // material id
    let mfs = acc.mfp_max as f64 * std::mem::size_of::<Manifold>() as f64;
    let warm = acc.warm_max as f64 * w.solver.island_diag.warm_bytes_per_slot as f64; // 实测 size_of，别估算
    let total_mb = (hot + cold + mfs + warm) / 1e6;
    println!(
        "工作集（模型）：热组 {:.1} MB ｜ 冷组 {:.1} MB ｜ 流形 {:.1} MB ｜ warm 槽 {} 条（{} B/条）≈ {:.1} MB ｜ **合计 {total_mb:.1} MB**（本机 L2/L3 ≈ 2/36 MB，见 sys_topology）",
        hot / 1e6,
        cold / 1e6,
        mfs / 1e6,
        acc.warm_max,
        w.solver.island_diag.warm_bytes_per_slot,
        warm / 1e6
    );
    // 每 tick 触碰（模型）与达成吞吐：用累计相位时间（下方按 tick 累加）。
    let mfp = acc.mfp_max as f64;
    let pts = acc.pts_max as f64;
    let cands = acc.cands_max as f64;
    let bytes_broad = n * 24.0 + cands * 32.0;
    let bytes_narrow = mfp * (96.0 + std::mem::size_of::<Manifold>() as f64);
    let bytes_solve = pts * 200.0;
    // 达成吞吐 = 字节/tick ÷ 每 tick 秒数（相位时间是**全程累计** ⇒ 先除 ticks）。
    // GB/s = b·ticks / (us_total · 1000)（b 字节，us_total 累计微秒）。
    let tk = ticks as f64;
    for (name, b, us_total) in [
        ("宽相", bytes_broad, acc.tb_sum),
        ("窄相", bytes_narrow, acc.tn_sum),
        ("解算", bytes_solve, acc.ts_sum),
    ] {
        let us_per_tick = us_total / tk;
        println!(
            "  {name}：触碰 {:.2} MB/tick ｜ 耗时 {:.2} ms/tick ｜ **达成 {:.2} GB/s**",
            b / 1e6,
            us_per_tick / 1000.0,
            b * tk / (us_total.max(1.0) * 1000.0)
        );
    }
}

/// 出口门槛判定（§3：≥ 30 FPS）。
fn print_gate(avg: f64) {
    if avg < 33.33 {
        println!("✅ §3 最低通过档（10万+10万 ≥30FPS）");
    } else {
        println!("⚠️ 未达 30 FPS");
    }
}

fn main() {
    let a = parse_args();
    let cfg = PhysConfig {
        threads: a.threads,
        velocity_iterations: a.iters,
        ..PhysConfig::default()
    };
    let mut w = build_scaled_world(cfg, a.n_static, a.n_dynamic);

    println!(
        "threads = {} | 静态 {} + 动态 {} | {} tick（预热 20）",
        a.threads, a.n_static, a.n_dynamic, a.ticks
    );
    for _ in 0..20 {
        w.step();
    }
    let mut acc = Acc::new(&w);
    run_ticks(&mut w, a.ticks, &mut acc);

    let avg = acc.total / a.ticks as f64;
    print_summary(&w, avg, acc.worst);
    report_working_set(&w, a.ticks, &acc);
    print_gate(avg);
}
