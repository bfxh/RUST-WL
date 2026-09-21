//! M1 规模场景（§3 最低通过档）：10 万动态 + 10 万静态，CPU headless。
//! 出口门槛：≥ 30 FPS（每 tick < 33.3 ms）。
//! 运行：cargo run --release -p vxl-phys --example m1_scale -- [threads] [static] [dynamic] [ticks] [iters]
//! 默认：threads=8, 静态 102400（320×320 瓦片）, 动态 100000, 300 tick, 迭代 16。

use std::time::Instant;

use vxl_phys::{Manifold, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let n_static: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(102_400);
    let n_dynamic: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    let ticks: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);

    let cfg = PhysConfig {
        threads,
        velocity_iterations: iters,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    // 注：地面 = 静态瓦片（不含高度场——双层地面会让每个盒子多算一次
    // 无接触的高度场对，规模档里是 10 万次/帧的白算）。

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

    println!("threads = {threads} | 静态 {n_static} + 动态 {n_dynamic} | {ticks} tick（预热 20）");
    for _ in 0..20 {
        w.step();
    }
    let mut total = 0.0f64;
    let mut worst = 0.0f64;
    // 相位时间累计（供末尾的工作集/达成吞吐读数； 每 tick 被清零 ⇒ 这里自己累加）。
    let (mut tb_sum, mut tn_sum, mut ts_sum) = (0.0f64, 0.0f64, 0.0f64);
    // ⚠️ **必须跨 tick 取峰值**：末帧快照会失真——盒子落定并入睡后求解器被清空
    //（）⇒ 8B 场景末帧读到 warm 0 条、流形 0.0 MB（实测踩过）。
    let (mut mfp_max, mut warm_max, mut pts_max, mut cands_max) = (0usize, 0usize, 0u64, 0usize);
    let (mut active_ticks, mut active_ms) = (0u32, 0.0f64);
    for t in 1..=ticks {
        // `PhaseTimings` 是**累计值**（见 docs/M1-PLAN.md 读数陷阱）⇒ 每 tick 清零，
        // 否则「窄相/求解」两列读到的是「自首帧起的累计」——按它调参会调错方向。
        w.reset_timings();
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        total += ms;
        worst = worst.max(ms);
        // 每 tick 一行 stderr（无缓冲，长跑可观测）。
        let tim = w.timings();
        tb_sum += (tim.broadphase_us) as f64;
        tn_sum += (tim.narrowphase_us) as f64;
        ts_sum += (tim.solve_us) as f64;
        let bd = w.broad.breakdown_us();
        mfp_max = mfp_max.max(w.manifolds().len());
        warm_max = warm_max.max(w.solver.island_diag.warm_count as usize);
        pts_max = pts_max.max(w.solver.island_diag.points as u64);
        cands_max = cands_max.max(w.broad.cand_total());
        let hh = w.health();
        if hh.awake_bodies > 0 {
            active_ticks += 1;
            active_ms += ms;
        }
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
        if t % 100 == 0 {
            let h = w.health();
            println!(
                "tick {t:3}: 近100均值 {:6.2} ms | contacts {} awake {} NaN {} deep {}",
                total / t as f64,
                h.contacts,
                h.awake_bodies,
                h.nan_bodies,
                h.deep_penetrations,
            );
        }
    }
    let avg = total / ticks as f64;
    let h = w.health();
    println!(
        "平均 {avg:.2} ms/tick → {:.1} FPS | 最差 {worst:.2} ms | NaN {} deep {}",
        1000.0 / avg,
        h.nan_bodies,
        h.deep_penetrations
    );

    // ── **工作集与达成吞吐**（2026-09-21 加；数据布局工程的第一步：把"访存受限"定量）──
    //
    // 为什么看这两个数：P2/P3/P4/T4 的共同真因是**访存延迟**（不是带宽饱和——把四个相位的
    // 达成吞吐算出来只有 1–2 GB/s，远低于单核 10–20 GB/s）。本段把"每 tick 要触碰多少字节"
    // 与"实际达到多少 GB/s"打出来，作为后续一切数据布局改动的**基准读数**。
    //
    // ⚠️ 口径：**字节数是模型**（按各相位的访问模式估），不是实测计数器——
    //   热组 32B/体（位姿）+ 32B/体（速度）见 `body.rs` 文档；冷组按字段 size_of 求和；
    //   求解按 200 B/接触点（约束写入约 120 + 体读取约 80）；宽相按 24B/体（叶盒）+ 32B/候选。
    //   要更准需要硬件计数器（本机没有 perf），故此处的用途是**比较不同规模/不同改动的相对值**。
    {
        let n = w.bodies.len() as f64;
        let act = active_ticks.max(1) as f64;
        println!(
            "活跃期（awake>0 的 {} / {ticks} tick）：均 {:.2} ms/tick → {:.1} FPS | 峰值（跨 tick）流形 {} ｜ warm 槽 {} ｜ 接触点 {} ｜ 候选 {}",
            active_ticks,
            active_ms / act,
            1000.0 / (active_ms / act).max(0.001),
            mfp_max,
            warm_max,
            pts_max,
            cands_max
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
        let mfs = mfp_max as f64 * std::mem::size_of::<Manifold>() as f64;
        let warm = warm_max as f64 * w.solver.island_diag.warm_bytes_per_slot as f64; // 实测 size_of，别估算
        let total_mb = (hot + cold + mfs + warm) / 1e6;
        println!(
            "工作集（模型）：热组 {:.1} MB ｜ 冷组 {:.1} MB ｜ 流形 {:.1} MB ｜ warm 槽 {} 条（{} B/条）≈ {:.1} MB ｜ **合计 {total_mb:.1} MB**（本机 L2/L3 ≈ 2/36 MB，见 sys_topology）",
            hot / 1e6,
            cold / 1e6,
            mfs / 1e6,
            warm_max,
            w.solver.island_diag.warm_bytes_per_slot,
            warm / 1e6
        );
        // 每 tick 触碰（模型）与达成吞吐：用累计相位时间（下方按 tick 累加）。
        let mfp = mfp_max as f64;
        let pts = pts_max as f64;
        let cands = cands_max as f64;
        let bytes_broad = n * 24.0 + cands * 32.0;
        let bytes_narrow = mfp * (96.0 + std::mem::size_of::<Manifold>() as f64);
        let bytes_solve = pts * 200.0;
        // 达成吞吐 = 字节/tick ÷ 每 tick 秒数（相位时间是**全程累计** ⇒ 先除 ticks）。
        // GB/s = b·ticks / (us_total · 1000)（b 字节，us_total 累计微秒）。
        let tk = ticks as f64;
        for (name, b, us_total) in [
            ("宽相", bytes_broad, tb_sum),
            ("窄相", bytes_narrow, tn_sum),
            ("解算", bytes_solve, ts_sum),
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

    if avg < 33.33 {
        println!("✅ §3 最低通过档（10万+10万 ≥30FPS）");
    } else {
        println!("⚠️ 未达 30 FPS");
    }
}
