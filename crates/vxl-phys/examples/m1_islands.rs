//! T4：**多小岛场景（碎片雨）并行扩展 + 串行 bit 级一致**。
//!
//! 验收（docs/M1-PLAN.md T4）：多小岛场景并行扩展 **≥3×/8 线程**；与串行
//! **bit 级一致保持**（§5 契约：岛间体集合不相交 ⇒ 解算结果与线程数无关）。
//!
//! **场景（碎片雨）**：`clusters` 个小簇，每簇 3 体（簇内互不接触 ⇒ 各自成岛），
//! 从不同高度落下散开；地面 = 静态盒。
//! 同一场景跑两遍（threads=1 / threads=8）：
//! - **扩展比** = 解算相位累计耗时之比（T4 的岛级并行就在这条路径上）+ 总 tick 之比；
//! - **一致性** = 末态 `state_hash()` 必须逐位相同。
//!
//! 运行：`cargo run --release -p vxl-phys --example m1_islands -- [簇数=4000] [tick=120] [高线程数=8]`

use std::time::Instant;

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 建碎片雨：`clusters` 簇 × 3 体；簇间无接触，簇内也留 ≥2m 空隙。
fn build(clusters: usize, threads: usize) -> World {
    let cfg = PhysConfig {
        threads,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(400.0, 0.5, 400.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let side = (clusters as f64).sqrt() as usize + 1;
    for c in 0..clusters {
        let cx = (c % side) as f32 * 4.0;
        let cz = (c / side) as f32 * 4.0;
        for k in 0..3usize {
            let y = 4.0 + k as f32 * 2.5 + ((c * 7 + k * 13) % 11) as f32 * 0.2;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.25),
                },
                Vec3::new(cx + k as f32 * 2.0, y, cz),
                Quat::IDENTITY,
                1000.0,
            );
        }
    }
    w
}

fn run(clusters: usize, ticks: usize, threads: usize) -> (f64, f64, u128, usize) {
    // 返回 (总 ms, 解算 ms, 末态哈希, 清醒数)；相位细分在 main 里另取（见下）。
    let mut w = build(clusters, threads);
    for _ in 0..10 {
        w.step();
    }
    let t0 = Instant::now();
    for _ in 0..ticks {
        w.step();
    }
    let total_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let solve_ms = w.solver.last_phase_us.1 as f64 / 1000.0;
    let h = w.state_hash();
    let awake = w.health().awake_bodies as usize;
    (total_ms, solve_ms, h, awake)
}

/// 只跑不打印的相位版：返回 `(宽相, 窄相, 求解, 积分, 其它, 合计)` µs 供 main 汇总。
/// **为什么需要它**：T4 门量的是"解算相位"耗时比，若该相位在 tick 里占比很小
/// （实测本机 2026-09-21 仅 0.5%），门的分子/分母口径就先于并行度成为瓶颈 —— 见相位占比行。
#[allow(clippy::type_complexity)]
fn run_phases(clusters: usize, ticks: usize, threads: usize) -> (u64, u64, u64, u64, u64, u64) {
    let mut w = build(clusters, threads);
    for _ in 0..10 {
        w.step();
    }
    w.reset_timings();
    for _ in 0..ticks {
        w.step();
    }
    let t = w.timings();
    (
        t.broadphase_us,
        t.narrowphase_us,
        t.solve_us,
        t.integrate_vel_us + t.integrate_pos_us,
        t.fields_us + t.ccd_us,
        t.total_us(),
    )
}

fn main() {
    let mut args = std::env::args().skip(1);
    let clusters: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(4000);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(120);
    let hi: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);

    println!(
        "碎片雨：{clusters} 簇 × 3 体 = {} 体 | {ticks} tick（预热 10）| 岛级并行验收 ≥3×（1 → {hi} 线程）",
        clusters * 3
    );
    let (t1, s1, h1, a1) = run(clusters, ticks, 1);
    let (th, sh, hh, ah) = run(clusters, ticks, hi);
    let sp_solve = s1 / sh.max(0.001);
    let sp_total = t1 / th.max(0.001);

    println!("threads=1 : 总 {t1:8.1}ms | 解算 {s1:8.1}ms | 末态清醒 {a1:5} | hash {h1:#x}");
    println!("threads={hi} : 总 {th:8.1}ms | 解算 {sh:8.1}ms | 末态清醒 {ah:5} | hash {hh:#x}");
    println!("扩展比：解算 **{sp_solve:.2}×** | 总 tick {sp_total:.2}×");
    let (bp, np, sv, ig, misc, tt) = run_phases(clusters, ticks, hi);
    println!(
        "相位占比（threads={hi}，{ticks} tick）：宽相 {:.1}% | 窄相 {:.1}% | 求解 {:.1}% | 积分 {:.1}% | 其它 {:.1}% | 合计 {:.1} ms（求解 {:.1} ms = {:.4} ms/tick）",
        100.0 * bp as f64 / tt.max(1) as f64,
        100.0 * np as f64 / tt.max(1) as f64,
        100.0 * sv as f64 / tt.max(1) as f64,
        100.0 * ig as f64 / tt.max(1) as f64,
        100.0 * misc as f64 / tt.max(1) as f64,
        tt as f64 / 1000.0,
        sv as f64 / 1000.0,
        sv as f64 / 1000.0 / ticks as f64
    );
    // **fork-join 平台开销**（T4 上限的直接嫌疑）：求解器每子步一次性开  个
    // scoped worker（solver 注释按 Windows 实测 ≈90 µs/个估），这里在本机同形量一遍——
    // 若远高于 90 µs，则"≥3×"在负载轻的场景里会先被 spawn/join 吃掉。
    {
        let n = 100usize;
        let t0 = Instant::now();
        for _ in 0..n {
            std::thread::scope(|sc| {
                for _ in 1..hi {
                    sc.spawn(|| {});
                }
            });
        }
        let per = t0.elapsed().as_secs_f64() * 1e6 / n as f64;
        println!(
            "fork-join 开销（本机、{hi} 线程、空活）：**{:.1} µs/次**（每次开 {} 个 worker；             求解器每子步一次）⇒ 每 tick ≈ {:.2} ms（substeps={}）",
            per,
            hi - 1,
            per * 1e-3 * 2.0,
            2
        );
    }
    let same = h1 == hh;
    println!(
        "串行/并行末态哈希：{}（{}）",
        if same {
            "逐位一致 ✅"
        } else {
            "不一致 ❌"
        },
        if same {
            h1.to_string()
        } else {
            format!("{h1:#x} vs {hh:#x}")
        }
    );
    let pass = sp_solve >= 3.0 && same;
    if pass {
        println!("✅ T4 PASS（解算扩展 {sp_solve:.2}× ≥ 3× 且 bit 级一致）");
    } else {
        eprintln!(
            "❌ T4 FAIL —— 解算扩展 {sp_solve:.2}×（需 ≥3×）/ bit 级一致 {}",
            if same { "是" } else { "否" }
        );
        std::process::exit(1);
    }
}
