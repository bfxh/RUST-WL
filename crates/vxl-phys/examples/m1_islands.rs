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
    // **岛级并行的内部拆分**（T4 饱和真因）：只有当串行占比（gather+scatter）解释不了
    // 饱和时，才轮到"负载不均/调度"这些次级解释。判读：串行占比 ≈ (1 − 1/扩展比) 的上限。
    {
        let mut w = build(clusters, hi);
        // **窗口累计**（本仓纪律：决定量必须窗口均值，不能用单点）：碎片雨有三个阶段
        // —— 下落（几乎无流形）→ 落定峰值 → **睡眠塌缩**（子岛睡眠生效后流形数骤降，
        // 实测 60 tick 末帧 11836 → 400 tick 末帧 707）⇒ 末帧单点与"全程累计均值"**都不代表场景成本**。
        // 这里取**后半程**窗口（tick ≥ 半程）求均值，并报出窗口端点与流形区间供判读。
        let (mut wf, mut wm, mut wp, mut wn) = (0u64, 0u64, 0u64, 0u64);
        let (mut mf_min, mut mf_max) = (u32::MAX, 0u32);
        let (mut it_sum, mut wm_sum) = (0u64, 0u64);
        for t in 0..(10 + ticks) {
            w.step();
            if t >= (10 + ticks) / 2 {
                let dd = &w.solver.island_diag;
                wf += dd.build_us;
                wm += dd.manifolds as u64;
                wp += dd.points as u64;
                it_sum += dd.iter_us;
                wm_sum += dd.warm_us;
                mf_min = mf_min.min(dd.manifolds);
                mf_max = mf_max.max(dd.manifolds);
                wn += 1;
            }
        }
        let d = w.solver.island_diag.clone();
        let gmax = d.group_us.iter().copied().max().unwrap_or(0);
        let gmin = d.group_us.iter().copied().min().unwrap_or(0);
        let gmean = if d.group_us.is_empty() {
            0.0
        } else {
            d.group_us.iter().sum::<u64>() as f64 / d.group_us.len() as f64
        };
        let mmin = d.group_manifs.iter().copied().min().unwrap_or(0);
        let mmax = d.group_manifs.iter().copied().max().unwrap_or(0);
        println!(
            "岛并行拆分（最后 tick）：岛 {} 流形 {} 组数 {}｜gather {} µs（串行）｜scope {} µs（并行）｜scatter {} µs（串行）｜点数 {}",
            d.islands,
            d.manifolds,
            d.g_count,
            d.gather_us,
            d.scope_us,
            d.scatter_us,
            w.solver.last_points.0
        );
        println!(
            "   每组耗时 µs: min {gmin} / 均值 {gmean:.0} / max {gmax}（离散度 {:.2}×）｜每组流形: min {mmin} / max {mmax}",
            if gmin > 0 { gmax as f64 / gmin as f64 } else { 0.0 }
        );
        // **规范指标（唯一口径）**：窗口均值（后半程 {wn} 个子步）——约束构建 CPU 合计
        // （各组之和）÷ 同窗口的流形/点数 ⇒ ns/流形、ns/点。**跨场景/跨改动比较只用这一组**，
        // 并同时看**窗口内的流形区间**（好判"测的是哪个阶段"）。
        println!(
            "   规范指标（窗口均值，后半程 {wn} 子步；流形 {mf_min}–{mf_max}）：约束构建 {:.0} µs/解算 ⇒ **{:.0} ns/流形、{:.1} ns/点**｜热启动 {:.0} µs｜迭代 {:.0} µs",
            wf as f64 / wn.max(1) as f64,
            1e3 * wf as f64 / wm.max(1) as f64,
            1e3 * wf as f64 / wp.max(1) as f64,
            wm_sum as f64 / wn.max(1) as f64,
            it_sum as f64 / wn.max(1) as f64
        );
        let mf = d.manifolds.max(1) as f64;
        let pt = d.points.max(1) as f64;
        println!(
            "   规范指标（同帧）：约束构建 {} µs ⇒ **{:.0} ns/流形、{:.1} ns/点**｜热启动 {} µs｜迭代 {} µs｜流形 {} 点 {}｜占比 构建{:.0}%/热启动{:.0}%/迭代{:.0}%",
            d.build_us,
            1e3 * d.build_us as f64 / mf,
            1e3 * d.build_us as f64 / pt,
            d.warm_us,
            d.iter_us,
            d.manifolds,
            d.points,
            100.0 * d.build_us as f64 / d.build_us.max(1) as f64,
            100.0 * d.warm_us as f64 / d.build_us.max(1) as f64,
            100.0 * d.iter_us as f64 / d.build_us.max(1) as f64
        );
        // 解算内部细分（累计）：(建岛, 约束构建, 热启动预施加, 迭代扫掠)。
        let d4 = w.solver.last_detail_us;
        let tot_d: f64 = d4.iter().map(|&x| x as f64).sum::<f64>().max(1.0);
        println!(
            "   解算细分（累计 µs）：建岛 {}（{:.0}%）｜约束构建 {}（{:.0}%）｜热启动预施加 {}（{:.0}%）｜迭代扫掠 {}（{:.0}%）",
            d4[0],
            100.0 * d4[0] as f64 / tot_d,
            d4[1],
            100.0 * d4[1] as f64 / tot_d,
            d4[2],
            100.0 * d4[2] as f64 / tot_d,
            d4[3],
            100.0 * d4[3] as f64 / tot_d
        );
    }
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
