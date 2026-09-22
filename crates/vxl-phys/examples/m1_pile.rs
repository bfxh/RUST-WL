//! M1 稳定性跑轮（§11 M1 出口「万级堆叠稳定」的验收工具）：
//! 20×20×25 = 10 000 全动态密堆，60Hz 固定步，600 tick，headless。
//!
//! 出口判据（本跑轮直接判定）：
//!   ① `awake → 0`（堆在窗口内入睡，允许指定 tick 前）；
//!   ② 末态 `deep = 0`（静默穿透=0）且 `max_depth ≤ skin×4`；
//!   ③ 入睡后 p50 显著低于活动期（接近零）——稳态不再耗预算。
//!
//! 运行：
//!   cargo run --release -p vxl-phys --example m1_pile -- [iters] [substeps] [threads] [ticks] [shock] [stab] [wakeK] [hold]
//! 默认：iters=16, substeps=1, threads=8, ticks=600, shock=0, stab=0（与 m0_gates 压力场景同构）。
//! `stab` = `PhysConfig::stabilization_iterations`（Rapier 式**无偏置末趟**，默认关；见
//! `SESSION-2026-09-18-SLEEP.md` §4：对 2000 体大堆 入睡 1735→1960，但 125 体场景 KE 退化
//! ⇒ 默认关、按需开）。**金样保真档配方 = substeps 8 + stab 2**。

use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;

fn build(
    iters: u32,
    substeps: u32,
    threads: usize,
    shock: u32,
    stab: u32,
    wake_gate_k: u32,
    hold: u32,
) -> World {
    let cfg = PhysConfig {
        velocity_iterations: iters,
        substeps,
        threads,
        shock_iterations: shock,
        stabilization_iterations: stab,
        wake_gate_k,
        settled_hold_iterations: hold,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let off = (SIDE as f32 - 1.0) * 0.5 * SPACING;
    for layer in 0..LAYERS {
        let y = 0.25 + layer as f32 * SPACING;
        for row in 0..SIDE {
            for col in 0..SIDE {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * SPACING - off, y, row as f32 * SPACING - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

fn main() {
    let mut args = std::env::args().skip(1);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let substeps: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let ticks: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let shock: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let stab: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let wake_gate_k: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let hold: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);

    let mut w = build(iters, substeps, threads, shock, stab, wake_gate_k, hold);
    let boxes = w.bodies.len();
    println!(
        "M1 稳定性跑轮：{boxes} 盒密堆（{SIDE}×{SIDE}×{LAYERS}）| iters {iters} 子步 {substeps} 线程 {threads} shock {shock} stab {stab} wakeK {wake_gate_k} hold {hold} | {ticks} tick"
    );
    for _ in 0..10 {
        w.step();
    }

    let mut sleep_tick: Option<u32> = None;
    let mut worst_depth = 0.0f32;
    let mut deep_ticks = 0u32;
    let mut last100: Vec<f64> = Vec::with_capacity(100);
    let mut total_ms = 0.0f64;
    // **抖动审计（SPEC §3：休眠体被重复唤醒 < 1 次/秒/体）**——与 `m0_gates` / `m1_scale`
    // 同一口径的逐体「睡→醒」翻转计数。⚠️ 本场景**今天还不睡**（awake→0 未达标，见 `M1-EXIT.md`
    // §2.2）⇒ 该量现读 0 属**结构必然**、不代表达标；它是给修好万级入睡之后备好的**同一把尺子**。
    // 整段在 `ms` 计时区之外 ⇒ 不污染读数；行尾带 ASCII 机读标签（照 `FINAL_HASH=` 先例）。
    let dyn_ids: Vec<usize> = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .collect();
    let mut flips_per_body: Vec<u32> = vec![0; w.bodies.len()];
    let mut prev_awake: Vec<bool> = (0..w.bodies.len()).map(|i| w.bodies.awake[i]).collect();
    let mut max_flips: u32 = 0;
    let mut active_ticks: u32 = 0;
    for t in 1..=ticks {
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        total_ms += ms;
        if t > ticks.saturating_sub(100) {
            last100.push(ms);
        }
        let h = w.health();
        if h.awake_bodies > 0 {
            active_ticks += 1;
            for &i in &dyn_ids {
                if !prev_awake[i] && w.bodies.awake[i] {
                    flips_per_body[i] += 1;
                    max_flips = max_flips.max(flips_per_body[i]);
                }
                prev_awake[i] = w.bodies.awake[i];
            }
        }
        if h.deep_penetrations > 0 {
            deep_ticks += 1;
        }
        worst_depth = worst_depth.max(h.max_depth);
        if sleep_tick.is_none() && h.awake_bodies == 0 {
            sleep_tick = Some(t);
        }
        if t % 100 == 0 || t == ticks {
            println!(
                "tick {t:3}: 累计均值 {:6.1} ms | awake {:5} | 近100 p50 {:6.2} ms | deep {} max_depth {:.3}",
                total_ms / t as f64,
                h.awake_bodies,
                if last100.is_empty() {
                    0.0
                } else {
                    let mut s = last100.clone();
                    s.sort_by(f64::total_cmp);
                    s[s.len() / 2]
                },
                h.deep_penetrations,
                h.max_depth
            );
        }
    }

    let mut s = last100.clone();
    s.sort_by(f64::total_cmp);
    let p50_tail = s.get(s.len() / 2).copied().unwrap_or(0.0);
    let h = w.health();

    // 速度分布诊断（入睡判据是「全岛 |v|<0.04 且 |ω|<0.05 持续 0.5s」——
    // 这里量化离判据还有多远：>0.04 / >0.1 / >0.5 各多少体，最大速度多少）。
    let (mut n_slow, mut n_mid, mut n_fast) = (0u32, 0u32, 0u32);
    let (mut vmax, mut wmax) = (0.0f32, 0.0f32);
    for i in 0..boxes {
        if !w.bodies.is_dynamic(i) {
            continue;
        }
        let v = w.bodies.linvel[i].length();
        let wv = w.bodies.angvel(i).length();
        vmax = vmax.max(v);
        wmax = wmax.max(wv);
        if v > 0.04 || wv > 0.05 {
            n_slow += 1;
        }
        if v > 0.1 || wv > 0.2 {
            n_mid += 1;
        }
        if v > 0.5 || wv > 1.0 {
            n_fast += 1;
        }
    }

    println!("----");
    println!(
        "入睡 tick：{} | 末态 awake {} | 末态 deep {} | 最大深度（全窗）{:.4} | 深穿透 tick 数 {}",
        sleep_tick
            .map(|t| t.to_string())
            .unwrap_or_else(|| "未入睡".into()),
        h.awake_bodies,
        h.deep_penetrations,
        worst_depth,
        deep_ticks
    );
    println!("尾窗(100) p50：{p50_tail:.3} ms");
    // 抖动审计（与 `m0_gates`/`m1_scale` 同口径；分母用活跃秒 = 更严）。行尾 ASCII 机读标签。
    {
        let act_s = (active_ticks.max(1) as f64) / 60.0;
        println!(
            "抖动：单体贴最大睡醒翻转 {} 次（活跃 {:.1} s ⇒ {:.2} 次/秒/体；SPEC §3 阈值 <1 ⇒ {}）｜ wake_flips={} wake_rate_per_s={:.2}",
            max_flips,
            act_s,
            max_flips as f64 / act_s,
            if (max_flips as f64) < act_s {
                "过"
            } else {
                "**不过**"
            },
            max_flips,
            max_flips as f64 / act_s
        );
    }
    println!(
        "速度分布：未达睡眠阈(>0.04/0.05) {n_slow} 体 | 中速(>0.1/0.2) {n_mid} 体 | 快速(>0.5/1.0) {n_fast} 体 | |v|max {vmax:.3} |ω|max {wmax:.3}"
    );
    // —— 诊断（2026-09-22 加）：残差**在哪**、离阈值**多远** ——
    // ① |v| 直方图（对数带）：整体刚过阈 vs 少数拖尾；
    // ② 按**高度三分位**统计缺口体：底层载重抖 vs 顶层还在沉/被挤出。
    let mut bins = [0u32; 7];
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for i in 0..boxes {
        let lin = w.bodies.linvel[i].length();
        let ang = w.bodies.angvel(i).length();
        let idx = if lin < 0.02 {
            0
        } else if lin < 0.04 {
            1
        } else if lin < 0.08 {
            2
        } else if lin < 0.16 {
            3
        } else if lin < 0.32 {
            4
        } else if lin < 1.0 {
            5
        } else {
            6
        };
        if lin >= 0.04 || ang >= 0.05 {
            bins[idx] += 1; // 只统计"离判据还差"的体（线速或角速任一超阈）
        }
        ymin = ymin.min(w.bodies.position[i].y);
        ymax = ymax.max(w.bodies.position[i].y);
    }
    let span = (ymax - ymin).max(1e-6);
    let (mut lo3, mut mid3, mut hi3) = (0u32, 0u32, 0u32);
    for i in 0..boxes {
        let lin = w.bodies.linvel[i].length();
        let ang = w.bodies.angvel(i).length();
        if lin < 0.04 && ang < 0.05 {
            continue;
        }
        let yy = (w.bodies.position[i].y - ymin) / span;
        if yy < 1.0 / 3.0 {
            lo3 += 1;
        } else if yy < 2.0 / 3.0 {
            mid3 += 1;
        } else {
            hi3 += 1;
        }
    }
    println!(
        "  缺口 |v| 直方图：<0.02 {} | <0.04 {} | <0.08 {} | <0.16 {} | <0.32 {} | <1.0 {} | ≥1.0 {}",
        bins[0], bins[1], bins[2], bins[3], bins[4], bins[5], bins[6]
    );
    println!("  缺口体高度三分位（y {ymin:.3}..{ymax:.3}）：底 {lo3} | 中 {mid3} | 顶 {hi3}");

    let slept = sleep_tick.map(|t| t < ticks).unwrap_or(false);
    let clean = h.nan_bodies == 0 && h.deep_penetrations == 0;
    let quiet = p50_tail < 1.0;
    if slept && clean && quiet {
        println!("✅ M1 万级堆叠稳定 PASS");
    } else {
        println!(
            "❌ M1 稳定 FAIL（入睡 {} / 干净 {} / 稳态安静 {}）",
            slept, clean, quiet
        );
        std::process::exit(1);
    }
}
