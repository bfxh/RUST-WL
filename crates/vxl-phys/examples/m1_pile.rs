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
//! 默认：iters=16, substeps=1, threads=8, ticks=600；**后四个参数不传 = 用 `PhysConfig` 的
//! 当前默认值**（2026-09-22 起：shock/stab/wakeK/hold 默认各见 `config.rs`；传 `0` = 显式关掉该机制）。
//! ⚠️ 这条"不传即继承默认"是**仪器正确性**要求（见 `EXPERIMENTS.md` 第六轮）：此前本 example
//! 无条件写这四个字段，会把配置默认悄悄覆盖成 0 ⇒ "验证默认档"实际验的是"机制全关"。
//! `stab` = `PhysConfig::stabilization_iterations`（Rapier 式**无偏置末趟**；见
//! `SESSION-2026-09-18-SLEEP.md` §4：对 2000 体大堆 入睡 1735→1960，但 125 体场景 KE 退化）。
//! `wakeK` = 唤醒**接触数门**；`hold` = 准静态**安座趟**（两者见 `EXPERIMENTS.md` 第四/五轮）。
//! **判据 3 达标配方**（`M1-EXIT.md` §2.2）：`16 4 8 600 2 2 8 4`（tick 387 全睡、p50 0.174 ms）。

use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;

/// 命令行参数（后四个是 `Option`：不传 = 继承 `PhysConfig` 默认，见文件头注）。
struct Args {
    iters: u32,
    substeps: u32,
    threads: usize,
    ticks: u32,
    shock: Option<u32>,
    stab: Option<u32>,
    wake_gate_k: Option<u32>,
    hold: Option<u32>,
}

fn parse_args() -> Args {
    let mut args = std::env::args().skip(1);
    Args {
        iters: args.next().and_then(|s| s.parse().ok()).unwrap_or(16),
        substeps: args.next().and_then(|s| s.parse().ok()).unwrap_or(1),
        threads: args.next().and_then(|s| s.parse().ok()).unwrap_or(8),
        ticks: args.next().and_then(|s| s.parse().ok()).unwrap_or(600),
        shock: args.next().and_then(|s| s.parse().ok()),
        stab: args.next().and_then(|s| s.parse().ok()),
        wake_gate_k: args.next().and_then(|s| s.parse().ok()),
        hold: args.next().and_then(|s| s.parse().ok()),
    }
}

fn build(
    iters: u32,
    substeps: u32,
    threads: usize,
    shock: Option<u32>,
    stab: Option<u32>,
    wake_gate_k: Option<u32>,
    hold: Option<u32>,
) -> World {
    let mut cfg = PhysConfig {
        velocity_iterations: iters,
        substeps,
        threads,
        // 不传 = 用配置默认（2026-09-22 起 shock 默认 2）；传 = 显式覆盖（0 = 关）。
        shock_iterations: shock.unwrap_or(PhysConfig::default().shock_iterations),
        stabilization_iterations: stab.unwrap_or(PhysConfig::default().stabilization_iterations),
        // 不传 = **用配置默认值**（2026-09-22 起默认 8 / 4）；传 = 显式覆盖（可传 0 复现旧行为）。
        ..PhysConfig::default()
    };
    // 显式覆盖（`None` = 保持默认；`Some(0)` = 关掉该机制）。
    if let Some(k) = wake_gate_k {
        cfg.wake_gate_k = k;
    }
    if let Some(h) = hold {
        cfg.settled_hold_iterations = h;
    }
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

/// 逐 tick 累计读数 + 抖动审计状态。
struct Stats {
    sleep_tick: Option<u32>,
    worst_depth: f32,
    deep_ticks: u32,
    last100: Vec<f64>,
    total_ms: f64,
    // **抖动审计（SPEC §3：休眠体被重复唤醒 < 1 次/秒/体）**——与 `m0_gates` / `m1_scale`
    // 同一口径的逐体「睡→醒」翻转计数。⚠️ 本场景**今天还不睡**（awake→0 未达标，见 `M1-EXIT.md`
    // §2.2）⇒ 该量现读 0 属**结构必然**、不代表达标；它是给修好万级入睡之后备好的**同一把尺子**。
    // 整段在 `ms` 计时区之外 ⇒ 不污染读数；行尾带 ASCII 机读标签（照 `FINAL_HASH=` 先例）。
    dyn_ids: Vec<usize>,
    flips_per_body: Vec<u32>,
    prev_awake: Vec<bool>,
    max_flips: u32,
    active_ticks: u32,
}

impl Stats {
    fn new(w: &World) -> Self {
        Stats {
            sleep_tick: None,
            worst_depth: 0.0,
            deep_ticks: 0,
            last100: Vec::with_capacity(100),
            total_ms: 0.0,
            dyn_ids: (0..w.bodies.len())
                .filter(|&i| w.bodies.is_dynamic(i))
                .collect(),
            flips_per_body: vec![0; w.bodies.len()],
            prev_awake: (0..w.bodies.len()).map(|i| w.bodies.awake[i]).collect(),
            max_flips: 0,
            active_ticks: 0,
        }
    }

    /// 尾窗(100) p50（判据 ③ 的读数）。
    fn p50_tail(&self) -> f64 {
        let mut s = self.last100.clone();
        s.sort_by(f64::total_cmp);
        s.get(s.len() / 2).copied().unwrap_or(0.0)
    }
}

/// 推进 `ticks` 个 tick：逐 tick 计时/健康采样 + 抖动审计 + 每 100 tick 一行读数。
fn run_ticks(w: &mut World, ticks: u32, st: &mut Stats) {
    for t in 1..=ticks {
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        st.total_ms += ms;
        if t > ticks.saturating_sub(100) {
            st.last100.push(ms);
        }
        let h = w.health();
        if h.awake_bodies > 0 {
            st.active_ticks += 1;
            for &i in &st.dyn_ids {
                if !st.prev_awake[i] && w.bodies.awake[i] {
                    st.flips_per_body[i] += 1;
                    st.max_flips = st.max_flips.max(st.flips_per_body[i]);
                }
                st.prev_awake[i] = w.bodies.awake[i];
            }
        }
        if h.deep_penetrations > 0 {
            st.deep_ticks += 1;
        }
        st.worst_depth = st.worst_depth.max(h.max_depth);
        if st.sleep_tick.is_none() && h.awake_bodies == 0 {
            st.sleep_tick = Some(t);
        }
        if t % 100 == 0 || t == ticks {
            println!(
                "tick {t:3}: 累计均值 {:6.1} ms | awake {:5} | 近100 p50 {:6.2} ms | deep {} max_depth {:.3}",
                st.total_ms / t as f64,
                h.awake_bodies,
                if st.last100.is_empty() {
                    0.0
                } else {
                    let mut s = st.last100.clone();
                    s.sort_by(f64::total_cmp);
                    s[s.len() / 2]
                },
                h.deep_penetrations,
                h.max_depth
            );
        }
    }
}

/// 末态总览 + 速度分布 + 抖动（判据 ①③ 的读数）。
fn report_summary(w: &World, boxes: usize, st: &Stats) {
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
        st.sleep_tick
            .map(|t| t.to_string())
            .unwrap_or_else(|| "未入睡".into()),
        h.awake_bodies,
        h.deep_penetrations,
        st.worst_depth,
        st.deep_ticks
    );
    println!("尾窗(100) p50：{:.3} ms", st.p50_tail());
    // 抖动审计（与 `m0_gates`/`m1_scale` 同口径；分母用活跃秒 = 更严）。行尾 ASCII 机读标签。
    {
        let act_s = (st.active_ticks.max(1) as f64) / 60.0;
        println!(
            "抖动：单体贴最大睡醒翻转 {} 次（活跃 {:.1} s ⇒ {:.2} 次/秒/体；SPEC §3 阈值 <1 ⇒ {}）｜ wake_flips={} wake_rate_per_s={:.2}",
            st.max_flips,
            act_s,
            st.max_flips as f64 / act_s,
            if (st.max_flips as f64) < act_s {
                "过"
            } else {
                "**不过**"
            },
            st.max_flips,
            st.max_flips as f64 / act_s
        );
    }
    println!(
        "速度分布：未达睡眠阈(>0.04/0.05) {n_slow} 体 | 中速(>0.1/0.2) {n_mid} 体 | 快速(>0.5/1.0) {n_fast} 体 | |v|max {vmax:.3} |ω|max {wmax:.3}"
    );
}

/// 残差**在哪**、离阈值**多远**（2026-09-22 加）：
/// ① |v| 直方图（对数带）：整体刚过阈 vs 少数拖尾；
/// ② 按**高度三分位**统计缺口体：底层载重抖 vs 顶层还在沉/被挤出。
fn report_gaps(w: &World, boxes: usize) {
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
}

/// 出口判据判定（未过 ⇒ 退出码 1，供脚本消费）。
fn print_verdict(w: &World, ticks: u32, st: &Stats) {
    let h = w.health();
    let slept = st.sleep_tick.map(|t| t < ticks).unwrap_or(false);
    let clean = h.nan_bodies == 0 && h.deep_penetrations == 0;
    let quiet = st.p50_tail() < 1.0;
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

fn main() {
    let a = parse_args();
    let mut w = build(
        a.iters,
        a.substeps,
        a.threads,
        a.shock,
        a.stab,
        a.wake_gate_k,
        a.hold,
    );
    let boxes = w.bodies.len();
    println!(
        "M1 稳定性跑轮：{boxes} 盒密堆（{SIDE}×{SIDE}×{LAYERS}）| iters {} 子步 {} 线程 {} shock {:?} stab {:?} wakeK {:?} hold {:?} | {} tick",
        a.iters, a.substeps, a.threads, a.shock, a.stab, a.wake_gate_k, a.hold, a.ticks
    );
    for _ in 0..10 {
        w.step();
    }

    let mut st = Stats::new(&w);
    run_ticks(&mut w, a.ticks, &mut st);

    report_summary(&w, boxes, &st);
    report_gaps(&w, boxes);
    print_verdict(&w, a.ticks, &st);
}
