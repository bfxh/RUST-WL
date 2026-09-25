//! M0 出口门槛（V2 §11：**1 万盒堆 60Hz**；口径 = V1 §3 万级场景形式）：
//!
//! - **门槛场景（gate）**：1 万静态盒地坪 + 1 千动态盒雨落（与旧 `m0_bench` 同形，
//!   供跨版本对账）——p50 ≤ 16.6ms 为 M0 出口条件；
//! - **压力场景（stress，仅报告不设门槛）**：20×20×25 层 10 000 全动态密堆——
//!   实测即 M1 顶尖化的靶场基线（8B 万级档 ≤16.6ms 属 M1 出口，见 V2 §8/§11）。
//!
//! 共同检查：NaN = 0、末态深穿透 = 0；双跑哈希对拍（同预热同刻）；相位 arena
//! 容量恒定、零溢出（§0.1 #10 的 600 tick 平稳性）。任一门槛项不过 → 退出码 1。
//!
//! 运行：cargo run --release -p vxl-phys --example m0_gates [输出 json 路径]

use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

const WARMUP_TICKS: u32 = 10;
const GATE_TICKS: u32 = 600;
const STRESS_TICKS: u32 = 120;
const HASH_PERIOD: u64 = 60;
const P50_LIMIT_MS: f64 = 16.6;

/// 门槛场景：1 万静态盒地坪（100×100）+ 1 千动态盒（12~40m 雨落）。与旧版同参。
fn build_gate_scene() -> World {
    let mut w = World::new(PhysConfig::default());
    w.add_heightfield(HeightField::flat(-60.0, -60.0, 121, 121, 1.0, 0.0));
    for k in 0..10_000usize {
        let x = (k % 100) as f32 - 50.0;
        let z = (k / 100) as f32 - 50.0;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..1_000usize {
        let x = ((k * 37) % 97) as f32 / 97.0 * 40.0 - 20.0;
        let z = ((k * 53) % 89) as f32 / 89.0 * 40.0 - 20.0;
        let y = 12.0 + ((k * 29) % 71) as f32 / 71.0 * 28.0;
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

/// 压力场景：20×20 底 × 25 层 = 10 000 全动态盒（half 0.25m，间距 0.52m）。
fn build_stress_scene() -> World {
    let side = 20usize;
    let layers = 25usize;
    let spacing = 0.52f32;
    let mut w = World::new(PhysConfig::default());
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let off = (side as f32 - 1.0) * 0.5 * spacing;
    for layer in 0..layers {
        let y = 0.25 + layer as f32 * spacing;
        for row in 0..side {
            for col in 0..side {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * spacing - off, y, row as f32 * spacing - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

struct RunReport {
    per_tick_ms: Vec<f64>,
    /// 全窗最大接触深度（瞬态峰值）。
    max_depth: f32,
    nan_seen: bool,
    /// 末态深穿透计数。
    deep_final: u32,
    /// 全窗深穿透出现过的 tick 数（瞬态压力记录）。
    deep_ticks: u32,
    /// 末态活跃动体数。
    awake_final: u32,
    /// 单体贴最大「睡→醒」翻转次数（抖动审计）。
    max_wake_flips: u32,
    hash_samples: Vec<(u64, u128)>,
    arena_capacity: usize,
    arena_high_water_first: usize,
    arena_high_water_last: usize,
    arena_allocs: u64,
    arena_overflows: u64,
}

fn run(world: &mut World, ticks: u32) -> RunReport {
    let boxes = world.bodies.len();
    let mut rep = RunReport {
        per_tick_ms: Vec::with_capacity(ticks as usize),
        max_depth: 0.0,
        nan_seen: false,
        deep_final: 0,
        deep_ticks: 0,
        awake_final: 0,
        max_wake_flips: 0,
        hash_samples: Vec::new(),
        arena_capacity: world.arenas.hash.capacity(),
        arena_high_water_first: 0,
        arena_high_water_last: 0,
        arena_allocs: 0,
        arena_overflows: 0,
    };
    let mut prev_awake: Vec<bool> = world.bodies.awake.clone();
    let mut flips: Vec<u32> = vec![0; boxes];
    let mut hash_seen = false;

    for t in 1..=ticks as u64 {
        let t0 = Instant::now();
        world.step();
        rep.per_tick_ms.push(t0.elapsed().as_secs_f64() * 1000.0);

        let h = world.health();
        if h.nan_bodies > 0 {
            rep.nan_seen = true;
        }
        if h.deep_penetrations > 0 {
            rep.deep_ticks += 1;
        }
        rep.max_depth = rep.max_depth.max(h.max_depth);

        for i in 0..boxes {
            if world.bodies.is_dynamic(i) && !prev_awake[i] && world.bodies.awake[i] {
                flips[i] += 1;
            }
            prev_awake[i] = world.bodies.awake[i];
        }

        if t % HASH_PERIOD == 0 {
            let hash = world.state_hash();
            if !hash_seen {
                rep.arena_high_water_first = world.arenas.hash.high_water();
                hash_seen = true;
            }
            rep.hash_samples.push((t, hash));
        }
    }

    let h = world.health();
    rep.deep_final = h.deep_penetrations;
    rep.awake_final = h.awake_bodies;
    rep.max_wake_flips = flips.iter().copied().max().unwrap_or(0);
    rep.arena_high_water_last = world.arenas.hash.high_water();
    rep.arena_capacity = world.arenas.hash.capacity();
    rep.arena_allocs = world.arenas.hash.allocs();
    rep.arena_overflows = world.arenas.hash.overflows();
    rep
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

struct Perf {
    p50: f64,
    avg: f64,
    p95: f64,
    min: f64,
    max: f64,
}

fn perf(rep: &RunReport) -> Perf {
    let mut s = rep.per_tick_ms.clone();
    s.sort_by(f64::total_cmp);
    Perf {
        p50: percentile(&s, 0.50),
        avg: rep.per_tick_ms.iter().sum::<f64>() / rep.per_tick_ms.len() as f64,
        p95: percentile(&s, 0.95),
        min: s[0],
        max: *s.last().unwrap(),
    }
}

/// 门槛场景的完整评估：跑轮 + 双跑对拍 + 各项判据。
struct GateEval {
    boxes: usize,
    rep: RunReport,
    perf: Perf,
    twin_match: bool,
    arena_stable: bool,
    p50_ok: bool,
    clean: bool,
    jitter_ok: bool,
    pass: bool,
    flip_rate: f64,
}

impl GateEval {
    fn last_hash(&self) -> (u64, u128) {
        *self.rep.hash_samples.last().expect("哈希采样非空")
    }
}

fn eval_gate() -> GateEval {
    let mut g = build_gate_scene();
    let boxes = g.bodies.len();
    for _ in 0..WARMUP_TICKS {
        g.step();
    }
    let rep = run(&mut g, GATE_TICKS);
    let p = perf(&rep);

    // 双跑对拍：同预热 + 同刻哈希必须一致（进程内确定性抽检）。
    let mut g2 = build_gate_scene();
    for _ in 0..WARMUP_TICKS {
        g2.step();
    }
    let rep2 = run(&mut g2, 120);
    let twin_match = rep2.hash_samples.len() == 2
        && rep.hash_samples.len() >= 2
        && rep2.hash_samples[0].1 == rep.hash_samples[0].1
        && rep2.hash_samples[1].1 == rep.hash_samples[1].1;

    let arena_stable = rep.arena_high_water_first == rep.arena_high_water_last
        && rep.arena_overflows == 0
        && rep.arena_capacity > 0;
    let p50_ok = p.p50 <= P50_LIMIT_MS;
    let clean = !rep.nan_seen && rep.deep_final == 0;
    // **抖动判据（SPEC §3 的速率口径）**：休眠体被重复唤醒 **< 1 次/秒/体**。
    // 计数器早就在（`max_wake_flips`＝单体贴最大「睡→醒」翻转），但此前只在健康行**报原始计数**、
    // 未换算成速率、也未进判据 ⇒ `M1-EXIT.md` §2.3 把它登记成"未验"（本次实现时核对到已有一半，
    // 该条也随之改写）。场景是 60Hz 固定步 ⇒ 秒数 = tick 数 / 60。
    let flip_rate = rep.max_wake_flips as f64 / (rep.per_tick_ms.len().max(1) as f64 / 60.0);
    let jitter_ok = flip_rate < 1.0;
    let pass = p50_ok && clean && twin_match && arena_stable && jitter_ok;
    GateEval {
        boxes,
        rep,
        perf: p,
        twin_match,
        arena_stable,
        p50_ok,
        clean,
        jitter_ok,
        pass,
        flip_rate,
    }
}

/// 门槛场景报表（读数 + 健康 + 哈希/arena）。
fn print_gate_report(e: &GateEval) {
    let gr = &e.rep;
    let gp = &e.perf;
    let (g_tick, g_hash) = e.last_hash();
    println!(
        "\n[门槛场景] {} 盒（1 万静态 + 1 千动态，V1 形式）× {GATE_TICKS} tick",
        e.boxes
    );
    println!(
        "  每 tick 毫秒：p50 {:.3} | 均值 {:.3} | p95 {:.3} | 最小 {:.3} | 最大 {:.3}",
        gp.p50, gp.avg, gp.p95, gp.min, gp.max
    );
    println!(
        "  健康：NaN={} 末态深穿透={}（瞬态 {} tick）最大深度={:.4} 末态活跃={} 最大睡醒翻转={}（≈{:.2} 次/秒/体，SPEC §3 阈值 <1 ⇒ {}）",
        if gr.nan_seen { "有" } else { "0" },
        gr.deep_final,
        gr.deep_ticks,
        gr.max_depth,
        gr.awake_final,
        gr.max_wake_flips,
        e.flip_rate,
        if e.jitter_ok { "过" } else { "**不过**" }
    );
    println!(
        "  哈希 tick {g_tick} = 0x{g_hash:032x} | 双跑对拍 {} | arena 容量 {}B 水位 {}/{}B 分配 {} 溢出 {}",
        if e.twin_match { "一致" } else { "不一致" },
        gr.arena_capacity,
        gr.arena_high_water_first,
        gr.arena_high_water_last,
        gr.arena_allocs,
        gr.arena_overflows
    );
}

/// 压力场景（报告型；M1 靶场基线）的评估结果。
struct StressEval {
    boxes: usize,
    rep: RunReport,
    perf: Perf,
}

fn run_stress() -> StressEval {
    let mut s = build_stress_scene();
    let boxes = s.bodies.len();
    for _ in 0..WARMUP_TICKS {
        s.step();
    }
    let rep = run(&mut s, STRESS_TICKS);
    let p = perf(&rep);
    StressEval {
        boxes,
        rep,
        perf: p,
    }
}

impl StressEval {
    fn last_hash(&self) -> (u64, u128) {
        *self.rep.hash_samples.last().expect("哈希采样非空")
    }
}

/// 压力场景报表（仅报告，不设门槛）。
fn print_stress_report(s: &StressEval) {
    let sr = &s.rep;
    let sp = &s.perf;
    println!(
        "\n[压力场景（仅报告，M1 顶尖化靶场）] {} 全动态密堆 × {STRESS_TICKS} tick",
        s.boxes
    );
    println!(
        "  每 tick 毫秒：p50 {:.1} | 均值 {:.1} | 最大 {:.1}｜健康：NaN={} 末态深穿透={} 最大深度={:.3} 末态活跃={}",
        sp.p50,
        sp.avg,
        sp.max,
        if sr.nan_seen { "有" } else { "0" },
        sr.deep_final,
        sr.max_depth,
        sr.awake_final
    );
}

/// 机读报告（JSON；字段与旧版逐字一致，供 CI/基线消费）。
fn build_json(e: &GateEval, s: &StressEval) -> String {
    let gr = &e.rep;
    let gp = &e.perf;
    let sr = &s.rep;
    let sp = &s.perf;
    let (g_tick, g_hash) = e.last_hash();
    let (s_tick, s_hash) = s.last_hash();
    format!(
        concat!(
            "{{\n",
            "  \"gate\": \"m0\",\n",
            "  \"env\": {{\"os\": \"{}\", \"arch\": \"{}\"}},\n",
            "  \"gate_scene\": {{\n",
            "    \"form\": \"v1_10k_static_plus_1k_dynamic\",\n",
            "    \"boxes\": {}, \"ticks\": {}, \"warmup\": {},\n",
            "    \"perf_ms\": {{\"p50\": {:.4}, \"avg\": {:.4}, \"p95\": {:.4}, \"min\": {:.4}, \"max\": {:.4}}},\n",
            "    \"health\": {{\"nan_seen\": {}, \"deep_final\": {}, \"deep_ticks\": {}, \"max_depth\": {:.5}, \"awake_final\": {}, \"max_wake_flips\": {}, \"max_wake_rate_per_s\": {:.3}}},\n",
            "    \"hash\": {{\"final_tick\": {}, \"final\": \"0x{:032x}\", \"twin_match\": {}}},\n",
            "    \"arena\": {{\"capacity\": {}, \"high_water_first\": {}, \"high_water_last\": {}, \"allocs\": {}, \"overflows\": {}, \"stable\": {}}},\n",
            "    \"verdict\": {{\"p50_within_limit\": {}, \"clean\": {}, \"jitter_ok\": {}, \"pass\": {}}}\n",
            "  }},\n",
            "  \"stress_scene\": {{\n",
            "    \"form\": \"all_dynamic_20x20x25_stack\", \"report_only\": true,\n",
            "    \"boxes\": {}, \"ticks\": {},\n",
            "    \"perf_ms\": {{\"p50\": {:.4}, \"avg\": {:.4}, \"max\": {:.4}}},\n",
            "    \"health\": {{\"nan_seen\": {}, \"deep_final\": {}, \"max_depth\": {:.5}, \"awake_final\": {}}},\n",
            "    \"hash\": {{\"final_tick\": {}, \"final\": \"0x{:032x}\"}}\n",
            "  }},\n",
            "  \"verdict\": {{\"pass\": {}}}\n",
            "}}\n"
        ),
        std::env::consts::OS,
        std::env::consts::ARCH,
        e.boxes,
        GATE_TICKS,
        WARMUP_TICKS,
        gp.p50,
        gp.avg,
        gp.p95,
        gp.min,
        gp.max,
        gr.nan_seen,
        gr.deep_final,
        gr.deep_ticks,
        gr.max_depth,
        gr.awake_final,
        gr.max_wake_flips,
        e.flip_rate,
        g_tick,
        g_hash,
        e.twin_match,
        gr.arena_capacity,
        gr.arena_high_water_first,
        gr.arena_high_water_last,
        gr.arena_allocs,
        gr.arena_overflows,
        e.arena_stable,
        e.p50_ok,
        e.clean,
        e.jitter_ok,
        e.pass,
        s.boxes,
        STRESS_TICKS,
        sp.p50,
        sp.avg,
        sp.max,
        sr.nan_seen,
        sr.deep_final,
        sr.max_depth,
        sr.awake_final,
        s_tick,
        s_hash,
        e.pass,
    )
}

/// 门槛判定（未过 ⇒ 退出码 1，阻断 CI）。
fn print_verdict(e: &GateEval) {
    if e.pass {
        println!("✅ M0 门槛 PASS（p50 ≤ {P50_LIMIT_MS}ms、健康、双跑一致、arena 平稳）");
    } else {
        eprintln!(
            "❌ M0 门槛 FAIL —— 阻断（p50_ok {} / clean {} / twin {} / arena {}）",
            e.p50_ok, e.clean, e.twin_match, e.arena_stable
        );
        std::process::exit(1);
    }
}

fn main() {
    println!(
        "vxl_phys M0 门槛（V2 §11：1 万盒堆 60Hz）| 环境：{} {} | 固定 60Hz",
        std::env::consts::OS,
        std::env::consts::ARCH
    );

    let g = eval_gate();
    print_gate_report(&g);

    let s = run_stress();
    print_stress_report(&s);

    let json = build_json(&g, &s);
    println!("\n{json}");

    if let Some(path) = std::env::args().nth(1) {
        if let Err(e) = std::fs::write(&path, &json) {
            eprintln!("写报告失败 {path}: {e}");
        } else {
            println!("报告已写入 {path}");
        }
    }

    print_verdict(&g);
}
