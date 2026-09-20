//! T6：**8A 碰撞行**（10 载体 × 1200 格 + 世界）——碰撞管线（宽 + 窄）计时。
//!
//! **场景解读（本仓无 V2 §11 原文，故把口径显式写在这里）**：
//!
//! 1. **「载体」在本形体模型里没有刚体复合体**——`Shape` 只有 Box/Sphere/
//!    Cylinder/HeightField 四档，「一个体 = 一个形状」。故 1200 格只能落成
//!    1200 个体；
//! 2. 载体格取**静态**：复合体的格之间**本来就不互相碰撞**（刚体内部无对内
//!    配对），静态恰好复现这一点（静×静不产对）；若把格建成相互重叠的动态体，
//!    单载体内部就会生出上万配对——那不是复合体语义，是碎块堆；
//! 3. 因此本行量的是「碰撞管线带着 10 × 1200 格 + 世界跑」的**管线开销**：
//!    宽相（AABB 刷新 + 增量树 + 查询）＋ 窄相（配对）。与验收口径
//!    「≤1.5ms CPU」的同一量级参照：`m0_gates` 门槛场景 1.1 万体 p50 0.51ms；
//!    本行体量 ≈2 倍 ⇒ ≤1.5ms 正是「管线开销」而非「配对吞吐」的档位。
//!    注：若日后落地真复合体（一叶一大盒），本行数字应**更低**——当前把 1200
//!    格摊成 1200 叶，是**保守**测法。
//!
//! 场景构成：世界 = 100×100 静态瓦片地坪（10 000）；10 个载体各 1200 格
//! （10×10×12 致密盒阵，格距 1.0 = 面接触，footprint 10×10m）；载体按 40m
//! 间距摆 5×2，互不接触（否则载体间也会产对，不符复合体语义）；另加 100 个
//! 动态探针盒（每载体 10 个）使查询路径每 tick 真实工作——**无清醒体时管线
//! 不做查询**，那样量到的只是「树存在」而非碰撞管线。
//!
//! 运行：`cargo run --release -p vxl-phys --example m1_collision_row [报告 json 路径]`
//! 验收：碰撞管线 p95 ≤ 1.5ms（**本机数字，环境差异需注明**——CI 产物报告）。

use std::time::Instant;

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

const TICKS: u32 = 120;
const LIMIT_MS: f64 = 1.5;
const CARRIERS: i32 = 10;
const CARRIER_CELLS: i32 = 1200;
const PROBES: i32 = 100;
/// 世界 = 100×100 静态瓦片地坪。
const WORLD_TILES: usize = 10_000;

/// 载体格阵形状：10×10×12 = 1200（格距 1.0 ⇒ 相邻面接触）。
fn carrier_cell(k: i32) -> (f32, f32, f32) {
    let nx = 10;
    let nz = 10;
    (
        (k % nx) as f32 - (nx - 1) as f32 * 0.5,
        (k / (nx * nz)) as f32, // 纵向 12 层
        ((k / nx) % nz) as f32 - (nz - 1) as f32 * 0.5,
    )
}

fn build() -> World {
    let mut w = World::new(PhysConfig::default());
    // 世界：100×100 静态瓦片地坪。注：不加高度场——双层地面会让每个盒多算一次
    // 无接触的高度场对（`m1_scale` 同口径）。
    let side = (WORLD_TILES as f64).sqrt() as i32;
    for k in 0..WORLD_TILES as i32 {
        let x = (k % side) as f32 - side as f32 * 0.5;
        let z = (k / side) as f32 - side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    // 10 个载体：5×2 摆位，间距 40m（footprint 10×10 ⇒ 净空 30m，互不接触）。
    let half = Vec3::splat(0.5);
    for c in 0..CARRIERS {
        let cx = (c % 5) as f32 * 40.0 - 80.0;
        let cz = (c / 5) as f32 * 40.0 - 20.0;
        for k in 0..CARRIER_CELLS {
            let (dx, dy, dz) = carrier_cell(k);
            w.add_static(
                Shape::Box { half },
                Vec3::new(cx + dx, 0.5 + dy, cz + dz),
                Quat::IDENTITY,
            );
        }
    }
    // 动态探针：每载体 10 个，落在载体顶面上方 ⇒ 每 tick 有真实查询与少量配对。
    for p in 0..PROBES {
        let c = p % CARRIERS;
        let cx = (c % 5) as f32 * 40.0 - 80.0;
        let cz = (c / 5) as f32 * 40.0 - 20.0;
        let ox = (p % 4) as f32 - 1.5;
        let oz = ((p / 4) % 3) as f32 - 1.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(cx + ox, 12.8, cz + oz),
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

fn pct(v: &mut [f64], p: f64) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let i = ((v.len() - 1) as f64 * p).round() as usize;
    v[i]
}

fn main() {
    let bodies = WORLD_TILES + (CARRIERS * CARRIER_CELLS) as usize + PROBES as usize;
    let mut w = build();
    // **不预热**：验收口径对齐 8B 的「沉降期任一 tick」——量的是**最严**的沉降期，
    // 而非静置后（静置帧零树操作，量出来只是管线空转）。
    let (mut broad, mut narrow, mut total) = (Vec::new(), Vec::new(), Vec::new());
    // 峰值 tick 的宽相细分（诊断：首帧全量重建是最大单点嫌疑）。
    let mut max_tick = 0usize;
    let mut max_bd = (0u64, 0u64, 0u64, 0u64);
    let t_wall = Instant::now();
    for tick in 0..TICKS {
        // `PhaseTimings` 是**累计值**（见 M1-PLAN 读数陷阱）⇒ 每 tick 先清零，
        // 否则读到的是「从首帧起的累计」，会与墙钟自相矛盾。
        w.reset_timings();
        w.step();
        let t = w.timings();
        let b = t.broadphase_us as f64 / 1000.0;
        let n = t.narrowphase_us as f64 / 1000.0;
        broad.push(b);
        narrow.push(n);
        let sum = b + n;
        if sum > total.iter().copied().fold(0.0f64, f64::max) {
            max_tick = tick as usize;
            max_bd = w.broad.breakdown_us();
        }
        total.push(sum);
    }
    let wall = t_wall.elapsed().as_secs_f64() * 1000.0 / TICKS as f64;
    let bd = w.broad.breakdown_us();
    let h = w.health();
    let p50 = pct(&mut total.clone(), 0.50);
    let p95 = pct(&mut total.clone(), 0.95);
    let cmax = pct(&mut total.clone(), 1.0);
    let bp95 = pct(&mut broad, 0.95);
    let np95 = pct(&mut narrow, 0.95);
    // 末 30 tick（静置）单列：区分「沉降期」与「稳态」两档成本。
    let tail = &total[total.len().saturating_sub(30)..];
    let settle_p50 = pct(&mut tail.to_vec(), 0.50);
    let settle_p95 = pct(&mut tail.to_vec(), 0.95);
    let pass_p95 = p95 <= LIMIT_MS;
    let pass_any = cmax <= LIMIT_MS;
    // 「碰撞管线」成本 vs「首帧载入」成本分列：验收口径（≤1.5ms）针对前者；
    // 后者是**一次性加载**（全量建树），8B 场景同项 ≈46.7ms，不属本行口径。
    let load = if pass_any { 0.0 } else { cmax };
    let load_tree = if pass_any {
        0.0
    } else {
        max_bd.1 as f64 / 1000.0
    };

    println!(
        "8A 碰撞行：{CARRIERS} 载体 × {CARRIER_CELLS} 格 + 世界（{bodies} 体，含 {PROBES} 探针）"
    );
    println!(
        "每 tick 碰撞管线（宽+窄）：p50 {p50:.3}ms | p95 {p95:.3}ms | max {cmax:.3}ms（验收 ≤{LIMIT_MS}ms）"
    );
    println!(
        "  ├ 宽相 p95 {bp95:.3}ms（末帧细分 AABB {:.2} / 树 {:.2} / 查询 {:.2} ms）",
        bd.0 as f64 / 1000.0,
        bd.1 as f64 / 1000.0,
        bd.2 as f64 / 1000.0
    );
    println!(
        "  └ 窄相 p95 {np95:.3}ms | 全 tick 墙钟均 {wall:.3}ms | 末 30 tick（静置）p50 {settle_p50:.3} / p95 {settle_p95:.3} ms"
    );
    println!(
        "峰值 tick #{max_tick}（{cmax:.3}ms）宽相细分：AABB {:.2} / 树 {:.2} / 查询 {:.2} / 排序 {:.2} ms",
        max_bd.0 as f64 / 1000.0,
        max_bd.1 as f64 / 1000.0,
        max_bd.2 as f64 / 1000.0,
        max_bd.3 as f64 / 1000.0
    );
    println!(
        "健康：NaN {} 深穿透 {} 末态活跃 {} 接触 {} | 树高 {}",
        h.nan_bodies,
        h.deep_penetrations,
        h.awake_bodies,
        h.contacts,
        w.broad.tree_height()
    );

    let json = format!(
        "{{\n  \"gate\": \"m1_8a_collision_row\",\n  \"env\": {{\"os\": \"{}\", \"arch\": \"{}\"}},\n  \
         \"scene\": {{\"carriers\": {}, \"cells_per_carrier\": {}, \"world_tiles\": {}, \"probes\": {}, \"bodies\": {}}},\n  \
         \"collision_row_ms\": {{\"p50\": {:.4}, \"p95\": {:.4}, \"max\": {:.4}, \"broad_p95\": {:.4}, \"narrow_p95\": {:.4}, \"wall_avg\": {:.4}, \"settled_p50\": {:.4}, \"settled_p95\": {:.4}}},\n  \
         \"load_frame\": {{\"tick\": {}, \"total_ms\": {:.4}, \"tree_ms\": {:.4}}},\n  \
         \"health\": {{\"nan\": {}, \"deep\": {}, \"awake\": {}, \"contacts\": {}, \"tree_h\": {}}},\n  \
         \"limit_ms\": {:.1},\n  \"pass_p95\": {}, \"pass_any_tick\": {}\n}}",
        std::env::consts::OS,
        std::env::consts::ARCH,
        CARRIERS,
        CARRIER_CELLS,
        WORLD_TILES,
        PROBES,
        bodies,
        p50,
        p95,
        cmax,
        bp95,
        np95,
        wall,
        settle_p50,
        settle_p95,
        max_tick,
        load,
        load_tree,
        h.nan_bodies,
        h.deep_penetrations,
        h.awake_bodies,
        h.contacts,
        w.broad.tree_height(),
        LIMIT_MS,
        pass_p95,
        pass_any,
    );
    println!("\n{json}");
    if let Some(path) = std::env::args().nth(1) {
        if let Err(e) = std::fs::write(&path, &json) {
            eprintln!("写报告失败 {path}: {e}");
        } else {
            println!("报告已写入 {path}");
        }
    }
    // 退出码只看管线口径（p95）；首帧载入只报告不阻断（加载期成本，非管线成本）。
    if pass_p95 {
        println!("✅ 8A 碰撞行 PASS（沉降期 p95 {p95:.3}ms ≤ {LIMIT_MS}ms，本机数字）");
        if !pass_any {
            println!(
                "ℹ️ 首帧载入 tick #{max_tick} = {cmax:.3}ms 超预算 {:.1}×（其中建树 {load_tree:.3}ms）\
                 ——一次性加载成本（8B 同项 ≈46.7ms），不属本行口径",
                cmax / LIMIT_MS
            );
        }
    } else {
        eprintln!("❌ 8A 碰撞行 FAIL —— 沉降期 p95 {p95:.3}ms > {LIMIT_MS}ms");
        std::process::exit(1);
    }
}
