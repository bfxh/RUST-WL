//! M0 确定性验证（§5）：**同一构造连续 10 轮** 各跑 600 tick，每 60 tick 记录
//! xxh3-128 状态哈希；任意轮与基线轮不一致 → 退出码 1（CI 阻断提交）。
//!
//! 输出 `FINAL_HASH=0x<32 hex>` 行供 CI 三编译器/双架构矩阵提取比对。
//!
//! 运行：cargo run --release -p vxl-phys --example determinism

use vxl_phys::{HeightField, PhysConfig, Quat, Recorder, Shape, Vec3, World};

const ROUNDS: usize = 10;
const TICKS: u64 = 600;
const PERIOD: u64 = 60;

fn build_and_run() -> (Recorder, u32) {
    let mut w = World::new(PhysConfig::default());
    w.add_heightfield(HeightField::flat(-15.0, -15.0, 31, 31, 1.0, 0.0));
    // 混合形状雨（盒/球/圆柱，确定性伪随机布局——无外部 RNG）。
    let mut n = 0u32;
    for k in 0..60u32 {
        let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 12.0 - 6.0;
        let z = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 12.0 - 6.0;
        let y = 2.0 + ((k.wrapping_mul(97)) % 400) as f32 / 100.0;
        let shape = match k % 3 {
            0 => Shape::Box {
                half: Vec3::splat(0.35),
            },
            1 => Shape::Sphere { radius: 0.3 },
            _ => Shape::Cylinder {
                half_height: 0.4,
                radius: 0.25,
            },
        };
        w.add_dynamic(shape, Vec3::new(x, y, z), Quat::IDENTITY, 1000.0);
        n += 1;
    }
    let mut rec = Recorder::new(PERIOD);
    for t in 1..=TICKS {
        w.step();
        rec.observe(t, w.state_hash());
    }
    (rec, n)
}

fn main() {
    println!(
        "vxl_phys 确定性验证：{ROUNDS} 轮 × {TICKS} tick（每 {PERIOD} tick xxh3-128 比对）| {} {}",
        std::env::consts::OS,
        std::env::consts::ARCH
    );
    let (baseline, n) = build_and_run();
    println!("动态体 {n}，每轮哈希采样 {} 个", baseline.hashes.len());

    let mut all_ok = true;
    for round in 2..=ROUNDS {
        let (rec, _) = build_and_run();
        if rec.matches(&baseline) {
            println!("第 {round:2} 轮：== 全等（{} 采样）", rec.hashes.len());
            continue;
        }
        all_ok = false;
        eprintln!("第 {round:2} 轮：❌ 与基线不一致");
        for ((t, hb), (_, hr)) in baseline.hashes.iter().zip(rec.hashes.iter()) {
            if hb != hr {
                eprintln!("  tick {t:4}: 基线 {hb:032x} vs 本轮 {hr:032x}");
            }
        }
    }

    let (last_tick, last_hash) = *baseline.hashes.last().expect("至少一次采样");
    println!("基线末态（tick {last_tick}）：{last_hash:032x}");
    println!("FINAL_HASH=0x{last_hash:032x}");
    if all_ok {
        println!("✅ 确定性 PASS（{ROUNDS} 轮 bit 级一致）");
    } else {
        eprintln!("❌ 确定性 FAIL —— 阻断提交（§5）");
        std::process::exit(1);
    }
}
