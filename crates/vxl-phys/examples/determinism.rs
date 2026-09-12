//! M0 确定性验证（§5）：同一构造两次 600 tick 全程运行，每 60 tick 哈希比对。
//! 任何不一致 → 退出码 1（CI 阻断提交）。
//!
//! 运行：cargo run --release -p vxl-phys --example determinism

use vxl_phys::{BodyType, HeightField, PhysConfig, Quat, Recorder, Shape, Vec3, World};

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
    let mut rec = Recorder::new(60);
    for t in 1..=600 {
        w.step();
        rec.observe(t, w.state_hash());
    }
    (rec, n)
}

fn main() {
    println!("RUST WL / vxl_phys 确定性验证：两次 600 tick 全程比对（每 60 tick 状态哈希）");
    let (a, n) = build_and_run();
    let (b, _) = build_and_run();
    let ok = a.matches(&b);
    println!("动态体 {n}，比对 {} 个哈希采样", a.hashes.len());
    for ((t, ha), (_, hb)) in a.hashes.iter().zip(b.hashes.iter()) {
        let mark = if ha == hb { "==" } else { "!!" };
        println!("tick {t:4}: {ha:016x} {mark} {hb:016x}");
    }
    if ok {
        println!("✅ 确定性 PASS（bit 级一致）");
    } else {
        eprintln!("❌ 确定性 FAIL —— 阻断提交（§5）");
        std::process::exit(1);
    }
    let _ = BodyType::Static;
}
