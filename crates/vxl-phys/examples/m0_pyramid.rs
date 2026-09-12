//! M0 演示：地面高度场 + 6 层盒金字塔 + 圆柱 + 球，跑 10 秒。
//! 验收点（§3/§12 M0）：无 NaN、无静默穿透、最终入睡、确定性哈希稳定。
//!
//! 运行：cargo run --release -p vxl-phys --example m0_pyramid

use vxl_phys::{HeightField, PhysConfig, Preset, Quat, Shape, Vec3, World};

fn main() {
    let mut w = World::new(PhysConfig::from_preset(Preset::Balanced));
    let ground = HeightField::flat(-25.0, -25.0, 51, 51, 1.0, 0.0);
    w.add_heightfield(ground);

    // 6 层金字塔（21 个盒）。
    let layers = 6;
    let mut bodies = 0usize;
    for layer in 0..layers {
        let count = layers - layer;
        for k in 0..count {
            let x = (k as f32 - (count as f32 - 1.0) * 0.5) * 1.05;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                vxl_phys::Vec3::new(x, 0.55 + layer as f32 * 1.02, 0.0),
                Quat::IDENTITY,
                1000.0,
            );
            bodies += 1;
        }
    }
    // 两侧圆柱 + 球。
    w.add_dynamic(
        Shape::Cylinder {
            half_height: 0.6,
            radius: 0.3,
        },
        vxl_phys::Vec3::new(-5.0, 6.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.add_dynamic(
        Shape::Sphere { radius: 0.4 },
        vxl_phys::Vec3::new(5.0, 6.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    bodies += 2;

    println!("RUST WL / vxl_phys M0 演示 —— {bodies} 动态体，预设 = Balanced");
    println!("tick | 物理耗时 ms | 活跃体 | 接触点 | 哈希(每60tick)");
    let start = std::time::Instant::now();
    let mut hash_log = String::new();
    for t in 1..=600 {
        let t0 = std::time::Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f32() * 1000.0;
        if t % 60 == 0 {
            let h = w.state_hash();
            hash_log.push_str(&format!("{h:016x} "));
            println!(
                "{t:4} | {ms:8.3} | {:6} | {:5} | {h:016x}",
                w.health().awake_bodies,
                w.health().contacts
            );
        }
    }
    let h = w.health();
    println!("总耗时 {:.2}s（含打印）", start.elapsed().as_secs_f32());
    println!("健康报告：{h:?}");
    println!("哈希序列：{hash_log}");
    if h.is_clean() {
        println!("✅ 无 NaN/Inf、无静默穿透（深度 ≤ skin×4）");
    } else {
        println!("❌ 稳定性检查未通过");
        std::process::exit(1);
    }
    let _ = &hash_log;
}
