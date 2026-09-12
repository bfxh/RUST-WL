//! M0 基准（§3 最低通过档的 M0 缩样）：1 万静态 + 1 千动态，CPU headless。
//! 规格 M0 出口：≥ 60 Hz（每 tick < 16.6 ms）。
//! Criterion 基准随 M1 引入（需 registry 依赖）；此处用 std Instant 报原始数据。
//!
//! 运行：cargo run --release -p vxl-phys --example m0_bench

use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut w = World::new(PhysConfig::default());

    // 静态地板。
    w.add_heightfield(HeightField::flat(-60.0, -60.0, 121, 121, 1.0, 0.0));

    // 1 万静态盒：100×100 网格瓦片（half 0.5，顶面 y=1.0）。
    let n_static = 10_000usize;
    for k in 0..n_static {
        let x = (k % 100) as f32 - 50.0;
        let z = (k / 100) as f32 - 50.0;
        w.add_static(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }

    // 1 千动态盒：从 12~40m 高空雨落。
    let n_dynamic = 1_000usize;
    for k in 0..n_dynamic {
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

    println!(
        "RUST WL / vxl_phys M0 基准：{n_static} 静态 + {n_dynamic} 动态，60Hz 固定步，240 tick"
    );

    // 预热 10 tick。
    for _ in 0..10 {
        w.step();
    }

    let ticks = 240u32;
    let mut total = 0.0f64;
    let mut worst = 0.0f64;
    let mut best = f64::MAX;
    let mut worst_tick = 0u32;
    for t in 1..=ticks {
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        total += ms;
        if ms > worst {
            worst = ms;
            worst_tick = t;
        }
        best = best.min(ms);
        if t % 60 == 0 {
            let h = w.health();
            println!(
                "tick {t:3}: avg {:.2} ms | contacts {} | awake {} | NaN {} | deep {}",
                total / t as f64,
                h.contacts,
                h.awake_bodies,
                h.nan_bodies,
                h.deep_penetrations
            );
        }
    }
    let avg = total / ticks as f64;
    println!("----");
    println!(
        "平均 tick：{avg:.3} ms（目标 < 16.67 ms → {} Hz）",
        1000.0 / avg
    );
    println!("最好 {best:.3} ms / 最差 {worst:.3} ms（tick {worst_tick}）");
    let h = w.health();
    println!("健康报告：{h:?}");
    if !h.is_clean() {
        println!("❌ 稳定性检查未通过");
        std::process::exit(1);
    }
    if avg < 16.67 {
        println!("✅ M0 性能门槛（1万静态+1千动态 ≥ 60Hz）通过");
    } else {
        println!("⚠️ 未达 60 Hz（M0 出口门槛），需优化宽相/窄相");
    }
}
