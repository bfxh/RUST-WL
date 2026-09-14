//! 临时对照跑轮（不随封档保留）：只用默认配置字段，编译期兼容新旧求解器，
//! 用于确认重堆失稳是否为存量问题。
use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let ticks: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let layers: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(25);
    let spacing: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.52);

    let cfg = PhysConfig {
        velocity_iterations: iters,
        threads,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let side = 20usize;
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
    println!("参数：iters {iters} 线程 {threads} 层 {layers} 间距 {spacing}");
    let boxes = w.bodies.len();
    for _ in 0..10 {
        w.step();
    }
    let mut worst = 0.0f32;
    let mut ignition: Option<(u32, f32)> = None;
    let t0 = Instant::now();
    for t in 1..=ticks {
        w.step();
        let mut vmax = 0.0f32;
        for i in 0..boxes {
            if w.bodies.is_dynamic(i) {
                vmax = vmax.max(w.bodies.linvel[i].length());
            }
        }
        worst = worst.max(vmax);
        if ignition.is_none() && vmax > 10.0 {
            ignition = Some((t, vmax));
        }
        if t % 50 == 0 {
            // 接触深度直方图（>0.02 / >0.1 / >0.25）+ 流形数：区分坏接触 vs 求解泵能。
            let (mut n2, mut n10, mut n25) = (0u32, 0u32, 0u32);
            for m in w.manifolds() {
                for p in &m.points {
                    if p.depth > 0.02 {
                        n2 += 1;
                    }
                    if p.depth > 0.1 {
                        n10 += 1;
                    }
                    if p.depth > 0.25 {
                        n25 += 1;
                    }
                }
            }
            let h = w.health();
            println!(
                "tick {t:3}: |v|max {vmax:7.2} | awake {} | 流形 {} | 深接触 >0.02 {n2} >0.1 {n10} >0.25 {n25}",
                h.awake_bodies,
                w.manifolds().len()
            );
        }
    }
    println!(
        "== 跑完：{ticks} tick 用时 {:.1}s | 全窗 |v|max {worst:.2} m/s | 点火（>10 m/s）{:?}",
        t0.elapsed().as_secs_f32(),
        ignition
    );
}
