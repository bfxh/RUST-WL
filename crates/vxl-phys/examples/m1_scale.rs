//! M1 规模场景（§3 最低通过档）：10 万动态 + 10 万静态，CPU headless。
//! 出口门槛：≥ 30 FPS（每 tick < 33.3 ms）。
//! 运行：cargo run --release -p vxl-phys --example m1_scale -- [threads] [static] [dynamic] [ticks] [iters]
//! 默认：threads=8, 静态 102400（320×320 瓦片）, 动态 100000, 300 tick, 迭代 16。

use std::time::Instant;

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let n_static: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(102_400);
    let n_dynamic: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    let ticks: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);

    let cfg = PhysConfig {
        threads,
        velocity_iterations: iters,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    // 注：地面 = 静态瓦片（不含高度场——双层地面会让每个盒子多算一次
    // 无接触的高度场对，规模档里是 10 万次/帧的白算）。

    let side = (n_static as f64).sqrt() as usize;
    for k in 0..n_static {
        let x = (k % side) as f32 - side as f32 * 0.5;
        let z = (k / side) as f32 - side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    let d_side = (n_dynamic as f64).sqrt() as usize + 1;
    for k in 0..n_dynamic {
        let x = (k % d_side) as f32 * 1.0 - d_side as f32 * 0.5;
        let z = (k / d_side) as f32 * 1.0 - d_side as f32 * 0.5;
        let y = 3.0 + ((k * 29) % 71) as f32 / 71.0 * 8.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }

    println!("threads = {threads} | 静态 {n_static} + 动态 {n_dynamic} | {ticks} tick（预热 20）");
    for _ in 0..20 {
        w.step();
    }
    let mut total = 0.0f64;
    let mut worst = 0.0f64;
    for t in 1..=ticks {
        let t0 = Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        total += ms;
        worst = worst.max(ms);
        // 每 tick 一行 stderr（无缓冲，长跑可观测）。
        let tim = w.timings();
        let bd = w.broad.breakdown_us();
        let th = w.broad.tree_height();
        eprintln!(
            "tick {t:3}: {ms:7.2} ms | broad {:6.2} (AABB {:5.2} 树 {:6.2} 查询 {:6.2}) tree_h {th:3} | 窄相 {:6.2} | solve 岛 {:6.2} 解算 {:6.2} 休眠 {:6.2} ms",
            tim.broadphase_us as f64 / 1000.0,
            bd.0 as f64 / 1000.0,
            bd.1 as f64 / 1000.0,
            bd.2 as f64 / 1000.0,
            tim.narrowphase_us as f64 / 1000.0,
            w.solver.last_phase_us.0 as f64 / 1000.0,
            w.solver.last_phase_us.1 as f64 / 1000.0,
            w.solver.last_phase_us.2 as f64 / 1000.0,
        );
        if t % 100 == 0 {
            let h = w.health();
            println!(
                "tick {t:3}: 近100均值 {:6.2} ms | contacts {} awake {} NaN {} deep {}",
                total / t as f64,
                h.contacts,
                h.awake_bodies,
                h.nan_bodies,
                h.deep_penetrations,
            );
        }
    }
    let avg = total / ticks as f64;
    let h = w.health();
    println!(
        "平均 {avg:.2} ms/tick → {:.1} FPS | 最差 {worst:.2} ms | NaN {} deep {}",
        1000.0 / avg,
        h.nan_bodies,
        h.deep_penetrations
    );
    if avg < 33.33 {
        println!("✅ §3 最低通过档（10万+10万 ≥30FPS）");
    } else {
        println!("⚠️ 未达 30 FPS");
    }
}
