//! M1 分相剖析：m0_bench 同场景，按阶段拆分每 tick 耗时。
//! 运行：cargo run --release -p vxl-phys --example m1_profile [threads]

use std::time::Instant;

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let threads: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let cfg = PhysConfig {
        threads,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    println!("threads = {threads}");
    w.add_heightfield(HeightField::flat(-60.0, -60.0, 121, 121, 1.0, 0.0));

    for k in 0..10_000usize {
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

    let mut wall = 0.0f64;
    let mut prev = PhaseSnapshot::default();
    let mut prev_wall = 0.0f64;
    for t in 1..=240u32 {
        let t0 = Instant::now();
        w.step();
        wall += t0.elapsed().as_secs_f64() * 1000.0;
        if t % 60 == 0 {
            let h = w.health();
            let cur = PhaseSnapshot::from_world(&w, wall);
            // 近 60 tick 的每 tick 平均值（上一窗口差值 / 60）。
            println!(
                "tick {t:3} contacts {:5} awake {:4} | 近60tick均值 ms/tick: total {:6.2} | broad {:5.2} narrow {:5.2} solve {:5.2} integr {:4.2} ccd {:5.2} fields {:4.2}",
                h.contacts,
                h.awake_bodies,
                (cur.total - prev_wall) / 60.0,
                (cur.broad - prev.broad) / 60.0,
                (cur.narrow - prev.narrow) / 60.0,
                (cur.solve - prev.solve) / 60.0,
                (cur.integ - prev.integ) / 60.0,
                (cur.ccd - prev.ccd) / 60.0,
                (cur.fields - prev.fields) / 60.0,
            );
            prev = cur;
            prev_wall = cur.total;
        }
    }
}

#[derive(Default, Clone, Copy)]
struct PhaseSnapshot {
    total: f64,
    broad: f64,
    narrow: f64,
    solve: f64,
    integ: f64,
    ccd: f64,
    fields: f64,
}

impl PhaseSnapshot {
    fn from_world(w: &World, wall_ms: f64) -> Self {
        let t = w.timings();
        let us = |v: u64| v as f64 / 1000.0;
        Self {
            total: wall_ms,
            broad: us(t.broadphase_us),
            narrow: us(t.narrowphase_us),
            solve: us(t.solve_us),
            integ: us(t.integrate_vel_us + t.integrate_pos_us),
            ccd: us(t.ccd_us),
            fields: us(t.fields_us),
        }
    }
}
