//! 最小解构跑轮（M1 T1 稳定性诊断，临时工具不随封档保留）：
//! 逐 tick 记录 动能/势能/总能量 的增量、分裂冲量通道的位移残差
//! （Δpos − v_final·dt = 偏置写回量，精确定量）、流形点数分布、最深穿透、
//! 清醒体数——用于定位留缝密堆「沸腾」的能量注入通道。
//!
//! 用法：cargo run --release -p vxl-phys --example diag_min -- \
//!       [layers=5] [side=1] [spacing=0.52] [iters=16] [mu=0.5] [ticks=300] [floor=hf|box|none] [threads=8] [xspacing=同 spacing] [baumgarte=0.2]

use std::time::Instant;

use vxl_phys::core::FrictionModel;
use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let layers: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(5);
    let side: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    let spacing: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.52);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let mu: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.5);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    let floor: String = args.next().unwrap_or_else(|| "hf".into());
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let xspacing: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(spacing);
    let baumgarte: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.2);

    let mut cfg = PhysConfig {
        velocity_iterations: iters,
        threads,
        baumgarte,
        ..PhysConfig::default()
    };
    cfg.friction = FrictionModel::Coulomb { mu };

    let mut w = World::new(cfg);
    match floor.as_str() {
        "box" => {
            w.add_static(
                Shape::Box {
                    half: Vec3::new(8.0, 0.5, 8.0),
                },
                Vec3::new(0.0, -0.5, 0.0),
                Quat::IDENTITY,
            );
        }
        "none" => {}
        _ => {
            // 地板放大到 ±20：密堆沸腾时盒会横向散开，小地板会让它们掉出边界
            // 自由落体，污染能量统计（KE 无界增长其实是坠入虚空）。
            w.add_heightfield(HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0));
        }
    }
    let off = (side as f32 - 1.0) * 0.5 * xspacing;
    for layer in 0..layers {
        let y = 0.25 + layer as f32 * spacing;
        for row in 0..side {
            for col in 0..side {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * xspacing - off, y, row as f32 * xspacing - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }

    let dyn_ids: Vec<usize> = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .collect();
    let g = 9.81f32;
    let dt = w.config.dt;
    let mut prev_pos: Vec<Vec3> = dyn_ids.iter().map(|&i| w.bodies.position[i]).collect();
    let mut prev_e = f32::NAN;
    let mut prev_pe = f32::NAN;
    let mut max_de = f32::MIN;

    println!(
        "场景：{layers} 层 × {side}×{side} | 竖距 {spacing} 横距 {xspacing} | iters {iters} | μ {mu} | 地板 {floor} | 线程 {threads} | 体数 {}",
        dyn_ids.len()
    );
    let t0 = Instant::now();
    for t in 1..=ticks {
        w.step();
        let (mut ke, mut pe) = (0.0f32, 0.0f32);
        let (mut vmax, mut wmax, mut vmax_i) = (0.0f32, 0.0f32, 0usize);
        let mut awake = 0usize;
        let (mut resid_max, mut resid_big) = (0.0f32, 0usize);
        for (k, &i) in dyn_ids.iter().enumerate() {
            let v = w.bodies.linvel[i];
            let wv = w.bodies.angvel(i);
            let m = 1.0 / w.bodies.inv_mass[i];
            let vv = v.length();
            if vv > vmax {
                vmax = vv;
                vmax_i = i;
            }
            let ww = wv.length();
            if ww > wmax {
                wmax = ww;
            }
            ke += 0.5 * m * vv * vv;
            let inv_i = w.bodies.local_inv_inertia[i];
            if inv_i.x > 0.0 {
                let q = w.bodies.rot(i);
                let wl = q.conjugate().rotate_vec3(wv);
                let ll = wl.mul_per_elem(Vec3::new(1.0 / inv_i.x, 1.0 / inv_i.y, 1.0 / inv_i.z));
                let lw = q.rotate_vec3(ll);
                ke += 0.5 * wv.dot(lw);
            }
            pe += m * g * w.bodies.position[i].y;
            if w.bodies.awake[i] {
                awake += 1;
            }
            let d = w.bodies.position[i] - prev_pos[k];
            prev_pos[k] = w.bodies.position[i];
            let rl = (d - v * dt).length();
            if rl > resid_max {
                resid_max = rl;
            }
            if rl > 5e-4 {
                resid_big += 1;
            }
        }
        let e = ke + pe;
        let dpe = if prev_pe.is_nan() { 0.0 } else { pe - prev_pe };
        prev_pe = pe;
        let de = if prev_e.is_nan() { 0.0 } else { e - prev_e };
        prev_e = e;
        if de > max_de {
            max_de = de;
        }
        let (mut m1, mut m2, mut m3, mut m4) = (0u32, 0u32, 0u32, 0u32);
        let mut max_depth = 0.0f32;
        for m in w.manifolds() {
            match m.points.len() {
                1 => m1 += 1,
                2 => m2 += 1,
                3 => m3 += 1,
                4 => m4 += 1,
                _ => {}
            }
            for p in &m.points {
                if p.depth > max_depth {
                    max_depth = p.depth;
                }
            }
        }
        if t <= 30 || t % 20 == 0 {
            let vb = &w.bodies;
            println!(
                "t {t:3} |v| {vmax:6.3} |w| {wmax:6.3} KE {ke:9.1} dPE {:9.1} dE {de:9.1} | 残差 {resid_max:.5}m ×{resid_big} | awake {awake:3} 流形 {} [{m1}/{m2}/{m3}/{m4}] 最深 {max_depth:.3} | vmax#{} y {:.3}",
                dpe,
                w.manifolds().len(),
                vmax_i,
                vb.position[vmax_i].y
            );
        }
    }
    // 末态：|v| 前 5 体明细。
    let mut order: Vec<usize> = dyn_ids.clone();
    order.sort_by(|&a, &b| {
        w.bodies.linvel[b]
            .length()
            .total_cmp(&w.bodies.linvel[a].length())
    });
    println!("== 末态 |v| 前 5：");
    for &i in order.iter().take(5) {
        println!(
            "  #{} pos {:.3},{:.3},{:.3} |v| {:.3} |w| {:.3} awake {}",
            i,
            w.bodies.position[i].x,
            w.bodies.position[i].y,
            w.bodies.position[i].z,
            w.bodies.linvel[i].length(),
            w.bodies.angvel(i).length(),
            w.bodies.awake[i]
        );
    }
    let awake = dyn_ids.iter().filter(|&&i| w.bodies.awake[i]).count();
    println!(
        "== 跑完：{ticks} tick 用时 {:.1}s | 清醒 {awake}/{} | 单 tick dE 峰值 {max_de:.1}",
        t0.elapsed().as_secs_f32(),
        dyn_ids.len()
    );
}
