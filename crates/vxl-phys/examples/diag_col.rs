//! 单列逐 tick 追踪（M1 T1 诊断，临时工具不随封档保留）：最小微抖复现
//! （2 列横距 1.0、5 层、串行、偏置通道可关）——逐 tick 打印目标列的
//! 位置/速度/角速度/四元数分量与参与流形的点数/深度，用于定位摇摆在
//! 速度通道中的能量来源。
//!
//! 用法：cargo run --release -p vxl-phys --example diag_col -- [mu=0.5] [baum=0.2] [ticks=240]

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let mu: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.5);
    let baum: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.2);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(240);

    let mut cfg = PhysConfig {
        velocity_iterations: 16,
        threads: 1,
        baumgarte: baum,
        ..PhysConfig::default()
    };
    cfg.friction = vxl_phys::core::FrictionModel::Coulomb { mu };

    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0));
    // 2 列（横距 1.0）——与 diag_min 的 side=2 xspacing=1.0 完全同构。
    let xspacing = 1.0f32;
    let spacing = 0.52f32;
    for layer in 0..5usize {
        let y = 0.25 + layer as f32 * spacing;
        for row in 0..2usize {
            for col in 0..2usize {
                let off = 0.5 * xspacing;
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
    // 列体：列(0,0) = 1,5,9,13,17；列(0,1) = 2,6,10,14,18。
    let col_a = [1usize, 5, 9, 13, 17];
    let col_b = [2usize, 6, 10, 14, 18];

    // 逐体能量审计：ΔKE − 重力功 = 接触功（聚合在列上则可看柱体是否被接触泵能）。
    let ke_of = |w: &World, i: usize| -> f32 {
        let m = 1.0 / w.bodies.inv_mass[i];
        let v = w.bodies.linvel[i];
        let mut ke = 0.5 * m * v.length_squared();
        let wv = w.bodies.angvel(i);
        let inv_i = w.bodies.local_inv_inertia[i];
        if inv_i.x > 0.0 {
            let q = w.bodies.rot(i);
            let wl = q.conjugate().rotate_vec3(wv);
            let ll = wl.mul_per_elem(Vec3::new(1.0 / inv_i.x, 1.0 / inv_i.y, 1.0 / inv_i.z));
            let lw = q.rotate_vec3(ll);
            ke += 0.5 * wv.dot(lw);
        }
        ke
    };
    let all: Vec<usize> = col_a.iter().chain(col_b.iter()).copied().collect();
    let mut prev_ke: Vec<f32> = all.iter().map(|&i| ke_of(&w, i)).collect();
    let mut cum_work = vec![0.0f32; all.len()];
    let mut prev_vy: Vec<f32> = all.iter().map(|&i| w.bodies.linvel[i].y).collect();

    println!(
        "场景：2 列 × 5 层 | 横距 {xspacing} 竖距 {spacing} | μ {mu} | baumgarte {baum} | 串行"
    );
    for t in 1..=ticks {
        w.step();
        // 能量审计（步进前速度取上一 tick 末值近似中点）。
        let mut work_line = format!("t {t:3} work");
        for (k, &i) in all.iter().enumerate() {
            let ke_now = ke_of(&w, i);
            let vy_now = w.bodies.linvel[i].y;
            let vy_mid = 0.5 * (prev_vy[k] + vy_now);
            let m = 1.0 / w.bodies.inv_mass[i];
            let w_grav = m * (-9.81) * vy_mid * (1.0 / 60.0);
            let w_contact = ke_now - prev_ke[k] - w_grav;
            cum_work[k] += w_contact;
            prev_ke[k] = ke_now;
            prev_vy[k] = vy_now;
            let _ = cum_work;
            work_line.push_str(&format!(
                " #{i}{:+8.3}/{:+7.2}",
                w_contact * 1000.0,
                cum_work[k]
            ));
        }
        if t <= 60 || (200..=240).contains(&t) {
            println!("{work_line}");
        }
        let interesting = t <= 60 || (200..=240).contains(&t);
        if !interesting {
            continue;
        }
        for (tag, col) in [("A", &col_a), ("B", &col_b)] {
            let mut line = format!("t {t:3} [{tag}]");
            for &i in col.iter() {
                let p = w.bodies.position[i];
                let v = w.bodies.linvel[i];
                let om = w.bodies.angvel(i);
                let q = w.bodies.rot(i);
                line.push_str(&format!(
                    " #{i} y{:.4} |v|{:6.4} vx{:7.4} vy{:7.4} |w|{:6.4} qx{:8.5} qz{:8.5}",
                    p.y,
                    v.length(),
                    v.x,
                    v.y,
                    om.length(),
                    q.x,
                    q.z
                ));
            }
            println!("{line}");
        }
        // 顶盒（17）参与流形的几何明细。
        for m in w.manifolds() {
            if m.a as usize == 17 || m.b as usize == 17 {
                let ds: Vec<String> = m.points.iter().map(|p| format!("{:.5}", p.depth)).collect();
                let px: Vec<String> = m
                    .points
                    .iter()
                    .map(|p| format!("({:.4},{:.4})", p.point.x, p.point.z))
                    .collect();
                println!(
                    "        mf ({},{}) n=({:.3},{:.3},{:.3}) pts={} depth=[{}] xz=[{}]",
                    m.a,
                    m.b,
                    m.normal.x,
                    m.normal.y,
                    m.normal.z,
                    m.points.len(),
                    ds.join(" "),
                    px.join(" ")
                );
            }
        }
    }
}
