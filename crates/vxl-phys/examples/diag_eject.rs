//! 泵能体追踪（M1 T1 诊断，临时工具）：45 盒最小沸腾复现上找「谁在被接触
//! 注入能量」——逐 tick 计算全体接触功（ΔKE − 重力功），打印瞬时正功前 3 体；
//! 当某体 |v| 首次越过阈值时，倾倒该体全部流形的几何明细（点数/深度/法线/
//! 接触点坐标）。用于定位角点接触的能量注入源。
//!
//! 用法：cargo run --release -p vxl-phys --example diag_eject -- \
//!       [side=3] [spacing=0.52] [iters=16] [mu=0.5] [ticks=300] [threads=1]

use vxl_phys::core::FrictionModel;
use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let side: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(3);
    let spacing: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.52);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let mu: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.5);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);

    let mut cfg = PhysConfig {
        velocity_iterations: iters,
        threads,
        ..PhysConfig::default()
    };
    cfg.friction = FrictionModel::Coulomb { mu };

    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0));
    let layers = 5usize;
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
    let dyn_ids: Vec<usize> = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .collect();
    let dt = w.config.dt;
    println!(
        "场景：{layers} 层 × {side}×{side} 间距 {spacing} | iters {iters} μ {mu} 线程 {threads} | 体数 {}",
        dyn_ids.len()
    );

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
    let mut prev_ke: Vec<f32> = dyn_ids.iter().map(|&i| ke_of(&w, i)).collect();
    let mut prev_vy: Vec<f32> = dyn_ids.iter().map(|&i| w.bodies.linvel[i].y).collect();
    let mut cum: Vec<f32> = vec![0.0; dyn_ids.len()];
    let mut total = 0.0f32;
    let mut dump_body: Option<(usize, usize)> = None; // (body, 起 dump 的 tick)

    for t in 1..=ticks {
        w.step();
        let mut vmax = 0.0f32;
        let mut vmax_i = 0usize;
        let mut works: Vec<(usize, f32)> = Vec::with_capacity(dyn_ids.len());
        for (k, &i) in dyn_ids.iter().enumerate() {
            let ke_now = ke_of(&w, i);
            let vy_now = w.bodies.linvel[i].y;
            let vy_mid = 0.5 * (prev_vy[k] + vy_now);
            let m = 1.0 / w.bodies.inv_mass[i];
            let w_grav = m * (-9.81) * vy_mid * dt;
            let wc = ke_now - prev_ke[k] - w_grav;
            cum[k] += wc;
            prev_ke[k] = ke_now;
            prev_vy[k] = vy_now;
            works.push((i, wc));
            let vv = w.bodies.linvel[i].length();
            if vv > vmax {
                vmax = vv;
                vmax_i = i;
            }
        }
        works.sort_by(|a, b| b.1.total_cmp(&a.1));
        let w_sum: f32 = works.iter().map(|w| w.1).sum();
        total += w_sum;
        let top: Vec<String> = works
            .iter()
            .take(3)
            .map(|&(i, x)| format!("#{i}{:+.0}mJ", x * 1000.0))
            .collect();
        println!(
            "t {t:3} |v|max {vmax:6.3} #{vmax_i} | Σ功 {w_sum:+8.1}J 累计 {total:+9.1}J | {top:?}"
        );
        // |v| 越过 1.0 起，dump 泵能首体的全部流形（含双方位置/朝向/速度，8 tick）。
        if vmax > 1.0 && dump_body.is_none() {
            dump_body = Some((works[0].0, t));
        }
        if let Some((bi, t0)) = dump_body {
            if t <= t0 + 7 {
                println!("    pump #{bi} 本 tick {:+.1}mJ", works[0].1 * 1000.0);
                for m in w.manifolds() {
                    let (a, b) = (m.a as usize, m.b as usize);
                    if a == bi || b == bi {
                        let ds: Vec<String> =
                            m.points.iter().map(|p| format!("{:.4}", p.depth)).collect();
                        println!(
                            "      mf ({a},{b}) n=({:.3},{:.3},{:.3}) pts={} d=[{}]",
                            m.normal.x,
                            m.normal.y,
                            m.normal.z,
                            m.points.len(),
                            ds.join(" ")
                        );
                        for &x in [a, b].iter() {
                            let p = w.bodies.position[x];
                            let q = w.bodies.rot(x);
                            let v = w.bodies.linvel[x];
                            let om = w.bodies.angvel(x);
                            println!(
                                "        #{x} pos ({:.4},{:.4},{:.4}) q({:.5},{:.5},{:.5}) v({:+.4},{:+.4},{:+.4}) |w|{:.4}",
                                p.x, p.y, p.z, q.x, q.y, q.z, v.x, v.y, v.z, om.length()
                            );
                        }
                    }
                }
            }
        }
    }
    // 末态：累计功 top 5。
    let mut idx: Vec<usize> = (0..dyn_ids.len()).collect();
    idx.sort_by(|&a, &b| cum[b].total_cmp(&cum[a]));
    println!("== 累计接触功 top5：");
    for &k in idx.iter().take(5) {
        let i = dyn_ids[k];
        println!(
            "  #{i} cum {:+.2}J |v| {:.3} pos ({:.3},{:.3},{:.3})",
            cum[k],
            w.bodies.linvel[i].length(),
            w.bodies.position[i].x,
            w.bodies.position[i].y,
            w.bodies.position[i].z
        );
    }
}
