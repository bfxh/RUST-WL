//! T5 金样对拍（提前做）：同场景双引擎行为对照——首要问题：25 层留缝塔的
//! 坍塌是物理还是数值？（Rapier 0.35 默认 = TGS-Soft 软接触 4 迭代）。
//!
//! 用法：cargo run --release -- [scene] [ticks] [vxl_iters]
//!   scene: tower25 = 25 层 × 10×10 | pile5 = 5 层 × 20×20 | col45 = 5 层 × 3×3
//!   （均：盒 0.5、密度 1000、μ 0.5、e 0、缝 2cm、盒地板）
//! 输出：逐 50 tick 双引擎汇总（|v|max / KE / 入睡数 / 最深穿透 / 高度带）
//!   + 参照体检查点位姿表（容差表雏形：末态 max |Δpos|）。

use rapier3d::prelude::*;
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

struct Scene {
    layers: usize,
    side: usize,
    spacing: f32,
}

fn scene_of(name: &str) -> Scene {
    match name {
        "pile5" => Scene {
            layers: 5,
            side: 20,
            spacing: 0.52,
        },
        "col45" => Scene {
            layers: 5,
            side: 3,
            spacing: 0.52,
        },
        _ => Scene {
            layers: 25,
            side: 10,
            spacing: 0.52,
        },
    }
}

fn spawn_positions(s: &Scene) -> Vec<Vec3> {
    let off = (s.side as f32 - 1.0) * 0.5 * s.spacing;
    let mut out = Vec::new();
    for layer in 0..s.layers {
        let y = 0.25 + layer as f32 * s.spacing;
        for row in 0..s.side {
            for col in 0..s.side {
                out.push(Vec3::new(
                    col as f32 * s.spacing - off,
                    y,
                    row as f32 * s.spacing - off,
                ));
            }
        }
    }
    out
}

fn build_vxl(
    s: &Scene,
    iters: u32,
    skin: f32,
    inner: u32,
    maxcorr: f32,
    freq: f32,
    substeps: u32,
) -> (World, Vec<usize>) {
    let cfg = PhysConfig {
        velocity_iterations: iters,
        threads: 8,
        contact_skin: skin,
        normal_inner: inner.max(1),
        max_corrective_velocity: maxcorr,
        contact_freq_hz: freq,
        substeps: substeps.max(1),
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(20.0, 0.5, 20.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    for p in spawn_positions(s) {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.25),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
    }
    let ids: Vec<usize> = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .collect();
    (w, ids)
}

fn build_rapier(s: &Scene) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
    let mut w = PhysicsWorld::new();
    // 地板：与 vxl 同几何（半 20×0.5×20，顶面 y=0）。
    let ground = w.bodies.insert(
        RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, -0.5, 0.0))
            .build(),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0).build(),
        ground,
        &mut w.bodies,
    );
    let mut handles = Vec::new();
    for p in spawn_positions(s) {
        let body = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(p.x, p.y, p.z))
                .build(),
        );
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.25, 0.25, 0.25)
                .density(1000.0)
                .friction(0.5)
                .restitution(0.0)
                .build(),
            body,
            &mut w.bodies,
        );
        handles.push(body);
    }
    (w, handles)
}

struct Sum {
    vmax: f32,
    ke: f32,
    sleeping: usize,
    deep: f32,
    ymin: f32,
    ymax: f32,
}

fn summary_vxl(w: &World, ids: &[usize]) -> Sum {
    let (mut vmax, mut ke, mut sleeping) = (0.0f32, 0.0f32, 0usize);
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for &i in ids {
        let m = 1.0 / w.bodies.inv_mass[i];
        let v = w.bodies.linvel[i];
        vmax = vmax.max(v.length());
        ke += 0.5 * m * v.length_squared();
        if !w.bodies.awake[i] {
            sleeping += 1;
        }
        let y = w.bodies.position[i].y;
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    let mut deep = 0.0f32;
    for m in w.manifolds() {
        for p in &m.points {
            deep = deep.max(p.depth);
        }
    }
    Sum {
        vmax,
        ke,
        sleeping,
        deep,
        ymin,
        ymax,
    }
}

fn summary_rapier(w: &PhysicsWorld, hs: &[RigidBodyHandle]) -> Sum {
    let (mut vmax, mut ke, mut sleeping) = (0.0f32, 0.0f32, 0usize);
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for &h in hs {
        let b = w.bodies.get(h).expect("body");
        let v = b.linvel();
        let m = b.mass();
        vmax = vmax.max(v.length());
        ke += 0.5 * m * v.length_squared();
        if b.is_sleeping() {
            sleeping += 1;
        }
        let y = b.translation().y;
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    let mut deep = 0.0f32;
    for pair in w.narrow_phase.contact_pairs() {
        for man in &pair.manifolds {
            for p in &man.points {
                deep = deep.max(-p.dist);
            }
        }
    }
    Sum {
        vmax,
        ke,
        sleeping,
        deep,
        ymin,
        ymax,
    }
}

/// 参照体：按索引均匀取 16 个（跨层/跨位置采样）。
fn ref_indices(n: usize) -> Vec<usize> {
    let step = (n / 16).max(1);
    (0..n).step_by(step).take(16).collect()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let scene_name = args.next().unwrap_or_else(|| "tower25".into());
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let vxl_iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let skin: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.01);
    let inner: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(4);
    let maxcorr: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(3.0);
    let freq: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(30.0);
    let substeps: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    let s = scene_of(&scene_name);

    let (mut vw, vids) = build_vxl(&s, vxl_iters, skin, inner, maxcorr, freq, substeps);
    let (mut rw, rhs) = build_rapier(&s);
    let refs = ref_indices(vids.len());
    println!(
        "场景 {scene_name}：{} 层 × {}×{} = {} 盒 | vxl iters {vxl_iters}（inner 4）| rapier 默认（TGS-Soft 4 迭代 + 软接触）| {ticks} tick",
        s.layers,
        s.side,
        s.side,
        vids.len()
    );
    println!("tick | 引擎 | |v|max | KE(J) | 入睡 | 最深 | y 带");
    for t in 1..=ticks {
        vw.step();
        rw.step();
        if t % 50 == 0 || t == ticks {
            let a = summary_vxl(&vw, &vids);
            let b = summary_rapier(&rw, &rhs);
            println!(
                "{t:4} | vxl    | {:7.3} | {:9.1} | {:5} | {:5.3} | [{:.2},{:.2}]",
                a.vmax, a.ke, a.sleeping, a.deep, a.ymin, a.ymax
            );
            println!(
                "{t:4} | rapier | {:7.3} | {:9.1} | {:5} | {:5.3} | [{:.2},{:.2}]",
                b.vmax, b.ke, b.sleeping, b.deep, b.ymin, b.ymax
            );
        }
    }
    // 末态参照体位姿对照（发散是预期：双求解器数值路径不同；看量级）。
    let mut max_d = 0.0f32;
    println!("== 末态参照体位姿（vxl vs rapier）：");
    for &k in &refs {
        let i = vids[k];
        let vp = vw.bodies.position[i];
        let rp = rw.bodies.get(rhs[k]).expect("body").translation();
        let d = (vp - Vec3::new(rp.x, rp.y, rp.z)).length();
        max_d = max_d.max(d);
        println!(
            "  体{k:5} vxl({:6.3},{:6.3},{:6.3}) rapier({:6.3},{:6.3},{:6.3}) |Δ| {d:.4}",
            vp.x, vp.y, vp.z, rp.x, rp.y, rp.z
        );
    }
    println!("== 末态 max |Δpos|（参照体）= {max_d:.4} m");
    // 嗡振画像：超阈分解（线性/角速分别）+ 最活跃体明细（含层高）。
    let (mut only_lin, mut only_ang, mut both, mut clean) = (0, 0, 0, 0);
    let mut top: Vec<(usize, f32, f32, f32)> = Vec::new();
    for &i in &vids {
        let v = vw.bodies.linvel[i].length();
        let w = vw.bodies.angvel(i).length();
        let (l, a) = (v >= 0.04, w >= 0.05);
        match (l, a) {
            (true, true) => both += 1,
            (true, false) => only_lin += 1,
            (false, true) => only_ang += 1,
            (false, false) => clean += 1,
        }
        top.push((i, v, w, vw.bodies.position[i].y));
    }
    println!(
        "== 阈值分解：仅线性超阈 {only_lin} | 仅角速超阈 {only_ang} | 双超 {both} | 阈值下 {clean}（共 {}）",
        vids.len()
    );
    top.sort_by(|a, b| b.1.total_cmp(&a.1));
    println!("== |v| 前 8（体, |v|, |ω|, y）：");
    for &(i, v, w, y) in top.iter().take(8) {
        println!("   #{i} |v| {v:.3} |w| {w:.3} y {y:.3}");
    }
    top.sort_by(|a, b| b.2.total_cmp(&a.2));
    println!("== |ω| 前 8（体, |v|, |ω|, y）：");
    for &(i, v, w, y) in top.iter().take(8) {
        println!("   #{i} |v| {v:.3} |w| {w:.3} y {y:.3}");
    }
}
