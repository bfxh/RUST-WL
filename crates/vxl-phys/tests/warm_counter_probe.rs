//! 判别探针：引擎路径（`World::step`）上暖启动计数到底动不动？
//!
//! 背景：`crates/vxl-phys/tests/compound_warm.rs` 在复合体场景里读到三个分支全 0，
//! 曾归因为"只有串行构建器计数"；但 `build_constraint` 只有一个调用点（并行驱动内）
//! ⇒ 该归因被推翻。本文件用**单盒**（非复合体、非睡眠）作对照：若单盒也全 0，
//! 则缺口是**普遍**的（引擎路径无读数）；若单盒有读数，则问题**只在复合体**那条路。

use vxl_phys::{
    warm_match_stats_take, FrictionModel, Material, PhysConfig, Quat, Shape, Vec3, World,
};

fn run_case(compound: bool) -> (u64, u64, u64, usize, usize) {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    if compound {
        let cid = w.add_compound(vec![vxl_phys::CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        }]);
        w.spawn_compound_body(cid, Vec3::new(0.0, 0.45, 0.0), Quat::IDENTITY, 500.0);
    } else {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 0.45, 0.0),
            Quat::IDENTITY,
            500.0,
        );
    }
    let mg = w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: 0.7 },
        restitution: 0.05,
    });
    for b in 0..w.bodies.len() {
        w.bodies.set_material(b, mg);
    }
    let mut contact_ticks = 0;
    for _ in 0..80 {
        w.step();
        if !w.manifolds().is_empty() {
            contact_ticks += 1;
            if contact_ticks >= 3 {
                break;
            }
        }
    }
    let _ = warm_match_stats_take();
    for _ in 0..4 {
        w.step();
    }
    let mut awake = 0;
    for i in 0..w.bodies.len() {
        if w.bodies.awake[i] {
            awake += 1;
        }
    }
    let (e, f, u) = warm_match_stats_take();
    (e, f, u, w.manifolds().len(), awake)
}

#[test]
fn plain_box_warm_counters_probe() {
    let r = run_case(false);
    println!("单盒: 计数={:?} 流形={} 清醒={}", (r.0, r.1, r.2), r.3, r.4);
}

#[test]
fn compound_warm_counters_probe() {
    let r = run_case(true);
    println!("复合: 计数={:?} 流形={} 清醒={}", (r.0, r.1, r.2), r.3, r.4);
}
