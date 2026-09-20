//! 临时诊断（验收后删除）：仿 M0 基准场景的 BVH 行为（树高/耗时）。

use std::time::Instant;

use vxl_phys_broad::{BroadPhase, BvhBroadPhase};
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Shape, Vec3};

#[test]
fn diag_bvh_bench_shape() {
    let mut b = BodySet::new();
    for k in 0..10_000usize {
        let x = (k % 100) as f32 - 50.0;
        let z = (k / 100) as f32 - 50.0;
        b.push_static(
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
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    let mut bp = BvhBroadPhase::new(0.01);
    let t0 = Instant::now();
    let p = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
    println!(
        "首帧: {:?} pairs={} 树高={}",
        t0.elapsed(),
        p.len(),
        bp.tree_height()
    );
    for f in 0..3 {
        let t0 = Instant::now();
        let p = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
        println!(
            "静置帧{f}: {:?} pairs={} 树高={}",
            t0.elapsed(),
            p.len(),
            bp.tree_height()
        );
    }
    for f in 0..5 {
        for i in 0..b.len() {
            if b.is_dynamic(i) {
                b.position[i].y -= 0.16;
            }
        }
        let t0 = Instant::now();
        let p = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
        println!("下落帧{f}: {:?} pairs={}", t0.elapsed(), p.len());
    }
}
