//! 临时诊断（接入验收后删除）：宽节点 BVH8 的规模 / 高度 / 节点内存账。
//!
//! 8B 场景规模 = 10 万静态 + 10 万动态 ⇒ 20 万体；二叉 AoS 树该规模
//! ≈40.5 万节点 × 48B ≈ 19.4MB、18 层、查询访存 ≈86MB/帧（见
//! docs/SESSION-2026-09-13.md）。此处量宽节点树的同口径数字。

use std::mem::size_of;
use std::time::Instant;

use vxl_phys_broad::wide::{WideBvh, WideNode};
use vxl_phys_broad::Aabb;
use vxl_phys_core::Vec3;

fn items(n: u32) -> Vec<(u32, Aabb)> {
    (0..n)
        .map(|k| {
            let x = ((k.wrapping_mul(2_654_435_761)) % 4000) as f32 / 4000.0 * 200.0 - 100.0;
            let y = ((k.wrapping_mul(40_503)) % 4000) as f32 / 4000.0 * 200.0 - 100.0;
            let z = ((k.wrapping_mul(97)) % 4000) as f32 / 4000.0 * 20.0;
            (
                k,
                Aabb {
                    min: Vec3::new(x - 0.5, y - 0.5, z - 0.5),
                    max: Vec3::new(x + 0.5, y + 0.5, z + 0.5),
                },
            )
        })
        .collect()
}

#[test]
fn diag_wide_scale_accounting() {
    println!("WideNode = {} B × WIDE=8", size_of::<WideNode>());
    for n in [8_000u32, 200_000] {
        let src = items(n);
        // 批构建（宽相首帧 / 重建路径）
        let mut bulk = WideBvh::new_with_capacity(src.len() + 4);
        let t0 = Instant::now();
        let _leaves = bulk.rebuild(&src);
        let d_bulk = t0.elapsed();
        let bulk_nodes = bulk.nodes().len();
        // 对照：二叉 AoS 树同规模全量重建（宽相现有路径）
        let mut bin = vxl_phys_broad::DynamicBvh::new(0.02);
        let t2 = Instant::now();
        let bin_leaves = bin.rebuild(&src);
        let d_bin = t2.elapsed();
        println!(
            "n={n}: 二叉 rebuild {:?} h={} 叶={}",
            d_bin,
            bin.root_height(),
            bin_leaves.len()
        );
        // 增量（宽相稳态路径：稳态只动清醒体，此处量最坏「全量逐个插入」）
        let mut inc = WideBvh::new_with_capacity(src.len() + 4);
        let t1 = Instant::now();
        for (b, a) in src.iter() {
            inc.insert(*b, *a);
        }
        let d_inc = t1.elapsed();
        println!(
            "n={n}: 宽批 h={} 节点={bulk_nodes} 叶={} {:?} | 宽增量 h={} 节点={} {:?}",
            bulk.height(),
            bulk.leaf_count(),
            d_bulk,
            inc.height(),
            inc.nodes().len(),
            d_inc
        );
        println!(
            "       节点内存: 批 {:.2}MB | 增量 {:.2}MB | 二叉同规模 ≈19.4MB",
            bulk_nodes as f64 * size_of::<WideNode>() as f64 / 1048576.0,
            inc.nodes().len() as f64 * size_of::<WideNode>() as f64 / 1048576.0,
        );
    }
}
