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

fn measure<F: FnMut(&Aabb, &mut Vec<u32>)>(name: &str, probes: &[Aabb], mut q: F) -> (f64, usize) {
    let mut buf: Vec<u32> = Vec::new();
    for p in probes {
        q(p, &mut buf);
        buf.clear();
    }
    let mut cand = 0usize;
    let t0 = Instant::now();
    for p in probes {
        q(p, &mut buf);
        cand += buf.len();
        buf.clear();
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    let per = probes.len() as f64;
    println!(
        "  {name:8} 总 {ms:7.2}ms | {:.0}ns/查询 | 候选 {:.2}/查询",
        ms * 1e6 / per,
        cand as f64 / per
    );
    (ms, cand)
}

fn visits<F: Fn(&Aabb) -> (u64, u64)>(name: &str, probes: &[Aabb], f: F) {
    let (mut nn, mut lf) = (0u64, 0u64);
    for p in probes {
        let (a, b) = f(p);
        nn += a;
        lf += b;
    }
    let per = probes.len() as f64;
    println!(
        "  {name:8} 访问 节点 {:.1}/查询 + 叶 {:.1}/查询",
        nn as f64 / per,
        lf as f64 / per
    );
}

/// **生产同形**：查询(fat) + 候选精确过滤（`aabbs[j]` 与 exact 相交）——宽相
/// 每帧对每个「重查体」做的完整工作。返回 (ms, 候选数, 配对数)。
fn measure_full<F: FnMut(&Aabb, &mut Vec<u32>)>(
    name: &str,
    src: &[(u32, Aabb)],
    probes: &[Aabb],
    mut q: F,
) -> (f64, usize, usize) {
    let mut buf: Vec<u32> = Vec::new();
    for p in probes {
        q(p, &mut buf);
        buf.clear();
    }
    let mut cand = 0usize;
    let mut pairs = 0usize;
    let t0 = Instant::now();
    for (k, p) in probes.iter().enumerate() {
        let exact = src[k * 10].1; // 探针源体（step_by(10) 与下方一致）
        q(p, &mut buf);
        cand += buf.len();
        for &j in buf.iter() {
            if src[j as usize].1.overlaps(&exact) {
                pairs += 1;
            }
        }
        buf.clear();
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    let per = probes.len() as f64;
    println!(
        "  {name:8} 全相位 {ms:7.2}ms | {:.0}ns/查询 | 候选 {:.2} | 配对 {:.2}",
        ms * 1e6 / per,
        cand as f64 / per,
        pairs as f64 / per
    );
    (ms, cand, pairs)
}

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

/// 三树形查询成本对拍（同场景同探针）：二叉 18 层 / 宽叶 6 层 / BVH8 6 层窄叶。
///
/// 这是数据布局方向的**决策实验**：宽叶实测查询 +90%（候选 8×），故第二版把
/// 「宽」移到内部节点、叶保持 2 体。此处量 每查询 ns 与 候选/查询——若 BVH8
/// 不比二叉便宜，整条宽节点方向作废。
#[test]
fn diag_query_cost_across_tree_shapes() {
    use vxl_phys_broad::{Bvh8, DynamicBvh, WideBvh};

    let n = 200_000u32;
    let src = items(n);
    // 探针 = 每 10 个取 1 的 fat 盒（≈ 8B 场景每帧重查体的量级）；边距见下方扫描。
    println!("三树形对拍：{n} 体 / 探针每 10 体取 1");

    let mut bin = DynamicBvh::new(0.02);
    let t0 = Instant::now();
    bin.rebuild(&src);
    let d_bin = t0.elapsed();
    let mut wid = WideBvh::new_with_capacity(src.len() + 4);
    let t0 = Instant::now();
    let _ = wid.rebuild(&src);
    let d_wid = t0.elapsed();
    let mut b8 = Bvh8::new();
    let t0 = Instant::now();
    b8.rebuild(&src);
    let d_b8 = t0.elapsed();

    println!(
        "建树：二叉 {d_bin:?} / 宽叶 {d_wid:?} / BVH8 {d_b8:?} | 层数 二叉 {} / 宽叶 {} / BVH8 {}",
        bin.root_height(),
        wid.height(),
        b8.height()
    );
    println!(
        "结构：二叉 节点 {}（≈19.4MB）/ 宽叶 节点 {}（{:.2}MB）/ BVH8 内部 {} + 叶 {}（{:.2}MB）",
        bin.node_count(),
        wid.nodes().len(),
        wid.nodes().len() as f64 * size_of::<WideNode>() as f64 / 1048576.0,
        b8.node_count(),
        b8.leaf_count(),
        b8.memory_bytes() as f64 / 1048576.0,
    );

    // 探针边距**必须扫**：生产用速度自适应 fat 边距（0.02..0.5），而大边距会
    // 放大「叶盒大」的劣势——只测小边距会得出与生产相反的结论（本实验第一版
    // 就踩了：0.05 边距下宽叶看着最快，生产里却慢 90%）。
    for m in [0.05f32, 0.25, 0.50] {
        let probes: Vec<Aabb> = src.iter().step_by(10).map(|(_, a)| a.grown(m)).collect();
        println!("—— 探针边距 {m} ——");
        measure("二叉", &probes, |p, o| bin.query(p, o));
        measure("宽叶", &probes, |p, o| wid.query(p, o));
        measure("BVH8", &probes, |p, o| b8.query(p, o));
        visits("宽叶", &probes, |p| wid.traversal_stats(p));
        visits("BVH8", &probes, |p| b8.traversal_stats(p));
        // 生产同形（查询 + 过滤）：三树必须配对数一致（完备性交叉验证）。
        let mut trio = [(0.0, 0usize, 0usize); 3];
        trio[0] = measure_full("二叉", &src, &probes, |p, o| bin.query(p, o));
        trio[1] = measure_full("宽叶", &src, &probes, |p, o| wid.query(p, o));
        trio[2] = measure_full("BVH8", &src, &probes, |p, o| b8.query(p, o));
        assert_eq!(trio[0].2, trio[1].2, "二叉/宽叶 配对数不一致（完备性）");
        assert_eq!(trio[0].2, trio[2].2, "二叉/BVH8 配对数不一致（完备性）");
    }
}
