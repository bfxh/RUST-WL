//! BVH8：**宽内部节点 + 窄叶**——T2 尾「数据布局」第二版原型（先量再改）。
//!
//! **为什么是这个形状（第一版实测结论）**：8 路**宽叶**（叶含 ≤8 体）实测
//! 树更新 −58% 但**查询 +90%**——叶粒度 8× 令候选总数 35.4 → 207.8 万/tick，
//! 而查询成本 ≈ **候选总数 × 4ns**（见 docs/M1-PLAN.md 证伪③）。结论：**宽要宽在
//! 内部节点**——保住 6 层遍历（二叉 18 层），同时把候选膨胀压到 ≤2×（叶容 2 体）。
//!
//! **本模块范围**：批量构建 + **增量**（插入 / 移动代理 / refit；M1 无删体故不含
//! 删除）+ 查询 + 自检。增量接法是 `wide.rs` 那套 **B 树式溢出传播**（新叶挂到
//! 最优父；父满 8 子则切 5+4 向祖父传，仅根溢出才新建根）⇒ 全叶同深。
//!
//! **决策实验（已跑，`diag_wide::diag_query_cost_across_tree_shapes`）**：同场景
//! 20 万体 / 2 万探针，生产同形相位（查询 + 候选精确过滤），三树形：
//!
//! | 探针边距 | 二叉 18 层 | 宽叶 6 层 | **BVH8 6 层窄叶** |
//! |---|---|---|---|
//! | 0.05 | 927ns（候选 3.1） | 773ns（23.8） | **619ns（6.4）** |
//! | 0.25 | 999ns（5.3） | 966ns（32.4） | **772ns（10.7）** |
//! | 0.50 | 1500ns（8.6） | 1008ns（39.5） | **830ns（15.3）** |
//!
//! 三树形**配对数逐项相同**（1.62/查询 ⇒ 完备性交叉验证过）。结论：窄叶宽内部
//! 在三个边距档全胜（−33%..−45%）⇒ 值得投入增量侧（本模块即该投入）。
//!
//! **生产接入实测（2026-09-13，8B 20 万体 ×120 tick）——结论是「孤立对拍会骗人」**：
//!
//! | 段 | 二叉（基线） | 本模块 叶容2 | 本模块 叶容1 |
//! |---|---|---|---|
//! | 树 均/峰 | 2.08 / 7.30 | 2.66 / 6.17 | 3.29 / 7.92 |
//! | 查询 均/峰 | **7.74** / 24.10 | 9.41 / 22.74 | 7.96 / **21.19** |
//! | ├ 候选 均 | **35.4 万** | 85.8 万 | 42.8 万 |
//! | broad 合计 均 | **10.5ms** | 12.3ms | 11.9ms |
//!
//! **机制（已定量，两轮证伪的收敛）**：生产查询相位 = 每清醒体「缓存命中则只用
//! 旧候选表」——绝大多数体**不遍历**，只做候选精确过滤（生产实测 ≈**32ns/条**，
//! 远高于隔离对拍里的 ≈4ns，因为生产里 `aabbs` 是 4.8MB 冷数组 + 竞技场写读）。
//! 所以 **6 层遍历省下的时间抵不过 1.2-2.4× 候选的过滤开销**；而窄叶又把树侧
//! refit 从二叉的 ~2-3 层抬到 6 层（叶盒小 ⇒ 逃逸后逐层重算并集）。
//! ⇒ **宽节点方向（宽叶/宽内部两版）在生产均为净负，已回退；瓶颈是候选量，
//! 不是遍历层数**（下一杠杆应打在「叶盒紧致度 / 候选过滤的访存局部性」上）。
//!
//! **结构**：两种节点分数组存（内部 236B、叶 40B）——若两者同构，叶会白白
//! 带上 192B 的 8 盒数组（20 万体 ⇒ 多 38MB）。id 高位 = 叶标志。
//!
//! **不变量（`validate` 守门）**：① 内部每个槽盒 == 该子的并集盒；② 子 ⊆ 父；
//! ③ 叶体数 1..=LEAF_CAP；④ 内部子数 2..=BRANCH；⑤ 高 = 1+max(子高)（叶 0）；
//! ⑥ 父指针自洽、根父为空；⑦ 叶深差 ≤1（按计数细分的固有性质——n 非 8 的幂
//! 时无法整除；严格等高只有增量侧的 B 树式分裂能做到）。

use crate::Aabb;
#[cfg(test)]
use vxl_phys_core::Vec3;

// ── 按域拆出的子模块（子目录 bvh8/）
mod bvh8_bits;
mod bvh8_impl;
pub use self::{bvh8_bits::*, bvh8_impl::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn aabb(x: f32, y: f32, half: f32) -> Aabb {
        let c = Vec3::new(x, y, 0.0);
        Aabb {
            min: c - Vec3::splat(half),
            max: c + Vec3::splat(half),
        }
    }

    pub(crate) fn layout(n: u32) -> Vec<(u32, Aabb)> {
        (0..n)
            .map(|k| {
                let x = ((k.wrapping_mul(2_654_435_761)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let y = ((k.wrapping_mul(40_503)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let half = 0.3 + ((k.wrapping_mul(97)) % 100) as f32 / 1000.0;
                (k, aabb(x, y, half))
            })
            .collect()
    }

    pub(crate) fn build(items: &[(u32, Aabb)]) -> Bvh8 {
        let mut t = Bvh8::new();
        t.rebuild(items);
        t
    }

    /// 叶深差 ≤1。**注**：按计数细分 + 叶容 2 时 n 非 8 的幂**无法**整除，
    /// 故批构建天然允许跨 1 层（如 17 体 ⇒ 7 个 2 体叶 + 1 个含 3 体的子树）。
    /// 严格等高只有**增量路径**能做到（B 树式分裂，见 `wide.rs`）；本原型只做批量，
    /// 故按真实性质断言（差 >1 才算退化）。
    pub(crate) fn assert_depth_span_le_one(t: &Bvh8) {
        for n8 in t.nodes.iter() {
            for k in 0..n8.count as usize {
                let ch = t.height_of(n8.child[k]);
                assert!(
                    ch + 1 == n8.height || ch + 2 == n8.height,
                    "子高差 >1（深度跨度失控）：子 {ch} vs 父 {}",
                    n8.height
                );
            }
        }
    }

    /// 规模覆盖：1..300 与稀疏/密集两档，全部 不变量 + 平衡 + 高度界。
    #[test]
    pub(crate) fn bvh8_build_keeps_invariants() {
        for n in [1u32, 2, 3, 5, 8, 9, 17, 64, 129, 300] {
            let items = layout(n);
            let t = build(&items);
            // 叶数界：每叶 1..=LEAF_CAP 体 ⇒ [ceil(n/LEAF_CAP), n]。
            // 上界取到 n 是因为奇数段会 2+1 拆（如 3 → 叶 2 + 叶 1），批构建不追求装满。
            let lc = t.leaf_count();
            assert!(
                lc >= (n as usize).div_ceil(LEAF_CAP) && lc <= n as usize,
                "n={n} 叶数 {lc} 越界"
            );
            assert_depth_span_le_one(&t);
            // 高度界：叶数 ≤ BRANCH^h · LEAF_CAP
            let leaves = t.leaf_count() as f32;
            let ideal = (leaves / LEAF_CAP as f32).log(BRANCH as f32).ceil();
            assert!(
                (t.height() as f32) <= ideal + 1.0,
                "n={n} 高度 {} 偏深（理想 {ideal}）",
                t.height()
            );
        }
    }

    /// 查询对暴力枚举：完备性 + 过取可解释（两档规模）。
    #[test]
    pub(crate) fn bvh8_query_matches_brute_force() {
        for n in [64u32, 1500] {
            let items = layout(n);
            let t = build(&items);
            for q in items.iter().step_by(31).map(|(_, b)| *b) {
                let mut got = Vec::new();
                t.query(&q, &mut got);
                got.sort_unstable();
                got.dedup();
                let mut want: Vec<u32> = items
                    .iter()
                    .filter(|(_, b)| b.overlaps(&q))
                    .map(|(id, _)| *id)
                    .collect();
                want.sort_unstable();
                for id in &want {
                    assert!(got.contains(id), "n={n} 漏体 {id}（完备性破坏）");
                }
                for id in got.iter().filter(|id| !want.contains(id)) {
                    // 过取必须来自「与 q 相交的叶」（叶容 2 ⇒ 邻居体可能被带出）
                    let leaf_box = t
                        .leaves
                        .iter()
                        .find(|l| l.body[..l.count as usize].contains(id))
                        .expect("体必有叶")
                        .aabb;
                    assert!(
                        leaf_box.overlaps(&q),
                        "多出体 {id} 来自与 q 不相交的叶（过取不可解释）"
                    );
                }
            }
        }
    }

    /// 空树 / 单体的边界。
    #[test]
    pub(crate) fn bvh8_edge_cases() {
        let mut t = Bvh8::new();
        t.rebuild(&[]);
        assert_eq!(t.root(), NULL);
        let mut out = Vec::new();
        t.query(&aabb(0.0, 0.0, 1.0), &mut out);
        assert!(out.is_empty());

        let one = vec![(7u32, aabb(1.0, 2.0, 0.5))];
        let t = build(&one);
        assert_eq!(t.height(), 0, "单体树 = 单叶");
        let mut out = Vec::new();
        t.query(&aabb(1.0, 2.0, 0.6), &mut out);
        assert_eq!(out, vec![7], "根叶必须自测盒（无父替它验）");
        let mut miss = Vec::new();
        t.query(&aabb(50.0, 50.0, 0.5), &mut miss);
        assert!(miss.is_empty(), "根叶也要按盒剪枝");
    }

    /// 构建确定性：同输入两次 ⇒ 结构逐位相同（§5）。
    #[test]
    pub(crate) fn bvh8_build_is_deterministic() {
        let items = layout(500);
        let (a, b) = (build(&items), build(&items));
        assert_eq!(a.node_count(), b.node_count());
        assert_eq!(a.leaf_count(), b.leaf_count());
        assert_eq!(a.height(), b.height());
        for (x, y) in a.leaves.iter().zip(b.leaves.iter()) {
            assert_eq!(x.body, y.body);
            assert_eq!(x.aabb.min.x, y.aabb.min.x);
        }
    }

    pub(crate) fn build_inc(items: &[(u32, Aabb)]) -> Bvh8 {
        let mut t = Bvh8::new();
        for (b, a) in items {
            t.insert(*b, *a);
            t.validate();
        }
        t
    }

    /// 增量插入：不变量 + **严格全叶同深**（B 树式分裂的固有性质，区别于批构建）。
    #[test]
    pub(crate) fn bvh8_incremental_insert_keeps_invariants() {
        for n in [1u32, 2, 3, 9, 64, 300] {
            let items = layout(n);
            let t = build_inc(&items);
            for n8 in t.nodes.iter() {
                for k in 0..n8.count as usize {
                    assert_eq!(
                        t.height_of(n8.child[k]),
                        n8.height - 1,
                        "n={n} 增量树子高不齐（B 树平衡破坏）"
                    );
                }
            }
            let leaves = t.leaf_count() as f32;
            let ideal = (leaves / LEAF_CAP as f32).log(BRANCH as f32).ceil();
            assert!(
                (t.height() as f32) <= ideal + 1.0,
                "n={n} 增量高度 {} 偏深（理想 {ideal}）",
                t.height()
            );
        }
    }

    /// 增量树查询对暴力枚举（完备性 + 过取可解释）。
    #[test]
    pub(crate) fn bvh8_incremental_query_matches_brute_force() {
        let items = layout(1500);
        let t = build_inc(&items);
        for q in items.iter().step_by(37).map(|(_, b)| *b) {
            let mut got = Vec::new();
            t.query(&q, &mut got);
            got.sort_unstable();
            got.dedup();
            for (id, b) in items.iter() {
                if b.overlaps(&q) {
                    assert!(got.contains(id), "漏体 {id}（完备性破坏）");
                }
            }
            for id in got.iter() {
                let leaf = leaf_index(t.leaf_of(*id as usize));
                assert!(
                    t.leaves[leaf].aabb.overlaps(&q),
                    "多出体 {id} 来自与 q 不相交的叶"
                );
            }
        }
    }

    /// 移动代理：容差内 zero-op；远移后新位置可查且不变量守住。
    #[test]
    pub(crate) fn bvh8_move_proxy_growth_only() {
        let items = layout(200);
        let mut t = build_inc(&items);
        let (_leaf, changed) = t.move_proxy_body(0, items[0].1, 0.02);
        assert!(changed, "带边距的首次移动应扩张叶盒");
        let (_l2, again) = t.move_proxy_body(0, items[0].1, 0.02);
        assert!(!again, "叶盒已含 target 时应免结构操作");
        let far = aabb(100.0, 100.0, 5.0);
        let (_l3, moved) = t.move_proxy_body(0, far, 0.02);
        assert!(moved, "远移必须报告变化");
        t.validate();
        let mut got = Vec::new();
        t.query(&aabb(100.0, 100.0, 6.0), &mut got);
        assert!(got.contains(&0), "远移后应可被新位置查到");
    }

    /// 批构建与增量两条路径的查询结果一致（同集合 ⇒ 接入可替换）。
    #[test]
    pub(crate) fn bvh8_bulk_and_incremental_agree() {
        let items = layout(600);
        let bulk = build(&items);
        let inc = build_inc(&items);
        for q in items.iter().step_by(53).map(|(_, b)| *b) {
            let mut a = Vec::new();
            let mut b = Vec::new();
            bulk.query(&q, &mut a);
            inc.query(&q, &mut b);
            a.sort_unstable();
            a.dedup();
            b.sort_unstable();
            b.dedup();
            // 两条路径的叶分组可能不同 ⇒ 过取集合可不同，但**真重叠体**必须都在。
            for (id, bx) in items.iter() {
                if bx.overlaps(&q) {
                    assert!(a.contains(id) && b.contains(id), "漏体 {id}");
                }
            }
        }
    }
}
