//! 宽节点 BVH（8 路）——T2 尾「数据布局」投入的承重件。
//!
//! **动机（访存账，见 docs/SESSION-2026-09-13.md）**：二叉 AoS 树在 8B 场景下
//! 节点 ≈40.5 万、18 层，单帧查询访存 ≈86MB ⇒ 远超 L3、访存受限（查询峰
//! 22.64ms / 树峰 7.34ms 皆源于此）。8 路节点 ⇒ 节点 ≈10.1 万、层 ≈6、访存
//! ≈29MB（**−67%**）。纯 SoA 拆分仅 −33%；失效范围 / fat 边距 / 树高调优
//! 均已被实测证伪（见会话记录）。
//!
//! **接入实测结论（T2 尾，已回退——先读这段再动手）**：8B 场景（20 万体）
//! 接入 `BvhBroadPhase` 后，**树峰 7.30 → 3.10ms（−58%，达验收）**、
//! **查询均 7.74 → 14.68ms（+90%）、峰 24.10 → 33.50ms** ⇒ **净负，已回退**。
//! 根因（已定量）：查询成本 ≈ **候选总数 × 4ns**；**叶含 8 体 ⇒ 候选粒度 8×**
//! （叶盒 = 体盒并集，比单体 fat 盒更松）——实测候选 35.4 万 → **207.8 万/tick**，
//! 与 +6.94ms 逐项吻合。**故「叶宽」是错误方向：宽节点要宽在内部节点，
//! 叶应保持 1-2 体**（下一版设计的依据，见 docs/M1-PLAN.md 证伪③）。
//!
//! **插入策略（B 树式溢出传播）**：新体先落叶；叶满（8 旧体 + 1 新体）⇒ 按最长轴
//! 切 4+5 成两叶，**多出的叶作为父的一条额外孩子**（父 `count`+1）而非替换原槽位；
//! 父满（8 子 + 1 新子）⇒ 按最长轴切 5+4 并向祖父传播；**仅根溢出**才新建根。
//!
//! 反例（已修）：原「原地替换分裂」每次分裂都让该路径永久加深一层 ⇒ 8000 体逐插
//! 实测 **20 层**（理想 ≈6，连二叉树 13 层都不如）；改溢出传播后高度回到 B 树界
//! `≤ ceil(log8(n/8)) + 1`，且**所有叶同深**（`wide_deep_insert_stays_balanced` 守门）。
//!
//! **本模块范围**：批量构建（`rebuild`：确定性 top-down 8 路均分）+ 增量插入/删除/
//! 移动 + 查询 + 不变量自检 + 暴力枚举对拍。**旋转平衡暂缺**：退化由宽相既有的
//! 「高度超阈值 ⇒ 批量重建」兜底（8B 场景该分支实测从未触发，tree_h=18 ⇒ 8 路旋转
//! 不是接入前置）。
//!
//! **体盒旁路**：8 路宽叶无法从叶盒反推单体盒（叶含 ≤8 体），故并行维护
//! `body_box: Vec<Aabb>`（`insert`/`move_proxy` 同步；`remove` 用它重算叶盒、
//! 分裂排序用它 ⇒ 不依赖调用方的体盒数组）。
//!
//! **不变量（`validate` + 测试守门）**：① 内部盒 = 子盒并集；② 叶盒 = 体内盒
//! 并集；③ 叶体数 1..=WIDE；④ 内部子数 2..=WIDE；⑤ 叶 ⊆ 全祖先盒（包含性 =
//! 查询完备性之根）；⑥ 高度 = 1+max(子高)（叶 0）；⑦ 父指针自洽。

use crate::Aabb;
#[cfg(test)]
use vxl_phys_core::Vec3;

// ── 按域拆出的子模块（子目录 wide/）
mod wide_bits;
mod wide_impl;
pub use self::{wide_bits::*, wide_impl::*};
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

    pub(crate) fn build_cap(items: &[(u32, Aabb)]) -> WideBvh {
        let mut t = WideBvh::new_with_capacity(items.len() + 4);
        for (body, b) in items {
            t.insert(*body, *b);
        }
        t.validate();
        t
    }

    /// 查询完备性 + 过取可解释（两棵树共用）。
    pub(crate) fn check_query(t: &WideBvh, items: &[(u32, Aabb)], q: &Aabb) {
        let mut got = Vec::new();
        t.query(q, &mut got);
        got.sort_unstable();
        got.dedup();
        for (id, b) in items.iter() {
            if b.overlaps(q) {
                assert!(got.contains(id), "漏体 {id}（完备性破坏）");
            }
        }
        for id in got.iter() {
            let leaf = t.leaf_of(*id as usize) as usize;
            assert!(
                t.nodes()[leaf].aabb.overlaps(q),
                "多出体 {id} 来自与 q 不相交的叶（过取不可解释）"
            );
        }
    }

    /// B 树平衡性（插入路径专有）：每个内部节点的子**等高** ⇒ 全叶同深。
    /// 这是「插入不退化」的结构性判据（高度界只是它的推论）。
    /// **仅对插入-only 树成立**（`remove` 的塌陷会留下浅叶）。
    pub(crate) fn assert_balanced(t: &WideBvh) {
        for (i, n) in t.nodes().iter().enumerate() {
            if n.height == 0 {
                continue;
            }
            for k in 0..n.count as usize {
                let c = &t.nodes()[n.slot[k] as usize];
                assert_eq!(
                    c.height,
                    n.height - 1,
                    "节点 {i} 子高不齐（B 树平衡破坏：插入退化回归）"
                );
            }
        }
    }

    /// 增量插入 1..64 逐个：不变量 + 高度不超理想 +2 + 每一步都平衡。
    #[test]
    pub(crate) fn wide_incremental_insert_keeps_invariants() {
        let mut t = WideBvh::new_with_capacity(128);
        for (k, (body, b)) in layout(64).iter().enumerate() {
            t.insert(*body, *b);
            t.validate();
            assert_balanced(&t);
            let n = (k + 1) as f32;
            let ideal = if n <= WIDE as f32 {
                0.0
            } else {
                (n / WIDE as f32).log(2.0).ceil()
            };
            assert!(
                (t.height() as f32) <= ideal + 2.0,
                "插入 {} 个后高度 {} 偏深（理想 ≈{ideal}）",
                k + 1,
                t.height()
            );
        }
    }

    /// 增量查询对暴力枚举：完备性 + 过取来自相交叶。
    #[test]
    pub(crate) fn wide_incremental_query_matches_brute_force() {
        let items = layout(1500);
        let t = build_cap(&items);
        for q in items.iter().step_by(37).map(|(_, b)| *b) {
            check_query(&t, &items, &q);
        }
    }

    /// 删除：摘一半 → 不变量 + 查询收缩；全摘 → 空树。
    #[test]
    pub(crate) fn wide_remove_keeps_invariants() {
        let items = layout(300);
        let mut t = build_cap(&items);
        let probe = aabb(0.0, 0.0, 50.0);
        for (body, _) in items.iter().filter(|(id, _)| id % 2 == 0) {
            t.remove(*body);
            t.validate();
        }
        let mut got = Vec::new();
        t.query(&probe, &mut got);
        got.sort_unstable();
        got.dedup();
        for id in &got {
            assert_eq!(id % 2, 1, "已删除体 {id} 仍被查出");
        }
        for (id, b) in items.iter().filter(|(id, _)| id % 2 == 1) {
            if b.overlaps(&probe) {
                assert!(got.contains(id), "未删除体 {id} 被漏");
            }
        }
        for (body, _) in items.iter().filter(|(id, _)| id % 2 == 1) {
            t.remove(*body);
            t.validate();
        }
        let mut empty = Vec::new();
        t.query(&probe, &mut empty);
        assert!(empty.is_empty(), "全删后仍查出 {} 个", empty.len());
    }

    /// 删后再插：溢出传播不得被「塌陷留下的浅叶」带偏（不变量守住）。
    #[test]
    pub(crate) fn wide_remove_then_reinsert_keeps_invariants() {
        let items = layout(600);
        let mut t = build_cap(&items);
        for (id, _) in items.iter().take(300) {
            t.remove(*id);
        }
        t.validate();
        for (id, b) in items.iter().take(300) {
            t.insert(*id, *b);
            t.validate();
        }
        check_query(&t, &items, &aabb(0.0, 0.0, 30.0));
        check_query(&t, &items, &aabb(5.0, -3.0, 2.0));
    }

    /// 移动：容差内 changed=false；远移后新位置可查。
    #[test]
    pub(crate) fn wide_move_proxy_growth_only() {
        let items = layout(40);
        let mut t = build_cap(&items);
        let leaf = t.leaf_of(0);
        // 叶盒 = 精确体盒（本模块不预烘 fat 边距）⇒ 带边距的首次调用必然扩张。
        let (leaf, changed) = t.move_proxy(leaf, items[0].1, 0.02);
        assert!(changed, "带边距的首次移动应扩张叶盒");
        // 同参数第二次：叶盒已含 target ⇒ 零结构操作。
        let (_leaf2, again) = t.move_proxy(leaf, items[0].1, 0.02);
        assert!(!again, "叶盒已含 target 时应免结构操作");
        let far = aabb(100.0, 100.0, 5.0);
        t.set_body_box(0, far);
        let (_leaf_far, changed2) = t.move_proxy(leaf, far, 0.02);
        assert!(changed2, "远移必须报告变化");
        t.validate();
        let mut got = Vec::new();
        t.query(&aabb(100.0, 100.0, 6.0), &mut got);
        assert!(got.contains(&0), "远移后的体应可被新位置查到");
    }

    /// **接入 `BvhBroadPhase` 的前置判据**：增量高度必须落在 B 树界内。
    /// 8000 体 ⇒ 下界 4（≥1000 叶，8 路 ⇒ 至少 `ceil(log8 1000)` 层）、
    /// 上界 6。修复前同一场景为 **20 层**（原地替换分裂每次加深路径）。
    #[test]
    pub(crate) fn wide_incremental_height_matches_ideal() {
        let items = layout(8000);
        let t = build_cap(&items);
        assert_balanced(&t);
        let h = t.height();
        assert!(h <= 6, "增量高度 {h} 超出 B 树界（应 ≤6；修复前为 20）");
        assert!(h >= 4, "增量高度 {h} 过浅——8000 体 8 路树不可能低于 4");
    }

    /// 增量高度 ≤ 批量高度 + 1（接入前置：否则高度阈值 `3·log2(n)+16` 会
    /// 频繁触发全量重建而抖动）+ 两棵树查询完备性一致。
    #[test]
    pub(crate) fn wide_incremental_height_within_one_of_bulk() {
        let items = layout(8000);
        let inc = build_cap(&items);
        let mut bulk = WideBvh::new_with_capacity(items.len() + 4);
        let leaves = bulk.rebuild(&items);
        assert_eq!(leaves.len(), items.len());
        for (id, _) in items.iter() {
            let leaf = leaves[*id as usize];
            assert_ne!(leaf, NULL, "体 {id} 无叶");
        }
        assert!(
            inc.height() <= bulk.height() + 1,
            "增量 {} vs 批量 {}",
            inc.height(),
            bulk.height()
        );
        for q in [
            aabb(0.0, 0.0, 12.0),
            aabb(8.0, 8.0, 1.5),
            aabb(-15.0, 3.0, 4.0),
        ] {
            check_query(&inc, &items, &q);
            check_query(&bulk, &items, &q);
        }
        // 批构建的遍历质量不得比增量差（同规模同查询）。
        let probe = aabb(0.0, 0.0, 12.0);
        let (in_nodes, _) = inc.traversal_stats(&probe);
        let (bu_nodes, _) = bulk.traversal_stats(&probe);
        assert!(
            bu_nodes <= in_nodes * 2 + 64,
            "批构建遍历数 {bu_nodes} 远差于增量 {in_nodes}"
        );
    }

    /// 按体 id 移动（宽相接入 API）：分裂会把体搬到新叶 ⇒ 调用方缓存的旧叶
    /// 下标失效，故必须由树内 `leaf_of` 权威解析；未入树的体按新体插入。
    #[test]
    pub(crate) fn wide_move_proxy_body_resolves_split_relocation() {
        // 同一小区域塞 16 体 ⇒ 必然发生叶分裂（体被搬离原叶）。
        let mut t = WideBvh::new_with_capacity(64);
        for k in 0..16u32 {
            t.insert(k, aabb(k as f32 * 0.1, 0.0, 0.4));
        }
        t.validate();
        assert_eq!(t.body_slots(), 64, "容量预置应计入体槽");
        // 取一个体远移：必须报告变化，且新位置可查。
        let far = aabb(50.0, 50.0, 1.0);
        let (_leaf, changed) = t.move_proxy_body(0, far, 0.05);
        assert!(changed, "远移必须报告变化");
        t.validate();
        let mut got = Vec::new();
        t.query(&aabb(50.0, 50.0, 2.0), &mut got);
        assert!(got.contains(&0), "远移后的体应可被新位置查到");
        // 同参数第二次：叶盒已含 target ⇒ 免结构操作（缓存可复用）。
        let (_l2, again) = t.move_proxy_body(0, far, 0.05);
        assert!(!again, "叶盒已含 target 时应免结构操作");
        // 未入树的体：按新体插入并报告变化（缓存失效语义）。
        let (leaf, fresh) = t.move_proxy_body(40, aabb(0.0, 0.0, 0.5), 0.05);
        assert!(fresh && leaf != NULL, "未入树的体应被插入");
        t.validate();
    }

    /// 叶溢出分裂：9 体 ⇒ 根宽 2、两叶 4+5、全部体仍可查（B 树传播最小例）。
    #[test]
    pub(crate) fn wide_leaf_split_keeps_all_bodies() {
        let items: Vec<(u32, Aabb)> = (0..9)
            .map(|k| (k, aabb(k as f32 * 0.5, 0.0, 0.4)))
            .collect();
        let t = build_cap(&items);
        assert_eq!(t.height(), 1, "9 体应恰好两层（叶 + 根）");
        let root = t.nodes()[t.root() as usize];
        assert_eq!(root.count, 2, "溢出应产生宽 2 的根");
        assert_balanced(&t);
        for (id, b) in items.iter() {
            check_query(&t, &items, b);
            let leaf = t.nodes()[t.leaf_of(*id as usize) as usize];
            assert!(leaf.height == 0 && leaf.count >= 4, "分裂后各叶应 ≥4 体");
        }
    }

    /// 深插入（5000 体）保持 B 树平衡（全叶同深）——结构性反退化判据。
    #[test]
    pub(crate) fn wide_deep_insert_stays_balanced() {
        let t = build_cap(&layout(5000));
        assert_balanced(&t);
        assert!(t.height() <= 6, "5000 体高度 {} 偏深", t.height());
    }
}
