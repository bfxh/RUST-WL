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

/// 内部节点最大子数。
pub const BRANCH: usize = 8;
/// 叶容量。**1**：候选粒度与二叉叶一一对应（生产实测：叶容 2 ⇒ 候选 35.4 → 85.8
/// 万/tick、2.4×，而生产每条候选 ≈32ns ⇒ 候选膨胀吃掉了 6 层遍历的收益；叶容 1
/// 则候选数与二叉同量级，只赚遍历层数）。代价：叶节点数 = 体数（20 万叶 ≈ 7MB）。
pub const LEAF_CAP: usize = 1;

const NULL: u32 = u32::MAX;
const LEAF_BIT: u32 = 1 << 31;

#[inline]
fn is_leaf(id: u32) -> bool {
    id & LEAF_BIT != 0
}

#[inline]
fn leaf_index(id: u32) -> usize {
    (id & !LEAF_BIT) as usize
}

/// 内部节点：`count` ∈ 2..=BRANCH 个子的盒 + id（盒与 id 分离存 ⇒ 遍历只读盒）。
#[derive(Clone, Copy, Debug)]
pub struct Node8 {
    /// 子盒（前 `count` 项有效）——**连续 192B = 3 条缓存行**，遍历友好。
    pub aabb: [Aabb; BRANCH],
    /// 子 id（叶 = `LEAF_BIT | idx`，内部 = 下标）。
    pub child: [u32; BRANCH],
    /// **本节点自身盒** = 子盒并集。必须存：① 遍历/自检要 O(1) 取它（否则每次
    /// 现算 8 次并集）；② refit 的早退判据是「本节点对外呈现（并集盒 + 高）变没变」
    /// ——不存它就只能比对「变化槽的盒」，那会在**新增子**时误判为未变（邻居槽
    /// 一动并集就变）而漏传祖先。
    pub box_union: Aabb,
    pub parent: u32,
    pub height: u32,
    pub count: u8,
}

/// 叶：≤ `LEAF_CAP` 个体（盒 = 体内盒并集）。
#[derive(Clone, Copy, Debug)]
pub struct Leaf8 {
    pub aabb: Aabb,
    pub body: [u32; LEAF_CAP],
    pub parent: u32,
    pub count: u8,
}

#[inline]
fn union(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

fn longest_axis(a: &Aabb) -> u32 {
    let d = a.max - a.min;
    if d.x >= d.y && d.x >= d.z {
        0
    } else if d.y >= d.z {
        1
    } else {
        2
    }
}

fn center_on(a: &Aabb, axis: u32) -> f32 {
    match axis {
        0 => (a.min.x + a.max.x) * 0.5,
        1 => (a.min.y + a.max.y) * 0.5,
        _ => (a.min.z + a.max.z) * 0.5,
    }
}

#[inline]
fn cmp_axis_center(a: &(u32, Aabb), b: &(u32, Aabb), axis: u32) -> std::cmp::Ordering {
    center_on(&a.1, axis)
        .total_cmp(&center_on(&b.1, axis))
        .then(a.0.cmp(&b.0))
}

#[inline]
fn same_box(a: &Aabb, b: &Aabb) -> bool {
    a.min.x == b.min.x
        && a.min.y == b.min.y
        && a.min.z == b.min.z
        && a.max.x == b.max.x
        && a.max.y == b.max.y
        && a.max.z == b.max.z
}

#[inline]
fn contains_box(outer: &Aabb, inner: &Aabb) -> bool {
    outer.min.x <= inner.min.x
        && outer.min.y <= inner.min.y
        && outer.min.z <= inner.min.z
        && outer.max.x >= inner.max.x
        && outer.max.y >= inner.max.y
        && outer.max.z >= inner.max.z
}

fn zero_box() -> Aabb {
    Aabb {
        min: vxl_phys_core::Vec3::ZERO,
        max: vxl_phys_core::Vec3::ZERO,
    }
}

#[inline]
fn perimeter(a: &Aabb) -> f32 {
    let d = a.max - a.min;
    2.0 * (d.x + d.y + d.z)
}

/// 把 `b` 并入 `a` 后的周长增量（最佳父/最佳子选择用）。
#[inline]
fn enlargement(a: &Aabb, b: &Aabb) -> f32 {
    perimeter(&union(a, b)) - perimeter(a)
}

/// 一组 (id, 盒) 的 (并集盒, 最大子高)——高由闭包给（叶/内部两种来源）。
fn subtree_bound<F: Fn(u32) -> u32>(kids: &[(u32, Aabb)], height_of: F) -> (Aabb, u32) {
    debug_assert!(!kids.is_empty());
    let mut a = kids[0].1;
    let mut h = height_of(kids[0].0);
    for (c, b) in kids.iter().skip(1) {
        a = union(&a, b);
        h = h.max(height_of(*c));
    }
    (a, h)
}

/// 8 路内部 + 窄叶 BVH（批量构建 + 增量插入/移动 + 查询）。
#[derive(Clone, Debug, Default)]
pub struct Bvh8 {
    nodes: Vec<Node8>,
    leaves: Vec<Leaf8>,
    root: u32,
    /// 体 id → 叶 id（`LEAF_BIT | idx`；未入树 = `NULL`）。
    leaf_of: Vec<u32>,
    /// 体 id → 体盒旁路（分裂排序 / 移动生长用）。
    body_box: Vec<Aabb>,
}

impl Bvh8 {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            leaves: Vec::new(),
            root: NULL,
            leaf_of: Vec::new(),
            body_box: Vec::new(),
        }
    }

    pub fn root(&self) -> u32 {
        self.root
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn leaf_count(&self) -> usize {
        self.leaves.len()
    }

    /// 树高（叶 0）。
    pub fn height(&self) -> u32 {
        if self.root == NULL || is_leaf(self.root) {
            0
        } else {
            self.nodes[self.root as usize].height
        }
    }

    /// 结构内存（字节）——与二叉 AoS 树同口径对账用。
    pub fn memory_bytes(&self) -> usize {
        self.nodes.len() * std::mem::size_of::<Node8>()
            + self.leaves.len() * std::mem::size_of::<Leaf8>()
    }

    #[inline]
    fn box_of(&self, id: u32) -> Aabb {
        if is_leaf(id) {
            self.leaves[leaf_index(id)].aabb
        } else {
            self.nodes[id as usize].box_union
        }
    }

    /// 内部节点「子盒并集」（由子槽现算）——写 `box_union` 前用。
    fn union_of_children(&self, idx: usize) -> Aabb {
        let n = self.nodes[idx].count as usize;
        let mut u = self.nodes[idx].aabb[0];
        for k in 1..n {
            u = union(&u, &self.nodes[idx].aabb[k]);
        }
        u
    }

    fn height_of(&self, id: u32) -> u32 {
        if is_leaf(id) {
            0
        } else {
            self.nodes[id as usize].height
        }
    }

    fn set_parent(&mut self, id: u32, p: u32) {
        if is_leaf(id) {
            self.leaves[leaf_index(id)].parent = p;
        } else {
            self.nodes[id as usize].parent = p;
        }
    }

    fn parent_of(&self, id: u32) -> u32 {
        if is_leaf(id) {
            self.leaves[leaf_index(id)].parent
        } else {
            self.nodes[id as usize].parent
        }
    }

    /// 全量重建（确定性 top-down 均分；叶 ≤ LEAF_CAP 即止）。
    /// `items` 须按体 id 升序（0..n）；返回每体的叶 id（与 `DynamicBvh::rebuild`
    /// 同契约，供宽相 `leaves` 对账）。
    pub fn rebuild(&mut self, items: &[(u32, Aabb)]) -> Vec<u32> {
        self.nodes.clear();
        self.leaves.clear();
        self.root = NULL;
        let cap = items
            .iter()
            .map(|(b, _)| *b as usize + 1)
            .max()
            .unwrap_or(0);
        self.leaf_of.clear();
        self.leaf_of.resize(cap, NULL);
        self.body_box.clear();
        self.body_box.resize(cap, zero_box());
        for (body, a) in items.iter() {
            self.body_box[*body as usize] = *a;
        }
        if !items.is_empty() {
            let mut work: Vec<(u32, Aabb)> = items.to_vec();
            self.root = self.build_range(&mut work);
        }
        self.validate();
        self.leaf_of.clone()
    }

    /// 体槽位数（≈ 最大体 id + 1）——调用方判「有无新体」用。
    pub fn body_slots(&self) -> usize {
        self.leaf_of.len()
    }

    /// 某体的叶 id（未入树 = `NULL`）。
    pub fn leaf_of(&self, body: usize) -> u32 {
        self.leaf_of[body]
    }

    fn ensure_capacity(&mut self, body: usize) {
        if self.leaf_of.len() <= body {
            self.leaf_of.resize(body + 1, NULL);
            self.body_box.resize(body + 1, zero_box());
        }
    }

    fn build_range(&mut self, items: &mut [(u32, Aabb)]) -> u32 {
        debug_assert!(!items.is_empty());
        if items.len() <= LEAF_CAP {
            return self.make_leaf(items);
        }
        let mut bound = items[0].1;
        for (_, a) in items.iter().skip(1) {
            bound = union(&bound, a);
        }
        let axis = longest_axis(&bound);
        let n = items.len();
        // 段数按「每段 ≈ BRANCH·LEAF_CAP」取 ⇒ 叶装满、层数不虚高；至少 2 段。
        let parts = BRANCH.min(n.div_ceil(LEAF_CAP));
        debug_assert!((2..=BRANCH).contains(&parts));
        // 边界自高向低切（前缀内选择 ⇒ 前缀不被打乱；自低向高会打乱已切好的段）。
        let mut hi = n;
        for j in (1..parts).rev() {
            let k = n * j / parts;
            debug_assert!(k < hi);
            items[..hi].select_nth_unstable_by(k, |a, b| cmp_axis_center(a, b, axis));
            hi = k;
        }
        let mut aabb = [zero_box(); BRANCH];
        let mut child = [0u32; BRANCH];
        let mut count = 0usize;
        let mut h = 0u32;
        let mut u = zero_box();
        for j in 0..parts {
            let lo = n * j / parts;
            let hi2 = n * (j + 1) / parts;
            if lo == hi2 {
                continue;
            }
            let c = self.build_range(&mut items[lo..hi2]);
            aabb[count] = self.box_of(c);
            child[count] = c;
            u = if count == 0 {
                aabb[0]
            } else {
                union(&u, &aabb[count])
            };
            h = h.max(self.height_of(c));
            count += 1;
        }
        let idx = self.nodes.len() as u32;
        self.nodes.push(Node8 {
            aabb,
            child,
            box_union: u,
            parent: NULL,
            height: h + 1,
            count: count as u8,
        });
        for c in child.iter().take(count) {
            self.set_parent(*c, idx);
        }
        idx
    }

    fn make_leaf(&mut self, items: &[(u32, Aabb)]) -> u32 {
        debug_assert!(!items.is_empty() && items.len() <= LEAF_CAP);
        let mut aabb = items[0].1;
        let mut body = [0u32; LEAF_CAP];
        for (k, (b, a)) in items.iter().enumerate() {
            aabb = union(&aabb, a);
            body[k] = *b;
        }
        let idx = self.leaves.len() as u32;
        self.leaves.push(Leaf8 {
            aabb,
            body,
            parent: NULL,
            count: items.len() as u8,
        });
        let id = LEAF_BIT | idx;
        for (b, _) in items.iter() {
            self.leaf_of[*b as usize] = id;
        }
        id
    }

    // ============ 增量（B 树式溢出传播；无删体 ⇒ 无 remove） ============

    /// 单体的新叶（增量插入用；批量构建走 `make_leaf` 装满 LEAF_CAP）。
    fn make_leaf1(&mut self, body: u32, aabb: Aabb) -> u32 {
        let idx = self.leaves.len() as u32;
        let mut b = [0u32; LEAF_CAP];
        b[0] = body;
        self.leaves.push(Leaf8 {
            aabb,
            body: b,
            parent: NULL,
            count: 1,
        });
        LEAF_BIT | idx
    }

    /// 增量插入：返回叶 id。贪心下行选「并入后周长增量最小」的子树，直到其子为叶
    /// ⇒ 新叶挂到该叶的父；父满 ⇒ 切 5+4 向上传播；仅根溢出才新建根。
    pub fn insert(&mut self, body: u32, aabb: Aabb) -> u32 {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
        let leaf = self.make_leaf1(body, aabb);
        self.leaf_of[body as usize] = leaf;
        if self.root == NULL {
            self.root = leaf;
            return leaf;
        }
        if is_leaf(self.root) {
            // 根是叶（树 ≤2 体）：新建根，两叶为子。
            let old = self.root;
            self.root = self.make_root(old, leaf);
            return leaf;
        }
        // 下行：cur 是内部节点；选最优子，若该子是叶 ⇒ cur 即插入父。
        let mut cur = self.root;
        loop {
            let n = self.nodes[cur as usize].count as usize;
            let mut best = 0usize;
            let mut best_inc = f32::MAX;
            for k in 0..n {
                let c = self.nodes[cur as usize].child[k];
                let inc = enlargement(&self.box_of(c), &aabb);
                if inc < best_inc {
                    best_inc = inc;
                    best = k;
                }
            }
            let c = self.nodes[cur as usize].child[best];
            if is_leaf(c) {
                break;
            }
            cur = c;
        }
        let p = cur;
        let n = self.nodes[p as usize].count as usize;
        if n < BRANCH {
            self.nodes[p as usize].child[n] = leaf;
            self.nodes[p as usize].aabb[n] = self.box_of(leaf);
            self.nodes[p as usize].count = (n + 1) as u8;
            self.leaves[leaf_index(leaf)].parent = p;
            self.refit_upwards(p);
            return leaf;
        }
        // 父满（8 子）：切 5+4，新兄弟向上传播。
        let sib = self.split_node(p, leaf);
        self.settle_split(p, sib);
        leaf
    }

    /// 新建根，两子为 `a`/`b`（返回新根下标）。
    fn make_root(&mut self, a: u32, b: u32) -> u32 {
        let (ba, bb) = (self.box_of(a), self.box_of(b));
        let idx = self.nodes.len() as u32;
        let mut aabb = [zero_box(); BRANCH];
        let mut child = [0u32; BRANCH];
        aabb[0] = ba;
        aabb[1] = bb;
        child[0] = a;
        child[1] = b;
        let h = self.height_of(a).max(self.height_of(b));
        self.nodes.push(Node8 {
            aabb,
            child,
            box_union: union(&ba, &bb),
            parent: NULL,
            height: h + 1,
            count: 2,
        });
        self.set_parent(a, idx);
        self.set_parent(b, idx);
        idx
    }

    /// 内部节点溢出（8 槽 + `extra` = 9 子）⇒ 按最长轴切 5+4：`node` 留 5，
    /// 新兄弟（返回值）带 4。槽盒/父指针/高全部就位（`node.parent` 不动）。
    fn split_node(&mut self, node: u32, extra: u32) -> u32 {
        let n = self.nodes[node as usize].count as usize;
        debug_assert_eq!(n, BRANCH);
        // 先收养 extra：若它排序后落保留侧，下方「右侧改父」循环不会碰它。
        self.set_parent(extra, node);
        let mut kids: Vec<(u32, Aabb)> = (0..n)
            .map(|k| {
                let c = self.nodes[node as usize].child[k];
                (c, self.nodes[node as usize].aabb[k])
            })
            .collect();
        kids.push((extra, self.box_of(extra)));
        let mut bound = kids[0].1;
        for (_, b) in kids.iter().skip(1) {
            bound = union(&bound, b);
        }
        let axis = longest_axis(&bound);
        kids.sort_by(|a, b| cmp_axis_center(a, b, axis));
        let keep = kids.len().div_ceil(2); // 9 ⇒ 5
        let (lb, lh) = subtree_bound(&kids[..keep], |id| self.height_of(id));
        let (rb, rh) = subtree_bound(&kids[keep..], |id| self.height_of(id));
        let idx = self.nodes.len() as u32;
        let mut sa = [zero_box(); BRANCH];
        let mut sc = [0u32; BRANCH];
        for (k, (c, b)) in kids.iter().skip(keep).enumerate() {
            sa[k] = *b;
            sc[k] = *c;
        }
        self.nodes.push(Node8 {
            aabb: sa,
            child: sc,
            box_union: rb,
            parent: NULL,
            height: rh + 1,
            count: (kids.len() - keep) as u8,
        });
        for (c, _) in kids.iter().skip(keep) {
            self.set_parent(*c, idx);
        }
        {
            let n8 = &mut self.nodes[node as usize];
            for (k, (c, b)) in kids.iter().take(keep).enumerate() {
                n8.aabb[k] = *b;
                n8.child[k] = *c;
            }
            n8.count = keep as u8;
            n8.box_union = lb;
            n8.height = lh + 1;
        }
        idx
    }

    /// 把 `sib` 挂到 `child` 的父：有位 ⇒ 挂尾 + refit（早退合法）；满 ⇒ 切分继续
    /// 向上；到根 ⇒ 新建根（宽 2）。**仅此一处**新建根。
    fn settle_split(&mut self, mut child: u32, mut sib: u32) {
        loop {
            let p = self.parent_of(child);
            if p == NULL {
                self.root = self.make_root(child, sib);
                return;
            }
            // `child` 若刚分裂过，其对外盒**已收缩** ⇒ 先刷新父槽：否则父的并集
            // 与后续切分都从陈旧副本取值（本模块第二处踩坑：`槽盒 ≠ 子盒 @ 2[0]`）。
            self.set_slot_box(p, child);
            let n = self.nodes[p as usize].count as usize;
            if n < BRANCH {
                self.nodes[p as usize].child[n] = sib;
                self.nodes[p as usize].aabb[n] = self.box_of(sib);
                self.nodes[p as usize].count = (n + 1) as u8;
                self.set_parent(sib, p);
                self.refit_upwards(p);
                return;
            }
            let sib2 = self.split_node(p, sib);
            child = p;
            sib = sib2;
        }
    }

    /// 自 `node` 向上重算「对外盒（子盒并集）+ 高」。早退：本层对外呈现逐位未变
    /// ⇒ 其上只依赖本层，必然不变。
    ///
    /// **关键**：父节点存的是「子盒**副本**」（槽盒），故每上一层前必须把本节点的
    /// 新盒写回父槽（`set_slot_box`）——否则并集从**陈旧副本**重算，变化会在一层
    /// 之上「消失」（本模块第一版就踩了这个：`槽盒 ≠ 子盒 @ 2[0]`）。
    /// 调用方契约：`node` 自身或其子的几何变化，其**母节点的槽盒须已刷新**。
    fn refit_upwards(&mut self, mut node: u32) {
        loop {
            let idx = node as usize;
            let n = self.nodes[idx].count as usize;
            let mut u = self.nodes[idx].aabb[0];
            let mut h = self.height_of(self.nodes[idx].child[0]);
            for k in 1..n {
                u = union(&u, &self.nodes[idx].aabb[k]);
                h = h.max(self.height_of(self.nodes[idx].child[k]));
            }
            let nh = h + 1;
            if same_box(&self.nodes[idx].box_union, &u) && self.nodes[idx].height == nh {
                return;
            }
            self.nodes[idx].box_union = u;
            self.nodes[idx].height = nh;
            let p = self.nodes[idx].parent;
            if p == NULL {
                return;
            }
            self.set_slot_box(p, idx as u32);
            node = p;
        }
    }

    /// 把 `child` 在 `parent` 槽里的盒刷成当前值（`child` 几何变化后调用）。
    fn set_slot_box(&mut self, parent: u32, child: u32) {
        let n = self.nodes[parent as usize].count as usize;
        let mut slot = usize::MAX;
        for k in 0..n {
            if self.nodes[parent as usize].child[k] == child {
                slot = k;
                break;
            }
        }
        debug_assert!(slot != usize::MAX, "child 不在 parent 槽内");
        let cb = self.box_of(child);
        self.nodes[parent as usize].aabb[slot] = cb;
    }

    /// 按体 id 移动代理：叶盒已含「体盒 + 边距」⇒ 零结构操作；否则生长并向上 refit。
    /// 叶下标由 `leaf_of` 权威解析（增量分裂不改叶下标，但接入侧不该缓存）。
    pub fn move_proxy_body(&mut self, body: u32, aabb: Aabb, margin: f32) -> (u32, bool) {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
        let leaf = self.leaf_of[body as usize];
        if leaf == NULL {
            return (self.insert(body, aabb), true);
        }
        let m = vxl_phys_core::Vec3::splat(margin);
        let target = Aabb {
            min: aabb.min - m,
            max: aabb.max + m,
        };
        let l = &self.leaves[leaf_index(leaf)];
        if contains_box(&l.aabb, &target) {
            return (leaf, false);
        }
        // 叶内多体：只能并集扩张（不得凭单体的盒收缩叶盒）。
        let single = l.count == 1;
        let old = l.aabb;
        let new_box = if single { target } else { union(&old, &target) };
        let parent = l.parent;
        self.leaves[leaf_index(leaf)].aabb = new_box;
        if parent != NULL {
            self.set_slot_box(parent, leaf);
            self.refit_upwards(parent);
        }
        (leaf, true)
    }

    /// 体盒同步（调用方直接改盒而非走 `move_proxy_body` 时用）。
    pub fn set_body_box(&mut self, body: u32, aabb: Aabb) {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
    }

    /// 查询：**追加**与 `q` 相交叶内的体 id（不清空/不排序去重；每体至多入一次）。
    ///
    /// 遍历要点：内部层把「子盒是否相交」在**父槽**判掉（`aabb[k]` 就是子盒，
    /// 连续读）⇒ 被弹出的叶其盒已被验过，直接发射；根若是叶则无父替它验。
    pub fn query(&self, q: &Aabb, out: &mut Vec<u32>) {
        if self.root == NULL {
            return;
        }
        if is_leaf(self.root) {
            let l = &self.leaves[leaf_index(self.root)];
            if l.aabb.overlaps(q) {
                self.emit(l, out);
            }
            return;
        }
        let mut stack: Vec<u32> = Vec::with_capacity(32);
        stack.push(self.root);
        while let Some(id) = stack.pop() {
            if is_leaf(id) {
                self.emit(&self.leaves[leaf_index(id)], out);
            } else {
                let n8 = &self.nodes[id as usize];
                for k in 0..n8.count as usize {
                    if n8.aabb[k].overlaps(q) {
                        stack.push(n8.child[k]);
                    }
                }
            }
        }
    }

    #[inline]
    fn emit(&self, l: &Leaf8, out: &mut Vec<u32>) {
        for k in 0..l.count as usize {
            out.push(l.body[k]);
        }
    }

    /// 遍历统计（与 `wide::WideBvh::traversal_stats` 同口径）：(访问节点数, 访问叶数)。
    /// 计数含被弹出但盒不相交者（对齐宽叶/二叉的对照口径）。
    pub fn traversal_stats(&self, q: &Aabb) -> (u64, u64) {
        let (mut nodes, mut leaves) = (0u64, 0u64);
        if self.root == NULL {
            return (0, 0);
        }
        if is_leaf(self.root) {
            return (0, 1);
        }
        let mut stack: Vec<u32> = Vec::with_capacity(32);
        stack.push(self.root);
        while let Some(id) = stack.pop() {
            if is_leaf(id) {
                leaves += 1;
            } else {
                nodes += 1;
                let n8 = &self.nodes[id as usize];
                for k in 0..n8.count as usize {
                    if n8.aabb[k].overlaps(q) {
                        stack.push(n8.child[k]);
                    }
                }
            }
        }
        (nodes, leaves)
    }

    /// 结构校验（测试/构建后自检）。
    pub fn validate(&self) {
        if self.root == NULL {
            assert!(self.nodes.is_empty() && self.leaves.is_empty());
            return;
        }
        assert_eq!(self.parent_of(self.root), NULL, "根父指针非空");
        self.check(self.root, None);
    }

    fn check(&self, id: u32, parent_box: Option<Aabb>) {
        let b = self.box_of(id);
        if let Some(pb) = parent_box {
            assert!(contains_box(&pb, &b), "包含性破坏：{id} 越出祖先盒");
        }
        if is_leaf(id) {
            let l = &self.leaves[leaf_index(id)];
            assert!(
                l.count >= 1 && l.count as usize <= LEAF_CAP,
                "叶体数越界：{}",
                l.count
            );
            assert!(same_box(&l.aabb, &b), "叶盒不自洽");
            return;
        }
        let n8 = &self.nodes[id as usize];
        assert!(
            n8.count >= 2 && n8.count as usize <= BRANCH,
            "内部子数越界：{}",
            n8.count
        );
        let mut h = 0u32;
        for k in 0..n8.count as usize {
            let c = n8.child[k];
            assert!(
                same_box(&n8.aabb[k], &self.box_of(c)),
                "槽盒 ≠ 子盒 @ {id}[{k}]"
            );
            assert_eq!(self.parent_of(c), id, "父指针不自洽 @ {id}[{k}]");
            h = h.max(self.height_of(c));
        }
        assert_eq!(n8.height, h + 1, "高度不自洽 @ {id}");
        assert!(
            same_box(&n8.box_union, &self.union_of_children(id as usize)),
            "box_union ≠ 子盒并集 @ {id}"
        );
        for k in 0..n8.count as usize {
            self.check(n8.child[k], Some(b));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb(x: f32, y: f32, half: f32) -> Aabb {
        let c = Vec3::new(x, y, 0.0);
        Aabb {
            min: c - Vec3::splat(half),
            max: c + Vec3::splat(half),
        }
    }

    fn layout(n: u32) -> Vec<(u32, Aabb)> {
        (0..n)
            .map(|k| {
                let x = ((k.wrapping_mul(2_654_435_761)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let y = ((k.wrapping_mul(40_503)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let half = 0.3 + ((k.wrapping_mul(97)) % 100) as f32 / 1000.0;
                (k, aabb(x, y, half))
            })
            .collect()
    }

    fn build(items: &[(u32, Aabb)]) -> Bvh8 {
        let mut t = Bvh8::new();
        t.rebuild(items);
        t
    }

    /// 叶深差 ≤1。**注**：按计数细分 + 叶容 2 时 n 非 8 的幂**无法**整除，
    /// 故批构建天然允许跨 1 层（如 17 体 ⇒ 7 个 2 体叶 + 1 个含 3 体的子树）。
    /// 严格等高只有**增量路径**能做到（B 树式分裂，见 `wide.rs`）；本原型只做批量，
    /// 故按真实性质断言（差 >1 才算退化）。
    fn assert_depth_span_le_one(t: &Bvh8) {
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
    fn bvh8_build_keeps_invariants() {
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
    fn bvh8_query_matches_brute_force() {
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
    fn bvh8_edge_cases() {
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
    fn bvh8_build_is_deterministic() {
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

    fn build_inc(items: &[(u32, Aabb)]) -> Bvh8 {
        let mut t = Bvh8::new();
        for (b, a) in items {
            t.insert(*b, *a);
            t.validate();
        }
        t
    }

    /// 增量插入：不变量 + **严格全叶同深**（B 树式分裂的固有性质，区别于批构建）。
    #[test]
    fn bvh8_incremental_insert_keeps_invariants() {
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
    fn bvh8_incremental_query_matches_brute_force() {
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
    fn bvh8_move_proxy_growth_only() {
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
    fn bvh8_bulk_and_incremental_agree() {
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
