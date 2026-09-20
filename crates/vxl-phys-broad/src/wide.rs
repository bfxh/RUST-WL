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

/// 每内部节点的最大子数（8 路 ⇒ 遍历层数为二叉的 1/(log2 8) = 1/3）。
pub const WIDE: usize = 8;

const NULL: u32 = u32::MAX;

/// 宽节点。`height == 0` 为叶（体 ≤ `WIDE` 个）；否则 `count` ∈ 2..=WIDE。
///
/// 注：`slot` 恰好 `WIDE` 项——溢出（第 9 个）**不入槽**，由分裂函数以
/// 额外参数接收（`split_leaf(leaf, extra)` / `split_node(node, extra)`），
/// 故本结构无「瞬态 9 子」状态，布局与 WIDE 二叉同宽。
#[derive(Clone, Copy, Debug)]
pub struct WideNode {
    pub aabb: Aabb,
    pub height: u32,
    pub parent: u32,
    /// 叶：体 id；内部：子节点下标。前 `count` 项有效。
    pub slot: [u32; WIDE],
    pub count: u8,
}

#[inline]
fn union(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

#[inline]
fn perimeter(a: &Aabb) -> f32 {
    let d = a.max - a.min;
    2.0 * (d.x + d.y + d.z)
}

/// 把 `b` 并入 `a` 后的周长增量（最佳子选择用）。
#[inline]
fn enlargement(a: &Aabb, b: &Aabb) -> f32 {
    perimeter(&union(a, b)) - perimeter(a)
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

/// (轴中心, 体/节点 id) 全序——分裂与批构建共用的确定性键。
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

#[inline]
fn zero_box() -> Aabb {
    Aabb {
        min: vxl_phys_core::Vec3::ZERO,
        max: vxl_phys_core::Vec3::ZERO,
    }
}

/// 8 路 BVH：批量构建 + 增量插入/删除/移动 + 查询。
#[derive(Clone, Debug, Default)]
pub struct WideBvh {
    nodes: Vec<WideNode>,
    root: u32,
    /// 体 id → 叶节点下标（删除后为 `NULL`）。
    leaf_of: Vec<u32>,
    /// 体 id → 体盒（旁路；宽叶无法反推单体盒）。
    body_box: Vec<Aabb>,
}

impl WideBvh {
    pub fn new() -> Self {
        Self::default_nodes()
    }

    fn default_nodes() -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL,
            leaf_of: Vec::new(),
            body_box: Vec::new(),
        }
    }

    /// 预置体容量（增量插入前调用；体 id 必须 < capacity）。
    pub fn new_with_capacity(capacity: usize) -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL,
            leaf_of: vec![NULL; capacity],
            body_box: vec![zero_box(); capacity],
        }
    }

    pub fn nodes(&self) -> &[WideNode] {
        &self.nodes
    }

    pub fn root(&self) -> u32 {
        self.root
    }

    pub fn leaf_of(&self, body: usize) -> u32 {
        self.leaf_of[body]
    }

    pub fn leaf_count(&self) -> usize {
        self.nodes.iter().filter(|n| n.height == 0).count()
    }

    pub fn height(&self) -> u32 {
        if self.root == NULL {
            0
        } else {
            self.nodes[self.root as usize].height
        }
    }

    fn ensure_capacity(&mut self, body: usize) {
        if self.leaf_of.len() <= body {
            self.leaf_of.resize(body + 1, NULL);
            self.body_box.resize(body + 1, zero_box());
        }
    }

    // ============ 批量构建（top-down 8 路均分） ============

    /// 全量重建（确定性 top-down 8 路均分）。`items` 须按体 id 升序（0..n），
    /// 返回每体的叶子节点索引（与 `bvh::DynamicBvh::rebuild` 同契约）。
    ///
    /// 每层按最长轴 `select_nth` 切 8 段（**按计数均分** ⇒ 树高
    /// `ceil(log8(n/8))+1`，与分布无关、绝不链化）；段 ≤ WIDE 即成叶。
    /// 键 = (轴中心, 体 id) 全序 ⇒ 分段集合由秩唯一确定（§5 确定性）。
    pub fn rebuild(&mut self, items: &[(u32, Aabb)]) -> Vec<u32> {
        self.nodes.clear();
        self.root = NULL;
        self.leaf_of.clear();
        self.leaf_of.resize(items.len(), NULL);
        self.body_box.clear();
        self.body_box.resize(items.len(), zero_box());
        for (body, a) in items.iter() {
            // 建树前先落体盒：叶盒 = 体内盒并集（不变量②）与后续分裂排序都读它。
            self.body_box[*body as usize] = *a;
        }
        if !items.is_empty() {
            let mut work: Vec<(u32, Aabb)> = items.to_vec();
            self.root = self.build_range(&mut work);
        }
        self.validate();
        self.leaf_of.clone()
    }

    /// 递归体：n ≤ WIDE 成叶；否则按最长轴切 8 段递归。
    ///
    /// 递归深度 = `ceil(log8(n/8))`（200k 体 ≈6 层），且**按计数均分**（不看
    /// 数据分布）⇒ 任何输入都不会退化成深递归。
    fn build_range(&mut self, items: &mut [(u32, Aabb)]) -> u32 {
        debug_assert!(!items.is_empty());
        if items.len() <= WIDE {
            return self.make_leaf(items);
        }
        let mut bound = items[0].1;
        for (_, a) in items.iter().skip(1) {
            bound = union(&bound, a);
        }
        let axis = longest_axis(&bound);
        let n = items.len();
        // 段数 = round(n/WIDE) 钳到 2..=WIDE：目标是**每段 ≈WIDE 体**（叶装满）。
        // 每层固定切 8 段会让 16 体的段碎成 8 个叶各 2 体——叶数翻倍、遍历变差
        // （实测 8000 体 4096 叶 vs 1024 叶）。
        let parts = (n + WIDE / 2) / WIDE;
        let parts = parts.clamp(2, WIDE);
        // 边界自高向低切（第 j 边界 = j·n/parts）：每次只在前缀内选择 ⇒ 前缀不
        // 被打乱 ⇒ 各段 = 秩区间 [j·n/parts, (j+1)·n/parts)。自低向高会打乱已切好的段。
        let mut hi = n;
        for j in (1..parts).rev() {
            let k = n * j / parts;
            debug_assert!(k < hi);
            items[..hi].select_nth_unstable_by(k, |a, b| cmp_axis_center(a, b, axis));
            hi = k;
        }
        let mut kids = [0u32; WIDE];
        let mut nk = 0usize;
        for j in 0..parts {
            let lo = n * j / parts;
            let hi = n * (j + 1) / parts;
            if lo == hi {
                continue;
            }
            kids[nk] = self.build_range(&mut items[lo..hi]);
            nk += 1;
        }
        self.make_internal(&kids[..nk])
    }

    /// 叶：体数 1..=WIDE；盒 = 体内盒并集；写 `leaf_of`。
    fn make_leaf(&mut self, items: &[(u32, Aabb)]) -> u32 {
        debug_assert!(!items.is_empty() && items.len() <= WIDE);
        let mut node = WideNode {
            aabb: items[0].1,
            height: 0,
            parent: NULL,
            slot: [0; WIDE],
            count: items.len() as u8,
        };
        for (k, (body, b)) in items.iter().enumerate() {
            node.aabb = union(&node.aabb, b);
            node.slot[k] = *body;
        }
        let idx = self.push_node(node);
        for (body, _) in items.iter() {
            self.leaf_of[*body as usize] = idx;
        }
        idx
    }

    /// 内部：子数 2..=WIDE；盒 = 子盒并集；高 = 1+max(子高)；接线父指针。
    fn make_internal(&mut self, kids: &[u32]) -> u32 {
        debug_assert!(kids.len() >= 2 && kids.len() <= WIDE);
        let first = self.nodes[kids[0] as usize];
        let mut node = WideNode {
            aabb: first.aabb,
            height: first.height,
            parent: NULL,
            slot: [0; WIDE],
            count: kids.len() as u8,
        };
        node.slot[0] = kids[0];
        for (k, c) in kids.iter().enumerate().skip(1) {
            let cn = self.nodes[*c as usize];
            node.aabb = union(&node.aabb, &cn.aabb);
            node.height = node.height.max(cn.height);
            node.slot[k] = *c;
        }
        node.height += 1;
        let idx = self.push_node(node);
        for c in kids.iter() {
            self.nodes[*c as usize].parent = idx;
        }
        idx
    }

    // ============ 增量插入（B 树式溢出传播） ============

    /// 增量插入：返回叶下标。
    ///
    /// 下行 = 自根逐层取「并入后周长增量最小」的子（平局取小下标 ⇒ 确定性）；
    /// 落叶有位则就地落位 + `refit_upwards`（仅叶盒变化 ⇒ 提前退出版合法）；
    /// 叶满则切 4+5 并**向上传播溢出**（见模块注释）。
    pub fn insert(&mut self, body: u32, aabb: Aabb) -> u32 {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
        if self.root == NULL {
            let mut leaf = WideNode {
                aabb,
                height: 0,
                parent: NULL,
                slot: [0; WIDE],
                count: 1,
            };
            leaf.slot[0] = body;
            let leaf = self.push_node(leaf);
            self.root = leaf;
            self.leaf_of[body as usize] = leaf;
            return leaf;
        }
        let mut cur = self.root;
        while self.nodes[cur as usize].height != 0 {
            let n = self.nodes[cur as usize].count as usize;
            let mut best = 0usize;
            let mut best_inc = f32::MAX;
            for k in 0..n {
                let child = self.nodes[cur as usize].slot[k] as usize;
                let inc = enlargement(&self.nodes[child].aabb, &aabb);
                if inc < best_inc {
                    best_inc = inc;
                    best = k;
                }
            }
            cur = self.nodes[cur as usize].slot[best];
        }
        let leaf = cur;
        let count = self.nodes[leaf as usize].count as usize;
        self.leaf_of[body as usize] = leaf;
        if count < WIDE {
            // 叶有位：落位 + 自父向上 refit（仅叶盒变化 ⇒ 可用提前退出版）。
            let node = &mut self.nodes[leaf as usize];
            node.slot[count] = body;
            node.count = (count + 1) as u8;
            node.aabb = union(&node.aabb, &aabb);
            let parent = self.nodes[leaf as usize].parent;
            self.refit_upwards(parent);
            return leaf;
        }
        // 叶满（8 体）：第 9 体不入槽，直接交给分裂（split_leaf 以 extra 接收）。
        let sib = self.split_leaf(leaf, body);
        self.settle_split(leaf, sib);
        self.leaf_of[body as usize]
    }

    /// 叶溢出（8 旧体 + `extra` 新体 = 9）⇒ 按最长轴排序切成 4+5：
    /// `leaf` 留前 4 体，新叶（返回值）带走后 5 体。两叶的盒/体数/`leaf_of`
    /// 全部就位；新叶父指针待 `settle_split` 挂接。
    fn split_leaf(&mut self, leaf: u32, extra: u32) -> u32 {
        let n = self.nodes[leaf as usize].count as usize;
        debug_assert_eq!(n, WIDE);
        let mut items: Vec<(u32, Aabb)> = (0..n)
            .map(|k| {
                let b = self.nodes[leaf as usize].slot[k];
                (b, self.body_box[b as usize])
            })
            .collect();
        items.push((extra, self.body_box[extra as usize]));
        // 轴取自 9 体并集（叶盒可能不含新体、且可能被 fat 边距撑大 ⇒ 不复用叶盒）。
        let mut bound = items[0].1;
        for (_, b) in items.iter().skip(1) {
            bound = union(&bound, b);
        }
        let axis = longest_axis(&bound);
        items.sort_by(|a, b| cmp_axis_center(a, b, axis));
        let keep = items.len() / 2; // 4
        let mut lslot = [0u32; WIDE];
        let mut rslot = [0u32; WIDE];
        let mut lbox = items[0].1;
        let mut rbox = items[keep].1;
        for (k, (b, bx)) in items.iter().enumerate() {
            if k < keep {
                lslot[k] = *b;
                lbox = union(&lbox, bx);
            } else {
                rslot[k - keep] = *b;
                rbox = union(&rbox, bx);
            }
        }
        let sib = self.push_node(WideNode {
            aabb: rbox,
            height: 0,
            parent: NULL,
            slot: rslot,
            count: (items.len() - keep) as u8,
        });
        {
            let node = &mut self.nodes[leaf as usize];
            node.slot = lslot;
            node.count = keep as u8;
            node.aabb = lbox;
        }
        for (b, _) in items.iter().take(keep) {
            self.leaf_of[*b as usize] = leaf;
        }
        for (b, _) in items.iter().skip(keep) {
            self.leaf_of[*b as usize] = sib;
        }
        sib
    }

    /// 内部节点溢出（8 子 + `extra` 新子 = 9）⇒ 按最长轴排序切成 5+4：
    /// `node` 留前 5 子，新兄弟（返回值）带走后 4 子。子父指针 / 盒 / 高 /
    /// 子数全部就位；`node.parent` 不动（新兄弟的父指针待 `settle_split` 挂接）。
    ///
    /// 切分不改变「并集盒」与「最大子高」⇒ 祖先的盒/高在切分前后逐位相同。
    fn split_node(&mut self, node: u32, extra: u32) -> u32 {
        let n = self.nodes[node as usize].count as usize;
        debug_assert_eq!(n, WIDE);
        // 先收养 `extra`：它尚未挂接（父指针为空），若排序后落在**保留侧**，
        // 下面的「右侧改父」循环不会碰它 ⇒ 必须现在指向 `node`。
        self.nodes[extra as usize].parent = node;
        let mut kids: Vec<(u32, Aabb)> = (0..n)
            .map(|k| {
                let c = self.nodes[node as usize].slot[k];
                (c, self.nodes[c as usize].aabb)
            })
            .collect();
        kids.push((extra, self.nodes[extra as usize].aabb));
        let mut bound = kids[0].1;
        for (_, b) in kids.iter().skip(1) {
            bound = union(&bound, b);
        }
        let axis = longest_axis(&bound);
        kids.sort_by(|a, b| cmp_axis_center(a, b, axis));
        let keep = kids.len().div_ceil(2); // 5
        let mut lslot = [0u32; WIDE];
        let mut rslot = [0u32; WIDE];
        for (k, (c, _)) in kids.iter().enumerate() {
            if k < keep {
                lslot[k] = *c;
            } else {
                rslot[k - keep] = *c;
            }
        }
        let (lbox, lh) = self.children_bound(&lslot[..keep]);
        let (rbox, rh) = self.children_bound(&rslot[..kids.len() - keep]);
        let sib = self.push_node(WideNode {
            aabb: rbox,
            height: rh + 1,
            parent: NULL,
            slot: rslot,
            count: (kids.len() - keep) as u8,
        });
        for (c, _) in kids.iter().skip(keep) {
            self.nodes[*c as usize].parent = sib;
        }
        {
            let kept = &mut self.nodes[node as usize];
            kept.slot = lslot;
            kept.count = keep as u8;
            kept.aabb = lbox;
            kept.height = lh + 1;
        }
        sib
    }

    /// 子下标集的 (并集盒, 最大子高)。
    fn children_bound(&self, kids: &[u32]) -> (Aabb, u32) {
        debug_assert!(!kids.is_empty());
        let mut aabb = self.nodes[kids[0] as usize].aabb;
        let mut h = self.nodes[kids[0] as usize].height;
        for c in kids.iter().skip(1) {
            let cn = self.nodes[*c as usize];
            aabb = union(&aabb, &cn.aabb);
            h = h.max(cn.height);
        }
        (aabb, h)
    }

    /// 把 `sib` 挂到 `child` 的父上：父有位 ⇒ 挂尾 + `refit_upwards`（提前退出
    /// 合法：祖先只经盒/高依赖本层）；父已满 ⇒ `split_node` 切分后继续向上；
    /// 一路到根仍溢出 ⇒ 新建根（宽 2）。**仅此一处**新建根。
    fn settle_split(&mut self, mut child: u32, mut sib: u32) {
        loop {
            let p = self.nodes[child as usize].parent;
            if p == NULL {
                // child 是根 ⇒ 建新根（两个子等高，故高 = 子高 + 1）。
                let ch = self.nodes[child as usize].height;
                let sh = self.nodes[sib as usize].height;
                let mut root = WideNode {
                    aabb: union(
                        &self.nodes[child as usize].aabb,
                        &self.nodes[sib as usize].aabb,
                    ),
                    height: ch.max(sh) + 1,
                    parent: NULL,
                    slot: [0; WIDE],
                    count: 2,
                };
                root.slot[0] = child;
                root.slot[1] = sib;
                let root = self.push_node(root);
                self.nodes[child as usize].parent = root;
                self.nodes[sib as usize].parent = root;
                self.root = root;
                return;
            }
            let pc = self.nodes[p as usize].count as usize;
            if pc < WIDE {
                self.nodes[sib as usize].parent = p;
                let node = &mut self.nodes[p as usize];
                node.slot[pc] = sib;
                node.count = (pc + 1) as u8;
                self.refit_upwards(p);
                return;
            }
            // 父已满：切 5+4（新兄弟 = 多出的那个），继续向上传播。
            let sib2 = self.split_node(p, sib);
            child = p;
            sib = sib2;
        }
    }

    fn push_node(&mut self, node: WideNode) -> u32 {
        self.nodes.push(node);
        (self.nodes.len() - 1) as u32
    }

    // ============ 删除 / 移动 ============

    /// 删除：叶内体数 >1 ⇒ 摘体；==1 ⇒ 摘叶；父剩 1 子 ⇒ 塌陷。
    pub fn remove(&mut self, body: u32) {
        let leaf = self.leaf_of[body as usize];
        if leaf == NULL {
            return;
        }
        self.leaf_of[body as usize] = NULL;
        let n = self.nodes[leaf as usize].count as usize;
        if n > 1 {
            let old = self.nodes[leaf as usize].slot;
            let mut slot = [0u32; WIDE];
            let mut count = 0u8;
            let mut aabb: Option<Aabb> = None;
            for b0 in old.iter().take(n) {
                if *b0 == body {
                    continue;
                }
                slot[count as usize] = *b0;
                count += 1;
                let b = self.body_box[*b0 as usize];
                aabb = Some(match aabb {
                    None => b,
                    Some(acc) => union(&acc, &b),
                });
            }
            let parent = self.nodes[leaf as usize].parent;
            let node = &mut self.nodes[leaf as usize];
            node.slot = slot;
            node.count = count;
            if let Some(a) = aabb {
                node.aabb = a;
            }
            self.refit_upwards(parent);
            return;
        }
        // 叶只含该体：摘叶。
        let parent = self.nodes[leaf as usize].parent;
        if parent == NULL {
            self.root = NULL;
            return;
        }
        let p = parent as usize;
        let old = self.nodes[p].slot;
        let pc = self.nodes[p].count as usize;
        let mut slot = [0u32; WIDE];
        let mut count = 0u8;
        let mut aabb: Option<Aabb> = None;
        for c in old.iter().take(pc) {
            let c = *c;
            if c == leaf {
                continue;
            }
            slot[count as usize] = c;
            count += 1;
            let b = self.nodes[c as usize].aabb;
            aabb = Some(match aabb {
                None => b,
                Some(acc) => union(&acc, &b),
            });
        }
        let grand = self.nodes[p].parent;
        if count == 1 {
            // 塌陷：唯一子提升至父的位置。
            let only = slot[0];
            self.nodes[only as usize].parent = grand;
            if grand == NULL {
                self.root = only;
            } else {
                let g = grand as usize;
                for k in 0..self.nodes[g].count as usize {
                    if self.nodes[g].slot[k] == parent {
                        self.nodes[g].slot[k] = only;
                        break;
                    }
                }
                self.refit_upwards(grand);
            }
            return;
        }
        let mut height = 0u32;
        for c in slot.iter().take(count as usize) {
            height = height.max(self.nodes[*c as usize].height);
        }
        let node = &mut self.nodes[p];
        node.slot = slot;
        node.count = count;
        node.height = height + 1;
        if let Some(a) = aabb {
            node.aabb = a;
        }
        self.refit_upwards(grand);
    }

    /// 移动代理：返回 (叶, 是否变化)。叶盒已含「体盒 + 边距」⇒ 零结构操作。
    pub fn move_proxy(&mut self, leaf: u32, aabb: Aabb, margin: f32) -> (u32, bool) {
        let m = vxl_phys_core::Vec3::splat(margin);
        let target = Aabb {
            min: aabb.min - m,
            max: aabb.max + m,
        };
        let cur = self.nodes[leaf as usize].aabb;
        if contains_box(&cur, &target) {
            return (leaf, false);
        }
        // 叶内多体：只能并集扩张（不得凭单体的盒收缩叶盒）。
        let single = self.nodes[leaf as usize].count == 1;
        let node = &mut self.nodes[leaf as usize];
        node.aabb = if single {
            target
        } else {
            union(&node.aabb, &target)
        };
        let parent = self.nodes[leaf as usize].parent;
        self.refit_upwards(parent);
        (leaf, true)
    }

    /// 体盒同步（`move_proxy` 的调用方在体盒变化后同步旁路）。
    pub fn set_body_box(&mut self, body: u32, aabb: Aabb) {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
    }

    /// 已登记的体槽位数（≈ 最大体 id + 1）——调用方判「有无新体」用。
    pub fn body_slots(&self) -> usize {
        self.leaf_of.len()
    }

    /// 按**体 id** 移动代理（宽相接入路径）。权威解析叶下标：分裂会把体搬到
    /// 新叶 ⇒ 调用方缓存的旧叶下标会失效。同步体盒旁路（分裂按它重算叶盒）。
    /// 未在树内的体按新体插入（返回 `changed = true`，调用方据此失效缓存）。
    pub fn move_proxy_body(&mut self, body: u32, aabb: Aabb, margin: f32) -> (u32, bool) {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
        let leaf = self.leaf_of[body as usize];
        if leaf == NULL {
            return (self.insert(body, aabb), true);
        }
        self.move_proxy(leaf, aabb, margin)
    }

    /// 自 `node` 向上 refit（盒 = 子盒并集；高 = 1+max(子高)），**提前退出**
    /// （本层盒与高度均未变 ⇒ 其上只依赖本层，必然不变）。
    ///
    /// **只允许用于「子集不变、仅几何变化」的场景**（插入落位 / 移动 / 叶内摘体）；
    /// 结构变化必须先把新子**写进槽位**再调用（本函数读子算盒）——二叉树那轮失败的
    /// 根因正是结构变化处漏写槽位而首层误判提前退出。
    fn refit_upwards(&mut self, mut node: u32) {
        while node != NULL {
            let n = self.nodes[node as usize].count as usize;
            let first = self.nodes[node as usize].slot[0] as usize;
            let mut aabb = self.nodes[first].aabb;
            let mut height = self.nodes[first].height;
            for k in 1..n {
                let c = self.nodes[node as usize].slot[k] as usize;
                aabb = union(&aabb, &self.nodes[c].aabb);
                height = height.max(self.nodes[c].height);
            }
            let cur = &self.nodes[node as usize];
            if cur.height == height + 1 && same_box(&cur.aabb, &aabb) {
                return;
            }
            let node_ref = &mut self.nodes[node as usize];
            node_ref.aabb = aabb;
            node_ref.height = height + 1;
            node = self.nodes[node as usize].parent;
        }
    }

    // ============ 查询 ============

    /// 查询：**追加**与 `q` 相交叶中的体 id（不清空 `out`、不去重排序；深度优先
    /// ⇒ 确定性顺序）。与 `bvh::DynamicBvh::query`（清空 + 排序去重）不同：
    /// 每叶至多入一次 ⇒ 本函数无重复；宽相调用方每次传空 `Vec` 即可。
    pub fn query(&self, q: &Aabb, out: &mut Vec<u32>) {
        if self.root == NULL {
            return;
        }
        let mut stack = vec![self.root];
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i as usize];
            if !node.aabb.overlaps(q) {
                continue;
            }
            if node.height == 0 {
                for k in 0..node.count as usize {
                    out.push(node.slot[k]);
                }
            } else {
                for k in (0..node.count as usize).rev() {
                    stack.push(node.slot[k]);
                }
            }
        }
    }

    /// 遍历统计（接入前量收益用）：(访问节点数, 访问叶数)。
    pub fn traversal_stats(&self, q: &Aabb) -> (u64, u64) {
        let (mut nodes, mut leaves) = (0u64, 0u64);
        if self.root == NULL {
            return (0, 0);
        }
        let mut stack = vec![self.root];
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i as usize];
            nodes += 1;
            if !node.aabb.overlaps(q) {
                continue;
            }
            if node.height == 0 {
                leaves += 1;
            } else {
                for k in 0..node.count as usize {
                    stack.push(node.slot[k]);
                }
            }
        }
        (nodes, leaves)
    }

    // ============ 不变量自检 ============

    pub fn validate(&self) {
        if self.root == NULL {
            // 死节点（remove 后未回收）允许残留：本模块不回收槽位，由批量重建
            // （宽相的高度阈值分支）整体重置。此处只要求「根为空」。
            return;
        }
        assert_eq!(self.nodes[self.root as usize].parent, NULL, "根父指针非空");
        self.check(self.root, None);
    }

    fn check(&self, i: u32, parent_box: Option<Aabb>) {
        let node = &self.nodes[i as usize];
        if let Some(pb) = parent_box {
            assert!(
                contains_box(&pb, &node.aabb),
                "包含性破坏：节点 {i} 越出祖先盒"
            );
        }
        if node.height == 0 {
            assert!(node.count >= 1 && node.count as usize <= WIDE, "叶体数越界");
            // 叶盒必须**包含**体内盒并集：移动代理会把叶盒扩到「体盒 + fat 边距」，
            // 故不能要求相等（相等只在不移动时成立）。
            let mut expect = self.body_box[node.slot[0] as usize];
            for k in 1..node.count as usize {
                expect = union(&expect, &self.body_box[node.slot[k] as usize]);
            }
            assert!(
                contains_box(&node.aabb, &expect),
                "叶盒未包含体内盒并集 @ {i}"
            );
            return;
        }
        assert!(
            node.count >= 2 && node.count as usize <= WIDE,
            "内部子数越界：{}",
            node.count
        );
        let mut expect = self.nodes[node.slot[0] as usize].aabb;
        let mut h = self.nodes[node.slot[0] as usize].height;
        for k in 1..node.count as usize {
            let c = self.nodes[node.slot[k] as usize];
            expect = union(&expect, &c.aabb);
            h = h.max(c.height);
        }
        assert_eq!(node.height, h + 1, "高度不自洽 @ {i}");
        assert!(same_box(&node.aabb, &expect), "盒 ≠ 子盒并集 @ {i}");
        for k in 0..node.count as usize {
            let c = node.slot[k];
            assert_eq!(self.nodes[c as usize].parent, i, "父指针不自洽 @ {i}");
            self.check(c, Some(node.aabb));
        }
    }
}

/// 体 id 集合排序去重（测试对照用）。
#[allow(dead_code)]
pub fn sorted_unique(mut v: Vec<u32>) -> Vec<u32> {
    v.sort_unstable();
    v.dedup();
    v
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

    fn build_cap(items: &[(u32, Aabb)]) -> WideBvh {
        let mut t = WideBvh::new_with_capacity(items.len() + 4);
        for (body, b) in items {
            t.insert(*body, *b);
        }
        t.validate();
        t
    }

    /// 查询完备性 + 过取可解释（两棵树共用）。
    fn check_query(t: &WideBvh, items: &[(u32, Aabb)], q: &Aabb) {
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
    fn assert_balanced(t: &WideBvh) {
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
    fn wide_incremental_insert_keeps_invariants() {
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
    fn wide_incremental_query_matches_brute_force() {
        let items = layout(1500);
        let t = build_cap(&items);
        for q in items.iter().step_by(37).map(|(_, b)| *b) {
            check_query(&t, &items, &q);
        }
    }

    /// 删除：摘一半 → 不变量 + 查询收缩；全摘 → 空树。
    #[test]
    fn wide_remove_keeps_invariants() {
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
    fn wide_remove_then_reinsert_keeps_invariants() {
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
    fn wide_move_proxy_growth_only() {
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
    fn wide_incremental_height_matches_ideal() {
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
    fn wide_incremental_height_within_one_of_bulk() {
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
    fn wide_move_proxy_body_resolves_split_relocation() {
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
    fn wide_leaf_split_keeps_all_bodies() {
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
    fn wide_deep_insert_stays_balanced() {
        let t = build_cap(&layout(5000));
        assert_balanced(&t);
        assert!(t.height() <= 6, "5000 体高度 {} 偏深", t.height());
    }
}
