//! wide_impl：从 wide.rs 按域拆出（纯搬移，语义未改）。
use super::*;
// 共享 AABB 助手在 crate 根（`aabb_util`）：孙模块的 `use super::*` 只到父模块 ⇒ 显式点名。
use crate::aabb_util::*;

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

/// 8 路 BVH：批量构建 + 增量插入/删除/移动 + 查询。
#[derive(Clone, Debug, Default)]
pub struct WideBvh {
    pub(crate) nodes: Vec<WideNode>,
    pub(crate) root: u32,
    /// 体 id → 叶节点下标（删除后为 `NULL`）。
    pub(crate) leaf_of: Vec<u32>,
    /// 体 id → 体盒（旁路；宽叶无法反推单体盒）。
    pub(crate) body_box: Vec<Aabb>,
}

impl WideBvh {
    pub fn new() -> Self {
        Self::default_nodes()
    }

    pub(crate) fn default_nodes() -> Self {
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

    pub(crate) fn ensure_capacity(&mut self, body: usize) {
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
    pub(crate) fn build_range(&mut self, items: &mut [(u32, Aabb)]) -> u32 {
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
    pub(crate) fn make_leaf(&mut self, items: &[(u32, Aabb)]) -> u32 {
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
    pub(crate) fn make_internal(&mut self, kids: &[u32]) -> u32 {
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
    pub(crate) fn split_leaf(&mut self, leaf: u32, extra: u32) -> u32 {
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
    pub(crate) fn split_node(&mut self, node: u32, extra: u32) -> u32 {
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
    pub(crate) fn children_bound(&self, kids: &[u32]) -> (Aabb, u32) {
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
    pub(crate) fn settle_split(&mut self, mut child: u32, mut sib: u32) {
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

    pub(crate) fn push_node(&mut self, node: WideNode) -> u32 {
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
    pub(crate) fn refit_upwards(&mut self, mut node: u32) {
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

    pub(crate) fn check(&self, i: u32, parent_box: Option<Aabb>) {
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
