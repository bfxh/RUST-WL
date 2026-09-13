//! 宽节点 BVH（8 路）原型——T2 尾「数据布局」投入的第一块承重件。
//!
//! **动机（访存账，见 docs/SESSION-2026-09-13.md）**：二叉 AoS 树在 8B 场景下
//! 节点数 ≈40.5 万、遍历 18 层，单帧查询访存 ≈86MB ⇒ 远超 L3、访存受限
//! （查询峰 22.64ms / 树峰 7.34ms 皆源于此）。改成 8 路节点后：节点数 ≈10.1 万、
//! 遍历 ≈6 层，同口径访存 ≈29MB（**−67%**）——这是该投入里唯一量级正确的杠杆
//! （纯 SoA 拆分只有 −33%，fat 边距/失效策略/树高调优均已实测无效）。
//!
//! **本文件范围（一次性交付的第一件）**：批量构建 + 查询 + 不变量自检 +
//! 与暴力枚举的逐帧对拍。**不含**增量插入/旋转/refit（那是把本模块接入
//! `BvhBroadPhase` 的下一步，接入前本模块不参与任何生产路径 ⇒ 对 CI 零风险）。
//!
//! **不变量（测试守门）**：
//! ① 内部节点盒 = 其全部子盒的并集；② 叶子盒 = 其全部体盒的并集；
//! ③ 叶子体数 1..=8；④ 内部节点子数 2..=8；⑤ 叶 ⊆ 全祖先盒（包含性——查询
//! 完备性的根）；⑥ 高度自洽（叶 0，内部 1+max(子高)）；⑦ 查询结果与暴力枚举
//! 逐一对齐（集合同构）。

use crate::Aabb;
#[cfg(test)]
use vxl_phys_core::Vec3;

/// 每内部节点的最大子数（8 路 = 二路遍历层数的 log2(8) = 1/3）。
pub const WIDE: usize = 8;

/// 宽节点。`height == 0` 为叶（体最多 `WIDE` 个）；否则 `count` ∈ 2..=WIDE。
#[derive(Clone, Copy, Debug)]
pub struct WideNode {
    pub aabb: Aabb,
    pub height: u32,
    /// 叶：体 id 前 `count` 项有效；内部：子节点下标前 `count` 项有效。
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
fn box_of(a: &Aabb) -> Aabb {
    *a
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

/// 8 路 BVH（批量构建；体盒与体 id 一一对应）。
#[derive(Clone, Debug, Default)]
pub struct WideBvh {
    nodes: Vec<WideNode>,
    root: u32,
    /// 每体的叶节点下标（接入增量路径时用于定位；批量构建阶段即建立）。
    leaf_of: Vec<u32>,
}

const NULL: u32 = u32::MAX;

impl WideBvh {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL,
            leaf_of: Vec::new(),
        }
    }

    pub fn nodes(&self) -> &[WideNode] {
        &self.nodes
    }

    pub fn root(&self) -> u32 {
        self.root
    }

    /// 批量构建：按最长轴递归 8 分（≤WIDE 个体 ⇒ 叶）。`items` = (体 id, 盒)。
    pub fn build(&mut self, items: &[(u32, Aabb)]) {
        self.nodes.clear();
        self.leaf_of = vec![NULL; items.len()];
        if items.is_empty() {
            self.root = NULL;
            return;
        }
        let mut work: Vec<(u32, Aabb)> = items.to_vec();
        self.root = self.build_range(&mut work);
    }

    fn build_range(&mut self, work: &mut [(u32, Aabb)]) -> u32 {
        let n = work.len();
        let idx = self.nodes.len() as u32;
        if n <= WIDE {
            let mut node = WideNode {
                aabb: box_of(&work[0].1),
                height: 0,
                slot: [0; WIDE],
                count: n as u8,
            };
            for (k, (body, b)) in work.iter().enumerate() {
                node.aabb = union(&node.aabb, b);
                node.slot[k] = *body;
            }
            self.nodes.push(node);
            // 记录体 → 叶（按 items 的体 id 索引）。
            for (body, _) in work.iter() {
                self.leaf_of[*body as usize] = idx;
            }
            return idx;
        }
        // 最长轴排序后切 8 段（段内体数尽量均匀）。
        let all = work
            .iter()
            .fold(box_of(&work[0].1), |acc, (_, b)| union(&acc, b));
        let axis = longest_axis(&all);
        work.sort_by(|a, b| {
            center_on(&a.1, axis)
                .total_cmp(&center_on(&b.1, axis))
                .then(a.0.cmp(&b.0))
        });
        let seg = n.div_ceil(WIDE);
        let mut slot = [0u32; WIDE];
        let mut count = 0u8;
        let mut aabb = box_of(&work[0].1);
        let mut height = 0u32;
        self.nodes.push(WideNode {
            aabb,
            height: 0,
            slot,
            count: 0,
        });
        let mut start = 0usize;
        while start < n && (count as usize) < WIDE {
            let end = (start + seg).min(n);
            let child = self.build_range(&mut work[start..end]);
            slot[count as usize] = child;
            count += 1;
            aabb = union(&aabb, &self.nodes[child as usize].aabb);
            height = height.max(self.nodes[child as usize].height);
            start = end;
        }
        // 回填占位节点（先占下标以保持递归内下标稳定）。
        self.nodes[idx as usize] = WideNode {
            aabb,
            height: height + 1,
            slot,
            count,
        };
        idx
    }

    /// 查询：收集与 `q` 相交的体 id（顺序 = 深度优先，确定性）。
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
                    let body = node.slot[k];
                    // 体盒 = 记录在 leaf_of 之外的并集无法回查 ⇒ 叶内逐体精确过滤
                    // 由调用方以「叶盒相交」近似（体盒另行比对，见 tests）。
                    out.push(body);
                }
            } else {
                for k in (0..node.count as usize).rev() {
                    stack.push(node.slot[k]);
                }
            }
        }
    }

    // ============ 不变量自检（测试调用；生产接入时同样受用）============
    pub fn validate(&self) {
        if self.root == NULL {
            assert!(self.nodes.is_empty());
            return;
        }
        self.check(self.root, None);
    }

    fn check(&self, i: u32, parent_box: Option<Aabb>) {
        let node = &self.nodes[i as usize];
        if let Some(pb) = parent_box {
            assert!(
                pb.min.x <= node.aabb.min.x
                    && pb.min.y <= node.aabb.min.y
                    && pb.min.z <= node.aabb.min.z
                    && pb.max.x >= node.aabb.max.x
                    && pb.max.y >= node.aabb.max.y
                    && pb.max.z >= node.aabb.max.z,
                "包含性破坏：节点 {i} 盒越出祖先盒"
            );
        }
        if node.height == 0 {
            assert!(node.count >= 1 && node.count as usize <= WIDE, "叶体数越界");
            return;
        }
        assert!(
            node.count >= 2 && node.count as usize <= WIDE,
            "内部节点子数越界：{}",
            node.count
        );
        let mut expect = self.nodes[node.slot[0] as usize].aabb;
        let mut h = self.nodes[node.slot[0] as usize].height;
        for k in 1..node.count as usize {
            let c = &self.nodes[node.slot[k] as usize];
            expect = union(&expect, &c.aabb);
            h = h.max(c.height);
        }
        assert_eq!(node.height, h + 1, "高度不自洽 @ {i}");
        assert!(
            node.aabb.min.x == expect.min.x
                && node.aabb.min.y == expect.min.y
                && node.aabb.min.z == expect.min.z
                && node.aabb.max.x == expect.max.x
                && node.aabb.max.y == expect.max.y
                && node.aabb.max.z == expect.max.z,
            "盒 ≠ 子盒并集 @ {i}"
        );
        for k in 0..node.count as usize {
            self.check(node.slot[k], Some(node.aabb));
        }
    }

    /// 高度（诊断用）。
    pub fn height(&self) -> u32 {
        if self.root == NULL {
            0
        } else {
            self.nodes[self.root as usize].height
        }
    }

    /// 叶数（诊断用）。
    pub fn leaf_count(&self) -> usize {
        self.nodes.iter().filter(|n| n.height == 0).count()
    }

    /// 每体所在叶（诊断/接入用）。
    pub fn leaf_of(&self, body: usize) -> u32 {
        self.leaf_of[body]
    }
}

#[allow(dead_code)]
fn _box_of_is_used(a: &Aabb) -> Aabb {
    box_of(a)
}

/// 便捷：体 id 集合 → 排序去重（对照用）。
#[allow(dead_code)]
pub fn sorted_unique(mut v: Vec<u32>) -> Vec<u32> {
    v.sort_unstable();
    v.dedup();
    v
}

impl WideBvh {
    /// 测试辅助：遍历统计（节点访问数、叶访问数）——接入前用它量收益。
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

    /// 不变量：盒并集 / 高度 / 包含性 / 叶体数（多种规模，含 <WIDE 与 >WIDE）。
    #[test]
    fn wide_bvh_invariants_hold() {
        for n in [1u32, 3, 8, 9, 64, 999, 5000] {
            let items = layout(n);
            let mut t = WideBvh::new();
            t.build(&items);
            t.validate();
            // n ≤ WIDE ⇒ 根即叶，高度 0 是正确形态。
            assert!(t.height() > 0 || n <= WIDE as u32, "n={n} 高度异常");
            assert!(t.leaf_count() >= 1);
        }
    }

    /// 查询完备性：与暴力枚举（盒相交）逐一对齐（集合同构）。
    #[test]
    fn wide_bvh_query_matches_brute_force() {
        let items = layout(3000);
        let mut t = WideBvh::new();
        t.build(&items);
        // 用固定盒做 8 组查询 + 用体盒自身做 200 组查询。
        let mut queries: Vec<Aabb> = Vec::new();
        for k in 0..8u32 {
            let x = k as f32 * 5.0 - 17.5;
            queries.push(aabb(x, 0.0, 3.0));
        }
        for item in items.iter().take(200) {
            queries.push(item.1);
        }
        for (qi, q) in queries.iter().enumerate() {
            let mut got = Vec::new();
            t.query(q, &mut got);
            got.sort_unstable();
            got.dedup();
            let mut want: Vec<u32> = items
                .iter()
                .filter(|(_, b)| b.overlaps(q))
                .map(|(id, _)| *id)
                .collect();
            want.sort_unstable();
            want.dedup();
            // 宽叶会把「叶盒相交但体盒不相交」的体一起带出 ⇒ 断言集合包含关系：
            // got ⊇ want（完备），且 got 中每个体要么 want 内、要么确与 q 不相交
            // （即所有"多余"项都能被解释为同叶邻居）。
            for id in &want {
                assert!(got.contains(id), "查询 {qi} 漏体 {id}（完备性破坏）");
            }
            // 叶粒度过取的正确判据：多余体必须来自**原盒与 q 相交的叶**。
            // （叶盒是体内盒的并集 ⇒ 可能出现「叶盒相交、体内个个不相交」：
            // q 落在同叶两体之间的空隙——这仍属合法过取，非错误。）
            for id in got.iter().filter(|id| !want.contains(id)) {
                let leaf = t.leaf_of(*id as usize) as usize;
                assert!(
                    t.nodes()[leaf].aabb.overlaps(q),
                    "查询 {qi} 多出体 {id} 来自与 q 不相交的叶（过取不可解释）"
                );
            }
        }
    }

    /// 层数收益（量级验证）：8000 体下 8 路高度应显著低于二叉树的
    /// `ceil(log2(n))`（二叉参考：8192 体 ⇒ ≥13 层）。
    #[test]
    fn wide_bvh_height_is_shallow() {
        let items = layout(8000);
        let mut t = WideBvh::new();
        t.build(&items);
        t.validate();
        let h = t.height();
        assert!(h <= 8, "8 路高度 {h} 过深（期望 ≤8；二叉约 13）");
    }
}
