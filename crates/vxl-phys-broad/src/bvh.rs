//! 动态 BVH（增量 AABB 树，bvh2 / Box2D b2DynamicTree 同族，§2.3 宽相主路径）。
//!
//! - 插入：沿「扩张代价最小」下降选兄弟（面积启发式，比较顺序固定）；
//! - 移动：fat AABB 包含则保持增量，否则删除重插；
//! - 平衡：祖先 refit + 子树旋转（b2Balance 同族）；
//! - 查询：显式栈遍历，先左后右（§5 确定性）；
//! - 上层按体索引升序执行 insert/move_proxy，输出对排序去重。

#![forbid(unsafe_code)]

use vxl_phys_core::Vec3;

use super::Aabb;

const NULL: u32 = u32::MAX;

#[derive(Clone, Copy, Debug)]
struct BvhNode {
    aabb: Aabb,
    left: u32,
    right: u32,
    parent: u32,
    /// 叶子：体 id；内部：NULL。
    body: u32,
    height: u32,
}

impl BvhNode {
    #[inline]
    fn is_leaf(&self) -> bool {
        self.left == NULL
    }
}

/// 增量动态 BVH。
#[derive(Clone, Debug)]
pub struct DynamicBvh {
    nodes: Vec<BvhNode>,
    free: Vec<u32>,
    root: u32,
    /// fat AABB 相对精确 AABB 的外扩（m）。
    pub fat_margin: f32,
}

/// AABB 周长（面积启发式代价；确定性：固定表达式顺序）。
fn perimeter(a: &Aabb) -> f32 {
    let wx = a.max.x - a.min.x;
    let wy = a.max.y - a.min.y;
    let wz = a.max.z - a.min.z;
    2.0 * (wx + wy + wz)
}

fn union_aabb(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

/// AABB 中心在指定轴上的坐标（中位分裂的排序键之一）。
fn center_axis(a: &Aabb, axis: u32) -> f32 {
    match axis {
        0 => (a.min.x + a.max.x) * 0.5,
        1 => (a.min.y + a.max.y) * 0.5,
        _ => (a.min.z + a.max.z) * 0.5,
    }
}

#[inline]
fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    a.min.x <= b.max.x
        && b.min.x <= a.max.x
        && a.min.y <= b.max.y
        && b.min.y <= a.max.y
        && a.min.z <= b.max.z
        && b.min.z <= a.max.z
}

impl DynamicBvh {
    pub fn new(fat_margin: f32) -> Self {
        Self {
            nodes: Vec::new(),
            free: Vec::new(),
            root: NULL,
            fat_margin,
        }
    }

    pub fn clear(&mut self) {
        self.nodes.clear();
        self.free.clear();
        self.root = NULL;
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len() - self.free.len()
    }

    /// 根节点高度（诊断：健康树 ≈ 1.4·log2(n)，链化则 ≈ n/2）。
    pub fn root_height(&self) -> u32 {
        if self.root == NULL {
            0
        } else {
            self.nodes[self.root as usize].height
        }
    }

    fn allocate(&mut self) -> u32 {
        if let Some(i) = self.free.pop() {
            i
        } else {
            self.nodes.push(BvhNode {
                aabb: Aabb {
                    min: Vec3::ZERO,
                    max: Vec3::ZERO,
                },
                left: NULL,
                right: NULL,
                parent: NULL,
                body: NULL,
                height: 0,
            });
            (self.nodes.len() - 1) as u32
        }
    }

    fn free_node(&mut self, i: u32) {
        self.free.push(i);
    }

    #[inline]
    fn fat(&self, a: &Aabb) -> Aabb {
        self.fat_with(a, self.fat_margin)
    }

    #[inline]
    fn fat_with(&self, a: &Aabb, margin: f32) -> Aabb {
        let m = Vec3::splat(margin);
        Aabb {
            min: a.min - m,
            max: a.max + m,
        }
    }

    /// 插入叶子（体 id + 精确 AABB），返回叶子节点索引。
    pub fn insert(&mut self, body: u32, aabb: Aabb) -> u32 {
        self.insert_fat(body, aabb, self.fat_margin)
    }

    /// 插入叶子，fat 边距显式给定（速度自适应边距用；见 `move_proxy_scaled`）。
    pub fn insert_fat(&mut self, body: u32, aabb: Aabb, margin: f32) -> u32 {
        let leaf = self.allocate();
        self.nodes[leaf as usize] = BvhNode {
            aabb: self.fat_with(&aabb, margin),
            left: NULL,
            right: NULL,
            parent: NULL,
            body,
            height: 0,
        };
        if self.root == NULL {
            self.root = leaf;
            return leaf;
        }
        // 1) 找兄弟：沿最小扩张代价下降（确定性：先左后右比较）。
        let leaf_aabb = self.nodes[leaf as usize].aabb;
        let mut cursor = self.root;
        while !self.nodes[cursor as usize].is_leaf() {
            let left = self.nodes[cursor as usize].left;
            let right = self.nodes[cursor as usize].right;
            let combined = perimeter(&union_aabb(&self.nodes[cursor as usize].aabb, &leaf_aabb));
            let descend_cost = 2.0 * (combined - perimeter(&self.nodes[cursor as usize].aabb));
            let left_cost = descend_cost + self.descend_cost(left, &leaf_aabb);
            let right_cost = descend_cost + self.descend_cost(right, &leaf_aabb);
            if left_cost < right_cost {
                cursor = left;
            } else {
                cursor = right;
            }
        }
        // 2) 新父节点，兄弟配对。
        let sibling = cursor;
        let old_parent = self.nodes[sibling as usize].parent;
        let new_parent = self.allocate();
        self.nodes[new_parent as usize] = BvhNode {
            aabb: union_aabb(&leaf_aabb, &self.nodes[sibling as usize].aabb),
            left: sibling,
            right: leaf,
            parent: old_parent,
            body: NULL,
            height: 1,
        };
        self.nodes[sibling as usize].parent = new_parent;
        self.nodes[leaf as usize].parent = new_parent;
        if old_parent == NULL {
            self.root = new_parent;
        } else if self.nodes[old_parent as usize].left == sibling {
            self.nodes[old_parent as usize].left = new_parent;
        } else {
            self.nodes[old_parent as usize].right = new_parent;
        }
        // 3) 祖先 refit + 平衡。
        self.fix_upwards(new_parent);
        leaf
    }

    fn descend_cost(&self, node: u32, leaf_aabb: &Aabb) -> f32 {
        let combined = union_aabb(&self.nodes[node as usize].aabb, leaf_aabb);
        if self.nodes[node as usize].is_leaf() {
            perimeter(&combined)
        } else {
            perimeter(&combined) - perimeter(&self.nodes[node as usize].aabb)
        }
    }

    /// 移除叶子。
    pub fn remove(&mut self, leaf: u32) {
        if leaf == self.root {
            self.root = NULL;
            self.free_node(leaf);
            return;
        }
        let parent = self.nodes[leaf as usize].parent;
        let grand = self.nodes[parent as usize].parent;
        let sibling = if self.nodes[parent as usize].left == leaf {
            self.nodes[parent as usize].right
        } else {
            self.nodes[parent as usize].left
        };
        if grand == NULL {
            self.root = sibling;
            self.nodes[sibling as usize].parent = NULL;
        } else {
            if self.nodes[grand as usize].left == parent {
                self.nodes[grand as usize].left = sibling;
            } else {
                self.nodes[grand as usize].right = sibling;
            }
            self.nodes[sibling as usize].parent = grand;
            self.fix_upwards(grand);
        }
        self.free_node(parent);
        self.free_node(leaf);
    }

    /// 移动代理（固定边距版；见 `move_proxy_scaled`）。
    pub fn move_proxy(&mut self, leaf: u32, aabb: Aabb) -> u32 {
        self.move_proxy_scaled(leaf, aabb, self.fat_margin).0
    }

    /// 移动代理（速度自适应边距版）：叶子已存的 fat 盒包含新精确盒 → 零结构
    /// 操作；逃出旧 fat 盒时首选**就地生长 refit**（T2 树风暴治理：叶盒取
    /// 旧 fat ∪ 新 fat、沿祖先 refit，无节点手术），仅当叶盒过度疏松
    /// （周长 > 4× 目标 fat 周长）才 remove + 以 `margin` 重插收紧。
    /// 返回（可能变化的叶子索引, 盒是否变化——查询缓存失效信号）。
    ///
    /// 动机（M1 宽相提速）：固定 0.02 边距下，下落体每帧位移 > 边距 → 每帧
    /// remove+insert（10 万体规模 = 每帧数万次结构操作，实测树更新 45~70ms）。
    /// 边距随「本帧位移·k」放大后，快速体可在其 fat 盒内连续多帧不动树；
    /// 再叠加就地生长，逃逸路径也基本零手术（实测树 p95 56→? 见 M1-PLAN）。
    /// 确定性：margin 只由（速度, dt）决定，纯函数，时序无关。
    pub fn move_proxy_scaled(&mut self, leaf: u32, aabb: Aabb, margin: f32) -> (u32, bool) {
        let fat = self.nodes[leaf as usize].aabb;
        if fat.contains(&aabb) {
            return (leaf, false);
        }
        let target = aabb.grown(margin);
        let grown = union_aabb(&fat, &target);
        if perimeter(&grown) <= 4.0 * perimeter(&target) {
            self.nodes[leaf as usize].aabb = grown;
            // refit 从叶的**父节点**起步（fix_upwards 假定节点有左右子）。
            let p = self.nodes[leaf as usize].parent;
            self.fix_upwards(p);
            (leaf, true)
        } else {
            let body = self.nodes[leaf as usize].body;
            self.remove(leaf);
            (self.insert_fat(body, aabb, margin), true)
        }
    }

    /// 祖先链 refit + 旋转平衡。
    fn fix_upwards(&mut self, mut node: u32) {
        while node != NULL {
            node = self.balance(node);
            let left = self.nodes[node as usize].left;
            let right = self.nodes[node as usize].right;
            self.nodes[node as usize].aabb = union_aabb(
                &self.nodes[left as usize].aabb,
                &self.nodes[right as usize].aabb,
            );
            self.nodes[node as usize].height = 1 + self.nodes[left as usize]
                .height
                .max(self.nodes[right as usize].height);
            node = self.nodes[node as usize].parent;
        }
    }

    /// 单节点旋转平衡（左右子树高度差 > 1 时），返回子树（新）根。
    fn balance(&mut self, a: u32) -> u32 {
        if self.nodes[a as usize].is_leaf() || self.nodes[a as usize].height < 2 {
            return a;
        }
        let b = self.nodes[a as usize].left;
        let c = self.nodes[a as usize].right;
        let balance = self.nodes[c as usize].height as i32 - self.nodes[b as usize].height as i32;
        if balance > 1 {
            self.rotate_up(a, c);
            return c;
        }
        if balance < -1 {
            self.rotate_up(a, b);
            return b;
        }
        a
    }

    /// 把 child（左右中较深的一支）旋转到 a 之上（b2DynamicTree::Balance 同族）。
    ///
    /// 标准旋转（子树身份不得互换）：
    /// - child 在 a 左槽 → 右旋：child.left 不动；child.right ← a；
    ///   a.left ← child 的原右子树；a 其余子不动。
    /// - child 在 a 右槽 → 左旋：child.right 不动；child.left ← a；
    ///   a.right ← child 的原左子树；a 其余子不动。
    ///
    /// 先按新结构重算 a 的高度/AABB，再算 child（child 高度依赖 a）。
    fn rotate_up(&mut self, a: u32, child: u32) {
        let child_was_left = self.nodes[a as usize].left == child;
        let f = self.nodes[child as usize].left;
        let g = self.nodes[child as usize].right;

        // child 取代 a 的位置。
        self.nodes[child as usize].parent = self.nodes[a as usize].parent;
        self.nodes[a as usize].parent = child;
        if self.nodes[child as usize].parent != NULL {
            let gp = self.nodes[child as usize].parent;
            if self.nodes[gp as usize].left == a {
                self.nodes[gp as usize].left = child;
            } else {
                self.nodes[gp as usize].right = child;
            }
        } else {
            self.root = child;
        }

        if child_was_left {
            // 右旋：g（child 原右子树）让位给 a.left；child.left = f 不动。
            self.nodes[child as usize].right = a;
            self.nodes[a as usize].left = g;
            self.nodes[g as usize].parent = a;
        } else {
            // 左旋：f（child 原左子树）让位给 a.right；child.right = g 不动。
            self.nodes[child as usize].left = a;
            self.nodes[a as usize].right = f;
            self.nodes[f as usize].parent = a;
        }

        // a 高度/AABB（其两个子位已确定）。
        let (al, ar) = (self.nodes[a as usize].left, self.nodes[a as usize].right);
        self.nodes[a as usize].aabb =
            union_aabb(&self.nodes[al as usize].aabb, &self.nodes[ar as usize].aabb);
        self.nodes[a as usize].height = 1 + self.nodes[al as usize]
            .height
            .max(self.nodes[ar as usize].height);
        // child 高度/AABB（含刚更新的 a）。
        let (cl, cr) = (
            self.nodes[child as usize].left,
            self.nodes[child as usize].right,
        );
        self.nodes[child as usize].aabb =
            union_aabb(&self.nodes[cl as usize].aabb, &self.nodes[cr as usize].aabb);
        self.nodes[child as usize].height = 1 + self.nodes[cl as usize]
            .height
            .max(self.nodes[cr as usize].height);
    }

    /// 全量重建（确定性中位数分裂，top-down）。`items` 须按体 id 升序
    /// （0..n），返回每体的叶子节点索引（`move_proxy` 用）。
    ///
    /// 面积启发式对结构化插入序（网格行优先等）会链化（树高 ≈ n/2）；
    /// 按最长轴中位分裂重建可得树高 ≈ log2(n)。分裂用
    /// `select_nth_unstable_by`（O(n) 选择）+ (中心坐标, body id) 全序，
    /// 结果与逐项顺序无关（§5 确定性）。
    pub fn rebuild(&mut self, items: &[(u32, Aabb)]) -> Vec<u32> {
        self.clear();
        let mut leaf_of = vec![NULL; items.len()];
        if items.is_empty() {
            return leaf_of;
        }
        let mut work: Vec<(u32, Aabb)> = items.to_vec();
        self.root = self.build_range(&mut work, &mut leaf_of);
        self.validate();
        leaf_of
    }

    fn build_range(&mut self, items: &mut [(u32, Aabb)], leaf_of: &mut [u32]) -> u32 {
        debug_assert!(!items.is_empty());
        if items.len() == 1 {
            // 直接分配叶子（与 insert 同口径存 fat AABB，保持增量容差）；
            // 重建绝不走 insert——insert 挂到 self.root 并旋转，会与
            // top-down 手工接线互相破坏父指针。
            let (body, aabb) = items[0];
            let leaf = self.allocate();
            self.nodes[leaf as usize] = BvhNode {
                aabb: self.fat(&aabb),
                left: NULL,
                right: NULL,
                parent: NULL,
                body,
                height: 0,
            };
            leaf_of[body as usize] = leaf;
            return leaf;
        }
        // 包围盒 → 最长轴。
        let mut bound = items[0].1;
        for (_, a) in items.iter().skip(1) {
            bound = union_aabb(&bound, a);
        }
        let ex = bound.max.x - bound.min.x;
        let ey = bound.max.y - bound.min.y;
        let ez = bound.max.z - bound.min.z;
        let axis = if ex >= ey && ex >= ez {
            0
        } else if ey >= ez {
            1
        } else {
            2
        };
        let mid = items.len() / 2;
        // 全序：(轴中心, body id)；f32 全序用 total_cmp。
        items.select_nth_unstable_by(mid, |a, b| {
            let ca = center_axis(&a.1, axis);
            let cb = center_axis(&b.1, axis);
            ca.total_cmp(&cb).then(a.0.cmp(&b.0))
        });
        let (left_items, right_items) = items.split_at_mut(mid);
        // 中位分裂后左右两侧的 body id 不连续：leaf_of 全量传入，按 body id 寻址。
        let left = self.build_range(left_items, leaf_of);
        let right = self.build_range(right_items, leaf_of);
        // 汇合父节点。
        let parent = self.allocate();
        let aabb = union_aabb(
            &self.nodes[left as usize].aabb,
            &self.nodes[right as usize].aabb,
        );
        let height = 1 + self.nodes[left as usize]
            .height
            .max(self.nodes[right as usize].height);
        self.nodes[parent as usize] = BvhNode {
            aabb,
            left,
            right,
            parent: NULL,
            body: NULL,
            height,
        };
        self.nodes[left as usize].parent = parent;
        self.nodes[right as usize].parent = parent;
        parent
    }

    /// 查询：返回 fat AABB 与给定 AABB 相交的所有叶子体 id（升序，去重）。
    pub fn query(&self, aabb: &Aabb, out: &mut Vec<u32>) {
        out.clear();
        if self.root == NULL {
            return;
        }
        let mut stack: Vec<u32> = Vec::with_capacity(64);
        stack.push(self.root);
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i as usize];
            if !overlaps(&node.aabb, aabb) {
                continue;
            }
            if node.is_leaf() {
                out.push(node.body);
            } else {
                // 先右后左入栈 → 出栈先左后右（确定性遍历序）。
                stack.push(node.right);
                stack.push(node.left);
            }
        }
        out.sort_unstable();
        out.dedup();
    }

    /// 结构校验（测试用）：父指针 / 高度 / 包围盒一致性 + 无环。
    pub fn validate(&self) {
        if self.root == NULL {
            return;
        }
        let mut stack = vec![(self.root, NULL)];
        while let Some((i, parent)) = stack.pop() {
            let node = &self.nodes[i as usize];
            assert_eq!(node.parent, parent, "父指针损坏 @ {i}");
            if node.is_leaf() {
                assert_eq!(node.height, 0, "叶子高度非零 @ {i}");
                assert_ne!(node.body, NULL, "叶子缺体 id @ {i}");
            } else {
                let h = 1 + self.nodes[node.left as usize]
                    .height
                    .max(self.nodes[node.right as usize].height);
                assert_eq!(node.height, h, "高度不一致 @ {i}");
                let expect = union_aabb(
                    &self.nodes[node.left as usize].aabb,
                    &self.nodes[node.right as usize].aabb,
                );
                assert_eq!(node.aabb, expect, "包围盒不一致 @ {i}");
                stack.push((node.left, i));
                stack.push((node.right, i));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb_at(x: f32, y: f32, half: f32) -> Aabb {
        let c = Vec3::new(x, y, 0.0);
        Aabb {
            min: c - Vec3::splat(half),
            max: c + Vec3::splat(half),
        }
    }

    /// 确定性伪随机布局（无外部 RNG）。
    fn layout(n: u32) -> Vec<(u32, Aabb)> {
        (0..n)
            .map(|k| {
                let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let y = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let half = 0.3 + ((k.wrapping_mul(97)) % 100) as f32 / 1000.0;
                (k, aabb_at(x, y, half))
            })
            .collect()
    }

    fn build(items: &[(u32, Aabb)]) -> (DynamicBvh, Vec<u32>) {
        let mut tree = DynamicBvh::new(0.02);
        let mut leaves = vec![NULL; items.len()];
        for &(body, aabb) in items {
            leaves[body as usize] = tree.insert(body, aabb);
        }
        tree.validate();
        (tree, leaves)
    }

    #[test]
    fn query_matches_brute_force() {
        let items = layout(200);
        let (tree, _) = build(&items);
        let fat = |a: &Aabb| {
            let m = Vec3::splat(0.02);
            Aabb {
                min: a.min - m,
                max: a.max + m,
            }
        };
        for probe in 0..20 {
            let (bx, by, half) = {
                let (_, a) = &items[probe * 7 % items.len()];
                (
                    (a.min.x + a.max.x) * 0.5,
                    (a.min.y + a.max.y) * 0.5,
                    (a.max.x - a.min.x) * 0.5,
                )
            };
            let q = aabb_at(bx, by, half + 2.0);
            let mut got = Vec::new();
            tree.query(&q, &mut got);
            // 树内存的是 fat AABB（+0.02），暴力对照按同样口径。
            let mut want: Vec<u32> = items
                .iter()
                .filter(|(_, a)| overlaps(&fat(a), &q))
                .map(|(i, _)| *i)
                .collect();
            want.sort_unstable();
            assert_eq!(got, want, "probe {probe}");
        }
    }

    #[test]
    fn incremental_moves_match_brute_force() {
        let items = layout(150);
        let (mut tree, mut leaves) = build(&items);
        // 增量移动：每体平移固定向量，move_proxy 与「全量重建」结果一致。
        for round in 1..=5u32 {
            let shift = round as f32 * 0.5;
            for &(body, aabb) in items.iter() {
                let moved = Aabb {
                    min: aabb.min + Vec3::new(shift, -shift * 0.5, 0.0),
                    max: aabb.max + Vec3::new(shift, -shift * 0.5, 0.0),
                };
                leaves[body as usize] = tree.move_proxy(leaves[body as usize], moved);
            }
            tree.validate();
            let moved_items: Vec<(u32, Aabb)> = items
                .iter()
                .map(|&(body, aabb)| {
                    (
                        body,
                        Aabb {
                            min: aabb.min + Vec3::new(shift, -shift * 0.5, 0.0),
                            max: aabb.max + Vec3::new(shift, -shift * 0.5, 0.0),
                        },
                    )
                })
                .collect();
            let mut got = Vec::new();
            tree.query(
                &Aabb {
                    min: Vec3::splat(-100.0),
                    max: Vec3::splat(100.0),
                },
                &mut got,
            );
            let mut want: Vec<u32> = moved_items.iter().map(|(i, _)| *i).collect();
            want.sort_unstable();
            assert_eq!(got, want, "round {round}");
        }
    }

    #[test]
    fn remove_and_reinsert_keeps_structure() {
        let items = layout(80);
        let (mut tree, leaves) = build(&items);
        for &leaf in &leaves[..40] {
            tree.remove(leaf);
        }
        tree.validate();
        let remaining: Vec<(u32, Aabb)> = items.iter().skip(40).cloned().collect();
        let mut got = Vec::new();
        tree.query(
            &Aabb {
                min: Vec3::splat(-100.0),
                max: Vec3::splat(100.0),
            },
            &mut got,
        );
        let mut want: Vec<u32> = remaining.iter().map(|(i, _)| *i).collect();
        want.sort_unstable();
        assert_eq!(got, want);
        // 重插被删的。
        for &(body, aabb) in items.iter().take(40) {
            tree.insert(body, aabb);
        }
        tree.validate();
        tree.query(
            &Aabb {
                min: Vec3::splat(-100.0),
                max: Vec3::splat(100.0),
            },
            &mut got,
        );
        let mut want: Vec<u32> = items.iter().map(|(i, _)| *i).collect();
        want.sort_unstable();
        assert_eq!(got, want);
    }

    #[test]
    fn dense_layout_query_parity() {
        // 密集布局：大量真实重叠对，全域查询 = 全体（配对级验证由 BvhBroadPhase 测试覆盖）。
        let items: Vec<(u32, Aabb)> = (0..40)
            .map(|k| {
                let x = (k % 6) as f32 * 0.5;
                let y = (k / 6) as f32 * 0.5;
                (k, aabb_at(x, y, 0.35))
            })
            .collect();
        let (tree, _) = build(&items);
        let mut got = Vec::new();
        tree.query(
            &Aabb {
                min: Vec3::splat(-100.0),
                max: Vec3::splat(100.0),
            },
            &mut got,
        );
        let mut want: Vec<u32> = items.iter().map(|(i, _)| *i).collect();
        want.sort_unstable();
        assert_eq!(got, want);
        // 局部查询命中左上角簇。
        tree.query(&aabb_at(0.0, 0.0, 0.6), &mut got);
        assert!(got.contains(&0) && got.contains(&1) && got.contains(&6));
    }
}
