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

// ── 按域拆出的子模块（子目录 bvh/）
mod bvh_impl;
mod bvh_types;
pub use self::bvh_types::*;
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn aabb_at(x: f32, y: f32, half: f32) -> Aabb {
        let c = Vec3::new(x, y, 0.0);
        Aabb {
            min: c - Vec3::splat(half),
            max: c + Vec3::splat(half),
        }
    }

    /// 确定性伪随机布局（无外部 RNG）。
    pub(crate) fn layout(n: u32) -> Vec<(u32, Aabb)> {
        (0..n)
            .map(|k| {
                let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let y = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
                let half = 0.3 + ((k.wrapping_mul(97)) % 100) as f32 / 1000.0;
                (k, aabb_at(x, y, half))
            })
            .collect()
    }

    /// ===== 最小复现组（T2 尾教训）：把树不变量压缩到 2-3 节点面，钉住
    /// 「结构改动前必须先看最小面」这条纪律。四块断言 = 高度公式 / 盒并集 /
    /// 父指针 / 包含性（叶子盒 ⊆ 祖先盒）。=====
    pub(crate) fn assert_tree_invariants(t: &DynamicBvh) {
        t.validate(); // 结构测试同源断言（高度/盒/父指针/叶子高度）
                      // 包含性：每个叶子的盒必须被其全部祖先盒包含（查询完备性的根）。
        for leaf in 0..t.nodes.len() as u32 {
            if !t.nodes[leaf as usize].is_leaf() {
                continue;
            }
            let box0 = t.nodes[leaf as usize].aabb;
            let mut cur = t.nodes[leaf as usize].parent;
            while cur != NULL {
                let ancestor_box = t.nodes[cur as usize].aabb;
                assert!(
                    ancestor_box.min.x <= box0.min.x
                        && ancestor_box.min.y <= box0.min.y
                        && ancestor_box.max.x >= box0.max.x
                        && ancestor_box.max.y >= box0.max.y,
                    "祖先盒未包含叶子：leaf {leaf} 祖先 {cur}"
                );
                cur = t.nodes[cur as usize].parent;
            }
        }
    }

    /// 两节点树：生长一个叶子后，父盒必须扩张且包含该叶子。
    #[test]
    pub(crate) fn min_tree_two_nodes_growth_refits_parent() {
        let mut t = DynamicBvh::new(0.02);
        let a = t.insert(0, aabb_at(0.0, 0.0, 0.5));
        let b = t.insert(1, aabb_at(3.0, 0.0, 0.5));
        assert_tree_invariants(&t);
        // 叶子 a 原地生长（远小于 4× 周长比 ⇒ 走就地生长分支）。
        let bigger = aabb_at(0.6, 0.0, 0.6);
        let (nl, changed) = t.move_proxy_scaled(a, bigger, 0.02);
        assert!(changed, "盒变化必须报 changed");
        assert_tree_invariants(&t);
        let _ = (b, nl);
    }

    /// 三节点：顺序插入形成链 ⇒ 触发旋转；旋转后不变量必须自洽。
    #[test]
    pub(crate) fn min_tree_three_nodes_rotation_keeps_invariants() {
        let mut t = DynamicBvh::new(0.02);
        let mut leaves = [NULL; 3];
        for k in 0..3u32 {
            leaves[k as usize] = t.insert(k, aabb_at(k as f32 * 10.0, 0.0, 0.5));
            assert_tree_invariants(&t);
        }
        // 逐叶位移（会走 fix_upwards ⇒ 可能触发 balance 旋转）。
        for k in 0..3u32 {
            let (_, changed) = t.move_proxy_scaled(
                leaves[k as usize],
                aabb_at(k as f32 * 10.0 + 2.0, 1.0, 0.5),
                0.02,
            );
            assert!(changed);
            assert_tree_invariants(&t);
        }
    }

    pub(crate) fn build(items: &[(u32, Aabb)]) -> (DynamicBvh, Vec<u32>) {
        let mut tree = DynamicBvh::new(0.02);
        let mut leaves = vec![NULL; items.len()];
        for &(body, aabb) in items {
            leaves[body as usize] = tree.insert(body, aabb);
        }
        tree.validate();
        (tree, leaves)
    }

    #[test]
    pub(crate) fn query_matches_brute_force() {
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
    pub(crate) fn incremental_moves_match_brute_force() {
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
    pub(crate) fn remove_and_reinsert_keeps_structure() {
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
    pub(crate) fn dense_layout_query_parity() {
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
