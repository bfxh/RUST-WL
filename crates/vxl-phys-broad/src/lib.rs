//! # vxl-phys-broad
//!
//! 宽相（§2.3）：「增量 AABB 树（bvh2 风格）为主 + 大世界空间哈希网格」。
//! M1 落地 `BvhBroadPhase`（增量动态 BVH，`bvh::DynamicBvh`）为主实现；
//! M0 的 `GridBroadPhase`（均匀空间哈希网格）保留作对照/大世界叠加。
//! `wide::WideBvh`（8 路宽节点）为已验收但**未接入**的实验模块：树更新更快
//! （树峰 7.34→3.10ms）但叶粒度令候选数 8×、查询更慢（均 7.47→14.68ms）
//! ⇒ 净负，见 `BvhBroadPhase` 结论注与 docs/M1-PLAN.md。
//!
//! 确定性（§5）：树插入/移动按体索引升序；查询显式栈、先左后右；
//! 输出对 `(a, b)`（a < b）按字典序排序去重。两者绝不迭代哈希结构本身。

#![forbid(unsafe_code)]

pub mod bvh;
/// BVH8（宽**内部**节点 + 窄叶）——T2 尾数据布局**第二版原型**：先量「6 层窄叶
/// 遍历是否真比 18 层二叉便宜」再决定投不投增量侧（见模块头注）。
pub mod bvh8;
/// 8 路宽节点 BVH（T2 尾数据布局投入；**实验模块，未接入生产路径**——
/// 接入实测净负，见 `BvhBroadPhase` 结论注）。
pub mod wide;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, Quat, Shape, Vec3};

pub use bvh::DynamicBvh;
pub use bvh8::Bvh8;
pub use wide::WideBvh;

/// 轴对齐包围盒（定义已下移至 `vxl-phys-core::Aabb`，此处再导出保持既有路径）。
pub use vxl_phys_core::Aabb;

// ── 按域拆出的子模块（子目录 src/）
mod bvh_phase;
mod grid_phase;
mod shape;
mod trait_phase;
pub use self::{bvh_phase::*, grid_phase::*, shape::*, trait_phase::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::SerialJobSystem;

    pub(crate) fn world() -> BodySet {
        BodySet::new()
    }

    /// BVH 与网格宽相在同场景下输出完全一致（确定性交叉验证）。
    #[test]
    pub(crate) fn bvh_matches_grid_across_frames() {
        let build = || {
            let mut b = world();
            // 地面静态瓦片 5×5。
            for gx in -2i32..=2 {
                for gz in -2i32..=2 {
                    b.push_static(
                        Shape::Box {
                            half: Vec3::new(0.5, 0.25, 0.5),
                        },
                        Vec3::new(gx as f32, -0.25, gz as f32),
                        Quat::IDENTITY,
                    );
                }
            }
            // 动体：确定性伪随机散布。
            for k in 0..40u32 {
                let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let z = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let y = 0.5 + ((k.wrapping_mul(97)) % 400) as f32 / 100.0;
                let shape = if k % 2 == 0 {
                    Shape::Sphere { radius: 0.3 }
                } else {
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    }
                };
                b.push_dynamic(shape, Vec3::new(x, y, z), Quat::IDENTITY, 1.0);
            }
            b
        };
        let mut grid = GridBroadPhase::new(2.0, 0.01);
        let mut bvh = BvhBroadPhase::new(0.01);
        for frame in 0..5u32 {
            let mut b = build();
            // 每帧给动体一个确定性位移（模拟增量移动）。
            for i in 0..b.len() {
                if b.is_dynamic(i) {
                    b.position[i].x += frame as f32 * 0.17;
                    b.position[i].y -= frame as f32 * 0.09;
                }
            }
            let p_grid = grid.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
            let p_bvh = bvh.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
            assert_eq!(p_grid, p_bvh, "frame {frame}");
            let _ = &mut b;
        }
    }

    /// 既有配对漏洞回归（T2 第十四段修复）：睡眠体不做查询，旧「dyn-dyn 只由
    /// 较大索引侧发射」规则会漏掉「清醒大索引体 vs 睡眠小索引体」——双侧发射
    /// 后必须检出（否则清醒体可穿过沉睡体）。
    #[test]
    pub(crate) fn awake_larger_pairs_with_sleeping_smaller() {
        let mut b = world();
        let small = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
        );
        let big = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        // 小索引体入睡（大索引体保持清醒）。
        b.awake[small as usize] = false;
        let mut bp = BvhBroadPhase::new(0.01);
        let pairs = bp.compute_pairs(&b, &[], &[], &SerialJobSystem);
        assert_eq!(pairs, &[(small.min(big), small.max(big))]);
    }

    /// T2 查询缓存**完备性**随机化不变式：多帧移动（含高速逃逸/生长/重插、
    /// 混合睡眠）下，BVH 输出对必须与暴力枚举（同源 stored_aabb 精确重叠）
    /// 逐一相等。缓存复用若漏对（错失新接近体）此测试立即红。
    #[test]
    pub(crate) fn bvh_pairs_match_brute_force_across_frames() {
        let mut b = world();
        for gx in -3i32..=3 {
            for gz in -3i32..=3 {
                b.push_static(
                    Shape::Box {
                        half: Vec3::new(0.5, 0.25, 0.5),
                    },
                    Vec3::new(gx as f32, -0.25, gz as f32),
                    Quat::IDENTITY,
                );
            }
        }
        for k in 0..160u32 {
            let x = ((k.wrapping_mul(2_654_435_761)) % 1000) as f32 / 1000.0 * 6.0 - 3.0;
            let z = ((k.wrapping_mul(40_503)) % 1000) as f32 / 1000.0 * 6.0 - 3.0;
            let y = 0.6 + ((k.wrapping_mul(97)) % 300) as f32 / 100.0;
            b.push_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.3),
                },
                Vec3::new(x, y, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        let mut bp = BvhBroadPhase::new(0.01);
        for frame in 0..60u32 {
            // 确定性移动：奇数体快（触发逃逸/生长/重插），偶数体慢（缓存命中）；
            // 每 3 帧让 1/5 体入睡（清醒-睡眠 混合路径）。
            for i in 0..b.len() {
                if !b.is_dynamic(i) {
                    continue;
                }
                let f = frame as f32;
                let s = if i % 2 == 0 { 0.01 } else { 0.12 };
                b.position[i].x += ((i as f32 * 0.37 + f * 0.11).sin()) * s;
                b.position[i].z += ((i as f32 * 0.53 + f * 0.17).cos()) * s;
                b.position[i].y -= if i % 2 == 0 { 0.002 } else { 0.02 };
                b.awake[i] = !(frame % 3 == 0 && i % 5 == 0);
            }
            let got = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
            // 暴力参照：i<j 精确 AABB 重叠（与宽相同规则——**至少一侧为
            // 「动态且清醒」**：沉睡体不查询、静-静不产对 ⇒ 静×睡与睡×睡
            // 均无对；清醒×睡/清醒×静由清醒侧查询命中）。
            let n = b.len();
            let mut want: Vec<(u32, u32)> = Vec::new();
            let sa: Vec<Aabb> = (0..n).map(|i| bp.stored_aabb(i).unwrap()).collect();
            for i in 0..n as u32 {
                for j in (i + 1)..n as u32 {
                    let (iu, ju) = (i as usize, j as usize);
                    let pi = b.is_dynamic(iu) && b.awake[iu];
                    let pj = b.is_dynamic(ju) && b.awake[ju];
                    if !pi && !pj {
                        continue;
                    }
                    if sa[iu].overlaps(&sa[ju]) {
                        want.push((i, j));
                    }
                }
            }
            want.sort_unstable();
            want.dedup();
            assert_eq!(got, want, "frame {frame}");
        }
    }

    #[test]
    pub(crate) fn overlapping_pair_found_once() {
        let mut b = world();
        let g = b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let d = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let far = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(50.0, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let pairs = bp.compute_pairs(&b, &[], &[], &SerialJobSystem);
        assert_eq!(pairs, &[(d.min(g), d.max(g))]);
        assert!(pairs.iter().all(|&p| p.1 != far && p.0 != far));
    }

    #[test]
    pub(crate) fn no_static_static_pairs() {
        let mut b = world();
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.1, 0.0, 0.0),
            Quat::IDENTITY,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        assert!(bp.compute_pairs(&b, &[], &[], &SerialJobSystem).is_empty());
    }

    #[test]
    pub(crate) fn deterministic_output() {
        let mut b = world();
        for k in 0..50 {
            let x = (k % 10) as f32 * 1.1;
            let z = (k / 10) as f32 * 1.1;
            b.push_dynamic(
                Shape::Sphere { radius: 0.5 },
                Vec3::new(x, 1.0, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let p1 = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
        let p2 = bp.compute_pairs(&b, &[], &[], &SerialJobSystem).to_vec();
        assert_eq!(p1, p2);
    }
}
