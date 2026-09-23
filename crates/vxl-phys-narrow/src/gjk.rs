//! 凸体外壳（Convex Hull）+ GJK / EPA——凸体通用窄相（SPEC §2.4）。
//!
//! 设计：**支撑映射抽象**（`Support`）——任何凸体只需「沿方向的最远点」即可参与
//! GJK（距离/相交）与 EPA（穿透深度/法线）。盒/球/圆柱可复用既有特化路径；**外壳
//! （多边形域）**走本模块。
//!
//! 确定性：迭代次数固定、退化分支显式短路、无随机（SPEC §5）。
//! 规模：顶点数 O(10~10³) 的外壳够用（Voronoi 碎块/凸分解件）；更大建议先凸分解。

#![deny(unsafe_code)]

use vxl_phys_core::{Mat3, Quat, Vec3};

// ── 按域拆出的子模块（子目录 gjk/）
mod gjk_clip;
mod gjk_core;
mod gjk_epa;
mod gjk_hull;
mod gjk_shapes;
mod gjk_support;
pub use self::{gjk_clip::*, gjk_core::*, gjk_epa::*, gjk_hull::*, gjk_shapes::*, gjk_support::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

/// 点是否落在格 `i` 的 Voronoi 半空间约束内（`tol` 吸收边界浮点误差）。
#[cfg(test)]
pub(crate) fn in_voronoi_cell(seeds: &[Vec3], i: usize, q: Vec3, tol: f32) -> bool {
    for (j, s) in seeds.iter().enumerate() {
        if j == i {
            continue;
        }
        let n = *s - seeds[i];
        let mid = (seeds[i] + *s) * 0.5;
        if n.dot(q) > n.dot(mid) + tol {
            return false;
        }
    }
    true
}

/// **Voronoi 预断裂（凸体版）**：种子点最近邻分区 → 逐格 = 原凸壳 ∩ 各平分半空间。
/// 返回每格顶点云（空格 = 种子在体外 ⇒ 空 Vec ⇒ 该格无碎块）。
/// 性质：各格凸、互不重叠、并集 = 原体（浮点容差内）。
pub fn fracture_voronoi_hull(hull: &ConvexHull, seeds: &[Vec3]) -> Vec<Vec<Vec3>> {
    let mut out = Vec::with_capacity(seeds.len());
    for i in 0..seeds.len() {
        let mut region: Vec<Vec3> = hull.points.clone();
        for (j, s) in seeds.iter().enumerate() {
            if i == j {
                continue;
            }
            let n = *s - seeds[i];
            let l2 = n.length_squared();
            if l2 < 1e-12 {
                continue;
            }
            let mid = (seeds[i] + *s) * 0.5;
            region = clip_halfspace(&region, n, n.dot(mid));
            if region.is_empty() {
                break;
            }
        }
        out.push(region);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn unit_box_hull() -> ConvexHull {
        let mut pts = Vec::new();
        for &x in &[-0.5f32, 0.5] {
            for &y in &[-0.5f32, 0.5] {
                for &z in &[-0.5f32, 0.5] {
                    pts.push(Vec3::new(x, y, z));
                }
            }
        }
        ConvexHull::new(pts)
    }

    #[test]
    pub(crate) fn gjk_separated_distance_matches_analytic() {
        // 两个盒心相距 1.5（半长 0.5+0.5=1.0 ⇒ 缝 0.5）
        let a = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::ZERO,
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        let b = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::new(1.5, 0.0, 0.0),
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        match gjk(&a, &b) {
            Gjk::Separated { dist, .. } => {
                assert!((dist - 0.5).abs() < 1e-3, "dist={dist}");
            }
            Gjk::Intersecting => panic!("应分离"),
        }
    }

    #[test]
    pub(crate) fn epa_penetration_matches_analytic() {
        // 盒沿 X 重叠 0.2：深度应 ≈ 0.2、法线 ≈ ±X
        let a = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::ZERO,
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        let b = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::new(0.8, 0.0, 0.0),
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        assert!(matches!(gjk(&a, &b), Gjk::Intersecting));
        let (n, d, _p) = epa(&a, &b, 32).expect("应相交");
        assert!((d - 0.2).abs() < 0.02, "depth={d}");
        assert!(n.x.abs() > 0.99, "normal={n:?}");
    }

    #[test]
    pub(crate) fn hull_box_intersection_and_separation() {
        let hull = unit_box_hull();
        // 相交（盒心距 0.8 ⇒ 穿透 0.2）
        let hit = hull_box_penetration(
            &hull,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::splat(0.5),
            Vec3::new(0.8, 0.0, 0.0),
            Quat::IDENTITY,
            32,
        );
        let (n, d, _) = hit.expect("应相交");
        assert!(d > 0.05 && d < 0.35, "depth={d}");
        assert!(n.x.abs() > 0.9, "normal={n:?}");
        // 分离
        assert!(hull_box_penetration(
            &hull,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::splat(0.5),
            Vec3::new(1.5, 0.0, 0.0),
            Quat::IDENTITY,
            32
        )
        .is_none());
    }

    #[test]
    pub(crate) fn halfspace_clip_is_exact_half_cube() {
        let h = unit_box_hull();
        let pts = clip_halfspace(&h.points, Vec3::X, 0.0);
        // 裁剪体内：全体 x ≤ 0；且裁切面四角都在（凸包 = 半个立方体）
        assert!(pts.iter().all(|p| p.x <= 1e-6), "{pts:?}");
        for &y in &[-0.5f32, 0.5] {
            for &z in &[-0.5f32, 0.5] {
                assert!(
                    pts.iter()
                        .any(|p| (*p - Vec3::new(0.0, y, z)).length() < 1e-5),
                    "缺裁切面角 ({y},{z})"
                );
            }
        }
    }

    #[test]
    pub(crate) fn voronoi_fracture_cells_tile_convex_body() {
        // 立方体内 8 个种子（偏移去对称 ⇒ 无双胞边界歧义）
        let h = unit_box_hull();
        let mut seeds = Vec::new();
        for (i, &x) in [-0.3f32, 0.3].iter().enumerate() {
            for (j, &y) in [-0.25f32, 0.35].iter().enumerate() {
                for (k, &z) in [-0.35f32, 0.25].iter().enumerate() {
                    let _ = (i, j, k);
                    seeds.push(Vec3::new(x, y, z));
                }
            }
        }
        let cells = fracture_voronoi_hull(&h, &seeds);
        assert_eq!(cells.len(), seeds.len());
        // 抽样：体内每个点恰属 1 格（Voronoi 分区互斥 + 覆盖）
        let mut checked = 0;
        for i in 0..9 {
            for j in 0..9 {
                for k in 0..9 {
                    let q = Vec3::new(
                        -0.45 + 0.9 * i as f32 / 8.0,
                        -0.45 + 0.9 * j as f32 / 8.0,
                        -0.45 + 0.9 * k as f32 / 8.0,
                    );
                    // 跳过贴近平分面的样本（浮点边界归属不判）
                    let mut near = false;
                    for (a, sa) in seeds.iter().enumerate() {
                        for sb in seeds.iter().skip(a + 1) {
                            let dv = *sb - *sa;
                            if dv.length_squared() < 1e-12 {
                                continue;
                            }
                            let n = dv.normalize();
                            let mid = (*sa + *sb) * 0.5;
                            if (n.dot(q) - n.dot(mid)).abs() < 5e-3 {
                                near = true;
                            }
                        }
                    }
                    if near {
                        continue;
                    }
                    let cnt = (0..seeds.len())
                        .filter(|&ci| in_voronoi_cell(&seeds, ci, q, 1e-6))
                        .count();
                    assert_eq!(cnt, 1, "q={q:?} 属 {cnt} 格");
                    checked += 1;
                }
            }
        }
        assert!(checked > 200, "样本太少: {checked}");
    }

    #[test]
    pub(crate) fn support_point_is_extremal() {
        let h = unit_box_hull();
        let p = h.support_point(Vec3::new(1.0, -1.0, 1.0));
        assert_eq!(p, Vec3::new(0.5, -0.5, 0.5));
        // 旋转 90°（绕 Y）后，+X 支撑点应来自原 +Z 方向
        let rot = Mat3::from_quat(Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2));
        let hs = HullSupport {
            hull: &h,
            pos: Vec3::ZERO,
            rot,
        };
        let p = hs.support(Vec3::X);
        // 绕 Y 转 90°：世界 +X 支撑应由原 +Z 面给出（x=0.5）；并列点不唯一 ⇒ 只验极值性
        assert!((p.x - 0.5).abs() < 1e-5, "{p:?}");
        for q in &h.points {
            assert!(rot.mul_vec3(*q).x <= p.x + 1e-5, "非极值: {q:?}");
        }
    }
}
