//! **半空间平面集提供者**（调研档 `SURVEY-SOFT-CLOTH-AND-CONVERSION.md` 的 T3-②）：
//! 把"任意凸体"表示成一组**外向半空间** `n_i·x ≤ d_i` 的交，用它当 [`ProviderColliders`]。
//!
//! **为什么值得**（外部对标结论）：SPH 粒子 / 布料节点要跟任意刚体碰撞时，"**点 vs 平面集**"比"采样
//! 体素 SDF"便宜得多（球/胶囊/凸包都能离散成几十个平面），而本仓的 provider 通道
//! （`Shape::Provider(id)` + `ProviderColliders`，ADR 0009）**已经开好** ⇒ **零架构改动**即可接入
//! ——液体边界投影、外壳顶点采样、刚体接触三类消费者都会走到它。
//! 参考思路来源：外部 demo `shyhdm/Opengl_Physx` 的 `FlowRigidColliders.h`（球/胶囊 → 7×16 平面 + 端盖）。
//! ⚠️ 该仓**无 LICENSE** ⇒ **只借思路，不抄代码**。
//!
//! **口径**（与 `TriMesh`/`VoxelVolume` 逐条对齐，见 `vxl_phys_core::interop::ProviderColliders`）：
//! - `depth` **正 = 穿透**：`depth = −sd`，`sd = max_i (n_i·x − d_i)`（凸体带符号距离——外侧取最外的面、
//!   内侧取最近的面 ⇒ 两种情形都对）；
//! - 法线 = **提供者表面外向法线**（= 决定性的那个平面的 `n_i`）；
//! - 带内判据 `depth ≥ −skin`（speculative margin：面上方 `skin` 内仍产"预期接触"，与高度场/三角网同口径）；
//! - `contacts_point_boundary` 走**默认实现**（trait 文档原话："解析面/半空间提供者无内点歧义"⇒ 原样转发）。
//!
//! ⚠️ **已知近似（写清不藏）**：
//! - `contacts_sphere` 按**单平面**投影（角部多面同时活跃时取最深那个，不做角点联合求解）；
//! - `contacts_box` 用**支撑角**（每个平面取盒沿 `−n_i` 的最深角）⇒ **面接触精确**，边/角接触按平面松弛。
//!   两处与 `TriMesh` 的"最近点/逐顶点"同档。
//! - 法线**须为单位向量**（构造时按 `1/|n|` 归一；`|n|` 非有限或为零的平面被拒收）。

use vxl_phys_broad::Aabb;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::Vec3;

/// 一组外向半空间 `n_i·x ≤ d_i` 的交（凸体）+ 宽相用的世界 AABB。
#[derive(Clone, Debug)]
pub struct HalfSpaceSet {
    planes: Vec<(Vec3, f32)>,
    bound: Aabb,
}

impl HalfSpaceSet {
    /// 空集：`bound` 由调用方给（空平面集 ⇒ 任何查询都不产接触，但 `contacts_point` 仍报"支持"）。
    pub fn new(bound: Aabb) -> Self {
        Self {
            planes: Vec::new(),
            bound,
        }
    }

    /// 追加一个平面（`n` = 外向法线，`offset` = 该面上一点满足 `n·x = offset`）。
    /// `n` 非有限/零长时**静默拒收**（返回 `false`）——宁可少一个面，也不要 NaN 传进求解器。
    pub fn push(&mut self, n: Vec3, offset: f32) -> bool {
        let len = n.length();
        if !len.is_finite() || len <= f32::MIN_POSITIVE {
            return false;
        }
        let inv = 1.0 / len;
        self.planes.push((n * inv, offset * inv));
        true
    }

    pub fn len(&self) -> usize {
        self.planes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.planes.is_empty()
    }

    /// 带符号距离（正 = 在外）与**决定性平面**下标（取 `max_i`）。
    fn signed(&self, x: Vec3) -> (f32, usize) {
        let mut best = f32::NEG_INFINITY;
        let mut k = 0usize;
        for (i, (n, d)) in self.planes.iter().enumerate() {
            let s = n.dot(x) - *d;
            if s > best {
                best = s;
                k = i;
            }
        }
        (best, k)
    }

    /// 把点投影到第 `k` 个平面上（= 提供者表面上的点，与 `TriMesh` 报的 `q` 同义）。
    fn project(&self, x: Vec3, k: usize) -> Vec3 {
        let (n, d) = self.planes[k];
        x - n * (n.dot(x) - d)
    }

    /// 盒在世界系下的 8 个角（`rot` 为单位四元数）。
    fn corners(half: Vec3, pos: Vec3, rot: vxl_phys_core::Quat) -> [Vec3; 8] {
        let mut out = [pos; 8];
        let mut i = 0usize;
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    let local = Vec3::new(sx * half.x, sy * half.y, sz * half.z);
                    out[i] = pos + rot.rotate_vec3(local);
                    i += 1;
                }
            }
        }
        out
    }
}

impl ProviderColliders for HalfSpaceSet {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(self.bound)
    }

    /// 「盒 vs 平面集」：每个平面取**沿 `−n` 最深的那个角**当接触点（面接触精确）。
    fn contacts_box(
        &self,
        _id: u32,
        half: Vec3,
        pos: Vec3,
        rot: vxl_phys_core::Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        if self.planes.is_empty() {
            return false;
        }
        let cs = Self::corners(half, pos, rot);
        // 每个平面：盒沿该法线的**最外角值** ⇒ 判「盒是否落在该半空间内」（O(平面数) 预计算）。
        let outer: Vec<f32> = self
            .planes
            .iter()
            .map(|(n, _)| {
                cs.iter()
                    .map(|c| n.dot(*c))
                    .fold(f32::NEG_INFINITY, f32::max)
            })
            .collect();
        let mut produced = false;
        for (pi, (n, d)) in self.planes.iter().enumerate() {
            // ⚠️ **面接触的有效性判据**：盒必须**落在其余所有半空间之内**（= 它正是从这一面接触实体的）。
            // 少了这条，凸体**侧壁的无限延展**会给出离谱的穿透——实测：盒悬在立方体顶面上方（底在
            // y=0.9、顶面在 y=1），却对「x ≥ −1」那一面报出 **1.5** 的穿透（盒整个在该半空间深处）。
            // 落地：把「盒没落在其内」的平面收集起来，只允许是 `pi` 自己。
            let not_inside: Vec<usize> = (0..self.planes.len())
                .filter(|j| outer[*j] > self.planes[*j].1)
                .collect();
            if not_inside != [pi] {
                continue;
            }
            let mut best = f32::INFINITY;
            let mut deepest = pos;
            for c in &cs {
                let s = n.dot(*c) - *d;
                if s < best {
                    best = s;
                    deepest = *c;
                }
            }
            let depth = -best;
            if depth < -skin {
                continue;
            }
            out.push(InteropContact {
                point: deepest - *n * best, // 落到提供者表面上
                normal: *n,
                depth,
                feature: pi as u32 + 1, // 0 = 无特征 ⇒ 平面序号从 1 起，跨帧稳定
            });
            produced = true;
        }
        produced
    }

    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        if self.planes.is_empty() {
            return true; // 支持查询（空集只是不产接触）
        }
        let (sd, k) = self.signed(p);
        let depth = -sd;
        if depth < -skin {
            return true;
        }
        out.push(InteropContact {
            point: self.project(p, k),
            normal: self.planes[k].0,
            depth,
            feature: k as u32 + 1,
        });
        true
    }

    /// 「球 vs 平面集」：逐平面 `depth = r − s_i`（单平面投影；角部按最深那个松弛）。
    fn contacts_sphere(
        &self,
        _id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        if self.planes.is_empty() {
            return false;
        }
        // 只报**最深的那个平面**（球对凸体的接触是单面/角点的，逐面全报会重复施加冲量）。
        let (sd, k) = self.signed(center);
        let depth = radius - sd;
        if depth < -skin {
            return false;
        }
        let (n, _) = self.planes[k];
        out.push(InteropContact {
            point: center - n * sd, // 提供者表面上的点
            normal: n,
            depth,
            feature: k as u32 + 1,
        });
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::Quat;

    /// 单位立方体（half = 1）在原点：6 个外向平面 `n·x ≤ 1`。
    fn unit_cube() -> HalfSpaceSet {
        let mut s = HalfSpaceSet::new(Aabb {
            min: Vec3::new(-1.0, -1.0, -1.0),
            max: Vec3::new(1.0, 1.0, 1.0),
        });
        for (n, d) in [
            (Vec3::new(1.0, 0.0, 0.0), 1.0),
            (Vec3::new(-1.0, 0.0, 0.0), 1.0),
            (Vec3::new(0.0, 1.0, 0.0), 1.0),
            (Vec3::new(0.0, -1.0, 0.0), 1.0),
            (Vec3::new(0.0, 0.0, 1.0), 1.0),
            (Vec3::new(0.0, 0.0, -1.0), 1.0),
        ] {
            assert!(s.push(n, d));
        }
        s
    }

    #[test]
    fn bounds_is_the_given_aabb() {
        let s = unit_cube();
        let b = s.bounds(7).expect("provider 已注册");
        assert_eq!((b.min.x, b.max.x), (-1.0, 1.0));
    }

    #[test]
    fn normal_and_offset_are_normalized_on_push() {
        let mut s = HalfSpaceSet::new(Aabb {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
        });
        assert!(
            s.push(Vec3::new(0.0, 3.0, 0.0), 3.0),
            "非单位法线应被归一后收下"
        );
        let (n, d) = s.planes[0];
        assert!((n.y - 1.0).abs() < 1e-6 && (d - 1.0).abs() < 1e-6);
        assert!(!s.push(Vec3::ZERO, 0.0), "零长法线应被拒收");
    }

    /// **口径钉子（外侧）**：面上方 `d` 处 ⇒ `depth = −d`（预期接触，**不是**穿透）。
    #[test]
    fn outside_point_gives_negative_depth_and_outward_normal() {
        let s = unit_cube();
        let skill = 0.01f32;
        let mut out = Vec::new();
        // 顶面上方 0.5（超出 skin ⇒ 不产接触，但仍报"支持查询"）
        assert!(s.contacts_point(0, Vec3::new(0.0, 1.5, 0.0), skill, &mut out));
        assert!(out.is_empty(), "超出 skin 不该产接触");
        // 顶面上方 0.005（带内 ⇒ 产"预期接触"）
        let _ = s.contacts_point(0, Vec3::new(0.0, 1.005, 0.0), skill, &mut out);
        let c = out[out.len() - 1];
        assert!(
            (c.depth + 0.005).abs() < 1e-5,
            "depth 应为 −0.005（实得 {}）",
            c.depth
        );
        assert!((c.normal.y - 1.0).abs() < 1e-6, "法线应为外向 +y");
        assert!((c.point.y - 1.0).abs() < 1e-5, "接触点应落在提供者表面上");
    }

    /// **口径钉子（内侧）**：体内离最近面 `t` 处 ⇒ `depth = +t`（穿透为正）+ 该面的外向法线。
    #[test]
    fn inside_point_gives_positive_depth_of_nearest_face() {
        let s = unit_cube();
        let mut out = Vec::new();
        // (0, 0, 0.9)：到 +z 面 0.1、到别的面 ≥0.9 ⇒ depth 应为 +0.1、法线 +z
        let _ = s.contacts_point(0, Vec3::new(0.0, 0.0, 0.9), 0.01, &mut out);
        let c = out[out.len() - 1];
        assert!(
            (c.depth - 0.1).abs() < 1e-5,
            "depth 应为 +0.1（实得 {}）",
            c.depth
        );
        assert!((c.normal.z - 1.0).abs() < 1e-6 && c.normal.x.abs() < 1e-6);
        assert!(c.point.z < c.depth + 1.0, "接触点应在 +z 面上");
    }

    /// 球：`depth = r − s`（`s` = 球心到面的带符号距离）；超出 `skin` 不产接触。
    #[test]
    fn sphere_depth_is_radius_minus_plane_distance() {
        let s = unit_cube();
        let mut out = Vec::new();
        // 球心在顶面上方 0.3、半径 0.5 ⇒ 穿透 0.2
        assert!(s.contacts_sphere(0, Vec3::new(0.0, 1.3, 0.0), 0.5, 0.01, &mut out));
        let c = out[out.len() - 1];
        assert!(
            (c.depth - 0.2).abs() < 1e-5,
            "depth 应为 0.2（实得 {}）",
            c.depth
        );
        assert!((c.normal.y - 1.0).abs() < 1e-6);
        // 球心在顶面上方 0.6、半径 0.5 ⇒ 间隙 0.1，超出 skin ⇒ 不产接触
        let n0 = out.len();
        let _ = s.contacts_sphere(0, Vec3::new(0.0, 1.6, 0.0), 0.5, 0.01, &mut out);
        assert_eq!(out.len(), n0, "间隙 > skin 时不该产接触");
    }

    /// 盒：半高 0.5 的盒底贴到顶面（盒心在 `1.0 + 0.5 + 穿透`）⇒ 面接触 **精确**。
    #[test]
    fn box_face_contact_is_exact() {
        let s = unit_cube();
        let mut out = Vec::new();
        let half = Vec3::new(0.5, 0.5, 0.5);
        // 盒心 y = 1.4 ⇒ 盒底在 0.9（比顶面低 0.1 ⇒ 穿透 0.1）
        assert!(s.contacts_box(
            0,
            half,
            Vec3::new(0.0, 1.4, 0.0),
            Quat::IDENTITY,
            0.01,
            &mut out
        ));
        // 有效性判据要把它压到**恰好一条**（顶面）——否则凸体侧壁也会报接触。
        assert_eq!(
            out.len(),
            1,
            "盒悬在顶面上方 ⇒ 只应对顶面产接触（实得 {} 条）",
            out.len()
        );
        let c = out[0];
        assert!(
            (c.depth - 0.1).abs() < 1e-5,
            "depth 应为 0.1（实得 {}）",
            c.depth
        );
        assert!((c.normal.y - 1.0).abs() < 1e-6, "法线应为外向 +y");
        assert!((c.point.y - 1.0).abs() < 1e-5, "接触点应在顶面上");
    }

    /// 空集：不产接触、但点查询报"支持"（调用方靠这个决定是否整体放弃）。
    #[test]
    fn empty_set_produces_nothing_but_supports_point_queries() {
        let s = HalfSpaceSet::new(Aabb {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
        });
        let mut out = Vec::new();
        assert!(s.contacts_point(0, Vec3::ZERO, 0.01, &mut out));
        assert!(out.is_empty());
        assert!(!s.contacts_box(
            0,
            Vec3::splat(1.0),
            Vec3::ZERO,
            Quat::IDENTITY,
            0.01,
            &mut out
        ));
    }
}
