//! prims：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl DefaultNarrowPhase {
    /// 球-凸体：球心 center、半径 radius；凸体形状 convex（世界变换 cpos/crot）。
    /// 返回 (球→凸体方向法线, 深度, 接触点)。
    pub(crate) fn sphere_convex_ab(
        &mut self,
        center: Vec3,
        radius: f32,
        convex: &Shape,
        cpos: Vec3,
        crot: Quat,
    ) -> Option<(Vec3, f32, Vec3)> {
        let idx = self.poly_for(convex)?;
        self.poly_b.fill(&self.polys[idx], cpos, crot);
        let (closest, d2, inside, in_n) = closest_point_on_poly(&self.poly_b, center);
        if inside {
            // 球心在凸体内部：max_plane_d < 0，表面距 = -max_plane_d。
            let max_d = max_plane_d_of(&self.poly_b, center);
            let depth = radius + max_d;
            if depth <= 0.0 {
                return None;
            }
            let surface = center - in_n * max_d;
            Some((-in_n, depth, surface))
        } else {
            let dist = d2.sqrt();
            if dist >= radius {
                return None;
            }
            let n = if dist > 1e-9 {
                (closest - center) * (1.0 / dist)
            } else {
                Vec3::Y
            };
            Some((n, radius - dist, closest))
        }
    }

    /// **胶囊中心线 ↔ 凸体**：中心线取最近点后复用球的解析路径（`sphere_convex_ab`）。
    /// 返回 `(中心线最近点, 对方表面点)`；无接触返回 `None`。
    ///
    /// 为什么不用 GJK/EPA（`EXPERIMENTS.md` R.1/R.2 两次实测）：EPA 对**光滑**支撑不适定
    /// （临界接触给 45° 假法线 + 42 m 假深度）；"收成芯再跑 GJK"又踩 GJK 的**面平局**退化
    /// （盒顶面对 ±Y 时支撑解到角点 + 无进展提前退出，实测 `d=7.077 / p_other=(5,0,5)`）。
    /// 解析最近点两条坑都不碰。
    ///
    /// 采样点为什么就是段上最近点：点到**凸**集的距离沿线段是凸函数 ⇒ 迭代
    /// `q ← 体上最近点(p)`、`p ← 段上最近点(q)` 的驻点即全局最近点对，且距离单调不增。
    pub(crate) fn capsule_axis_reach(
        &mut self,
        s0: Vec3,
        s1: Vec3,
        radius: f32,
        other: &Shape,
        opos: Vec3,
        orot: Quat,
    ) -> Option<(Vec3, Vec3)> {
        let idx = self.poly_for(other)?;
        self.poly_b.fill(&self.polys[idx], opos, orot);
        let seg = s1 - s0;
        let seg_len2 = seg.length_squared();
        let mut p = (s0 + s1) * 0.5;
        for _ in 0..4 {
            let (q, _d2, inside, in_n) = closest_point_on_poly(&self.poly_b, p);
            // 内部时同样把 q 拉到"朝最近面"的表面上，迭代方向才有效。
            let q = if inside {
                p + in_n * -max_plane_d_of(&self.poly_b, p)
            } else {
                q
            };
            let t = if seg_len2 > 1e-18 {
                ((q - s0).dot(seg) / seg_len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let p_new = s0 + seg * t;
            let done = (p_new - p).length_squared() <= 1e-12;
            p = p_new;
            if done {
                break;
            }
        }
        // 采样点交给球的解析路径（其内部会再填一次 `poly_b`，成本可接受）。
        let (_n, _depth, surf) = self.sphere_convex_ab(p, radius, other, opos, orot)?;
        Some((p, surf))
    }

    /// **胶囊体 × 凸体：GJK 距离**。返回 `(n_cap→other, dist, p_cap, p_other)`；
    /// 线段与对方重叠（`Intersecting`）时改用 EPA 求穿透法线（返回 `dist = -depth`）。
    ///
    /// 为什么不能用 SAT/裁剪：胶囊是**光滑**外形，SAT 需要有限面集。为什么不能直接用
    /// EPA：EPA 只在**重叠**时可用，而"胶囊接触"的定义是**线段到对方的距离 < radius**
    /// （此时线段本身可能完全没碰到对方）。GJK 距离恰好给出这个量。
    pub(crate) fn capsule_reach(
        cap: &gjk::CapsuleSupport,
        other: &dyn gjk::Support,
    ) -> Option<(Vec3, f32, Vec3, Vec3)> {
        match gjk::gjk(cap, other) {
            gjk::Gjk::Separated {
                point_a,
                point_b,
                dist,
            } => {
                let n = if dist > 1e-9 {
                    (point_b - point_a) * (1.0 / dist)
                } else {
                    Vec3::Y
                };
                Some((n, dist, point_a, point_b))
            }
            gjk::Gjk::Intersecting => {
                let (n, depth, p) = gjk::epa(cap, other, 32)?;
                Some((n, -depth, p, p))
            }
        }
    }

    /// 接触点统一选点：按深度降序 + 空间去重（min_sep 内视为同一点）+ 截断 ≤4。
    /// 展平多面体的同一顶点会以浮点噪声级差异重复出现，不去重会造成
    /// 单角多倍冲量 → 永续 rocking（能量泵）。
    pub(crate) fn select_contacts(&mut self, min_sep: f32) -> bool {
        if self.cand.is_empty() {
            return false;
        }
        self.cand.sort_by(|x, y| {
            y.depth
                .total_cmp(&x.depth)
                .then(x.point.x.total_cmp(&y.point.x))
                .then(x.point.y.total_cmp(&y.point.y))
                .then(x.point.z.total_cmp(&y.point.z))
        });
        let min2 = min_sep * min_sep;
        // 复用 scratch（T3：旧实现每调用一次 Vec::with_capacity(4) —— 8B 场景
        // ~22 万次/帧的堆分配）。语义逐位不变：按深度序取 ≤4 个非重复点。
        let mut kept = std::mem::take(&mut self.kept_buf);
        kept.clear();
        for &c in self.cand.iter() {
            if kept.len() >= 4 {
                break;
            }
            let dup = kept
                .iter()
                .any(|k| (k.point - c.point).length_squared() < min2);
            if !dup {
                kept.push(c);
            }
        }
        self.cand.clear();
        self.cand.extend_from_slice(&kept);
        self.kept_buf = kept;
        !self.cand.is_empty()
    }
}
