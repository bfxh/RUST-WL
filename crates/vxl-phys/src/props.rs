//! props：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// **复合体的精确质量属性**（关于**复合体原点**，与本仓"体原点即质心"的表示一致）：
/// 按子形状求和 + 平行轴。
///
/// `M = Σ mᵢ`；`I = Σ [Rᵢ·diag(Iᵢ)·Rᵢᵀ + mᵢ(|oᵢ|²E − oᵢoᵢᵀ)]`，旋转项只取对角
/// （`(R I Rᵀ)_kk = Σ_a R[k][a]²·I_a`，本仓惯量表示就是逐个 `Vec3` 对角）。
///
/// ⚠️ 两条取舍（见 `TECH-SURVEY.md` A9 ④）：① 平行轴用**原点**而非真质心 ⇒ 质心偏移的力矩
/// 效应不建模（与锥同款；本仓无质心偏移字段）；② **离对角项被丢弃**（对称/轴对齐的复合体
/// 本来就为零）。空复合体或零质量返回 `None`（调用方保留兜底近似）。
pub(crate) fn compound_mass_props(children: &[CompoundChild], density: f32) -> Option<(f32, Vec3)> {
    if children.is_empty() {
        return None;
    }
    let mut m_total = 0.0f32;
    let mut i_diag = Vec3::ZERO;
    for ch in children {
        let mp = vxl_phys_core::mass_props(&ch.shape, density);
        let m = mp.mass;
        if m <= 0.0 || !m.is_finite() {
            continue;
        }
        m_total += m;
        let i_own = Vec3::new(
            1.0 / mp.local_inv_inertia.x,
            1.0 / mp.local_inv_inertia.y,
            1.0 / mp.local_inv_inertia.z,
        );
        let r = vxl_phys_core::Mat3::from_quat(ch.rot);
        let rot_diag = Vec3::new(
            r.m[0][0] * r.m[0][0] * i_own.x
                + r.m[0][1] * r.m[0][1] * i_own.y
                + r.m[0][2] * r.m[0][2] * i_own.z,
            r.m[1][0] * r.m[1][0] * i_own.x
                + r.m[1][1] * r.m[1][1] * i_own.y
                + r.m[1][2] * r.m[1][2] * i_own.z,
            r.m[2][0] * r.m[2][0] * i_own.x
                + r.m[2][1] * r.m[2][1] * i_own.y
                + r.m[2][2] * r.m[2][2] * i_own.z,
        );
        let o = ch.offset;
        let o2 = o.length_squared();
        let par = Vec3::new(o2 - o.x * o.x, o2 - o.y * o.y, o2 - o.z * o.z) * m;
        i_diag = i_diag + rot_diag + par;
    }
    if m_total <= 0.0 || !m_total.is_finite() {
        return None;
    }
    let inv = Vec3::new(
        if i_diag.x > 0.0 { 1.0 / i_diag.x } else { 0.0 },
        if i_diag.y > 0.0 { 1.0 / i_diag.y } else { 0.0 },
        if i_diag.z > 0.0 { 1.0 / i_diag.z } else { 0.0 },
    );
    Some((1.0 / m_total, inv))
}

/// 形状的平均迎风面积估计（阻力用）：盒 = 三对面面积均值，球 = πr²，
/// 其余（含外壳）取局部 AABB 近似；不可估计返回 0（不施加阻力）。
pub(crate) fn cross_section_area(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half } => 4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0,
        Shape::Sphere { radius } => std::f32::consts::PI * radius * radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => 2.0 * radius * (2.0 * half_height) / 2.0 + std::f32::consts::PI * radius * radius,
        // 胶囊：中段按圆柱（含帽时略低估，阻力估计够用）。
        Shape::Capsule {
            half_height,
            radius,
        } => 2.0 * radius * (2.0 * half_height) / 2.0 + std::f32::consts::PI * radius * radius,
        // 锥：侧面投影按三角剖面（底宽 2r、高 2h ⇒ 面积 r·h）另加底圆盘；
        // 锥尖一端无面 ⇒ 比同尺寸圆柱略低估（阻力估计够用）。
        Shape::Cone {
            half_height,
            radius,
        } => radius * half_height + std::f32::consts::PI * radius * radius,
        Shape::ConvexHull { half, .. } => {
            4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0
        }
        // 复合体：同外壳（局部 AABB 并集半长的外接盒近似；阻力估计够用）。
        Shape::Compound { half, .. } => {
            4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0
        }
        Shape::HeightField(_) | Shape::Provider(_) => 0.0,
    }
}

/// 形状的体积（浮力用）：**以单位密度反解质量**（`mass_props(shape, 1.0)` 的质量即体积），
/// 与质量/惯量的形状实现**同源** ⇒ 不会出现"浮力按 A 体积、质量按 B 体积"的错配。
/// 地形/提供者体（无体积语义）返回 0（不施加浮力）。
pub(crate) fn shape_volume(shape: &Shape) -> f32 {
    match shape {
        Shape::HeightField(_) | Shape::Provider(_) => 0.0,
        _ => vxl_phys_core::mass_props(shape, 1.0).mass,
    }
}

/// 形状的包围半径（浮力的**表面采样点**用；与 `cross_section_area` 同族近似）。
/// 半球/锥等取局部 AABB 的最大半长 ⇒ 采样点落在体表附近即可（不做精确解析）。
pub(crate) fn body_half_extent(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half } | Shape::ConvexHull { half, .. } | Shape::Compound { half, .. } => {
            half.x.max(half.y).max(half.z)
        }
        Shape::Sphere { radius } => radius,
        Shape::Cylinder {
            half_height,
            radius,
        }
        | Shape::Capsule {
            half_height,
            radius,
        }
        | Shape::Cone {
            half_height,
            radius,
        } => (half_height * half_height + radius * radius).sqrt(),
        Shape::HeightField(_) | Shape::Provider(_) => 0.0,
    }
}

/// 流体**粒子**（不含 2b 边界粒子）的 AABB；无粒子 ⇒ `None`。
/// 介质耦合（2a）与 2b 边界粒子生成的**共用预滤**：每 tick 一次 O(n)，
/// 体先过包围盒，避免全库逐体采样。
pub(crate) fn particle_bounds(sys: &vxl_phys_fluid::FluidSystem) -> Option<(Vec3, Vec3)> {
    let pos = sys.positions();
    let first = *pos.first()?;
    let (mut lo, mut hi) = (first, first);
    for p in &pos[1..] {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    Some((lo, hi))
}
