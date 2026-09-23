//! shape：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 形状 → 世界 AABB（含 margin 膨胀）。
/// 高度场体的包围盒由调用方经 `hf_bounds[id]` 提供（高度场数据在 narrow/terrain 层）。
pub fn shape_aabb(
    shape: &Shape,
    pos: Vec3,
    rot: Quat,
    margin: f32,
    hf_bounds: &[Aabb],
    provider_bounds: &[Aabb],
) -> Aabb {
    let half = match *shape {
        Shape::Box { half } => {
            // 精确：|R|·half（旋转矩阵逐元素绝对值作用于半长）。
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][0].abs() * half.x + r.m[0][1].abs() * half.y + r.m[0][2].abs() * half.z,
                r.m[1][0].abs() * half.x + r.m[1][1].abs() * half.y + r.m[1][2].abs() * half.z,
                r.m[2][0].abs() * half.x + r.m[2][1].abs() * half.y + r.m[2][2].abs() * half.z,
            )
        }
        Shape::Sphere { radius } => Vec3::splat(radius),
        // 胶囊体：**精确** AABB —— 线段（局部 ±Y·h）的 |R| 投影 ⊕ radius。
        Shape::Capsule {
            half_height,
            radius,
        } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][1].abs() * half_height + radius,
                r.m[1][1].abs() * half_height + radius,
                r.m[2][1].abs() * half_height + radius,
            )
        }
        // 圆锥：顶点 ∪ 底圆盘（凸包 ⇒ 取两者极值即**精确**）。顶点贡献 |R[i][1]·h|，
        // 底圆盘再外扩 r·√(R[i][0]² + R[i][2]²)（旋转矩阵行 i 单位长）。
        Shape::Cone {
            half_height,
            radius,
        } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][1].abs() * half_height
                    + radius * (r.m[0][0] * r.m[0][0] + r.m[0][2] * r.m[0][2]).sqrt(),
                r.m[1][1].abs() * half_height
                    + radius * (r.m[1][0] * r.m[1][0] + r.m[1][2] * r.m[1][2]).sqrt(),
                r.m[2][1].abs() * half_height
                    + radius * (r.m[2][0] * r.m[2][0] + r.m[2][2] * r.m[2][2]).sqrt(),
            )
        }
        // 复合体：同外壳（局部 AABB 并集半长 + |R| 变换保守）。
        Shape::Compound { half, .. } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][0].abs() * half.x + r.m[0][1].abs() * half.y + r.m[0][2].abs() * half.z,
                r.m[1][0].abs() * half.x + r.m[1][1].abs() * half.y + r.m[1][2].abs() * half.z,
                r.m[2][0].abs() * half.x + r.m[2][1].abs() * half.y + r.m[2][2].abs() * half.z,
            )
        }
        // 凸体外壳：局部 AABB 半长（宽相只需保守界）。
        Shape::ConvexHull { half, .. } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][0].abs() * half.x + r.m[0][1].abs() * half.y + r.m[0][2].abs() * half.z,
                r.m[1][0].abs() * half.x + r.m[1][1].abs() * half.y + r.m[1][2].abs() * half.z,
                r.m[2][0].abs() * half.x + r.m[2][1].abs() * half.y + r.m[2][2].abs() * half.z,
            )
        }
        // 圆柱 ⊆ 外接盒（r, hh, r），经 |R| 变换保守。
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            let ext = Vec3::new(radius, half_height, radius);
            Vec3::new(
                r.m[0][0].abs() * ext.x + r.m[0][1].abs() * ext.y + r.m[0][2].abs() * ext.z,
                r.m[1][0].abs() * ext.x + r.m[1][1].abs() * ext.y + r.m[1][2].abs() * ext.z,
                r.m[2][0].abs() * ext.x + r.m[2][1].abs() * ext.y + r.m[2][2].abs() * ext.z,
            )
        }
        Shape::HeightField(id) => {
            let b = hf_bounds.get(id as usize).copied().unwrap_or(Aabb {
                min: Vec3::splat(0.0),
                max: Vec3::splat(0.0),
            });
            return Aabb {
                min: b.min - Vec3::splat(margin),
                max: b.max + Vec3::splat(margin),
            };
        }
        // 外部碰撞提供者体（体素/网格…）：AABB 由提供者给（静态 Marker 语义）。
        Shape::Provider(id) => {
            let b = provider_bounds.get(id as usize).copied().unwrap_or(Aabb {
                min: Vec3::splat(0.0),
                max: Vec3::splat(0.0),
            });
            return Aabb {
                min: b.min - Vec3::splat(margin),
                max: b.max + Vec3::splat(margin),
            };
        }
    };
    Aabb {
        min: pos - half - Vec3::splat(margin),
        max: pos + half + Vec3::splat(margin),
    }
}
