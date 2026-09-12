//! 碰撞形状定义（§2.1）。
//!
//! 圆柱在碰撞层用固定 `CYLINDER_SEGMENTS` 边内接棱柱近似（§2.4 凸体路径统一），
//! 质量属性仍按解析圆柱计算（§2.1）。

/// 高度场句柄（数据本体在 `vxl-phys-narrow::heightfield`，账本在 `vxl-phys-terrain`）。
pub type HeightFieldId = u32;

/// 圆柱碰撞棱柱的边数（确定性要求：固定值，不可运行时改）。
pub const CYLINDER_SEGMENTS: u32 = 16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box { half: Vec3 },
    Sphere { radius: f32 },
    Cylinder { half_height: f32, radius: f32 },
    HeightField(HeightFieldId),
}

use crate::math::Vec3;

impl Shape {
    /// 包围球半径（宽相粗筛/保守 AABB 用）。
    pub fn bounding_sphere_radius(&self) -> f32 {
        match *self {
            Shape::Box { half } => half.length(),
            Shape::Sphere { radius } => radius,
            Shape::Cylinder {
                half_height,
                radius,
            } => (half_height * half_height + radius * radius).sqrt(),
            Shape::HeightField(_) => f32::INFINITY,
        }
    }

    pub fn kind_name(&self) -> &'static str {
        match self {
            Shape::Box { .. } => "box",
            Shape::Sphere { .. } => "sphere",
            Shape::Cylinder { .. } => "cylinder",
            Shape::HeightField(_) => "heightfield",
        }
    }
}
