//! 碰撞形状定义（§2.1）。
//!
//! 圆柱在碰撞层用固定 `CYLINDER_SEGMENTS` 边内接棱柱近似（§2.4 凸体路径统一），
//! 质量属性仍按解析圆柱计算（§2.1）。

/// 高度场句柄（数据本体在 `vxl-phys-narrow::heightfield`，账本在 `vxl-phys-terrain`）。
pub type HeightFieldId = u32;

/// 外部碰撞提供者句柄（数据本体由域 crate 持有；窄相经 `interop::ProviderColliders` 查询）。
pub type ProviderId = u32;

/// 圆柱碰撞棱柱的边数（确定性要求：固定值，不可运行时改）。
pub const CYLINDER_SEGMENTS: u32 = 16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box {
        half: Vec3,
    },
    Sphere {
        radius: f32,
    },
    Cylinder {
        half_height: f32,
        radius: f32,
    },
    /// **胶囊体**（本地方向 = +Y）：半径 `radius` 的球心沿 Y 轴线段
    /// `±half_height` 扫掠而成（两段半球帽 + 中段圆柱）。
    /// 碰撞走**解析支撑函数**（`gjk::CapsuleSupport`）——GJK/EPA 原样复用，
    /// 不做多边形化（多边形化的圆弧滚动会"打摆"，见 TECH-SURVEY A9）。
    Capsule {
        half_height: f32,
        radius: f32,
    },
    /// **圆锥**（本地方向 = +Y）：底面（半径 `radius`）在 `y = −half_height`，顶点在
    /// `y = +half_height`（高 = `2·half_height`）。
    /// 碰撞走**多面化**（`ConvexPolytope::cone_polytope`，与圆柱同族的棱面近似）——锥侧面是
    /// **光滑**面，直接喂 EPA 会复现胶囊那类不适定（`TECH-SURVEY.md` A9 / `EXPERIMENTS.md` R.2）。
    /// ⚠️ 本仓"体原点＝质心"这一前提对锥**不成立**（锥质心在底面上方 `H/4` 处）；惯量按
    /// **关于体原点**给出（含平行轴项），质心偏移的力矩效应不建模。
    Cone {
        half_height: f32,
        radius: f32,
    },
    HeightField(HeightFieldId),
    /// **外部碰撞提供者体**（体素/网格/喷溅场…；ROUTE §2.1 兼容轴）：
    /// id 索引 `interop::ProviderColliders`。静态 Marker 体，AABB 由提供者给。
    Provider(ProviderId),
    /// **凸体外壳**（多边形域）：`hull` 索引窄相 `HullStore`（点云在窄相自持，
    /// `Shape` 保持 Copy）；`half` = 点云局部 AABB 半长（宽相 + 惯量近似用）。
    ConvexHull {
        hull: u32,
        half: Vec3,
    },
    /// **复合体**（刚性多形状体）：`compound` 索引窄相 `CompoundStore`（子形状 = 形状 +
    /// 局部平移/旋转，顺序即特征序）；`half` = 子形状局部 AABB 并集半长（宽相 + 惯量近似用）。
    ///
    /// 窄相按子形状**展开为子对**并递归复用配对路径（特征号按子序号左移 16 位编码，
    /// 避免多条流形在同一暖缓存键上串号）。子形状**不得**再是复合体（构建期拒绝，防递归）。
    Compound {
        compound: u32,
        half: Vec3,
    },
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
            Shape::Capsule {
                half_height,
                radius,
            } => half_height + radius,
            // 顶点距原点 `half_height`、底圈距原点 `√(h²+r²)` ⇒ 取后者。
            Shape::Cone {
                half_height,
                radius,
            } => (half_height * half_height + radius * radius).sqrt(),
            Shape::ConvexHull { half, .. } => half.length(),
            Shape::Compound { half, .. } => half.length(),
            Shape::HeightField(_) | Shape::Provider(_) => f32::INFINITY,
        }
    }

    pub fn kind_name(&self) -> &'static str {
        match self {
            Shape::Box { .. } => "box",
            Shape::Sphere { .. } => "sphere",
            Shape::Cylinder { .. } => "cylinder",
            Shape::Capsule { .. } => "capsule",
            Shape::Cone { .. } => "cone",
            Shape::HeightField(_) => "heightfield",
            Shape::Provider(_) => "provider",
            Shape::ConvexHull { .. } => "hull",
            Shape::Compound { .. } => "compound",
        }
    }
}
