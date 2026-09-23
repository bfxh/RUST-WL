//! gjk_shapes：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 球支撑（解析；`dir = 0` 退回球心 ⇒ 确定性）。
pub struct SphereSupport {
    pub radius: f32,
    pub pos: Vec3,
}

impl Support for SphereSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let l = dir.length();
        if l < 1e-12 {
            self.pos
        } else {
            self.pos + dir * (self.radius / l)
        }
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 胶囊体支撑（解析）：局部 **Y 轴线段 `±half_height` ⊕ 半径球**。
///
/// 关键：**不做多边形化**——多边形化的圆弧在滚动/倾斜承载时会"打摆"（每过一个面片
/// 一次冲击），这正是当前 cylinder 走多面体路径的代价。支撑函数式接入 GJK/EPA 即可拿到
/// 光滑外形的精确最远点（`TECH-SURVEY.md` A9）。
pub struct CapsuleSupport {
    pub half_height: f32,
    pub radius: f32,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for CapsuleSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        // 线段端点：局部 Y 方向取号（`dir = 0` 取 +h ⇒ 确定性）。
        let ld = self.rot.transpose_mul_vec3(dir);
        let sy = if ld.y >= 0.0 {
            self.half_height
        } else {
            -self.half_height
        };
        let seg = self.rot.mul_vec3(Vec3::new(0.0, sy, 0.0));
        // 球帽：沿世界方向偏移 radius（`dir = 0` 退回线段端点 ⇒ 确定性）。
        let l = dir.length();
        let n = if l < 1e-12 {
            Vec3::ZERO
        } else {
            dir * (1.0 / l)
        };
        self.pos + seg + n * self.radius
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 圆柱/圆锥（**多面化表示**）的支撑：与 `ConvexPolytope::cylinder_polytope` /
/// `cone_polytope` **同一套 `segments` 边内接多边形**（角度量化到最近顶点 ⇒ 支撑 = 顶点取极值）。
///
/// ⚠️ **与胶囊的对比（别混淆）**：这里是**多面体**支撑（顶点有限）⇒ 喂 EPA 良态；胶囊是
/// **光滑**面 ⇒ EPA 不适定（`EXPERIMENTS.md` R.2/R.3）。**不要**为了"更圆"去掉角度量化，
/// 那会退化成光滑支撑、复现胶囊那类 45° 假法线。
pub struct PrismSupport {
    pub half_height: f32,
    pub radius: f32,
    pub segments: u32,
    /// `true` = 圆锥（顶点在 +Y、底面在 −Y）；`false` = 圆柱（两端圆盘）。
    pub cone: bool,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for PrismSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let ld = self.rot.transpose_mul_vec3(dir);
        let n = self.segments.max(8) as f32;
        let two_pi = 2.0 * core::f32::consts::PI;
        // 角度量化到**最近顶点**（等分 ⇒ 最大点积即最近角；`dir = 0` 取 0 角 ⇒ 确定性）。
        let ang = if ld.x == 0.0 && ld.z == 0.0 {
            0.0
        } else {
            ld.z.atan2(ld.x)
        };
        let k = (ang / two_pi * n).round().rem_euclid(n);
        let th = two_pi * k / n;
        let (s, c) = th.sin_cos();
        let (rx, rz) = (self.radius * c, self.radius * s);
        let rim_dot = self.radius * (c * ld.x + s * ld.z);
        if self.cone {
            // 锥：`max(顶点·dir, 底圈最近顶点·dir)`——不能只看 `ld.y` 的符号（近水平方向时底圈更大）。
            let apex_dot = self.half_height * ld.y;
            let base_dot = -self.half_height * ld.y + rim_dot;
            let local = if apex_dot >= base_dot {
                Vec3::new(0.0, self.half_height, 0.0)
            } else {
                Vec3::new(rx, -self.half_height, rz)
            };
            self.pos + self.rot.mul_vec3(local)
        } else {
            // 圆柱：两盘同角度的最近顶点取极值 ⇒ 盘心沿 ±Y 取号即最大。
            let y = if ld.y >= 0.0 {
                self.half_height
            } else {
                -self.half_height
            };
            self.pos + self.rot.mul_vec3(Vec3::new(rx, y, rz))
        }
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 形状 → 支撑体（窄相统一入口）。
pub enum ShapeSupport<'a> {
    Hull(HullSupport<'a>),
    Box(BoxSupport),
    Sphere(SphereSupport),
    Capsule(CapsuleSupport),
    /// 圆柱/圆锥（多面化表示；见 `PrismSupport`）。
    Prism(PrismSupport),
}

impl Support for ShapeSupport<'_> {
    fn support(&self, dir: Vec3) -> Vec3 {
        match self {
            ShapeSupport::Hull(s) => s.support(dir),
            ShapeSupport::Box(s) => s.support(dir),
            ShapeSupport::Sphere(s) => s.support(dir),
            ShapeSupport::Capsule(s) => s.support(dir),
            ShapeSupport::Prism(s) => s.support(dir),
        }
    }
    fn interior(&self) -> Vec3 {
        match self {
            ShapeSupport::Hull(s) => s.interior(),
            ShapeSupport::Box(s) => s.interior(),
            ShapeSupport::Sphere(s) => s.interior(),
            ShapeSupport::Capsule(s) => s.interior(),
            ShapeSupport::Prism(s) => s.interior(),
        }
    }
}

// ---------------------------------------------------------------------------
// 凸体切割（半空间裁剪 + Voronoi 预断裂）——「更一般的凸体/网格切割」的凸体侧。
// ---------------------------------------------------------------------------
