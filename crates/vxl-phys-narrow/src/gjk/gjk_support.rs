//! gjk_support：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 支撑映射（世界系）：凸体沿 `dir` 的最远点 + 一个内部点。
pub trait Support {
    fn support(&self, dir: Vec3) -> Vec3;
    /// 内部点（用于 GJK 的初始方向与 EPA 的剥离）。
    fn interior(&self) -> Vec3;
}

/// 外壳体的支撑映射（世界位姿）。
pub struct HullSupport<'a> {
    pub hull: &'a ConvexHull,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for HullSupport<'_> {
    fn support(&self, dir: Vec3) -> Vec3 {
        let local_dir = self.rot.transpose_mul_vec3(dir);
        self.pos + self.rot.mul_vec3(self.hull.support_point(local_dir))
    }

    fn interior(&self) -> Vec3 {
        self.pos + self.rot.mul_vec3(self.hull.centroid)
    }
}

/// 盒体的支撑映射（与外壳同款接口；供 GJK 通用路径复用）。
pub struct BoxSupport {
    pub half: Vec3,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for BoxSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let ld = self.rot.transpose_mul_vec3(dir);
        let local = Vec3::new(
            if ld.x >= 0.0 {
                self.half.x
            } else {
                -self.half.x
            },
            if ld.y >= 0.0 {
                self.half.y
            } else {
                -self.half.y
            },
            if ld.z >= 0.0 {
                self.half.z
            } else {
                -self.half.z
            },
        );
        self.pos + self.rot.mul_vec3(local)
    }

    fn interior(&self) -> Vec3 {
        self.pos
    }
}
