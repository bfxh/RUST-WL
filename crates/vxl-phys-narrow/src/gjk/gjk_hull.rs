//! gjk_hull：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 凸体（外壳）：顶点集 + 预计算质心（供 EPA 剥离用）。
#[derive(Clone, Debug, Default)]
pub struct ConvexHull {
    pub points: Vec<Vec3>,
    /// 质心（顶点均值；作为「体内点」用于 GJK/EPA 的剥离方向）。
    pub centroid: Vec3,
}

impl ConvexHull {
    pub fn new(points: Vec<Vec3>) -> Self {
        let n = points.len().max(1) as f32;
        let mut c = Vec3::ZERO;
        for p in &points {
            c += *p;
        }
        Self {
            points,
            centroid: c * (1.0 / n),
        }
    }

    /// 沿 `dir`（单位，非零）最远的顶点。
    #[inline]
    pub fn support_point(&self, dir: Vec3) -> Vec3 {
        let mut best = self.points[0];
        let mut best_d = best.dot(dir);
        for &p in &self.points[1..] {
            let d = p.dot(dir);
            if d > best_d {
                best_d = d;
                best = p;
            }
        }
        best
    }
}
