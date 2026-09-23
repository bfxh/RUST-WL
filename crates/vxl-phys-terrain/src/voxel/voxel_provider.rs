//! voxel_provider：从 voxel.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl CollisionProvider for VoxelVolume {
    fn bounds(&self) -> Aabb {
        // 有占据格 → 用占据包围盒（更紧）；否则退化为网格范围。
        self.occupied_bounds().unwrap_or(Aabb {
            min: self.origin,
            max: self.origin
                + Vec3::new(self.nx as f32, self.ny as f32, self.nz as f32) * self.step,
        })
    }

    /// 表面最近点：SDF + 有限差分法线（步长 = 半格，确定性固定序）。
    fn closest_point(&self, p: Vec3) -> Option<SurfaceHit> {
        let d = self.sdf(p);
        if d > self.step * 1.5 {
            return None; // 远离表面：视为无碰撞（与高度场「范围外 None」同语义）
        }
        let e = self.step * 0.5;
        let gx = self.sdf(p + Vec3::new(e, 0.0, 0.0)) - self.sdf(p - Vec3::new(e, 0.0, 0.0));
        let gy = self.sdf(p + Vec3::new(0.0, e, 0.0)) - self.sdf(p - Vec3::new(0.0, e, 0.0));
        let gz = self.sdf(p + Vec3::new(0.0, 0.0, e)) - self.sdf(p - Vec3::new(0.0, 0.0, e));
        let g = Vec3::new(gx, gy, gz);
        let n = if g.length_squared() > 1e-12 {
            g.normalize()
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        Some(SurfaceHit {
            point: p - n * d,
            normal: n,
            signed_dist: d,
        })
    }
}
