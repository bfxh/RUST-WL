//! 高度场原语（§2.4 高度场特化的数据面；体素账本在 `vxl-phys-terrain`）。
//!
//! 双线性高度采样 + 解析梯度法线；列裁剪/局部采样在窄相查询中按需进行。

#![forbid(unsafe_code)]

use vxl_phys_core::Vec3;

/// 均匀网格高度场：`heights[iz * nx + ix]`，XZ 平面，Y 向上。
/// `spacing` = 相邻采样点间距（米）。
#[derive(Clone, Debug, Default)]
pub struct HeightField {
    pub origin_x: f32,
    pub origin_z: f32,
    pub nx: u32,
    pub nz: u32,
    pub spacing: f32,
    pub heights: Vec<f32>,
}

impl HeightField {
    /// 平地构造。
    pub fn flat(origin_x: f32, origin_z: f32, nx: u32, nz: u32, spacing: f32, height: f32) -> Self {
        assert!(nx >= 2 && nz >= 2, "HeightField 需要 nx/nz >= 2");
        assert!(spacing > 0.0);
        Self {
            origin_x,
            origin_z,
            nx,
            nz,
            spacing,
            heights: vec![height; (nx * nz) as usize],
        }
    }

    pub fn width_x(&self) -> f32 {
        (self.nx - 1) as f32 * self.spacing
    }

    pub fn width_z(&self) -> f32 {
        (self.nz - 1) as f32 * self.spacing
    }

    #[inline]
    pub fn contains(&self, x: f32, z: f32) -> bool {
        x >= self.origin_x
            && x <= self.origin_x + self.width_x()
            && z >= self.origin_z
            && z <= self.origin_z + self.width_z()
    }

    #[inline]
    pub fn height_ix(&self, ix: u32, iz: u32) -> f32 {
        self.heights[(iz * self.nx + ix) as usize]
    }

    #[inline]
    pub fn set_height(&mut self, ix: u32, iz: u32, h: f32) {
        self.heights[(iz * self.nx + ix) as usize] = h;
    }

    /// 双线性采样：返回 (高度，法线)。法线由解析梯度给出。
    pub fn sample(&self, x: f32, z: f32) -> Option<(f32, Vec3)> {
        if !self.contains(x, z) {
            return None;
        }
        let fx = (x - self.origin_x) / self.spacing;
        let fz = (z - self.origin_z) / self.spacing;
        let ix = ((fx.floor() as i32).clamp(0, self.nx as i32 - 2)) as u32;
        let iz = ((fz.floor() as i32).clamp(0, self.nz as i32 - 2)) as u32;
        let u = fx - ix as f32;
        let v = fz - iz as f32;
        let h00 = self.height_ix(ix, iz);
        let h10 = self.height_ix(ix + 1, iz);
        let h01 = self.height_ix(ix, iz + 1);
        let h11 = self.height_ix(ix + 1, iz + 1);
        let h =
            h00 * (1.0 - u) * (1.0 - v) + h10 * u * (1.0 - v) + h01 * (1.0 - u) * v + h11 * u * v;
        let dhdx = ((h10 - h00) * (1.0 - v) + (h11 - h01) * v) / self.spacing;
        let dhdz = ((h01 - h00) * (1.0 - u) + (h11 - h10) * u) / self.spacing;
        Some((h, Vec3::new(-dhdx, 1.0, -dhdz).normalize()))
    }

    /// 挖掘/账本接口的底层写入（供 vxl-phys-terrain 使用）。
    pub fn dig_column(&mut self, x: f32, z: f32, depth: f32) -> bool {
        if !self.contains(x, z) {
            return false;
        }
        let ix = (((x - self.origin_x) / self.spacing).floor() as i32).clamp(0, self.nx as i32 - 1)
            as u32;
        let iz = (((z - self.origin_z) / self.spacing).floor() as i32).clamp(0, self.nz as i32 - 1)
            as u32;
        let i = (iz * self.nx + ix) as usize;
        self.heights[i] -= depth;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_sample_normal_is_up() {
        let hf = HeightField::flat(-10.0, -10.0, 21, 21, 1.0, 0.0);
        let (h, n) = hf.sample(3.3, -2.7).unwrap();
        assert!(h.abs() < 1e-6);
        assert!(n.y > 0.999);
    }

    #[test]
    fn ramp_gradient_normal() {
        let mut hf = HeightField::flat(0.0, 0.0, 11, 11, 1.0, 0.0);
        for iz in 0..11 {
            for ix in 0..11 {
                hf.set_height(ix, iz, ix as f32);
            }
        }
        let (h, n) = hf.sample(5.5, 5.0).unwrap();
        assert!((h - 5.5).abs() < 1e-5);
        // 斜率 dh/dx = 1 → 法线 = normalize(-1, 1, 0)。
        let inv = core::f32::consts::FRAC_1_SQRT_2;
        assert!((n.x + inv).abs() < 1e-4);
        assert!((n.y - inv).abs() < 1e-4);
    }

    #[test]
    fn outside_is_none() {
        let hf = HeightField::flat(0.0, 0.0, 5, 5, 1.0, 0.0);
        assert!(hf.sample(100.0, 0.0).is_none());
    }
}

// ——————————————————————————————————————————————————————————————
// 互操作层（见 `docs/ROUTE.md` §2.1/§5 与 `vxl_phys_core::interop`）：
// 高度场作为**第一个 CollisionProvider 实现**（不改行为——本 impl 不接既有管线，
// 仅提供跨域接口能力；引擎既有高度场窄相路径保持不变）。
// ——————————————————————————————————————————————————————————————

impl vxl_phys_core::interop::CollisionProvider for HeightField {
    fn bounds(&self) -> vxl_phys_core::Aabb {
        // XZ 覆盖范围 × 高度极值（列扫描；调用频率低）。
        let mut lo = f32::INFINITY;
        let mut hi = f32::NEG_INFINITY;
        for &h in &self.heights {
            lo = lo.min(h);
            hi = hi.max(h);
        }
        if !lo.is_finite() {
            lo = 0.0;
            hi = 0.0;
        }
        vxl_phys_core::Aabb {
            min: Vec3::new(self.origin_x, lo, self.origin_z),
            max: Vec3::new(
                self.origin_x + self.width_x(),
                hi,
                self.origin_z + self.width_z(),
            ),
        }
    }

    /// 表面最近点：`sample` 给高度与解析法线；有向距离取竖直差在法线上的投影
    /// （斜坡一阶近似，与窄相竖直列采样同族；专用最优解留给 provider 专项优化）。
    fn closest_point(&self, p: Vec3) -> Option<vxl_phys_core::interop::SurfaceHit> {
        let (h, n) = self.sample(p.x, p.z)?;
        let surface = Vec3::new(p.x, h, p.z);
        Some(vxl_phys_core::interop::SurfaceHit {
            point: surface,
            normal: n,
            signed_dist: (p - surface).dot(n),
        })
    }
}

#[cfg(test)]
mod interop_tests {
    use super::*;
    use vxl_phys_core::interop::CollisionProvider;

    #[test]
    fn flat_field_closest_point_and_box_contacts() {
        let hf = HeightField::flat(-10.0, -10.0, 21, 21, 1.0, 0.5);
        let hit = hf.closest_point(Vec3::new(0.25, 1.0, -0.25)).unwrap();
        assert!((hit.point.y - 0.5).abs() < 1e-6);
        assert!((hit.signed_dist - 0.5).abs() < 1e-6);
        assert!((hit.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6);
        // 盒（半 0.5）落在 y=0.9 ⇒ 底面 4 角穿透 0.1
        let mut out = Vec::new();
        let any = hf.contacts_box(
            Vec3::splat(0.5),
            Vec3::new(0.0, 0.9, 0.0),
            vxl_phys_core::Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any);
        assert_eq!(out.len(), 4);
        for c in &out {
            assert!((c.depth - 0.1).abs() < 1e-5, "depth={}", c.depth);
        }
        assert!(hf.closest_point(Vec3::new(99.0, 1.0, 0.0)).is_none());
    }

    #[test]
    fn ramp_normal_points_downhill_outward() {
        let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        for iz in 0..11u32 {
            for ix in 0..11u32 {
                hf.set_height(ix, iz, ix as f32);
            }
        }
        let hit = hf.closest_point(Vec3::new(2.5, 3.0, 0.0)).unwrap();
        assert!(hit.normal.x < -0.5, "法线应逆坡：{:?}", hit.normal);
        assert!(hit.normal.y > 0.5);
    }
}
