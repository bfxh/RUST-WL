//! 高度场原语（§2.4 高度场特化的数据面；体素账本在 `vxl-phys-terrain`）。
//!
//! 双线性高度采样 + 解析梯度法线；列裁剪/局部采样在窄相查询中按需进行。

#![forbid(unsafe_code)]

use vxl_phys_core::Vec3;

/// 均匀网格高度场：`heights[iz * nx + ix]`，XZ 平面，Y 向上。
#[derive(Clone, Debug, Default)]
pub struct HeightField {
    pub origin_x: f32,
    pub origin_z: f32,
    pub nx: u32,
    pub nz: u32,
    pub cell: f32,
    pub heights: Vec<f32>,
}

impl HeightField {
    /// 平地构造。
    pub fn flat(origin_x: f32, origin_z: f32, nx: u32, nz: u32, cell: f32, height: f32) -> Self {
        assert!(nx >= 2 && nz >= 2, "HeightField 需要 nx/nz >= 2");
        assert!(cell > 0.0);
        Self {
            origin_x,
            origin_z,
            nx,
            nz,
            cell,
            heights: vec![height; (nx * nz) as usize],
        }
    }

    pub fn width_x(&self) -> f32 {
        (self.nx - 1) as f32 * self.cell
    }

    pub fn width_z(&self) -> f32 {
        (self.nz - 1) as f32 * self.cell
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
        let fx = (x - self.origin_x) / self.cell;
        let fz = (z - self.origin_z) / self.cell;
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
        let dhdx = ((h10 - h00) * (1.0 - v) + (h11 - h01) * v) / self.cell;
        let dhdz = ((h01 - h00) * (1.0 - u) + (h11 - h10) * u) / self.cell;
        Some((h, Vec3::new(-dhdx, 1.0, -dhdz).normalize()))
    }

    /// 挖掘/账本接口的底层写入（供 vxl-phys-terrain 使用）。
    pub fn dig_column(&mut self, x: f32, z: f32, depth: f32) -> bool {
        if !self.contains(x, z) {
            return false;
        }
        let ix =
            (((x - self.origin_x) / self.cell).floor() as i32).clamp(0, self.nx as i32 - 1) as u32;
        let iz =
            (((z - self.origin_z) / self.cell).floor() as i32).clamp(0, self.nz as i32 - 1) as u32;
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
