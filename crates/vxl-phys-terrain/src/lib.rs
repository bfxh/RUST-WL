//! # vxl-phys-terrain
//!
//! 可破坏地形（§4.9）：高度场体素账本 + 柱状支撑图。
//! M0：高度场集合管理 + 挖掘（dig）+ 包围盒输出；断裂/Voronoi 在 vxl-phys-destruction。

#![forbid(unsafe_code)]

use vxl_phys_broad::Aabb;
use vxl_phys_core::{Quat, Shape, Vec3};
pub use vxl_phys_narrow::heightfield::HeightField;

/// 地形集合：多个高度场，id = 插入序（与 marker 体 `Shape::HeightField(id)` 对应）。
#[derive(Default)]
pub struct TerrainSet {
    heightfields: Vec<HeightField>,
}

impl TerrainSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(&mut self, hf: HeightField) -> u32 {
        let id = self.heightfields.len() as u32;
        self.heightfields.push(hf);
        id
    }

    pub fn get(&self, id: u32) -> Option<&HeightField> {
        self.heightfields.get(id as usize)
    }

    pub fn get_mut(&mut self, id: u32) -> Option<&mut HeightField> {
        self.heightfields.get_mut(id as usize)
    }

    pub fn len(&self) -> usize {
        self.heightfields.len()
    }

    pub fn is_empty(&self) -> bool {
        self.heightfields.is_empty()
    }

    /// 窄相查询视图（按 id 索引）。
    pub fn slice(&self) -> &[HeightField] {
        &self.heightfields
    }

    /// 挖掘：在 (x, z) 列下挖 depth（账本写入；确定性：单列单写）。
    pub fn dig(&mut self, id: u32, x: f32, z: f32, depth: f32) -> bool {
        match self.heightfields.get_mut(id as usize) {
            Some(hf) => hf.dig_column(x, z, depth),
            None => false,
        }
    }

    /// 高度场世界包围盒（供宽相 marker 体使用；含高度极值）。
    pub fn bounds(&self, id: u32) -> Option<Aabb> {
        let hf = self.heightfields.get(id as usize)?;
        let mut mn = f32::MAX;
        let mut mx = f32::MIN;
        for &h in &hf.heights {
            mn = mn.min(h);
            mx = mx.max(h);
        }
        Some(Aabb {
            min: Vec3::new(hf.origin_x, mn, hf.origin_z),
            max: Vec3::new(hf.origin_x + hf.width_x(), mx, hf.origin_z + hf.width_z()),
        })
    }
}

/// 供门面创建 marker 体（静态、形状 = HeightField(id)）。
pub fn heightfield_marker_shape(id: u32) -> Shape {
    Shape::HeightField(id)
}

/// marker 体的标称变换（AABB 用 bounds() 而非变换，因此这里给零）。
pub const MARKER_TRANSFORM: (Vec3, Quat) = (Vec3::ZERO, Quat::IDENTITY);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_dig() {
        let mut t = TerrainSet::new();
        let id = t.add(HeightField::flat(0.0, 0.0, 5, 5, 1.0, 2.0));
        assert!(t.dig(id, 2.5, 2.5, 0.5));
        let hf = t.get(id).unwrap();
        assert!((hf.height_ix(2, 2) - 1.5).abs() < 1e-6);
        assert!(!t.dig(id, 100.0, 100.0, 0.5));
    }

    #[test]
    fn bounds_reflect_heights() {
        let mut t = TerrainSet::new();
        let id = t.add(HeightField::flat(0.0, 0.0, 5, 5, 1.0, 1.0));
        let b = t.bounds(id).unwrap();
        assert!((b.min.y - 1.0).abs() < 1e-6);
        assert!((b.max.x - 4.0).abs() < 1e-6);
    }
}
