//! 轴对齐包围盒（**自 broad 下移至 core**：宽相/地形/互操作层共用此定义；
//! `vxl-phys-broad` 以 `pub use` 再导出，调用点路径不变）。

use crate::Vec3;

/// 轴对齐包围盒。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    #[inline]
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }

    /// 空盒（min = +∞、max = −∞）：`contains` 恒假——缓存失效哨兵。
    pub const EMPTY: Aabb = Aabb {
        min: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
        max: Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
    };

    /// `self` 是否完整包含 `o`（含边界）。
    #[inline]
    pub fn contains(&self, o: &Aabb) -> bool {
        o.min.x >= self.min.x
            && o.min.y >= self.min.y
            && o.min.z >= self.min.z
            && o.max.x <= self.max.x
            && o.max.y <= self.max.y
            && o.max.z <= self.max.z
    }

    /// 各向同性膨胀 `m`。
    #[inline]
    pub fn grown(&self, m: f32) -> Aabb {
        Aabb {
            min: self.min - Vec3::new(m, m, m),
            max: self.max + Vec3::new(m, m, m),
        }
    }
}

/// 两盒并集。
#[inline]
pub fn union_aabb(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: Vec3::new(
            a.min.x.min(b.min.x),
            a.min.y.min(b.min.y),
            a.min.z.min(b.min.z),
        ),
        max: Vec3::new(
            a.max.x.max(b.max.x),
            a.max.y.max(b.max.y),
            a.max.z.max(b.max.z),
        ),
    }
}
