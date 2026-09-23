//! bvh_types：从 bvh.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) const NULL: u32 = u32::MAX;

/// 建树工作项。`key` = 当前层最长轴上的中心（每层算一次，见 `build_range`）。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Work {
    pub(crate) body: u32,
    pub(crate) aabb: Aabb,
    pub(crate) key: f32,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct BvhNode {
    pub(crate) aabb: Aabb,
    pub(crate) left: u32,
    pub(crate) right: u32,
    pub(crate) parent: u32,
    /// 叶子：体 id；内部：NULL。
    pub(crate) body: u32,
    pub(crate) height: u32,
}

impl BvhNode {
    #[inline]
    pub(crate) fn is_leaf(&self) -> bool {
        self.left == NULL
    }
}

/// 增量动态 BVH。
#[derive(Clone, Debug)]
pub struct DynamicBvh {
    pub(crate) nodes: Vec<BvhNode>,
    pub(crate) free: Vec<u32>,
    pub(crate) root: u32,
    /// fat AABB 相对精确 AABB 的外扩（m）。
    pub fat_margin: f32,
}

/// AABB 周长（面积启发式代价；确定性：固定表达式顺序）。
pub(crate) fn perimeter(a: &Aabb) -> f32 {
    let wx = a.max.x - a.min.x;
    let wy = a.max.y - a.min.y;
    let wz = a.max.z - a.min.z;
    2.0 * (wx + wy + wz)
}

pub(crate) fn union_aabb(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

/// AABB 中心在指定轴上的坐标（中位分裂的排序键之一）。
pub(crate) fn center_axis(a: &Aabb, axis: u32) -> f32 {
    match axis {
        0 => (a.min.x + a.max.x) * 0.5,
        1 => (a.min.y + a.max.y) * 0.5,
        _ => (a.min.z + a.max.z) * 0.5,
    }
}

#[inline]
pub(crate) fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    a.min.x <= b.max.x
        && b.min.x <= a.max.x
        && a.min.y <= b.max.y
        && b.min.y <= a.max.y
        && a.min.z <= b.max.z
        && b.min.z <= a.max.z
}
