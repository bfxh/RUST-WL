//! aabb_util：**共享的 AABB 助手**（去重自 bvh8_impl.rs、wide_impl.rs）。
//! 逐字相同的实现只保留这一份——此前三套 BVH 各抄一遍（实测各 ~90 行）。

use super::*;

pub(crate) fn union(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

pub(crate) fn perimeter(a: &Aabb) -> f32 {
    let d = a.max - a.min;
    2.0 * (d.x + d.y + d.z)
}

pub(crate) fn enlargement(a: &Aabb, b: &Aabb) -> f32 {
    perimeter(&union(a, b)) - perimeter(a)
}

pub(crate) fn longest_axis(a: &Aabb) -> u32 {
    let d = a.max - a.min;
    if d.x >= d.y && d.x >= d.z {
        0
    } else if d.y >= d.z {
        1
    } else {
        2
    }
}

pub(crate) fn center_on(a: &Aabb, axis: u32) -> f32 {
    match axis {
        0 => (a.min.x + a.max.x) * 0.5,
        1 => (a.min.y + a.max.y) * 0.5,
        _ => (a.min.z + a.max.z) * 0.5,
    }
}

pub(crate) fn cmp_axis_center(a: &(u32, Aabb), b: &(u32, Aabb), axis: u32) -> std::cmp::Ordering {
    center_on(&a.1, axis)
        .total_cmp(&center_on(&b.1, axis))
        .then(a.0.cmp(&b.0))
}

pub(crate) fn same_box(a: &Aabb, b: &Aabb) -> bool {
    a.min.x == b.min.x
        && a.min.y == b.min.y
        && a.min.z == b.min.z
        && a.max.x == b.max.x
        && a.max.y == b.max.y
        && a.max.z == b.max.z
}

pub(crate) fn contains_box(outer: &Aabb, inner: &Aabb) -> bool {
    outer.min.x <= inner.min.x
        && outer.min.y <= inner.min.y
        && outer.min.z <= inner.min.z
        && outer.max.x >= inner.max.x
        && outer.max.y >= inner.max.y
        && outer.max.z >= inner.max.z
}

pub(crate) fn zero_box() -> Aabb {
    Aabb {
        min: vxl_phys_core::Vec3::ZERO,
        max: vxl_phys_core::Vec3::ZERO,
    }
}
