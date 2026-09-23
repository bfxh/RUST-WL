//! bvh8_bits：从 bvh8.rs 按域拆出（纯搬移，语义未改）。

/// 内部节点最大子数。
pub const BRANCH: usize = 8;
/// 叶容量。**1**：候选粒度与二叉叶一一对应（生产实测：叶容 2 ⇒ 候选 35.4 → 85.8
/// 万/tick、2.4×，而生产每条候选 ≈32ns ⇒ 候选膨胀吃掉了 6 层遍历的收益；叶容 1
/// 则候选数与二叉同量级，只赚遍历层数）。代价：叶节点数 = 体数（20 万叶 ≈ 7MB）。
pub const LEAF_CAP: usize = 1;

pub(crate) const NULL: u32 = u32::MAX;
pub(crate) const LEAF_BIT: u32 = 1 << 31;

#[inline]
pub(crate) fn is_leaf(id: u32) -> bool {
    id & LEAF_BIT != 0
}

#[inline]
pub(crate) fn leaf_index(id: u32) -> usize {
    (id & !LEAF_BIT) as usize
}
