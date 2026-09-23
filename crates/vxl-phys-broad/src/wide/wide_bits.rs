//! wide_bits：从 wide.rs 按域拆出（纯搬移，语义未改）。

/// 每内部节点的最大子数（8 路 ⇒ 遍历层数为二叉的 1/(log2 8) = 1/3）。
pub const WIDE: usize = 8;

pub(crate) const NULL: u32 = u32::MAX;
