//! warm：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 单接触点的已求解冲量缓存（warm starting + 接触回收）。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct WarmPoint {
    pub(crate) pn: f32,
    pub(crate) pt1: f32,
    pub(crate) pt2: f32,
    /// 接触特征 ID（窄相命名，跨帧稳定；0 = 无特征）。
    pub(crate) feature: u32,
    /// 局部锚点（双方体局部坐标；M1 接触回收核心——锚点固定在材料点上，
    /// 帧间只推进距离；Rapier `contact_recycling` 同族）。
    pub(crate) la: Vec3,
    pub(crate) lb: Vec3,
    /// **烘焙时**的接触深度（正 = 穿透）。本帧有效深度 = 此值 + 锚点
    /// 当前累计分离量沿法向的投影（存更新值会复合累计 → 二次增长，实测
    /// 推出 0.44 m/s 虚假分离速度；必须存烘焙值）。
    pub(crate) depth0: f32,
}

impl WarmPoint {
    /// 零点哨兵（定长数组初始化用）。
    pub(crate) const EMPTY: WarmPoint = WarmPoint {
        pn: 0.0,
        pt1: 0.0,
        pt2: 0.0,
        feature: 0,
        la: Vec3::ZERO,
        lb: Vec3::ZERO,
        depth0: 0.0,
    };
}
