//! warm_data：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 回退匹配的**深度跳变门**（米）：`|本帧裁剪深度 − 锚点烘焙深度|` 超限 ⇒ 拒配。
/// 这是 OPEN-PROBLEMS P1 候选①的正确形式——**注意别写成「有效深度 vs 裁剪深度」**：
/// 锚点随体刚性移动时 `depth0 + sep·n ≡ cp.depth` 恒成立（实测差 ≤1e-6），
/// 那样写等于不设门（本轮踩过）。直接量 `cp.depth − depth0` 有真信号：插桩实测
/// 「刚触地型」匹配 = 锚点在分离态烘焙（d0 = −3.7mm）后被拿到触地帧用
/// （d = +0.2mm）⇒ 跳变 3.9mm、且携带加载相冲量。**阈值扫描（125 体复现 dE 峰值）**：
/// 1mm → 63.8、0.5mm → 27.8、**0.25mm → 18.2（取此档，验收线 ≤20）**、0.1mm → 17.7
/// （更紧开始逼近「全拒」；全拒 = 16.2 但塔崩）。物理含义：dd ≈ v·dt ⇒ 只对
/// 「深度变化快于 ~1.5 cm/s」的接触拒配，与睡眠阈（4 cm/s）同量级。
pub(crate) const FB_DEPTH_JUMP: f32 = 0.00025;

#[derive(Clone, Copy, Debug)]
pub(crate) struct WarmManifold {
    pub(crate) normal: Vec3,
    /// 点数据**定长内联**（流形点 ≤4，窄相已截断）——此前是 `Vec<WarmPoint>`，
    /// 每 tick 22 万次小 Vec 的分配 + 释放实测 ≈11.4ms（泄漏探针）；内联后零堆。
    pub(crate) points: [WarmPoint; 4],
    pub(crate) n: u8,
    /// 求解印章（本 solve 调用是否刷新过；剪枝用，见 `warm 槽位表` 注）。
    pub(crate) seen: u32,
}

impl WarmManifold {
    pub(crate) const EMPTY: WarmManifold = WarmManifold {
        normal: Vec3::ZERO,
        points: [WarmPoint::EMPTY; 4],
        n: 0,
        seen: 0,
    };

    #[inline]
    pub(crate) fn pts(&self) -> &[WarmPoint] {
        &self.points[..self.n as usize]
    }
}

/// **warm 缓存键**：`(体 a, 体 b, 特征空间)`。
///
/// 特征空间来自 `ContactPoints::space()`（**显式通道**，窄相复合体展开时按子形状序号填；
/// 非复合体恒 0）。**为什么必须有它**：同一体对会同时存在多条流形（复合体每个子形状一条），
/// 共用 `(a, b)` 键会相互覆盖 ⇒ 该对的暖缓存**整条失效**（实测 `warm_counter_probe`：
/// 两球复合体计数全 0，单子形状复合体 32）。非复合体场景空间恒为 0 ⇒ 键退化为 `(a, b, 0)`。
///
/// ⚠️ **不要**改用 `feature` 的高位当空间：窄相特征号高位被"侧别/裁剪路/哈希"编码占用，
/// 哈希逐帧漂移 ⇒ 普通场景暖启动会随机失效（实测 `default_tier_stability` 的 top_y 漂 3 mm，
/// 见 `TECH-SURVEY.md` A9 ④）。
pub(crate) type WarmKey = (u32, u32, u32);

/// 死槽键（槽位表空洞哨兵）。
pub(crate) const DEAD_KEY: WarmKey = (u32::MAX, u32::MAX, u32::MAX);

/// warm 回写条目：(槽号（`u32::MAX` = 新键）, 键, 数据)。
pub(crate) type WarmOutEntry = (u32, WarmKey, WarmManifold);
