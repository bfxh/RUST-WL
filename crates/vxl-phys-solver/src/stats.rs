//! stats：从 lib.rs 按域拆出（纯搬移，语义未改）。

/// **warm 匹配分支计数**（诊断；atomic 宽松序，只计数、不改行为 ⇒ 哈希不变）。
///
/// 用途：判定「本仓流形是否**特征稳定**」——这是"检测每步一次 / 便宜子步"能否成立的
/// 前提。§9（P1：检测上提，塔崩 |v| 44.4）与 §10（复用约束 + prepare，全变体崩）把失败
/// 归因于"本引擎流形是**裁剪产物** ⇒ 材料点配对跨帧漂移"；但窄相的 `feature` 设计本
/// 就是**几何特征哈希**（`feat_intersect`：入射棱对 × 参考侧平面，"同三元组重现同 ID"），
/// 即**该归因缺实测支撑**。本计数给出依据：
/// - **精确命中率高** ⇒ 配对稳定 ⇒ 归因错误，§9/§10 的失败须另找机理
///   （便宜子步这条最值钱的结构杠杆应重新评估）；
/// - **命中率低** ⇒ 归因成立（且原因多半是参考面翻转 / 裁剪输出churn）。
pub(crate) static WARM_EXACT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static WARM_FALLBACK: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
pub(crate) static WARM_MISS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// **回退命中的成因分解**（诊断；只计数）：特征是"ID 变了但材料点还在附近"，
/// 拆开看变了哪一种 ⇒ 判定修复入口（见 vxl_phys_narrow::feature_kind 注）。
pub(crate) static WB_SIDE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static WB_CLIP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static WB_HASH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static WB_SAME: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// **睡眠诊断计数**（只计数、不改行为 ⇒ 不影响哈希）。用途：分辨"岛为什么没睡"。
/// 动机（`PLAN-solver-limits.md`）：`16/1/128` 塔末态 **0 体超阈**却仍有 ~625 体不睡，
/// 而"牵连唤醒清零"假说已否证 ⇒ 必须**先量**：是"有人快"（`all_slow=false`），
/// 还是"`all_slow=true` 但 `min_timer` 攒不满 `sleep_time`"（成员 churn 拖低）。
pub(crate) static SLEEP_D_FAST: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static SLEEP_D_WAIT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static SLEEP_D_SLEPT: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
pub(crate) static SLEEP_D_WAIT_MAX_MS: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// **睡眠诊断（深档）**：计数"每个子步到底有多少个体超阈"。
///
/// 用途：判定**路 A（子块睡眠）的前提是否成立**。A 的睡眠判据是"**逐体**安静"，
/// 而现行判据是"整岛安静"；若塔里**绝大多数体在子步尺度就超阈**，则 A 的逐体计时器
/// 同样攒不满 `sleep_time` ⇒ **A 给不了任何东西**（见 `PLAN-solver-limits.md`）。
/// `false`（默认）⇒ 保留早退、不计数、**零开销**；`true` ⇒ 扫完整个岛并计数（只计数，不改行为）。
pub(crate) const SLEEP_DIAG_DEEP: bool = false;
pub(crate) static SLEEP_D_FAST_BODY: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);
pub(crate) static SLEEP_D_BODY_ALL: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// 取深档睡眠诊断：(超阈体·子步累计, 受检体·子步累计)。
pub fn sleep_diag_deep_take() -> (u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        SLEEP_D_FAST_BODY.swap(0, Relaxed),
        SLEEP_D_BODY_ALL.swap(0, Relaxed),
    )
}

/// 取睡眠诊断：(有人快而拒, 全慢但未满, 入睡, 等待中 `min_timer` 最大值(ms))。
pub fn sleep_diag_take() -> (u64, u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        SLEEP_D_FAST.swap(0, Relaxed),
        SLEEP_D_WAIT.swap(0, Relaxed),
        SLEEP_D_SLEPT.swap(0, Relaxed),
        SLEEP_D_WAIT_MAX_MS.swap(0, Relaxed),
    )
}

/// **参考面变化的代理**（诊断；只计数）：匹配上的点里，"接触法向变了"（`dot < 0.99999`）
/// 与"法向几乎不变"各占多少。
///
/// 用途（A6 参考面稳定性）：`feature` 的哈希部分 = `入射棱对 × 参考侧平面 k`
/// （`k = ref_base + k'`）。**换参考面**（`ref_base` 变）通常伴随法向改变；而在**同一面内**
/// 换裁剪侧平面（`k'` 变）则法向不变、只有哈希变 ⇒ 用"法向是否变"即可把 K.1 的
/// "哈希变 55.9%" 拆成「换面」与「同面换裁剪面」两块。
pub(crate) static WN_FLIP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static WN_SAME: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 读取并清零参考面代理计数：`(法向变了, 法向几乎不变)`。
pub fn warm_normal_flip_take() -> (u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (WN_FLIP.swap(0, Relaxed), WN_SAME.swap(0, Relaxed))
}

/// 读取并清零回退成因分解：`(侧别翻转, 裁剪路变化, 哈希变化, 特征相同)`。
pub fn warm_fallback_kind_take() -> (u64, u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        WB_SIDE.swap(0, Relaxed),
        WB_CLIP.swap(0, Relaxed),
        WB_HASH.swap(0, Relaxed),
        WB_SAME.swap(0, Relaxed),
    )
}

/// 读取并清零 warm 匹配计数：`(精确特征命中, 近邻回退命中, 未匹配)`。
pub fn warm_match_stats_take() -> (u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        WARM_EXACT.swap(0, Relaxed),
        WARM_FALLBACK.swap(0, Relaxed),
        WARM_MISS.swap(0, Relaxed),
    )
}

/// **解算记账**（诊断；只计数）：点总数 / 其中分离点（`depth < 0`，吃 spec 额度）数 /
/// `spec` 之和 ×1000 / `bias` 之和 ×1000。
///
/// 用途（`EXPERIMENTS.md` 末节 N/P 的复用余价）：判定"复用是否让去穿透/闭合额度被
/// **重复发放**"——复用下几何冻结，若同一份 `sep` 被逐子步当额度发出去，就是系统性的
/// 能量注入；"每子步 spec 之和"与"每子步 bias 之和"在复用开关下的对比即可判。
pub(crate) static DA_PTS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static DA_SEP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static DA_SPEC: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub(crate) static DA_BIAS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 读取并清零解算记账：`(点总数, 分离点数, spec 和 ×1000, bias 和 ×1000)`。
pub fn solve_accounting_take() -> (u64, u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        DA_PTS.swap(0, Relaxed),
        DA_SEP.swap(0, Relaxed),
        DA_SPEC.swap(0, Relaxed),
        DA_BIAS.swap(0, Relaxed),
    )
}

/// 回退匹配的**接触状态门**（米）：只有本帧裁剪深度 > 此阈值的回退匹配才允许/// 暖启动——**分离/预期接触（depth ≤ 0）拒配**（按新接触处理，暖冲量清零、
/// 锚点重烘焙）。依据：错配锚点的暖冲量会过驱动「间歇角点接触」（125 体族的
/// 触发画像 = 留缝角点间歇接触，见 OPEN-PROBLEMS P1）；而塔的承重腿是持续
/// 正深度接触、不受影响。**非距离判别式**（距离类门已实测分不开有害/必需匹配）。
pub(crate) const FB_DEPTH_MIN: f32 = 0.0;
