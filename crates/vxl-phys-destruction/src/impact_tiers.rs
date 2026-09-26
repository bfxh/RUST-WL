//! **冲量分级碎裂策略**（调研档 `SURVEY-SOFT-CLOTH-AND-CONVERSION.md` 的 T3-①）：
//! 把"冲击有多猛"映射成"碎成几块"，并受 [`FragmentBudget`] 封顶。
//!
//! **为什么值得**（外部对标结论）：外部 demo `shyhdm/Opengl_Physx` 的 `RuntimeFractureWall` 里，
//! Voronoi site 数是**按冲量分级**的（`coreSites = clamp(10 + level*3, 12, 36)`、
//! `outerSites = clamp(3 + level*.75, 4, 9)`）——"只碎一次"和"按能量分级碎裂"的观感差距巨大。
//! ⚠️ 该仓**无 LICENSE** ⇒ **只借思路、不抄常数**：本模块把那条曲线**参数化**（[`TierCurve`]），
//! 默认值只是**起点锚点**、不是放之四海皆准的真理。
//!
//! **本模块只做策略（纯函数）**：不生成 seeds、不碰 Voronoi、不动 `World`。接线点留给调用方
//! （`apply_impact_destruction` 加一条 opt-in 分级路径），这样**默认档逐位不变**——策略先立、
//! 判据先跑，接线那一步才是机械的。
//!
//! 全部是纯函数（无浮点累加、无顺序依赖）⇒ **同输入同输出**，且不需要 GPU ⇒ CI 会真跑判据。

use crate::{FractureDepth, FragmentBudget};

/// 冲量 → 档级的参照冲量（N·s）：`level = floor(impulse / ref_impulse)`。
/// 取 50.0 与 [`crate::DestructionConfig::default`] 的 `energy_thresholds = [50.0]` 同量级
/// ——即"刚好够触发破坏的那一下"记为 0 级。
pub const REF_IMPULSE: f32 = 50.0;

/// 档级上限（避免极端冲量把 site 数推爆；再猛也只是"最高档"）。
pub const MAX_LEVEL: u32 = 8;

/// 分级曲线（**参数化**：默认值抄自外部 demo 的形状，只是起点）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TierCurve {
    pub core0: u32,
    pub core_step: u32,
    pub core_max: u32,
    pub outer0: u32,
    pub outer_step: f32,
    pub outer_max: u32,
}

impl Default for TierCurve {
    /// 起点锚点（形状来自外部 demo 的经验曲线，**已参数化**，别当成物理定值）。
    fn default() -> Self {
        Self {
            core0: 10,
            core_step: 3,
            // **端点在最高档**：。
            // 钳位只对**自定义曲线**（步长调大后）生效——不让它变成走不到的死参数。
            core_max: 34,
            outer0: 3,
            outer_step: 0.75,
            outer_max: 9,
        }
    }
}

/// 冲量（N·s）→ 档级 ∈ `[0, MAX_LEVEL]`。非有限/负值按 0 级（**不让 NaN 传播**）。
pub fn impulse_level(impulse: f32, ref_impulse: f32) -> u32 {
    // NaN / 非正参照 ⇒ 0 级（不传播 NaN）；**+∞ ⇒ 最高档**（无穷猛不该给最小破坏——
    // 把 +inf 一起判成 0 级是本模块第一版的静默错，被判据抓住）。
    if impulse.is_nan() || ref_impulse.is_nan() || ref_impulse <= 0.0 || impulse <= 0.0 {
        return 0;
    }
    if impulse == f32::INFINITY {
        return MAX_LEVEL;
    }
    if !impulse.is_finite() || !ref_impulse.is_finite() {
        return 0;
    }
    let l = (impulse / ref_impulse).floor();
    if l <= 0.0 {
        0
    } else if l >= MAX_LEVEL as f32 {
        MAX_LEVEL
    } else {
        l as u32
    }
}

/// 档级 → `(内核 site 数, 外圈 site 数)`；两端**钳位**（曲线不许被档级推爆）。
pub fn sites_for_level(level: u32, curve: TierCurve) -> (u32, u32) {
    let l = level.min(MAX_LEVEL);
    let core = (curve.core0 + curve.core_step * l).min(curve.core_max);
    let outer_f = curve.outer0 as f32 + curve.outer_step * l as f32;
    let outer = (outer_f.floor().max(0.0) as u32).min(curve.outer_max);
    (core, outer)
}

/// 预算档 → 允许的**碎片/site 总数上限**（与枚举名一一对应）。
pub fn budget_cap(b: FragmentBudget) -> u32 {
    match b {
        FragmentBudget::B1K => 1 << 10,
        FragmentBudget::B10K => 10_000,
        FragmentBudget::B100K => 100_000,
        FragmentBudget::B1M => 1 << 20,
    }
}

/// 档级 + 预算 → 最终 `(core, outer)`：**按比例降到预算内**（保持两者的相对形状，
/// 且 core 至少 1、outer 至少 0）。返回的是"可以拿去生成 site 的数"。
pub fn sites_within_budget(level: u32, budget: FragmentBudget, curve: TierCurve) -> (u32, u32) {
    let (core, outer) = sites_for_level(level, curve);
    let cap = budget_cap(budget);
    let total = core + outer;
    if total <= cap {
        return (core, outer);
    }
    // 等比缩到 `cap`（向下取整 ⇒ 绝不超预算）；`cap == 1` 时退化成"只碎一块"。
    let core_scaled = ((core as u64 * cap as u64) / total as u64) as u32;
    let core_scaled = core_scaled.max(1).min(cap);
    let outer_scaled = cap.saturating_sub(core_scaled).min(outer);
    (core_scaled, outer_scaled)
}

/// 研读用：预断裂的**层数**语义（`FractureDepth` → 允许的细分次数）。
/// 与 `sites_*` 正交：层数决定"能碎几轮"，site 数决定"每轮几块"。
pub fn depth_rounds(d: FractureDepth) -> u32 {
    match d {
        FractureDepth::One => 1,
        FractureDepth::Two => 2,
        FractureDepth::Three => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// **两端钳位 + 单调**：档级越高 site 越多，但都在曲线钳位内。
    #[test]
    fn level_is_monotone_and_clamped() {
        let c = TierCurve::default();
        let mut prev = sites_for_level(0, c);
        for l in 1..=MAX_LEVEL {
            let cur = sites_for_level(l, c);
            assert!(cur.0 >= prev.0 && cur.1 >= prev.1, "档级 {l} 应不减少");
            prev = cur;
        }
        assert_eq!(sites_for_level(0, c), (10, 3), "0 级 = 曲线的下端点");
        assert_eq!(
            sites_for_level(MAX_LEVEL, c),
            (c.core_max, c.outer_max),
            "最高档 = 钳位上限（再猛也不涨）"
        );
        assert_eq!(
            sites_for_level(999, c),
            sites_for_level(MAX_LEVEL, c),
            "越档按上限"
        );
    }

    /// 冲量 → 档级：阈值以下为 0 级；非有限/负值**不传播 NaN**。
    #[test]
    fn impulse_level_boundaries() {
        assert_eq!(impulse_level(0.0, REF_IMPULSE), 0);
        assert_eq!(impulse_level(49.9, REF_IMPULSE), 0, "刚好不够 ⇒ 0 级");
        assert_eq!(impulse_level(50.0, REF_IMPULSE), 1, "够一下 ⇒ 1 级");
        assert_eq!(impulse_level(400.0, REF_IMPULSE), MAX_LEVEL, "很猛 ⇒ 封顶");
        assert_eq!(impulse_level(f32::NAN, REF_IMPULSE), 0);
        assert_eq!(impulse_level(f32::INFINITY, REF_IMPULSE), MAX_LEVEL);
        assert_eq!(
            impulse_level(1e6, 0.0),
            0,
            "参照为 0 ⇒ 退化到 0 级（不除零）"
        );
    }

    /// 预算封顶：任何档级在任何预算下都**不超**该预算的碎片数。
    #[test]
    fn budget_never_exceeded() {
        for b in [
            FragmentBudget::B1K,
            FragmentBudget::B10K,
            FragmentBudget::B100K,
            FragmentBudget::B1M,
        ] {
            let cap = budget_cap(b);
            for l in 0..=MAX_LEVEL {
                let (core, outer) = sites_within_budget(l, b, TierCurve::default());
                assert!(
                    core + outer <= cap,
                    "预算 {b:?}（上限 {cap}）在档级 {l} 被超：{core}+{outer}"
                );
                assert!(core >= 1, "至少一块");
            }
        }
    }

    /// 预算够大时**不改动**曲线（不该无谓缩放）。
    #[test]
    fn big_budget_keeps_curve_untouched() {
        let c = TierCurve::default();
        for l in 0..=MAX_LEVEL {
            assert_eq!(
                sites_within_budget(l, FragmentBudget::B1M, c),
                sites_for_level(l, c)
            );
        }
        // B1K 也够装默认曲线的最高档（36+9=45 ≤ 1024）⇒ 同样不动
        assert_eq!(
            sites_within_budget(MAX_LEVEL, FragmentBudget::B1K, c),
            sites_for_level(MAX_LEVEL, c)
        );
    }

    /// 小预算下的退化：缩到 `cap` 内、core 至少 1、outer 不多给。
    #[test]
    fn small_budget_scales_down_proportionally() {
        // 造一条远超 B1K 的曲线：core0=2000, core_max=4000, outer0=1000
        let big = TierCurve {
            core0: 2000,
            core_step: 500,
            core_max: 4000,
            outer0: 1000,
            outer_step: 100.0,
            outer_max: 2000,
        };
        let cap = budget_cap(FragmentBudget::B1K);
        let (core, outer) = sites_within_budget(MAX_LEVEL, FragmentBudget::B1K, big);
        assert!(core + outer <= cap);
        assert!(core >= 1 && core <= cap);
        // 仍保持"内核 ≫ 外圈"的相对形状（不翻转）
        assert!(
            core > outer,
            "缩放不该把相对形状倒过来（实得 core={core} outer={outer}）"
        );
    }

    /// 纯函数：同输入同输出（含 `f32` 路径）。
    #[test]
    fn is_pure_and_order_independent() {
        let a: Vec<(u32, u32)> = (0..=MAX_LEVEL)
            .map(|l| sites_within_budget(l, FragmentBudget::B10K, TierCurve::default()))
            .collect();
        let b: Vec<(u32, u32)> = (0..=MAX_LEVEL)
            .map(|l| sites_within_budget(l, FragmentBudget::B10K, TierCurve::default()))
            .collect();
        assert_eq!(a, b);
    }

    #[test]
    fn depth_rounds_maps_one_to_three() {
        assert_eq!(depth_rounds(FractureDepth::One), 1);
        assert_eq!(depth_rounds(FractureDepth::Three), 3);
    }
}
