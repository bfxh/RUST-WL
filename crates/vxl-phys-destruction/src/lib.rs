//! # vxl-phys-destruction
//!
//! 预断裂/运行时断裂/碎片（§4.9）—— M2 落地（集成方实装验收）。
//!
//! - 预断裂：Voronoi 分片（质心排序确定性），离线/装载期生成；
//! - 运行时断裂：冲击能量阈值 E_threshold（按材质 J 表）触发 Voronoi 细分一层；
//!   碎片继承动量；
//! - 结构稳定性：连接约束网络按应力断裂 → 连锁坍塌；碎片休眠阈值放宽一档。

#![forbid(unsafe_code)]

/// §4.9 碎片预算档。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FragmentBudget {
    B1K,
    B10K,
    B100K,
    /// 〔目标 GPU〕
    B1M,
}

/// 细分层数 ∈ {1, 2, 3}。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FractureDepth {
    One,
    Two,
    Three,
}

#[derive(Clone, Debug)]
pub struct DestructionConfig {
    pub budget: FragmentBudget,
    pub depth: FractureDepth,
    /// 材质冲击能量阈值表（J）。
    pub energy_thresholds: Vec<f32>,
}

impl Default for DestructionConfig {
    fn default() -> Self {
        Self {
            budget: FragmentBudget::B1K,
            depth: FractureDepth::One,
            energy_thresholds: vec![50.0],
        }
    }
}
