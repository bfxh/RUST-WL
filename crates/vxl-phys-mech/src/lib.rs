//! # vxl-phys-mech
//!
//! 机械（§1 vxl-phys-mech）：齿轮/皮带/活塞/马达约束组 —— M2+ 落地。
//! 全部实现为求解器关节约束组（§2.5 关节族扩展），不引入新求解器。

#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MechJoint {
    /// 齿轮：传动比 + 相位。
    Gear,
    /// 皮带：等速 + 打滑阈值。
    Belt,
    /// 活塞：直线驱动 + 冲程限位。
    Piston,
    /// 马达：目标角速度 + 最大扭矩。
    Motor,
}

/// 可断裂/可限位/可阻尼的关节通用参数（§2.5 关节族）。
#[derive(Clone, Copy, Debug)]
pub struct JointParams {
    pub break_stress: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub damping: f32,
}

impl Default for JointParams {
    fn default() -> Self {
        Self {
            break_stress: f32::INFINITY,
            lower_limit: f32::NEG_INFINITY,
            upper_limit: f32::INFINITY,
            damping: 0.0,
        }
    }
}
