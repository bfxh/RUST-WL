//! types：从 joints.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 关节类型（与 PhysArena 的 JointKind 同构；`Spring` 归入 `Distance`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    /// 球形：两个锚点重合（3 线性约束），相对转动自由。
    Spherical,
    /// 转动：锚点重合 + 相对转动只允许绕 `axis`（锁 2 个角自由度）。
    Revolute,
    /// 固定：锚点重合 + 相对转动全锁（6 自由度）。
    Fixed,
    /// 棱柱：只允许沿 `axis` 平移（锁 2 线性 + 3 角自由度）。
    Prismatic,
    /// 距离：两锚点距离 = `rest`（1 约束）。
    Distance,
}

/// 一条关节（引擎侧 IR；锚点/轴均为**体局部**）。
#[derive(Clone, Copy, Debug)]
pub struct Joint {
    pub kind: JointKind,
    pub a: u32,
    pub b: u32,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    /// 局部轴（转动/棱柱用；球形/固定/距离忽略）。
    pub axis_a: Vec3,
    pub axis_b: Vec3,
    /// 距离关节的静止长度。
    pub rest: f32,
    /// **马达目标**（转动：自由轴的相对角速度 rad/s；棱柱：沿轴的相对线速度 m/s）。
    /// `motor_max_force <= 0` = 无马达（默认）⇒ 逐位等价于无马达行为。
    pub motor_target: f32,
    /// 马达最大力/矩（N 或 N·m）；每子步的冲量上钳 = `motor_max_force · dt`。
    pub motor_max_force: f32,
    /// **转动限位**（rad，绕自由轴）与**棱柱行程限位**（m，沿轴；`lower >= upper`
    /// 视为未设）。判据用**当前相对姿态/锚点几何**直接算（无跨帧累计状态 ⇒ 无漂移、
    /// 逐位确定），越界时才建单边行 + 偏置推回。两者都在马达之后解（限位最后说话）。
    pub limit_lower: f32,
    pub limit_upper: f32,
}

impl Joint {
    pub fn new(kind: JointKind, a: u32, b: u32, anchor_a: Vec3, anchor_b: Vec3) -> Self {
        Self {
            kind,
            a,
            b,
            anchor_a,
            anchor_b,
            axis_a: Vec3::X,
            axis_b: Vec3::X,
            rest: 0.0,
            motor_target: 0.0,
            motor_max_force: 0.0,
            limit_lower: 1.0,
            limit_upper: -1.0,
        }
    }

    /// 加**转动限位**（rad，绕 `axis_a`；`lower >= upper` 视为未设）。
    pub fn with_limits(mut self, lower: f32, upper: f32) -> Self {
        self.limit_lower = lower;
        self.limit_upper = upper;
        self
    }

    /// 加**马达**（转动：目标角速度 rad/s；棱柱：目标线速度 m/s；`max_force <= 0` = 关）。
    /// 关节限位仍未实现（需累计相对转角状态）。
    pub fn with_motor(mut self, target_velocity: f32, max_force: f32) -> Self {
        self.motor_target = target_velocity;
        self.motor_max_force = max_force;
        self
    }

    pub fn with_axis(mut self, axis: Vec3) -> Self {
        self.axis_a = axis;
        self.axis_b = axis;
        self
    }

    pub fn with_rest(mut self, rest: f32) -> Self {
        self.rest = rest;
        self
    }
}

/// 关节集合（世界持有；每帧求解一遍）。
#[derive(Default)]
pub struct JointSet {
    pub joints: Vec<Joint>,
}

/// 关节求解参数（与接触通道同口径：软约束偏置 + CFM 正则化近似）。
#[derive(Clone, Copy)]
pub(crate) struct JointParams {
    /// 位置偏置率（1/s）：`β·inv_dt`。
    pub(crate) bias_inv_dt: f32,
    /// `1/dt`（马达冲量上钳换算用：`max_force·dt = max_force / inv_dt`）。
    pub(crate) inv_dt: f32,
    /// 速度级残差下限（早退判据，m/s）。
    pub(crate) eps: f32,
    /// 最少迭代数（早退前的保底）。
    pub(crate) min_iters: u32,
}
