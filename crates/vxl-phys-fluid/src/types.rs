//! types：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 边界粒子生成输入：体原点位姿 + 速度（体面速度 = `linvel + angvel×r`）。
///
/// `pos` = **体原点**（本仓体原点即质心；锥例外，见 `Shape::Cone` 注）；`rot` = 体姿态。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyPose {
    pub pos: Vec3,
    pub rot: Quat,
    pub linvel: Vec3,
    pub angvel: Vec3,
}

/// §4.8 流体技术族。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FluidFamily {
    /// WCSPH + XSPH（CPU，30 万粒 @30 FPS 档）。
    CpuSph,
    /// Position Based Fluids（GPU，1000 万粒目标档）。
    GpuPbf,
    /// FLIP/APIC（GPU 高精档，稀疏哈希网格）。
    GpuFlip,
}

/// 密度约束迭代档（PBF）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DensityIterations {
    Three,
    Four,
}
