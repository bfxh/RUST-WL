//! # vxl-phys-soft
//!
//! 软体/布料（§4.6/§4.7，XPBD）—— M3 落地。本 crate 现为参数与接口骨架，
//! 数值全部来自规格书，供游戏侧提前引用。
//!
//! - XPBD 距离/体积约束（Müller et al. 2020；compliance α 单位 m/N）；
//! - 刚度档 α：近刚 1e-7 / 硬 1e-6 / 标准 1e-5 / 软 1e-4 / 果冻 3e-4；
//! - 布料 = 结构/剪切/弯曲三组约束 + Bridson 线化面元气动力；
//! - 撕裂：应变 ε_break ∈ {0.3, 0.5, 1.0, ∞}。

#![forbid(unsafe_code)]

/// §4.6 软体刚度档（compliance α，m/N）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stiffness {
    NearRigid,
    Hard,
    Standard,
    Soft,
    Jelly,
    Custom(f32),
}

impl Stiffness {
    pub fn alpha(self) -> f32 {
        match self {
            Stiffness::NearRigid => 1e-7,
            Stiffness::Hard => 1e-6,
            Stiffness::Standard => 1e-5,
            Stiffness::Soft => 1e-4,
            Stiffness::Jelly => 3e-4,
            Stiffness::Custom(a) => a,
        }
    }
}

/// §4.6/§4.7 撕裂应变阈值。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TearStrain {
    E03,
    E05,
    E10,
    None,
}

impl TearStrain {
    pub fn eps(self) -> f32 {
        match self {
            TearStrain::E03 => 0.3,
            TearStrain::E05 => 0.5,
            TearStrain::E10 => 1.0,
            TearStrain::None => f32::INFINITY,
        }
    }
}

/// 布料三组约束的独立 compliance（§4.7）。
#[derive(Clone, Copy, Debug)]
pub struct ClothConstraints {
    pub structural: Stiffness,
    pub shear: Stiffness,
    pub bending: Stiffness,
    pub tear: TearStrain,
}

impl Default for ClothConstraints {
    fn default() -> Self {
        Self {
            structural: Stiffness::Standard,
            shear: Stiffness::Soft,
            bending: Stiffness::Jelly,
            tear: TearStrain::E05,
        }
    }
}

/// 自碰撞参数（空间哈希 + 粒子半径；开启代价 ≤50%〔目标〕）。
#[derive(Clone, Copy, Debug)]
pub struct SelfCollision {
    pub enabled: bool,
    pub particle_radius: f32,
}

impl Default for SelfCollision {
    fn default() -> Self {
        Self {
            enabled: false,
            particle_radius: 0.05,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alpha_table_matches_spec() {
        assert_eq!(Stiffness::NearRigid.alpha(), 1e-7);
        assert_eq!(Stiffness::Hard.alpha(), 1e-6);
        assert_eq!(Stiffness::Standard.alpha(), 1e-5);
        assert_eq!(Stiffness::Soft.alpha(), 1e-4);
        assert_eq!(Stiffness::Jelly.alpha(), 3e-4);
    }

    #[test]
    fn tear_table_matches_spec() {
        assert_eq!(TearStrain::E03.eps(), 0.3);
        assert_eq!(TearStrain::None.eps(), f32::INFINITY);
    }
}
