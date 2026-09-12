//! 材质（§4.4/§4.5）：逐材质摩擦模型 + 恢复系数；材质对组合律。
//!
//! - 库仑（单 μ）/ 静+动（μs, μk + v_crit）/ 各向异性（μ1/μ2 + 方向角）；
//! - 摩擦组合：√(μ₁·μ₂)（Box2D 同族），各向异性暂取等效均值（方向应用在 M2 传送带）；
//! - 恢复系数组合：max（§4.5 组合律），恢复速度阈值在 PhysConfig（低于阈值 e 视作 0）。

#![forbid(unsafe_code)]

use crate::config::FrictionModel;

pub type MaterialId = u32;

/// 物理材质。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub friction: FrictionModel,
    /// 恢复系数 e ∈ [0, 1]。
    pub restitution: f32,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            friction: FrictionModel::Coulomb { mu: 0.5 },
            restitution: 0.0,
        }
    }
}

impl Material {
    pub const fn new(friction: FrictionModel, restitution: f32) -> Self {
        Self {
            friction,
            restitution,
        }
    }

    /// 摩擦组合：√(μ_eff(a)·μ_eff(b))。
    pub fn combine_friction(a: &FrictionModel, b: &FrictionModel) -> f32 {
        (a.effective_mu() * b.effective_mu()).sqrt()
    }

    /// 材质对组合（确定性纯函数）。
    pub fn combine(a: &Material, b: &Material) -> (f32, f32) {
        let mu = Self::combine_friction(&a.friction, &b.friction);
        // §4.5：e 取 max 组合律。
        let e = a.restitution.max(b.restitution);
        (mu, e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn friction_combine_geometric_mean() {
        let a = Material::new(FrictionModel::Coulomb { mu: 0.4 }, 0.0);
        let b = Material::new(FrictionModel::Coulomb { mu: 0.9 }, 0.0);
        let (mu, _) = Material::combine(&a, &b);
        assert!((mu - (0.36f32).sqrt()).abs() < 1e-6);
    }

    #[test]
    fn restitution_takes_max() {
        let a = Material::new(FrictionModel::Coulomb { mu: 0.5 }, 0.1);
        let b = Material::new(FrictionModel::Coulomb { mu: 0.5 }, 0.8);
        let (_, e) = Material::combine(&a, &b);
        assert_eq!(e, 0.8);
        let (_, e2) = Material::combine(&b, &a);
        assert_eq!(e2, 0.8);
    }

    #[test]
    fn static_kinetic_uses_mu_k_in_m1() {
        let a = Material::new(
            FrictionModel::StaticKinetic {
                mu_s: 0.9,
                mu_k: 0.6,
                v_crit: 1.0,
            },
            0.0,
        );
        let b = Material::new(FrictionModel::Coulomb { mu: 0.6 }, 0.0);
        let (mu, _) = Material::combine(&a, &b);
        assert!((mu - (0.6f32 * 0.6).sqrt()).abs() < 1e-5);
    }
}
