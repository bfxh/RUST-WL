//! 配置与真实度档位（§4）。
//!
//! 每个字段对应规格书的一档：「技术名 + 参数数值」。运行时切换 = 当前 tick 末生效
//! （状态一致），M0 起以启动期固定为主。

use crate::math::Vec3;

/// §4.15 预设（游戏侧命名引用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preset {
    /// 街机：迭代 4 / 子步 1 / skin 0.02 / 库仑。
    Arcade,
    /// 平衡：迭代 8 / 子步 2 / skin 0.01 / 静+动。
    Balanced,
    /// 真实：迭代 16 / 子步 4 / skin 0.005 / 各向异性。
    Realistic,
    /// 极致：迭代 32 / 子步 8 / skin 0.002 / 各向异性。
    Extreme,
}

/// §4.4 摩擦模型（实现位置：接触约束摩擦锥缩放，不改求解器结构）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FrictionModel {
    /// 库仑（单 μ）。
    Coulomb { mu: f32 },
    /// 静+动（μs, μk）+ 临界速度 v_crit（Stribeck 线化）。
    StaticKinetic { mu_s: f32, mu_k: f32, v_crit: f32 },
    /// 各向异性：两主轴 μ1/μ2 + 方向角（弧度，体局部 XZ 面；传送带/履带用）。
    Anisotropic { mu1: f32, mu2: f32, angle: f32 },
}

impl FrictionModel {
    /// M0 求解器使用的等效标量 μ（各向异性取均值；Stribeck 取 μk，μs 在 M1 经 v_crit 接入）。
    pub fn effective_mu(&self) -> f32 {
        match *self {
            FrictionModel::Coulomb { mu } => mu,
            FrictionModel::StaticKinetic { mu_k, .. } => mu_k,
            FrictionModel::Anisotropic { mu1, mu2, .. } => 0.5 * (mu1 + mu2),
        }
    }
}

/// 主配置。`Default` = 规格书默认档（迭代 16、子步 1、skin 0.01）。
#[derive(Clone, Debug, PartialEq)]
pub struct PhysConfig {
    /// 基础步长，固定 60 Hz（§5）。
    pub dt: f32,
    /// §4.2 每 60Hz 帧子步 ∈ {1,2,4,8,16}。
    pub substeps: u32,
    /// §4.1 TGS-Soft/顺序冲量速度迭代 ∈ {1,4,8,16,32,64}，默认 16。
    pub velocity_iterations: u32,
    /// §4.3 接触 speculative margin（skin）四档：0.02/0.01/0.005/0.002。
    pub contact_skin: f32,
    /// §4.3 GJK 收敛容差（M0 SAT 路径仅存档）。
    pub gjk_tolerance: f32,
    /// §4.12 CCD 触发速度阈值（m/s）；`INFINITY` = 关。
    pub ccd_speed_threshold: f32,
    /// §4.4 摩擦模型。
    pub friction: FrictionModel,
    /// §4.5 恢复系数 e ∈ [0,1]（M0 全局；材质对在 M1）。
    pub restitution: f32,
    /// §4.5 恢复速度阈值：低于它的碰撞 e 视作 0（防微弹跳）。
    pub restitution_threshold: f32,
    /// 位置修正（Baumgarte）系数。
    pub baumgarte: f32,
    /// 线性 slop（穿透容差，不参与 bias）。
    pub linear_slop: f32,
    /// 限速（M0 的防隧道保守闸；CCD 落地后放宽）。
    pub max_linear_velocity: f32,
    pub max_angular_velocity: f32,
    /// §4.11 休眠阈值：默认线性 0.04 m/s / 角速 0.05 rad/s / 计时 0.5 s。
    pub sleep_linear: f32,
    pub sleep_angular: f32,
    pub sleep_time: f32,
    /// 重力（通过力场注册表注入，见 vxl-phys-field）。
    pub gravity: Vec3,
}

impl Default for PhysConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            substeps: 1,
            velocity_iterations: 16,
            contact_skin: 0.01,
            gjk_tolerance: 1e-5,
            ccd_speed_threshold: f32::INFINITY,
            friction: FrictionModel::Coulomb { mu: 0.5 },
            restitution: 0.0,
            restitution_threshold: 1.0,
            baumgarte: 0.2,
            linear_slop: 0.005,
            max_linear_velocity: 100.0,
            max_angular_velocity: 50.0,
            sleep_linear: 0.04,
            sleep_angular: 0.05,
            sleep_time: 0.5,
            gravity: Vec3::new(0.0, -9.81, 0.0),
        }
    }
}

impl PhysConfig {
    /// §4.15 预设 → 迭代/子步/skin/摩擦。软体 α、流体族、破坏预算在各自模块。
    pub fn from_preset(p: Preset) -> Self {
        let mut c = Self::default();
        match p {
            Preset::Arcade => {
                c.velocity_iterations = 4;
                c.substeps = 1;
                c.contact_skin = 0.02;
                c.friction = FrictionModel::Coulomb { mu: 0.7 };
            }
            Preset::Balanced => {
                c.velocity_iterations = 8;
                c.substeps = 2;
                c.contact_skin = 0.01;
                c.friction = FrictionModel::StaticKinetic {
                    mu_s: 0.7,
                    mu_k: 0.5,
                    v_crit: 1.0,
                };
            }
            Preset::Realistic => {
                c.velocity_iterations = 16;
                c.substeps = 4;
                c.contact_skin = 0.005;
                c.friction = FrictionModel::Anisotropic {
                    mu1: 0.7,
                    mu2: 0.5,
                    angle: 0.0,
                };
            }
            Preset::Extreme => {
                c.velocity_iterations = 32;
                c.substeps = 8;
                c.contact_skin = 0.002;
                c.friction = FrictionModel::Anisotropic {
                    mu1: 0.8,
                    mu2: 0.6,
                    angle: 0.0,
                };
            }
        }
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preset_table_matches_spec_4_15() {
        let a = PhysConfig::from_preset(Preset::Arcade);
        assert_eq!(a.velocity_iterations, 4);
        assert_eq!(a.substeps, 1);
        assert!((a.contact_skin - 0.02).abs() < 1e-6);
        let e = PhysConfig::from_preset(Preset::Extreme);
        assert_eq!(e.velocity_iterations, 32);
        assert_eq!(e.substeps, 8);
        assert!((e.contact_skin - 0.002).abs() < 1e-6);
    }

    #[test]
    fn default_is_spec_default() {
        let c = PhysConfig::default();
        assert_eq!(c.velocity_iterations, 16);
        assert_eq!(c.sleep_linear, 0.04);
        assert_eq!(c.sleep_angular, 0.05);
        assert_eq!(c.sleep_time, 0.5);
        assert_eq!(c.restitution_threshold, 1.0);
    }
}
