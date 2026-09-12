//! # vxl-phys-vehicle
//!
//! 车辆（§4.10，射线悬挂 + 刷子轮胎模型）—— M2 落地（404 现役参数迁入）。
//!
//! - 悬挂：轮心射线（stiffness/damping/行程三参数）；
//! - 轮胎：简化刷子模型（α_peak、载荷敏感、μ(s) 纵滑曲线）；
//! - 传动：引擎扭矩曲线 + 齿比表 + 差速（开放/限滑系数）；
//! - 档位：街机（线性轮胎 + 包络钳制）/ 真实（全刷子 + 载荷转移）。

#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehiclePreset {
    /// 线性轮胎 + 包络钳制。
    Arcade,
    /// 全刷子模型 + 载荷转移。
    Realistic,
}

/// 射线悬挂三参数。
#[derive(Clone, Copy, Debug)]
pub struct Suspension {
    pub stiffness: f32,
    pub damping: f32,
    pub travel: f32,
    pub rest_length: f32,
}

/// 刷子轮胎参数。
#[derive(Clone, Copy, Debug)]
pub struct BrushTire {
    pub peak_slip_angle: f32,
    pub load_sensitivity: f32,
    /// 纵滑摩擦曲线 μ(s) 的峰值滑移率。
    pub peak_slip_ratio: f32,
    pub mu_peak: f32,
    pub mu_slide: f32,
}

/// 传动参数。
#[derive(Clone, Copy, Debug)]
pub struct Drivetrain {
    pub gear_ratios: [f32; 6],
    pub final_drive: f32,
    /// 限滑系数（0 = 开放差速）。
    pub lsd_coefficient: f32,
    pub max_engine_torque: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct VehicleConfig {
    pub preset: VehiclePreset,
    pub suspension: Suspension,
    pub tire: BrushTire,
    pub drivetrain: Drivetrain,
    /// 漂移：侧滑角阈值 + 回正力矩项。
    pub drift_slip_angle_threshold: f32,
    pub drift_aligning_torque: f32,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            preset: VehiclePreset::Arcade,
            suspension: Suspension {
                stiffness: 30_000.0,
                damping: 3_000.0,
                travel: 0.25,
                rest_length: 0.5,
            },
            tire: BrushTire {
                peak_slip_angle: 0.15,
                load_sensitivity: 0.8,
                peak_slip_ratio: 0.15,
                mu_peak: 1.2,
                mu_slide: 0.9,
            },
            drivetrain: Drivetrain {
                gear_ratios: [3.6, 2.2, 1.5, 1.1, 0.9, 0.75],
                final_drive: 3.7,
                lsd_coefficient: 0.3,
                max_engine_torque: 400.0,
            },
            drift_slip_angle_threshold: 0.35,
            drift_aligning_torque: 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gear_ratios_monotonic_down() {
        let d = VehicleConfig::default().drivetrain;
        for w in d.gear_ratios.windows(2) {
            assert!(w[0] > w[1]);
        }
    }
}
