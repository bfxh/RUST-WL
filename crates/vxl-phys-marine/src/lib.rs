//! # vxl-phys-marine
//!
//! 船只（§1 vxl-phys-marine）：浮力采样/波浪耦合 —— M2+ 落地。

#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug)]
pub struct MarineConfig {
    /// 水密度 kg/m³。
    pub water_density: f32,
    /// 船体浮力采样点数（网格化）。
    pub buoyancy_samples: u32,
    /// 阻尼（切向/法向）。
    pub tangential_drag: f32,
    pub normal_drag: f32,
    /// Gerstner 波参数（波数/振幅/方向）。
    pub wave_amplitude: f32,
    pub wave_length: f32,
}

impl Default for MarineConfig {
    fn default() -> Self {
        Self {
            water_density: 1000.0,
            buoyancy_samples: 64,
            tangential_drag: 0.1,
            normal_drag: 1.5,
            wave_amplitude: 0.3,
            wave_length: 8.0,
        }
    }
}
