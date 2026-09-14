//! # vxl-phys-aero
//!
//! 气动域（面元气动力）—— M2+ 落地。
//! 每三角面：F = ½ρ·Cd·A·(v_wind − v_tri)·|v_rel|（Bridson 线化气动力，与布料 §4.7 同通道）。

#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug)]
pub struct AeroConfig {
    /// 空气密度 kg/m³。
    pub air_density: f32,
    /// 面法向阻力系数。
    pub drag_coefficient: f32,
    /// 升力线斜率（简化薄翼）。
    pub lift_slope: f32,
    pub wind: [f32; 3],
}

impl Default for AeroConfig {
    fn default() -> Self {
        Self {
            air_density: 1.225,
            drag_coefficient: 1.0,
            lift_slope: 5.0,
            wind: [0.0, 0.0, 0.0],
        }
    }
}
