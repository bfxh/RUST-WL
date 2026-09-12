//! # vxl-phys-fluid
//!
//! 流体（§4.8，按技术族分档）—— M3 CPU 档 / M4 GPU 档。参数骨架先行。
//!
//! - CPU SPH：WCSPH + XSPH 黏度（Monaghan 核）；
//! - GPU PBF：Müller 2013，poly6/spiky 核，密度约束迭代 3~4；
//! - GPU FLIP/APIC：PIC/FLIP 传输 + APIC 仿射速度，稀疏哈希网格（512³ 活跃格）；
//! - 刚体耦合：Akinci 边界粒子两层（体积/压力）。

#![forbid(unsafe_code)]

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

#[derive(Clone, Debug)]
pub struct FluidConfig {
    pub family: FluidFamily,
    pub density_iterations: DensityIterations,
    /// 静止密度 ρ0（kg/m³，水 = 1000）。
    pub rest_density: f32,
    /// 核半径 h。
    pub smoothing_radius: f32,
    /// XSPH 黏度系数（SPH 族）。
    pub xsph_viscosity: f32,
    /// 张力不稳定性抑制（spiky 梯度）。
    pub tensile_instability_suppression: bool,
}

impl Default for FluidConfig {
    fn default() -> Self {
        Self {
            family: FluidFamily::CpuSph,
            density_iterations: DensityIterations::Three,
            rest_density: 1000.0,
            smoothing_radius: 0.1,
            xsph_viscosity: 0.1,
            tensile_instability_suppression: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_cpu_sph() {
        let c = FluidConfig::default();
        assert_eq!(c.family, FluidFamily::CpuSph);
        assert_eq!(c.rest_density, 1000.0);
    }
}
