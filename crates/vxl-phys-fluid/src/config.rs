//! config：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// WCSPH 参数（PLAN-0.3 §1）。
#[derive(Clone, Debug)]
pub struct FluidConfig {
    pub family: FluidFamily,
    pub density_iterations: DensityIterations,
    /// 静止密度 ρ0（kg/m³，水 = 1000）。
    pub rest_density: f32,
    /// 核半径 h。
    pub smoothing_radius: f32,
    /// XSPH 黏度系数 ε（速度平滑；晶格标定下权重和 ≈ 1）。
    pub xsph_viscosity: f32,
    /// 张力不稳定性抑制（spiky 梯度；CPU 档恒用 spiky，保留为口径标记）。
    pub tensile_instability_suppression: bool,
    /// 声速 c（m/s）⇒ Tait 刚度 B = c²ρ0/γ。
    pub sound_speed: f32,
    /// Tait 指数 γ（默认 7）。
    pub gamma_tait: f32,
    /// 每 tick 的子步数。
    pub substeps: u32,
    /// **相位并行线程数**（1 = 串行，默认）。
    ///
    /// 并行对象 = 逐粒独立的相位：**密度**与**力+黏度**（`phase_us` 实测这两项占
    /// 30 万粒档的 **96%**：密度 32% / 力 64%）。**逐位一致契约**：每个粒子的邻域
    /// 遍历序不随分块变化（见 `UniformGrid::for_neighbors_in`）⇒ 并行结果与串行
    /// **逐位相同**（不是"近似相同"）；唯一跨粒子的量 `bforce`（2b 反作用）在有
    /// 边界粒子时走**串行补趟**，同样保逐位。
    /// ⇒ 默认 1 时**完全不进入并行路径**（与旧行为逐位一致）。
    pub threads: usize,
    /// **2b 边界粒子的层数**（默认 2；SPEC §4.8 的两层形态）。
    /// 2026-09-22 起可调：实测用于"层数 ↑ ⇒ 反作用波动 ↓"这条候选
    /// （波动会透传到体上——漂浮体 y 峰峰 19.9 mm、|ω| 峰峰 1.22 rad/s，见
    /// `crates/vxl-phys/tests/float_quiet_probe.rs`）。
    pub boundary_layers: u32,
    /// 重力（m/s²）。
    pub gravity: Vec3,
    /// CFL 限速：|v| ≤ frac·h/dt（防穿隧/防爆炸）。
    pub max_speed_frac: f32,
    /// Monaghan 人工黏度 α（Π = α·c·μ/ρ̄，仅作用于接近对）：
    /// 耗散法向压缩波——没有它，钉在边界上的底层是个无阻尼硬弹簧，
    /// 柱体落位时反弹成尘（互穿死区 + 飞散），静水态永远建不起来。
    pub artificial_viscosity: f32,
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
            // 声速 c：声学 CFL c·dt/h ≤ ~0.5（dt = 1/(60·substeps)，h = 0.1
            // ⇒ c = 10 时 c·dt/h ≈ 0.42）；c = ρ0gH 压缩误差 ~ ρ0gH/c²。
            sound_speed: 10.0,
            gamma_tait: 7.0,
            substeps: 4,
            threads: 1,
            boundary_layers: 2,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            max_speed_frac: 0.4,
            artificial_viscosity: 1.0,
        }
    }
}

/// 均匀格总数上限；超出则格边逐级加倍粗化（粗化不减正确性：
/// r > h 的候选被核函数零剔除）。
/// **单一来源已移到 `vxl_phys_core::grid::GRID_MAX_BINS`**——GPU 侧每子步重算箱子要用同一个数。
pub(crate) use vxl_phys_core::grid::GRID_MAX_BINS;
