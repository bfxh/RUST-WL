//! system：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// WCSPH 粒子流体系统（SoA；零外部依赖）。
pub struct FluidSystem {
    pub(crate) cfg: FluidConfig,
    pub(crate) h: f32,
    pub(crate) h2: f32,
    /// poly6 系数 315/(64πh⁹)。
    pub(crate) k6: f32,
    /// spiky 梯度幅系数 45/(πh⁶)。
    pub(crate) ks: f32,
    /// W(0) = poly6 自身项。
    pub(crate) w0: f32,
    /// Tait 刚度 B = c²ρ0/γ。
    pub(crate) b_tait: f32,
    /// 单粒质量（晶格标定：m = ρ0 / Σ_lattice W，见 `new`）。
    pub(crate) mass: f32,
    /// 边界接触带（粒子中心距表面 < skin 触发投影）。
    pub(crate) skin: f32,
    /// 边界提供者 id（统一 id 空间；静态固体）。
    pub(crate) boundaries: Vec<u32>,
    pub(crate) pos: Vec<Vec3>,
    pub(crate) vel: Vec<Vec3>,
    pub(crate) dens: Vec<f32>,
    pub(crate) press: Vec<f32>,
    pub(crate) acc: Vec<Vec3>,
    pub(crate) xsph: Vec<Vec3>,
    pub(crate) grid: UniformGrid,
    /// 接触缓冲（复用，免每粒分配）。
    pub(crate) contacts: Vec<InteropContact>,
    /// 晶格间距（`new` 给定；边界粒子的采样间距与体积标定同源于它）。
    pub(crate) spacing: f32,
    /// **流体粒子数**：`pos`/`vel`/... 的**前缀**长度；`≥ n_fluid` 的是边界粒子
    /// （同数组 ⇒ 既有核/式一字不改地作用于边界粒子，见模块头）。
    pub(crate) n_fluid: usize,
    /// 逐粒质量：流体 = `mass`，边界 = `ρ0·V_b`。流体段取同一个 f32 ⇒ 无边界时
    /// 与旧的常量乘法逐位等价。
    pub(crate) pmass: Vec<f32>,
    /// 每体边界段 `(体 id, 体原点, start, end)`（`start..end` = 全局粒子索引区间）。
    pub(crate) spans: Vec<(u32, Vec3, u32, u32)>,
    /// 反作用输出：每体 `(体 id, 力, 绕体原点的力矩)`；每个子步末整体重写。
    pub(crate) breact: Vec<(u32, Vec3, Vec3)>,
    /// 每边界粒子的受力累加（每子步清零；`force_pass` 里借出以便写入）。
    pub(crate) bforce: Vec<Vec3>,
    /// 形状 → 局部两层采样缓存（形状集小 ⇒ 线性查找；免逐 tick 重建）。
    pub(crate) lattice_cache: Vec<(Shape, BoundaryLattice)>,
    /// **相位计时累加器**（微秒；仅诊断，不改行为）：网格/密度/压力/力/积分+边界。
    /// 口径：每子步各相位一次 `probe::us`（`vxl-phys-core::probe`，wasm 下退化为 0）。
    pub(crate) phase_us: [u64; 5],
}
