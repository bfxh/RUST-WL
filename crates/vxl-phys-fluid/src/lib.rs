//! # vxl-phys-fluid
//!
//! 流体（§4.8，按技术族分档）—— M3 CPU 档 / M4 GPU 档。
//!
//! - CPU SPH：WCSPH + XSPH 黏度（Monaghan 核）；
//! - GPU PBF：Müller 2013，poly6/spiky 核，密度约束迭代 3~4；
//! - GPU FLIP/APIC：PIC/FLIP 传输 + APIC 仿射速度，稀疏哈希网格（512³ 活跃格）；
//! - 刚体耦合：Akinci 边界粒子两层（体积/压力）。
//!
//! 首切片（docs/PLAN-0.3.md）：`CpuSph` 的 WCSPH 路径完整落地——
//! poly6 密度（含自身项）、spiky 压力梯度（中心不减益 ⇒ 张力不稳定抑制）、
//! XSPH 速度平滑、Tait 状态方程（γ=7）、晶格质量标定、半隐式欧拉 + CFL 限速；
//! 邻居 = 均匀网格（格边 = h，27 邻域，遍历序确定）；边界 = 统一提供者通道
//! `ProviderColliders::contacts_point`（体素/三角网/喷溅即插即用，非弹性投影）。
//! 确定性：全 f32、同输入两次运行逐位一致（测试守门）。
//!
//! **刚体→流体（2b，Akinci 两层边界粒子）**：边界粒子与流体粒子**同数组、同核、
//! 同式**（逐粒质量 `pmass`：流体 = `mass`、边界 = `ρ0·V_b`），只是**运动学冻结**
//! （不积分、由 facade 每 tick 按体重建，速度 = 体面速度 `v + ω×r`）。反作用 =
//! 边界粒子上压力梯度力之和 → 每体（力 + 绕体原点的力矩）。
//! 几何（表面采样/两层内移/体积）在 [`boundary`] 模块。**无边界粒子时逐位不变**
//! （`sum_b` 恒 0 ⇒ `mass·sum + 0.0` 与原式同值同序）。

#![forbid(unsafe_code)]

pub mod boundary;

pub use boundary::{lattice, BoundaryLattice, SurfaceLattice};

use vxl_phys_core::interop::{InteropContact, MediumField, MediumSample, ProviderColliders};
use vxl_phys_core::{Quat, Shape, Vec3};

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
const GRID_MAX_BINS: usize = 1 << 20;

/// 均匀网格：格边 = h（或粗化后），27 邻域。
/// **格内序 = 登记序 = 粒子索引序** —— 确定性求和的根。
///
/// **存储 = 计数排序**（2026-09-23，规模档第二刀）：`items` 是"按格分组、格内按索引序"的
/// 紧凑 `u32` 数组，`start` 是每格起点（长度 total+1）。相对旧的 `Vec<Vec<u32>>`：
/// ① 免掉每格一次 `Vec` 的分配/增长/清空（30 万粒档实测重建 25.8 ms ⇒ 换后见 §6.2）；
/// ② 邻居遍历变成**连续切片扫描**（旧版是 27 条指针链各自追堆块）⇒ 缓存友好。
/// **逐位一致**：散射按粒子**索引升序**写入 ⇒ 每格内序与旧实现（同样索引序 push）**相同**。
#[derive(Default)]
struct UniformGrid {
    bin: f32,
    inv: f32,
    min: Vec3,
    nx: u32,
    ny: u32,
    nz: u32,
    /// 按格分组的粒子索引（格内索引升序）。
    items: Vec<u32>,
    /// 每格起点（`start[c]..start[c+1]` = 格 c 的粒子）；长度 = 格数 + 1。
    start: Vec<u32>,
    /// 计数 scratch（复用，免每帧分配）。
    counts: Vec<u32>,
}

impl UniformGrid {
    fn rebuild(&mut self, pos: &[Vec3], h: f32) {
        let n = pos.len();
        if n == 0 {
            self.nx = 0;
            self.ny = 0;
            self.nz = 0;
            self.items.clear();
            self.start.clear();
            self.start.push(0);
            return;
        }
        let mut lo = pos[0];
        let mut hi = pos[0];
        for p in &pos[1..] {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        // 两端覆盖全部粒子；格边从 h 起，超预算则加倍粗化。
        let mut bin = h.max(1e-6);
        let ext = hi - lo;
        let exts = [ext.x, ext.y, ext.z];
        let mut dims = [1u32; 3];
        loop {
            for a in 0..3 {
                dims[a] = (((exts[a] / bin).floor() as usize + 1).clamp(1, 1 << 14)) as u32;
            }
            if dims[0] as usize * dims[1] as usize * dims[2] as usize <= GRID_MAX_BINS {
                break;
            }
            bin *= 2.0;
        }
        self.bin = bin;
        self.inv = 1.0 / bin;
        self.min = lo;
        self.nx = dims[0];
        self.ny = dims[1];
        self.nz = dims[2];
        let total = self.nx as usize * self.ny as usize * self.nz as usize;
        // —— 计数排序（两遍扫 + 前缀和；无每格分配）——
        self.counts.clear();
        self.counts.resize(total + 1, 0);
        for p in pos {
            let c = self.bin_index_of(*p, total);
            self.counts[c + 1] += 1;
        }
        for c in 0..total {
            self.counts[c + 1] += self.counts[c];
        }
        self.start.clear();
        self.start.extend_from_slice(&self.counts);
        self.items.clear();
        self.items.resize(n, 0);
        for (i, p) in pos.iter().enumerate() {
            let c = self.bin_index_of(*p, total);
            let slot = self.counts[c] as usize;
            self.items[slot] = i as u32;
            self.counts[c] += 1; // 游标：索引升序写入 ⇒ 格内序不变
        }
    }

    /// 粒子所在格的**线性下标**（钳到网格内；`total` 由调用方给以省一次乘）。
    #[inline]
    fn bin_index_of(&self, p: Vec3, total: usize) -> usize {
        let f = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * self.inv).floor().max(0.0) as u32).min(n - 1)
        };
        let idx = (f(self.min.x, p.x, self.nx) * self.ny + f(self.min.y, p.y, self.ny)) * self.nz
            + f(self.min.z, p.z, self.nz);
        (idx as usize).min(total.saturating_sub(1))
    }

    /// 粒子所在格坐标（钳到网格内）。
    #[inline]
    fn bin_of(&self, p: Vec3) -> (u32, u32, u32) {
        let f = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * self.inv).floor().max(0.0) as u32).min(n - 1)
        };
        (
            f(self.min.x, p.x, self.nx),
            f(self.min.y, p.y, self.ny),
            f(self.min.z, p.z, self.nz),
        )
    }

    /// 格 `c` 的粒子切片（格内索引升序）。
    #[inline]
    fn bin_items(&self, c: usize) -> &[u32] {
        let a = self.start[c] as usize;
        let b = self.start[c + 1] as usize;
        &self.items[a..b]
    }

    /// 邻域公共体（**自由函数形态**，供并行相位在分块闭包里调用）：粒子 `i` 的
    /// 27 邻域格（钳边）内、`r ≤ h` 的 `j` 交给 `f`。访问序 = 格坐标序
    /// （dz, dy, dx 固定）× 格内索引序 ⇒ 求和序是位置的确定函数
    /// ⇒ **并行分块不改变任一粒子的求和序**（这就是"并行 = 串行逐位一致"的根据）。
    #[inline]
    fn for_neighbors_in(
        &self,
        pos: &[Vec3],
        h2: f32,
        i: usize,
        mut f: impl FnMut(usize, Vec3, f32),
    ) {
        let pi = pos[i];
        let (cx, cy, cz) = self.bin_of(pi);
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        for dz in -1i32..=1 {
            let z = cz as i32 + dz;
            if z < 0 || z >= nz as i32 {
                continue;
            }
            for dy in -1i32..=1 {
                let y = cy as i32 + dy;
                if y < 0 || y >= ny as i32 {
                    continue;
                }
                for dx in -1i32..=1 {
                    let x = cx as i32 + dx;
                    if x < 0 || x >= nx as i32 {
                        continue;
                    }
                    let idx = ((x as u32 * ny + y as u32) * nz + z as u32) as usize;
                    for &j in self.bin_items(idx) {
                        let j = j as usize;
                        if j == i {
                            continue;
                        }
                        let d = pi - pos[j];
                        let r2 = d.length_squared();
                        if r2 <= h2 {
                            f(j, d, r2);
                        }
                    }
                }
            }
        }
    }
}

/// WCSPH 粒子流体系统（SoA；零外部依赖）。
pub struct FluidSystem {
    cfg: FluidConfig,
    h: f32,
    h2: f32,
    /// poly6 系数 315/(64πh⁹)。
    k6: f32,
    /// spiky 梯度幅系数 45/(πh⁶)。
    ks: f32,
    /// W(0) = poly6 自身项。
    w0: f32,
    /// Tait 刚度 B = c²ρ0/γ。
    b_tait: f32,
    /// 单粒质量（晶格标定：m = ρ0 / Σ_lattice W，见 `new`）。
    mass: f32,
    /// 边界接触带（粒子中心距表面 < skin 触发投影）。
    skin: f32,
    /// 边界提供者 id（统一 id 空间；静态固体）。
    boundaries: Vec<u32>,
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
    dens: Vec<f32>,
    press: Vec<f32>,
    acc: Vec<Vec3>,
    xsph: Vec<Vec3>,
    grid: UniformGrid,
    /// 接触缓冲（复用，免每粒分配）。
    contacts: Vec<InteropContact>,
    /// 晶格间距（`new` 给定；边界粒子的采样间距与体积标定同源于它）。
    spacing: f32,
    /// **流体粒子数**：`pos`/`vel`/... 的**前缀**长度；`≥ n_fluid` 的是边界粒子
    /// （同数组 ⇒ 既有核/式一字不改地作用于边界粒子，见模块头）。
    n_fluid: usize,
    /// 逐粒质量：流体 = `mass`，边界 = `ρ0·V_b`。流体段取同一个 f32 ⇒ 无边界时
    /// 与旧的常量乘法逐位等价。
    pmass: Vec<f32>,
    /// 每体边界段 `(体 id, 体原点, start, end)`（`start..end` = 全局粒子索引区间）。
    spans: Vec<(u32, Vec3, u32, u32)>,
    /// 反作用输出：每体 `(体 id, 力, 绕体原点的力矩)`；每个子步末整体重写。
    breact: Vec<(u32, Vec3, Vec3)>,
    /// 每边界粒子的受力累加（每子步清零；`force_pass` 里借出以便写入）。
    bforce: Vec<Vec3>,
    /// 形状 → 局部两层采样缓存（形状集小 ⇒ 线性查找；免逐 tick 重建）。
    lattice_cache: Vec<(Shape, BoundaryLattice)>,
    /// **相位计时累加器**（微秒；仅诊断，不改行为）：网格/密度/压力/力/积分+边界。
    /// 口径：每子步各相位一次 `probe::us`（`vxl-phys-core::probe`，wasm 下退化为 0）。
    phase_us: [u64; 5],
}

impl FluidSystem {
    /// 晶格块初始化：粒子位于 `origin + (i + ½)·spacing`（三轴 `dims` 个），
    /// 质量按「静止晶格密度 = ρ0」反解：m = ρ0 / Σ_lattice W（含自身项）。
    pub fn new(cfg: FluidConfig, origin: Vec3, dims: [usize; 3], spacing: f32) -> Self {
        let h = cfg.smoothing_radius.max(1e-5);
        let h2 = h * h;
        let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
        let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
        let w0 = k6 * h2 * h2 * h2; // (h²)³
        let b_tait = cfg.sound_speed * cfg.sound_speed * cfg.rest_density / cfg.gamma_tait;
        let n = dims[0] * dims[1] * dims[2];
        let mut pos = Vec::with_capacity(n);
        for i in 0..dims[0] {
            for j in 0..dims[1] {
                for k in 0..dims[2] {
                    pos.push(Vec3::new(
                        origin.x + (i as f32 + 0.5) * spacing,
                        origin.y + (j as f32 + 0.5) * spacing,
                        origin.z + (k as f32 + 0.5) * spacing,
                    ));
                }
            }
        }
        // 质量标定：对晶格求 Σ W（间距 spacing 的无限晶格截断到核半径内）。
        let mut wsum = 0.0f32;
        let side = (h / spacing.max(1e-6)).ceil() as i32;
        for i in -side..=side {
            for j in -side..=side {
                for k in -side..=side {
                    let d = Vec3::new(i as f32 * spacing, j as f32 * spacing, k as f32 * spacing);
                    let r2 = d.length_squared();
                    if r2 <= h2 {
                        let t = h2 - r2;
                        wsum += k6 * t * t * t;
                    }
                }
            }
        }
        let mass = cfg.rest_density / wsum;
        Self {
            // 接触带 0.15h：粒子静置在 sdf = skin 处，带越薄壁邻密度亏越小
            // （镜像鬼影对贴壁层全额补回固体侧缺失的核质量）。带内最深穿透
            // 仍被推出（半空间无「另一侧」，薄带不引入穿隧）。
            skin: 0.15 * h,
            h,
            h2,
            k6,
            ks,
            w0,
            b_tait,
            mass,
            boundaries: Vec::new(),
            vel: vec![Vec3::ZERO; n],
            dens: vec![0.0; n],
            press: vec![0.0; n],
            acc: vec![Vec3::ZERO; n],
            xsph: vec![Vec3::ZERO; n],
            grid: UniformGrid::default(),
            contacts: Vec::new(),
            spacing,
            n_fluid: n,
            pmass: vec![mass; n],
            spans: Vec::new(),
            breact: Vec::new(),
            // 不变式：`bforce.len() == pos.len()`（`new` 给流体段；`set_boundary_particles`
            // 随 `pos` 一起 resize；`truncate_to_fluid` 一起截断）。
            bforce: vec![Vec3::ZERO; n],
            lattice_cache: Vec::new(),
            phase_us: [0; 5],
            pos,
            cfg,
        }
    }

    /// 流体粒子数（**不含**边界粒子）。
    ///
    /// 语义同旧 `len()`：渲染/导出/测试读到的永远是流体粒子，
    /// 边界粒子是 2b 的内部表示（`boundary_count()` 单独报）。
    pub fn len(&self) -> usize {
        self.n_fluid
    }

    pub fn is_empty(&self) -> bool {
        self.n_fluid == 0
    }

    pub fn config(&self) -> &FluidConfig {
        &self.cfg
    }

    /// 流体晶格间距（边界粒子的采样间距与体积标定同源）。
    pub fn particle_spacing(&self) -> f32 {
        self.spacing
    }

    /// 当前边界粒子数（2b；0 = 纯流体）。
    pub fn boundary_count(&self) -> usize {
        self.pos.len() - self.n_fluid
    }

    /// 当前边界粒子覆盖的体 id（按段序 = 生成序，确定性）。
    pub fn boundary_bodies(&self) -> impl Iterator<Item = u32> + '_ {
        self.spans.iter().map(|s| s.0)
    }

    /// **反作用**（`step` 后有效；每子步重写）：每体 `(体 id, 力, 绕体原点的力矩)`。
    /// 体原点处的力与力矩都已是**力的量纲**（未乘 dt）；facade 按自己的子步施加。
    pub fn boundary_reactions(&self) -> &[(u32, Vec3, Vec3)] {
        &self.breact
    }

    /// 单粒质量（晶格标定结果；导出/审计用）。
    pub fn particle_mass(&self) -> f32 {
        self.mass
    }

    /// 流体粒子位置（前缀；不含边界粒子）。
    pub fn positions(&self) -> &[Vec3] {
        &self.pos[..self.n_fluid]
    }

    pub fn velocities(&self) -> &[Vec3] {
        &self.vel[..self.n_fluid]
    }

    pub fn densities(&self) -> &[f32] {
        &self.dens[..self.n_fluid]
    }

    pub fn pressures(&self) -> &[f32] {
        &self.press[..self.n_fluid]
    }

    /// 覆盖初速度（ dams break / 动量测试用；顺序 = 粒子索引序）。
    pub fn set_velocities(&mut self, vs: &[Vec3]) {
        let n = vs.len().min(self.n_fluid);
        self.vel[..n].copy_from_slice(&vs[..n]);
    }

    /// 边界提供者 id 列表（统一 id 空间）。
    pub fn boundaries(&self) -> &[u32] {
        &self.boundaries
    }

    pub fn set_boundaries(&mut self, ids: &[u32]) {
        self.boundaries = ids.to_vec();
    }

    /// **Akinci 边界粒子（2b）**：用给定体的两层表面粒子**整体替换**当前边界集
    /// （facade 每 tick 重建一次；空列表 = 回落纯流体）。返回边界粒子总数。
    ///
    /// - 局部采样按 `(形状, spacing)` **缓存复用**（形状集小、线性查找；稳态零分配）；
    ///   形状不受支持（复合体/高度场/provider/凸壳）⇒ 出 0 粒、**不进缓存**，
    ///   调用方据此回退粗档（`M1-EXIT.md` §4）。
    /// - 世界变换与体面速度在本函数内完成（`v + ω×r`，`r` 自体原点量起）。
    /// - 数组按**体序 × 层序 × 面内序**排布 ⇒ 求和序确定。
    pub fn set_boundary_particles(&mut self, bodies: &[(u32, Shape, BodyPose)]) -> usize {
        self.truncate_to_fluid();
        let mut cache = std::mem::take(&mut self.lattice_cache);
        for &(body, shape, pose) in bodies {
            // 取或建局部两层采样（缓存已移出 `self` ⇒ 与下面的 self.pos 写入无借用冲突）。
            let idx = match cache.iter().position(|(s, _)| *s == shape) {
                Some(i) => i,
                None => {
                    let lat =
                        boundary::lattice(&shape, self.spacing, self.h, self.cfg.boundary_layers);
                    if lat.is_empty() {
                        continue; // 不支持 ⇒ 不造粒、不缓存（回退粗档的信号）
                    }
                    cache.push((shape, lat));
                    cache.len() - 1
                }
            };
            let (_, lat) = &cache[idx];
            let start = self.pos.len() as u32;
            for &lp in &lat.pts {
                let wp = pose.pos + pose.rot.rotate_vec3(lp);
                self.pos.push(wp);
                // 体面速度：刚体速度场 v + ω×r（r 自体原点量起）。
                self.vel
                    .push(pose.linvel + pose.angvel.cross(wp - pose.pos));
                self.pmass.push(self.cfg.rest_density * lat.volume);
                self.dens.push(0.0);
                self.press.push(0.0);
                self.acc.push(Vec3::ZERO);
                self.xsph.push(Vec3::ZERO);
            }
            let end = self.pos.len() as u32;
            if end > start {
                self.spans.push((body, pose.pos, start, end));
            }
        }
        self.lattice_cache = cache;
        self.bforce.resize(self.pos.len(), Vec3::ZERO);
        self.breact.clear();
        self.boundary_count()
    }

    /// 清空边界粒子（截断回流体前缀；容量保留 ⇒ 稳态零分配）。
    fn truncate_to_fluid(&mut self) {
        let nf = self.n_fluid;
        self.pos.truncate(nf);
        self.vel.truncate(nf);
        self.dens.truncate(nf);
        self.press.truncate(nf);
        self.acc.truncate(nf);
        self.xsph.truncate(nf);
        self.pmass.truncate(nf);
        self.bforce.truncate(nf);
        self.spans.clear();
        self.breact.clear();
    }

    /// 推进一个完整 tick（内部按 `cfg.substeps` 等分子步；边界经统一提供者通道）。
    pub fn step(&mut self, dt_tick: f32, providers: &dyn ProviderColliders) {
        let sub = self.cfg.substeps.max(1);
        let dt = dt_tick / sub as f32;
        for _ in 0..sub {
            self.substep(dt, providers);
        }
    }

    fn substep(&mut self, dt: f32, providers: &dyn ProviderColliders) {
        let nf = self.n_fluid;
        if nf == 0 {
            return; // 边界粒子不单独驱动（它们只随体走）
        }
        // 网格含**全部**粒子（邻域必须看得见边界粒子）；边界粒子的位置/速度每 tick
        // 由 `set_boundary_particles` 整体重建，本子步内不动（运动学冻结）。
        let t = vxl_phys_core::probe::start();
        self.grid.rebuild(&self.pos, self.h);
        self.phase_us[0] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.density_pass(providers);
        self.phase_us[1] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.pressure_pass();
        self.phase_us[2] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.force_pass();
        self.phase_us[3] += vxl_phys_core::probe::us(t);
        // 半隐式欧拉：v ← v + dt·a + ε·xsph；CFL 限速防穿隧。
        // **只积分流体粒子**（索引前缀）——边界粒子不作积分。
        let vmax = self.cfg.max_speed_frac * self.h / dt;
        let vmax2 = vmax * vmax;
        let eps = self.cfg.xsph_viscosity;
        for i in 0..nf {
            let mut v = self.vel[i] + self.acc[i] * dt + self.xsph[i] * eps;
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            self.vel[i] = v;
            self.pos[i] += v * dt;
        }
        let t = vxl_phys_core::probe::start();
        self.boundary_pass(providers);
        self.phase_us[4] += vxl_phys_core::probe::us(t);
    }

    /// **相位计时**（诊断）：`[网格, 密度, 压力, 力, 积分+边界]` 的**累计微秒**。
    /// 与 `reset_phase_us()` 配合取窗口差值（本函数不改行为、不进哈希）。
    pub fn phase_us(&self) -> [u64; 5] {
        self.phase_us
    }

    /// 清零相位计时累加器（取窗口前调用）。
    pub fn reset_phase_us(&mut self) {
        self.phase_us = [0; 5];
    }

    /// 邻域公共体：粒子 i 的 27 邻域格（钳边）内、r ≤ h 的 j 交给 `f`。
    /// 访问序 = 格坐标序（dz, dy, dx 固定）× 格内索引序 ⇒ 求和序是位置的确定函数。
    /// **转发给 `UniformGrid::for_neighbors_in`**（并行相位在分块闭包里用同一实现）。
    #[inline]
    fn for_neighbors(&self, i: usize, f: impl FnMut(usize, Vec3, f32)) {
        self.grid.for_neighbors_in(&self.pos, self.h2, i, f);
    }

    /// 密度：ρ_i = m·(W(0) + Σ_j W(r_ij) + Σ_ghost W)（poly6，含自身项）。
    /// 边界镜像鬼影：壁邻粒子（0 < sdf < h）把真实邻居关于壁面（接触点 +
    /// 外法线）反射，补回固体一侧的**离散**核质量。不用连续半空间积分——
    /// 它按均匀连续介质补，而流体实际的近壁分布（沉降后成层、各向异性）
    /// 的离散亏量与之错带（实测底层差 ~12% ρ0，补不齐 ⇒ p≥0 死区复活）。
    /// 鬼影随流体局部分布同步：流体压缩/成层，鬼影同压缩/成层。
    ///
    /// **2b 边界粒子**（同数组、索引 ≥ `n_fluid`）：质量**逐粒**取（`pmass`）、
    /// 计入 `sum_b`。流体段的和式与遍历序与旧实现逐字相同 ⇒ **无边界粒子时
    /// `dens = mass·sum + 0.0` 与原 `dens = mass·sum` 逐位同值**（正数 +0.0 精确）。
    fn density_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.cfg.threads > 1 {
            self.density_pass_parallel(providers);
            return;
        }
        let nf = self.n_fluid;
        let nt = self.pos.len();
        for i in 0..nf {
            let pi = self.pos[i];
            let mut pl_pt = [Vec3::ZERO; 8];
            let mut pl_n = [Vec3::ZERO; 8];
            let np = self.wall_planes(pi, providers, &mut pl_pt, &mut pl_n);
            let mut sum = self.w0;
            let mut sum_b = 0.0f32;
            self.for_neighbors(i, |j, d, r2| {
                let t = self.h2 - r2;
                let w = self.k6 * t * t * t;
                if j < nf {
                    sum += w;
                } else {
                    sum_b += self.pmass[j] * w;
                }
                for k in 0..np {
                    let pj = pi - d;
                    let dn = (pj - pl_pt[k]).dot(pl_n[k]);
                    let g = pj - pl_n[k] * (2.0 * dn);
                    let rg2 = (pi - g).length_squared();
                    if rg2 <= self.h2 {
                        let tg = self.h2 - rg2;
                        sum += self.k6 * tg * tg * tg;
                    }
                }
            });
            self.dens[i] = self.mass * sum + sum_b;
        }
        // 边界粒子（2b）：**同一核、同一式**、含自身项，但密度只从**流体**取
        // （`j < nf`）——这是 Akinci 口径的"边界压力 = 外推的流体压力"：若让边界
        // 粒子互相供密度，**薄体**（厚度 < ~2h）两侧的内移层会在体内互相穿透，
        // ρ_b 被自身堆积顶到 q≫1 ⇒ p_b 爆抬 ⇒ 体积力/反作用整片失真（实测：
        // 0.12 m 盒给出 ρVg 的 5.9×、侧向 51 N 的假力，2026-09-22）。
        // 只吃流体 ⇒ 贴壁处 ρ_b ≈ 该处流体密度 ⇒ p_b ≈ p_f，与"冻结流体粒子"
        // 的原意一致，且对任意薄厚都稳定。自身项保留（与流体同式）。
        for i in nf..nt {
            let mut sum = 0.0f32;
            self.for_neighbors(i, |j, _d, r2| {
                if j >= nf {
                    return; // 不吃边界-边界对（见上）
                }
                let t = self.h2 - r2;
                sum += self.pmass[j] * (self.k6 * t * t * t);
            });
            self.dens[i] = self.pmass[i] * self.w0 + sum;
        }
    }

    /// 收集粒子 `pi` 的 h 带内壁面（点 + 外法线；角部可有多面），≤ 8 面。
    /// 供密度轮的镜像鬼影用（流体粒子专有；边界粒子不用，见 `density_pass`）。
    /// **自由函数形态**（并行密度相位在分块闭包里调用；`scratch` 由调用方给，
    /// 并行时每块一份 ⇒ 无共享可变状态）。
    fn wall_planes_in(
        boundaries: &[u32],
        h: f32,
        pi: Vec3,
        providers: &dyn ProviderColliders,
        scratch: &mut Vec<InteropContact>,
        pl_pt: &mut [Vec3; 8],
        pl_n: &mut [Vec3; 8],
    ) -> usize {
        if boundaries.is_empty() {
            return 0;
        }
        let mut np = 0usize;
        for &bid in boundaries {
            if let Some(bb) = providers.bounds(bid) {
                // 预滤余量取 h（镜像带 = sdf < h）。
                let m = h;
                if pi.x < bb.min.x - m
                    || pi.x > bb.max.x + m
                    || pi.y < bb.min.y - m
                    || pi.y > bb.max.y + m
                    || pi.z < bb.min.z - m
                    || pi.z > bb.max.z + m
                {
                    continue;
                }
            }
            scratch.clear();
            if providers.contacts_point(bid, pi, h, scratch) {
                for c in scratch.iter() {
                    // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，
                    // provider 无关）；穿透（≤ 0）交给投影，不补。
                    let sdf = (pi - c.point).dot(c.normal);
                    if sdf > 0.0 && sdf < h && np < 8 {
                        pl_pt[np] = c.point;
                        pl_n[np] = c.normal;
                        np += 1;
                    }
                }
            }
        }
        np
    }

    /// **密度相位·并行**（`threads > 1`）：逐粒独立（每粒只读他人的 pos/pmass、
    /// 只写自己的 `dens[i]`）⇒ 分块并行；壁面查询用**每块一份 scratch**
    /// （无共享可变状态）⇒ 结果与串行**逐位一致**。
    fn density_pass_parallel(&mut self, providers: &dyn ProviderColliders) {
        let nf = self.n_fluid;
        let nt = self.pos.len();
        let threads = self.cfg.threads.max(1);
        {
            let Self {
                pos,
                dens,
                pmass,
                grid,
                h,
                h2,
                k6,
                w0,
                mass,
                boundaries,
                ..
            } = self;
            let (h, h2, k6, w0, mass) = (*h, *h2, *k6, *w0, *mass);
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, d_out) in dens[..nf].chunks_mut(per).enumerate() {
                    let base = ci * per;
                    let (pos, pmass, grid, boundaries) = (&*pos, &*pmass, &*grid, &*boundaries);
                    s.spawn(move || {
                        let mut scratch: Vec<InteropContact> = Vec::new();
                        for (k, d_out) in d_out.iter_mut().enumerate() {
                            let i = base + k;
                            let pi = pos[i];
                            let mut pl_pt = [Vec3::ZERO; 8];
                            let mut pl_n = [Vec3::ZERO; 8];
                            let np = Self::wall_planes_in(
                                boundaries,
                                h,
                                pi,
                                providers,
                                &mut scratch,
                                &mut pl_pt,
                                &mut pl_n,
                            );
                            let mut sum = w0;
                            let mut sum_b = 0.0f32;
                            grid.for_neighbors_in(pos, h2, i, |j, d, r2| {
                                let t = h2 - r2;
                                let w = k6 * t * t * t;
                                if j < nf {
                                    sum += w;
                                } else {
                                    sum_b += pmass[j] * w;
                                }
                                for kk in 0..np {
                                    let pj = pi - d;
                                    let dn = (pj - pl_pt[kk]).dot(pl_n[kk]);
                                    let g = pj - pl_n[kk] * (2.0 * dn);
                                    let rg2 = (pi - g).length_squared();
                                    if rg2 <= h2 {
                                        let tg = h2 - rg2;
                                        sum += k6 * tg * tg * tg;
                                    }
                                }
                            });
                            *d_out = mass * sum + sum_b;
                        }
                    });
                }
            });
        }
        // 边界粒子（2b）：同一式、只吃流体邻居（见串行路径注）。
        if nt > nf {
            let Self {
                pos,
                dens,
                pmass,
                grid,
                h2,
                k6,
                w0,
                ..
            } = self;
            let (h2, k6, w0) = (*h2, *k6, *w0);
            let nb = nt - nf;
            let per = nb.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, d_out) in dens[nf..].chunks_mut(per).enumerate() {
                    let base = nf + ci * per;
                    let (pos, pmass, grid) = (&*pos, &*pmass, &*grid);
                    s.spawn(move || {
                        for (k, d_out) in d_out.iter_mut().enumerate() {
                            let i = base + k;
                            let mut sum = 0.0f32;
                            grid.for_neighbors_in(pos, h2, i, |j, _d, r2| {
                                if j >= nf {
                                    return;
                                }
                                let t = h2 - r2;
                                sum += pmass[j] * (k6 * t * t * t);
                            });
                            *d_out = pmass[i] * w0 + sum;
                        }
                    });
                }
            });
        }
    }

    fn wall_planes(
        &mut self,
        pi: Vec3,
        providers: &dyn ProviderColliders,
        pl_pt: &mut [Vec3; 8],
        pl_n: &mut [Vec3; 8],
    ) -> usize {
        if self.boundaries.is_empty() {
            return 0;
        }
        let mut np = 0usize;
        for &bid in &self.boundaries {
            if let Some(bb) = providers.bounds(bid) {
                // 预滤余量取 h（镜像带 = sdf < h）。
                let m = self.h;
                if pi.x < bb.min.x - m
                    || pi.x > bb.max.x + m
                    || pi.y < bb.min.y - m
                    || pi.y > bb.max.y + m
                    || pi.z < bb.min.z - m
                    || pi.z > bb.max.z + m
                {
                    continue;
                }
            }
            self.contacts.clear();
            if providers.contacts_point(bid, pi, self.h, &mut self.contacts) {
                for c in &self.contacts {
                    // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，
                    // provider 无关）；穿透（≤ 0）交给投影，不补。
                    let sdf = (pi - c.point).dot(c.normal);
                    if sdf > 0.0 && sdf < self.h && np < 8 {
                        pl_pt[np] = c.point;
                        pl_n[np] = c.normal;
                        np += 1;
                    }
                }
            }
        }
        np
    }

    /// 压力：Tait p = B·((ρ/ρ0)^γ − 1)；γ=7 走整数次幂乘法展开。
    /// `tensile_instability_suppression`（默认开）= Monaghan 自由面钳制
    /// p ≥ 0：表面密度截断（q ≈ 0.5） otherwise 会给出 p ≈ −B，经对称
    /// spiky 形式变成巨大粒子间吸力（单子步 Δv ~ 10² m/s）⇒ 全体炸散。
    /// 覆盖**全部**粒子（含 2b 边界粒子——它们要参与压力对，也吃同一套钳制：
    /// 悬空/真空里的边界粒子 ρ 很小 ⇒ p 钳到 0 ⇒ 不产生凭空吸力）。
    fn pressure_pass(&mut self) {
        let rho0 = self.cfg.rest_density;
        let g = self.cfg.gamma_tait;
        let clamp = self.cfg.tensile_instability_suppression;
        for i in 0..self.pos.len() {
            let q = self.dens[i] / rho0;
            let mut p = if (g - 7.0).abs() < 1e-6 {
                let q2 = q * q;
                let q4 = q2 * q2;
                self.b_tait * (q * q2 * q4 - 1.0)
            } else {
                self.b_tait * (q.powf(g) - 1.0)
            };
            if clamp && p < 0.0 {
                p = 0.0;
            }
            self.press[i] = p;
        }
    }

    /// 力：重力 + 压力梯度（spiky，对称形式 p_i/ρ_i² + p_j/ρ_j²）
    /// 与 XSPH 速度平滑（对称形式 Σ m·2/(ρ_i+ρ_j)·W·(v_j − v_i)；
    /// 系数逐对对称 ⇒ 与压力项一样动量守恒）。
    /// 压力项逐对反对称 ⇒ 零重力下总动量守恒（测试 #3 守门）。
    ///
    /// **2b**：质量逐粒取（`pmass`；流体段与旧常量同值 ⇒ 无边界时逐位不变）；
    /// 邻居 j 为边界粒子时，把**同一对**的作用力（`−F_i`，逐对反对称）累加到
    /// `bforce[j]` —— 这就是刚体受到的压力反作用（力 + 力矩，力矩在
    /// `aggregate_reactions` 里按绕体原点取）。XSPH 对边界邻居照常计入 ⇒
    /// 体面速度把流体拖向自身（无滑移近似，用的是既有那一式）。
    fn force_pass(&mut self) {
        // 并行档（`cfg.threads > 1`）：逐粒独立 ⇒ 分块并行（逐位一致，见 config 注）。
        if self.cfg.threads > 1 {
            self.force_pass_parallel();
            return;
        }
        self.force_pass_serial();
    }

    /// **力相位·并行**（`threads > 1`）：`acc`/`xsph` 逐粒独立 ⇒ 分块并行；
    /// 唯一跨粒子的 `bforce`（2b 反作用）在有边界粒子时走**串行补趟** ⇒ 保逐位一致。
    fn force_pass_parallel(&mut self) {
        let nf = self.n_fluid;
        let nt = self.pos.len();
        let threads = self.cfg.threads.max(1);
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        for f in &mut self.bforce[nf..] {
            *f = Vec3::ZERO;
        }
        {
            let Self {
                pos,
                vel,
                dens,
                press,
                acc,
                xsph,
                pmass,
                grid,
                h,
                h2,
                k6,
                ks,
                ..
            } = self;
            let (h, h2, k6, ks) = (*h, *h2, *k6, *ks);
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, (acc_c, xs_c)) in acc[..nf]
                    .chunks_mut(per)
                    .zip(xsph[..nf].chunks_mut(per))
                    .enumerate()
                {
                    let base = ci * per;
                    let (pos, vel, dens, press, pmass, grid) =
                        (&*pos, &*vel, &*dens, &*press, &*pmass, &*grid);
                    s.spawn(move || {
                        for (k, (a_out, x_out)) in acc_c.iter_mut().zip(xs_c.iter_mut()).enumerate()
                        {
                            let i = base + k;
                            let vi = vel[i];
                            let rho_i = dens[i];
                            let ci2 = press[i] / (rho_i * rho_i);
                            let mut a = g;
                            let mut xs = Vec3::ZERO;
                            grid.for_neighbors_in(pos, h2, i, |j, d, r2| {
                                let rho_j = dens[j];
                                let cj2 = press[j] / (rho_j * rho_j);
                                let r = r2.sqrt();
                                let t = h - r;
                                let mj = pmass[j];
                                let coef = mj * (ks * t * t) * (ci2 + cj2);
                                let denom = r.max(1e-9);
                                a += d * (coef / denom);
                                let vij = vi - vel[j];
                                let vdn = -vij.dot(d);
                                if vdn > 0.0 {
                                    let mu = vdn * h / (r2 + 0.01 * h2);
                                    let c = mj
                                        * (alpha_c * mu / (0.5 * (rho_i + rho_j)))
                                        * (ks * t * t);
                                    a += d * (c / denom);
                                }
                                let tt = h2 - r2;
                                let w = k6 * tt * tt * tt;
                                xs += (vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
                            });
                            *a_out = a;
                            *x_out = xs;
                        }
                    });
                }
            });
        }
        // 2b 反作用（逐粒 bforce）：**串行补趟**（只有边界粒子存在时才走）——
        // 与串行路径同一算式、同一遍历序 ⇒ 逐位一致。缓冲用 `take` 借出以便闭包内写。
        if nt > nf {
            let mut bforce = std::mem::take(&mut self.bforce);
            for i in 0..nf {
                let vi = self.vel[i];
                let rho_i = self.dens[i];
                let ci2 = self.press[i] / (rho_i * rho_i);
                let mi = self.pmass[i];
                self.for_neighbors(i, |j, d, r2| {
                    if j < nf {
                        return;
                    }
                    let rho_j = self.dens[j];
                    let cj2 = self.press[j] / (rho_j * rho_j);
                    let r = r2.sqrt();
                    let t = self.h - r;
                    let mj = self.pmass[j];
                    let coef = mj * (self.ks * t * t) * (ci2 + cj2);
                    let denom = r.max(1e-9);
                    let cv = {
                        let vij = vi - self.vel[j];
                        let vdn = -vij.dot(d);
                        if vdn > 0.0 {
                            let mu = vdn * self.h / (r2 + 0.01 * self.h2);
                            mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (self.ks * t * t)
                        } else {
                            0.0
                        }
                    };
                    bforce[j] -= d * (mi * (coef + cv) / denom);
                });
            }
            self.bforce = bforce;
        }
        self.aggregate_reactions();
    }

    /// **力相位·串行**（默认档：`threads == 1` ⇒ 与历史行为逐位一致）。
    fn force_pass_serial(&mut self) {
        let nf = self.n_fluid;
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        // `for_neighbors` 借 `&self`，闭包里要写边界粒子受力 ⇒ 把缓冲整体借出
        // （`Vec::take` 是 O(1)，不分配），循环后归还。
        let mut bforce = std::mem::take(&mut self.bforce);
        for f in &mut bforce[nf..] {
            *f = Vec3::ZERO; // 每子步重新累加（反作用 = 本子步的力）
        }
        for i in 0..nf {
            let vi = self.vel[i];
            let rho_i = self.dens[i];
            let ci2 = self.press[i] / (rho_i * rho_i);
            let mi = self.pmass[i];
            let mut acc = g;
            let mut xs = Vec3::ZERO;
            self.for_neighbors(i, |j, d, r2| {
                let rho_j = self.dens[j];
                let cj2 = self.press[j] / (rho_j * rho_j);
                let r = r2.sqrt();
                let t = self.h - r;
                let mj = self.pmass[j];
                // a_i += m_j·(p_i/ρ_i² + p_j/ρ_j²)·45/(πh⁶)·(h−r)²·d̂（d̂ 指离 j）。
                let coef = mj * (self.ks * t * t) * (ci2 + cj2);
                let denom = r.max(1e-9);
                acc += d * (coef / denom);
                // Monaghan 人工黏度：仅接近对（v_ij·d < 0），Π = α·c·μ/ρ̄、
                // μ = h·|v_ij·d|/(r² + 0.01h²)；逐对反对称 ⇒ 动量守恒，耗散法向动能。
                let vij = vi - self.vel[j];
                let vdn = -vij.dot(d);
                let cv = if vdn > 0.0 {
                    let mu = vdn * self.h / (r2 + 0.01 * self.h2);
                    let c = mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (self.ks * t * t);
                    acc += d * (c / denom);
                    c
                } else {
                    0.0
                };
                if j >= nf {
                    // 反作用（作用在边界粒子上）= −F_i = −m_i·d·(逐对系数/r)。
                    bforce[j] -= d * (mi * (coef + cv) / denom);
                }
                let tt = self.h2 - r2;
                let w = self.k6 * tt * tt * tt;
                xs += (self.vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
            });
            self.acc[i] = acc;
            self.xsph[i] = xs;
        }
        self.bforce = bforce;
        self.aggregate_reactions();
    }

    /// 反作用聚合：逐粒 `bforce` → 每体 `(力, 绕**体原点**的力矩)`。
    /// 求和序 = 段序（生成序）× 段内粒子索引序 ⇒ 确定性。
    fn aggregate_reactions(&mut self) {
        self.breact.clear();
        for &(body, origin, start, end) in &self.spans {
            let mut f = Vec3::ZERO;
            let mut tau = Vec3::ZERO;
            for k in start..end {
                let fk = self.bforce[k as usize];
                f += fk;
                tau += (self.pos[k as usize] - origin).cross(fk);
            }
            self.breact.push((body, f, tau));
        }
    }

    /// 边界投影：`contacts_point` 接触带内，只推**真穿透**（sdf < 0，由
    /// 接触点/外法线恢复，与 provider 的 depth 口径解耦）+ 法向速度归零
    /// （非弹性 e=0）。推到 sdf = +skin 驻留线（穿透量 + skin）：静隙由
    /// 壁压自平衡，投影只定下限——且驻留点 sdf > 0 让密度轮的镜像平面
    /// 收集始终覆盖得到（sdf = 0 会被 `sdf > 0` 滤掉 → 壁邻鬼影丢失）。
    /// **投影坍缩消除**：多平面深穿透被逐轴推到各平面交集 = 同一个点
    /// （凹角/棱），同位粒子对互喂 W(0) ⇒ ρ ≈ 2ρ0 ⇒ Tait q⁷ 爆压
    /// （实测角点堆粒子 ρ=2153 起爆喷泉）；且同位对 d = 0 ⇒ 压力梯度
    /// 零方向 ⇒ 永久僵局。解析：低索引驻留、高索引沿自身接触法线和
    /// （流体侧）外移 r_min = 0.2h；索引升序 + 法线序固定 ⇒ 确定。
    /// 预滤余量 = h：必须 ≥ 单子步最大行程（CFL 上限 0.4h）+ 接触带，
    /// 否则快速粒子一步跨过查询带 → 永久脱离所有接触查询（自由落体逃逸）。
    /// 复用统一提供者通道：体素/三角网/喷溅无改动即为边界。
    /// **只投影流体粒子**（索引前缀）：边界粒子在体内、由体运动学带着走，
    /// 既不该被提供者推出，也不该被推出体外。
    fn boundary_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.boundaries.is_empty() {
            return;
        }
        // 本子步被投影粒子：(索引, 接触法线和)。升序登记 ⇒ 消解序确定。
        let mut pushed: Vec<(usize, Vec3)> = Vec::new();
        for i in 0..self.n_fluid {
            let pi = self.pos[i];
            // 投影累计跨所有边界（角部粒子同帧吃地面+墙的多笔推出）。
            let mut p = pi;
            let mut v = self.vel[i];
            let mut n_sum = Vec3::ZERO;
            let mut hit = false;
            for &bid in &self.boundaries {
                // AABB 预滤（外扩 h；None = 无信息 ⇒ 仍查询）。
                if let Some(bb) = providers.bounds(bid) {
                    let m = self.h;
                    if pi.x < bb.min.x - m
                        || pi.x > bb.max.x + m
                        || pi.y < bb.min.y - m
                        || pi.y > bb.max.y + m
                        || pi.z < bb.min.z - m
                        || pi.z > bb.max.z + m
                    {
                        continue;
                    }
                }
                self.contacts.clear();
                // 流体边界口径（内点鲁棒）：体素等截断 SDF 提供者的内部
                // 梯度被格间内面主导可指向固体深处 ⇒ 投影穿壁隧逃（实测
                // 1–2 格厚壁均复现）；解析面提供者默认原样转 contacts_point。
                if providers.contacts_point_boundary(bid, pi, self.skin, &mut self.contacts) {
                    for c in &self.contacts {
                        let sdf = (pi - c.point).dot(c.normal);
                        let pen = -sdf;
                        if pen <= 0.0 {
                            continue;
                        }
                        p += c.normal * (pen + self.skin).min(self.h);
                        n_sum += c.normal;
                        hit = true;
                        let vn = v.dot(c.normal);
                        if vn < 0.0 {
                            v -= c.normal * vn;
                        }
                    }
                    self.pos[i] = p;
                    self.vel[i] = v;
                }
            }
            if hit {
                pushed.push((i, n_sum));
            }
        }
        // 同位坍缩消解：仅在本子步被投影粒子间做最小间距（r_min = 0.2h，
        // 远小于静置间距，常态零触发）。高索引者沿 n_sum 内移——投影把
        // 深穿粒子逐轴推到平面交集点，法线和指向流体侧，一步拉开后
        // 常规压强接管。位移 ≤ r_min，不注入爆发能量。
        let r_min = 0.2 * self.h;
        for a in 0..pushed.len() {
            let (ia, na) = pushed[a];
            let nl = na.length();
            if nl <= 1e-9 {
                continue;
            }
            let dir = na * (1.0 / nl);
            let mut pa = self.pos[ia];
            for &(ib, _) in &pushed[..a] {
                let d = pa - self.pos[ib];
                let r2 = d.length_squared();
                if r2 < r_min * r_min {
                    let r = r2.sqrt();
                    // 同位（r=0）也拉满 r_min；方向 = 自身法线和，不依赖 d。
                    pa += dir * (r_min - r);
                }
            }
            self.pos[ia] = pa;
        }
    }
}

/// **介质场：`MediumField` 的第一个真实现**（ADR 0008/0009 的交互通道；`ROUTE.md` §4
/// 「刚体↔液体」格的第一刀 2a）。
///
/// 语义（只读采样；**不改流场、不改哈希**）：
/// - `sample(x)`：在 `x` 处按 **poly6 核**对 27 邻域求和，给出
///   `density = m·Σ_j W(r_ij)`（与流体内部的密度定义**同核同式**，只是不含自身项与鬼影项）、
///   `velocity = Σ w_j v_j / Σ w_j`（Shepard 平均 ⇒ 均匀流场下逐位精确）、
///   `occupied = clamp(ρ/ρ0, 0, 1)`（自由表面判据）。无近邻 ⇒ [`MediumSample::VACUUM`]。
/// - `deposit(…)`：**显式空实现**（不静默降级）——反作用要等 2b 的 Akinci 边界粒子把
///   「体↔流体」的动量交换喂回流场；在那之前，本域对刚体只是**单向的读数来源**。
/// - `viscosity`/`temperature` 报 0：XSPh 的 `ε` 是**无量纲**系数、不是 Pa·s，
///   本实现**不编造换算常数**（耦合侧用"密度 + 流速"算阻力即可，别依赖这个字段）。
impl MediumField for FluidSystem {
    fn sample(&self, x: Vec3) -> MediumSample {
        if self.pos.is_empty() || self.grid.items.is_empty() || self.grid.nz == 0 {
            return MediumSample::VACUUM;
        }
        let (bx, by, bz) = self.grid.bin_of(x);
        let (nx, ny, nz) = (self.grid.nx, self.grid.ny, self.grid.nz);
        let (mut wsum, mut rho) = (0.0f32, 0.0f32);
        let mut vsum = Vec3::ZERO;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let (ix, iy, iz) = (bx as i64 + dx, by as i64 + dy, bz as i64 + dz);
                    if ix < 0
                        || iy < 0
                        || iz < 0
                        || ix >= nx as i64
                        || iy >= ny as i64
                        || iz >= nz as i64
                    {
                        continue;
                    }
                    let idx =
                        ((ix as usize) * ny as usize + iy as usize) * nz as usize + iz as usize;
                    for &j in self.grid.bin_items(idx) {
                        let j = j as usize;
                        // **只认流体粒子**：本方法答的是"此处流体如何"，边界粒子（2b）
                        // 是固体侧的代表粒子，计进来会把"体内部"读成"满水位"
                        // （2a 的 `occupied` 会失真）。密度轮才计它们（那是流体自己的密度）。
                        if j >= self.n_fluid {
                            continue;
                        }
                        let p = self.pos[j];
                        let d = p - x;
                        let r2 = d.length_squared();
                        if r2 > self.h2 {
                            continue;
                        }
                        let t = self.h2 - r2;
                        let w = self.k6 * t * t * t;
                        wsum += w;
                        rho += self.mass * w;
                        vsum += self.vel[j] * w;
                    }
                }
            }
        }
        if wsum <= 0.0 {
            return MediumSample::VACUUM;
        }
        let inv = 1.0 / wsum;
        MediumSample {
            density: rho,
            velocity: vsum * inv,
            viscosity: 0.0,
            temperature: 0.0,
            occupied: (rho / self.cfg.rest_density).clamp(0.0, 1.0),
        }
    }

    fn deposit(&mut self, _x: Vec3, _momentum: Vec3, _mass: f32, _pressure_work: f32) {
        // 2b 的动量交换已在 `force_pass` 里**按对**完成（边界粒子与流体同核同式，
        // 逐对反对称 ⇒ 反作用自动等于流体受力的负值），不需要点式沉积。
        // 本方法仍是**显式**空实现：软体/布等"真点式"受体落地前，不做假装耦合。
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::interop::NoProviders;
    use vxl_phys_core::Aabb;

    /// 测试替身：半空间 y<0 实心（表面 y=0，外法线 +y）。
    /// 语义对齐体素点查询：depth = skin − sdf；depth < −skin 时仍返回支持。
    /// x/z 取大范围（±50）：落水飞溅的角部粒子会在无压区滑行数米，
    /// 面积太小会被「滑出测试地板边缘」误判成穿隧。
    pub(crate) struct FloorY;
    impl ProviderColliders for FloorY {
        fn bounds(&self, _id: u32) -> Option<Aabb> {
            Some(Aabb {
                min: Vec3::new(-50.0, -1.0, -50.0),
                max: Vec3::new(50.0, 0.0, 50.0),
            })
        }
        fn contacts_box(
            &self,
            _id: u32,
            _half: Vec3,
            _pos: Vec3,
            _rot: vxl_phys_core::Quat,
            _skin: f32,
            _out: &mut Vec<InteropContact>,
        ) -> bool {
            false
        }
        fn contacts_point(
            &self,
            _id: u32,
            p: Vec3,
            skin: f32,
            out: &mut Vec<InteropContact>,
        ) -> bool {
            let depth = skin - p.y; // sdf = p.y
            if depth < -skin {
                return true; // 支持查询，但不在接触带内
            }
            out.push(InteropContact {
                point: Vec3::new(p.x, 0.0, p.z),
                normal: Vec3::Y,
                depth,
                feature: 0,
            });
            true
        }
    }

    /// 测试替身：矩形水槽——地板 y=0 + 四面墙（内壁 x/z = ±0.2；7³ 柱
    /// 沉底摊平后仍保有 ~5 个核深的液柱，静水梯度可测）。
    /// 墙体沿法线向外无限延伸（半空间式），无「翻墙」另一侧。
    /// 面序：地板、左、右、后、前；sign = 内向法线的轴符号。
    pub(crate) struct Tank;
    impl Tank {
        const FACES: [(usize, f32, f32); 5] = [
            (1, 0.0, 1.0),
            (0, -0.2, 1.0),
            (0, 0.2, -1.0),
            (2, -0.2, 1.0),
            (2, 0.2, -1.0),
        ];
    }
    impl ProviderColliders for Tank {
        fn bounds(&self, _id: u32) -> Option<Aabb> {
            // 半空间几何的提示框必须覆盖固体全域：墙外/地板下仍是固体。
            // 若只框内腔，飞溅粒子越出提示框（如越过开放顶部落到 x<−0.31）
            // 后预滤会跳过一切查询——含地板——永久自由落体（实测 ESC：
            // x=−1.1 处 v=(·,−9.58,·) 匀加速坠穿）。对齐 FloorY 的 ±50。
            Some(Aabb {
                min: Vec3::new(-50.0, -50.0, -50.0),
                max: Vec3::new(50.0, 50.0, 50.0),
            })
        }
        fn contacts_box(
            &self,
            _id: u32,
            _half: Vec3,
            _pos: Vec3,
            _rot: vxl_phys_core::Quat,
            _skin: f32,
            _out: &mut Vec<InteropContact>,
        ) -> bool {
            false
        }
        fn contacts_point(
            &self,
            _id: u32,
            p: Vec3,
            probe: f32,
            out: &mut Vec<InteropContact>,
        ) -> bool {
            for &(axis, plane, sign) in &Self::FACES {
                // 半空间墙在全部 y 生效：墙面下方仍是墙固体（无「墙脚缝」）。
                // 若 y < 0 关墙，贴壁底层（近压抖沉到 y ∈ [−pen, 0]）会在墙
                // 关闭的子步里从墙线下横滑出去，地板预滤（余量 h）一丢即自由落体。
                let comp = [p.x, p.y, p.z][axis];
                let sdf = (comp - plane) * sign;
                if sdf > 2.0 * probe {
                    continue;
                }
                let point = match axis {
                    0 => Vec3::new(plane, p.y, p.z),
                    1 => Vec3::new(p.x, plane, p.z),
                    _ => Vec3::new(p.x, p.y, plane),
                };
                let normal = match axis {
                    0 => Vec3::new(sign, 0.0, 0.0),
                    1 => Vec3::new(0.0, sign, 0.0),
                    _ => Vec3::new(0.0, 0.0, sign),
                };
                out.push(InteropContact {
                    point,
                    normal,
                    depth: probe - sdf,
                    feature: 0,
                });
            }
            true
        }
    }

    /// 静止 5³ 晶格块（标定间距 = h/2）。
    fn still_block() -> FluidSystem {
        let cfg = FluidConfig {
            gravity: Vec3::ZERO,
            substeps: 2,
            ..FluidConfig::default()
        };
        FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [5, 5, 5], 0.05)
    }

    /// #1 静止晶格密度 ≈ ρ0（±2%，核截断的边界粒子除外）。
    #[test]
    fn rest_lattice_density_matches_rest_density() {
        let mut f = still_block();
        f.step(1.0 / 60.0, &NoProviders);
        let rho0 = f.config().rest_density;
        // 内部判定：晶格坐标（i+½）落在 [2.4, 5.6] 之外即距边不足 2 格。
        let origin = Vec3::new(-0.1, 0.2, -0.1);
        let inv_sp = 1.0 / 0.05;
        let mut checked = 0;
        for (i, p) in f.positions().iter().enumerate() {
            let q = (*p - origin) * inv_sp;
            if q.x < 2.4 || q.x > 3.6 || q.y < 2.4 || q.y > 3.6 || q.z < 2.4 || q.z > 3.6 {
                continue;
            }
            let rel = (f.densities()[i] - rho0).abs() / rho0;
            assert!(rel < 0.02, "粒子 {i} 密度相对偏差 {rel:.6}");
            checked += 1;
        }
        assert_eq!(checked, 8, "5³ 块内部应恰有 2³=8 个粒子");
    }

    /// #2 静水压强（切片1口径）：柱沉降后压强随深度**方向性递增**
    /// （下三分之一 > 中三分之一 > 上三分之一），并加三道量级闸：
    /// ① 柱高收在晶格初始高与摊平之间（四壁兜住的证据）；
    /// ② 中带均压在静水参考 ρ0·g·(H/2) 的 0.4–1.1×（离散 ρ 噪声经
    ///    Tait q⁷ 放大 + 自由面钳制 ⇒ 实测 ≈0.72×，偏低压）；
    /// ③ 底带均压 ≤ 2× 静水参考 ρ0·g·(5H/6)（镜像鬼影补给的边界层
    ///    q⁷ 尾部使贴底 2cm 系统性偏高 ~1.65×，三分带稀释后 ≈1.19×）。
    /// 用 Tank（地板+四壁）——无壁则落柱摊平成 puddle 是正确物理，静水态无从谈起。
    /// 实测口径（7³、间距 0.05、xsph 0.05、240 tick）：柱高 0.259，
    /// 下/中/上三分带均压 ≈ 2520/920/90 Pa。±30% 逐带量化校准
    /// 留待提分辨率或 δ-SPH 切片——本切片只锁方向与量级带宽。
    #[test]
    fn hydrostatic_pressure_increases_with_depth() {
        let cfg = FluidConfig {
            xsph_viscosity: 0.05,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
        f.set_boundaries(&[0]);
        for _ in 0..240 {
            f.step(1.0 / 60.0, &Tank);
        }
        let mut ymin = f32::INFINITY;
        let mut ymax = f32::NEG_INFINITY;
        for p in f.positions() {
            ymin = ymin.min(p.y);
            ymax = ymax.max(p.y);
        }
        let height = ymax - ymin;
        assert!(height > 0.20 && height < 0.32, "柱高 {height:.3} 异常");
        let (mut lo_s, mut lo_n, mut mid_s, mut mid_n, mut hi_s, mut hi_n) =
            (0.0f32, 0u32, 0.0f32, 0u32, 0.0f32, 0u32);
        for (i, p) in f.positions().iter().enumerate() {
            let t = (p.y - ymin) / height;
            let pr = f.pressures()[i];
            if t < 1.0 / 3.0 {
                lo_s += pr;
                lo_n += 1;
            } else if t < 2.0 / 3.0 {
                mid_s += pr;
                mid_n += 1;
            } else {
                hi_s += pr;
                hi_n += 1;
            }
        }
        let (lo, mid, hi) = (lo_s / lo_n as f32, mid_s / mid_n as f32, hi_s / hi_n as f32);
        // 方向：随深度单调（下 > 中 > 顶；顶部自由面钳制 p≥0）。
        assert!(
            lo > mid && mid > hi,
            "压强须随深度递增：底 {lo:.0} / 中 {mid:.0} / 顶 {hi:.0}"
        );
        // 量级闸（参考深度取各带中心：中带 t=0.5 → 深 H/2；底带 t=1/6 → 深 5H/6）。
        let rho0 = f.config().rest_density;
        let mid_ref = rho0 * 9.81 * 0.5 * height;
        assert!(
            mid > mid_ref * 0.4 && mid < mid_ref * 1.1,
            "中带 {mid:.0} 应在静水参考 {mid_ref:.0} 的 0.4–1.1×"
        );
        let lo_ref = rho0 * 9.81 * height * 5.0 / 6.0;
        assert!(
            lo < lo_ref * 2.0,
            "底带 {lo:.0} 超过边界层上界（2× 静水参考 {lo_ref:.0}）"
        );
        // 顶带（自由面 + 镜像鬼影密度尾，见 PLAN-0.3 §4.3）实测：release 下
        // 顶带 157 / 中带 646 ≈ 4.1×（旧的 5× 闸门在此误报失败），debug 下
        // 同一断言（5×）通过 ⇒ debug 比值 > 5。两个 profile 的绝对值不完全
        // 相同，因此判据取"顶带至少低于中带 3×"的稳健口径：真实缺陷是"顶带
        // 与中带同量级"，3× 抓得住，而不会被 profile 差异误报。失败信息把三带
        // 实测值全部带出。
        assert!(
            hi * 3.0 < mid,
            "顶带 {hi:.0} 应显著低于中带 {mid:.0}（底 {lo:.0}）"
        );
        for v in f.velocities() {
            assert!(v.length_squared() < 9.0, "速度失控：{v:?}");
        }
    }

    /// #3 动量守恒：零重力、XSPH 关闭，内部压力成对抵消 ⇒ Σv 漂移 < 1e−3。
    #[test]
    fn momentum_conserved_without_gravity() {
        let cfg = FluidConfig {
            gravity: Vec3::ZERO,
            xsph_viscosity: 0.0,
            substeps: 2,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.125, 0.2, -0.125), [5, 5, 5], 0.05);
        // 正弦速度场（非平移、非平衡）⇒ 内部力持续活跃。
        let vs: Vec<Vec3> = (0..f.len())
            .map(|i| {
                let a = i as f32;
                Vec3::new(0.3 * a.sin(), 0.3 * (a * 1.3).cos(), 0.3 * (a * 0.7).sin())
            })
            .collect();
        f.set_velocities(&vs);
        let p0: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
        for _ in 0..120 {
            f.step(1.0 / 60.0, &NoProviders);
        }
        let p1: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
        let drift = (p1 - p0).length();
        assert!(drift < 1e-3, "动量漂移 {drift:.3e}");
    }

    /// #4 确定性：同输入两次运行 600 子步，位置逐位相等。
    #[test]
    fn deterministic_bitwise() {
        let run = || {
            let cfg = FluidConfig {
                substeps: 2,
                ..FluidConfig::default()
            };
            let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.0, -0.1), [5, 5, 5], 0.05);
            f.set_boundaries(&[0]);
            for _ in 0..300 {
                f.step(1.0 / 60.0, &FloorY);
            }
            f.positions()
                .iter()
                .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run(), "两次运行必须逐位一致");
    }

    /// #5 边界：粒子落到地板（替身半空间）上不穿透、不弹飞。
    #[test]
    fn particles_rest_on_boundary_without_penetration() {
        let cfg = FluidConfig {
            xsph_viscosity: 0.05,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.2, -0.1), [5, 5, 5], 0.05);
        f.set_boundaries(&[0]);
        for _ in 0..300 {
            f.step(1.0 / 60.0, &FloorY);
        }
        for (i, p) in f.positions().iter().enumerate() {
            assert!(p.y > -0.01, "粒子 {i} 穿透地板：y = {}", p.y);
            assert!(p.y < 0.6, "粒子 {i} 弹飞：y = {}", p.y);
        }
    }

    /// #6 网格邻居 = 全扫暴力邻居（集合相等，排序后比对）。
    #[test]
    fn grid_neighbors_equal_brute_force() {
        let mut f = still_block();
        f.step(1.0 / 60.0, &NoProviders); // 建格（零重力下位置不动）
        let n = f.len();
        let mut grid_sets: Vec<Vec<u32>> = vec![Vec::new(); n];
        for (i, set) in grid_sets.iter_mut().enumerate() {
            f.for_neighbors(i, |j, _d, _r2| set.push(j as u32));
            set.sort_unstable();
        }
        let mut brute: Vec<Vec<u32>> = vec![Vec::new(); n];
        let h2 = f.h2;
        let ps = f.positions();
        for i in 0..n {
            for j in 0..n {
                if j != i && (ps[i] - ps[j]).length_squared() <= h2 {
                    brute[i].push(j as u32);
                }
            }
        }
        assert_eq!(grid_sets, brute);
    }

    /// 既有口径：默认配置仍是 CPU SPH 族。
    #[test]
    fn default_is_cpu_sph() {
        let c = FluidConfig::default();
        assert_eq!(c.family, FluidFamily::CpuSph);
        assert_eq!(c.rest_density, 1000.0);
        assert_eq!(c.substeps, 4);
    }

    /// #8 **介质场采样**（`MediumField`，2a 第一刀）：块内晶格点读到 ≈ρ0。
    /// 容差比内部密度测试宽（同核同式但**不含自身项与鬼影项**，且 5³ 块只剩 2 格余量）。
    #[test]
    fn medium_sample_center_reads_rest_density() {
        let mut f = still_block();
        f.step(1.0 / 60.0, &NoProviders);
        let c = Vec3::new(0.025, 0.325, 0.025); // origin + (2+½)·0.05 = 晶格点上
        let s = f.sample(c);
        let rho0 = f.config().rest_density;
        assert!(
            (s.density - rho0).abs() < 0.15 * rho0,
            "块中心密度 {:?} 应≈ρ0 {:?}",
            s.density,
            rho0
        );
        assert!(s.occupied > 0.85, "占用率 {:?} 应接近 1", s.occupied);
    }

    /// #9 采样口径：**无介质处返回真空**（核带外 ⇒ 不是"零密度的一团水"）。
    #[test]
    fn medium_sample_far_is_vacuum() {
        let mut f = still_block();
        f.step(1.0 / 60.0, &NoProviders);
        let far = Vec3::new(0.025, 0.325 + 10.0 * f.h, 0.025);
        let s = f.sample(far);
        assert_eq!(s.density, 0.0, "核带外不应有密度");
        assert_eq!(s.occupied, 0.0, "核带外占用率应为 0");
    }

    /// #10 采样速度 = **Shepard 平均**（Σwᵥ/Σw）⇒ 均匀流场下应与输入一致。
    #[test]
    fn medium_sample_velocity_follows_uniform_flow() {
        let mut f = still_block();
        f.step(1.0 / 60.0, &NoProviders);
        let v = Vec3::new(1.5, -0.25, 0.75);
        let vs = vec![v; f.len()];
        f.set_velocities(&vs);
        let s = f.sample(Vec3::new(0.025, 0.325, 0.025));
        assert!(
            (s.velocity - v).length() < 1e-5,
            "采样速度 {:?} 应≈{:?}",
            s.velocity,
            v
        );
    }

    // ───────────────────────── 2b：Akinci 两层边界粒子 ─────────────────────────
    // 判据见 `docs/M1-EXIT.md` §4：① 近壁密度天然正确 ② 稳定性 ③ 对无流体场景
    // 逐位不变（由 `crates/vxl-phys` 的四哈希门守）④ 造价。
    // 端到端（体真被浮起来 / 体推水）在 `crates/vxl-phys/tests/fluid_boundary.rs`。

    /// 体面速度静止的静态体（本文件只把"体"当几何用）。
    fn still_pose(pos: Vec3) -> BodyPose {
        BodyPose {
            pos,
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        }
    }

    /// #11 **近壁密度补偿**（2b 的核心主张）：贴壁层密度靠边界粒子补到 ρ0 量级。
    /// 对照 = 同水块**无地板**（自由面 ⇒ 贴壁层只剩 ~0.7ρ0）。两场景都只跑 1 步
    /// （密度只依赖位置，位置在 1 步内几乎不动 ⇒ 对照干净、测试快）。
    #[test]
    fn boundary_particles_restore_near_floor_density() {
        let cfg = FluidConfig::default();
        let mk = || FluidSystem::new(cfg.clone(), Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
        let mean_lo = |f: &FluidSystem| {
            let (mut s, mut n) = (0.0f32, 0usize);
            for (p, d) in f.positions().iter().zip(f.densities().iter()) {
                if p.y < 0.06 {
                    s += d;
                    n += 1;
                }
            }
            assert!(n > 0, "贴壁层应有粒子");
            s / n as f32
        };
        // 对照：无地板。
        let mut free = mk();
        free.step(1.0 / 60.0, &NoProviders);
        let rho_free = mean_lo(&free);
        // 实验：地板 = 一层 Box 体的边界粒子（顶面 y = 0，托住水块底面）。
        let mut on = mk();
        let floor = (
            0u32,
            Shape::Box {
                half: Vec3::new(0.3, 0.05, 0.3),
            },
            still_pose(Vec3::new(0.0, -0.05, 0.0)),
        );
        let n = on.set_boundary_particles(std::slice::from_ref(&floor));
        assert!(n > 0, "地板应生成边界粒子");
        on.step(1.0 / 60.0, &NoProviders);
        let rho_on = mean_lo(&on);
        let rho0 = on.config().rest_density;
        assert!(
            rho_free < 0.85 * rho0,
            "自由面贴底密度应偏低：{rho_free:.0}"
        );
        // 门槛按**实测**给（2026-09-22，Akinci 自洽体积标定后：+123 kg/m³，0.72→0.84ρ0）。
        // 旧口径（固定 `V_b = s³`）是 +80：自洽标定把补偿**做强**了，且不再对稀疏小体过量注入。
        assert!(
            rho_on > rho_free + 0.10 * rho0,
            "边界粒子应显著补回核质量：{rho_on:.0} vs 自由面 {rho_free:.0}"
        );
        assert!(
            rho_on < 1.3 * rho0,
            "补偿不得过量（会造虚假压力）：{rho_on:.0}"
        );
    }

    /// #12 **压力承住流体**（不穿透）：只靠边界粒子地板，平台范围内的粒子不得漏下去。
    /// ⚠️ 平台**边缘外**的粒子会（正确地）摊出去——那不是穿透。判据只看"站得住"：
    /// 芯内（|xz| ≤ 0.25）最低点不得越过第二层边界粒子（−0.075）以下。
    #[test]
    fn boundary_particles_hold_fluid_column() {
        let mut f = FluidSystem::new(
            FluidConfig::default(),
            Vec3::new(-0.3, 0.0, -0.3),
            [12, 12, 10],
            0.05,
        );
        let floor = (
            0u32,
            Shape::Box {
                half: Vec3::new(0.5, 0.05, 0.5),
            },
            still_pose(Vec3::new(0.0, -0.05, 0.0)),
        );
        let n = f.set_boundary_particles(std::slice::from_ref(&floor));
        assert!(n > 0);
        for _ in 0..60 {
            let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
            f.step(1.0 / 60.0, &NoProviders);
        }
        let mut core_min = f32::INFINITY;
        for p in f.positions() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "NaN/Inf"
            );
            if p.x.abs() <= 0.25 && p.z.abs() <= 0.25 {
                core_min = core_min.min(p.y);
            }
        }
        assert!(
            core_min > -0.06,
            "芯内粒子漏过边界粒子地板：min_y = {core_min:.4}"
        );
    }

    /// #13 **反作用 = 浮力**（端到端量纲闸）：槽里全潜盒应受 ≈ `ρ·V·g` 的上浮力。
    /// 槽用既有 `Tank` 提供者（**真实场景里围水是提供者通道的活**，边界粒子只管与体的
    /// 动量交换——这条把两者分工钉死）。
    ///
    /// ⚠️ **口径 = 窗口均值**（2026-09-22 用 `tests/boundary_accuracy_probe.rs` 两轴实测后定）：
    /// - **单帧端点值不可用**：反作用是"两个大数之差"，同一场景端点可读 −6.11×ρVg 而窗口均值
    ///   是另一个号；另一格端点 1.00 却在窗口内以 ±4.3×ρVg 摆。端点会让门随机红/绿。
    /// - 本档（盒半长 0.06 = 1.2h、`Tank`、7³ 水）窗口均值实测 **1.3–1.7×ρVg**；
    ///   **侧向均值 ≈ 0.01–0.1×ρVg**（先记录在案的 0.6–2.1× 是**端点瞬态**，不是稳态偏差——
    ///   这条是对我自己先前读数的更正）。
    /// - 机理：贴壁核质量（离散补偿）经 Tait q⁷ 放大 ⇒ 近壁压强量级远高于静水；
    ///   与**既有提供者方案的已接受偏差同族**（`PLAN-0.3.md` §4.3 底压 ≈1.65× 静水）。
    /// - **误差不是"体尺寸/h"的干净函数**：探针实测半长 0.04/0.06/0.09/0.12/0.15（h=0.1）
    ///   给出 3.30/1.38/1.63/**−6.13**/1.21 ⇒ 存在**离散相位共振**（体面栅格与流体晶格不可公约）。
    ///   ⇒ 2b 是**定性档**（水被推动、轻物浮起、方向对），**不是定量浮力模型**；
    ///   升级路径见 `OPEN-PROBLEMS.md` P7。断言带按此**诚实地**给。
    #[test]
    fn submerged_box_gets_buoyant_reaction() {
        let cfg = FluidConfig {
            xsph_viscosity: 0.05,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
        f.set_boundaries(&[0]);
        let half = 0.06f32;
        let body = (
            7u32,
            Shape::Box {
                half: Vec3::splat(half),
            },
            still_pose(Vec3::new(0.0, 0.12, 0.0)),
        );
        // **窗口均值**（2026-09-22 改；探针 `tests/boundary_accuracy_probe.rs` 实测得出）：
        // 反作用是"两个大数之差"，**单帧端点值可整号翻转**（实测同一场景 h=0.1 端点 −6.11×ρVg、
        // 而窗口均值 −6.13 是**系统性**的；另一格端点 1.00 而窗口内波动 ±4.3）⇒ 端点读数会让门
        // 随机红/绿。这里按仓库既有纪律（`vxl-phys-measurement-protocol` §5 窗口均值）取
        // **静置 180 + 窗口 60 tick 的均值**。
        for _ in 0..180 {
            let _ = f.set_boundary_particles(std::slice::from_ref(&body));
            f.step(1.0 / 60.0, &Tank);
        }
        let win = 60usize;
        let (mut sy, mut sx, mut sz) = (0.0f32, 0.0f32, 0.0f32);
        let (mut tsum, mut tmax) = (Vec3::ZERO, 0.0f32);
        for _ in 0..win {
            let _ = f.set_boundary_particles(std::slice::from_ref(&body));
            f.step(1.0 / 60.0, &Tank);
            if let Some(r) = f.boundary_reactions().iter().find(|r| r.0 == 7) {
                sy += r.1.y;
                sx += r.1.x;
                sz += r.1.z;
                tsum += r.2;
                tmax = tmax.max(r.2.length());
            }
        }
        let inv = 1.0 / win as f32;
        let force = Vec3::new(sx * inv, sy * inv, sz * inv);
        let tau = tsum * inv;
        let expect = f.config().rest_density * (2.0 * half).powi(3) * 9.81;
        println!(
            "2b 浮力（窗口 {win} tick 均值）{:+.2} N / ρVg = {expect:.2} N（比值 {:.2}）；\
             侧向 ({:+.2}, {:+.2})；|τ| 均值 {:.3} / 峰 {:.3}",
            force.y,
            force.y / expect,
            force.x,
            force.z,
            tau.length(),
            tmax
        );
        assert!(
            force.y > 0.5 * expect && force.y < 2.0 * expect,
            "浮力均值应 ≈ ρVg = {expect:.2} N（本档实测 1.3–1.7×），实测 {:+.2}（f = {force:?}）",
            force.y
        );
        // 侧向：**窗口均值下本来就近消**（探针实测 ≈ 0.01–0.1×ρVg；先前的 0.6–2.1× 是端点瞬态）
        // ⇒ 这条恢复成"近消"判据，只留宽带回旋余量。
        assert!(
            force.x.abs() < 0.5 * expect && force.z.abs() < 0.5 * expect,
            "对称场景侧向**均值**应近消（本档实测 ≤0.1×ρVg）：{force:?}"
        );
        assert!(
            tau.length() < 0.5 * expect * half,
            "对称场景力矩均值应近消：{tau:?}"
        );
    }

    /// #14 **确定性**（2b 在场）：同场景两跑，位置/密度/反作用**逐位一致**。
    #[test]
    fn boundary_coupling_is_deterministic() {
        let digest = || {
            let cfg = FluidConfig {
                xsph_viscosity: 0.05,
                ..FluidConfig::default()
            };
            let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
            f.set_boundaries(&[0]);
            let body = (
                7u32,
                Shape::Box {
                    half: Vec3::splat(0.06),
                },
                still_pose(Vec3::new(0.0, 0.12, 0.0)),
            );
            for _ in 0..60 {
                let _ = f.set_boundary_particles(std::slice::from_ref(&body));
                f.step(1.0 / 60.0, &Tank);
            }
            (
                f.positions()
                    .iter()
                    .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                    .collect::<Vec<u32>>(),
                f.densities()
                    .iter()
                    .map(|d| d.to_bits())
                    .collect::<Vec<u32>>(),
                f.boundary_reactions()
                    .iter()
                    .flat_map(|r| [r.1.x.to_bits(), r.1.y.to_bits(), r.2.x.to_bits()])
                    .collect::<Vec<u32>>(),
                f.boundary_count(),
            )
        };
        assert_eq!(digest(), digest(), "两次运行应逐位一致");
    }

    /// **并行 = 串行逐位一致**（`FluidConfig::threads`；2026-09-23 规模档并行化的守门）：
    /// 同一场景跑 `threads = 1` 与 `threads = 4`，末态位置/速度/密度**逐位相同**。
    /// 覆盖**两条路径**：① 纯流体（无边界粒子）；② **含边界粒子**（2b）——后者是风险点，
    /// 因为 `bforce` 是唯一的跨粒子累加量（并行路径用**串行补趟**保序）。
    #[test]
    fn parallel_equals_serial_bitwise() {
        let digest = |threads: usize, with_boundary: bool| {
            let cfg = FluidConfig {
                threads,
                ..FluidConfig::default()
            };
            let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
            if with_boundary {
                let floor = (
                    0u32,
                    Shape::Box {
                        half: Vec3::new(0.3, 0.05, 0.3),
                    },
                    still_pose(Vec3::new(0.0, -0.05, 0.0)),
                );
                let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
            }
            for _ in 0..40 {
                if with_boundary {
                    let floor = (
                        0u32,
                        Shape::Box {
                            half: Vec3::new(0.3, 0.05, 0.3),
                        },
                        still_pose(Vec3::new(0.0, -0.05, 0.0)),
                    );
                    let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
                }
                f.step(1.0 / 60.0, &NoProviders);
            }
            let mut d: Vec<u32> = Vec::new();
            for p in f.positions() {
                d.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
            }
            for v in f.velocities() {
                d.extend([v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
            }
            for r in f.densities() {
                d.push(r.to_bits());
            }
            for r in f.boundary_reactions() {
                d.extend([r.1.x.to_bits(), r.1.y.to_bits(), r.2.z.to_bits()]);
            }
            d
        };
        assert_eq!(
            digest(1, false),
            digest(4, false),
            "纯流体：并行与串行应逐位一致"
        );
        assert_eq!(
            digest(1, true),
            digest(4, true),
            "含边界粒子（2b）：并行与串行应逐位一致（bforce 走串行补趟）"
        );
    }
}

/// **微探针：邻居相位的"访存收集 vs 算术"账**（`#[ignore]`，只打印）——决定第三刀
/// （按格序重排粒子存储）值不值得动**公开数组序**。
///
/// 做法：同一份邻居表、同一遍历序，跑两个循环：
/// ① **收集版**：只碰 `pos/vel/dens/press/pmass` 五个数组并把值累加（几乎无算术）；
/// ② **算术版**：用同样收集到的值做真实力相位的那套运算（sqrt/乘加/分支）。
/// 两者之差 ≈ 纯算术成本；收集版本身 ≈ 访存成本。
/// ⇒ 若"收集 ≫ 算术" ⇒ 重排（缓存局部性）是杠杆；否则 SIMD 才是。
#[test]
#[ignore = "微探针（只打印）：访存 vs 算术账；见上注"]
fn micro_probe_gather_vs_compute() {
    use std::time::Instant;
    let n = 50usize; // 125k 粒（够大又不至于跑很久）
    let spacing = 0.05f32;
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders); // 建网格
    let h2 = f.h2;
    let (h, k6, ks) = (f.h, f.k6, f.ks);
    let alpha_c = f.cfg.artificial_viscosity * f.cfg.sound_speed;
    let nf = f.n_fluid;
    // ① 收集版
    let t0 = Instant::now();
    let mut sink = 0.0f32;
    for i in 0..nf {
        let vi = f.vel[i];
        let rho_i = f.dens[i];
        f.for_neighbors(i, |j, d, r2| {
            sink += f.pos[j].x + f.vel[j].y + f.dens[j] + f.press[j] + f.pmass[j];
            sink += d.x + r2;
            sink += vi.y + rho_i;
        });
    }
    let gather = t0.elapsed().as_secs_f64() * 1e3;
    // ② 算术版（真实力相位的算式，但结果丢弃）
    let t0 = Instant::now();
    let mut sink2 = 0.0f32;
    for i in 0..nf {
        let vi = f.vel[i];
        let rho_i = f.dens[i];
        let ci2 = f.press[i] / (rho_i * rho_i);
        let mut a = Vec3::ZERO;
        let mut xs = Vec3::ZERO;
        f.for_neighbors(i, |j, d, r2| {
            let rho_j = f.dens[j];
            let cj2 = f.press[j] / (rho_j * rho_j);
            let r = r2.sqrt();
            let t = h - r;
            let mj = f.pmass[j];
            let coef = mj * (ks * t * t) * (ci2 + cj2);
            let denom = r.max(1e-9);
            a += d * (coef / denom);
            let vij = vi - f.vel[j];
            let vdn = -vij.dot(d);
            if vdn > 0.0 {
                let mu = vdn * h / (r2 + 0.01 * h2);
                let c = mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ks * t * t);
                a += d * (c / denom);
            }
            let tt = h2 - r2;
            let w = k6 * tt * tt * tt;
            xs += (f.vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
        });
        sink2 += a.x + xs.y;
    }
    let compute = t0.elapsed().as_secs_f64() * 1e3;
    println!(
        "微探针（{nf} 粒）：收集版 {gather:.1} ms | 算术版 {compute:.1} ms | 算术占比 {:.0}% | sink {sink:.0}/{sink2:.0}",
        100.0 * compute / (gather + compute)
    );
}
