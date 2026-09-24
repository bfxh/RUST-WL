//! fluid_access：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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

    /// **邻域网格导出**（只读；GPU 后端与诊断用）。
    ///
    /// 语义：与 CPU 相位**同一张表**（计数排序：`start[c]..start[c+1]` = 格 c 的粒子、
    /// 格内按**索引升序**）⇒ GPU 侧按同一序枚举邻域即可与 CPU **同求和序**
    /// （这是 `docs/PLAN-gpu.md` §4 口径 A"能位级就位级"的前提）。
    pub fn neighbor_grid(&self) -> NeighborGrid<'_> {
        NeighborGrid {
            min: self.grid.min,
            inv: self.grid.inv,
            dims: (self.grid.nx, self.grid.ny, self.grid.nz),
            start: &self.grid.start,
            items: &self.grid.items,
        }
    }

    /// 流体粒子位置（前缀；不含边界粒子）。
    pub fn positions(&self) -> &[Vec3] {
        &self.pos[..self.n_fluid]
    }

    /// **全部粒子**（含 2b 边界粒子）的 `(pos, vel, pmass, n_fluid)`——**GPU 后端/耦合**用。
    /// 渲染与导出请走 `positions()`（只给流体前缀）。
    ///
    /// 语义（与 CPU 的 `substep` 逐条对应）：密度/压力/力跑**全部**粒子（邻域必须看得见边界粒子），
    /// 而**积分（含 XSPH/CFL）只跑 `0..n_fluid`**（边界粒子是运动学冻结的）。
    pub fn raw_particles(&self) -> (&[Vec3], &[Vec3], &[f32], usize) {
        (&self.pos, &self.vel, &self.pmass, self.n_fluid)
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
}
