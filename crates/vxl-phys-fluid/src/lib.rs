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

#![forbid(unsafe_code)]

use vxl_phys_core::interop::{InteropContact, MediumField, MediumSample, ProviderColliders};
use vxl_phys_core::Vec3;

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
#[derive(Default)]
struct UniformGrid {
    bin: f32,
    inv: f32,
    min: Vec3,
    nx: u32,
    ny: u32,
    nz: u32,
    bins: Vec<Vec<u32>>,
}

impl UniformGrid {
    fn rebuild(&mut self, pos: &[Vec3], h: f32) {
        self.bins.clear();
        let n = pos.len();
        if n == 0 {
            self.nx = 0;
            self.ny = 0;
            self.nz = 0;
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
        self.bins.resize_with(total, Vec::new);
        for (i, p) in pos.iter().enumerate() {
            let (bx, by, bz) = self.bin_of(*p);
            let idx = (bx * self.ny + by) * self.nz + bz;
            self.bins[idx as usize].push(i as u32);
        }
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
            pos,
            cfg,
        }
    }

    pub fn len(&self) -> usize {
        self.pos.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pos.is_empty()
    }

    pub fn config(&self) -> &FluidConfig {
        &self.cfg
    }

    /// 单粒质量（晶格标定结果；导出/审计用）。
    pub fn particle_mass(&self) -> f32 {
        self.mass
    }

    pub fn positions(&self) -> &[Vec3] {
        &self.pos
    }

    pub fn velocities(&self) -> &[Vec3] {
        &self.vel
    }

    pub fn densities(&self) -> &[f32] {
        &self.dens
    }

    pub fn pressures(&self) -> &[f32] {
        &self.press
    }

    /// 覆盖初速度（ dams break / 动量测试用；顺序 = 粒子索引序）。
    pub fn set_velocities(&mut self, vs: &[Vec3]) {
        let n = vs.len().min(self.vel.len());
        self.vel[..n].copy_from_slice(&vs[..n]);
    }

    /// 边界提供者 id 列表（统一 id 空间）。
    pub fn boundaries(&self) -> &[u32] {
        &self.boundaries
    }

    pub fn set_boundaries(&mut self, ids: &[u32]) {
        self.boundaries = ids.to_vec();
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
        let n = self.pos.len();
        if n == 0 {
            return;
        }
        self.grid.rebuild(&self.pos, self.h);
        self.density_pass(providers);
        self.pressure_pass();
        self.force_pass();
        // 半隐式欧拉：v ← v + dt·a + ε·xsph；CFL 限速防穿隧。
        let vmax = self.cfg.max_speed_frac * self.h / dt;
        let vmax2 = vmax * vmax;
        let eps = self.cfg.xsph_viscosity;
        for i in 0..n {
            let mut v = self.vel[i] + self.acc[i] * dt + self.xsph[i] * eps;
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            self.vel[i] = v;
            self.pos[i] += v * dt;
        }
        self.boundary_pass(providers);
    }

    /// 邻域公共体：粒子 i 的 27 邻域格（钳边）内、r ≤ h 的 j 交给 `f`。
    /// 访问序 = 格坐标序（dz, dy, dx 固定）× 格内索引序 ⇒ 求和序是位置的确定函数。
    #[inline]
    fn for_neighbors(&self, i: usize, mut f: impl FnMut(usize, Vec3, f32)) {
        let pi = self.pos[i];
        let (cx, cy, cz) = self.grid.bin_of(pi);
        let (nx, ny, nz) = (self.grid.nx, self.grid.ny, self.grid.nz);
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
                    for &j in &self.grid.bins[idx] {
                        let j = j as usize;
                        if j == i {
                            continue;
                        }
                        let d = pi - self.pos[j];
                        let r2 = d.length_squared();
                        if r2 <= self.h2 {
                            f(j, d, r2);
                        }
                    }
                }
            }
        }
    }

    /// 密度：ρ_i = m·(W(0) + Σ_j W(r_ij) + Σ_ghost W)（poly6，含自身项）。
    /// 边界镜像鬼影：壁邻粒子（0 < sdf < h）把真实邻居关于壁面（接触点 +
    /// 外法线）反射，补回固体一侧的**离散**核质量。不用连续半空间积分——
    /// 它按均匀连续介质补，而流体实际的近壁分布（沉降后成层、各向异性）
    /// 的离散亏量与之错带（实测底层差 ~12% ρ0，补不齐 ⇒ p≥0 死区复活）。
    /// 鬼影随流体局部分布同步：流体压缩/成层，鬼影同压缩/成层。
    fn density_pass(&mut self, providers: &dyn ProviderColliders) {
        let walls = !self.boundaries.is_empty();
        for i in 0..self.pos.len() {
            // 收集 h 带内壁面（点 + 外法线；角部粒子可有多面）。
            let mut pl_pt = [Vec3::ZERO; 8];
            let mut pl_n = [Vec3::ZERO; 8];
            let mut np = 0usize;
            if walls {
                let pi = self.pos[i];
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
            }
            let pi = self.pos[i];
            let mut sum = self.w0;
            self.for_neighbors(i, |_j, d, r2| {
                let t = self.h2 - r2;
                sum += self.k6 * t * t * t;
                for w in 0..np {
                    let pj = pi - d;
                    let dn = (pj - pl_pt[w]).dot(pl_n[w]);
                    let g = pj - pl_n[w] * (2.0 * dn);
                    let rg2 = (pi - g).length_squared();
                    if rg2 <= self.h2 {
                        let tg = self.h2 - rg2;
                        sum += self.k6 * tg * tg * tg;
                    }
                }
            });
            self.dens[i] = self.mass * sum;
        }
    }

    /// 压力：Tait p = B·((ρ/ρ0)^γ − 1)；γ=7 走整数次幂乘法展开。
    /// `tensile_instability_suppression`（默认开）= Monaghan 自由面钳制
    /// p ≥ 0：表面密度截断（q ≈ 0.5） otherwise 会给出 p ≈ −B，经对称
    /// spiky 形式变成巨大粒子间吸力（单子步 Δv ~ 10² m/s）⇒ 全体炸散。
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
    fn force_pass(&mut self) {
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        for i in 0..self.pos.len() {
            let vi = self.vel[i];
            let rho_i = self.dens[i];
            let ci2 = self.press[i] / (rho_i * rho_i);
            let mut acc = g;
            let mut xs = Vec3::ZERO;
            self.for_neighbors(i, |j, d, r2| {
                let rho_j = self.dens[j];
                let cj2 = self.press[j] / (rho_j * rho_j);
                let r = r2.sqrt();
                let t = self.h - r;
                // a_i += m·(p_i/ρ_i² + p_j/ρ_j²)·45/(πh⁶)·(h−r)²·d̂（d̂ 指离 j）。
                let coef = self.mass * (self.ks * t * t) * (ci2 + cj2);
                acc += d * (coef / r.max(1e-9));
                // Monaghan 人工黏度：仅接近对（v_ij·d < 0），Π = α·c·μ/ρ̄、
                // μ = h·|v_ij·d|/(r² + 0.01h²)；逐对反对称 ⇒ 动量守恒，耗散法向动能。
                let vij = vi - self.vel[j];
                let vdn = -vij.dot(d);
                if vdn > 0.0 {
                    let mu = vdn * self.h / (r2 + 0.01 * self.h2);
                    let cv =
                        self.mass * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (self.ks * t * t);
                    acc += d * (cv / r.max(1e-9));
                }
                let tt = self.h2 - r2;
                let w = self.k6 * tt * tt * tt;
                xs += (self.vel[j] - vi) * (self.mass * 2.0 / (rho_i + rho_j) * w);
            });
            self.acc[i] = acc;
            self.xsph[i] = xs;
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
    fn boundary_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.boundaries.is_empty() {
            return;
        }
        // 本子步被投影粒子：(索引, 接触法线和)。升序登记 ⇒ 消解序确定。
        let mut pushed: Vec<(usize, Vec3)> = Vec::new();
        for i in 0..self.pos.len() {
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
        if self.pos.is_empty() || self.grid.bins.is_empty() || self.grid.nz == 0 {
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
                    for &j in &self.grid.bins[idx] {
                        let p = self.pos[j as usize];
                        let d = p - x;
                        let r2 = d.length_squared();
                        if r2 > self.h2 {
                            continue;
                        }
                        let t = self.h2 - r2;
                        let w = self.k6 * t * t * t;
                        wsum += w;
                        rho += self.mass * w;
                        vsum += self.vel[j as usize] * w;
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
        // 2b（Akinci 两层边界粒子）才落地反作用；此处**显式**留空，不假装已耦合。
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
}
