//! fluid_step：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl FluidSystem {
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
    pub(crate) fn truncate_to_fluid(&mut self) {
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

    pub(crate) fn substep(&mut self, dt: f32, providers: &dyn ProviderColliders) {
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
    pub(crate) fn for_neighbors(&self, i: usize, f: impl FnMut(usize, Vec3, f32)) {
        self.grid.for_neighbors_in(&self.pos, self.h2, i, f);
    }
}
