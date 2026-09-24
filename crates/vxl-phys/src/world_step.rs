//! world_step：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) mod fluid_stepper;
pub(crate) mod narrow_tier;

impl World {
    /// 推进一个固定 60Hz tick（内部按 config.substeps 细分，§4.2）。
    pub fn step(&mut self) {
        let substeps = self.config.substeps.max(1);
        let dt = self.config.dt / substeps as f32;
        // **运动自适应**（见 `detect_once_per_tick` 注）：只有准静态时才敢复用流形表。
        // 判据：本 tick 的最大位移 `max|v|·dt` ≤ 半个 skin ⇒ 接触集在一 tick 内不会
        // 实质变化（运动中冻结检测会漏掉"逼近中的接触"，把平滑接触变成硬碰撞——
        // 塔沉降期实测 KE ×300）。
        let reuse = detect_once_per_tick() && {
            let mut v2 = 0.0f32;
            for i in 0..self.bodies.len() {
                if self.bodies.awake[i] {
                    v2 = v2.max(self.bodies.linvel[i].length_squared());
                }
            }
            v2.sqrt() * self.config.dt <= 0.5 * self.config.contact_skin
        };
        for k in 0..substeps {
            self.substep(dt, k == 0, reuse);
        }
        self.fluid_pass();
        self.tick += 1;
    }

    /// **液体域通道（切片1：单向耦合）**：体解算+积分完毕后，流体以自身
    /// `FluidConfig::substeps` 的固定子步数推进一个 tick（`config.dt`）。
    /// 边界碰撞走统一提供者通道（`Providers` 实现 `ProviderColliders`，
    /// 体素/网格/喷溅同 id 空间）；无流体时零成本短路。
    ///
    /// **2b**（`add_fluid_with_boundary_coupling` 注册的流体）：先按近域体重建
    /// 边界粒子集再步进 ⇒ 流体本 tick 就"看见"体的新位姿（体子步已完成）；
    /// 反作用由下一次 `medium_pass` 施加（与 2a 同口径：一 tick 滞后，值取最新状态）。
    pub(crate) fn fluid_pass(&mut self) {
        if self.fluids.is_empty() {
            return;
        }
        for fi in 0..self.fluids.len() {
            if self.fluid_2b.get(fi).copied().unwrap_or(false) {
                self.refresh_fluid_boundary(fi);
            }
            let dt = self.config.dt;
            // **卡上步进档**（§13.7 选项 A）：把重建好的边界段交给后端，主机不推进自己的流体状态。
            if !self.fluid_stepper_pass(fi) {
                self.fluids[fi].0.step(dt, &self.providers);
            }
        }
    }

    /// **2b 边界粒子重建**（每 tick 一次）：把近域体的表面两层粒子装进该流体系统，
    /// 并重建**覆盖集**（2a 让位判据）。
    ///
    /// - 近域判据 = 体包围球（+ 核半径）与流体粒子 AABB 相交 ⇒ **保守**（宁多造不漏）。
    /// - 形状不受支持（复合体/高度场/provider/凸壳）⇒ 跳过（该体继续走 2a 粗档）。
    /// - 静态体也生成（让静态几何对流体**可感**）；反作用只对清醒动态体施加（段③）。
    /// - 确定性：体按索引升序、`set_boundary_particles` 按参数序排布 ⇒ 求和序固定。
    pub(crate) fn refresh_fluid_boundary(&mut self, fi: usize) {
        self.fluid_boundary_scratch.clear();
        self.fluid_boundary_covered.clear();
        self.fluid_boundary_covered.resize(self.bodies.len(), false);
        let bb = fluid_stepper::bounds_of(&self.fluids[fi])
            .or_else(|| particle_bounds(&self.fluids[fi].0));
        if let Some((lo, hi)) = bb {
            let pad = self.fluids[fi].0.config().smoothing_radius;
            for i in 0..self.bodies.len() {
                let shape = self.bodies.shape[i];
                if !vxl_phys_fluid::boundary::supports(&shape) {
                    continue;
                }
                let c = self.bodies.position[i];
                let r = shape.bounding_sphere_radius() + pad;
                if c.x < lo.x - r
                    || c.x > hi.x + r
                    || c.y < lo.y - r
                    || c.y > hi.y + r
                    || c.z < lo.z - r
                    || c.z > hi.z + r
                {
                    continue;
                }
                self.fluid_boundary_scratch.push((
                    i as u32,
                    shape,
                    vxl_phys_fluid::BodyPose {
                        pos: c,
                        rot: self.bodies.rot(i),
                        linvel: self.bodies.linvel[i],
                        angvel: self.bodies.angvel(i),
                    },
                ));
                self.fluid_boundary_covered[i] = true;
            }
        }
        // `set_boundary_particles` 要 `&scratch` 而 `self.fluids` 要 `&mut`：借出后归还
        // （`Vec::take` 是 O(1)），避免两个 `self` 字段的可变/共享借用冲突。
        let scratch = std::mem::take(&mut self.fluid_boundary_scratch);
        let _ = self.fluids[fi].0.set_boundary_particles(&scratch);
        self.fluid_boundary_scratch = scratch;
    }

    /// 介质通道**段①**：喷溅场作介质（只累加二次阻力 `F = −½·ρ·Cd·A·|v_rel|·v_rel`；
    /// 密度 0 的场直接跳过）。**冻结行为**（PhysArena/喷溅场景的哈希以它为基准）⇒ 纯搬移。
    fn splat_medium_pass(&mut self) {
        const DRAG_CD: f32 = 1.0;
        for id in 0..self.providers.len() as u32 {
            let Some(f) = self.providers.splat(id) else {
                continue;
            };
            if f.medium_density <= 0.0 {
                continue;
            }
            let bb = self.provider_bounds[id as usize];
            for i in 0..self.bodies.len() {
                if !self.bodies.is_dynamic(i) {
                    continue;
                }
                let p = self.bodies.position[i];
                if p.x < bb.min.x
                    || p.x > bb.max.x
                    || p.y < bb.min.y
                    || p.y > bb.max.y
                    || p.z < bb.min.z
                    || p.z > bb.max.z
                {
                    continue; // 场外 = 真空
                }
                use vxl_phys_core::interop::MediumField as _;
                let m = f.sample(p);
                if m.density <= 0.0 {
                    continue;
                }
                let v_rel = self.bodies.linvel[i] - m.velocity;
                let sp = v_rel.length();
                if sp < 1e-6 {
                    continue;
                }
                let a = cross_section_area(&self.bodies.shape[i]);
                if a <= 0.0 {
                    continue;
                }
                self.bodies.force[i] += v_rel * (-0.5 * m.density * DRAG_CD * a * sp);
            }
        }
    }

    /// **介质通道**：把「介质状提供者」的状态作用到刚体上（单侧：介质 → 体）。
    ///
    /// **两段，物理分量不同（别混）**：
    /// ① **喷溅场作介质**（既有，2026-09 起）：只累加二次阻力 `F = −½·ρ·Cd·A·|v_rel|·v_rel`。
    ///    密度 0 的场直接跳过；**本段是冻结行为**（PhysArena/喷溅场景的哈希以它为基准）⇒ 不动。
    /// ② **流体作介质**（2a，2026-09-22，`ROUTE.md` §4「刚体↔液体」格）：**浮力 + 阻力**——
    ///    浮力按阿基米德 `F = −g·ρ_med·V·frac_sub`（`frac_sub` = 采样点占用率均值，
    ///    即"浸没体积分数"的代理），阻力同 ①。采样点 = **体心 + 4 个水平表面点**
    ///    （单点采样在水线处是阶跃 ⇒ 盒子会抖；四点平均把水线过渡抹平）。
    ///    ⇒ 比水轻的盒浮起、比水重的盒下沉（物理量纲齐备，不看质量以外的旋钮）。
    ///
    /// 确定性：场/流体按注册序、体按索引序、采样点固定序 ⇒ 全索引序可复现；
    /// `v_rel` 取体心速度减介质流速（力矩本切片不施加，与 ① 一致）。
    /// **零成本短路**：无流体 / 无介质密度 / 体不在介质包围盒内 ⇒ 不采样。
    pub(crate) fn medium_pass(&mut self) {
        const DRAG_CD: f32 = 1.0;
        self.splat_medium_pass();

        // ── ② 流体作介质（2a，见上方文档）：浮力（阿基米德）+ 阻力 ──
        // （2b 覆盖的体在此**让位**：其浮力/阻力由 Akinci 反作用接管，见段③）
        use vxl_phys_core::interop::MediumField as _;
        if self.fluids.is_empty() {
            return;
        }
        let g = self.config.gravity;
        for (fi, (sys, _, st)) in self.fluids.iter().enumerate() {
            // **卡上步进的流体不参与 2a 介质采样**：主机那份状态不再推进 ⇒ 采样是陈的
            // （§13.7 的契约）⇒ 整体让位（该流体的 2b 覆盖判据同样不适用）。
            if st.is_some() {
                continue;
            }
            let pos = sys.positions();
            if pos.is_empty() {
                continue;
            }
            let two_b = self.fluid_2b.get(fi).copied().unwrap_or(false);
            // 粒子包围盒（每 tick 一次 O(n)；体先过包围盒，避免全库逐体采样）。
            let (lo, hi) = match particle_bounds(sys) {
                Some(b) => b,
                None => continue,
            };
            let pad = sys.config().smoothing_radius; // 核半径余量：表面外仍有介质影响
            for i in 0..self.bodies.len() {
                if !self.bodies.is_dynamic(i) || !self.bodies.awake[i] {
                    continue; // 睡眠体不受外力（唤醒后自然恢复；与"睡眠按静态处理"一致）
                }
                if two_b && self.fluid_boundary_covered.get(i).copied().unwrap_or(false) {
                    continue; // 2b 已覆盖 ⇒ 二选一，不叠加
                }
                let c = self.bodies.position[i];
                if c.x < lo.x - pad
                    || c.x > hi.x + pad
                    || c.y < lo.y - pad
                    || c.y > hi.y + pad
                    || c.z < lo.z - pad
                    || c.z > hi.z + pad
                {
                    continue;
                }
                let vol = shape_volume(&self.bodies.shape[i]);
                if vol <= 0.0 {
                    continue;
                }
                let r = body_half_extent(&self.bodies.shape[i]);
                // 浸没体积分数：体心 + 4 个水平表面点的占用率均值（单点在水线处是阶跃）。
                let pts = [
                    c,
                    Vec3::new(c.x + r, c.y, c.z),
                    Vec3::new(c.x - r, c.y, c.z),
                    Vec3::new(c.x, c.y, c.z + r),
                    Vec3::new(c.x, c.y, c.z - r),
                ];
                let (mut occ, mut dens) = (0.0f32, 0.0f32);
                let mut vmed = Vec3::ZERO;
                for p in pts {
                    let s = sys.sample(p);
                    occ += s.occupied;
                    dens += s.density;
                    vmed += s.velocity;
                }
                let inv = 1.0 / pts.len() as f32;
                let frac_sub = (occ * inv).clamp(0.0, 1.0);
                if frac_sub <= 0.0 {
                    continue;
                }
                let rho = dens * inv;
                let v_rel = self.bodies.linvel[i] - vmed * inv;
                let sp = v_rel.length();
                let mut f = -g * (rho * vol * frac_sub);
                let a = cross_section_area(&self.bodies.shape[i]);
                if a > 0.0 && sp > 1e-6 {
                    f += v_rel * (-0.5 * rho * DRAG_CD * a * sp);
                }
                self.bodies.force[i] += f;
            }
        }
    }

    /// **2b 反作用回流**：把各 2b 流体的边界粒子反作用（力 + 绕体原点的力矩）加到体上。
    /// 量纲 = **力**（不是冲量）：与 2a 一样在每个体子步施加一次 ⇒ 一个 tick 的冲量
    /// = `F·dt`（施加次数 × 子步 dt = tick dt）。睡眠体不吃外力（与 2a 同口径）。
    pub(crate) fn fluid_reaction_pass(&mut self) {
        for fi in 0..self.fluids.len() {
            if !self.fluid_2b.get(fi).copied().unwrap_or(false) {
                continue;
            }
            let reacts = fluid_stepper::reactions_of(&self.fluids[fi]);
            for &(body, f, tau) in reacts {
                let i = body as usize;
                if i >= self.bodies.len() || !self.bodies.is_dynamic(i) || !self.bodies.awake[i] {
                    continue;
                }
                self.bodies.force[i] += f;
                self.bodies.torque[i] += tau;
            }
        }
    }

    pub(crate) fn substep(&mut self, dt: f32, first: bool, reuse_manifolds: bool) {
        // 1) 力场（重力在 World::new 注入注册表）+ 介质耦合（喷溅场作介质）。
        // 计时走跨目标探针：wasm32-unknown-unknown 无时钟（`Instant::now()`
        // 会 panic），该目标下退化为 0；原生行为不变。
        let t0 = vxl_phys_core::probe::start();
        self.fields.apply(&mut self.bodies);
        self.medium_pass();
        // 2b（Akinci 边界粒子）反作用：与 2a 同段位（体子步开始处、积分之前），
        // 只对 2b 注册的流体生效 ⇒ 未开的场景零成本、逐位不变。
        self.fluid_reaction_pass();
        self.timings.fields_us += vxl_phys_core::probe::us(t0);
        // 2) 速度积分。
        let t0 = vxl_phys_core::probe::start();
        let maxl = self.config.max_linear_velocity;
        let maxa = self.config.max_angular_velocity;
        Integrator::integrate_velocities(&mut self.bodies, Vec3::ZERO, dt, maxl, maxa);
        self.timings.integrate_vel_us += vxl_phys_core::probe::us(t0);
        // 3) 宽相（先注入步长：速度自适应 fat 边距用）。
        //    **检测每步一次**（实验开关 `detect_once_per_tick` + 准静态判据）：
        //    非首子步且准静态时跳过宽相 + 窄相，复用本 tick 首子步的流形表。
        let detect = first || !reuse_manifolds;
        let t0 = vxl_phys_core::probe::start();
        if detect {
            self.broad.set_step(dt);
            let pairs = self
                .broad
                .compute_pairs(
                    &self.bodies,
                    &self.hf_bounds,
                    &self.provider_bounds,
                    self.jobs.as_ref(),
                )
                .to_vec();
            self.timings.broadphase_us += vxl_phys_core::probe::us(t0);
            // 4) 窄相。
            let t0 = vxl_phys_core::probe::start();
            // 速度充气视野（见 `narrow::DefaultNarrowPhase::set_predict_dt`）：
            // 预测时长 = **距下一次检测的间隔**（复用流形 ⇒ 整个 tick；否则不预测）。
            // 关闭检测复用时恒传 0 ⇒ 现行行为逐位不变。
            let predict_dt = if reuse_manifolds { self.config.dt } else { 0.0 };
            self.narrow.set_predict_dt(predict_dt);
            // 卡上窄相档（§17.9）：未注册 / 跑不通 / `predict_dt > 0` ⇒ 整趟回退 CPU（不半途混用）。
            if !self.narrow_tier_pass(&pairs, predict_dt) {
                self.narrow.collide(
                    &self.bodies,
                    &pairs,
                    self.terrain.slice(),
                    &self.providers,
                    &mut self.manifolds,
                    self.jobs.as_ref(),
                );
            }
            self.timings.narrowphase_us += vxl_phys_core::probe::us(t0);
            self.pairs = pairs;
        }
        // 4.5) 冲击快照（**解算前**）：provider 对的接近速度与接触点写给破坏管线。
        //      放在这里而不是让管线读末态速度——子步/迭代会把法向速度解掉，
        //      管线读末态就会漏掉"这一瞬间撞上了"这件事（dt 无关性）。
        self.record_impacts();
        // 5) 求解 + 岛级休眠（唤醒语义在岛内：外部唤醒/新接触自动传播全岛）。
        //    关节：唤醒传播必须**赶在接触解算之前**（否则关节链在"已判沉睡"
        //    的岛上晚一步醒），关节冲量本身在接触解算之后施加。
        let t0 = vxl_phys_core::probe::start();
        self.joints.wake(&mut self.bodies);
        self.solver.solve(
            &mut self.bodies,
            &self.manifolds,
            &self.config,
            dt,
            self.jobs.as_ref(),
        );
        self.joints.solve(&mut self.bodies, &self.config, dt);
        self.timings.solve_us += vxl_phys_core::probe::us(t0);
        // 6) 位置积分。
        let t0 = vxl_phys_core::probe::start();
        Integrator::integrate_positions(&mut self.bodies, dt);
        self.timings.integrate_pos_us += vxl_phys_core::probe::us(t0);
        // 6.5) **无偏置趟**（Rapier TGS-Soft 语义：带偏置趟 → 位置积分 → 无偏置趟）：
        //      去穿透已由 6) 的位置推进兑现，这里把「修正速度」（erp 去穿透偏置 +
        //      切向漂移回拉）从**最终速度**里移除——它们此前会作为真实动能留在体上。
        //      关节不重复求解（本件只动接触通道；关节另有 joint_iterations 预算）。
        if self.config.stabilization_iterations > 0 {
            let t0 = vxl_phys_core::probe::start();
            self.solver.solve_unbiased(
                &mut self.bodies,
                &self.manifolds,
                &self.config,
                dt,
                self.jobs.as_ref(),
            );
            self.timings.solve_us += vxl_phys_core::probe::us(t0);
        }
        // 7) 选择性 CCD（§4.12）：对高速体回扫本子步位移，命中即钳位 + 清法向速度。
        let t0 = vxl_phys_core::probe::start();
        self.ccd_pass(dt);
        self.timings.ccd_us += vxl_phys_core::probe::us(t0);
    }
}
