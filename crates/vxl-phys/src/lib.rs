//! # vxl-phys
//!
//! `vxl_phys` 引擎门面（§1 依赖注入）：组装默认管线
//! core + broad + narrow + solver + integrate + island，游戏引擎（Bevy）只通过
//! 本 crate 的 `World` 消费物理（§0「纯 Rust 物理，引擎只做渲染壳」）。
//!
//! 步进序（每子步）：
//! 1. 力场 → 2. 速度积分 → 3. 宽相 → 4. 窄相 → 5. 求解 + 岛级休眠
//!    → 6. 位置积分。
//!
//! 确定性（§5）：固定步长、严格 f32、有序归约；`state_hash()` 每 60 tick 比对。

#![forbid(unsafe_code)]

pub use vxl_phys_broad::{Aabb, BroadPhase, BvhBroadPhase, GridBroadPhase};
pub use vxl_phys_core as core;
pub use vxl_phys_core::{
    BodyId, BodySet, BodyType, FrictionModel, JobSystem, Material, PhysConfig, Preset, Quat,
    ScopedPool, SerialJobSystem, Shape, Vec3,
};
pub use vxl_phys_field::{FieldRegistry, ForceField, GravityField};
pub use vxl_phys_integrate::Integrator;
pub use vxl_phys_narrow::heightfield::HeightField;
pub use vxl_phys_narrow::{ContactPoint, DefaultNarrowPhase, Manifold, NarrowPhase};
pub use vxl_phys_replay::{Fnv1aHash, Recorder, StateHash};
pub use vxl_phys_solver::{ccd, ImpulseSolver};
pub use vxl_phys_terrain::TerrainSet;

/// §3 稳定性健康报告（NaN/Inf 计数、静默穿透计数、抖动审计的输入）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HealthReport {
    pub nan_bodies: u32,
    /// 深度 > skin×4 的接触计数（§3：静默穿透 = 0）。
    pub deep_penetrations: u32,
    pub max_depth: f32,
    pub awake_bodies: u32,
    pub contacts: u32,
}

impl HealthReport {
    pub fn is_clean(&self) -> bool {
        self.nan_bodies == 0 && self.deep_penetrations == 0
    }
}

/// 分相耗时计数（§11 性能计数器：每帧物理 ms 按阶段拆分）。
/// 累计微秒，跨 `step` 累加，`reset_timings()` 归零。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhaseTimings {
    pub fields_us: u64,
    pub integrate_vel_us: u64,
    pub broadphase_us: u64,
    pub narrowphase_us: u64,
    pub solve_us: u64,
    pub integrate_pos_us: u64,
    pub ccd_us: u64,
}

impl PhaseTimings {
    pub fn total_us(&self) -> u64 {
        self.fields_us
            + self.integrate_vel_us
            + self.broadphase_us
            + self.narrowphase_us
            + self.solve_us
            + self.integrate_pos_us
            + self.ccd_us
    }
}

/// 物理世界（固定步长契约：调用方以 `config.dt` 的整数倍节拍调用 `step`）。
pub struct World {
    pub config: PhysConfig,
    pub bodies: BodySet,
    pub terrain: TerrainSet,
    /// 宽相（§2.3 主路径 = 增量 BVH；可用 `with_broadphase` 换网格等实现）。
    pub broad: Box<dyn BroadPhase>,
    pub narrow: DefaultNarrowPhase,
    pub solver: ImpulseSolver,
    pub fields: FieldRegistry,
    /// 任务调度（§6 依赖注入：SerialJobSystem / ScopedPool / 自定义实现）。
    pub jobs: Box<dyn JobSystem>,
    pub tick: u64,
    pairs: Vec<(u32, u32)>,
    manifolds: Vec<Manifold>,
    ccd_manifolds: Vec<Manifold>,
    hf_bounds: Vec<Aabb>,
    timings: PhaseTimings,
}

impl World {
    pub fn new(config: PhysConfig) -> Self {
        let broad = Box::new(BvhBroadPhase::new(config.contact_skin));
        Self::with_broadphase(config, broad)
    }

    /// 自定义宽相注入（依赖注入，§1；默认增量 BVH）。
    pub fn with_broadphase(config: PhysConfig, broad: Box<dyn BroadPhase>) -> Self {
        let mut fields = FieldRegistry::new();
        fields.add(Box::new(GravityField { g: config.gravity }));
        let skin = config.contact_skin;
        let mut bodies = BodySet::new();
        // 配置里的摩擦/恢复 = 默认材质（槽 0）；逐材质用 add_material 覆盖。
        bodies.materials[0] = Material::new(config.friction, config.restitution);
        // §6 调度注入：threads ≤ 1 → 串行（默认，回归对照基准）。
        let jobs: Box<dyn JobSystem> = if config.threads > 1 {
            Box::new(ScopedPool::new(config.threads))
        } else {
            Box::new(SerialJobSystem)
        };
        Self {
            broad,
            narrow: DefaultNarrowPhase::new(skin),
            solver: ImpulseSolver::new(skin),
            config,
            bodies,
            terrain: TerrainSet::new(),
            fields,
            jobs,
            tick: 0,
            pairs: Vec::new(),
            manifolds: Vec::new(),
            ccd_manifolds: Vec::new(),
            hf_bounds: Vec::new(),
            timings: PhaseTimings::default(),
        }
    }

    /// 累计分相耗时（自上次 `reset_timings` 起）。
    pub fn timings(&self) -> PhaseTimings {
        self.timings
    }

    pub fn reset_timings(&mut self) {
        self.timings = PhaseTimings::default();
    }

    pub fn add_static(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId {
        self.bodies.push_static(shape, position, rotation)
    }

    pub fn add_dynamic(
        &mut self,
        shape: Shape,
        position: Vec3,
        rotation: Quat,
        density: f32,
    ) -> BodyId {
        self.bodies.push_dynamic(shape, position, rotation, density)
    }

    /// 注册材质并返回 id（§4.4/§4.5）；`BodySet::set_material` 绑定到体。
    pub fn add_material(&mut self, material: core::Material) -> core::MaterialId {
        self.bodies.add_material(material)
    }

    /// 加入高度场：地形账本 + 静态 marker 体（宽相经 marker AABB 参与对生成）。
    pub fn add_heightfield(&mut self, hf: HeightField) -> BodyId {
        let id = self.terrain.add(hf);
        let bounds = self.terrain.bounds(id).expect("just inserted");
        self.hf_bounds.push(bounds);
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::HeightField(id), pos, rot)
    }

    pub fn add_field(&mut self, field: Box<dyn ForceField>) {
        self.fields.add(field);
    }

    /// 当前接触流形（渲染/游戏逻辑只读视图）。
    pub fn manifolds(&self) -> &[Manifold] {
        &self.manifolds
    }

    pub fn state_hash(&self) -> u64 {
        Fnv1aHash.hash_bodies(&self.bodies)
    }

    /// 推进一个固定 60Hz tick（内部按 config.substeps 细分，§4.2）。
    pub fn step(&mut self) {
        let substeps = self.config.substeps.max(1);
        let dt = self.config.dt / substeps as f32;
        for _ in 0..substeps {
            self.substep(dt);
        }
        self.tick += 1;
    }

    fn substep(&mut self, dt: f32) {
        // 1) 力场（重力在 World::new 注入注册表）。
        let t0 = std::time::Instant::now();
        self.fields.apply(&mut self.bodies);
        self.timings.fields_us += t0.elapsed().as_micros() as u64;
        // 2) 速度积分。
        let t0 = std::time::Instant::now();
        let maxl = self.config.max_linear_velocity;
        let maxa = self.config.max_angular_velocity;
        Integrator::integrate_velocities(&mut self.bodies, Vec3::ZERO, dt, maxl, maxa);
        self.timings.integrate_vel_us += t0.elapsed().as_micros() as u64;
        // 3) 宽相。
        let t0 = std::time::Instant::now();
        let pairs = self
            .broad
            .compute_pairs(&self.bodies, &self.hf_bounds, self.jobs.as_ref())
            .to_vec();
        self.timings.broadphase_us += t0.elapsed().as_micros() as u64;
        // 4) 窄相。
        let t0 = std::time::Instant::now();
        self.narrow.collide(
            &self.bodies,
            &pairs,
            self.terrain.slice(),
            &mut self.manifolds,
            self.jobs.as_ref(),
        );
        self.timings.narrowphase_us += t0.elapsed().as_micros() as u64;
        self.pairs = pairs;
        // 5) 求解 + 岛级休眠（唤醒语义在岛内：外部唤醒/新接触自动传播全岛）。
        let t0 = std::time::Instant::now();
        self.solver.solve(
            &mut self.bodies,
            &self.manifolds,
            &self.config,
            dt,
            self.jobs.as_ref(),
        );
        self.timings.solve_us += t0.elapsed().as_micros() as u64;
        // 6) 位置积分。
        let t0 = std::time::Instant::now();
        Integrator::integrate_positions(&mut self.bodies, dt);
        self.timings.integrate_pos_us += t0.elapsed().as_micros() as u64;
        // 7) 选择性 CCD（§4.12）：对高速体回扫本子步位移，命中即钳位 + 清法向速度。
        let t0 = std::time::Instant::now();
        self.ccd_pass(dt);
        self.timings.ccd_us += t0.elapsed().as_micros() as u64;
    }

    /// 选择性 CCD（§4.12）：保守推进采样。
    ///
    /// 位置积分已把体推到 p1；这里从 p0 = p1 − v·dt 回扫到 p1，找到首个与
    /// 静态/沉睡目标接触的采样段，把体钳位到上一安全采样点，并清零指向
    /// 表面的法向速度（M1 非弹性停下）。动-动 CCD 在 M2 接入。
    fn ccd_pass(&mut self, dt: f32) {
        if !self.config.ccd_speed_threshold.is_finite() {
            return;
        }
        let n = self.bodies.len();
        let flagged: Vec<usize> = (0..n)
            .filter(|&i| ccd::needs_ccd(&self.bodies, i, &self.config, dt))
            .collect();
        for &i in &flagged {
            let v = self.bodies.linvel[i];
            let steps = ccd::step_count(&self.bodies, i, &self.config, dt);
            let sub = dt / steps as f32;
            let p1 = self.bodies.position[i];
            let p0 = p1 - v * dt;

            // 扫掠 AABB → 候选（静态/沉睡体）+ 高度场。
            let lo = p0.min(p1);
            let hi = p0.max(p1);
            let swept = Aabb {
                min: lo - Vec3::splat(self.config.contact_skin),
                max: hi + Vec3::splat(self.config.contact_skin),
            };
            let mut candidates: Vec<u32> = Vec::new();
            self.broad.query_aabb(&swept, &mut candidates);
            candidates.retain(|&j| {
                let j = j as usize;
                j != i && (!self.bodies.is_dynamic(j) || !self.bodies.awake[j])
            });

            let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(candidates.len());
            for &j in &candidates {
                let (a, b) = if (j as usize) < i {
                    (j, i as u32)
                } else {
                    (i as u32, j)
                };
                pairs.push((a, b));
            }
            // 高度场 marker 体本身是静态叶子，已在 candidates 内。
            let mut hit: Option<(f32, Vec3)> = None;
            for s in 1..=steps {
                let t = sub * s as f32;
                self.bodies.position[i] = p0 + v * t;
                self.narrow.collide(
                    &self.bodies,
                    &pairs,
                    self.terrain.slice(),
                    &mut self.ccd_manifolds,
                    self.jobs.as_ref(),
                );
                if let Some(m) = self.ccd_manifolds.first() {
                    // 法线 a→b；换算成「推离表面、指向动体」的方向。
                    let n_into_body = if m.a == i as u32 { -m.normal } else { m.normal };
                    hit = Some((t, n_into_body));
                    break;
                }
            }
            if let Some((t_hit, n_into_body)) = hit {
                let t_safe = (t_hit - sub).max(0.0);
                self.bodies.position[i] = p0 + v * t_safe;
                let vn = self.bodies.linvel[i].dot(n_into_body);
                if vn < 0.0 {
                    self.bodies.linvel[i] -= n_into_body * vn;
                }
            } else {
                self.bodies.position[i] = p1;
            }
        }
    }

    /// §3 稳定性指标采集。
    pub fn health(&self) -> HealthReport {
        let mut rep = HealthReport {
            max_depth: 0.0,
            ..HealthReport::default()
        };
        for i in 0..self.bodies.len() {
            if !self.bodies.position[i].is_finite() || !self.bodies.linvel[i].is_finite() {
                rep.nan_bodies += 1;
            }
            // 活跃度只对动体有意义（静态 marker 恒置 awake=true 但不参与休眠）。
            if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                rep.awake_bodies += 1;
            }
        }
        let limit = self.config.contact_skin * 4.0;
        for m in &self.manifolds {
            rep.contacts += m.points.len() as u32;
            for p in &m.points {
                rep.max_depth = rep.max_depth.max(p.depth);
                if p.depth > limit {
                    rep.deep_penetrations += 1;
                }
            }
        }
        rep
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ground_world() -> World {
        let mut w = World::new(PhysConfig::default());
        let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
        w.add_heightfield(hf);
        w
    }

    #[test]
    fn box_falls_and_rests_on_heightfield() {
        let mut w = ground_world();
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 3.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..240 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 静置在 y ≈ 0.5 + 少许穿透修正余量。
        assert!(y > 0.45 && y < 0.62, "y = {y}");
        assert!(w.health().is_clean());
    }

    #[test]
    fn determinism_same_construction_same_hash() {
        let run = || {
            let mut w = ground_world();
            for k in 0..12 {
                let x = (k % 4) as f32 * 1.2;
                let z = (k / 4) as f32 * 1.2;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.4),
                    },
                    Vec3::new(x, 2.0 + (k as f32) * 0.9, z),
                    Quat::IDENTITY,
                    1.0,
                );
            }
            for _ in 0..180 {
                w.step();
            }
            w.state_hash()
        };
        assert_eq!(run(), run());
    }

    /// §5/§6 并行契约：并行（threads=8）与串行（threads=1）结果 bit 级一致。
    /// 场景需超过各相并行门槛（>4096 体）才能真正走到并行路径。
    #[test]
    fn parallel_matches_serial_bitwise() {
        let run = |threads: usize| {
            let cfg = PhysConfig {
                threads,
                ..PhysConfig::default()
            };
            let mut w = World::new(cfg);
            w.add_heightfield(HeightField::flat(-40.0, -40.0, 81, 81, 1.0, 0.0));
            for k in 0..4200usize {
                let x = (k % 70) as f32 - 35.0;
                let z = (k / 70) as f32 - 30.0;
                w.add_static(
                    Shape::Box {
                        half: Vec3::new(0.5, 0.5, 0.5),
                    },
                    Vec3::new(x, 0.5, z),
                    Quat::IDENTITY,
                );
            }
            for k in 0..900 {
                let x = ((k * 37) % 97) as f32 / 97.0 * 30.0 - 15.0;
                let z = ((k * 53) % 89) as f32 / 89.0 * 30.0 - 15.0;
                let y = 4.0 + ((k * 29) % 71) as f32 / 71.0 * 10.0;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.4),
                    },
                    Vec3::new(x, y, z),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
            for _ in 0..150 {
                w.step();
            }
            w.state_hash()
        };
        let serial = run(1);
        let parallel = run(8);
        assert_eq!(serial, parallel, "并行与串行状态哈希必须一致（§5）");
    }

    #[test]
    fn pyramid_settles_and_sleeps() {
        let mut w = ground_world();
        let layers = 4;
        for layer in 0..layers {
            let count = layers - layer;
            for k in 0..count {
                let x = (k as f32 - (count as f32 - 1.0) * 0.5) * 1.05;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.5),
                    },
                    Vec3::new(x, 0.55 + layer as f32 * 1.02, 0.0),
                    Quat::IDENTITY,
                    1.0,
                );
            }
        }
        for _ in 0..600 {
            w.step();
        }
        let h = w.health();
        assert!(h.is_clean(), "{h:?}");
        // 塔应全部入睡（§3：无持续抖动）。
        let awake = h.awake_bodies;
        assert_eq!(awake, 0, "awake = {awake}");
    }

    #[test]
    fn restitution_bounce_and_threshold() {
        let mut w = World::new(PhysConfig {
            restitution: 0.8,
            ..PhysConfig::default()
        });
        w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
        let b = w.add_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let mut max_after_fall = 0.0f32;
        for _ in 0..1800 {
            w.step();
            max_after_fall = max_after_fall.max(w.bodies.linvel[b as usize].y);
        }
        // e=0.8 应产生明显反弹（> 2 m/s 向上），30 秒内经恢复阈值衰减到静止。
        assert!(max_after_fall > 2.0, "bounce vy = {max_after_fall}");
        assert!(w.bodies.linvel[b as usize].y.abs() < 0.3);
    }

    #[test]
    fn digging_removes_support() {
        let mut w = ground_world();
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.6, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..120 {
            w.step();
        }
        let y_rest = w.bodies.position[b as usize].y;
        // 在盒子正下方挖 3×3 列块（2m 深）。
        for gx in -1i32..=1 {
            for gz in -1i32..=1 {
                w.terrain.dig(0, gx as f32, gz as f32, 2.0);
            }
        }
        w.bodies.wake(b as usize);
        for _ in 0..120 {
            w.step();
        }
        let y_after = w.bodies.position[b as usize].y;
        assert!(y_after < y_rest - 1.0, "rest {y_rest} → after {y_after}");
    }

    #[test]
    fn material_pair_restitution_and_friction() {
        // §4.4/§4.5：e 取材质对 max——球(e=0.9) 落到地面(e=0.0) → 反弹按 0.9。
        let mut w = World::new(PhysConfig::default());
        let ground_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.5 }, 0.0));
        let ball_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.2 }, 0.9));
        w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
        let b = w.add_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        w.bodies.set_material(b as usize, ball_mat);
        let _ = ground_mat;
        let mut max_bounce = 0.0f32;
        for _ in 0..1200 {
            w.step();
            max_bounce = max_bounce.max(w.bodies.linvel[b as usize].y);
        }
        // 0.9 × 9.9 m/s 落地速度 ≈ 8.9 m/s 首次反弹。
        assert!(max_bounce > 4.0, "material pair bounce vy = {max_bounce}");
        assert!(w.health().is_clean());
    }

    #[test]
    fn ccd_stops_fast_sphere_at_thin_wall() {
        // 薄墙 half_z=0.1；球 r=0.2 以 120 m/s（单帧位移 2m）射向墙体。
        // 采样带（墙厚+球径=0.6m）< 位移 2m 且起点偏移 → 离散步进必然穿透。
        let build = |ccd_on: bool| {
            let mut w = World::new(PhysConfig {
                ccd_speed_threshold: if ccd_on { 30.0 } else { f32::INFINITY },
                max_linear_velocity: 200.0,
                ..PhysConfig::default()
            });
            w.add_static(
                Shape::Box {
                    half: Vec3::new(5.0, 5.0, 0.1),
                },
                Vec3::ZERO,
                Quat::IDENTITY,
            );
            let s = w.add_dynamic(
                Shape::Sphere { radius: 0.2 },
                Vec3::new(0.0, 0.0, -19.0),
                Quat::IDENTITY,
                1.0,
            );
            w.bodies.set_linvel(s as usize, Vec3::new(0.0, 0.0, 120.0));
            (w, s)
        };
        // CCD 关：穿透（球越过墙面 z>0.3）。
        let (mut w_off, s_off) = build(false);
        for _ in 0..20 {
            w_off.step();
        }
        let z_off = w_off.bodies.position[s_off as usize].z;
        assert!(z_off > 1.0, "预期穿透，实际 z = {z_off}");
        // CCD 开：钳位在墙前。
        let (mut w_on, s_on) = build(true);
        for _ in 0..20 {
            w_on.step();
        }
        let z_on = w_on.bodies.position[s_on as usize].z;
        assert!(
            (-1.0..=0.9).contains(&z_on),
            "CCD 应拦下球，实际 z = {z_on}"
        );
        let h = w_on.health();
        assert!(h.is_clean(), "{h:?}");
    }

    #[test]
    fn ccd_disabled_by_default_keeps_slow_scene_unchanged() {
        // 默认阈值 INFINITY：确定性场景与 M0 行为一致（回归守门）。
        let run = || {
            let mut w = ground_world();
            for k in 0..6 {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.4),
                    },
                    Vec3::new(k as f32 * 1.1, 2.0 + k as f32 * 0.9, 0.0),
                    Quat::IDENTITY,
                    1.0,
                );
            }
            for _ in 0..180 {
                w.step();
            }
            w.state_hash()
        };
        assert_eq!(run(), run());
    }
}
