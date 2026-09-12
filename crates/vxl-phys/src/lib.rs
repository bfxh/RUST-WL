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

pub use vxl_phys_broad::{Aabb, BroadPhase, GridBroadPhase};
pub use vxl_phys_core as core;
pub use vxl_phys_core::{
    BodyId, BodySet, BodyType, FrictionModel, PhysConfig, Preset, Quat, Shape, Vec3,
};
pub use vxl_phys_field::{FieldRegistry, ForceField, GravityField};
pub use vxl_phys_integrate::Integrator;
pub use vxl_phys_narrow::heightfield::HeightField;
pub use vxl_phys_narrow::{ContactPoint, DefaultNarrowPhase, Manifold, NarrowPhase};
pub use vxl_phys_replay::{Fnv1aHash, Recorder, StateHash};
pub use vxl_phys_solver::ImpulseSolver;
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

/// 物理世界（固定步长契约：调用方以 `config.dt` 的整数倍节拍调用 `step`）。
pub struct World {
    pub config: PhysConfig,
    pub bodies: BodySet,
    pub terrain: TerrainSet,
    pub broad: GridBroadPhase,
    pub narrow: DefaultNarrowPhase,
    pub solver: ImpulseSolver,
    pub fields: FieldRegistry,
    pub tick: u64,
    pairs: Vec<(u32, u32)>,
    manifolds: Vec<Manifold>,
    hf_bounds: Vec<Aabb>,
}

impl World {
    pub fn new(config: PhysConfig) -> Self {
        let mut fields = FieldRegistry::new();
        fields.add(Box::new(GravityField { g: config.gravity }));
        let skin = config.contact_skin;
        Self {
            broad: GridBroadPhase::new(2.0, skin),
            narrow: DefaultNarrowPhase::new(skin),
            solver: ImpulseSolver::new(skin),
            config,
            bodies: BodySet::new(),
            terrain: TerrainSet::new(),
            fields,
            tick: 0,
            pairs: Vec::new(),
            manifolds: Vec::new(),
            hf_bounds: Vec::new(),
        }
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
        self.fields.apply(&mut self.bodies);
        // 2) 速度积分。
        let maxl = self.config.max_linear_velocity;
        let maxa = self.config.max_angular_velocity;
        Integrator::integrate_velocities(&mut self.bodies, Vec3::ZERO, dt, maxl, maxa);
        // 3) 宽相。
        let pairs = self
            .broad
            .compute_pairs(&self.bodies, &self.hf_bounds)
            .to_vec();
        // 4) 窄相。
        self.narrow.collide(
            &self.bodies,
            &pairs,
            self.terrain.slice(),
            &mut self.manifolds,
        );
        self.pairs = pairs;
        // 5) 求解 + 岛级休眠（唤醒语义在岛内：外部唤醒/新接触自动传播全岛）。
        self.solver
            .solve(&mut self.bodies, &self.manifolds, &self.config, dt);
        // 6) 位置积分。
        Integrator::integrate_positions(&mut self.bodies, dt);
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
}
