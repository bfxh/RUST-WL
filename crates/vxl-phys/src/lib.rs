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
    BodyId, BodySet, BodyType, FrictionModel, JobSystem, Material, PhaseArena, PhysConfig, Preset,
    Quat, ScopedPool, SerialJobSystem, Shape, Vec3,
};
pub use vxl_phys_field::{FieldRegistry, ForceField, GravityField};
pub use vxl_phys_integrate::Integrator;
pub use vxl_phys_narrow::heightfield::HeightField;
pub use vxl_phys_narrow::{ContactPoint, DefaultNarrowPhase, Manifold, NarrowPhase};
pub use vxl_phys_replay::{Recorder, StateHash, Xxh3Hash};
pub use vxl_phys_solver::{ccd, ImpulseSolver};
pub use vxl_phys_terrain::TerrainSet;

/// 外部碰撞提供者集合（门面持有；实现 `interop::ProviderColliders` 供窄相查询）。
#[derive(Default)]
pub struct Providers {
    vols: Vec<vxl_phys_terrain::voxel::VoxelVolume>,
}

impl Providers {
    /// 注册体素体，返回其 id（= 注册序）。
    pub fn push(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> u32 {
        let id = self.vols.len() as u32;
        self.vols.push(vol);
        id
    }

    pub fn len(&self) -> usize {
        self.vols.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vols.is_empty()
    }

    /// provider(id) 的世界包围盒（宽相 AABB 供给；与 `CollisionProvider::bounds` 同义）。
    pub fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        self.vols.get(id as usize).map(|v| v.bounds())
    }

    pub fn voxel(&self, id: u32) -> Option<&vxl_phys_terrain::voxel::VoxelVolume> {
        self.vols.get(id as usize)
    }

    pub fn voxel_mut(&mut self, id: u32) -> Option<&mut vxl_phys_terrain::voxel::VoxelVolume> {
        self.vols.get_mut(id as usize)
    }
}

impl vxl_phys_core::interop::ProviderColliders for Providers {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        self.vols.get(id as usize).map(|v| v.bounds())
    }

    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.vols.get(id as usize) {
            Some(v) => vxl_phys_terrain::voxel::contacts_box_voxel(v, half, pos, rot, skin, out),
            None => false,
        }
    }

    fn contacts_sphere(
        &self,
        id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.vols.get(id as usize) {
            Some(v) => vxl_phys_terrain::voxel::contacts_sphere_voxel(v, center, radius, skin, out),
            None => false,
        }
    }
}

/// 帧级相位暂存（§0.1 #10：相内 bump，相末 reset，全帧零 free）。
///
/// M0 已接入的消费者 = 帧末状态哈希的规范化缓冲（每次哈希 alloc→打包→reset，
/// 600 tick 内 high_water 恒定、overflows = 0）；宽/窄/求解三相的缓冲化与其
/// R1 顶尖化同批接入（M1）——机制与计数在本相已实证。
#[derive(Debug)]
pub struct FrameArenas {
    pub hash: PhaseArena,
}

impl FrameArenas {
    pub fn new() -> Self {
        Self {
            hash: PhaseArena::with_capacity(HASH_SCRATCH_BYTES),
        }
    }
}

impl Default for FrameArenas {
    fn default() -> Self {
        Self::new()
    }
}

/// 哈希规范化暂存预算（固定 8KB；流式打包与体数无关，永不溢出）。
pub const HASH_SCRATCH_BYTES: usize = 8 << 10;

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
    /// 帧级相位暂存（§0.1 #10；M0 接入 = 哈希规范化缓冲）。
    pub arenas: FrameArenas,
    pub tick: u64,
    pairs: Vec<(u32, u32)>,
    manifolds: Vec<Manifold>,
    ccd_manifolds: Vec<Manifold>,
    hf_bounds: Vec<Aabb>,
    /// 外部碰撞提供者集合（体素/网格…；ROUTE §2.1 兼容轴）与其 AABB。
    providers: Providers,
    provider_bounds: Vec<Aabb>,
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
            arenas: FrameArenas::new(),
            tick: 0,
            pairs: Vec::new(),
            manifolds: Vec::new(),
            ccd_manifolds: Vec::new(),
            hf_bounds: Vec::new(),
            providers: Providers::default(),
            provider_bounds: Vec::new(),
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

    /// 注册一个**体素体**为碰撞提供者（新域接入的第一条路径，ROUTE §3/§5）：
    /// 加入提供者集合 + 静态 Marker 体 `Shape::Provider(id)`；宽相 AABB 由
    /// 提供者的 bounds 供给（与高度场同机制）。返回 marker 体 id。
    pub fn add_voxel(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> BodyId {
        let id = self.providers.push(vol);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }

    /// **破坏（M3 第一块）**：把体素体盒域内的占据格转为**刚体碎块**
    /// （贪心合并成盒 → 逐个动态体），并从体素体里移除。返回碎块数。
    /// 确定性：提取顺序 = 固定扫描序（见 `VoxelVolume::extract_boxes`）；
    /// 碎块质量 = `density × 8·hx·hy·hz`。
    pub fn spawn_box_debris(&mut self, id: u32, min: Vec3, max: Vec3, density: f32) -> usize {
        self.spawn_box_debris_vel(id, min, max, density, Vec3::ZERO)
    }

    /// 同 [`spawn_box_debris`](Self::spawn_box_debris)，但碎块带初速 `vel`
    /// （冲击破坏用：碎块继承部分冲击速度 ⇒ 与冲击体的相对速度被压低）。
    pub fn spawn_box_debris_vel(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        density: f32,
        vel: Vec3,
    ) -> usize {
        let Some(vol) = self.providers.voxel_mut(id) else {
            return 0;
        };
        let boxes = vol.extract_boxes(min, max);
        let n = boxes.len();
        for (c, h) in boxes {
            let mass = density * 8.0 * h.x * h.y * h.z;
            let b =
                self.bodies
                    .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, mass.max(1e-3));
            self.bodies.linvel[b as usize] = vel;
        }
        self.refresh_provider_bounds();
        n
    }

    /// **冲击破坏（M3）**：扫描最近一次检测的流形，对「动体 × provider(id)」的
    /// **高速接触**在接触点处挖出并转为碎块（挖出半径随冲击速度增长）。
    /// 返回本次产生的碎块总数。确定性：按流形序处理、挖域为轴对齐盒、
    /// 提取顺序固定（见 `extract_boxes`）。
    ///
    /// `speed_threshold` = 触发阈值（m/s，取动体速度）；`density` = 碎块密度。
    pub fn apply_impact_destruction(
        &mut self,
        id: u32,
        speed_threshold: f32,
        density: f32,
    ) -> usize {
        // 先在只读扫描里收集「挖点」（按流形序），再逐个挖 —— 保持确定性。
        let mut digs: Vec<(Vec3, f32, Vec3)> = Vec::new();
        for m in &self.manifolds {
            let (sa, sb) = (
                self.bodies.shape[m.a as usize],
                self.bodies.shape[m.b as usize],
            );
            let (other, prov) = match (sa, sb) {
                (Shape::Provider(p), _) => (m.b, p),
                (_, Shape::Provider(p)) => (m.a, p),
                _ => continue,
            };
            if prov != id || !self.bodies.is_dynamic(other as usize) {
                continue;
            }
            let v = self.bodies.linvel[other as usize];
            let sp = v.length();
            if sp < speed_threshold {
                continue;
            }
            // 接触点 = 流形点均值（确定性）
            let n = m.points.len().max(1) as f32;
            let mut c = Vec3::ZERO;
            for p in m.points.iter() {
                c += p.point;
            }
            digs.push((c * (1.0 / n), sp, v));
        }
        let mut total = 0usize;
        for (c, sp, v) in digs {
            // 挖出半径随**实际冲击速度**增长（钳到 0.25..0.9 m）
            let r = (0.2 + 0.06 * sp).clamp(0.25, 0.9);
            // 挖域沿**冲击方向**前推 r：碎块生成在墙体内、避开冲击体本体
            // （否则与冲击体深度重叠 ⇒ 分离冲量注入能量，实测 KE 异常增长）。
            let dir = if v.length_squared() > 1e-9 {
                v.normalize()
            } else {
                Vec3::ZERO
            };
            let center = c + dir * r;
            // 碎块**静止生成**（初速留给调用方用 `spawn_box_debris_vel` 显式给；
            // 引擎不凭空造动量——「继承半速」实测是能量源，已否）。
            total += self.spawn_box_debris_vel(
                id,
                center - Vec3::splat(r),
                center + Vec3::splat(r),
                density,
                Vec3::ZERO,
            );
        }
        total
    }

    /// 提供者集合只读视图（体素体诊断/可视化用）。
    pub fn providers(&self) -> &Providers {
        &self.providers
    }

    /// 提供者数据变化（如体素挖洞）后刷新宽相 AABB（确定性：按 id 序全量重算）。
    pub fn refresh_provider_bounds(&mut self) {
        for id in 0..self.providers.len() as u32 {
            if let Some(b) = self.providers.bounds(id) {
                self.provider_bounds[id as usize] = b;
            }
        }
    }

    pub fn add_field(&mut self, field: Box<dyn ForceField>) {
        self.fields.add(field);
    }

    /// 当前接触流形（渲染/游戏逻辑只读视图）。
    pub fn manifolds(&self) -> &[Manifold] {
        &self.manifolds
    }

    /// 规范状态哈希（§5 xxh3-128 唯一真值；热组 32B 记录按 id 序打包）。
    ///
    /// 规范化缓冲走帧级相位 arena（`arenas.hash`）：每次调用 alloc→打包→reset，
    /// 全帧零堆分配、缓冲永不增长。因世界自持暂存，故取 `&mut self`。
    pub fn state_hash(&mut self) -> u128 {
        let scratch = self
            .arenas
            .hash
            .alloc(HASH_SCRATCH_BYTES, 8)
            .expect("哈希暂存预算固定 8KB，流式打包不随体数增长");
        let h = vxl_phys_replay::hash_bodies_streaming(&self.bodies, scratch);
        self.arenas.hash.reset();
        h
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
        // 3) 宽相（先注入步长：速度自适应 fat 边距用）。
        let t0 = std::time::Instant::now();
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
        self.timings.broadphase_us += t0.elapsed().as_micros() as u64;
        // 4) 窄相。
        let t0 = std::time::Instant::now();
        self.narrow.collide(
            &self.bodies,
            &pairs,
            self.terrain.slice(),
            &self.providers,
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
                    &self.providers,
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

    /// **M2 贯通切片**（ROUTE §7）：**刚体 ↔ 体素**——盒经 `Shape::Provider` 路径
    /// 落在体素地面上并入睡（跨域唯一通道 `ProviderColliders` 的第一条端到端用例）。
    #[test]
    fn box_falls_and_rests_on_voxel_provider() {
        let mut w = World::new(PhysConfig::default());
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
        let marker = w.add_voxel(vol);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 2.5, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 静置在体素顶面（y=1.0）上方：y ≈ 1.5 + 少许穿透修正余量。
        assert!(y > 1.42 && y < 1.60, "y = {y}");
        assert!(!w.bodies.awake[b as usize], "盒应已入睡（静置 10s）");
        assert!(w.health().is_clean());
        // marker 体（provider）保持静止：位置零漂移。
        assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
    }

    /// M2 provider 通道扩到**球**：球经 SDF 解析接触（`depth = r − sdf(c)`）
    /// 落在体素地面上并入睡；顺带覆盖「斜坡不穿透」。
    #[test]
    fn sphere_rests_on_voxel_provider() {
        let mut w = World::new(PhysConfig::default());
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
        w.add_voxel(vol);
        let b = w.add_dynamic(
            Shape::Sphere { radius: 0.4 },
            Vec3::new(0.25, 2.0, 0.25),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 静置在体素顶面（y=1.0）上方：y ≈ 1.4
        assert!(y > 1.32 && y < 1.50, "y = {y}");
        assert!(!w.bodies.awake[b as usize], "球应已入睡");
        assert!(w.health().is_clean());
    }

    /// **M3 破坏切片**：体素柱被「切掉顶部」⇒ 顶部转成刚体碎块，落在余柱上停驻；
    /// 余柱（仍在体素体里）与碎块共同构成确定性可继续推进的场景。
    #[test]
    fn carve_top_spawns_debris_resting_on_column() {
        let mut w = World::new(PhysConfig::default());
        // 柱：X/Z ∈ [−0.5,0.5]、Y ∈ [0,4)，格边长 0.5（8 层 × 2×2）
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 8, 8);
        vol.fill_box(Vec3::new(-0.5, 0.0, -0.5), Vec3::new(0.5, 4.0, 0.5));
        w.add_voxel(vol);
        // 切掉顶部 1m（Y ∈ [3,4)）⇒ 碎块（2×2×2 格 ⇒ 贪心合并为 1 个 1m 立方）
        let n = w.spawn_box_debris(
            0,
            Vec3::new(-0.5, 3.0, -0.5),
            Vec3::new(0.5, 4.0, 0.5),
            1000.0,
        );
        assert_eq!(n, 1, "顶部 8 格应合并为 1 个碎块盒");
        for _ in 0..600 {
            w.step();
        }
        let h = w.health();
        assert!(h.is_clean(), "无 NaN / 无深穿透");
        // 碎块落在余柱顶面（y=3.0）上方：中心 ≈ 3.5
        let mut top = 0.0f32;
        for i in 0..w.bodies.len() {
            if w.bodies.is_dynamic(i) {
                top = top.max(w.bodies.position[i].y);
            }
        }
        assert!(top > 3.3 && top < 3.7, "碎块应停在余柱上，实际 top={top}");
    }

    /// **M3 冲击破坏**：高速盒撞体素墙 ⇒ 接触点处挖洞并产出碎块；墙体素减少、
    /// 场景干净，且**整轮可复现**（同一构造两次 → 碎块数/末态哈希一致）。
    #[test]
    fn impact_carves_wall_and_spawns_debris() {
        let run = || -> (usize, usize, usize, u128) {
            let mut w = World::new(PhysConfig::default());
            // 地板（整幅 1 层）+ 墙（X ∈ [0,0.5]、Y ∈ [0.5,2.5)、Z ∈ [−2,2)）
            let mut vol = vxl_phys_terrain::voxel::VoxelVolume::new(
                Vec3::new(-4.0, 0.0, -4.0),
                0.5,
                16,
                16,
                16,
            );
            vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 0.5, 4.0));
            vol.fill_box(Vec3::new(0.0, 0.5, -2.0), Vec3::new(0.5, 2.5, 2.0));
            let filled0 = vol.filled_count();
            w.add_voxel(vol);
            // 炮弹：半 0.4 的盒，以 12 m/s 冲墙
            let bullet = w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(-3.0, 1.0, 0.0),
                Quat::IDENTITY,
                2000.0,
            );
            w.bodies.linvel[bullet as usize] = Vec3::new(12.0, 0.0, 0.0);
            let mut debris_total = 0usize;
            for _ in 0..240 {
                w.step();
                debris_total += w.apply_impact_destruction(0, 3.0, 1000.0);
            }
            let filled1 = w.providers.voxel(0).unwrap().filled_count();
            let bodies = w.bodies.len();
            (debris_total, filled0 - filled1, bodies, w.state_hash())
        };
        let (debris, carved, _bodies, hash1) = run();
        assert!(debris > 0, "应触发冲击破坏（产出碎块）");
        assert!(carved > 0, "墙体素应减少（挖洞）carved={carved}");
        let (debris2, carved2, _b, hash2) = run();
        assert_eq!((debris, carved), (debris2, carved2), "破坏应可复现（计数）");
        assert_eq!(hash1, hash2, "破坏应可复现（末态哈希逐位一致）");
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
