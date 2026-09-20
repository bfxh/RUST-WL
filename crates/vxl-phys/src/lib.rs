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
//! 液体域（切片1，单向耦合）：每 tick 体子步全部完成后，`fluid_pass` 以流体
//! 自身的固定子步数推进全部流体系统（`World::add_fluid` 注册）。
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
pub use vxl_phys_narrow::{CompoundChild, ContactPoint, DefaultNarrowPhase, Manifold, NarrowPhase};
pub use vxl_phys_replay::{Recorder, StateHash, Xxh3Hash};
pub use vxl_phys_solver::joints::{Joint, JointKind, JointSet};
pub use vxl_phys_solver::{ccd, warm_match_stats_take, ImpulseSolver};
pub use vxl_phys_terrain::TerrainSet;

/// 提供者条目（统一 id 空间：体素体 / 高斯喷溅场…）。
pub enum ProviderEntry {
    Voxel(vxl_phys_terrain::voxel::VoxelVolume),
    /// **高斯喷溅场**（喷溅域的物理代理：隐式场提供者，见 `vxl-phys-splat`）。
    Splat(vxl_phys_splat::GaussianSplatField),
    /// **三角网格**（网格域：静态关卡几何，薄壳接触，见 `vxl-phys-terrain::mesh`）。
    Mesh(vxl_phys_terrain::mesh::TriMesh),
}

/// **复合体的精确质量属性**（关于**复合体原点**，与本仓"体原点即质心"的表示一致）：
/// 按子形状求和 + 平行轴。
///
/// `M = Σ mᵢ`；`I = Σ [Rᵢ·diag(Iᵢ)·Rᵢᵀ + mᵢ(|oᵢ|²E − oᵢoᵢᵀ)]`，旋转项只取对角
/// （`(R I Rᵀ)_kk = Σ_a R[k][a]²·I_a`，本仓惯量表示就是逐个 `Vec3` 对角）。
///
/// ⚠️ 两条取舍（见 `TECH-SURVEY.md` A9 ④）：① 平行轴用**原点**而非真质心 ⇒ 质心偏移的力矩
/// 效应不建模（与锥同款；本仓无质心偏移字段）；② **离对角项被丢弃**（对称/轴对齐的复合体
/// 本来就为零）。空复合体或零质量返回 `None`（调用方保留兜底近似）。
fn compound_mass_props(children: &[CompoundChild], density: f32) -> Option<(f32, Vec3)> {
    if children.is_empty() {
        return None;
    }
    let mut m_total = 0.0f32;
    let mut i_diag = Vec3::ZERO;
    for ch in children {
        let mp = vxl_phys_core::mass_props(&ch.shape, density);
        let m = mp.mass;
        if m <= 0.0 || !m.is_finite() {
            continue;
        }
        m_total += m;
        let i_own = Vec3::new(
            1.0 / mp.local_inv_inertia.x,
            1.0 / mp.local_inv_inertia.y,
            1.0 / mp.local_inv_inertia.z,
        );
        let r = vxl_phys_core::Mat3::from_quat(ch.rot);
        let rot_diag = Vec3::new(
            r.m[0][0] * r.m[0][0] * i_own.x
                + r.m[0][1] * r.m[0][1] * i_own.y
                + r.m[0][2] * r.m[0][2] * i_own.z,
            r.m[1][0] * r.m[1][0] * i_own.x
                + r.m[1][1] * r.m[1][1] * i_own.y
                + r.m[1][2] * r.m[1][2] * i_own.z,
            r.m[2][0] * r.m[2][0] * i_own.x
                + r.m[2][1] * r.m[2][1] * i_own.y
                + r.m[2][2] * r.m[2][2] * i_own.z,
        );
        let o = ch.offset;
        let o2 = o.length_squared();
        let par = Vec3::new(o2 - o.x * o.x, o2 - o.y * o.y, o2 - o.z * o.z) * m;
        i_diag = i_diag + rot_diag + par;
    }
    if m_total <= 0.0 || !m_total.is_finite() {
        return None;
    }
    let inv = Vec3::new(
        if i_diag.x > 0.0 { 1.0 / i_diag.x } else { 0.0 },
        if i_diag.y > 0.0 { 1.0 / i_diag.y } else { 0.0 },
        if i_diag.z > 0.0 { 1.0 / i_diag.z } else { 0.0 },
    );
    Some((1.0 / m_total, inv))
}

/// 形状的平均迎风面积估计（阻力用）：盒 = 三对面面积均值，球 = πr²，
/// 其余（含外壳）取局部 AABB 近似；不可估计返回 0（不施加阻力）。
fn cross_section_area(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half } => 4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0,
        Shape::Sphere { radius } => std::f32::consts::PI * radius * radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => 2.0 * radius * (2.0 * half_height) / 2.0 + std::f32::consts::PI * radius * radius,
        // 胶囊：中段按圆柱（含帽时略低估，阻力估计够用）。
        Shape::Capsule {
            half_height,
            radius,
        } => 2.0 * radius * (2.0 * half_height) / 2.0 + std::f32::consts::PI * radius * radius,
        // 锥：侧面投影按三角剖面（底宽 2r、高 2h ⇒ 面积 r·h）另加底圆盘；
        // 锥尖一端无面 ⇒ 比同尺寸圆柱略低估（阻力估计够用）。
        Shape::Cone {
            half_height,
            radius,
        } => radius * half_height + std::f32::consts::PI * radius * radius,
        Shape::ConvexHull { half, .. } => {
            4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0
        }
        // 复合体：同外壳（局部 AABB 并集半长的外接盒近似；阻力估计够用）。
        Shape::Compound { half, .. } => {
            4.0 * (half.x * half.y + half.y * half.z + half.z * half.x) / 3.0
        }
        Shape::HeightField(_) | Shape::Provider(_) => 0.0,
    }
}

/// 外部碰撞提供者集合（门面持有；实现 `interop::ProviderColliders` 供窄相查询）。
#[derive(Default)]
pub struct Providers {
    entries: Vec<ProviderEntry>,
}

impl Providers {
    /// 注册体素体，返回其 id（= 注册序，全提供者共用一个 id 空间）。
    pub fn push(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Voxel(vol));
        id
    }

    /// 注册高斯喷溅场（同 id 空间）。
    pub fn push_splat(&mut self, field: vxl_phys_splat::GaussianSplatField) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Splat(field));
        id
    }

    /// 注册三角网格（静态关卡；同 id 空间）。
    pub fn push_mesh(&mut self, mesh: vxl_phys_terrain::mesh::TriMesh) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Mesh(mesh));
        id
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// provider(id) 的世界包围盒（宽相 AABB 供给；与 `CollisionProvider::bounds` 同义）。
    pub fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => {
                use vxl_phys_core::interop::ProviderColliders;
                f.bounds(id)
            }
            ProviderEntry::Mesh(m) => {
                use vxl_phys_core::interop::ProviderColliders;
                m.bounds(id)
            }
        }
    }

    pub fn voxel(&self, id: u32) -> Option<&vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 网格只读视图（渲染/诊断）。
    pub fn mesh(&self, id: u32) -> Option<&vxl_phys_terrain::mesh::TriMesh> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Mesh(m) => Some(m),
            _ => None,
        }
    }

    pub fn voxel_mut(&mut self, id: u32) -> Option<&mut vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get_mut(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 喷溅场只读视图（渲染桥/诊断）。
    pub fn splat(&self, id: u32) -> Option<&vxl_phys_splat::GaussianSplatField> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Splat(f) => Some(f),
            _ => None,
        }
    }
}

impl vxl_phys_core::interop::ProviderColliders for Providers {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => f.bounds(id),
            ProviderEntry::Mesh(m) => m.bounds(id),
        }
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
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_box_voxel(v, half, pos, rot, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_box(id, half, pos, rot, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_box(id, half, pos, rot, skin, out),
            None => false,
        }
    }

    fn contacts_point(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_point_voxel(v, p, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_point(id, p, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_point(id, p, skin, out),
            None => false,
        }
    }

    /// 流体边界口径：体素走内点鲁棒变体（截断 SDF 在薄壁内部被格间内面
    /// 主导 ⇒ 中心差分法线可指向固体深处，投影穿壁隧逃——见切片1实测）；
    /// 其余提供者（解析面/半空间无内点歧义）沿用 `contacts_point`。
    fn contacts_point_boundary(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_point_voxel_solid(v, p, skin, out)
            }
            _ => self.contacts_point(id, p, skin, out),
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
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_sphere_voxel(v, center, radius, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_sphere(id, center, radius, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_sphere(id, center, radius, skin, out),
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
    /// 接触求解 + 关节求解（同一阶段计时；关节通道无关节时零成本）。
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

/// **解算前**的冲击快照（破坏管线消费；见 `World::record_impacts`）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactRecord {
    /// provider id（体素/网格/喷溅同 id 空间）。
    pub provider: u32,
    /// 冲击体（动态体）索引。
    pub body: u32,
    /// 接触点（流形点均值，世界系）。
    pub point: Vec3,
    /// 沿接触法向的接近速度（m/s，> 0 表示正在靠近）。
    pub approach: f32,
    /// 该体解算前的速度（挖坑方向用）。
    pub velocity: Vec3,
}

/// 物理世界（固定步长契约：调用方以 `config.dt` 的整数倍节拍调用 `step`）。
/// **检测每步一次**（实验开关；`false` = 现行"每子步检测"）。
///
/// `true` 时：宽相/窄相只在每个 tick 的**首子步**跑，后续子步复用同一张流形表。
/// 动机：文档算过它值 **−27% 帧**（金字塔相位里 429 µs 是同一帧内的第二遍重复检测）。
/// 本仓 2026-09-14 的 P1 实验（`00a1ebf`）在塔上崩（|v| 44.4）并据此宣告"必须与 P2
/// 同批"，但：**那次实现未入档**（该提交只改文档），归因已被两次修正
/// （见 `EXPERIMENTS.md` 末节 K/K.1 与 DESIGN §10 的更正注），且当时之后落地了三件
/// 相关能力——`build_constraint` 每子步从**当前位姿**重算深度/锚点、warm 表稠密槽位、
/// A3 按组紧凑（求解热路径与 `BodySet` 解耦）。⇒ 值得重测（K.1 已证 tick 内复用
/// 流形时特征 ID 逐子步完全相同 ⇒ warm 精确命中、暖启动完整）。
fn detect_once_per_tick() -> bool {
    false
}

pub struct World {
    pub config: PhysConfig,
    pub bodies: BodySet,
    pub terrain: TerrainSet,
    /// 宽相（§2.3 主路径 = 增量 BVH；可用 `with_broadphase` 换网格等实现）。
    pub broad: Box<dyn BroadPhase>,
    pub narrow: DefaultNarrowPhase,
    pub solver: ImpulseSolver,
    /// **关节约束族**（§2.5）：接触解算之后、位置积分之前整帧求解一遍；
    /// 空集时零成本（`solve` 首行短路）。
    pub joints: JointSet,
    pub fields: FieldRegistry,
    /// 任务调度（§6 依赖注入：SerialJobSystem / ScopedPool / 自定义实现）。
    pub jobs: Box<dyn JobSystem>,
    /// 帧级相位暂存（§0.1 #10；M0 接入 = 哈希规范化缓冲）。
    pub arenas: FrameArenas,
    pub tick: u64,
    pairs: Vec<(u32, u32)>,
    manifolds: Vec<Manifold>,
    ccd_manifolds: Vec<Manifold>,
    /// **解算前的冲击记录**（破坏管线消费）：接触生成后、求解之前快照。
    /// 不能读"解算后"的速度——子步/迭代会把法向接近速度解得接近 0，管线
    /// 因此看不见这次冲击（实测：2 子步下 12 m/s 炮弹撞墙不再挖洞）。
    impacts: Vec<ImpactRecord>,
    hf_bounds: Vec<Aabb>,
    /// 外部碰撞提供者集合（体素/网格…；ROUTE §2.1 兼容轴）与其 AABB。
    providers: Providers,
    provider_bounds: Vec<Aabb>,
    /// 已注册流体系统（液体域；边界 provider id 随行存档）。
    fluids: Vec<(vxl_phys_fluid::FluidSystem, Vec<u32>)>,
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
            joints: JointSet::default(),
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
            impacts: Vec::new(),
            hf_bounds: Vec::new(),
            providers: Providers::default(),
            provider_bounds: Vec::new(),
            fluids: Vec::new(),
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

    /// 注册**三角网格**（网格域：静态关卡几何）：返回 provider id + 静态 marker 体。
    /// 接触语义为薄壳（`depth = skin − dist(最近面)`，法线 = 面法线），见
    /// `vxl_phys_terrain::mesh` 模块文档（含「皮肤带 < bin/2」精度前提）。
    pub fn add_mesh(&mut self, mut mesh: vxl_phys_terrain::mesh::TriMesh) -> BodyId {
        mesh.build_grid(); // 加速结构（桶边长 = max(均边, 0.5)）
        let id = self.providers.push_mesh(mesh);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }

    /// 注册**高斯喷溅场**（喷溅域）：返回 provider id + 生成静态 marker 体。
    /// 刚体（盒/球/外壳）经统一提供者通道与喷溅体接触（隐式场 `(τ−σ)/|∇σ|`）。
    pub fn add_splat_field(&mut self, mut field: vxl_phys_splat::GaussianSplatField) -> BodyId {
        field.rebuild_grid(); // 加速结构（与全扫逐位一致；核数不足则内部退回全扫）
        let id = self.providers.push_splat(field);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }

    /// 添加一条**关节**（§2.5 关节约束族：球/转动/固定/棱柱/距离）。
    /// 锚点/轴均为体局部量；返回关节 id（关节序即求解序 ⇒ 确定性）。
    /// v1 未接限位与马达（见 `vxl_phys_solver::joints` 模块文档）。
    pub fn add_joint(&mut self, joint: Joint) -> u32 {
        self.joints.add(joint)
    }

    /// 由 provider marker 体（[`Self::add_voxel`]/`add_mesh`/`add_splat_field`
    /// 返回的静态体）反查 provider id（流体边界列表用，见 [`Self::add_fluid`]）。
    pub fn provider_id_of(&self, body: BodyId) -> Option<u32> {
        match &self.bodies.shape[body as usize] {
            Shape::Provider(id) => Some(*id),
            _ => None,
        }
    }

    /// 注册**流体系统**（液体域，WCSPH）：返回流体索引（[`Self::fluids`] 取用）。
    /// `boundaries` = 边界碰撞的 provider id 列表（与体素/网格同一 id 空间，
    /// 经 [`Self::provider_id_of`] 由 marker 体反查）；切片1为单向耦合——
    /// 流体被刚体几何约束（驻留投影），对刚体无反作用（水推箱属切片2）。
    pub fn add_fluid(&mut self, mut sys: vxl_phys_fluid::FluidSystem, boundaries: &[u32]) -> usize {
        sys.set_boundaries(boundaries);
        self.fluids.push((sys, boundaries.to_vec()));
        self.fluids.len() - 1
    }

    /// 已注册流体系统及其边界 provider id（渲染读 `.0.positions()` / `.0.velocities()`）。
    pub fn fluids(&self) -> &[(vxl_phys_fluid::FluidSystem, Vec<u32>)] {
        &self.fluids
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

    /// **球域挖洞**（任意形状切割第一步；爆炸/弹坑形态）：提取球内格 → 碎块 + 移除。
    /// 返回碎块数。确定性：格心判据 + 固定贪心扫描序（见 `extract_sphere`）。
    pub fn carve_sphere(&mut self, id: u32, center: Vec3, radius: f32, density: f32) -> usize {
        let Some(vol) = self.providers.voxel_mut(id) else {
            return 0;
        };
        let boxes = vol.extract_sphere(center, radius);
        let n = boxes.len();
        for (c, h) in boxes {
            let mass = density * 8.0 * h.x * h.y * h.z;
            self.bodies
                .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, mass.max(1e-3));
        }
        self.refresh_provider_bounds();
        n
    }

    /// **Voronoi 预断裂（M3）**：把域内体素按「距最近种子」分块（划分，守恒），
    /// 每块提取为刚体碎块并移除。`seeds` 由调用方给（可用
    /// `vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered` 生成确定性抖动种子）。
    /// 返回碎块总数。
    pub fn fracture_voronoi(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        seeds: &[Vec3],
        density: f32,
    ) -> usize {
        let Some(vol) = self.providers.voxel_mut(id) else {
            return 0;
        };
        let cells = vol.fracture_voronoi(min, max, seeds);
        let mut n = 0usize;
        for (_si, boxes) in cells {
            for (c, h) in boxes {
                let mass = density * 8.0 * h.x * h.y * h.z;
                self.bodies
                    .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, mass.max(1e-3));
                n += 1;
            }
        }
        self.refresh_provider_bounds();
        n
    }

    /// **M3 冲击破坏**（调用方在每次 `step()` 之后调用）：对指定 provider，
    /// 把本 tick **解算前**记录的冲击（沿接触法向的接近速度 ≥ `speed_threshold`）
    /// 转成弹坑：球心 = 接触点 + 冲击方向 × 1.05r，半径随冲击速度增长（0.25..0.9 m），
    /// 碎块**静止生成**（引擎不凭空造动量）。返回生成的碎块数。
    /// 确定性：按记录序处理、挖域为轴对齐盒、提取顺序固定（见 `extract_boxes`）。
    ///
    /// 记录在窄相之后、求解之前快照（`record_impacts`）：管线读"解算后速度"会
    /// 漏掉已被子步解掉的冲击（这正是 2 子步配置下炮弹不再挖洞的根因）。
    pub fn apply_impact_destruction(
        &mut self,
        id: u32,
        speed_threshold: f32,
        density: f32,
    ) -> usize {
        // 先在只读扫描里收集「挖点」（按冲击记录序），再逐个挖 —— 保持确定性。
        let mut digs: Vec<(Vec3, f32, Vec3, f32)> = Vec::new();
        for rec in &self.impacts {
            if rec.provider != id || !self.bodies.is_dynamic(rec.body as usize) {
                continue;
            }
            // **冲击判据 = 沿接触法向的接近速度**（不是体速！）：贴地滑行是切向
            // 运动，体速很大但不该破坏（实测：8 m/s 滑行把脚下地板挖穿）。
            let approach = rec.approach;
            if approach < speed_threshold {
                continue;
            }
            let v = rec.velocity;
            let other = rec.body;
            let sp = approach;
            // 接触点 = 记录时的流形点均值（确定性；已在 record_impacts 算好）
            let c = rec.point;
            // 冲击体沿冲击方向的半径（保守：包围球半径）——挖域从它之外开始
            let reach = self.bodies.shape[other as usize]
                .bounding_sphere_radius()
                .min(2.0);
            digs.push((c, sp, v, reach));
        }
        let mut total = 0usize;
        for (c, sp, v, reach) in digs {
            // 挖出半径随**实际冲击速度**增长（钳到 0.25..0.9 m）
            let r = (0.2 + 0.06 * sp).clamp(0.25, 0.9);
            let dir = if v.length_squared() > 1e-9 {
                v.normalize()
            } else {
                Vec3::ZERO
            };
            // **弹坑 = 球域**（任意形状切割第一步）：球心 = 接触点 + 冲击方向 ×
            // 1.05r ⇒ 坑的**近缘正好落在接触点**、整体在材料里（薄墙会被打穿，
            // 物理如此）；与冲击体只在接近点相切、不重叠。
            let _ = reach;
            let center = c + dir * (r * 1.05);
            // 碎块**静止生成**（初速留给调用方用 `spawn_box_debris_vel` 显式给；
            // 引擎不凭空造动量——「继承半速」实测是能量源，已否）。
            total += self.carve_sphere(id, center, r, density);
        }
        total
    }

    /// 注册凸体外壳（点云，局部坐标）→ hull id。
    pub fn add_hull(&mut self, points: Vec<Vec3>) -> u32 {
        self.narrow.add_hull(points)
    }

    /// 本 tick 的冲击快照（只读；诊断/外部管线用）。
    pub fn impacts(&self) -> &[ImpactRecord] {
        &self.impacts
    }

    /// 窄相之后、求解之前：把「动体 × provider」对的接近速度与接触点快照进
    /// `self.impacts`（本子步重记 ⇒ 每次 `step` 结束时是**最后一个子步**的接触
    /// 快照，与"读末态流形"的旧语义对齐，但速度是解算前的）。
    fn record_impacts(&mut self) {
        self.impacts.clear();
        for m in &self.manifolds {
            let (sa, sb) = (
                self.bodies.shape[m.a as usize],
                self.bodies.shape[m.b as usize],
            );
            let (other, prov, other_is_a) = match (sa, sb) {
                (Shape::Provider(p), _) => (m.b, p, false),
                (_, Shape::Provider(p)) => (m.a, p, true),
                _ => continue,
            };
            if !self.bodies.is_dynamic(other as usize) {
                continue;
            }
            let v = self.bodies.linvel[other as usize];
            // 法线 a→b：other 在 a 侧 ⇒ 接近速度 = v·n；在 b 侧 ⇒ −v·n。
            let approach = if other_is_a {
                v.dot(m.normal)
            } else {
                -v.dot(m.normal)
            };
            let n = m.points.len().max(1) as f32;
            let mut c = Vec3::ZERO;
            for p in m.points.iter() {
                c += p.point;
            }
            self.impacts.push(ImpactRecord {
                provider: prov,
                body: other,
                point: c * (1.0 / n),
                approach,
                velocity: v,
            });
        }
    }

    /// **凸体预断裂**（「更一般的凸体/网格切割」的凸体侧）：原壳 ✕ 种子 ⇒
    /// 逐格生成凸碎块体（tiling 原体，局部坐标；缺口 = 种子在体外）。
    /// 典型用法：装载期把完整壳换成一堆碎块体，撞击即自然散架。
    pub fn spawn_hull_pieces(
        &mut self,
        hull: u32,
        seeds: &[Vec3],
        pos: Vec3,
        rot: Quat,
        density: f32,
    ) -> Vec<u32> {
        let Some(regions) = self.narrow.fracture_hull(hull, seeds) else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(regions.len());
        for region in regions {
            if region.len() < 4 {
                continue; // 退化格（无体积）
            }
            let id = self.narrow.add_hull(region);
            out.push(self.spawn_hull_body(id, pos, rot, density));
        }
        out
    }

    /// 外壳点云（局部；渲染/转储用）。
    pub fn hull_points(&self, hull: u32) -> &[Vec3] {
        self.narrow.hull_points(hull)
    }

    /// 生成**凸体外壳动态体**（多边形域）：点云已在 `add_hull` 注册。
    /// 半长取点云局部 AABB（宽相/惯量近似）；质量按 AABB 盒密度。
    pub fn spawn_hull_body(&mut self, hull: u32, pos: Vec3, rot: Quat, density: f32) -> u32 {
        let half = self.narrow.hull_half_extents(hull);
        self.bodies.push_dynamic(
            Shape::ConvexHull { hull, half },
            pos,
            rot,
            density.max(1e-3),
        )
    }

    /// 注册复合体（子形状 = 形状 + 局部平移/旋转）→ compound id。
    pub fn add_compound(&mut self, children: Vec<CompoundChild>) -> u32 {
        self.narrow.add_compound(children)
    }

    /// 子形状局部 AABB 并集半长（宽相/惯量近似用；空复合体 = ZERO）。
    pub fn compound_half_extents(&self, compound: u32) -> Vec3 {
        self.narrow.compound_half_extents(compound)
    }

    /// 复合体子形状表（诊断/外部管线用）。
    pub fn compound_children(&self, compound: u32) -> &[CompoundChild] {
        self.narrow.compound_children(compound)
    }

    /// 生成**复合体动态体**：子形状已在 `add_compound` 注册。
    /// 半长取子形状局部 AABB 并集（保守：子半长按包围球）；质量暂按该并集盒近似
    /// （精确并集 = 按子形状质量求和 + 平行轴，待接；见 `TECH-SURVEY.md` A9 ④）。
    pub fn spawn_compound_body(
        &mut self,
        compound: u32,
        pos: Vec3,
        rot: Quat,
        density: f32,
    ) -> u32 {
        let half = self.narrow.compound_half_extents(compound);
        let density = density.max(1e-3);
        let id = self
            .bodies
            .push_dynamic(Shape::Compound { compound, half }, pos, rot, density);
        // 质量/惯量按**子形状精确求和**覆写（`push_dynamic` 里只有并集 AABB 兜底近似）。
        if let Some((inv_mass, inv_inertia)) =
            compound_mass_props(self.narrow.compound_children(compound), density)
        {
            let i = id as usize;
            self.bodies.inv_mass[i] = inv_mass;
            self.bodies.local_inv_inertia[i] = inv_inertia;
        }
        id
    }

    /// 生成**复合体静态体**（同 `add_static` 语义）。
    pub fn add_compound_static(&mut self, compound: u32, pos: Vec3, rot: Quat) -> u32 {
        let half = self.narrow.compound_half_extents(compound);
        self.bodies
            .push_static(Shape::Compound { compound, half }, pos, rot)
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
    fn fluid_pass(&mut self) {
        if self.fluids.is_empty() {
            return;
        }
        for (sys, _) in self.fluids.iter_mut() {
            sys.step(self.config.dt, &self.providers);
        }
    }

    /// **介质通道（喷溅场作介质）**：对每个动体 × 每个「密度 > 0」的喷溅场均采样，
    /// 累加二次阻力 `F = −½·ρ·Cd·A·|v_rel|·v_rel`（单侧耦合，见 `MediumField`）。
    /// 确定性：场按 id 序、体按索引序；中线量 `v_rel = v − 介质流速`。
    /// 零成本短路：介质密度 0 的场不采样（不是介质的喷溅场完全不受影响）。
    fn medium_pass(&mut self) {
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

    fn substep(&mut self, dt: f32, first: bool, reuse_manifolds: bool) {
        // 1) 力场（重力在 World::new 注入注册表）+ 介质耦合（喷溅场作介质）。
        // 计时走跨目标探针：wasm32-unknown-unknown 无时钟（`Instant::now()`
        // 会 panic），该目标下退化为 0；原生行为不变。
        let t0 = vxl_phys_core::probe::start();
        self.fields.apply(&mut self.bodies);
        self.medium_pass();
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
            self.narrow
                .set_predict_dt(if reuse_manifolds { self.config.dt } else { 0.0 });
            self.narrow.collide(
                &self.bodies,
                &pairs,
                self.terrain.slice(),
                &self.providers,
                &mut self.manifolds,
                self.jobs.as_ref(),
            );
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
                // CCD 逐采样已在时间维上走路径 ⇒ 不再叠加视野预测（显式清零，
                // 否则会继承主窄相调用设的 `predict_dt`）。
                self.narrow.set_predict_dt(0.0);
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
                    // **只有「正在接近表面」的采样才算命中**（命中判据细化，本轮修复）：
                    // 贴地滑行/静置的体在每个采样都天生有接触，按「有接触即命中」会
                    // 把它钳回起点、原地锁死（实测：弹体滑到墙前 0.1 m 停住）。
                    // 接近判据：法线指向动体 ⇒ 速度沿它 < 0 即压向表面。
                    let closing = n_into_body.dot(v) < 0.0;
                    if closing {
                        hit = Some((t, n_into_body));
                        break;
                    }
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

    /// 立方体点云（外壳测试用；顶点序固定 ⇒ 确定性）。
    fn cube_hull_points(half: f32) -> Vec<Vec3> {
        let mut pts = Vec::new();
        for &x in &[-half, half] {
            for &y in &[-half, half] {
                for &z in &[-half, half] {
                    pts.push(Vec3::new(x, y, z));
                }
            }
        }
        pts
    }

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

    /// **M0.3 液体域**（ROUTE §7）：流体块经门面落入**体素盆**（地板+四壁，
    /// 一并覆盖体素 provider 的顶面/内壁/角点接触路径）并停驻。
    /// `add_fluid` + `fluid_pass` 端到端；单向耦合，marker 体不受扰动。
    /// （不用悬浮板：驻留投影不消耗切向速度，冲击横流会沿板面滑出板缘——
    /// 那是正确物理，但场景里板外无物，跑出者永远下坠，断言无从谈起。）
    #[test]
    fn fluid_rests_on_voxel_provider() {
        let mut w = World::new(PhysConfig::default());
        // 体素盆：外廓 0.8×0.8m、格边 0.2；地板层顶面 y=0.2，壁高到 y=0.8，
        // 内腔 0.4×0.4（与 fluid 域 Tank 测试同腔口）。
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-0.4, 0.0, -0.4), 0.2, 4, 4, 4);
        vol.fill_box(Vec3::new(-0.4, 0.0, -0.4), Vec3::new(0.4, 0.2, 0.4)); // 地板
        vol.fill_box(Vec3::new(0.2, 0.2, -0.4), Vec3::new(0.4, 0.8, 0.4)); // +x 壁
        vol.fill_box(Vec3::new(-0.4, 0.2, -0.4), Vec3::new(-0.2, 0.8, 0.4)); // −x 壁
        vol.fill_box(Vec3::new(-0.2, 0.2, 0.2), Vec3::new(0.2, 0.8, 0.4)); // +z 壁
        vol.fill_box(Vec3::new(-0.2, 0.2, -0.4), Vec3::new(0.2, 0.8, -0.2)); // −z 壁
        let marker = w.add_voxel(vol);
        let vid = w.provider_id_of(marker).expect("marker 是 provider 体");
        // 铸装近平衡块（8×8 贴腔口 + 5 层 ≈ 实测静水充高 0.44，320 粒）。
        // 不用方块自落：任何带落差的方块入盆，WCSPH 驻留瞬态（底部镜像
        // 鬼影密度尾 → 压实波在块顶心聚焦）都会把顶心粒子以近钳制速度
        // （实测 ~9 m/s）垂直喷过敞口壁顶——0.8 m 重落、0.1 m 轻落、触底
        // 就位皆复现，是 PLAN-0.3 §4 已记录的求解器瞬态而非边界失效；
        // 边界本身在全部场景中零穿壁。铸装后瞬态消失，本测试只验驻留与
        // 边界。
        let sys = vxl_phys_fluid::FluidSystem::new(
            vxl_phys_fluid::FluidConfig::default(),
            Vec3::new(-0.175, 0.25, -0.175),
            [8, 8, 5],
            0.05,
        );
        let fid = w.add_fluid(sys, &[vid]);
        for _ in 0..300 {
            w.step();
        }
        let f = &w.fluids()[fid].0;
        for (i, p) in f.positions().iter().enumerate() {
            assert!(p.y > 0.15, "粒子 {i} 穿透盆底：y = {}", p.y);
            assert!(p.y < 0.9, "粒子 {i} 飞出：y = {}", p.y);
            assert!(
                p.x.abs() < 0.45 && p.z.abs() < 0.45,
                "粒子 {i} 越出盆壁：({}, {})",
                p.x,
                p.z
            );
        }
        // 单向耦合：marker 体（provider）保持静止。
        assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
        assert!(w.health().is_clean());
    }

    /// **L1**：外壳落在高度场上（顶点采样；此前不受理 ⇒ 直接穿地）。
    #[test]
    fn hull_rests_on_heightfield() {
        let mut w = ground_world(); // 平地高度场（y = 0）
        let hull = w.add_hull(cube_hull_points(0.5));
        let b = w.spawn_hull_body(hull, Vec3::new(0.1, 3.0, -0.1), Quat::IDENTITY, 1.0);
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 静置在平地（y=0）上方：半长 0.5 ⇒ y ≈ 0.5
        assert!(y > 0.40 && y < 0.62, "y = {y}");
        assert!(w.health().is_clean());
    }

    /// **M3 多边形域**：凸体外壳落在体素地面上（外壳 × 提供者 = 顶点采样多点流形）。
    #[test]
    fn hull_rests_on_voxel_provider() {
        let mut w = World::new(PhysConfig::default());
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
        w.add_voxel(vol);
        let hull = w.add_hull(cube_hull_points(0.5));
        let b = w.spawn_hull_body(hull, Vec3::new(0.1, 2.5, -0.1), Quat::IDENTITY, 1.0);
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 静置在体素顶面（y=1.0）上方：外壳半长 0.5 ⇒ y ≈ 1.5
        assert!(y > 1.42 && y < 1.70, "y = {y}");
        assert!(!w.bodies.awake[b as usize], "外壳应已入睡");
        assert!(w.health().is_clean());
    }

    /// **M3 凸体切割**：立方体外壳 ✕ 8 种子 ⇒ 8 个凸碎块，逐个落在体素地面上。
    #[test]
    fn fractured_hull_pieces_rest_on_voxel_provider() {
        let mut w = World::new(PhysConfig::default());
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
        w.add_voxel(vol);
        let hull = w.add_hull(cube_hull_points(0.5));
        let seeds: Vec<Vec3> = [
            Vec3::new(-0.3, -0.25, -0.35),
            Vec3::new(-0.3, -0.25, 0.25),
            Vec3::new(-0.3, 0.35, -0.35),
            Vec3::new(-0.3, 0.35, 0.25),
            Vec3::new(0.3, -0.25, -0.35),
            Vec3::new(0.3, -0.25, 0.25),
            Vec3::new(0.3, 0.35, -0.35),
            Vec3::new(0.3, 0.35, 0.25),
        ]
        .to_vec();
        let pieces =
            w.spawn_hull_pieces(hull, &seeds, Vec3::new(0.0, 2.2, 0.0), Quat::IDENTITY, 1.0);
        assert_eq!(pieces.len(), 8, "应有 8 块");
        for _ in 0..900 {
            w.step();
        }
        // 全体落到地面带内且干净
        for &p in &pieces {
            let y = w.bodies.position[p as usize].y;
            assert!(y > 0.9 && y < 2.0, "碎块 y = {y} 不在带内");
        }
        assert!(w.health().is_clean());
    }

    /// **M3 喷溅域**：盒落在一团高斯喷溅上并停住（隐式场 σ(p) = Σ 核；
    /// 接触走统一提供者通道 `contacts_box`）。
    #[test]
    fn box_rests_on_gaussian_splat_field() {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(0.5);
        // 半径 1.2 球状栅格（间距 0.4、核半径 0.35）⇒ 顶面等值面 ≈ y = 1.6
        for i in -3..=3 {
            for j in -3..=3 {
                for k in -3..=3 {
                    let c = Vec3::new(i as f32 * 0.4, j as f32 * 0.4, k as f32 * 0.4);
                    if c.length() <= 1.2 {
                        f.push(vxl_phys_splat::Splat::isotropic(c, 0.35, 1.0));
                    }
                }
            }
        }
        assert!(f.len() > 100);
        w.add_splat_field(f);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(0.0, 3.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        // 停在等值面之上（盒半长 0.4 + 顶面 ≈ 1.6 ⇒ 中心 ≈ 2.0）
        assert!(y > 1.5 && y < 2.6, "y = {y}");
        assert!(w.health().is_clean());
    }

    /// **介质耦合（喷溅域第三层）**：稀薄喷溅云（σ < iso ⇒ 不产生接触）作介质
    /// ⇒ 落体被二次阻力减速；介质密度 0 时为纯自由落体（对照）。
    /// 口径：两条 run 仅差 `medium_density`，其余位姿/初始条件完全相同。
    #[test]
    fn splat_medium_drag_slows_falling_body() {
        let run = |medium_density: f32| -> (f32, f32) {
            let mut w = World::new(PhysConfig::default());
            let mut f = vxl_phys_splat::GaussianSplatField::new(4.0); // iso 高 ⇒ 纯介质、无接触
            for k in 0..16 {
                f.push(vxl_phys_splat::Splat::isotropic(
                    Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
                    0.45,
                    0.6,
                ));
            }
            f.medium_density = medium_density;
            f.medium_velocity = Vec3::ZERO;
            w.add_splat_field(f);
            let b = w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.3),
                },
                Vec3::new(0.0, 9.0, 0.0),
                Quat::IDENTITY,
                1.0,
            );
            for _ in 0..90 {
                w.step();
            }
            (
                w.bodies.position[b as usize].y,
                w.bodies.linvel[b as usize].y,
            )
        };
        let (y_free, v_free) = run(0.0);
        let (y_drag, v_drag) = run(6.0);
        println!("free: y={y_free:.3} v={v_free:.3} | drag: y={y_drag:.3} v={v_drag:.3}");
        assert!(
            y_drag > y_free + 0.2,
            "介质应显著减速：free y={y_free} drag y={y_drag}"
        );
        assert!(
            v_drag > v_free + 0.5,
            "末速应更高（落得更慢）：{v_free} vs {v_drag}"
        );
    }

    /// **网格域（M3 扩展）**：盒落在三角网格地面上并停住（薄壳接触；
    /// 8 顶点 + 6 面心采样 ⇒ 底四角多点接触）。
    #[test]
    fn box_rests_on_mesh_ground() {
        let mut w = World::new(PhysConfig::default());
        // 4×4 米网格地面（y = 0，朝上）
        let ground = vxl_phys_terrain::mesh::TriMesh::quad(
            Vec3::ZERO,
            Vec3::X * 2.0,
            Vec3::Z * 2.0,
            Vec3::Y,
        );
        w.add_mesh(ground);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.3, 2.0, -0.2),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..600 {
            w.step();
        }
        let y = w.bodies.position[b as usize].y;
        assert!(y > 0.42 && y < 0.70, "y = {y}"); // 静置在网格面上方（半长 0.5）
        assert!(w.health().is_clean());
    }

    /// **网格域**：球在斜网格上按**面法线**接触并沿坡下滑（球无滚阻必下滚——
    /// 因此断言「接触带内 + 法向速度被抑制 + 沿坡下滑」，而非「停在坡上」）。
    #[test]
    fn sphere_contacts_sloped_mesh_along_face_normal() {
        let mut w = World::new(PhysConfig::default());
        // 斜面：沿 +u 抬升（面法线 n 偏向 −X）
        let slope = 0.25f32;
        let n = Vec3::new(-slope, 1.0, 0.0).normalize();
        let u = Vec3::new(1.0, slope, 0.0).normalize() * 8.0;
        let v = Vec3::Z * 8.0;
        let ground = vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, u, v, n);
        w.add_mesh(ground);
        let ball = w.add_dynamic(
            Shape::Sphere { radius: 0.4 },
            Vec3::new(0.0, 1.5, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        // 触面后约 0.5 秒：仍在接触带内、法向速度被压制、沿 −u 下滑
        for _ in 0..90 {
            w.step();
        }
        let p = w.bodies.position[ball as usize];
        let vel = w.bodies.linvel[ball as usize];
        let dist = p.dot(n);
        let v_n = vel.dot(n);
        let down = vel.dot(u.normalize());
        assert!(
            dist > 0.30 && dist < 0.75,
            "应在接触带内（半径 0.4）：沿法线 {dist}"
        );
        assert!(v_n.abs() < 1.0, "法向速度应被接触抑制：{v_n}");
        assert!(down < -0.05, "应沿坡下滑（−u 方向）：v·u = {down}");
        assert!(w.health().is_clean());
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

    /// **CCD 回归**（本轮修复）：开启 CCD 后，**贴地滑行**的体不得被锁死。
    /// 修前：每个扫描采样都有地面接触 ⇒ 判「命中」⇒ 钳回起点、原地停住。
    /// 修后：只有「沿法向接近」的采样才算命中 ⇒ 切向滑行不受影响。
    #[test]
    fn ccd_does_not_lock_sliding_body() {
        let cfg = PhysConfig {
            ccd_speed_threshold: 5.0,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
        w.add_heightfield(hf);
        // 贴地盒（底面 y=0.5 略上方）以 8 m/s 沿 +X 滑行（超过 CCD 阈值 5）
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.55, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        w.bodies.linvel[b as usize] = Vec3::new(8.0, 0.0, 0.0);
        for _ in 0..60 {
            w.step();
        }
        let x = w.bodies.position[b as usize].x;
        // 摩擦会减速，但绝不该「原地不动」：修前 x ≈ 0，修后应有明显位移
        assert!(x > 2.0, "CCD 不应锁死滑行体：x = {x}");
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
