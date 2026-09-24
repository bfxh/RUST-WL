//! world_build：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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
        bodies.materials[0] = Material::new(config.friction, config.restitution); // 默认材质（槽 0；逐材质用 add_material 覆盖）
                                                                                  // §6 调度注入：threads ≤ 1 → 串行（默认，回归对照基准）。
        let jobs: Box<dyn JobSystem> = if config.threads > 1 {
            Box::new(ScopedPool::new(config.threads))
        } else {
            Box::new(SerialJobSystem)
        };
        Self {
            broad,
            narrow: DefaultNarrowPhase::new(skin).into(),
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
            fluid_2b: Vec::new(),
            fluid_boundary_scratch: Vec::new(),
            fluid_boundary_covered: Vec::new(),
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
        self.fluids.push((sys, boundaries.to_vec(), None));
        self.fluid_2b.push(false);
        self.fluids.len() - 1
    }

    /// 注册流体系统并**开启 2b（Akinci 两层边界粒子）**：每 tick 按**近域体**
    /// （静态 + 动态，形状受支持）重建边界粒子（`FluidSystem::set_boundary_particles`），
    /// 并把**反作用（力 + 绕体原点的力矩）**回流到体上（`medium_pass` 段③）。
    /// 被覆盖的体由 2b 接管 ⇒ **2a 的介质场浮力/阻力对这些体让位**（不叠加）；
    /// 形状不受支持或不在近域的体仍走 2a 粗档（既不叠加、也不留空）。
    ///
    /// 与 [`Self::add_fluid`] 的唯一区别就是这一条 ⇒ 默认档位、既有场景逐位不变。
    pub fn add_fluid_with_boundary_coupling(
        &mut self,
        sys: vxl_phys_fluid::FluidSystem,
        boundaries: &[u32],
    ) -> usize {
        let id = self.add_fluid(sys, boundaries);
        if let Some(f) = self.fluid_2b.get_mut(id) {
            *f = true;
        }
        id
    }
}
