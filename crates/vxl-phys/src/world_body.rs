//! world_body：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl World {
    /// 该流体是否开了 2b（对账/测试用）。
    pub fn fluid_boundary_coupling(&self, fluid: usize) -> bool {
        self.fluid_2b.get(fluid).copied().unwrap_or(false)
    }

    /// 2b **覆盖集**快照（与 `bodies` 同序；上次边界粒子生成的结果）。
    pub fn fluid_boundary_covered(&self) -> &[bool] {
        &self.fluid_boundary_covered
    }

    /// 已注册流体系统及其边界 provider id（渲染读 `.0.positions()` / `.0.velocities()`；第三槽 = 卡上步进后端）。
    pub fn fluids(&self) -> &[crate::world_step::fluid_stepper::FluidSlot] {
        &self.fluids
    }

    /// **破坏（M3 第一块）**：把体素体盒域内的占据格转为**刚体碎块**
    /// （贪心合并成盒 → 逐个动态体），并从体素体里移除。返回碎块数。
    /// 确定性：提取顺序 = 固定扫描序（见 `VoxelVolume::extract_boxes`）；
    /// 碎块质量 = `density × 8·hx·hy·hz`。
    pub fn spawn_box_debris(&mut self, id: u32, min: Vec3, max: Vec3, density: f32) -> usize {
        self.spawn_box_debris_vel(id, min, max, density, Vec3::ZERO)
    }

    /// 同 [`spawn_box_debris`](Self::spawn_box_debris)，但碎块带初速 `vel`（冲击破坏用：碎块继承部分冲击速度）。
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
    pub(crate) fn record_impacts(&mut self) {
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
}
