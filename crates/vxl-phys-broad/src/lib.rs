//! # vxl-phys-broad
//!
//! 宽相（§2.3）：「增量 AABB 树（bvh2 风格）为主 + 大世界空间哈希网格」。
//! M1 落地 `BvhBroadPhase`（增量动态 BVH，`bvh::DynamicBvh`）为主实现；
//! M0 的 `GridBroadPhase`（均匀空间哈希网格）保留作对照/大世界叠加。
//! `wide::WideBvh`（8 路宽节点）为已验收但**未接入**的实验模块：树更新更快
//! （树峰 7.34→3.10ms）但叶粒度令候选数 8×、查询更慢（均 7.47→14.68ms）
//! ⇒ 净负，见 `BvhBroadPhase` 结论注与 docs/M1-PLAN.md。
//!
//! 确定性（§5）：树插入/移动按体索引升序；查询显式栈、先左后右；
//! 输出对 `(a, b)`（a < b）按字典序排序去重。两者绝不迭代哈希结构本身。

#![forbid(unsafe_code)]

pub mod bvh;
/// BVH8（宽**内部**节点 + 窄叶）——T2 尾数据布局**第二版原型**：先量「6 层窄叶
/// 遍历是否真比 18 层二叉便宜」再决定投不投增量侧（见模块头注）。
pub mod bvh8;
/// 8 路宽节点 BVH（T2 尾数据布局投入；**实验模块，未接入生产路径**——
/// 接入实测净负，见 `BvhBroadPhase` 结论注）。
pub mod wide;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, Quat, Shape, Vec3};

pub use bvh::DynamicBvh;
pub use bvh8::Bvh8;
pub use wide::WideBvh;

/// 轴对齐包围盒。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    #[inline]
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }

    /// 空盒（min = +∞、max = −∞）：`contains` 恒假——缓存失效哨兵。
    pub const EMPTY: Aabb = Aabb {
        min: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
        max: Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
    };

    /// `self` 是否完整包含 `o`（含边界）。
    #[inline]
    pub fn contains(&self, o: &Aabb) -> bool {
        o.min.x >= self.min.x
            && o.min.y >= self.min.y
            && o.min.z >= self.min.z
            && o.max.x <= self.max.x
            && o.max.y <= self.max.y
            && o.max.z <= self.max.z
    }

    /// 各向同性膨胀 `m`。
    #[inline]
    pub fn grown(&self, m: f32) -> Aabb {
        Aabb {
            min: self.min - Vec3::new(m, m, m),
            max: self.max + Vec3::new(m, m, m),
        }
    }
}

/// 形状 → 世界 AABB（含 margin 膨胀）。
/// 高度场体的包围盒由调用方经 `hf_bounds[id]` 提供（高度场数据在 narrow/terrain 层）。
pub fn shape_aabb(shape: &Shape, pos: Vec3, rot: Quat, margin: f32, hf_bounds: &[Aabb]) -> Aabb {
    let half = match *shape {
        Shape::Box { half } => {
            // 精确：|R|·half（旋转矩阵逐元素绝对值作用于半长）。
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][0].abs() * half.x + r.m[0][1].abs() * half.y + r.m[0][2].abs() * half.z,
                r.m[1][0].abs() * half.x + r.m[1][1].abs() * half.y + r.m[1][2].abs() * half.z,
                r.m[2][0].abs() * half.x + r.m[2][1].abs() * half.y + r.m[2][2].abs() * half.z,
            )
        }
        Shape::Sphere { radius } => Vec3::splat(radius),
        // 圆柱 ⊆ 外接盒（r, hh, r），经 |R| 变换保守。
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            let ext = Vec3::new(radius, half_height, radius);
            Vec3::new(
                r.m[0][0].abs() * ext.x + r.m[0][1].abs() * ext.y + r.m[0][2].abs() * ext.z,
                r.m[1][0].abs() * ext.x + r.m[1][1].abs() * ext.y + r.m[1][2].abs() * ext.z,
                r.m[2][0].abs() * ext.x + r.m[2][1].abs() * ext.y + r.m[2][2].abs() * ext.z,
            )
        }
        Shape::HeightField(id) => {
            let b = hf_bounds.get(id as usize).copied().unwrap_or(Aabb {
                min: Vec3::splat(0.0),
                max: Vec3::splat(0.0),
            });
            return Aabb {
                min: b.min - Vec3::splat(margin),
                max: b.max + Vec3::splat(margin),
            };
        }
    };
    Aabb {
        min: pos - half - Vec3::splat(margin),
        max: pos + half + Vec3::splat(margin),
    }
}

pub trait BroadPhase {
    /// 输出确定性有序对（a < b，字典序升序，去重）。
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        jobs: &dyn JobSystem,
    ) -> &[(u32, u32)];

    /// 步长告知（速度自适应边距用；每子步调用一次，dt ≤ 0 表示未知）。
    /// 默认空实现（不需要该信息的宽相可直接忽略）。
    fn set_step(&mut self, _dt: f32) {}

    /// 任意 AABB 命中查询（CCD 扫掠区域用，§4.12）；输出体 id 升序去重。
    /// 默认空实现（不需要该能力的宽相可直接忽略）。
    fn query_aabb(&mut self, _aabb: &Aabb, _out: &mut Vec<u32>) {}

    /// 诊断：上一帧子阶段耗时（µs）=(AABB, 树更新, 查询, 排序)；默认全 0。
    fn breakdown_us(&self) -> (u64, u64, u64, u64) {
        (0, 0, 0, 0)
    }

    /// 诊断：树高（链路审计；默认 0 = 不适用）。
    fn tree_height(&self) -> u32 {
        0
    }

    /// 诊断：上一帧候选总数（查询返回的候选条目数之和）——候选粒度审计；
    /// 默认 0 = 不适用。
    fn cand_total(&self) -> usize {
        0
    }
}

type CellKey = (i32, i32, i32);

/// 均匀空间哈希网格宽相。
pub struct GridBroadPhase {
    /// 网格边长（m）；应 ≥ 最大物体直径。
    pub cell_size: f32,
    skin: f32,
    cells_static: HashMap<CellKey, Vec<u32>>,
    cells_dynamic: HashMap<CellKey, Vec<u32>>,
    aabbs: Vec<Aabb>,
    pairs: Vec<(u32, u32)>,
}

impl GridBroadPhase {
    pub fn new(cell_size: f32, skin: f32) -> Self {
        Self {
            cell_size: if cell_size > 1e-4 { cell_size } else { 1.0 },
            skin,
            cells_static: HashMap::new(),
            cells_dynamic: HashMap::new(),
            aabbs: Vec::new(),
            pairs: Vec::new(),
        }
    }

    #[inline]
    fn cell_of(&self, v: Vec3) -> CellKey {
        const LIMIT: f32 = 1_000_000.0;
        let inv = 1.0 / self.cell_size;
        let x = (v.x * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let y = (v.y * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let z = (v.z * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        (x, y, z)
    }

    fn insert(
        cells: &mut HashMap<CellKey, Vec<u32>>,
        aabb: &Aabb,
        cell_min: CellKey,
        cell_max: CellKey,
        id: u32,
    ) {
        let mut x = cell_min.0;
        while x <= cell_max.0 {
            let mut y = cell_min.1;
            while y <= cell_max.1 {
                let mut z = cell_min.2;
                while z <= cell_max.2 {
                    cells.entry((x, y, z)).or_default().push(id);
                    z += 1;
                }
                y += 1;
            }
            x += 1;
        }
        let _ = aabb;
    }
}

impl BroadPhase for GridBroadPhase {
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        _jobs: &dyn JobSystem,
    ) -> &[(u32, u32)] {
        self.cells_static.clear();
        self.cells_dynamic.clear();
        self.pairs.clear();

        let n = bodies.len();
        self.aabbs.clear();
        self.aabbs.reserve(n);
        for i in 0..n {
            let aabb = shape_aabb(
                &bodies.shape[i],
                bodies.position[i],
                bodies.rot(i),
                self.skin,
                hf_bounds,
            );
            self.aabbs.push(aabb);
        }

        // 静态：仅插入。
        for i in 0..n {
            if bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            Self::insert(&mut self.cells_static, &self.aabbs[i], cmin, cmax, i as u32);
        }
        // 动态：先全部插入（dyn-dyn 去重靠 j > i），再统一查询。
        for i in 0..n {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            Self::insert(
                &mut self.cells_dynamic,
                &self.aabbs[i],
                cmin,
                cmax,
                i as u32,
            );
        }
        // 查询。
        for i in 0..n {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            let mut x = cmin.0;
            while x <= cmax.0 {
                let mut y = cmin.1;
                while y <= cmax.1 {
                    let mut z = cmin.2;
                    while z <= cmax.2 {
                        let key = (x, y, z);
                        if let Some(list) = self.cells_static.get(&key) {
                            for &j in list {
                                // 输出约定：对内 a < b（下游窄相/求解器的法线方向依赖它）。
                                let (a, b) = if (i as u32) < j {
                                    (i as u32, j)
                                } else {
                                    (j, i as u32)
                                };
                                if self.aabbs[a as usize].overlaps(&self.aabbs[b as usize]) {
                                    self.pairs.push((a, b));
                                }
                            }
                        }
                        if let Some(list) = self.cells_dynamic.get(&key) {
                            for &j in list {
                                if j as usize > i && self.aabbs[i].overlaps(&self.aabbs[j as usize])
                                {
                                    self.pairs.push((i as u32, j));
                                }
                            }
                        }
                        z += 1;
                    }
                    y += 1;
                }
                x += 1;
            }
        }

        self.pairs.sort_unstable();
        self.pairs.dedup();
        &self.pairs
    }

    fn query_aabb(&mut self, aabb: &Aabb, out: &mut Vec<u32>) {
        out.clear();
        for cells in [&self.cells_static, &self.cells_dynamic] {
            for list in cells.values() {
                for &j in list {
                    if self.aabbs.get(j as usize).is_some_and(|b| b.overlaps(aabb)) {
                        out.push(j);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

/// 增量 BVH 宽相（§2.3 主路径）。
///
/// - 代理更新按体索引升序（确定性）；树跨帧保持（增量 refit/平衡）；
/// - fat margin = 2×skin（≥ 2cm），静置帧零树操作；
/// - M1 不提供删体 API（`World` 无删体），叶子与体索引一一对应。
///
/// **宽节点（8 路）实测结论（T2 尾，2026-09-13）**：`wide::WideBvh` 已按其
/// 验收接入过一版，树峰 7.34 → **3.10ms**（叶含 8 体 ⇒ 多数移动落在叶盒内、
/// 免 refit），但查询均 7.47 → **14.68ms**、峰 22.64 → **33.50ms**——
/// 叶粒度让候选数 8×（实测 候选均 **208 万/tick**，真实接触对仅 ≈2.25/体），
/// 查询是**候选受限**而非遍历受限 ⇒ 净负收益，**回退**（详见 docs/M1-PLAN.md）。
pub struct BvhBroadPhase {
    tree: DynamicBvh,
    skin: f32,
    /// 当前子步 dt（`set_step` 注入；速度自适应 fat 边距用）。
    dt: f32,
    leaves: Vec<u32>,
    /// 精确 AABB（含 skin 膨胀），与树内 fat AABB 分离。
    aabbs: Vec<Aabb>,
    pairs: Vec<(u32, u32)>,
    /// 查询缓存（M1 T2 查询提速）：每体候选列表（arena 偏移/长度）+ 上次查询
    /// 所用 fat 盒。完备性：树不变式「精确盒 ⊆ fat 盒」+ 缓存 fat 盒自上次
    /// 查询未变 ⇒ 两体 fat 盒若在末态相交则在上次查询时已相交——候选必然
    /// 已入表（逃出缓存盒或树代理被重插时失效重查；配对双侧发射 + 去重）。
    cache_fat: Vec<Aabb>,
    cand_off: Vec<u32>,
    cand_len: Vec<u32>,
    cand_arena: Vec<u32>,
    /// 上一帧「参与查询」位（动态且清醒）——睡眠状态翻转检测用（见 `compute_pairs`
    /// 的不变式注：翻转帧必须全缓存失效，否则睡眠侧不查询 + 清醒侧缓存陈旧
    /// 会漏掉新接近对；不变式测试 `bvh_pairs_match_brute_force_across_frames` 守门）。
    prev_awake: Vec<bool>,
    /// 诊断：上一帧各子阶段耗时（µs）：(AABB, 树更新/重建, 查询, 排序)。
    pub last_breakdown_us: (u64, u64, u64, u64),
    /// 诊断：上一帧候选总数（查询返回的候选条目数之和）——候选粒度/b 因子审计。
    pub last_cand_total: usize,
}

impl BvhBroadPhase {
    pub fn new(skin: f32) -> Self {
        Self {
            tree: DynamicBvh::new((skin * 2.0).max(0.02)),
            skin,
            dt: 1.0 / 60.0,
            leaves: Vec::new(),
            aabbs: Vec::new(),
            pairs: Vec::new(),
            cache_fat: Vec::new(),
            cand_off: Vec::new(),
            cand_len: Vec::new(),
            cand_arena: Vec::new(),
            prev_awake: Vec::new(),
            last_breakdown_us: (0, 0, 0, 0),
            last_cand_total: 0,
        }
    }

    /// 速度自适应 fat 边距（M1 宽相提速的核心开关之一）：
    /// `base + |v_lin|·dt·1.5`，上钳 0.5m。旧档上限 0.25 在查询缓存 + 就地
    /// 生长到位后放宽（T2 第十四段）：代价结构已变——宽 fat 盒只增**候选数**
    /// 不再增**遍历数**（缓存命中时零遍历），快体（少）的多占候选换逃逸率降。
    /// 确定性：只由（速度, dt）决定；同一状态 → 同一边距 → 同一树形。
    #[inline]
    /// 速度自适应边距（`base + v·dt·K`，上限 0.5）。
    ///
    /// **K 的标定（2026-09-14 第三段实测，8B 同窗口）**：查询相位拆段计时发现
    /// 成本**几乎全在「逃逸重查」**（refresh 均 8.08ms）而精确过滤只有 1.22ms
    /// （3.8ns/条，与隔离档一致）⇒ 杠杆是**逃逸频率**，不是过滤局部性
    /// （修正此前的 32ns/条归因；W2/W7「边距不是杠杆」的结论也据此修正为
    /// 「收窄边距有害、放大才是方向」）。K 扫描：1.5（旧值）→ 14.67/43.02（K=6，
    /// 取此档）/ 14.94/48.97（K=12，峰反涨——快体大盒把候选推高）。
    /// `K=6` 实测 broad 均 17.03→**14.67**、峰 43.50→43.02、树 均 2.65→**1.77**；
    /// **逐位中性**（配对仍精确过滤 ⇒ 物理不变；`m0_gates` 哈希不变、twin_match ✓）。
    fn fat_margin_for(&self, v: Vec3) -> f32 {
        let base = (self.skin * 2.0).max(0.02);
        let speed = v.length();
        (base + speed * self.dt * 6.0).min(0.5)
    }

    /// 树高（诊断/负载审计：健康树 ≈ 1.4·log2(n)）。
    pub fn tree_height(&self) -> u32 {
        self.tree.root_height()
    }

    /// 诊断：宽相存储的精确 AABB（可视化/验证用，§12.3 调试可视化钩子）。
    pub fn stored_aabb(&self, i: usize) -> Option<Aabb> {
        self.aabbs.get(i).copied()
    }
}

impl BroadPhase for BvhBroadPhase {
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        jobs: &dyn JobSystem,
    ) -> &[(u32, u32)] {
        let t_aabb = std::time::Instant::now();
        self.pairs.clear();
        let n = bodies.len();
        // 注：绝不 clear——AABB 数组跨帧保留（睡眠/静态体沿用上帧值，
        // 增量分支只重算清醒动体；clear 会把它们清成零 AABB）。
        if self.aabbs.len() > n {
            self.aabbs.truncate(n);
        }
        self.aabbs.resize(
            n,
            Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            },
        );
        let threads = jobs.threads();
        // 0.5) 睡眠状态翻转检测（T2 查询缓存完备性的关键补丁）：任一体的
        //      「参与查询」位（动态且清醒）帧间变化 ⇒ 本帧全缓存失效。
        //      原因：睡眠侧不查询，其新邻居只能靠清醒侧查询兜底；而清醒侧
        //      可能因未逃出自身 fat 盒而复用旧候选表 → 漏对（不变式测试
        //      `bvh_pairs_match_brute_force_across_frames` 实测抓出）。
        //      完备性再证：两体 fat 盒均冻结且不相交时，精确盒（⊆ fat）不可能
        //      新相交——故「翻转帧」是唯一漏洞，翻转帧全体重查即封闭。
        //      确定性：只读 awake 位（纯状态），与线程数无关。
        self.prev_awake.resize(n, true);
        let mut flip = false;
        for i in 0..n {
            let participates = bodies.is_dynamic(i) && bodies.awake[i];
            if self.prev_awake[i] != participates {
                self.prev_awake[i] = participates;
                flip = true;
            }
        }
        if flip {
            for f in self.cache_fat.iter_mut() {
                *f = Aabb::EMPTY;
            }
        }
        // 全量 AABB 分支与「是否重建树」**解耦**（T2）：AABB 本就增量维护——
        // 仅首次（叶子未建）/ 体数变化（新体）需要全量重算（否则静态体 /
        // 睡眠体会带着零 AABB 进树）；纯「树链化超限」的重建 tick 复用现有
        // AABB（旧实现让重建 tick 白付一次 20 万体全量 AABB ≈16ms）。
        // 非全量分支 = 增量：只处理「清醒动体」（睡眠体与静态体位置不变，
        // AABB 与树内代理均无需更新——稳态零树操作，M1 规模档的成败手）。
        let leaves_missing = self.leaves.len() != n;
        let rebuild_due = !leaves_missing && n >= 32 && {
            let limit = 3.0 * (n as f32).log2() + 16.0;
            self.tree.root_height() as f32 > limit
        };
        let full = leaves_missing;
        // 0) AABB 计算：纯函数按下标写槽位（§6 并行契约）。分块并行且
        //    spawn 数受控（for_each_chunk_mut：≤ threads−1，禁止线程爆炸）。
        //    门槛 32768：单体内 AABB ≈ 30ns，低于此并行开销（≈0.6ms 启动）不划算。
        {
            let aabbs = &mut self.aabbs;
            let skin = self.skin;
            let bodies_ref: &BodySet = bodies;
            let hfs: &[Aabb] = hf_bounds;
            vxl_phys_core::schedule::for_each_chunk_mut(
                aabbs,
                threads,
                32768,
                |start, _len, slot| {
                    for (k, s) in slot.iter_mut().enumerate() {
                        let i = start + k;
                        if !full && (!bodies_ref.is_dynamic(i) || !bodies_ref.awake[i]) {
                            continue; // 睡眠/静态：位置未变，沿用上帧 AABB。
                        }
                        *s = shape_aabb(
                            &bodies_ref.shape[i],
                            bodies_ref.position[i],
                            bodies_ref.rot(i),
                            skin,
                            hfs,
                        );
                    }
                },
            );
        }
        let d_aabb = t_aabb.elapsed().as_micros() as u64;
        let t_tree = std::time::Instant::now();
        // 1) 代理更新（需要重建：中位分裂全量重建；否则增量只动清醒体）。
        //    面积启发式对结构化插入序（网格行优先）会链化，
        //    阈值 = 3·log2(n) + 16（确定性纯函数，不依赖时序）。
        //    树变异按体索引升序串行（结构操作不可并行）。
        if n >= 32 && (self.leaves.is_empty() || rebuild_due) {
            let items: Vec<(u32, Aabb)> = (0..n).map(|i| (i as u32, self.aabbs[i])).collect();
            self.leaves = self.tree.rebuild(&items);
            // 全量重建：所有缓存失效（代理盒全部重设）。
            self.cache_fat.clear();
            self.cache_fat.resize(n, Aabb::EMPTY);
        } else {
            for i in 0..n {
                if (i as u32) >= self.leaves.len() as u32 {
                    self.leaves.push(self.tree.insert(i as u32, self.aabbs[i]));
                    self.cache_fat.resize(n, Aabb::EMPTY);
                } else if bodies.is_dynamic(i) && bodies.awake[i] {
                    // M1：速度自适应边距——快速体在其 fat 盒内连续多帧零结构操作。
                    let m = self.fat_margin_for(bodies.linvel[i]);
                    let (nl, changed) =
                        self.tree
                            .move_proxy_scaled(self.leaves[i], self.aabbs[i], m);
                    if changed {
                        // 代理盒变化（就地生长或重插）→ 查询缓存失效（T2）。
                        self.leaves[i] = nl;
                        self.cache_fat[i] = Aabb::EMPTY;
                    }
                }
            }
        }
        let d_tree = t_tree.elapsed().as_micros() as u64;
        let t_query = std::time::Instant::now();
        // 2) 清醒动体查询（dyn-dyn 双侧发射由最终排序去重收敛；dyn-static 由
        //    动体侧发起）。睡眠体不查询：沉睡体不产生新接触；被唤醒/被撞由
        //    对方（清醒体）的查询反向命中（睡眠叶仍在树内），唤醒语义不变。
        //    查询缓存（T2）：仅对逃出缓存 fat 盒的体重走树（候选安全复用，
        //    见 `cache_fat` 注）；随后并行只读消费候选 + 精确过滤。
        let dyns: Vec<u32> = (0..n as u32)
            .filter(|&i| {
                let i = i as usize;
                bodies.is_dynamic(i) && bodies.awake[i]
            })
            .collect();
        self.cache_fat.resize(n, Aabb::EMPTY);
        self.cand_off.resize(n, 0);
        self.cand_len.resize(n, 0);
        // 刷新块划分与查询块一致（块序 = 体区间序 → 合并确定性）。
        let n_chunks = if threads > 1 && dyns.len() >= 4096 {
            dyns.len().div_ceil(dyns.len().div_ceil(threads))
        } else {
            1
        };
        let chunk_len = dyns.len().div_ceil(n_chunks);
        {
            // 分块并行重查（只读树；每块本地 arena + 条目表），随后串行合并。
            // 条目 = (体, 本地偏移, 长度, fat 盒)。
            type RefreshChunk = (Vec<u32>, Vec<(u32, u32, u32, Aabb)>);
            let this = &*self;
            let dyns_ref: &[u32] = &dyns;
            let bodies_ref: &BodySet = bodies;
            let mut refresh: Vec<RefreshChunk> =
                (0..n_chunks).map(|_| (Vec::new(), Vec::new())).collect();
            vxl_phys_core::schedule::for_each_chunk_mut(
                &mut refresh,
                threads,
                2,
                |start_slot, _len, slots| {
                    let mut tmp: Vec<u32> = Vec::new();
                    for (k, (arena, entries)) in slots.iter_mut().enumerate() {
                        let oi = start_slot + k;
                        let start = oi * chunk_len;
                        let end = ((oi + 1) * chunk_len).min(dyns_ref.len());
                        for &i in &dyns_ref[start..end] {
                            let iu = i as usize;
                            let exact = this.aabbs[iu];
                            if this.cache_fat[iu].contains(&exact) {
                                continue; // 未逃出缓存盒：候选复用。
                            }
                            let m = this.fat_margin_for(bodies_ref.linvel[iu]);
                            let fat = exact.grown(m);
                            this.tree.query(&fat, &mut tmp);
                            let base = arena.len() as u32;
                            arena.extend_from_slice(&tmp);
                            entries.push((i, base, tmp.len() as u32, fat));
                        }
                    }
                },
            );
            let mut cand_total = 0usize;
            for (arena, entries) in refresh {
                let arena_base = self.cand_arena.len() as u32;
                self.cand_arena.extend_from_slice(&arena);
                for (i, off, len, fat) in entries {
                    let iu = i as usize;
                    self.cand_off[iu] = arena_base + off;
                    self.cand_len[iu] = len;
                    self.cache_fat[iu] = fat;
                    cand_total += len as usize;
                }
            }
            self.last_cand_total = cand_total;
        }
        {
            let this = &*self;
            let dyns_ref: &[u32] = &dyns;
            let n_chunks = if threads > 1 && dyns.len() >= 4096 {
                dyns.len().div_ceil(dyns.len().div_ceil(threads))
            } else {
                1
            };
            let mut outs: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n_chunks];
            let chunk_len = dyns.len().div_ceil(n_chunks);
            vxl_phys_core::schedule::for_each_chunk_mut(
                &mut outs,
                threads,
                2,
                |start_slot, _len, slots| {
                    // slots[k] = outs[start_slot + k]（块内逐槽对应各自的体区间）。
                    for (k, co) in slots.iter_mut().enumerate() {
                        let oi = start_slot + k;
                        let start = oi * chunk_len;
                        let end = ((oi + 1) * chunk_len).min(dyns_ref.len());
                        for &i in &dyns_ref[start..end] {
                            let iu = i as usize;
                            let off = this.cand_off[iu] as usize;
                            let len = this.cand_len[iu] as usize;
                            for &j in &this.cand_arena[off..off + len] {
                                let ju = j as usize;
                                if ju == iu {
                                    continue;
                                }
                                if this.aabbs[iu].overlaps(&this.aabbs[ju]) {
                                    let (a, b) = if iu < ju { (i, j) } else { (j, i) };
                                    co.push((a, b));
                                }
                            }
                        }
                    }
                },
            );
            for mut co in outs {
                self.pairs.append(&mut co);
            }
        }
        // arena 压实（少见：仅当垃圾占比高时；重建各体偏移）。
        if self.cand_arena.len() > 4_000_000 {
            let mut fresh: Vec<u32> = Vec::with_capacity(self.cand_arena.len());
            for &i in &dyns {
                let iu = i as usize;
                let off = self.cand_off[iu] as usize;
                let len = self.cand_len[iu] as usize;
                if len == 0 {
                    continue;
                }
                let noff = fresh.len() as u32;
                fresh.extend_from_slice(&self.cand_arena[off..off + len]);
                self.cand_off[iu] = noff;
            }
            self.cand_arena = fresh;
        }
        let d_query = t_query.elapsed().as_micros() as u64;
        let t_sort = std::time::Instant::now();
        self.pairs.sort_unstable();
        self.pairs.dedup();
        self.last_breakdown_us = (d_aabb, d_tree, d_query, t_sort.elapsed().as_micros() as u64);
        &self.pairs
    }

    fn query_aabb(&mut self, aabb: &Aabb, out: &mut Vec<u32>) {
        self.tree.query(aabb, out);
    }

    fn set_step(&mut self, dt: f32) {
        if dt > 0.0 && dt.is_finite() {
            self.dt = dt;
        }
    }

    fn breakdown_us(&self) -> (u64, u64, u64, u64) {
        self.last_breakdown_us
    }

    fn tree_height(&self) -> u32 {
        self.tree.root_height()
    }

    fn cand_total(&self) -> usize {
        self.last_cand_total
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::SerialJobSystem;

    fn world() -> BodySet {
        BodySet::new()
    }

    /// BVH 与网格宽相在同场景下输出完全一致（确定性交叉验证）。
    #[test]
    fn bvh_matches_grid_across_frames() {
        let build = || {
            let mut b = world();
            // 地面静态瓦片 5×5。
            for gx in -2i32..=2 {
                for gz in -2i32..=2 {
                    b.push_static(
                        Shape::Box {
                            half: Vec3::new(0.5, 0.25, 0.5),
                        },
                        Vec3::new(gx as f32, -0.25, gz as f32),
                        Quat::IDENTITY,
                    );
                }
            }
            // 动体：确定性伪随机散布。
            for k in 0..40u32 {
                let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let z = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let y = 0.5 + ((k.wrapping_mul(97)) % 400) as f32 / 100.0;
                let shape = if k % 2 == 0 {
                    Shape::Sphere { radius: 0.3 }
                } else {
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    }
                };
                b.push_dynamic(shape, Vec3::new(x, y, z), Quat::IDENTITY, 1.0);
            }
            b
        };
        let mut grid = GridBroadPhase::new(2.0, 0.01);
        let mut bvh = BvhBroadPhase::new(0.01);
        for frame in 0..5u32 {
            let mut b = build();
            // 每帧给动体一个确定性位移（模拟增量移动）。
            for i in 0..b.len() {
                if b.is_dynamic(i) {
                    b.position[i].x += frame as f32 * 0.17;
                    b.position[i].y -= frame as f32 * 0.09;
                }
            }
            let p_grid = grid.compute_pairs(&b, &[], &SerialJobSystem).to_vec();
            let p_bvh = bvh.compute_pairs(&b, &[], &SerialJobSystem).to_vec();
            assert_eq!(p_grid, p_bvh, "frame {frame}");
            let _ = &mut b;
        }
    }

    /// 既有配对漏洞回归（T2 第十四段修复）：睡眠体不做查询，旧「dyn-dyn 只由
    /// 较大索引侧发射」规则会漏掉「清醒大索引体 vs 睡眠小索引体」——双侧发射
    /// 后必须检出（否则清醒体可穿过沉睡体）。
    #[test]
    fn awake_larger_pairs_with_sleeping_smaller() {
        let mut b = world();
        let small = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
        );
        let big = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        // 小索引体入睡（大索引体保持清醒）。
        b.awake[small as usize] = false;
        let mut bp = BvhBroadPhase::new(0.01);
        let pairs = bp.compute_pairs(&b, &[], &SerialJobSystem);
        assert_eq!(pairs, &[(small.min(big), small.max(big))]);
    }

    /// T2 查询缓存**完备性**随机化不变式：多帧移动（含高速逃逸/生长/重插、
    /// 混合睡眠）下，BVH 输出对必须与暴力枚举（同源 stored_aabb 精确重叠）
    /// 逐一相等。缓存复用若漏对（错失新接近体）此测试立即红。
    #[test]
    fn bvh_pairs_match_brute_force_across_frames() {
        let mut b = world();
        for gx in -3i32..=3 {
            for gz in -3i32..=3 {
                b.push_static(
                    Shape::Box {
                        half: Vec3::new(0.5, 0.25, 0.5),
                    },
                    Vec3::new(gx as f32, -0.25, gz as f32),
                    Quat::IDENTITY,
                );
            }
        }
        for k in 0..160u32 {
            let x = ((k.wrapping_mul(2_654_435_761)) % 1000) as f32 / 1000.0 * 6.0 - 3.0;
            let z = ((k.wrapping_mul(40_503)) % 1000) as f32 / 1000.0 * 6.0 - 3.0;
            let y = 0.6 + ((k.wrapping_mul(97)) % 300) as f32 / 100.0;
            b.push_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.3),
                },
                Vec3::new(x, y, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        let mut bp = BvhBroadPhase::new(0.01);
        for frame in 0..60u32 {
            // 确定性移动：奇数体快（触发逃逸/生长/重插），偶数体慢（缓存命中）；
            // 每 3 帧让 1/5 体入睡（清醒-睡眠 混合路径）。
            for i in 0..b.len() {
                if !b.is_dynamic(i) {
                    continue;
                }
                let f = frame as f32;
                let s = if i % 2 == 0 { 0.01 } else { 0.12 };
                b.position[i].x += ((i as f32 * 0.37 + f * 0.11).sin()) * s;
                b.position[i].z += ((i as f32 * 0.53 + f * 0.17).cos()) * s;
                b.position[i].y -= if i % 2 == 0 { 0.002 } else { 0.02 };
                b.awake[i] = !(frame % 3 == 0 && i % 5 == 0);
            }
            let got = bp.compute_pairs(&b, &[], &SerialJobSystem).to_vec();
            // 暴力参照：i<j 精确 AABB 重叠（与宽相同规则——**至少一侧为
            // 「动态且清醒」**：沉睡体不查询、静-静不产对 ⇒ 静×睡与睡×睡
            // 均无对；清醒×睡/清醒×静由清醒侧查询命中）。
            let n = b.len();
            let mut want: Vec<(u32, u32)> = Vec::new();
            let sa: Vec<Aabb> = (0..n).map(|i| bp.stored_aabb(i).unwrap()).collect();
            for i in 0..n as u32 {
                for j in (i + 1)..n as u32 {
                    let (iu, ju) = (i as usize, j as usize);
                    let pi = b.is_dynamic(iu) && b.awake[iu];
                    let pj = b.is_dynamic(ju) && b.awake[ju];
                    if !pi && !pj {
                        continue;
                    }
                    if sa[iu].overlaps(&sa[ju]) {
                        want.push((i, j));
                    }
                }
            }
            want.sort_unstable();
            want.dedup();
            assert_eq!(got, want, "frame {frame}");
        }
    }

    #[test]
    fn overlapping_pair_found_once() {
        let mut b = world();
        let g = b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let d = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let far = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(50.0, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let pairs = bp.compute_pairs(&b, &[], &SerialJobSystem);
        assert_eq!(pairs, &[(d.min(g), d.max(g))]);
        assert!(pairs.iter().all(|&p| p.1 != far && p.0 != far));
    }

    #[test]
    fn no_static_static_pairs() {
        let mut b = world();
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.1, 0.0, 0.0),
            Quat::IDENTITY,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        assert!(bp.compute_pairs(&b, &[], &SerialJobSystem).is_empty());
    }

    #[test]
    fn deterministic_output() {
        let mut b = world();
        for k in 0..50 {
            let x = (k % 10) as f32 * 1.1;
            let z = (k / 10) as f32 * 1.1;
            b.push_dynamic(
                Shape::Sphere { radius: 0.5 },
                Vec3::new(x, 1.0, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let p1 = bp.compute_pairs(&b, &[], &SerialJobSystem).to_vec();
        let p2 = bp.compute_pairs(&b, &[], &SerialJobSystem).to_vec();
        assert_eq!(p1, p2);
    }
}
