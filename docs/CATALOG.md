# 类目（CATALOG）—— crate / 模块 / 库 分类编目

> 一句话：**按「层 → 域 → 工具」分类，按「状态」和「热路径」标注**。
> **本文件是文档级分类**：它不改任何代码结构、不引入运行时开销（无动态分发、
> 无替身层、无包装类型）——「分类」只发生在文档与 crate 边界上。
> 相关：`ROUTE.md`（三轴标准）、`SPEC.md` §1（DAG）、`INDEX.md`（文档地图）。

## 0. 库清单（依赖事实）

- 工作区共 **19 个 crate**，**零外部依赖**（`[dependencies]` 里只有 `vxl-phys-*` path 依赖）。
- 唯一例外：`gold-sample/`（**独立包，不进 workspace**）依赖 `rapier3d`——仅用于
  **金样对拍**（T5），不参与引擎构建。
- 依赖方向（DAG，禁止环；`Cargo.toml` 头注与 SPEC §1 一致）：
  `core ← {broad, narrow, integrate, solver, field, replay}`；`narrow ← terrain ← 门面`。
- 静态审计：`deny.toml`（cargo-deny）；CI 含三编译器矩阵 + 词汇禁令扫描。

## 1. 分类维度

| 维度 | 取值 | 说明 |
|---|---|---|
| **层** | 柱石 / 管线 / 互操作 / 域 / 桥接 / 门面 | 决定依赖方向：**只能向下依赖**（域 → 接口/柱石，不得域间互依） |
| **状态** | ✅ 落地 / 🔄 在研 / 🦴 骨架（参数级）/ ⬜ 未建 | 与 `ROUTE.md` 里程碑表一致 |
| **热路径** | 🔥 每 tick × 每体/每接触 / 🌤 每 tick 一次 / ❄️ 事件级 | 决定允许的写法（见 §4） |

## 2. 分层编目（19 crate）

### 柱石层（base layer）

| crate | 职责 | 关键类型/接口 | 依赖 | 状态 | 热路径 | 规模 |
|---|---|---|---|---|---|---|
| `vxl-phys-core` | 数学(严格 f32)/配置/材质/SoA 体数据/调度抽象/**互操作四接口** | `Vec3/Quat/Mat3/Aabb`、`BodySet`、`PhysConfig`、`interop::{CollisionProvider, ProviderColliders, MediumField, StateBridge, ConstraintElement}` | —（零依赖） | ✅ | 🔥（数学/体数据被每帧读） | 2.2k / 24 测 |

### 管线层（rigid pipeline，每 tick 依次跑）

| crate | 职责 | 关键类型/接口 | 依赖 | 状态 | 热路径 | 规模 |
|---|---|---|---|---|---|---|
| `vxl-phys-broad` | 宽相：二叉 AoS 增量 BVH（生产）+ 网格哈希（对照）+ 两个**未接线**宽树研究模块 | `BroadPhase::{compute_pairs, query_aabb, cand_total}`、`DynamicBvh` | core | ✅ | 🔥 每 tick 全量 | 3.9k / 30 测 |
| `vxl-phys-narrow` | 窄相：盒 SAT + 裁剪流形 / 球解析 / 高度场特化 / **provider 通道** | `NarrowPhase::collide`、`Manifold`、`CollisionProvider` 实现（高度场） | core | ✅ | 🔥 每 tick 每对 | 2.5k / 22 测 |
| `vxl-phys-solver` | 求解：顺序冲量 + 软接触 + 摩擦锥 + 并查集分岛 + 岛级休眠 + warm 槽位表 | `ImpulseSolver::solve`、`SolverParams` | core, narrow | ✅ | 🔥 每 tick × 迭代 | 1.3k / 1 测 |
| `vxl-phys-integrate` | 半隐式欧拉 + 固定步/子步 | `Integrator::{integrate_velocities, integrate_positions}` | core | ✅ | 🔥 每 tick | 112 / 2 测 |
| `vxl-phys-field` | 力场注册表（重力/风/爆炸/吸引/排斥/涡流） | `ForceField` trait、`FieldRegistry` | core | ✅ | 🌤 每 tick 一次 | 94 / 0 测 |

### 互操作层（兼容轴的唯一通道）

| 位置 | 职责 | 关键接口 | 状态 |
|---|---|---|---|
| `core::interop` | 跨域四接口 + 规范类型（`InteropContact`/`SurfaceHit`/`MediumSample`/`BridgeKind`/`ElementKind`） | — | ✅（ADR 0008/0009） |
| `broad::shape_aabb` + `BroadPhase` | 外部提供者体的宽相 AABB（`provider_bounds` 供给） | — | ✅ |
| `narrow` provider 分支 | 盒/球 × provider 的接触生成（接触带按速度自适应） | `ProviderColliders` | ✅ |

### 域层（domain plugins；一域一 crate，只向下依赖）

| crate | 域 | 现有关键件 | 依赖 | 状态 | 热路径 |
|---|---|---|---|---|---|
| `vxl-phys-terrain` | 体素/地形 | `TerrainSet`（高度场账本）+ **`voxel::VoxelVolume`**（占据位图 + 局域 SDF + `CollisionProvider`）+ 贪心提取/球域切割 + **`mesh::TriMesh`**（任意三角网薄壳提供者 + 均匀网格加速）+ **`contacts_point_voxel_solid`**（流体边界终版口径：占据门 + 开放面推进） | core, narrow, broad | ✅（M3 第一块） | 🌤 每 tick（体素/网格查询在外层调用时进 🔥） |
| `vxl-phys-destruction` | 断裂/碎块形状 | 骨架（Voronoi 预断裂待做） | — | 🦴 | ❄️ |
| `vxl-phys-soft` | 软体/布（XPBD） | 参数骨架（compliance 档位） | — | 🦴 | 🔥（实现后） |
| `vxl-phys-fluid` | 液体（SPH/PBF/FLIP） | **WCSPH 求解器**：poly6 密度（含自身项）/ spiky 对称压力梯度 / Tait γ=7 / Monaghan 人工黏度 + XSPH / 镜像鬼影边界密度；确定性均匀网格 27 邻域；SoA + 半隐式欧拉 4 子步（`FluidSystem`）；**`MediumField` 实现**（2a 采样侧，2026-09-22）：`sample`＝27 邻域 poly6 插值出 密度/流速/占用率（**只读、不改流场**）；`deposit` 显式留空待 2b（Akinci） | core | ✅（0.3 切片 1，CPU 档） | 🔥 每 tick（4 子步 × 27 邻域） |
| `vxl-phys-wheeled` | 车辆（射线悬挂/轮胎） | 参数骨架 | — | 🦴 | 🌤 |
| `vxl-phys-aero` | 风/气动（面元） | 参数骨架 | — | 🦴 | 🌤 |
| `vxl-phys-marine` | 海洋/浮力 | 参数骨架 | — | 🦴 | 🌤 |
| `vxl-phys-mech` | 机械/关节族 | 参数骨架 | — | 🦴 | 🔥（关节求解） |
| `vxl-phys-gpu` | GPU 后端 trait | 骨架（wgpu/rust-gpu 后端 trait） | — | 🦴 | — |

### 桥接层（对外/对工具）

| crate | 职责 | 关键件 | 状态 |
|---|---|---|---|
| `vxl-phys-replay` | 状态哈希（xxh3-128 规范化）+ 录制/回放 | `state_hash`、`Recorder` | ✅ |
| `vxl-phys-ffi` | C ABI 门面（版本化投递） | 骨架 + 2 测 | 🦴/✅ 混合 |
| `vxl-phys-memfind` | 内存子串工具（SPEC §10，独立于引擎） | 2 测 | ✅ |

### 门面层

| crate | 职责 | 关键件 | 状态 |
|---|---|---|---|
| `vxl-phys` | `World` 组装（默认管线）+ **破坏/体素 API**（`add_voxel`/`carve_sphere`/`spawn_box_debris`/`apply_impact_destruction`）+ **流体 API**（`add_fluid`/`fluids`/`fluid_pass`）+ CCD | `World::{step, add_*, providers, health, state_hash}` | ✅ | 

## 3. 热 crate 的内部模块分类

| crate | 模块 | 分类 | 热路径要点 |
|---|---|---|---|
| `broad` | `bvh.rs` | 生产树 | `move_proxy_scaled`（就地生长 + 提前退出 refit `fix_upwards_grow`，见 M1-PLAN §9） |
| `broad` | `lib.rs` | 缓存/查询 | 查询缓存（fat 盒）＋ 候选过滤；边距 `fat_margin_for`（K=6，M1-PLAN §10） |
| `broad` | `wide.rs` / `bvh8.rs` | **研究资产**（未接线，ADR 0001） | 保留 B 树插入/全叶同深/三树形对拍 |
| `narrow` | `lib.rs` 分派 | 形状对路由 | 盒对 SAT 快路径 → 通用凸体路径 → 高度场特化 → **provider 分支** |
| `narrow` | `heightfield.rs` | 高度场原语 + provider 实现 | 双线性采样 + 解析法线 |
| `narrow` | `simd.rs` | SSE2 四轴 SAT（ADR 0002） | 逐位透明（标量孪生 + 位级测试守门） |
| `solver` | 分岛/分组 | 每 tick 一次 | 并查集 + 岛组并行（`solve_island_group`） |
| `solver` | 约束构建/gather/scatter | 每 tick | `build_constraint`（暖匹配 + 质量项 + 软目标） |
| `solver` | 迭代环 | 🔥 最热（解算 95%，M1-PLAN §12） | `solve_constraint`（内层扫掠 + 2×2 切向联立） |
| `solver` | warm 槽位表 | 每 tick | 稠密槽位 + 原位写 + 单遍剪枝（M1-PLAN §16-§18） |
| `solver` | 休眠/CCD | 每 tick / 事件级 | `sleep`（岛级原子）＋ `ccd.rs`（命中判据：沿法向接近） |
| `terrain` | `voxel.rs` | 域表示 | 占据位图 + 局域 SDF（自适应查询范围）+ 贪心提取/球域切割 |
| `terrain` | `mesh.rs` | 域表示（三角网） | 薄壳语义：`depth = skin − 最近三角形距离`、法线 = 面法线（无内外判定）；Ericson 最近点闭式解；均匀网格（格边 = 平均边长、下限 0.5，三角形 AABB 登记覆盖格，3×3×3 邻域查询）；`build_grid()` 建表（`GRID_MAX_BINS` 封顶），不建表回退全扫；精度前提 `skin < 格边/2` |

## 4. 性能纪律（分类不得影响性能——用户令）

1. **分类是文档级**：本次编目**零代码改动**（只加本文件与索引链接）。
2. **热路径禁止**（本仓已实测背书的教训，见 `EXPERIMENTS.md`）：
   - 动态分发（`dyn`）**只在跨域边界**（provider 通道，温/冷路径 ✓）；热路径一律静态泛型/枚举分派；
   - 热路径不分配（warm 通道的分配 churn 实测 ≈11.4ms/tick，M1-PLAN §18）；
   - 热路径不随机访问哈希桶（warm 表从 `HashMap` 换稠密槽位：−34.8ms/tick，§16）；
   - 每点每迭代的数据布局改动必须**先量再改**（点布局/体侧缓存/扁平化/预取/快速哈希
     五路全否，见 §15/§7-6/W8/S21）。
3. **新增域的形状**：只允许加「crate + 四接口实现」，不得改热路径代码（ADR 0008/0009）；
   若必须进热路径，先给**量化上界**（ADR 0002 的 ≥5% 收益律）。
4. **分类落地方式**（未来若要把「层」写进代码）：用 **模块边界/可见性**（`pub(crate)`
   与 crate 边界）而非包装类型——零运行时开销；跨 crate 的边界已经由 DAG 保证 ✓。

## 2026-09-14 新增

- **vxl-phys-splat**（域模块·新）：高斯喷溅隐式场 `GaussianSplatField`——
  密度 `σ(p)=Σw·exp(−½α(p))`（各向异性二次型）、距离 `(τ−σ)/|∇σ|`、法线 `−∇σ/|∇σ|`；
  实现 `ProviderColliders`（点/盒/球三查）。**物理参数 = 渲染参数**（无第二套表示），
  渲染桥 `export_splats` 零拷贝导出。热路径 O(#核) 且 3σ 截断；均匀网格/BVH 加速待办。
- **vxl-phys-narrow/gjk.rs**（窄相·热路径）：`ConvexHull` 支撑 + GJK（单纯形 Voronoi
  分类，距离收敛判据）+ EPA（地平线重建；退化 → 6 轴 SAT）+ `clip_halfspace` +
  `fracture_voronoi_hull`。**当前每对重建支撑体（O(n) 拷贝）**——外壳数量上量后
  应加按 (hull, pose) 的支撑缓存。
- **vxl-phys-terrain**：新增 `contacts_point_voxel`（点查询：`depth = skin − sdf`）与
  `VoxelVolume::dims/origin/step` 访问器。
- **interop**：`ProviderColliders::contacts_point`（默认 false = 不支持）。

## 2026-09-14 追加（二）

- **vxl-phys-splat（介质层）**：实现 `MediumField`（`sample` 密度/流速/黏性/占用率；
  `deposit` = 单向耦合占位）。`medium_density = 0` ⇒ 短路（不作介质的场零成本）。
- **vxl-phys 门面**：`medium_pass`（力场 → 介质阻力 → 速度积分），二次阻力
  `F = −½ρCdA|v_rel|v_rel`；`cross_section_area` 形状迎风面积估计。

## 2026-09-15 新增

- **vxl-phys-fluid（域模块·从骨架落地）**：CPU WCSPH 求解器（PLAN-0.3）——
  密度 poly6（含自身项）、压力 spiky 对称梯度（中心不减益 ⇒ 抑制张力不稳定）、
  Tait γ=7（声速定刚度）、Monaghan 人工黏度 + XSPH、镜像鬼影边界密度；邻居 =
  均匀网格（格边 = h，27 邻域，格内按索引序 ⇒ 求和序是位置的确定函数）；半隐式
  欧拉 4 子步、单子步行程钳制。`FluidSystem` SoA（pos/vel/dens/press + 网格）。
  测试 7/7（PLAN-0.3 §3；#2 静水压按实测口径诚实重写）。
- **vxl-phys-terrain**：`contacts_point_voxel_solid`——流体边界**终版口径**：
  占据位图判内外（不用 sdf 符号，截断 SDF 近壳内侧会误报）+ 内点恒推最近
  **开放面** + 壳面/截断 SDF 双回退；旧 `contacts_point_voxel` 保留（外部
  provider 兼容委托）。试错史（零厚度 typo / SDF 门误报棘轮）见 PLAN-0.3 §4.1。
- **interop**：`ProviderColliders::contacts_point_boundary` 默认 trait 方法
  （流体边界专用通道；默认逐位委托 `contacts_point`）。
- **vxl-phys 门面**：`add_fluid(sys, boundaries)` / `fluids()` / `fluid_pass`
  （体解算后按子步推进；切片 1 **单向耦合**——流体受几何约束，不反作用刚体）。
- **转储格式 VXLD v3**：头部追加流体系统数；帧尾追加流体粒子节（每系统：
  粒子数 + 位置 3f32）。`examples/showcase` 升六域同场（石盆铸装水块 320 粒）；
  新 `examples/dam_break`（塌坝 504 粒）；`render_demo.py` 支持 v1–v3 解析 +
  蓝色软团渲染 + `--src/--dst/--dist/--ty/--label`。
