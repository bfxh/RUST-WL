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
| `vxl-phys-terrain` | 体素/地形 | `TerrainSet`（高度场账本）+ **`voxel::VoxelVolume`**（占据位图 + 局域 SDF + `CollisionProvider`）+ 贪心提取/球域切割 | core, narrow, broad | ✅（M3 第一块） | 🌤 每 tick（体素查询在外层调用时进 🔥） |
| `vxl-phys-destruction` | 断裂/碎块形状 | 骨架（Voronoi 预断裂待做） | — | 🦴 | ❄️ |
| `vxl-phys-soft` | 软体/布（XPBD） | 参数骨架（compliance 档位） | — | 🦴 | 🔥（实现后） |
| `vxl-phys-fluid` | 液体（SPH/PBF/FLIP） | 参数骨架 | — | 🦴 | 🔥（实现后，GPU 为主） |
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
| `vxl-phys` | `World` 组装（默认管线）+ **破坏/体素 API**（`add_voxel`/`carve_sphere`/`spawn_box_debris`/`apply_impact_destruction`）+ CCD | `World::{step, add_*, providers, health, state_hash}` | ✅ | 

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
