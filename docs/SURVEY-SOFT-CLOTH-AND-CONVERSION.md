# 调研：XPBD/布料 · 体素↔多边形转换的物理 · 外部仓库对标（2026-09-26）

> 触发：用户三问 —— ①XPBD 做了吗、布料是不是有问题；②"体素转多边形途中的物理"（要可开关、要能被大量东西挤爆、
> 要不要附上前几何形态的效果）；③看 `github.com/shyhdm/Opengl_Physx` 能借鉴什么。
> 形态：**调研 + 精选待办**（不新增功能、不动代码）。所有结论带 `文件:行号` 或"实测/源码读到"标注。

## 1. 现状：三条问题的答案（先说结论）

### 1.1 XPBD —— **没做。零实现，只有骨架 + 死声明**

| 项 | 实情 | 证据 |
|---|---|---|
| 软体/布料 crate | `vxl-phys-soft` **111 行纯参数骨架**（`Stiffness` α 档、`TearStrain`、`ClothConstraints`、`SelfCollision`），**无任何粒子/约束/积分代码** | `crates/vxl-phys-soft/src/lib.rs:1-111` |
| 它接线了吗 | **没有**：`vxl-phys` 的 `Cargo.toml` 不依赖它，全仓无任何 crate 依赖它 | `crates/vxl-phys/Cargo.toml` |
| 统一约束元素 | `ElementKind::Xpbd`、`ConstraintElement` trait **声明了但零实现**（而 trait 文档自述"M1 内不接线"） | `crates/vxl-phys-core/src/interop.rs:98-99`、`:277` |
| 求解器本体 | `vxl-phys-solver` 是**顺序冲量（TGS-Soft 风格，逐行对齐 Rapier 0.35）**，软接触用 **ERP/CFM 正则化**，**不是** PBD/XPBD 的 α̃ 柔度；`PhysConfig` 里也没有 compliance | `crates/vxl-phys-solver/src/lib.rs:1-14`、`types.rs:61-88`；`config.rs`（无 α 字段） |
| 路线位置 | **M4「软体/布：XPBD + 与刚体共求解器」**，出口判据已定（"悬臂/旗飘金样 + 刚度档表"）；当前状态"参数骨架" | `docs/ROUTE.md:131`、`:53-54`；`docs/SPEC.md:137-147`（§4.6/§4.7） |

⇒ **"布料处理有问题"要正确定性**：布料**根本没有实现**，所以不存在"处理得对不对"；但**架构上也没为它准备好**，
而**最硬的缺口不在 XPBD 本身**，在**"三角形作为一等几何"**这条：

| 布料必需的能力 | 本仓现状 |
|---|---|
| 三角网作为**碰撞形状** | **没有**：`Shape` 只有 Box/Sphere/Cylinder/Capsule/Cone/HeightField/Provider/ConvexHull/Compound | `crates/vxl-phys-core/src/shape.rs:16-64` |
| 三角网 vs 形状的接触 | 只有 **box/sphere/hull** 三种对 provider 开放（`_ => false`）；**cylinder/cone/capsule vs 三角网目前不产生接触** | `crates/vxl-phys-narrow/src/pair_shaped.rs:117,157-177` |
| 三角网**自碰撞** / 布-布 | 没有 | 同上（provider-provider 直接返回不支持，`:141-143`） |
| 距离/弯曲/体积约束 | 只有刚体关节（`JointKind` 5 种），**无** XPBD 意义的结构/剪切/弯曲约束 | `crates/vxl-phys-solver/src/joints/types.rs:5-16` |
| 面元气动力（§4.7 的布料受力） | `vxl-phys-aero` = **28 行纯配置**（4 个 f32），无任何几何输入、无消费方 | `crates/vxl-phys-aero/src/lib.rs:1-28` |

### 1.2 体素↔多边形转换的物理 —— **四个子需求里，三个半是空白**

| 用户要的 | 本仓现状 |
|---|---|
| 体素 → **多边形** 的转换 | **零**：`marching/isosurface/dual_contour/remesh/voxeliz/polygoniz` 全仓零命中。现有的跨表示只有两条：**体素 → 轴对齐盒碎块**（新建刚体，不是网格：`extract_boxes`/`spawn_box_debris`）与**网格 → 静态 provider**（`TriMesh`，无体素化） | `crates/vxl-phys-terrain/src/voxel/voxel_volume.rs:204,232`；`crates/vxl-phys/src/world_body.rs:24,55,74,107`；`crates/vxl-phys-terrain/src/mesh.rs:26,322` |
| **"转换途中的物理"** | **零**：`morph/transition/blend/prev_shape/legacy/materialize/handover` 全仓零命中。既有的"交接"都是**域间让位（二选一、不叠加）**：2a/2b 流体让位、GPU 档跑不通整趟回退 CPU | `crates/vxl-phys/src/world_struct.rs:36`；`world_step.rs:198`；`world_step/narrow_tier.rs:92,181` |
| **"要不要附上前几何形态的效果"** | **零**。`StateBridge`（表示转换接口）**声明了但无实现**，只在注释里被提到 | `crates/vxl-phys-core/src/interop.rs:257`；`crates/vxl-phys-splat/src/lib.rs:6` |
| **"能被大量东西挤爆"** | **没有全局容量/超载/降级机制**。只有局部上限（CCD 段数、限速、流体格预算粗化）与**唯一一个"丢约束"机制**（参与式降点：第 3 轮起跳过"至今零冲量的浅缝点"，深穿透绝不跳）。"压力累积/拥挤度量"只有唤醒门 `wake_gate_k`（语义是唤醒规则，不是容量保护）。`FragmentBudget{B1K..B1M}` 是枚举、**无消费者** | `crates/vxl-phys-solver/src/island.rs:26-54,80-83`；`crates/vxl-phys-core/src/config.rs:166`；`crates/vxl-phys-destruction/src/lib.rs:16` |

⇒ **顺带**：路线图里已经有一张"作用体×受体"矩阵，把**体素×软体=支撑/穿刺**、**软体×液体=湿布（双向）**、
**软体×风=面元气动**都写下了（`docs/ROUTE.md:80-84`）——**"转换途中的物理"不在那张矩阵里**，属新增维度。

### 1.3 外部仓库 `shyhdm/Opengl_Physx` —— **对 XPBD/布料/SPH 零可借鉴**（这条要直说）

实测（Contents API 逐文件读，`src/` 72 个文件）：它是 **PhysX 5 + NvBlast + NvFlow 的 OpenGL 胶水 demo**
（2026-09 刚起步，Windows/MSVC 独占，**无 LICENSE**）。

- **没有自研求解器**；**没有布料**（文件名/grep 双零命中；`xPBD` 只是 `PxPBDParticleSystem` 的子串假象）；
  **没有 SPH**；**没有 marching cubes / 体素→网格**（体素只用于烟，且**直接 raymarch、永不转多边形**）。
- ⇒ **"看它能借鉴什么"在求解器维度上是空集**。可借鉴的是 4 条**工程技巧**（详见 §3），与 XPBD 无关。

## 2. 精选待办（按"判据现成度 × 收益/代价"排；不动代码，供拍板）

### T1（M4 前置）把"三角形作为一等几何"补齐 —— **布料的最硬缺口，且不依赖 XPBD**
- 缺的是：`Shape` 里的三角网、`pair_shaped` 的 trimesh×{box,sphere,hull,cylinder,capsule,cone}、
  **自碰撞**。判据可复用既有 provider 那套（`contacts_point`/`contacts_box` + `trimesh_*` 探针的静置/穿网判据）。
- **最小第一片建议**：一个**绳索（1D 链）**——它只需要"距离约束 + 点-形状接触"，**不碰三角网自碰撞**，
  却能把 XPBD 的核心循环（子步 × 约束投影 × 拉格朗日乘子/柔度）打通并立起判据（能量守恒 + 悬垂形状）。
- 出口沿用 ROUTE §131：**悬臂/旗飘金样 + 刚度档表**；新增金样前先按本仓先例**默认关**（`PLAN-solver-limits.md:82-84`）。

### T2（新维度）体素↔多边形"转换途中的物理" —— 拆成 **4 个可独立开关**的子特性
正好对上本仓既有的开关形态（`PhysConfig` 0=关字段 / `Option<Box<dyn …>>` 槽 + `set_*` 注册 + 整趟回退 /
新文件 `world_step/<feature>.rs`；见 §1.4）。**建议顺序**：

| # | 子特性 | 第一片做什么 | 判据（可量化） |
|---|---|---|---|
| 2.1 | **转换本身**（体素 SDF → 网格） | CPU 侧对 `VoxelVolume` 做一次**表面提取**（surface nets 或 marching cubes），离线工具先跑 | 提取网格与 SDF 的**零等值面一致性**、面积/体积守恒、与 `voxel_rest_probe` 的静置高度一致 |
| 2.2 | **转换途中的物理** | `world_step/conversion.rs` 新文件 + 开关：转换中的体**仍是物理体**（体素体继续当 provider），网格体作动态体，**两者由同一位姿驱动**（"双重表示"窗口） | 窗口期内**接触集合连续**（体素侧的接触与网格侧一致，或按显式规则交接）；结束时无穿透突变 |
| 2.3 | **要不要附上前几何形态的效果** | **显式交接策略三档**：(i) 不继承（全新体）；(ii) **继承速度/角速度**（动量连续）；(iii) 继承约束/warm 残量 | **动量/能量在转换瞬间的跳变有界**（阈值写死并测）；(iii) 还要 warm 匹配率 |
| 2.4 | **能被大量东西挤爆** | 给转换通道**显式预算**（碎片数/顶点数/并发转换数）+ **超载时的确定性降级**（退化凸包/盒代理、或合并、或排队——**三档选一并写判据**） | 超预算时**行为确定**（同输入同结果）+ 不低于"代理档"的碰撞保真度；"挤爆"作为**物理可见现象**需要先有**拥挤/压力度量**（本仓现在只有 `wake_gate_k`，不足） |

⚠️ 2.4 的"挤爆"要先定义语义：**是"物理被压塌/碎裂"还是"求解器超载降级"**——这两件事判据完全不同，
建议**分开命名**（前者属 M3 破坏域，后者属新容量机制）。

### T3 借外部仓库的 4 条（三档判定，按用户既有的"吸收协议"）

| 档 | 条目 | 为什么 / 落到哪 |
|---|---|---|
| **吸收** | ① **冲量分级 Voronoi site 数**（`coreSites = clamp(10+level*3,12,36)` 等）+ 按冲量重碎裂 | 本仓已有 `fracture_voronoi`/`FragmentBudget`（无消费者）⇒ 这是把预算接上消费者的现成经验值；与 PhysX 无关 |
| **吸收** | ② **刚体几何 → 半空间平面集**（球/胶囊离散成 112 平面 + 端盖；凸包取多边形平面）当接触提供者 | 本仓 provider 通道已开（`contacts_point`）⇒ "点 vs 平面集"比采样体素 SDF 便宜，且盒/凸包/胶囊覆盖 90% 场景 |
| **吸收** | ③ **预判式碎裂调度**（`impactTime = d/v` + 弹道项 ⇒ 提前 N 帧启动准备） | 把"生成碎裂网格/cook"的毫秒级尖峰挪出撞击帧；纯逻辑零依赖（注意它的 `4.905f` 硬编码重力、需参数化） |
| **有界** | ④ **稀疏体素 raymarch 而非多边形化**；零拷贝 GPU 缓冲（`cuGraphics*` 模式 ↔ wgpu external buffer） | 对**烟/雾/体积介质**可当渲染档（本仓体素已上卡）；但**不要**为它引入第二套图形 API |
| **不吸收** | 整仓代码；三套运行时（GL+Vulkan+CUDA）拼接；Windows/MSVC 独占；屏幕空间流体渲染当最终方案 | **无 LICENSE ⇒ 法律红线**（概念可学、源码不可抄）；后三条对一个已有统一 GPU 后端的 Rust 引擎是纯负债 |

## 3. 与既有路线的关系（别重复造）

- **M4 已定义**（软体/布 + XPBD + 共求解器 + 出口判据）⇒ T1 就是 M4 的前置项，**不是新计划**；
  本文只补一条：**先做"三角形一等几何"与"绳索最小闭环"，别先写 XPBD 求解器**（否则会先攒一个用不上的约束求解核心）。
- **转换物理（T2）是新维度**：ROUTE 的作用矩阵（§80-84）没有它；`StateBridge`（表示转换）是它的既有接口位置（零实现）。
- **开关形态照抄既有先例**（`config.rs:141` 的"0=关、关闭时逐位不变"、`narrow_tier.rs:92` 的"整趟回退"、
  新文件 `world_step/<feature>.rs` 避开尺寸门）⇒ T2 的四个子特性都能做到"默认关、既有判据不受影响"。
