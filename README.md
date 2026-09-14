# RUST WL —— vxl_phys 自研 Rust 物理引擎

> 依据：[`docs/SPEC.md`](docs/SPEC.md)（引擎权威规格，V1.3）+ 上游总架构
> `VoxelForge-V3/docs/PHYS-ENGINE-VXLPHYS-V2.md`（v2 权威，Rust 路线；施工分解见
> `VoxelForge-V3/docs/SPEC-VXLPHYS-M0-V1.md`）；M0 实测记录见
> [`docs/M0-GATES.md`](docs/M0-GATES.md)。
> 定位：**独立商业级 Rust 物理引擎**，纯 Rust、确定性优先、模块化 crate DAG，
> 与任何消费工程零关联；仓库目录名「RUST WL」= **Rust 物理（WuLi）**。

## 状态：M1 刚体核心在研（路线顶层口径见 [`docs/ROUTE.md`](docs/ROUTE.md)）

| 里程碑 | 内容 | 状态 |
|---|---|---|
| **M0 MVE** | 盒/圆柱/高度场 + 顺序冲量 + 岛 + 确定性哈希；1 万静态+1 千动态 ≥60Hz | ✅ 竖切可跑 |
| **M0 骨架续建（V2 §11）** | 相位 arena + hot/cold 32B（`core/mem.rs`）、xxh3-128 规范化哈希（10 轮断言）、m0_gates 门槛（p50 1.09ms）、CI 四门 + 三编译器矩阵 + 词汇禁令扫描 | ✅ 交付（记录：`docs/M0-GATES.md`） |
| **M1 刚体核心（在研）** | 稳定/金样/性能收口；性能档缺口与架构诊断见 `OPEN-PROBLEMS.md` P3、`DESIGN-staged-solver.md` | 🔄 在研 |
| **M2 互操作核心** | 四接口（Provider/Medium/Bridge/Element）+ 迁移窄相/地形为 provider + 贯通示例；并入原「键图 + 双 ABI + trait 插件面」 | ⬜ |
| **M3 破坏/地形（体素）** | Voronoi 预断裂 + 运行时切割 + 体素块→刚体/粒子 | ⬜ |
| **M4 软体/布** | XPBD + 与刚体共求解器 | ⬜ |
| **M5 液体** | WCSPH → PBF/FLIP + 刚体双向耦合 | ⬜ |
| **M6 风/海洋/车辆/机械** | 面元气动 + 浮力 + 约束族 | ⬜ |
| **M7 喷溅 + 收口** | 3DGS 三层用法（渲染桥/物理代理/隐式场）+ 全域收敛 | ⬜ |

> 三轴标准（**兼容 · 性能 · 真实化**）、域矩阵与「怎么搞」以 `docs/ROUTE.md` 为准；
> 本表里程碑顺序按该文件修订（原 V2 §11 顺序已被替代）。

## 结构（§1 crate DAG，禁止环）

```
crates/
├─ vxl-phys-core        数学(严格f32)/质量属性/配置/SoA体数据/调度抽象   [零外部依赖]
├─ vxl-phys-broad       宽相：均匀空间哈希网格（M1 增量 BVH bvh2 风格）
├─ vxl-phys-narrow      窄相：SAT 凸-凸 + 参考面裁剪流形 + 球解析 + 高度场特化
├─ vxl-phys-solver      顺序冲量(TGS-Soft 同族) + 并查集分岛 + 休眠
├─ vxl-phys-integrate   半隐式欧拉 + 固定步/子步
├─ vxl-phys-field       力场注册表（重力/风/吸引）
├─ vxl-phys-terrain     高度场账本（挖掘/包围盒）
├─ vxl-phys-replay      状态哈希（FNV-1a→xxh3 可换）+ 回放记录器
├─ vxl-phys             门面 World（默认管线组装）
├─ vxl-phys-soft        软体/布料 XPBD 参数骨架        [M3]
├─ vxl-phys-fluid       SPH/PBF/FLIP 参数骨架          [M3/M4]
├─ vxl-phys-wheeled     地面行驶域骨架（悬挂/轮胎/传动） [M2]
├─ vxl-phys-aero        气动域骨架（面元气动力）         [M2+]
├─ vxl-phys-marine      浮力采样/波浪骨架              [M2+]
├─ vxl-phys-mech        齿轮/皮带/活塞/马达骨架        [M2+]
├─ vxl-phys-destruction Voronoi 预断裂/运行时断裂骨架  [M2]
├─ vxl-phys-gpu         wgpu/rust-gpu 后端 trait 骨架  [M4]
├─ vxl-phys-memfind     内存子串（标量基线）           [M1+]
└─ vxl-phys-ffi         C ABI 批量接口约定             [M2]
```

依赖注入：`BroadPhase` / `NarrowPhase` / `ForceField` / `StateHash` / `PhysGpuBackend`
均为 trait，自定义实现不改核心。默认最小管线 = core+broad+narrow+solver+integrate+island。

## 快速开始

```bash
# 全量测试（含确定性/稳定性验收测试）
cargo test --workspace --release

# M0 出口门槛：1 万静态+1 千动态 ≥60Hz + 健康 + 双跑哈希 + arena 平稳（JSON 报告）
cargo run --release -p vxl-phys --example m0_gates

# 确定性验证：10 轮 × 600 tick 全等（不一致 = 退出码 1）
cargo run --release -p vxl-phys --example determinism

# 词汇禁令扫描（裁决 61）
bash scripts/vocab_scan.sh .

# 并发原语 loom 模型（可选本地跑；CI 四门之一）
RUSTFLAGS="--cfg loom" LOOM_MAX_PREEMPTIONS=3 cargo test -p vxl-phys-core --test loom_primitives
```

四门禁（CI，`feat/m0-gates` 起）：miri（core 子集）/ loom / TSan / ASan + 三编译器矩阵
（MSVC/GCC/Clang）+ aarch64 跨平台哈希比对；详见 `.github/workflows/ci.yml` 与
`docs/M0-GATES.md`。

## 确定性承诺（§5）

- 固定 60 Hz + 可配子步；渲染帧率与物理解耦；
- 严格 f32：`#![forbid(unsafe_code)]`、无 fast-math、无平台 intrinsic、归约有序
  （岛内约束按 (a,b,点序) 固定排序；跨岛顺序无关）；
- 状态哈希按 `f32::to_bits` 有序归约（M0 FNV-1a 64，接口可换 xxh3）；
- 跨平台矩阵（x86_64 × {MSVC,GCC,Clang} × {O0,O2,O3,LTO} + aarch64）在 CI 中逐步开启，
  哈希不一致 = 阻断提交。

## 提交门（§11）

- 单元 + 集成 + 属性测试（proptest 于 M1 引入）；金样对照：与 Rapier 同场景逐帧位姿对照（M1 接入 dev-dependency）；
- Criterion 基准 + 固定种子场景输入（M1 引入，需 crates.io 网络）；
- 性能报告字段 = 每帧物理 ms / 实体数 / 并行扩展比 / GPU 加速比 / 汇编收益。

## 许可

MIT OR Apache-2.0（依赖审计禁 GPL 传染）。

## 本机注意事项

- 仓库位于非 ASCII 路径（`D:\开发`）时，MinGW（`x86_64-pc-windows-gnu`）链接器
  无法解析目标文件路径。`.cargo/config.toml` 已把 `target-dir` 固定到
  `C:/vxl-wl-target` 规避；切换到 MSVC 工具链后可删除该配置。
- M0 零外部依赖（离线可构建）；xxh3 / Criterion / proptest / Rapier 金样 /
  wgpu 随 M1+ 引入（需要 crates.io 网络）。

