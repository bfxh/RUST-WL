# RUST WL —— vxl_phys 自研 Rust 物理引擎

> 依据：[`docs/PHYS-ENGINE-VXLPHYS-V1.md`](docs/PHYS-ENGINE-VXLPHYS-V1.md)
> （PHYS-ENGINE-VXLPHYS-V1 规格书）。定位：**独立商业级 Rust 物理引擎**，
> 纯 Rust、确定性优先、模块化 crate DAG；404 是它的第一个大型客户。
> 仓库目录名「RUST WL」= **Rust 物理（WuLi）**。

## 状态：M0 MVE（进行中）

| 里程碑 | 内容 | 状态 |
|---|---|---|
| **M0 MVE** | 盒/圆柱/高度场 + 顺序冲量 + 岛 + 确定性哈希；1 万静态+1 千动态 ≥60Hz | ✅ 竖切可跑（见下） |
| M1 刚体商业级 | §3 最低通过档 + §4.1-4.5/4.11-4.14 全档 + TGS-Soft + BVH + GJK/EPA + 并行 JobSystem | ⬜ |
| M2 车辆+地形+破坏 | §4.9/4.10 | ⬜ |
| M3 软体/布料/流体 CPU | §4.6-4.8 CPU 档 | ⬜ |
| M4 GPU | §8 全部 | ⬜ |

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
├─ vxl-phys-vehicle     射线悬挂+刷子轮胎参数骨架      [M2]
├─ vxl-phys-vehicles-air 面元气动力骨架                [M2+]
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

# M0 演示：6 层金字塔 + 圆柱 + 球，10 秒，健康检查
cargo run --release -p vxl-phys --example m0_pyramid

# M0 基准：1 万静态 + 1 千动态，门槛 ≥ 60 Hz
cargo run --release -p vxl-phys --example m0_bench

# 确定性验证：两次 600 tick 全程哈希比对（不一致 = 退出码 1）
cargo run --release -p vxl-phys --example determinism
```

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

