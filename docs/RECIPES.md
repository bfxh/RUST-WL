# 复现配方汇编（RECIPES.md）

> 全部基准/复现的命令行，Windows Git Bash + `CARGO_TARGET_DIR=C:/vxl-wl-target`。
> 所有配方都要求 release；确定性判定用同机双跑四哈希（ADR 0004）。

## 门禁链（每次源码改动落地前，ADR 0005）

```bash
cd "/d/开发/RUST WL"
export CARGO_TARGET_DIR=C:/vxl-wl-target
cargo fmt --all
cargo test --release > /tmp/test.log 2>&1; echo "test=$?"
cargo clippy --workspace --all-targets -- -D warnings > /tmp/clippy.log 2>&1; echo "clippy=$?"
bash scripts/vocab_scan.sh . > /tmp/vocab.log 2>&1; echo "vocab=$?"
/c/vxl-wl-tools/typos.exe . > /tmp/typos.log 2>&1; echo "typos=$?"
```

注意：**输出重定向 + 显式 `$?`**（管道会吞退出码，fb51911 事故）；日志文件先看尾部。

## 行为门（三命令四哈希）

| 场景 | 命令 | 基线（2026-09-15，求解预算 12×2 + 收敛早退标定后） |
|---|---|---|
| 门槛 + 压力 | `cargo run --release -p vxl-phys --example m0_gates` | 门槛 `0x4efce595fa2bd5219fb17d26fabcb189`、末态活跃 0、PASS；压力 `0x4426912b99c40ac5081c7a7547877eb0`（report-only，p50 ≈186 ms） |
| 确定性 | `cargo run --release -p vxl-phys --example determinism` | `FINAL_HASH=0xc5f1273240121304b5d5b1cd431636cb`（10 轮逐位一致） |
| T4 碎片雨 | `cargo run --release -p vxl-phys --example m1_islands` | 解算扩展 ≥3×（实测 4.80×）+ 串行/并行末态哈希逐位一致。**别加 `--` 参数**：会被当成第一个位置参数（clusters），4000 会跑到 ticks 上 |

**2026-09-15 哈希换代说明**（一次标定，两处行为改变）：
1. 求解预算 16×内层 4（64 扫掠）→ **12×内层 2（24 扫掠）**；
2. 新增**收敛早退**（每次扫掠累计最大速度级修正，跑满 6 次外层后残差 < 0.002 m/s 即停）。

上一档（可回溯）：12×2 无早退 `0xa2b4a080…` / `0x72c6e73b…` / `0x98d32c4a…`；
更早 16×4 `0xbdf0cee6…` / `0x2cc8c488…` / `0x8142fe05…`；再早 `0x443778a6…` /
`0x067ed531…` / `0x7684f708…`。标定数据与判据见 `EXPERIMENTS.md`「求解预算标定」节。

哈希按行为变化更新是**预期流程**（ADR 0004）：落地记录里必须显式写出新旧值与原因。

金样（另用独立 target 目录，避免污染主缓存）：
`CARGO_TARGET_DIR=C:/vxl-wl-target-gold cargo run --release -p gold-sample -- <scene> 600 16 0.01 4 3.0 30 4`
——**第 4 个位置参数（substeps）必须是 4**，这是塔能站住的配方，勿改。
三场景：`col45` / `pile5` / `tower25`。

## T1 求解稳定性（快回路，单轮 <1s）

```bash
# 125 体复现（泵定位专用）：5×5×5 留缝角点极限环，dE 峰值 148.9、125/125 不睡
cargo run --release -p vxl-phys --example diag_min -- 5 5 0.52 16 0.5 300 box 8 0.52 0.2 0 4
# 参数序：layers side spacing iters mu ticks floor threads xspacing baumgarte shock inner [skin]
# 单旋钮消融只改一个参数（μ=0 全睡 / 横距 0.55 全睡 / iters 32 只降能量）
```

| side | 体数 | 末态 |
|---|---|---|
| 2 / 3 | 20 / 45 | 全睡 ✅ |
| **5** | **125** | **125/125 极限环**（T1 主靶） |

## 8B 规模与分相（T2/T3 读数）

```bash
# 每相位每 tick 值（harness 已修 reset_timings）；树/查询/候选/岛数逐 tick 打印
cargo run --release -p vxl-phys --example m1_scale -- 8 102400 100000 120 16
# 解算成本扫描（§7）：末参 iters ∈ {4,16,32}，读「解算」列
cargo run --release -p vxl-phys --example m1_scale -- 8 102400 100000 40 <iters>
```

读数陷阱：`m1_scale` 的 `broad` 列 = 四项之和（每 tick）；但**别用累计值做逐相位
分析**（PhaseTimings 默认累计，harness 内部已 `reset_timings()`）。

## 其余验收基准

```bash
# T4 并行扩展（12 000 体碎片雨）：扩展倍数 + 串行/并行哈希逐位一致
cargo run --release -p vxl-phys --example m1_islands
# T6 8A 碰撞行（10×1200 静态格 + 10k tiles + 100 探针；验收 p95 ≤1.5ms）
cargo run --release -p vxl-phys --example m1_collision_row
# T1 验收主体（10k 动力堆 600 tick；当前不达标：pile5 1195/2000）
cargo run --release -p vxl-phys --example m1_pile
```

## 宽相研究资产（不在生产路径，ADR 0001）

```bash
# 三树形 A/B（二叉 / WideBvh 宽叶 / BVH8）：measure_full + margin 扫描 + cand_total
cargo test --release -p vxl-phys-broad --test diag_wide
cargo test --release -p vxl-phys-broad   # wide 21 项 + bvh8 30 项测试
```

## 窄相 SIMD 守门（ADR 0002）

```bash
cargo test --release -p vxl-phys-narrow
# simd_matches_scalar_bitwise：4000 组正交基盒对，SIMD 与标量孪生逐位一致
# simd_tail_lanes_are_ignored：尾 lane 无效值不泄漏
```
