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
**2026-09-15 更正**：clippy 这一项此前若干轮是**红的**（`solve_constraint` 8 参数漏挂
`#[allow(clippy::too_many_arguments)]`，`normal_inner` 落地时引入、非本轮）——已按仓库
同风格补挂，门禁链现在真的五项全绿。凡是"门禁全绿"的结论都必须**逐项贴退出码**，
不能只看 test。

## 行为门（三命令四哈希）

| 场景 | 命令 | 基线（2026-09-15，参与式降点档） |
|---|---|---|
| 门槛 + 压力 | `cargo run --release -p vxl-phys --example m0_gates` | 门槛 `0x6219d1866b20c002d806d3d699a487ff`、末态活跃 0、PASS；压力 `0x63e5eb35b71b8c84ade4a053aeecb900`（report-only） |
| 确定性 | `cargo run --release -p vxl-phys --example determinism` | `FINAL_HASH=0xd8601988ad7989ffb58ba8b956c2f8db`（10 轮逐位一致） |
| T4 碎片雨 | `cargo run --release -p vxl-phys --example m1_islands` | 解算扩展 ≥3×（实测 4.39×）+ 串行/并行末态哈希逐位一致。**别加 `--` 参数**：会被当成第一个位置参数（clusters），4000 会跑到 ticks 上 |

**2026-09-15 参与式降点换代**（`vxl_phys_solver::point_reduce_after = 3`）：
`0x6219d186…` / `0x63e5eb35…` / `0xd8601988…`（当前）←
`0x4dcf5d46…` / `0x1e855f89…` / `0x3a8c778e…`（旧世代；把 `point_reduce_after`
改回 0 可逐位复现，A/B 对照用）。**旧世代回退法**：常函数改 0 → 重建 → 三哈希应逐位复现。

**关节族落地（2026-09-15，M2 首切片）三个门哈希未变**（关节不参与这些场景，
`PhysConfig` 新增 `joint_iterations` 字段对它们零影响）——哈希**不变**与换代一样要记录。

**2026-09-15 哈希换代链**（同日多轮标定，全部可回溯；新→旧）：
6 扫掠档（当前，`0x4dcf5d46…` / `0x1e855f89…` / `0x3a8c778e…`）→
世界逆惯量预积（`0x7d5f3723…` / `0x8984a075…` / `0x6120a78c…`）→
2 子步 × 5（`0xa53dc9d4…` / `0x8c680bca…` / `0x56e3f638…`）→
12×2 + 早退（`0x4efce595…` / `0x4426912b…` / `0xc5f12732…`）→
12×2（`0xa2b4a080…` / `0x72c6e73b…` / `0x98d32c4a…`）→
16×4（`0xbdf0cee6…` / `0x2cc8c488…` / `0x8142fe05…`）→
`0x443778a6…` / `0x067ed531…` / `0x7684f708…`。

哈希按行为变化更新是**预期流程**（ADR 0004）：落地记录里必须显式写出新旧值与原因。

金样（另用独立 target 目录，避免污染主缓存）：
`CARGO_TARGET_DIR=C:/vxl-wl-target-gold cargo run --release -p gold-sample -- <scene> 600 16 0.01 4 3.0 30 4`
——**第 4 个位置参数（substeps）必须是 4**，这是塔能站住的配方，勿改。
三场景：`col45` / `pile5` / `tower25`。

## 求解成本定位（每点开销）

```bash
# 1) 固定/每扫掠两半分离：扫掠 = 2×iters（金字塔 phase 读数做线性拟合）
for k in 1 2 3 6; do cargo run --release -q -p vxl-phys --example arena_bench pyramid --iters $k; done
#    参考：2 扫掠 1010.6 µs / 4 → 1382.6 / 6 → 1664.9 / 12 → 2253.4 ⇒ 每扫掠 ≈124 µs、固定 ≈762 µs
# 2) 固定部分落在哪：读 `求解细分/步` 行（ImpulseSolver::last_detail_us = 建岛/约束构建/热启动/扫掠）
cargo run --release -q -p vxl-phys --example arena_bench pyramid
```

**改动级证据只用同进程交错 A/B**（跨时间点的绝对值差会被会话降频污染——本轮实测
同一份代码 40 分钟内从"构建 392 µs"漂到"490 µs"）：

```bash
cd /d/开发/physarena
npm run build:vxl && cp public/vendor/vxl/vxl_phys_wasm.wasm out/new.wasm   # 新构建
cd "/d/开发/RUST WL" && git stash push -- crates/vxl-phys-solver/src/lib.rs crates/vxl-phys/examples/arena_bench.rs
cd /d/开发/physarena && npm run build:vxl && cp public/vendor/vxl/vxl_phys_wasm.wasm out/old.wasm  # 旧构建
cd "/d/开发/RUST WL" && git stash pop
cd /d/开发/physarena && node scripts/vxl-simd-ab.mjs out/old.wasm out/new.wasm   # 3 轮交替 + 位指纹
```

要点：**别用 `git stash push -- <源码文件>` 做 A/B**——Mimosa hook 会按"Bash 直接写
源码"拒绝（stash 会改写工作区源码）。改用**只读 worktree** 检出旧提交，各自独立
target 目录：

```bash
cd "/d/开发/RUST WL"
git worktree add "D:/开发/RUST-WL-old" <旧提交>
cd "D:/开发/RUST-WL-old" && CARGO_TARGET_DIR=C:/vxl-wl-target-old \
  cargo build --release -q -p vxl-phys --example arena_bench
# 交替跑两个二进制（3 轮），同温窗才有可比性：
OLD=C:/vxl-wl-target-old/release/examples/arena_bench.exe
NEW=C:/vxl-wl-target/release/examples/arena_bench.exe
for r in 1 2 3; do for s in pyramid ballpit; do
  echo "$s 旧 $($OLD $s | grep -o 'p50 [0-9.]*')  新 $($NEW $s | grep -o 'p50 [0-9.]*')"
done; done
git worktree remove "D:/开发/RUST-WL-old" --force   # 收尾
```

A/B 跑完必须 `npm run build:vxl && npm run build` 把 `public/` 与 `dist/` 都刷成
当前构建（否则浏览器测的是旧 wasm——本轮与上一轮都踩过）。**换代哈希的改动还要
过金样**（见下），两道门都贴读数才算过。

## 金样（保真度门，换代哈希时必跑）

```bash
cd "/d/开发/RUST WL/gold-sample"     # 独立 workspace：-p 要在这里用
CARGO_TARGET_DIR=C:/vxl-wl-target-gold cargo run --release -q -- col45 600 16 0.01 4 3.0 30 4
# 三场景 col45 / pile5 / tower25；判据：末态「全睡 + 最深穿透 ≈0 + y 带不塌」，
# 与 Rapier 同列对照。**本轮教训**：扫掠预积角向量改动时序 −3.8% 但让 col45
# 从 45/45 入睡退到 40/45 ⇒ 整项回退（见 EXPERIMENTS）。
```

**保真档（2026-09-18 实测，用于"要更好的速度列/入睡列"时）**：把第 8 个参数
（`substeps`）从 4 加到 **8 或 16**——tower25 读数（同配方其余不变）：

| substeps | \|v\|max@400 | KE@400 | 最深 | 阈值下（可睡体）/2500 | 成本 |
|---|---|---|---|---|---|
| 4（默认） | ~0.29 | ~900 | 0.013–0.017 | 294 | 206 ms/tick |
| 8 | 0.162 | 301 | 0.009 | **1112** | ~2× |
| 16 | 0.170 | 109 | 0.008 | **2080** | ~4× |

⇒ 要更保真的读数用 `... 16 0.01 4 3.0 30 8`；**默认档不要换**（8B 侧回归不可接受，
理由见 EXPERIMENTS 末节）。

**保真档之二：无偏置末趟**（第 10 参 `stabilization`，0 = 关闭＝引擎默认；第 9 参是
`dump` 路径，实验时传 `-` ＝ 不转储）。它是 Rapier 的"带偏置趟 → 位置积分 → 无偏置趟"，
把去穿透偏置从**最终速度**里移除。
**当前最佳保真档 = `... 16 0.01 4 3.0 30 8 - 2`**（8 子步 + 末趟 2 遍）：

| 场景（600 tick） | 基线 | stab=2（4 子步） | 备注 |
|---|---|---|---|
| col45 | 45/45 入睡 | **45/45** | 中性 |
| pile5 | 1735/2000、KE ~10 | **1960/2000、KE 1.6** | 大改善 |
| tower25 | KE 905、最深 0.017 | KE 650、最深 **0.009** | 改善但不入睡 |

⚠️ **它不是普适改善**：T1 主靶 **125 体留缝角点场景退化**（末态 KE 1.6→18.1）——
`diag_min ... 4 0.02 <stab>` 可复现（第 13 参 skin 必须显式传，默认 0.02 与原场景相同）。
所以**默认档保持 0**；只在"以堆/塔保真为主"的对拍里开。

**性能权衡档：检测每步一次**（编译期开关 `detect_once_per_tick`，
`crates/vxl-phys/src/lib.rs` 顶部；默认 `false`）。开时非首子步跳过宽相+窄相、复用本
tick 流形表，且**仅在准静态**（`max|v|·dt ≤ 0.5·skin`）才复用：

| 判据 | 关 | 开 |
|---|---|---|
| pyramid 宽相+窄相 | 295 µs | **149 µs**（约 −14% 帧） |
| tower25 KE（stab=2） | 605 | 1046（×1.7，**不崩**） |
| tower25 可睡体 | 342 | 237 |
| 125 体 末态 KE | 1.6 | **0.5** |

**哈希世代（开启态）**：门槛 `0xb8d4bb4e0c71a1a5c54d6b2fdc2e4bfd`、压力
`0x94983ea167e4ef261646a24e943c8c4f`；确定性 `0xd8601988…` 未变（该示例未进入复用路径）。
关闭态与基线三哈希逐位一致。详见 `EXPERIMENTS.md` 末节 L。

**两个读数陷阱（本轮踩过）**：
1. `max_corrective_velocity`（第 6 参）**往下扫才有信息**：它钳的是去穿透偏置速度
   `erp_inv_dt·pen`，塔场景实测该值 ≈0.1–0.24 ⇒ **≥0.2 的三档逐位相同**（3.0/0.5/0.2）；
   0.1 起才变，0.02 会毁掉去穿透（|v|max→1.08）。
2. 判「Rapier 是不是因为阈值宽才睡」不能靠社区口径（线性 0.4 是错的）：**读源码**
   `rapier3d-0.35.3`（本地 registry 有）：`normalized_linear_threshold = 0.05`，比较
   **最远点速度** `|v|+|ω|·extent` **外加位置位移率**；角速度对带碰撞体的体被折进前者。
   ⇒ 我们塔的点速度 ~0.31 ≫ 0.05，**Rapier 也不会让它睡**——差距是真实运动差。

## 关节族（M2 首切片）自检

```bash
# PhysArena jointProbe 逐字复刻（5 种关节；期望：全 PASS、峰值 0.0000 m、
# 悬挂体末态 y 与初值相同 = 关节真的在顶住重力。掉到 y≈0.4 = 自由落体的假通过）
cargo run --release -p vxl-phys --example arena_bench joints
# 动态关节链：40 环铰链 + 40 段固定塔（600 步）；期望链 ≤0.02 m、塔 ≤0.03 m、cos≈1
cargo run --release -p vxl-phys --example arena_bench joint_chains
# 关节迭代预算敏感性（默认 joint_iterations=8；链分离 3 迭代 4.8cm → 8 迭代 2.0cm）
#   改档：编辑 PhysConfig::joint_iterations（无 CLI 开关，避免与接触 --iters 混淆）
```

| 判据 | 读数（joint_iterations=8） |
|---|---|
| 5 探针锚点分离 | 峰值 0.0000 m（容差 0.2/0.2/0.3/0.2/0.35） |
| 铰链链 40 环最大分离 | 0.0204 m（3.4% 链节长），0.135 ms/步 |
| 悬挂塔最大分离 / 姿态 | 0.0253 m / 相邻轴同向 cos = 1.0000，0.127 ms/步 |

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
