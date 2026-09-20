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
| 门槛 + 压力 | `cargo run --release -p vxl-phys --example m0_gates` | 门槛 `0x6a932b44622d9bb5b06cd2d6005c9c64`、末态活跃 **4**、PASS；压力 `0x417be20a8e49c9b0436987415ac9961a`（report-only） |
| 确定性 | `cargo run --release -p vxl-phys --example determinism` | `FINAL_HASH=0x7acbdfac46b03aaaebc04ca0c30bfc4f`（10 轮逐位一致） |
| T4 碎片雨 | `cargo run --release -p vxl-phys --example m1_islands` | 解算扩展 ≥3×（实测 4.39×）+ 串行/并行末态哈希逐位一致。**别加 `--` 参数**：会被当成第一个位置参数（clusters），4000 会跑到 ticks 上 |
| **默认档长跑稳定性**（新增 2026-09-20） | `cargo test --release -p vxl-phys --test default_tier_stability` | 两个测试：① 冻结读数 `top_y 2.7285 / Σv² 1.8124 / awake 216 / manifolds 919`（6×6×6、3000 步、默认档，两次连跑逐位一致；**2026-09-21 换代**，旧世代 `2.7223 / 2.1488 / 216 / 835`）；② **金丝雀**——降到 4 扫掠必须明显不同（实测流形 919→455、Σv²→2.2089、清醒→188、top_y→2.7347），否则场景不灵敏、门无效。**为什么要它**：金样配方自带 `16` 迭代 ⇒ 默认档（6 扫掠）的改动**金样门看不见**（`EXPERIMENTS` 记过的覆盖缺口）。改动默认档时更新那四个冻结值并按 ADR 0004 记换代理由；`--nocapture` 可读实际读数 |

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

**2026-09-21 换代：切向漂移回拉「只对粘着接触生效」**（`vxl_phys_solver` 的
`DRIFT_STICK_RATIO = 0.99`——滑动接触的锚点分离**就是**真实材料滑移，回拉会**抹掉真实滑动**
并因力臂 ∝ μ 地注入转矩，是角向残差的主项）：
门槛 `0x6219d186…` → **`0x6a932b44…`** · 压力 `0x63e5eb35…` → **`0x417be20a…`** ·
确定性 `0xd8601988…` → **`0x7acbdfac…`** · 默认档冻结 `2.7223/2.1488/216/835` →
**`2.7285/1.8124/216/919`**。
**换来的质量（同口径对照）**：金样三场景——塔超阈总数 483→**247**（其中角向 261→**122**，
tower 2000 tick 角向 202→**93**）、col45 Δpos 0.0118→**0.0048**、
pile5 **满睡 2000/2000**（基线 1980）且 Δpos 0.0157→**0.0059**；
默认档 `arena_bench pyramid --steps 3000` 最大穿透**不变**（−0.0199 m）、
Σv² 1.4718→**1.0517（−29%）**；`m0_gates` 末态活跃 51→**4**；arena 自检 **19/0/0**
（逐探针与基线相同）。
**代价**：塔 ms/活跃tick **+3.6%**（135.6→140.5）；`m0_gates` **瞬态**最大深度
0.2561→**0.6587**（末态深穿透仍 0）。
**注**：上表旧基线写的"末态活跃 0"是**陈旧读数**（同哈希下实测基线为 51），本次照实记为 4。
存档：`OPEN-PROBLEMS.md` P1（含可复现改动全文、四种变体 Pareto 对照、位置级路线为何关闭）。

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

## 浏览器对拍自检（PhysArena，19 探针 × 9 引擎）

```bash
cd /d/开发/physarena
npm run build:vxl && npm run build      # 刷成当前构建（先做，否则测的是旧 wasm）
(npx vite preview --port 4173 &)        # 自检脚本走 http://localhost:4173
ARENA_TAG=verify node scripts/arena-drive.mjs selftest   # 落盘 out/selftest-verify.json
```
**基线（2026-09-20 实测，当前 HEAD）**：vxl-phys = **19 pass / 0 degraded / 0 fail**。
形状四项全部转为**真实现**：`shape-capsule`（解析支撑 + 窄相解析最近点）、`shape-cylinder`
（多面体棱柱 + 桥/适配层接线）、`shape-cone`（新增 `Shape::Cone`，多面化）、
`shape-compound`（新增 `Shape::Compound`，窄相按子形状展开）。**关节 5/5 与稳定性 5/5 全过**
（含 `stability-sleep`）。同场：Rapier 19/0/0、Crashcat 18/1、Bullet 16/3、Havok 15/4、
cannon-es 15/4、Jolt 13/6、PhysX 13/6、Oimo 10/9。脚本另报 1 条资源加载失败（HTTP 未找到；
判定"失败项 0"，不影响结论）。

**历史基线（同日早先，A9 收口之前）**：15 pass / 4 degraded / 0 fail（boot 4.5 ms）——4 项
degraded **全是形状近似**（capsule / cone 引擎侧没有这两种形状；cylinder 未接线；复合体退化为
AABB 角点并集凸包）。那正是 `TECH-SURVEY.md` §6 的 **A9**，当日四项全部收口。
⇒ **形状保真不再是本仓在浏览器对拍里的缺口**；下一步看的是稳定性/性能类探针。

## 金样（保真度门，换代哈希时必跑）

```bash
cd "/d/开发/RUST WL/gold-sample"     # 独立 workspace：-p 要在这里用
CARGO_TARGET_DIR=C:/vxl-wl-target-gold cargo run --release -q -- col45 600 16 0.01 1 3.0 30 16
# **参考配方（2026-09-20 起）= iters 16 / inner 1 / substeps 16**（同总扫掠数 256 下，
#   把预算从"重复扫"挪到"重新线性化"：残差与入睡两列同时变好，代价塔上 +30%）。
#   旧配方 16/4/4 的读数**按历史保留、不与新表混比**。
# 三场景 col45 / pile5 / tower25；判据：末态「全睡 + 最深穿透 ≈0 + y 带不塌」，
# 与 Rapier 同列对照。**本轮教训**：扫掠预积角向量改动时序 −3.8% 但让 col45
# 从 45/45 入睡退到 40/45 ⇒ 整项回退（见 EXPERIMENTS）。
```

**基线（2026-09-20 起，**参考配方 16/1/16**）**——600 tick 末态：

| 场景 | vxl 末态 | Rapier 同列 | 旧配方 16/4/4 对照 |
|---|---|---|---|
| col45 | **45/45 入睡**、KE 0.0、超阈 0、最深 0.000、y 带 [0.25, 2.25]（活跃 456/600 tick，**0.70 ms/活跃tick**） | 45/45、−0.000 | 45/45、KE 0.0（无变化） |
| pile5 | **1980/2000 入睡**、KE **0.1**、超阈 **0**、最深 0.006、y 带完整（**14.89 ms/活跃tick**） | 2000/2000、−0.000 | 1735/2000、KE 9.9 ⇒ **+245 体入睡、KE ÷99** |
| tower25 | **0/2500 入睡**、\|v\|max **0.091**、KE **103.5**、超阈 **483**、最深 0.008、y 带 [0.24, 12.20]（**136.6 ms/活跃tick**） | 2500/2500、KE 0.0 | 0/2500、\|v\| 0.224、KE 822、超阈 2206 ⇒ KE ÷8、超阈 ÷4.6（代价 +30%） |

⇒ 旧配方读数**一律按历史保留、不与本表混比**（跨时间点绝对值差会被会话降频污染，见上文）。
⇒ tower25 仍未入睡（超阈 483 体 × 单岛原子规则）是 `OPEN-PROBLEMS.md` P1 的已知缺口；
   **pile5 已接近满睡**（1980/2000）。

**⚠️ 参考配方的适用域：高摩擦（μ≥2.5）下 `16/1/16` 会垮 —— 已实测（2026-09-20）**：
`VXL_MU=2.5/3/4/5` + **col45**（45 体单列、**亚秒级复现**）从 **μ≥2.5 起整体垮塌**
（μ=5：KE 6752、y 带 2.25→0.80、Δpos 5.83 m）；μ=2.0 仍稳（45/45 睡、Δpos 0.0094）。
**同总扫掠 256、只改分配**（μ=5，col45）：

| 分配（iters/inner/substeps） | 重线性化次数 | μ=5 末态 |
|---|---|---|
| 16 / 4 / 4 | 4 | 30/45 睡、KE 0.4、y 带完整（稳） |
| **16 / 2 / 8** | 8 | **45/45 睡、KE 0.0（完好）** |
| 16 / 1 / 16 | 16 | **垮塌**、KE 6752、y 带塌到 0.80 |

⇒ ① 现行金样判据**只测 μ=0.5 ⇒ 测不出这个洞**，μ 档须进判据；
② 该失败**不随 dt 变小而消失**（substeps 64 更糟：y 带到 −121.98、KE 347 219），
   故**不是**积分稳定性极限；失败随重线性化次数增长（与"逐子步注入"同族）。

**⚠️ 另一侧的洞：`inner=1` 配**低子步**也会发散（2026-09-20）**：
`16/1/4`（inner=1、substeps=4）在塔 1200 tick 上 **KE 44 万 J**、y 带到 **−358** ⇒ **发散**。
⇒ **`inner=1` 需要足够的子步**（此前"inner=1 稳定"只在 substeps=16 测过 ⇒ 范围修正）。
**合起来看子步的稳定区随 μ 移动**：μ=0.5 需 substeps ≥ 8；μ=5 在 substeps=16 已破、
substeps=8 完好 ⇒ **`substeps=8` 是跨 μ 的稳健点**（这正是 `16/2/8` 的取值）。
**子步标度（窗口均值，塔 600–1200，inner=1）**：总超阈 1539(sub8) → 414(sub16) → **154(sub32)**，
比值 3.7×→2.7×（**减速**）；其中线性 561→168→18（加速崩塌）而角向 468→211→117（**减速 ⇒ 有地板**）
⇒ 角向残差**不能靠加子步消除**。

**`16/2/8` 在 μ=0.5 的实测（2026-09-20，候选替换配方；此前无读数）**：

| 场景 | 16/4/4（旧） | **16/2/8** | 16/1/16（现行参考） |
|---|---|---|---|
| col45 600 | 45/45、KE 0.0 | **45/45、KE 0.0** | 45/45、KE 0.0 |
| pile5 600 | 1735/2000、KE 9.9 | **2000/2000、KE 0.0** | 1980/2000、KE 0.1 |
| tower25 300 超阈 | 2206 | **1568** | 587 |
| tower25 600 超阈 / KE | — / 822 | **1476 / 326.6** | 483 / 103.5 |

⇒ **`16/2/8` 在旧配方与现行参考之间，且是唯一在高摩擦下不垮的档**：
塔残差 1476（比 16/1/16 差 3×、比 16/4/4 好 1.5×），但 **pile5 满睡 2000/2000（三者中最好）**
⇒ 若"格调统一、又要高摩擦鲁棒"是目标，**`16/2/8` 是 Pareto 更优的参考配方候选**；
塔残差与重线性化次数**单调**（2206 → 1476 → 483）⇒ 这是一条**可调的旋钮**，不是取舍。
（代价列未单独复测：同为 256 扫掠、重检测 8 次 < 16 次 ⇒ 应落在 105–131 ms/活跃tick 之间。）

**配方侧的"睡眠优先"替代（2026-09-20 实测，塔 300 tick）**——同**总扫掠数 256**
（`iters 16 × inner × substeps`）下，把预算从"重复扫"挪到"重新线性化"：

| 配方（iters / inner / substeps） | 每迭代扫掠 | \|v\|max | **超阈体数** | 阈值下 | ms/活跃tick |
|---|---|---|---|---|---|
| 16 / 4 / 4（现行保真配方） | 16 | 0.305 | **2206** | 294 | 105 |
| 16 / 1 / 16（睡眠优先） | 16 | 0.093 | **587** | 1913 | 131 |

⇒ 同扫掠数下**超阈体数少 3.8×**，代价 **+25%**（多出的 12 次检测很便宜，≈5 ms/次）；
⚠️ `|v|max` 是**离群指标**（子步再加到 32 时它反飙到 0.568，而超阈体数继续降到 277）
⇒ **判睡眠一律看"超阈体数"**。

**三场景验证（2026-09-20 实测，600 tick）**：

| 场景 | 现行 16/4/4 | **候选 16/1/16** | 判定 |
|---|---|---|---|
| col45 | 45/45 入睡、KE 0.0 | **45/45 入睡、KE 0.0、超阈 0** | 无变化（都满） |
| pile5 | 1735/2000、KE 9.9 | **1980/2000、KE 0.1、超阈 0** | **+245 体入睡、KE ÷99** |
| tower25 | 0/2500、超阈 2206 | 0/2500、**超阈 587** | 大幅接近（仍差 587 体） |

⇒ **"inner=1 会伤堆叠稳定性"这一风险被实测否掉**（col45 仍 45/45 全睡，pile5 反而多睡 245 体）
⇒ 这不是取舍：**残差与入睡两列同时变好**，代价 +25%（105→131 ms/活跃tick）。
**建议把 `16/1/16` 升为参考配方**：保真对拍若要跨时间比较，应在此配方下**重建基线**
（旧读数按历史保留，别混比）。
判据：四哈希（本改动不动引擎默认 ⇒ 应逐位不变）+ 金样三场景（本次已过）+ arena 19/0/0。

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
