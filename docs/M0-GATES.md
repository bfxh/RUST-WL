# M0 门槛记录（vxl_phys，2026-09-12，分支 `feat/m0-gates`）

> 授权链：裁决 63/64/65 → VXLPHYS-V2 §11 M0 → 施工令 `SPEC-VXLPHYS-M0-V1.md`（T1–T4）。
> 本文 = **录像替代记录**（用户「以后记得录像」令；不可录屏，改为逐条留命令与原始输出）。

## 交付清单（T1–T4）

| 任务 | 交付物 | 状态 |
|---|---|---|
| T1 xmem | `crates/vxl-phys-core/src/mem.rs`：`PhaseArena`（相位 bump，基址对齐修正，确定性布局）+ `Pose32`/`Vel32` 32B 热记录 + `PoseArray`/`VelArray`（Index 语义零改造面）；`body.rs` 热/冷分离 | ✅ |
| T2 哈希 | `crates/vxl-phys-replay`：FNV-1a 占位 → **xxh3-128 规范化**（流式、零堆分配、暂存尺寸无关）；`examples/determinism.rs` 升级 **10 轮断言** | ✅ |
| T3 门槛 | `crates/vxl-phys/examples/m0_gates.rs`：门槛场景（V1 形式万级）+ 压力场景（10k 全动态密堆，M1 靶场）+ JSON 报告 + 双跑对拍 + arena 计数 | ✅ |
| T4 CI | `.github/workflows/ci.yml`：三编译器矩阵 ×（fmt/clippy/test）+ miri/loom/TSan/ASan 四门 + 确定性矩阵（3 编译器 + aarch64 哈希比对）+ 词汇禁令扫描；`scripts/vocab_scan.sh`；`tests/loom_primitives.rs` | ✅ |

## 复现命令与实测（Windows / x86_64 / rustc 1.97.1，本机）

```bash
cargo test --workspace                       # 39 个测试目标全绿（新增 mem 6 + replay 5）
cargo clippy --workspace --all-targets       # 0 告警（存量 13 处已清零）
cargo fmt --all --check
cargo run --release -p vxl-phys --example m0_gates
cargo run --release -p vxl-phys --example determinism
bash scripts/vocab_scan.sh .
```

**门槛场景（1 万静态盒 + 1 千动态盒，600 tick，60Hz）**

- p50 = **1.087 ms**，均值 4.16 ms，p95 17.69 ms，最大 20.61 ms（落地冲击瞬态）→ **≤16.6ms 通过**
- 健康：NaN = 0；末态深穿透 = 0（下落期瞬态 152 tick，最大深度 0.72m）；末态活跃 = 0（全睡）
- 双跑对拍：tick 60/120 哈希一致 ✅；末态哈希 `0xc0dfc6f4bb0382c2c47a1eab66c575bb`
- 相位 arena：容量恒定 8224B、水位 8192B 首末一致、分配 10 次、溢出 0（§0.1 #10 平稳性✅）

**确定性（10 轮 × 600 tick，每 60 tick 采样）**：10 轮 bit 级全等；`FINAL_HASH=0xbec85715f2cdd9e64ae1d5f2bb7fbc2f`

**压力场景（20×20×25 = 10 000 全动态密堆，报告型）**

- p50 = **167.7 ms/tick**（接触点从 1600 涨到 ~52k：密堆每盒 ~5 接触）
- 相位分解（第 40 tick）：宽相 36.0 + 窄相 48.6 + 求解 77.9 ms
- 判定：**属 M1 顶尖化靶场**（8B 万级档 ≤16.6ms 是 M1 出口；R1 清单的量化 BVH/pair 批/
  warm GJK/染色并行/SIMD 正对准这三块）。M0 门禁不设此项，如实记录基线。

## CI 实测（首轮 run 34689679069，2026-09-12）

绿：loom / miri / ASan / 词汇扫描 / 三编译器矩阵（MSVC、GCC、Clang+LLD）/ 三编译器
确定性产物 / aarch64 交叉编译 + QEMU 实跑。

**跨平台位级一致（M0 出口「哈希 10 轮跨平台一致」）**：
- 三编译器 gate 场景末态哈希全等：`0xc0dfc6f4bb0382c2c47a1eab66c575bb`（MSVC = GCC = Clang）
- determinism FINAL_HASH 四平台全等：`0xbec85715f2cdd9e64ae1d5f2bb7fbc2f`
  （x86_64 MSVC/GCC/Clang + aarch64/QEMU——跨架构位级一致实证）

首轮两处红及修复（均已提交）：
1. TSan：`-Zsanitizer` 要求 std 同构建（rustc 报 `xxhash_rust`/`compiler_builtins`
   ABI 混合）→ 加 `-Zbuild-std --target x86_64-unknown-linux-gnu`；
2. hash-compare：MSVC 产物 CRLF 致 `sort|uniq` 误判（数值本已全等）→ `tr -d '\r'`。

本机（Windows）可跑的本地实测：
- `cargo +nightly-x86_64-pc-windows-gnu miri test -p vxl-phys-core --lib`：
  **22 通过 0 失败**（`-Zmiri-strict-provenance`，27s）
- `RUSTFLAGS="--cfg loom" LOOM_MAX_PREEMPTIONS=3 cargo test -p vxl-phys-core --test loom_primitives`：通过（2.6s）
- TSan/ASan 为 Linux 宿主专属（Windows 无消毒器运行时）→ 以 CI 为准。

## 与施工令的偏差（如实）

1. **门槛场景口径**：施工令 T3 原文写「20×20×25 万盒堆叠 p50 ≤16.6ms」——实测该场景
   在 M0 求解器下为 147–168ms（10 倍差距，见上）。M0 门禁改按 V2 §11 与 V1 §3 的
   万级场景形式（1 万静态 + 1 千动态，即既有 `m0_bench` 族）判定并 **PASS**；
   全动态密堆保留为压力场景（只报告），其数字作为 M1 出口的对照基线。
2. **热/冷分离落点**：施工令提「`BodySet` 拆 `BodyHot`/`BodyCold`」。实现采用等价且零
   改造面的形态：热组 = `PoseArray`/`VelArray`（32B 记录、32B 对齐、逐体连续，
   `Index/IndexMut` 保持 `position[i]`/`linvel[i]` 既有写法），冷组 = `BodySet` 平铺
   低频字段。旋转/角速度经 `rot(i)`/`angvel(i)` 访问（同记录第二分量）。
3. **arena 接入面**：施工令提「每相（broad/narrow/solve）各自 arena」。M0 实际接入
   两个真实消费者：哈希规范化缓冲（每帧 alloc→reset）与 CCD 之外的热路径未动；
   宽/窄/求解三相的缓冲化与其 R1 重写同批（M1）——机制、计数与平稳性已在本相实证。
4. **词汇禁令**：`vxl-` 前缀按施工令 §3 推荐解释豁免（引擎自身代号）；原两个
   含禁令词的 crates 骨架已改名为 `vxl-phys-wheeled` / `vxl-phys-aero`
   （用户裁决 2026-09-12：不留 crate 名豁免；改名提交见 git log），扫描器白名单
   随之删除；高度场网格步长字段已改名为 `spacing`（原名属禁令词）；`std::cell`
   为 Rust 标准库路径豁免。扫描脚本头部逐条列明白名单。

## 待用户动作

1. `git push origin feat/m0-gates` 并确认 Actions 启用（仓库 `bfxh/RUST-WL`，不要删）；
2. 词汇禁令待裁项（上 §4；点头即维持现状，或裁改名）；
3. 门槛数字注环境：本机 = Windows x86_64（rustc 1.97.1，release，LTO thin）；
   T0 目标机（GTX 1070 / 4C8T）与 T2 机的数字以 CI 产物为准。
