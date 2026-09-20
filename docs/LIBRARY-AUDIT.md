# LIBRARY-AUDIT —— 依赖 / 库选型体检（外部审计）

- 日期：2026-09-14｜审计方：unified-rx-mcp（外部只读审计，未改动任何代码）
- 方法：按 `spec/LIBRARY-POLICY.md` **三问**（**理念与设计契合 > 版本前沿 > 省 token**）
  逐件评估；数据源 = workspace 全 manifest + Cargo.lock 反向依赖反查 + deny.toml +
  ci.yml；不跑构建（纯静态核对）。
- 结论先行：**依赖姿态已达"极小面 + 纪律齐"水准——16 个 crate 仅 1 个运行期
  第三方依赖**；三问无高危项；下面 5 条为"到前沿线"的收敛项，全部低成本。

## 一、按类清单（§LIBRARY-POLICY §六 格式）

### ① 运行期第三方（1 件）
- **xxhash-rust 0.8.18**（features=["xxh3"]，仅 `vxl-phys-replay`）→ 用途=§5 确定性域
  内容散列（xxh3-128）。**理念契合：高**——确定性回放需要"构建稳定、跨平台位一致"
  的哈希，语言内置 `DefaultHasher` 明确不承诺跨版本稳定，自研哈希不值得；这是
  "该用库就用"的正例。**版本姿势：当前线 ✓**（0.8 系列活跃；0.8.18 为近版）。
  许可 BSL-1.0 已入 deny 白名单并注明 ✓。
- 其余 15 个 crate 的 `[dependencies]` = 纯 workspace 内部路径依赖（零外部）✓。

### ② 并发模型检查（cfg 门控 1 件）
- **loom 0.7.2**（`vxl-phys-core`，`[target.'cfg(loom)'.dependencies]` + 
  `unexpected_cfgs` check-cfg 配置）→ 并发原语模型检查的领域事实标准；
  **门控写法=现代最佳实践**（常规构建零外部重依赖的注释与意图一致）。
  **版本姿势：当前线 ✓**（0.7 系列为现行版）。lock 中 tracing 家族/cc/nu-ansi-term/
  windows-sys 全部为 loom 的传递树（反查实锤），**无"被遗忘的运行期依赖"**。

### ③ 构建期 / 系统绑定
- 无直用 `cc` / `bindgen` / `windows-sys`（后两者仅现于 loom 传递树）✓。

### ④ 自研域（**维持**的建议）
- 数学/内存/调度（core 自述"零外部重依赖"）、求解器等自研：与确定性域理念
  **契合**——跨平台位一致、可回放，是物理引擎的合理自持面。**仅当**未来出现
  稀疏线代/特征值/优化器等重型数值需求时，再按三问单点评估（**前沿候选：faer**；
  评估前置条件=确定性回放回归通过）。

### ⑤ 协议 / 许可 / 供应链姿态（非"库"但同类）
- `deny.toml`：许可白名单只列实际出现项（新增许可=显式开锁）、`yanked="deny"`、
  advisory 豁免带 expiry 纪律、`[graph]` 三平台 CI 各跑 ✓ 真在跑
 （`ci.yml:112` `cargo deny check` + `cargo machete` 未使用依赖检查）。
- `ci.yml`：四门禁 × 三编译器矩阵 + miri + loom + TSan + ASan + 强化档 clippy +
  action SHA 固定（zizmor 注明唯一滚动例外）——**供应链面属同类项目上游水位**。

## 二、发现（按优先级；全部低风险低成本）

| # | 级别 | 发现 | 建议 |
|---|---|---|---|
| F1 | **中** | `[workspace.package]` **edition = "2021"**（另 `resolver = "2"`）——相对当前稳定线落后一代 | 升 **edition 2024**：`cargo fix --edition` 逐 crate 走 + 全 CI 回归（本仓无宏生态，迁移面小）；随后 `resolver = "3"`（edition 2024 语义配套，依赖去重更优）。2024 的 let-chains 等对物理代码可读性有直接收益 |
| F2 | 中 | 无 **`rust-version`（MSRV）**——CI 用 stable 滚动，deny 的"可复现"理念未在 manifest 落一行 | `[workspace.package] rust-version = "<当前 stable 基线>"`，未来声明支持线时可下调验证 |
| F3 | 低 | `vxl-phys-splat` 对 core 用 `{ path = "../vxl-phys-core" }` 裸路径，其余 crate 统一 `{ workspace = true }` | 统一为 workspace 表引用（顺手消一处风格漂移） |
| F4 | 低 | `[workspace.dependencies]` 只覆盖 9/16 成员（未覆盖者多无被依赖关系，属有意为之时宜写明） | 加一行注释说明"只登记被依赖项"，或按需补全 |
| F5 | 可选 | 基准/属性测试无第三方（无 criterion/divan/proptest）——若 bench 走自研 harness 亦可 | **可选**：现代轻量基准 = **divan**（编译快、无黑箱）；求解器不变量 = **proptest**。非红线，按 token/收益自定 |

## 三、复核记录（审计留痕）

- 反查口径：Cargo.lock 逐包 dependencies 反向索引 → `tracing<-loom`、
  `smallvec<-tracing-subscriber`、`cc<-generator`、`windows-sys<-nu-ansi-term`、
  `loom<-vxl-phys-core`、`xxhash-rust<-vxl-phys-replay`——全部可归因，无孤儿/陈旧条目。
- 上表 F1/F2 不改变任何红线；F6（初评疑"deny 声称 CI 阻断但未见执行"）**已当场
  消解**：`ci.yml:112` 实际执行。
- 本报告只读产出，未改动被审仓库；若采纳建议，建议逐条走本仓既有 CI 回归。
