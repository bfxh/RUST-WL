# 发版约定与流程（RELEASE.md）

> **状态：提案，待拍板**（2026-09-21 立）。本档 + `.github/workflows/release.yml`
> 都**不发布任何东西**——workflow 只在**打 tag 时**才触发，现在没有任何 tag，所以它是惰性的。
> **要真正发一版，只需你给一句话的版本号规则**（见 §2 的三个选项）。

## 1. 现状（先说清楚：发版基础设施此前为零）

- **没有任何 tag**；`Cargo.toml` 版本 `0.1.0`（workspace 根 + 各 crate）。
- `.github/workflows/` 只有 `ci.yml`（成熟：SHA 固定 action、最小权限、成本分层、
  汇总门 `gates-summary`）——**没有 release 相关工作流**。
- 许可证：`MIT OR Apache-2.0`（M0 脚手架带入）。
- **发布物是什么并不显然**：引擎是 workspace 内多个 crate；浏览器侧的 wasm 桥在
  **另一个仓库**（`physarena/wasm-bridge`，非 git）⇒ 本仓能发布的只有
  **tag + Release 说明 + 门禁产物**，除非另外决定"发布 crate 到 crates.io"。

## 2. 版本号规则（**需你选一个**，这是唯一的阻塞项）

| 选项 | 形式 | 适用 | 说明 |
|---|---|---|---|
| **A（推荐）** | **`v0.1.0-m1`** | 与里程碑对齐 | 本仓的进度语言是 **M0 / M1 / M2**；版本带里程碑后缀能直接对上"这一版到哪" |
| B | `v0.1.0` | 纯语义化版本 | 简单，但看不出里程碑 |
| C | `v0.1.0-2026.09.21` | 日期式 | 适合高频快照；与前两者不互斥，可作预发布号 |

**预发布语义**：`-m1` / `-2026.09.21` 都会被 git/gh 视为**预发布**（prerelease），
正式版就是 `v0.1.0`——这与"M1 尚未全部达标"的现状相符（见 `OPEN-PROBLEMS.md`
的 P2/P3/P4 与 T5 列）。

## 3. 发版门（打 tag 前必须全绿；与仓内纪律一致）

1. **五项门禁链**：`cargo fmt --all`、`cargo test --release`、
   `cargo clippy --workspace --all-targets -- -D warnings`、
   `bash scripts/vocab_scan.sh .`、`/c/vxl-wl-tools/typos.exe .`（输出重定向 + 显式 `$?`）。
2. **行为门（ADR 0004）**：`m0_gates`（门槛 + 压力哈希）、`determinism`、
   `default_tier_stability`（四个冻结值 + 金丝雀）。
3. **保真门**：金样三场景（`docs/RECIPES.md` 金样段；**注意用当前参考配方**）。
4. **外部对照**：arena 自检 `19 pass / 0 degraded / 0 fail`（本机可跑，见 `RECIPES.md`
   的"浏览器对拍自检"节）。
5. **若这一版包含行为改变**：按 ADR 0004 在落地记录里**显式写出新旧哈希与原因**。

> `release.yml` 会自动跑其中的**可自动化部分**（1 + 2 的门槛/确定性 + 3），
> 并把结果附到 Release；**arena 与人工判定不进 CI**（它需要在带浏览器的环境跑）。

## 4. 发版流程（照做即可）

```bash
# ① 在干净的工作区、且在正确的分支上（当前开发线 = feat/m0-gates）
git status --short          # 必须为空（协作时尤其注意：别把别人的在制品带上）
git log --oneline -1

# ② 本地过 §3 的 1–3（或信任 CI）
cargo test --release && cargo clippy --workspace --all-targets -- -D warnings

# ③ 打 tag（以选项 A 为例）并推送 —— **这一步就是"发布"，需你确认**
git tag -a v0.1.0-m1 -m "M1 快照：<一句话>"
git push origin v0.1.0-m1

# ④ 之后由 `.github/workflows/release.yml` 自动：跑门 → 建 GitHub Release（自动生成说明）
gh release view v0.1.0-m1 --web
```

**回退**：`git push --delete origin <tag>` + `gh release delete <tag>`（公开过的 Release
可能已被抓取，删除不等于没人见过——所以打 tag 前先确认）。

## 6. ✅ 首次发版实测：门红过，根因是**一处未提交的格式修复**（2026-09-21）

打 `v0.1.0-m1` 后 `release` workflow 在 `fmt` 步失败（15 秒），`ci` 在同一提交上同样红：

```
Diff in .../crates/vxl-phys-narrow/src/simd.rs:133
-        // SAFETY: …（8 空格）      ← **提交里**的形态：不合规
+                        // SAFETY: …（24 空格）  ← rustfmt 要的形态
```

**⚠️ 我最初的诊断是错的，记录在此以免后人重蹈**：我先归因为"本仓没有
`rust-toolchain.toml` ⇒ 本地/CI 的 rustfmt 版本不同"。**实测否证**：

- 本地 `rustc 1.97.1` 的 rustfmt 对**提交版** `simd.rs` **同样**要求 24 空格，与 runner **一致**
  ⇒ **不存在版本差**；
- 真正的差别只是"**提交里是 8 空格、工作区里已是 24 空格**"——本地
  `cargo fmt --all --check` 之所以为 0，是因为它验证的是**已修好的工作区**，不是提交。
- 这个坑值得记：**在"工作区"上跑门禁，得到的是工作区的判定**；要判"提交是否合规"，
  必须对 `git show HEAD:<file>` 的产物跑工具（本档用的就是这个办法）。

**修法（已落地）**：那 4 行是**并发协作者工作区里未提交的编辑**；验证它与
`rustfmt --edition 2021` 对提交版的输出**逐字节一致**（纯机械、零人工成分）后**代其落盘**
（`style(narrow)` 提交，含归属说明）。⇒ **CI 转绿 ⇒ 发版门可通过**。
**不需要加 `rust-toolchain.toml`**（它不修这个问题，还会无端给全仓钉工具链）；
"是否固定工具链"仍可作为独立议题，但**不是本次红的成因**。

### 首发已完成（2026-09-21）

**`v0.1.0-m1` 已发布**：<https://github.com/bfxh/RUST-WL/releases/tag/v0.1.0-m1>
（`prerelease: true`、`draft: false`、作者 `github-actions[bot]`）。
附产物：`clippy.log` / `determinism.log` / `gold-col45.txt` / `gold-pile5.txt` /
`gold-tower25.txt` / `m0.log` / `test.log` —— **门禁证据随版本存档**。

首发踩到并修掉的两个**工作流自身**缺陷（都记在此以免复发）：
1. **`bash -e` 吞诊断**：`cargo fmt ... ; cat fmt.log` 在 `-e` 下，前者失败即中止 ⇒
   `cat` 永不执行，只看到 "exit code 1"。⇒ 各门步骤改为 `set +e` → 取 `$?` → **先打诊断再 `exit $rc`**。
2. **附空文件会被拒**：门禁全绿时 `fmt.log` 是**空文件**，`gh release create` 附它会
   `HTTP 400: Bad Content-Length`，**整个 Release 被回滚**（现象像"没建成"）。
   ⇒ 只附**非空**文件（`find ... -size +0c`），并让步骤**幂等**
   （Release 已存在时改用 `gh release upload --clobber`）。

**另一个可复用的教训**：`workflow_dispatch` 的手动重跑用的是**默认分支**上的工作流文件，
而本仓的开发在 feature 分支 ⇒ 补建/重跑最实际的办法是**重指 tag**（删+重建+推），
而不是 `gh workflow run`。本档 §4 的流程因此以"重指 tag"为主。

### 与本档流程的关系

- 打 tag **不等于**发布完成：`release` job `needs: gates`，门红 ⇒ **不建 Release**（设计如此）。
- 因此当前状态是：**tag 已推、Release 未建**。等上面第 1 或第 2 项落地（CI 转绿）后，
  用一次手动触发即可补建（工作流支持指定既有 tag）：

```bash
gh workflow run release.yml -f tag=v0.1.0-m1
```

- **回退**（若决定收回该 tag）：`git push --delete origin v0.1.0-m1`。


## 5. 已知的**陈旧项**（发版前顺手修，否则 Release 说明会误导）

- ⚠️ `ci.yml` 的 `gold-sample` job 仍在跑**旧配方** `16 0.01 4 3.0 30 4`（注释写"必须
  substeps=4"），而仓内**参考配方已升为 `16/1/16`**（`RECIPES.md`），且今天实测
  **`16/1/4` 会发散**（`KE 44 万 J`）。该 job 是报告层（`continue-on-error`）不拦门，
  但它的注释与配方都该跟参考配方对齐。
- `Cargo.toml` 的版本是否随 tag 走（0.1.0 → 0.1.0-m1？）**未定**：若不上 crates.io，
  可以只在 tag 上体现，不必改 `Cargo.toml`（改了会连带 `Cargo.lock`）。
