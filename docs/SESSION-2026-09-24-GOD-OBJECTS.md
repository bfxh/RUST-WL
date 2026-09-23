# 上帝对象清零（2026-09-24）——交接档

> 一句话：**文件级上帝对象已清零、去重一处、门已立在 CI 上**；剩下 35 个**函数级** >120 行，
> 下一批从这里接着做。会话烧量已到该拆的线（≈116 MB）。

## 1. 已落地（分支 `feat/solver-limits`，9 个提交 `8c650bd` → `336ab59`，CI 全绿）

| 提交 | 内容 |
|---|---|
| `8c650bd` | 门接入本仓：`scripts/god_gate.py` + `god.gate.json` + `god-baseline.json`；`gate_all.sh` 加 `god_selftest`/`god` 两步；CI 静态层同步；`ci_shape_lock.sh` 加 `need_text` 钉住 |
| `d48fc3f` | narrow 3247 → 最大 671（12 块）；门修两处真 bug（量具虚高 / 可见性误加） |
| `6a7ddf0` | solver 2019 → 最大 614；joints 1007 → 483；拆分器四条新规则 |
| `1817fa7` | phys 2172 → 最大 710；fluid 2073 → 663；门再修两处判据（掩码行延续 / 文档注释归属） |
| `1935a6c` `80c2041` | broad 三件 + gjk + voxel + arena_bench + broad/lib —— **文件级清零** |
| `4d00a50` | 测试文件去重 import；示例改**目录式**（`examples/<name>/main.rs`）；门加模块路径硬校验 |
| `dff4f13` | 三套 BVH 重复的 AABB 助手 → 共享 `aabb_util.rs`（**−146 行**） |
| `336ab59` | GPU 三函数瘦身：`new` 310→83、`phases_on_adapter` 293→125、`grid_on_adapter` 247→129 |

**验收证据**：`bash scripts/gate_all.sh` 每批全绿，且**金样门读数与重构前逐项相同**
（col45 45/45、pile5 2000/2000、tower25 2396/2500；Δpos max 0.0034/0.0041/0.0950）⇒ 纯搬移、行为零变化。
另：`cargo clippy --workspace --all-targets -- -D warnings` 零告警（**只看 `cargo build` 会漏**测试/示例）。

## 2. 工具链（都在 ZCode workspace，不进仓）

| 工具 | 用途 / 关键规则 |
|---|---|
| `god_gate.py`（**在仓内**） | 尺寸棘轮。`--list --top N` 看排行 / `--write-baseline` 收紧 / `--selftest` 金丝雀（4 条） |
| `split_plan.py` | **计划驱动**文件级拆分（每块单独决定是否 wrap impl）。硬校验：**模块路径**（`lib.rs`/`main.rs`/`examples/<name>/main.rs` 的子模块平级；**非根文件**必须在同名目录） |
| `split_tests.py` | 把 `#[cfg(test)] mod tests { … }` 搬成独立文件（去一层缩进；**体内已有 `use super::*;` 就别重复加**） |
| `extract_block.py` | 函数级瘦身：锚点定位 + 自动 dedent + `prelude`（补结构体定义）+ `tail`（追加返回表达式）+ `replace`（**子串替换，注意 `&dens_b` 会命中 `&dens_bgl`**） |
| `dedup_fns.py` | 同名函数去重：**逐字比对后**才提取，不一致整批中止 |
| `to_example_dir.py` | `examples/x.rs` → `examples/x/main.rs`（示例要拆模块时必须先做） |
| `cf_census.py` | 控制流普查：把 `break/continue/return` 归到所属循环，判 Mode M（可机械搬）/ Mode F（要上缴） |

## 3. 余项（35 个函数 >120 行；入口命令见下）

```bash
python scripts/god_gate.py --root . --list --top 40
```

| 类 | 个 | 函数 | 配方 |
|---|---|---|---|
| 库内**大分发** | 6 | `process_pair_shaped` 663（narrow）· `solve_phase` 551（solver）· `build_constraint` 340 · `compute_pairs` 256（broad）· `solve_joint` 214 · `solve_island_group` 210 | **Mode F**：`cf_census.py --target <file>:<fn>` 普查 → 每个 arm/段提成**同型返回**的 helper，调用点 `if helper(...) { return; }`；`return Err(..)` 同型可直接传播 |
| 库内**单块** | 5 | `clip` 203（narrow/sat）· `contacts_box_voxel` 174 · `grid_on_adapter` 132 · `make_phase_pipes` 125 · `mass_props` 121 | `extract_block.py` 直接搬（后三个只超 2–8 行，最便宜） |
| **示例 main** | 19 | `trimesh_rest_probe` 360 · `showcase` 344 · `m1_scale` 263 · `splat_rest_probe` 249 · `fluid_buoyancy` 245 · `arena_bench/probes_a::bench` 224 · `m1_pile` 205 … | 同 `extract_block`：场景构造 / 推进 / 报表 三段 |
| 测试 + 脚本 | 5 | `float_quiet_motion` 144 · `diag_wide` 137 · `render_demo.py::main` 241 | 同上 |

**类型级 6 个是"已登记例外"**（`PhysConfig` 31 / `FluidSystem` 26 / `DefaultNarrowPhase` 33 /
`Packet` 27 / `bvh8` 31 / `wide` 29）——纯数据记录，理由在 `god.gate.json` 的 `_type_exempt_doc`：
**只有方法数才是复杂度**。

## 4. 每批的验收顺序（照抄）

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings     # 必须 --all-targets
cargo test --release --workspace
python scripts/god_gate.py --root .                        # 门
python scripts/god_gate.py --root . --write-baseline       # 只准减；理由写进提交信息
bash scripts/gate_all.sh                                   # 全量（含金样门：读数必须逐项相同）
git status --porcelain                                     # 漏 add 会出现"本地绿 CI 红"
git push && gh run list --workflow=ci.yml --limit 1        # 核 CI
```

## 5. 这批踩过的坑（别再踩）

1. **模块路径**：非根文件（`src/bvh.rs`、`pipeline.rs`）的子模块必须在**同名目录**；**示例二进制**
   （`examples/x.rs`）的子模块不能是兄弟 `.rs`（会被当成独立目标）⇒ 先 `to_example_dir.py`。
2. **中段顶层 `use …;` 必须收归根**（否则别的子模块找不到 `Vec3`）；
   **impl 的文档注释要跟着 impl 走**（否则孤儿文档）。
3. **再导出三档**：有 `pub` 条目 ⇒ `pub use`；只有 `pub(crate)` 且真被引用 ⇒ `pub(crate) use`；
   impl-only ⇒ 不导。**孙模块拿不到根的再导出** ⇒ 跨层共享要 `use crate::x::*;`。
4. **`replace` 是子串替换**（`&dens_b` → `&dens_bgl` 前缀误伤）；**`drop(data)`/`unmap()` 留在调用方**
   且顺序不能反（先 drop 再 unmap）。
5. **回滚要按文件点名**：`git checkout -- <目录>` 会把**已拆好但未提交**的成果一起抹掉。
6. **基线类门按工作区记录**：`git add` 漏文件 ⇒ 基线错位 ⇒ 本地绿而 CI 红。
