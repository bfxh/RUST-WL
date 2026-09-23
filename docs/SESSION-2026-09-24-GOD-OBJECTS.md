# 上帝对象清零（2026-09-24）——交接档

> 一句话：**文件级已清零**；**函数级门面内剩 29 个**（>120 行；B8+B9 两批已清 6 个）。
> 本档是下一会话的入口（烧量到线就在这儿收尾）。

## 1. 已落地（分支 `feat/solver-limits`，15 个提交 `8c650bd` → `bfa33c8`，CI 全绿）

| 提交 | 内容 |
|---|---|
| `8c650bd` | 门接入本仓：`scripts/god_gate.py` + `god.gate.json` + `god-baseline.json`；`gate_all.sh` 加 `god_selftest`/`god`；CI 静态层同步；`ci_shape_lock.sh` 钉住 |
| `d48fc3f` | narrow 3247 → 最大 671（12 块）；门修两处真 bug（量具虚高 / 可见性误加） |
| `6a7ddf0` | solver 2019 → 最大 614；joints 1007 → 483；拆分器四条新规则 |
| `1817fa7` | phys 2172 → 最大 710；fluid 2073 → 663；门再修两处判据 |
| `1935a6c` `80c2041` | broad 三件 + gjk + voxel + arena_bench + broad/lib —— **文件级清零** |
| `4d00a50` | 测试文件去重 import；示例改**目录式**；门加模块路径硬校验 |
| `dff4f13` | 三套 BVH 重复的 AABB 助手 → 共享 `aabb_util.rs`（**−146 行**） |
| `336ab59` | GPU 三函数：`Packet::new` 310→83、`phases_on_adapter` 293→125、`grid_on_adapter` 247→129 |
| `9a7d1fa` | `.gitignore` 加 `__pycache__/` |
| `d42d29f` | **B8 三个库内单块**：`mass_props` 121→**94**、`grid_on_adapter` 132→**89**、`make_phase_pipes` 125→**95**；新增 `GridPipes`；修 `grid.rs` 一处孤儿文档 |
| `d6124a6` | **B9a**：`clip` 203→**41**（四步方法）、`epa_from_simplex` 132→**89**（三个纯函数）；加 `type BoxAxes` |
| `bfa33c8` | **B9b**：`sat` 178→**74**（`build_axes` / `box_pair_scan` / `axis_extents`）+ `box_axes()` 访问器（`sat`/`clip` 里同一段 match 各抄一遍 ⇒ 合一） |

**验收证据（每批都一样）**：`bash scripts/gate_all.sh` 全绿，且**金样门读数与重构前逐项相同**
——col45 **45/45**、pile5 **2000/2000**、tower25 **2396/2500**，Δpos max **0.0034 / 0.0041 / 0.0950**
（⇒ 纯搬移、行为零变化）。`clippy --workspace --all-targets -- -D warnings` 零告警
（**只看 `cargo build` 会漏**测试/示例）。

## 2. 工具链（都在 ZCode workspace，不进仓）

| 工具 | 用途 / 关键规则 |
|---|---|
| `god_gate.py`（**在仓内**） | 尺寸棘轮。`--list --top N` 排行 / `--write-baseline` 收紧 / `--selftest` 金丝雀。放行两条通道：小涨（≤10%、至少 8 行）**或净账**（函数降幅 ≥ 文件涨幅） |
| `extract_block.py` | 函数级瘦身主力：锚点定位（`start`/`until`/`scope`/`nth`）+ 自动 dedent + `pre`/`prelude`/`tail`/`replace` + 三重自检。**五条边界见 §5** |
| `extract_fn_arms.py` | **match 臂专用**（自动认臂 + 联合模式回填）——臂不要用 `extract_block.py` |
| `cf_census.py` | 控制流普查：`break/continue/return` 归到所属循环，判 Mode M（可机械搬）/ Mode F（上缴） |
| `split_plan.py` / `split_tests.py` / `to_example_dir.py` | 文件级拆分 / 测试模块外移 / 示例改目录式 |
| `dedup_fns.py` | 同名函数去重：**逐字比对后**才提取，不一致整批中止 |
| `fn_blocks.py` | 看块边界（辅助定锚点） |

## 3. 余项：**门面内 29 个**函数 >120 行（另有 `gold-sample/src/main.rs::main` 335 行，在 config 排除面内）

**计数口径（重要）**：门只报**每文件最大的那个**函数 ⇒ 按文件数会**低估**（旧版本档写"35 个"，实际当时
是 36；`sat.rs::sat` 178 行就是这样被漏掉的）。按函数的数法（可复核）：

```bash
python - <<'PY'
import sys, pathlib; sys.path.insert(0, "scripts"); import god_gate
root = pathlib.Path(".")
for p in sorted(root.rglob("*.rs")):
    if "target" in p.parts: continue
    for name, n, kind in god_gate.brace_metrics(p.read_text(encoding="utf-8")):
        if kind == "fn" and n > 120: print(n, p, name)
PY
```

| 类 | 个 | 函数 | 配方 |
|---|---|---|---|
| 库内**大分发** | 6 | `process_pair_shaped` 663（narrow）· `solve_phase` 551（solver）· `build_constraint` 340 · `compute_pairs` 256（broad）· `solve_joint` 214 · `solve_island_group` 210 | **Mode F**：`cf_census.py` 普查 → arm/段提成同型 helper，调用点 `if helper(..) { return; }` |
| 库内**单块** | 1 | `contacts_box_voxel` 174（terrain/voxel） | 见下 |
| **示例 main** | 19 | `trimesh_rest_probe` 360 · `showcase` 344 · `gpu_density_probe` 272 · `gpu_tick_probe` 266 · `m1_scale` 263 · `splat_rest_probe` 249 · `fluid_buoyancy` 245 · `arena_bench/probes_a::bench` 224 · `m1_pile` 205 · `diag_min` 188 · `gpu_grid_probe` 183 · `trimesh_escape_probe` 181 · `m1_islands` 178 · `m0_gates` 174 · `diag_eject` 149 · `m1_collision_row` 138 · `diag_col` 135 · `voxel_rest_probe` 129 · `dam_break` 122 | `extract_block`：场景构造 / 推进 / 报表 三段 |
| 测试 + 脚本 | 3 | `float_motion_vs_local_water_motion` 144 · `diag_query_cost_across_tree_shapes` 137 · `render_demo.py::main` 241 | 同上 |

**建议切点（已侦察）**：

1. `contacts_box_voxel`（174）：提格收集（L40-55）、`sd` 闭包（L57-81 → 顶层 `fn sd_over_cells`）、
   逐面采样（L106-133 → `([usize; 6], [f32; 6])`）、逐面发射（L159-）。采样/发射段要的固定量多
   （`v`/`cells`/`m`/`h`/`d3`/`pos`/`skin`）⇒ 先定义一个 `struct BoxQuery<'a>` 收拢（**别硬塞 8 个参数**，
   clippy `too_many_arguments` 会拦）。
   ⚠️ **顺带发现一处死代码**：L134-149 的"主导面选择"结果 `best` 已无人用（`let _ = best;`），
   `deepest` 也只喂那条判据 ⇒ 整段可退化成 `if count.iter().all(|&c| c == 0) { return false; }`
   （语义等价：全零时发射循环本来也什么都不发）。**单独一次改动做，别混进纯搬移批**。
2. `solve_island_group`（210）：每段本来就有 `ISLAND_SEG_PROBE` 计时壳（t_build / t_warm / t_iter…）
   ⇒ **按段提 helper 最自然**；参数表会很长，照抄现有 `#[allow(clippy::too_many_arguments)]`（热路径）。
3. `solve_joint`（214）：未侦察，同段搬。
4. 示例 `main` 19 个：形态相同（建场景 → 推进 → 报表）⇒ 可先做最大的 `trimesh_rest_probe` 定配方，
   其余照抄（每例一个提交或几例一批）。

**类型级 6 个是"已登记例外"**（`PhysConfig` 31 / `FluidSystem` 26 / `DefaultNarrowPhase` 33 /
`Packet` 27 / `bvh8` 31 / `wide` 29）——纯数据记录，理由在 `god.gate.json` 的 `_type_exempt_doc`：
**只有方法数才是复杂度**。

## 4. 每批的验收顺序（照抄；**顺序别换**）

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings     # 必须 --all-targets
cargo test --release --workspace
python scripts/god_gate.py --root .                        # 门
python scripts/god_gate.py --root . --write-baseline       # 只准减；理由写进提交信息
bash scripts/gate_all.sh                                   # 全量（含金样门：读数必须逐项相同）
git branch --show-current && git status --porcelain        # 漏 add 会出现"本地绿 CI 红"
git push && gh run list --workflow=ci.yml --limit 1        # 核 CI（约 8–9 分钟，别同步等）
```

`gate_all.sh` 会自己抢**机器级锁**（`~/.rx/perf-gate.lock`）——它跑的时候**别并发别的 cargo 构建**
（计时门会脏；金样门在 Windows 上还会撞 `.exe` 锁报 os error 5）。**后台跑它被中断会留下锁**，
下次它会自动回收（`mmin +30` 过期），不用手工删。

## 5. 这批踩过的坑（别再踩）

1. **模块路径**：非根文件（`src/bvh.rs`）的子模块必须在**同名目录**；**示例二进制**的子模块不能是兄弟
   `.rs` ⇒ 先 `to_example_dir.py`。
2. **中段顶层 `use …;` 必须收归根**；**impl 的文档注释要跟着 impl 走**（否则孤儿文档）。
3. **再导出三档**：有 `pub` 条目 ⇒ `pub use`；只有 `pub(crate)` 且真被引用 ⇒ `pub(crate) use`；
   impl-only ⇒ 不导。**孙模块拿不到根的再导出**。
4. **`replace` 是子串替换**（`&dens_b` 会命中 `&dens_bgl`）；**`drop(data)`/`unmap()` 留在调用方**
   且顺序不能反（先 drop 再 unmap）。
5. **回滚要按文件点名**：`git checkout -- <目录>` 会把**已拆好但未提交**的成果一起抹掉。
6. **基线类门按工作区记录**：`git add` 漏文件 ⇒ 基线错位 ⇒ 本地绿而 CI 红。
7. **门的棘轮**：`--write-baseline` **之后不许再动文件**（哪怕只加一行 `///`）——probe.rs 482→483 实栽，
   要么先改完再写基线、要么改完重写一遍（**净账通道**会放行并打印"合法交换"）。

### `extract_block.py` 的五条边界（B8/B9 实测；改工具前先看这段）

1. **插入点必须在搬运区间之前**：`--at-before-fn` / `before_fn` 指到区间**之后**的函数 ⇒
   `lines[at:cstart]` 成空切片，搬走的内容**既留在原地又进了 helper**（被"行数不自洽"抓回）。
   **且 op 是顺序执行的**：前一个 op 搬走的内容会改变后一个 op 的锚点计数
   （B9b 实栽：op1 搬走第一个 `if let Some((aa, ab))` ⇒ op2 的 `nth` 要从 2 改 1）。锚点不匹配时
   工具**拒写**（exit 2，磁盘不动）——先看它报的"只有 N 处"，别急着改文件。
2. **尾表达式不在搬运区间里**：`--sig` 声明了返回值但块内没有 `return`/不是表达式 ⇒ helper 缺尾
   （E0308）。**搬带返回值的块前先看块的末句是表达式还是语句**；用 `tail`（单行）或把 `let` 去掉
   让 `match` 直接当尾表达式。
3. **搬运区间会连调用方的 `break` 一起搬走**：`if bi == usize::MAX { break; }` 进了 helper
   （那里没有循环可 break ⇒ E0268）⇒ 区间**切在逃逸语句之前**，或把逃逸改造成 helper 的返回值
   （B9a 改成 `Option`）。
4. **闭包 → 顶层 fn / 参数换形态**：捕获的 owned 值变引用、`Vec<T>` 变 `&[T]` ⇒ 体内原来的
   `&x`/`&device` 成了多余借用（clippy `needless_borrow`，B8/B9 各栽）。
5. **match 臂不要用本工具**：它把模式行 + 臂花括号一起搬走，helper 里只剩半截臂（mass.rs 实栽）
   ⇒ 臂走 `extract_fn_arms.py`，或把 `match` 外壳一起搬。

**另外两条 clippy 拦路（都在纯搬移里冒出来）**：
- `let` + 立即返回（`let (a, b) = match …; (a, b)`）⇒ `let_and_return`：直接把 `match` 当尾表达式。
- helper 尾部的 `return …;`（搬来后成了函数最后一句）⇒ `needless_return`：去掉 `return` 与其分号。
- 参数超过 7 个 ⇒ `too_many_arguments`：**要么收拢成结构体**（见 `GridPipes`/`BoxQuery`），
  要么按热路径惯例 `#[allow(...)]`（solver 侧已有先例）。
