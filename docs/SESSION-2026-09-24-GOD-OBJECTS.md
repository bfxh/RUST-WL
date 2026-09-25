# 上帝对象清零（2026-09-24）——交接档

> 一句话：**文件级已清零**；**函数级门面内剩 2 个**（`process_pair_shaped` 663 / `solve_phase` 551；**其余全清**——示例 main、脚本、broad 与 solver 侧均已达标，B18–B25 共十七件；实测口径见 §3）。
> 本档是下一会话的入口（烧量到线就在这儿收尾）。

## 1. 已落地（分支 `feat/solver-limits`，25 个提交 `8c650bd` → `b3ac4fd`，CI 全绿）

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
| `81432f3` | **B10a**：`contacts_box_voxel` 174→**99**（`gather_cells` / `sd_over_cells` / `face_samples`）+ 采样循环与发射循环的采样点推导**去重** |
| `bcf5cb5` | **B10b**：`solve_island_group` 210→**79**（按 `ISLAND_SEG_PROBE` 计时段拆五个 helper） |
| `981cbcf` | **B11**：示例探针重复段收口 —— `dam_break` 122→**101**、`voxel_rest_probe` 129→**102**（`settle_on_floor` 收三组同形探针） |
| `c3905ee` | **B12**：`solve_joint` 214→**38**（五段横幅各一 helper；累加器 `max_dv` 改值进值出） |
| `d211afa` | **B13**：`diag_col` 135→**100**（`ke_of` 闭包提成同名 fn——调用点一字未动）、`m1_collision_row` 138→**106**（`run_ticks` 用类型别名避长元组 + `finish`） |
| `2d46461` | **B14**：`diag_eject` 149→**97**（`ke_of` / `dump_body_manifolds` / `print_cum_top5`） |
| `dbd21bb` | **B15**：`m1_islands` 178→**80**（99 行"岛并行拆分读数"整块提 `report_island_breakdown`） |
| `5fa7f49` | **B16**：`diag_query_cost_across_tree_shapes` 137→**57**（三棵**嵌套 fn** 用 `hoist` 上提模块级） |
| `b3ac4fd` | **B17**：`float_motion_vs_local_water_motion` 144→**114**（两档重复的"零冲击就位"⇒ `surface_height(couple)`） |
| `402e2c0` | **B18**：示例 main 前两件（最大的两个）—— `trimesh_rest_probe` 360→**60**（七段提纯为模块级 fn，`end_state_report` 顺手删死参数 `cfg`）、`showcase` 344→**56**（六域建景各一 fn + `Scene` 句柄包 + 转储拆 `write_header`/`write_body_records`/`write_voxel_bits`/`write_fluid_particles`/`write_frame`）；判据=**逐位 A/B 对拍**（见 §3 第 4 条） |
| `ba007bb` | **B19**：示例 main 再清三件 —— `m1_scale` 263→**79**（`Args`/`Acc` 记录 + `run_ticks`/`print_tick_diag`/`report_working_set`）、`splat_rest_probe` 249→**70**（②③④⑤ 各一段 fn + `flat_field_world` 收四次同形建世界）、`fluid_buoyancy` 245→**70**（`add_tank`/`pour_water`/`spawn_density_boxes` 收 2a/2b 逐字重复段 + `Run`/`Run2b`）；判据=**掩计时后全字对拍**（三份 `norm_*.py`：89/57/25 行 IDENTICAL） |
| `19c7c66` | **B20**：示例 main 再清四件 —— `m1_pile` 205→**59**、`diag_min` 188→**85**、`trimesh_escape_probe` 181→**57**、`m0_gates` 174→**75**；判据=**掩计时后全字对拍**（`norm_pile.py`/`norm_diag.py`/原样 `diff`/`norm_m0.py`；m0 的 `arena 容量` 经三次复跑判为**非确定量**后掩掉——见 §5 第 9 条） |
| `9417718` | **B21**：GPU 示例三件 —— `gpu_tick_probe` 311→**74**、`gpu_density_probe` 272→**101**、`gpu_grid_probe` 183→**111**；判据=`norm_tick.py`/`norm_density.py`/`norm_grid.py`（掩计时行；grid 那件**表哈希 + 逐位比对全等**、tick 那件**漂移表 8 行全等**）——均 IDENTICAL |
| `3b9e252` | **B22**：arena_bench 两件（**示例 main 到此全清**）—— `probes_a::bench` 224→**109**（`measure_steps`/`track_exits`/`collect_stats`/`report_spread`/`report` + `Stats` 记录）、`probes_b::scene_joint_chains` 131→**8**（`hinge_chain`/`hanging_tower` 两段各一 fn）；判据=`norm_arena.py`（wall）/ `norm_jc.py`（joint_chains）——均 IDENTICAL |
| `ce695d0` | **B23**：`scripts/render_demo.py` 241→**60**（**脚本类清零**）—— `parse_args`/`load_fonts`/`voxel_faces`（含面表缓存）/`add_{voxel,mesh,splat,fluid,body,shadow}_prims`/`draw_background`/`paint`/`overlay`（逐域一函数）；判据=**渲染产物逐位相同**（GIF sha256 + 50 张 PNG 合集 sha256 两侧一致） |
| `a5a1665` | **B24**：`compute_pairs` 256→**76**（`broad` 的 trait 方法；**是 Mode M 而不是 Mode F**——五段都只访问 `self` 上的缓冲、无逃逸控制流 ⇒ 各提一个私有方法：`detect_sleep_flip`/`update_aabbs`/`update_proxies`/`query_chunks`/`refresh_candidates`/`collect_pairs`/`compact_arena`；顺带把两段重复的块数计算收成 `query_chunks`）；判据=`m1_scale` 掩计时对拍 **89 行 IDENTICAL**（tree_h/候选列全等）+ `cargo test -p vxl-phys-broad` **33 项全绿**（含 `bvh_matches_grid_across_frames` 跨帧配对集合一致） |
| `本批`（哈希下批回填） | **B25**：`build_constraint` 340→**104**（solver 侧；**同样是 Mode M**——逐点循环各段只依赖循环内局部量 ⇒ 提 `material_pair`/`match_warm_point`/`resolve_anchor`（+`AnchorGeom`）/`contact_masses`/`soft_contact`/`tangential_drift` 六个 helper）；判据=`arena_bench wall` 掩计时对拍 **0 行差异**（**warm 匹配行逐字相同**：精确特征 557060 / 近邻回退 27711 / 未匹配 134467）+ `cargo test -p vxl-phys-solver` 12 项绿 |

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

## 3. 余项：**门面内 2 个**函数 >120 行（`process_pair_shaped` 663 与 `solve_phase` 551——narrow 与 solver 的两个最大分发；**其余全清**。另有 `gold-sample/src/main.rs::main` 335 行，在 config 排除面内）

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
| 库内**大分发** | 2 | `process_pair_shaped` 663（narrow）· `solve_phase` 551（solver） | **先按 Mode M 试**（B24/B25 的实测教训）：`compute_pairs` 256→76 与 `build_constraint` 340→104 **都不是 Mode F**——只依赖 `self`/循环内局部量、无逃逸 ⇒ 提私有方法或模块级 helper 即可。**只有确认有逃逸控制流**（`break`/`continue`/`return` 指向被搬区间之外的循环）才上 `cf_census.py` + `Step` 枚举（Mode F 配方：arm/段提同型 helper，调用点 `if helper(..) { return; }`；**多段累加器改值进值出**，B12 的 `max_dv` 范例） |
| **示例 main** | **0** | — | **已全清**：B11/B13–B15 六件 + B18 两件 + B19 三件 + B20 四件 + B21 三件 + B22 两件 = 二十件（含探针/示例目录式与 arena_bench 的子模块函数）。配方与判据见 §3 第 4 条 |
| 测试 + 脚本 | **0** | — | **已清零**（B23：`render_demo.py::main` 241→**60**；配方 = 按域提 `add_*_prims` + `voxel_faces`/`paint`/`overlay`，判据 = **渲染产物 sha256**（GIF 与 PNG 帧合集）——比打印读数更强） |

**建议切点（已侦察）**：

1. `compute_pairs`（256，broad 的 trait 方法）：五相位 + 探针计时（`0)` AABB / `0.5)` 睡眠翻转检测 /
   `1)` 代理更新 / `2)` 清醒动体查询 / `3)` 精确过滤 + 排序去重）⇒ 每个 `// N)` 段一个 helper；
   缓冲都在 `self` 上 ⇒ 仍是方法（签名长，按热路径惯例 allow）。
2. **已做**（B12）：`solve_joint` 214→38（五段横幅各一 helper）。留下的可复用招：
   横幅段提 helper 时**讲顺序/纪律的注释留在调用方**（那是调用序的约束）；段内的累加器
   （`max_dv`）改成**值进值出**；裸 `if` 段用 `scope: inner` + `replace_outer`（`pre` 补 `if` 行、
   `tail` 收括号 —— 但 `tail` 只有一行，"收括号 + 返回累加器"要分两步，返回那行手补）。
3. **B10a 留下的死代码**：`contacts_box_voxel` 里"主导面选择"那一段的结果 `best` 已无人用
   （`let _ = best;`），`deepest` 也只喂那条判据 ⇒ 整段可退化成
   `if count.iter().all(|&c| c == 0) { return false; }`（语义等价：全零时发射循环本来也什么都不发）。
   **单独一次改动做，别混进纯搬移批**。
4. 示例 `main`：形态相同（建场景 → 推进 → 报表）⇒ **B18–B20 已定配方**（做完最大的九件）：
   `trimesh_rest_probe` 360→**60**、`showcase` 344→**56**（六域建景各一 fn + `Scene` 句柄包；
   转储按 头/体记录/体素位/流体粒/帧 拆五个 `fn(&mut impl Write, ...)`）；
   `m1_scale` 263→**79**（`Args`/`Acc` 两个记录结构 + `run_ticks`/`print_tick_diag`/
   `report_working_set`）、`splat_rest_probe` 249→**70**（②③④⑤ 各一段 fn +
   `flat_field_world` 收四次同形建世界）、`fluid_buoyancy` 245→**70**（`add_tank`/`pour_water`/
   `spawn_density_boxes` 收 2a/2b 逐字重复段 + `Run`/`Run2b` 句柄包）；
   `m1_pile` 205→**59**（`Args` + `Stats`（含抖动审计状态）+ `run_ticks`/`report_summary`/
   `report_gaps`/`print_verdict`）、`diag_min` 188→**85**（⚠️ **参数解析与建景留在 main**——
   见 §5 第 10 条的"胶水成本"；只提 `run_ticks`（统计 + 打印就地）+ 两个报表）、
   `trimesh_escape_probe` 181→**57**（【A】落点/【B】尺寸扫描/【D】球内点/【C】迭代预算 各一段 fn）、
   `m0_gates` 174→**75**（`GateEval`/`StressEval` 两个评估结构 + `eval_gate`/`print_gate_report`/
   `run_stress`/`print_stress_report`/`build_json`/`print_verdict`）；
   **B21 三件 GPU 示例**：`gpu_tick_probe` 311→**74**（`Args`/`FluidSetup`/`GpuTiming`/`Report`
   四个记录 + `parse_args`/`build_fluid`/`build_packet`/`run_drift`/`time_cpu`/`time_gpu`/`report`）、
   `gpu_density_probe` 272→**101**（**CPU 参考实现整块提 `cpu_reference(&FluidSystem)`**——
   只吃一个参数最省胶水；`ukey`/`ulp`/`cmp3` 提模块级；`Cmp` 结构 + `compare`/`report`）、
   `gpu_grid_probe` 183→**111**（`cmp_u32`/`bins_from_table`/`hash_of` 提模块级 +
   `TableCmp`/`Scene` + `report`；打印那次**闭包不能直接用**——提到模块级要改签名）；
   **B22 arena_bench 两件（示例 main 全清）**：`probes_a::bench` 224→**109**（`measure_steps`
   （采样）/ `track_exits`（长跑 + 离场追踪）/ `collect_stats`（汇总读数 → `Stats`）/
   `report_spread`（分布 + 极值）/ `report`（规模/相位/窄相/承载力/warm/细分）；`bench` 本体只剩
   12 行编排）、`probes_b::scene_joint_chains` 131→**8**（`hinge_chain`/`hanging_tower`
   两段各一 fn——**同形重复段收口**的又一例）。**示例类到此全清**。
   **B23 `scripts/render_demo.py`** 241→**60**（Python，门用 `ast` 量 ⇒ **不需要编译**）：
   `parse_args` / `load_fonts` / `voxel_faces`（面表缓存留在 main 那两行）/ `add_voxel_prims` /
   `add_mesh_prims` / `add_splat_prims` / `add_fluid_prims` / `add_body_prims` /
   `add_shadow_prims` / `draw_background` / `paint` / `overlay`——**逐域一函数**，main 只剩
   每帧编排。**判据 = 渲染产物逐位相同**（`--src showcase_a.bin`：GIF sha256 与 50 张
   PNG 帧的合集 sha256 两侧一致）——这类"产物型"脚的判据比打印读数更强，可照抄。
   **示例/探针的改写多半是"等价改写"**而不是纯搬移（多个 for 里各写一遍同样的
   "建世界 → 落体 → 打印"⇒ 收成一个收闭包的 helper）⇒ 判据**除门链外再加一条 A/B 对拍**：
   `git stash` 回旧版、同一口径各跑一遍，**读数逐字相同**。B11 验的是打印读数（`diff`）；
   showcase 是二进制 dump ⇒ **逐位对拍**（`cmp_showcase.py` 逐字段解析两侧 dump，只掩帧内 `ms`）；
   计时类输出（m1_scale / splat / fluid）⇒ **只掩计时字段、其余浮点读数全字比较**
   （`norm_m1.py` / `norm_splat.py` / `norm_fluid.py`，都在 workspace）。抽判据时的确定性列清单：
   m1 取 `tree_h/候选/岛/流形/点/组数/contacts/awake/NaN/deep` + 工作集模型值 + `wake_flips`。
   ⚠️ **掩码必须吃掉计时字段的整段宽度**：`{dt:8.1}` 这类右对齐字段在耗时位数变化时前导空格
   数也会变 ⇒ 只替换数字会留空格差、报假红（实测踩过一次，`\s+\d+\.\d+` 才干净）。
   注：`add_fluid` 返回 `usize`（索引）而**非** `BodyId`；`add_mesh` 前取 `providers().len()`
   才是 provider id。注：Mimosa 拦"路径来自参数"的脚本（`open(sys.argv[1])`）⇒ 对拍脚本
   改用**常量路径 + 父目录校验**。

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
# ⚠️ 判据 =「最新 run 的 headSha == HEAD」且 success：连推两次会取消旧 run（见 §5 第 8 条）
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
8. **连推两次 ⇒ 旧 CI run 被取消**（B18/B19 实测）：concurrency 会取消**进行中**的同名 run
   ⇒ 看到"前置门 cancelled + 汇总 failure"**不是真失败**，是自家后续 push 干的（`gh run view <id>`
   里失败门名字是 `cancelled` 而不是 `failure`）。**核 CI 的正确判据**：`gh run list --limit 1` 的
   `headSha` **必须等于当前 HEAD**、且该 run 绿——被取消的旧 run 只代表"当时那次 push"。
   `--exit-status | tail` 管道会把退出码变成 `tail` 的 ⇒ **别用退出码判 CI**，看 run 的 conclusion。
   **省一次 push**：文档里的提交哈希回填**随下一批提交**（滞后一拍），别为回填单独推一次。
9. **判据字段要先验"它确定吗"**（B20 实测）：`m0_gates` 的 `arena 容量` 同一二进制连跑三次读到
   8240/8256/8240（并行 arena 扩容受时序影响）⇒ 拿它当对拍判据会**假红**；真判据是
   `state_hash`（三次逐位相同）。**做法**：对拍前先连跑同一二进制 2–3 次，只把**三次一致**的量
   当判据（本批脚本都在 workspace：`norm_pile.py`/`norm_diag.py`/`norm_m0.py`，escape 那件直接 `diff`）。
10. **拆函数的"胶水成本"会吃掉门的净账**（B20 实栽，`diag_min` 首版被判红）：14 字段 `Args` +
    16 字段 `TickStat` ⇒ 文件 202→340（+138）而最长函数只降 103 ⇒ 门按"净账为正"判红（§4 的门规则）。
    **配方**：参数解析与建景**留在 main**（14 个位置参数不值得包结构体），只提"主循环 + 报表"；
    逐 tick 的统计量用**局部变量 + 就地打印**（不建结构体）⇒ 文件 234（+32）、函数 188→85 ⇒ 放行。
    **纪律**：拆完**先跑 `god_gate`** 再 fmt/clippy——别把"净账为正"留到收尾才发现。
11. **`cargo fmt --all` 之后又改文件 ⇒ gate_all 会在 fmt 步停**（B20 实测）：它自动修复后要求重跑，
   而**自动修复也改了源码** ⇒ 对拍要**再验证一次**（本批 diag_min 重验后仍 IDENTICAL）。
12. **GPU 探针的判据要挑"不敏感"的那几个**（B21 实测）：`gpu_density_probe` 的「**最大 ulp 差**」
   跨版本会变（`1148846126 → 1148846120`，±6），而**最大绝对差 / 逐位相同数 / >2ulp 计数全同**
   ——前者是**近零粒子的噪声敏感极值**（ukey 跨零映射给出 1e9 量级），受编译器重排影响，
   按 `PLAN-gpu.md` §9.2 的"量化容差口径"**不是语义判据** ⇒ 对拍掩掉它、保留后三者。
   另：计时字段既有 `{:.3}` 浮点也有 `{:.0}` **整数**（`壁钟 4928 ms`、`一次性 setup 584 ms`、
   `GPU **0.057 ms/轮**`）⇒ **掩码要把整数形态一起掩**（B21 为此连修两次脚本）。
   另：`PacketCfg` 是 `Copy` ⇒ `pc.clone()` 会被 clippy `clone_on_copy` 判红（B21 实栽）。
13. **右对齐字段的宽度差又栽一次**（B22，第三次）：`arena_bench` 的 `等效 {:>6.0} FPS`、
   `求解器内部：… {:>8.1} µs` 在数值位数变化时前导空格数不同 ⇒ 掩码**必须吃掉前导空格**
   （`\s*\d+(\.\d+)?`），只替换数字会报假红（B19/B21 各记过一次——**这条已第三次踩，写死在
   所有 `norm_*.py` 里了**）。
   另：`Stats` 这类**含 `Vec` 的记录不能按值解构**（`let Stats { .. } = *s;` 会 move 报错）
   ⇒ 要么逐字段拷（`let p50 = s.p50;`），要么把 `Vec` 字段拆成独立参数。
14. **`solve_phase`（551，最后两个之一）的拆分设计 —— 2026-09-25 探查后未做完，已回滚留档**：
    五段分界清楚（头 ~30 行参数推导 / 分岛 / 岛桶 / gather / 解算 / scatter+warm / 休眠判定），
    但**不能按段直接提函数**——解算段要传 6 组"按组缓冲"（`group_lv`/`group_av`/`group_iw`/
    `group_im`/`build_bufs`/`warm_outs`）**加** `local_of`，光参数列表就 18–24 行
    ⇒ **辅助函数会因此自己超 120**（实测：把段 3–4c 整段提出去后 `solve_phase` 降到 200 行，
    而新方法仍有 ~130 行，仍不达标）。
    **正解 = 先做结构打包**：`struct GroupBufs { lv, av, iw, im, build, warm_out, local_of }`
    （7 字段；`island_*` 与 `warm_index` 可另设 `WarmTables`）——把 take/restore 与传参都收成
    一个参数，各段方法才装得下。两条硬约束：
    ① **四个计时点（`t_island`/`t_fill`/`t_solve`/`t_sleep`）必须留在 `solve_phase`**
    （`fill_us`/`island_build_us`/`scope_us`/`scatter_us` 的分界依赖它们；B24 已踩过同型——
    `t_fill` 夹在"分岛"与"填桶"之间，把两段合成一个函数就会丢分界）；
    ② `islands` 来自 `self.island_pool` ⇒ 只能 `mem::take` 出来用（才能与 `&mut self` 的其它
    字段共存），所以"整段解算"要由**一个方法**持有 take/restore，而不是散在各段里。
    已试过且可编译/god 门放行的两块（回滚前状态）：`build_union_find` + `fill_island_buckets`
    （分岛与填桶，中间夹 `t_fill`）、`gather_groups`/`solve_groups`/`scatter_groups`/
    `commit_warm_slots`/`sleep_pass`(+`_island`/`_subisland`)——**下个会话从 `GroupBufs` 起步即可**。
    ⚠️ 另记一条操作教训：本件折腾中出现过一次"Edit 误把 `sleep_pass_island` 的
    `sleep_timer=0`/`sleep_resets+=1` 两行换成空调用"的**真实逻辑破坏**（被 diff/编译抓回）
    ⇒ **长会话里每步改完立刻编译 + 用 `rg` 复核关键行**，别攒到最后。

### `extract_block.py` 的九条边界（B8/B9/B10 实测；改工具前先看这段）

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
6. **注释当锚点会撞上下一个花括号**（B10b 实栽）：`scope` 从锚点行起找"第一个含 `{` 的行"——
   `// 收集 warm 更新…` 后面紧跟 `if let Some(t) = t_it {` ⇒ 只搬走 1 行、**自检还全过**
   （它确实是合法块）⇒ 靠编译器抓回（`cannot find value t`）。**判据**：锚点要么本身是块首行，
   要么它到目标块之间没有别的花括号。**遇到就 `git checkout -- <该文件>` 重做，别手工修那半截**。
7. **`scope: inner` + `replace_outer` 会连循环变量一起搬走**：`for c in cbuf.iter() { … }` 的体用 `c`
   ⇒ helper 里 `c` 不存在（E0425）。体要用循环变量时**保留循环、helper 收单条**
   （B10b 改成 `collect_warm_update(c, …)` 逐条调用）。
8. **插入点会带走"文档 + 属性"两样**：落在 `pub(crate) fn X(` 上时，X 的 `///` 文档**和**
   `#[allow(...)]` 之类属性都会留在原地粘到新 helper 上（B10b 因此报"属性重复 + 22 参数超限"）
   ⇒ 提完要**把文档与属性一起搬回原位**。
9. **参数改名要连体内一起改**（B10b）：签名里把 `settled_hold`/`hold_max_vn` 写成 `rounds`/`max_vn`、
   体内仍用旧名 ⇒ E0425 一串。**照抄原变量名最省事**（helper 形参名 = 体内用的名字）。

**另外两条 clippy 拦路（都在纯搬移里冒出来）**：
- `let` + 立即返回（`let (a, b) = match …; (a, b)`）⇒ `let_and_return`：直接把 `match` 当尾表达式。
- helper 尾部的 `return …;`（搬来后成了函数最后一句）⇒ `needless_return`：去掉 `return` 与其分号。
- 参数超过 7 个 ⇒ `too_many_arguments`：**要么收拢成结构体**（见 `GridPipes`/`BoxQuery`），
  要么按热路径惯例 `#[allow(...)]`（solver 侧已有先例）。
