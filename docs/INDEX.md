# 文档地图（docs/INDEX.md）

> 一句话：**查决策 → `adr/`；查「试过没用的」→ `EXPERIMENTS.md`；查「还没解决的」→ `OPEN-PROBLEMS.md`；
> 查「怎么想、怎么量」→ `KNOWLEDGE.md`；查「怎么跑」→ `RECIPES.md`；查时间线叙事 → `M1-PLAN.md`。**

整理法（2026-09-14 采用）：决策记录用 Michael Nygard 五段式
（Title / Status / Context / Decision / Consequences，见
[cognitect 博客原文](https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions.html)、
[adr.github.io](https://adr.github.io/)）；实验按「假设/方法/结果/裁决」记否证日志；
决策只增不改，被推翻的新开一条并标注取代（superseded）。

## 按问题找文档

| 你想知道…… | 去哪 |
|---|---|
| **要什么、三轴标准（兼容/性能/真实化）、域矩阵、怎么搞** | **`ROUTE.md`（路线顶层口径，新）** |
| 系统要什么（需求/档位/验收） | `SPEC.md`（V1.3，真源；**§3 性能、§4 真实度档、§6 并发**） |
| 系统怎么搭（crate DAG、步进管线、验收映射） | `ARCHITECTURE-M0.md` |
| M0 交付与门槛实测 | `M0-GATES.md` |
| 某条路线为什么选/不选 | `adr/0001`–`0007` |
| 某个优化为什么没做/被回退 | `EXPERIMENTS.md`（每条有 commit） |
| 当前还开着的问题、下一步入口、验收判据 | `OPEN-PROBLEMS.md`（P3 = 口径 + 重做入口） |
| **求解器结构性重做怎么做（staged/聚类/SoA-SIMD/染色）** | `DESIGN-staged-solver.md` |
| 方法论（能量审计、判定实验、读数陷阱）与量化模型 | `KNOWLEDGE.md` |
| 所有基准/复现的命令行 | `RECIPES.md` |
| 某天到底发生了什么（叙事/排障过程） | `M1-PLAN.md` 各段 + `SESSION-*.md` |

## 文档角色定义（写新文档前先看这里）

- **SPEC.md** —— 需求真源。改它须走裁决链（63/64/65 → V2）。其余文档不得与它冲突。
- **ARCHITECTURE-M0.md** —— 结构事实：模块怎么连、每 tick 跑什么。与代码同步改。
- **adr/NNNN-*.md** —— **决策**（做过选择的地方，含代价与后果）。一份一决策，1–2 页，
  只增不改；被取代时置 superseded 并指向新编号。
- **EXPERIMENTS.md** —— **否证日志**：测过且无收益/负收益的路线。每条 = 假设 / 方法 /
  结果 / 裁决 / commit，一行为主，细节链回 M1-PLAN 对应段。作用：**下一轮勿重做**。
- **OPEN-PROBLEMS.md** —— **未解问题台账**：当前缺口、已排除清单、候选入口、
  验收判据（三边并列）。解决即移入 M1-PLAN 落地段或 adr。
- **KNOWLEDGE.md** —— **可迁移知识**：方法论（怎么设计实验、怎么避免读数陷阱）与
  量化模型（成本结构式子），加文献锚点（Catto GDC 系列、Rapier 文档）。
  这里的东西换一个引擎/里程碑仍然成立。
- **M1-PLAN.md / SESSION-*.md** —— **时间线档案**（append-only 叙事）。新的净收益落地
  仍先写这里，再由 INDEX/OPEN-PROBLEMS/adr 摘引。它越来越长是设计使然，不要往里塞
  结构化索引。

## 当前里程碑状态速览（2026-09-14 第五段：口径落定后，详见 OPEN-PROBLEMS.md）

- M0 ✅（门禁 PASS，哈希门禁在档）。
- **M1 真实缺口（口径已落定）**：SPEC §3 最低通过 = **10 万动态+10 万静态、简单碰撞、
  30 FPS（整 tick ≤33.3ms）**；实测 8B 密堆总均 **330ms ⇒ 差 9.9×**（解算 = 87ms 固定
  + 12.75ms/迭代，降迭代已关闭）。**这不是调参能关的缺口**——要结构性重做
  （staged 语义 + 接触聚类 + SoA/SIMD + 约束图染色）或核「简单碰撞」场景口径
  （V2 原文不在本机，需用户提供）。
- 已达标/已落地：T3（SIMD SAT + 体轴缓存）、T4（5.45× 逐位一致）、T6（p95 0.166ms）、
  T1 两处匹配侧修复（ADR 0003/0007：125 体 dE 峰值 148.9→18.2、pile5 入睡 1195→1700、
  tower 更好；睡眠侧匹配线已关闭——全拒也不睡）、T2 边距速度项 K=6（broad 均 −12%、
  树均 −33%、逐位中性）。
- 决策记录 `adr/0001`–`0007`；**M2 不得启动**。
