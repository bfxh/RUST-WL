---
name: 缺陷
about: 记录与预期不符的引擎行为（穿透、抖动、非确定、性能回归）
title: ''
labels: ''
assignees: ''
---

<!-- 标题写中文行动或结果句：一句话说明「什么场景下出了什么错」。 -->
一句话说明错误结果。

<details>
<summary>复现、预期与验收</summary>

- 复现场景（`m0_gates` / `m1_scale` / `gold-sample/` 哪个场景；或最小构造）：
- 复现命令（可直接粘贴，含 release 与参数）：
- 实际结果（原始输出片段，不要转述）：
- 预期结果：
- 环境（平台 / 编译器 / 是否 CI）：
- 验收条件（可判定的数字或哈希，例如「|v| ≤ 0.1 且 m0_gates PASS」）：

</details>

<details>
<summary>确定性与回归信息（能填就填）</summary>

- 是否可稳定复现（每次都一样 / 偶发）：
- 相关哈希（`m0_gates` gate_scene.hash.final / `determinism` FINAL_HASH）：
- 上次已知正常的状态（提交号 / 段号）：
- 已排除的方向（试过什么、为什么无效——负面结果同样要记录）：

</details>
