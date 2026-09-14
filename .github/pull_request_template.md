<!-- 关联任务：写 `docs/M1-PLAN.md 第 N 段` 或里程碑编号（T1~T6）；无关联请说明来源。 -->

关联任务：

<details>
<summary>变更与验证</summary>

- 变更（做了什么、为什么这样做）：
- 验证（**原始输出要点**：命令 + 关键数字 + 结论；不写「已验证」三个字了事）：

</details>

<details>
<summary>确定性影响（本项目提交门的硬指标）</summary>

<!-- 触及引擎行为的改动必须回答；纯文档/工具改动写「不适用」并说明。 -->

- `m0_gates` 哈希：旧 `0x…`（全量 u128）→ 新 `0x…`；或「逐位不变」
- `determinism` FINAL_HASH（10 轮）：
- 若哈希变化：变化原因（算术序 / 分支路径 / 参数默认值 …）与「是否影响金样容差」的判断：
- 新钉板测试：本次改动引入的不变式测试名（无常量则写「无」并说明为何不需要）：

</details>

<details>
<summary>提交门自检（本地等价可跑部分）</summary>

- [ ] `cargo fmt --all --check`
- [ ] `cargo clippy --workspace --all-targets -- -D warnings`（CI 另有强化档）
- [ ] `cargo test --workspace --release`
- [ ] `cargo doc --workspace --no-deps`（`RUSTDOCFLAGS=-D warnings`）
- [ ] `bash scripts/discipline_scan.sh .`（零 unsafe / 零 f64 / 零 SIMD 内建）
- [ ] `bash scripts/vocab_scan.sh .`
- [ ] `cargo deny check`（依赖层：仅在依赖变动时需要）
- [ ] `cargo run --release -p vxl-phys --example m0_gates`（触及引擎行为时）
- 说明：miri / loom / TSan / ASan / 三编译器矩阵 / aarch64 哈希比对**只在 CI 跑**，
  本地跑不了不影响提交，但**不允许**因此宣称「CI 已过」——等 CI 结论。

</details>

<details>
<summary>风险与回退</summary>

- 已知风险 / 负面结果（无效的尝试也要记录，避免后人重走）：
- 回退方式（单提交回退？还是需要附带数据迁移/参数回滚）：

</details>
