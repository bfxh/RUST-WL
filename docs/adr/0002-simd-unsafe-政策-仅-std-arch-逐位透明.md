# ADR 0002：SIMD/unsafe 政策——仅 std::arch SSE2、逐位透明、标量孪生

- 状态：**accepted**（2026-09-14，用户在选项 (a)/(b) 中明确批准 (b)）
- 取代：无；被取代：无

## 背景（Context）

M1 约束集原本 `forbid(unsafe_code)`，实测把 T3 窄相（峰 28.68 vs 验收 25）与解算
优化全部压到「标量算术 + 访存」地板。用户裁决：放开 unsafe/SIMD（岛内并行 +
向量化，估 −2~4×），但**仍然只用 Rust**（`std::arch`，无 C/C++/汇编）。
同时本仓有一条硬不变量：**确定性哈希门禁**（m0_gates/stress/determinism 四哈希
逐位一致）——向量化最常见的破坏方式恰恰是改变浮点求值顺序（lane 内 dot 的
加法次序、`abs`/比较的实现）。

## 决定（Decision）

1. 范围：仅 `std::arch::x86_64` 的 SSE2 基线指令集（编译器保证存在，不做运行时
   dispatch）；每处 unsafe 以**模块级** `#[allow(unsafe_code)]` 圈定，
   crate 根仍 `#![deny(unsafe_code)]`。
2. **逐位透明是验收的一部分**：每个 SIMD 内核必须配一个标量孪生函数
   （`sat_scan_scalar`），并由测试 `simd_matches_scalar_bitwise`
   （4000 组正交基盒对逐位比对）与 `simd_tail_lanes_are_ignored`
   （尾 lane 无效值不泄漏）守门。位透明规则具体到写法：dot 固定为
   `add(add(mul,mul),mul)`，`abs` 用符号掩码（`_mm_andnot_ps`），标量侧复刻同一规则。
3. 先量化后开工：候选路线必须给出**实测上界**。解算侧两条候选
   （同岛 4 点 4-lane、SoA+单岛向量化）经量化净收益 ≤10%，**不开工**（见
   EXPERIMENTS.md 解算区）——本政策放开的不是「想上就上」，是「量到值得才上」。

## 后果（Consequences）

- 正面：T3 窄相 p95 25.19→**23.30ms**（≤25 ✅ 落地），四哈希逐位不变；
  x86_64 之外的平台会编译失败/退化——本引擎当前只承诺 Windows x86_64。
- 负面：unsafe 面积永久存在，靠「孪生 + 逐位测试」维持；新增 SIMD 必须重复这套
  流程（成本每次 ~半天），不能只写 intrinsics 不写孪生。
- 明确放弃：跨平台可移植的 portable SIMD（`std::simd` 未稳定）、运行时特性派发。
