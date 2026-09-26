#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""asm-gate：手写汇编须满足 §7 四律的最小机器判据（结构性硬判，SPEC §7）。

§7：手写汇编仅限 <1% 顶点热点，且须（1）is_x86_feature_detected!/is_aarch64_feature_
detected! 分发 +（2）标量回退 +（3）基准证明优于 intrinsics ≥5% +（4）逐条注释
（意图/寄存器/clobber/平台差异）+（5）跨平台测试，四者缺一即禁止合入。

本门做**最小静态校验**：含 asm!/global_asm! 的 .rs 文件，必须
  (a) 同文件内存在特性检测调用（is_x86/aarch64_feature_detected!）；
  (b) 同文件存在「标量回退 / fallback / scalar」字样注释（asm 在注释/字符串里不计数）。
缺任一项即红。其余三律（基准 ≥5% / 跨平台测试）由人工 + CI bench 把关，本门不覆盖。
注：core::arch SIMD 内建（_mm_* 等）不是 asm!，不在本门范围——§7 第一层明确鼓励。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

ASM_RE = re.compile(r"\b(?:global_asm|asm)!")
DET_RE = re.compile(r"\b(?:is_x86_feature_detected|is_aarch64_feature_detected)!")
FALLBACK_RE = re.compile(r"(fallback|标量回退|scalar\s*回退|回退路径)", re.I)


def main():
    root = gc.repo_root()
    bad = []
    for rel in gc.list_rs(root, git_tracked=False):
        if not rel.endswith(".rs"):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            raw = f.read()
        masked = gc.mask(raw)
        if not ASM_RE.search(masked):
            continue  # 注释/字符串里的 asm! 不算真实使用
        problems = []
        if not DET_RE.search(masked):
            problems.append("缺特性检测 is_x86/aarch64_feature_detected!")
        if not FALLBACK_RE.search(raw):  # 回退注释在原文里找（注释已被掩码抹掉）
            problems.append("缺标量回退注释(fallback/scalar)")
        if problems:
            bad.append((rel, problems))
    if not bad:
        print("✅ asm-gate: 手写汇编均附特性检测 + 标量回退（§7 四律最小校验）")
        sys.exit(0)
    print("❌ asm-gate: 含 asm! 的文件未满足 §7 最小判据（特性检测 + 标量回退）")
    for rel, problems in bad[:40]:
        print(f"   {rel}: {'; '.join(problems)}")
    sys.exit(1)


if __name__ == "__main__":
    main()
