#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fastmath-gate：禁 fast-math 内建（硬判，SPEC §5「禁 fast-math」）。

引擎确定性优先：固定步长 + 严格 f32 + 有序归约（§5）；回放与跨平台哈希 bit 级
一致是身份门（§12.1）。Rust 的 fast-math 内建（`core::intrinsics::f*_fast`）会破坏
确定性（重排/FTZ/DAZ 类行为），与 §5「禁 fast-math、sim 路径无 platform intrinsics
重排」直接冲突。本门扫全部 `.rs`（经 Rust 感知掩码，跳过注释/字符串），命中即红。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

# fast-math 内建：fadd_fast / fsub_fast / fmul_fast / fdiv_fast / frem_fast /
# fabs_fast / fsqrt_fast / fceil_fast / ffloor_fast / fround_fast / ftrunc_fast
FAST_RE = re.compile(r"\b(?:f(?:add|sub|mul|div|rem|abs|sqrt|ceil|floor|round|trunc)_fast)\b")


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, git_tracked=False):
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        for m in FAST_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            hits.append((rel, line, m.group(0)))
    if not hits:
        print("✅ fastmath-gate: 无 fast-math 内建（§5 确定性禁令）")
        sys.exit(0)
    print("❌ fastmath-gate: 发现 fast-math 内建，违反 §5「禁 fast-math」（破坏确定性回放/跨平台哈希一致）")
    for rel, line, name in hits[:40]:
        print(f"   {rel}:{line}  {name}")
    sys.exit(1)


if __name__ == "__main__":
    main()
