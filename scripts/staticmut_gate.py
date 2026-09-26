#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""staticmut-gate：禁 static mut（硬判，SPEC §0 模块化「无全局状态」）。

全局可变状态破坏「模块化、无隐式耦合、可独立测试/替换」与确定性复现。
在 #![forbid(unsafe_code)] 下 static mut 本就编不过（实测库存 0），本门作纵深防御——
一旦某 crate 放开 forbid 或误加 static mut，立即红。扫描经 Rust 感知掩码，跳过注释/字符串。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

STATIC_MUT_RE = re.compile(r"\bstatic\s+mut\b")


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, git_tracked=False):
        if not rel.endswith(".rs"):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        for m in STATIC_MUT_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            hits.append((rel, line))
    if not hits:
        print("✅ staticmut-gate: 无 static mut（§0 模块化无全局状态）")
        sys.exit(0)
    print("❌ staticmut-gate: 发现 static mut（全局可变状态），违反 §0「无全局状态」")
    for rel, line in hits[:40]:
        print(f"   {rel}:{line}")
    sys.exit(1)


if __name__ == "__main__":
    main()
