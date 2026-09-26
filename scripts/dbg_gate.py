#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dbg-gate：禁 dbg! 残留（硬判，引擎核心纪律）。

`dbg!` 是临时调试打印，确定性物理核心不应有任何遗留输出（会污染 headless 跑分 /
回归日志、破坏「核心场景禁用可选 feature 仍可运行」的清洁性）。tests/examples/benches
里的 dbg! 允许，故只扫各 crate 的 src/。存量须为 0。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

DBG_RE = re.compile(r"\bdbg!\s*\(")
# 排除 tests / examples / benches 兄弟目录（与 src 平级），以及 src/bin 入口
SRC_EXCL = re.compile(r"(?:/tests/|/examples/|/benches/|/src/bin/|^tests/|^examples/|^benches/)")


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, git_tracked=False):
        reln = rel.replace("\\", "/")
        if not reln.endswith(".rs") or SRC_EXCL.search(reln):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        for m in DBG_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            hits.append((reln, line))
    if not hits:
        print("✅ dbg-gate: src 内无 dbg! 残留（调试打印不该进引擎核心）")
        sys.exit(0)
    print("❌ dbg-gate: src 内发现 dbg!（调试打印遗漏）")
    for rel, line in hits[:40]:
        print(f"   {rel}:{line}")
    sys.exit(1)


if __name__ == "__main__":
    main()
