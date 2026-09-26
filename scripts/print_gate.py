#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""print-gate：禁引擎核心直接写 stdout/stderr（硬判，引擎核心纪律）。

物理核心是确定性、headless 可跑的库；直接 `println!`/`eprintln!`/`print!`/`eprint!`
会污染跑分/回归日志、破坏「核心场景禁用可选 feature 仍可运行」的清洁性，也泄露内部状态。
调试输出应走 tracing/日志门面或在 CLI/bin 层做。tests/examples/benches 允许，故只扫 src/。
存量须为 0。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

PRT_RE = re.compile(r"\b(?:e?print)(ln)?!\s*\(")
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
        for m in PRT_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            hits.append((reln, line))
    if not hits:
        print("✅ print-gate: src 内无直接 stdout/stderr 写（引擎核心应走日志门面/CLI 层）")
        sys.exit(0)
    print("❌ print-gate: src 内发现 println!/eprintln!/print!/eprint!（引擎核心不该直接写终端）")
    for rel, line in hits[:40]:
        print(f"   {rel}:{line}")
    sys.exit(1)


if __name__ == "__main__":
    main()
