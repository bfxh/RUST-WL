#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""clone-gate：.clone() 计数棘轮（性能纪律，物理核心 SoA/所有权优先）。

物理核心走 SoA + 所有权（§0 模块化、§2 批量积分），`.clone()` 在热路径是隐性拷贝开销、
确定性回放里也须谨慎。存量进基线（实测 4，全在 vxl-phys-narrow），只准减，新增即红。
tests/examples/benches 不计（排除兄弟目录）。
"""
import os
import re
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

_CLONE = re.compile(r"\.clone\s*\(")
SRC_EXCL = re.compile(r"(?:/tests/|/examples/|/benches/|/src/bin/|^tests/|^examples/|^benches/)")


def scan(files, root):
    res = {}
    for rel in files:
        reln = rel.replace("\\", "/")
        if not reln.endswith(".rs") or SRC_EXCL.search(reln):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        c = len(_CLONE.findall(src))
        if c:
            res[reln] = c
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("clone-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
