#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""unwrap-gate：unwrap/expect 计数棘轮（SPEC 未明列，但确定性引擎应少 panic 路径）。

`Result::unwrap`/`expect` 在物理核心里意味着「此处可能 panic」——确定性回放下
panic = 整局作废。存量进基线，只准减，新增即红。
"""
import os
import re
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

_UNWRAP = re.compile(r"\.(unwrap|expect)\s*\(")


def scan(files, root):
    res = {}
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        c = len(_UNWRAP.findall(src))
        if c:
            res[rel] = c
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("unwrap-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
