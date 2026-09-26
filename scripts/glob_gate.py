#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""glob-gate：glob 导入 `use ...::*` 计数棘轮。

glob 导入把一整片名字空间灌进作用域，遮蔽「谁真正依赖谁」，也让重构时死导出难查。
存量进基线，只准减，新增即红。
"""
import os
import re
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

_GLOB = re.compile(r"\buse\b[^;{}]*::\*")


def scan(files, root):
    res = {}
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        c = len(_GLOB.findall(src))
        if c:
            res[rel] = c
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("glob-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
