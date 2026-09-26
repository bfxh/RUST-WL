#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""god-gate：单文件行数棘轮（文件名即罪证，超长文件 = 该拆）。

每文件行数进基线，只准减，新增即红（新增文件须显式 --write 重记基线，恰是「加文件
要先想清楚它该多大」的卡点）。行长门（linelen-gate）管横向，本门管纵向。
"""
import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc


def scan(files, root):
    res = {}
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            n = sum(1 for _ in f)
        res[rel] = n
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("god-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
