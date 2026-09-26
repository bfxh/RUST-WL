#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""linelen-gate：超长行棘轮（>200 列）。

行长必须量原始文本（不过掩码——注释/字符串里的长行同样是真长行，编辑器会折行、
diff 会难读）。存量进基线，只准减；新文件若含 >200 列行即红，正常宽度新文件不拦。
"""
import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

ABS = 200


def scan(files, root):
    res = {}
    for rel in files:
        mx = 0
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            for line in f:
                L = len(line.rstrip("\n").rstrip("\r"))
                if L > mx:
                    mx = L
        if mx > ABS:
            res[rel] = mx
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("linelen-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
