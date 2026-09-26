#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""unsafe-gate：unsafe 块计数棘轮（SPEC §5/§6.1：核心 forbid(unsafe_code)，全库 Safe Rust 优先）。

`#![forbid(unsafe_code)]` 已在 core 编译期强制；此门把「unsafe 块数」也纳入棘轮，
朝「全库零 unsafe」收敛（§6.1 讨论仅在独立调度 crate 放宽，届时该 crate 需显式豁免）。
"""
import os
import re
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

_UNSAFE = re.compile(r"\bunsafe\b")


def scan(files, root):
    res = {}
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        c = len(_UNSAFE.findall(src))
        if c:
            res[rel] = c
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("unsafe-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
