#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""args-gate：函数形参个数棘轮（>7 棘轮；>12 硬禁）。

形参 >7 = 相邻同类型参数能被对调而编译器不吭声（物理引擎里极易把 inertia 与 mass
顺序写反）。>12 视为不可维护，硬禁（存量须为 0）。
"""
import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(
        gc.fn_metric_gate(
            "args-gate", "args", soft=7, hard=20,
            git_tracked=a.git_tracked, write=a.write, top=a.top, list_only=a.list,
        )
    )


if __name__ == "__main__":
    main()
