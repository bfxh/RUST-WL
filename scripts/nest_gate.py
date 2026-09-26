#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nest-gate：函数嵌套深度棘轮（>5 棘轮；>8 硬禁）。

嵌套深度 = 函数体内 `{` 块的最大层数。深嵌套 = 圈复杂度数分支套几层，读不动、改错。
>8 视为不可维护，硬禁（存量须为 0）。
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
            "nest-gate", "max_nest", soft=5, hard=8,
            git_tracked=a.git_tracked, write=a.write, top=a.top, list_only=a.list,
        )
    )


if __name__ == "__main__":
    main()
