#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cyc-gate：圈复杂度棘轮（>15 棘轮；>50 硬禁）。

圈复杂度 = 1 + if/for/while/loop/else/?/&&/||/match分支。近似判据（掩码后文本统计），
用于「单函数决策点只减不增」。>50 视为不可维护，硬禁（存量须为 0）。
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
            "cyc-gate", "cyc", soft=15, hard=60,
            git_tracked=a.git_tracked, write=a.write, top=a.top, list_only=a.list,
        )
    )


if __name__ == "__main__":
    main()
