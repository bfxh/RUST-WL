#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""todo-gate：todo!/unimplemented!/panic!/unreachable! 计数棘轮。

确定性物理引擎里 `panic!`/`unreachable!` 是「断言此处不可达」——一旦可达就是整局
回放作废，且 `unimplemented!` 是占位未完成。存量进基线，只准减，新增即红。
（注释里的 `// TODO` 不走宏，不被此门抓；此门只数真正会编译进二进制的宏调用。）
"""
import os
import re
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

_TODO = re.compile(r"\b(todo|unimplemented|panic|unreachable)!\s*\(")


def scan(files, root):
    res = {}
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        c = len(_TODO.findall(src))
        if c:
            res[rel] = c
    return res


def main():
    p = argparse.ArgumentParser()
    gc.add_args(p)
    a = p.parse_args()
    sys.exit(gc.run_gate("todo-gate", scan, a.git_tracked, a.write, a.top, a.list))


if __name__ == "__main__":
    main()
