#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cpp-gate：禁 Rust↔C++ 互操作依赖（硬判，SPEC §0「禁 C++ 物理核心与引擎类型侵入」）。

物理核心纯 Rust（§0「物理核心零 C++」）；C++ 侵入破坏「纯 Rust」「模块化可独立
替换」与确定性（C++ 行为不可控、跨平台哈希难一致）。本门扫全部 crate 的 Cargo.toml
依赖名，发现 cxx / autocxx / cpp 系即红。存量须为 0。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

FORBID = {
    "cxx", "cxxbridge-macro", "autocxx", "autocxx-engine", "autocxx-build", "cpp",
}


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_cargo_toml(root):
        with open(os.path.join(root, rel), encoding="utf-8") as f:
            text = f.read()
        for dep in gc.parse_cargo_deps(text):
            if dep in FORBID:
                hits.append((rel, dep))
    if not hits:
        print("✅ cpp-gate: 无 Rust↔C++ 互操作依赖（§0 纯 Rust 禁令）")
        sys.exit(0)
    print("❌ cpp-gate: 发现 C++ 互操作依赖，违反 §0「禁 C++ 物理核心/引擎类型侵入」")
    for rel, dep in hits:
        print(f"   {rel}: {dep}")
    sys.exit(1)


if __name__ == "__main__":
    main()
