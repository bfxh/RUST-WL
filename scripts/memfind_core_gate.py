#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""memfind-core-gate：vxl-phys-memfind 不进求解核心（硬判，SPEC §10）。

§10「vxl-phys-memfind：用途限定资产管线索引/调试符号定位/序列化定位/碰撞模式查找；
不进求解核心」。本门扫全部 crate 的 Cargo.toml：资产/调试面 crate（replay/ffi/gpu）
允许依赖 memfind；其余 crate（求解核心）在任一依赖段引用 memfind → 红。存量须为 0。

实现：list_cargo_toml 取全部 manifest，按 package name 区分资产面与求解核心，
非资产面 crate 在 [dependencies]/[dev-dependencies]/[build-dependencies] 任一段
含 vxl-phys-memfind 即命中。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

MEMFIND = "vxl-phys-memfind"
# 资产/调试面 crate 允许依赖 memfind（§10 用途）；其余一律禁止
ALLOWED = {"vxl-phys-replay", "vxl-phys-ffi", "vxl-phys-gpu", MEMFIND}


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_cargo_toml(root):
        text = open(os.path.join(root, rel), encoding="utf-8").read()
        name = gc.cargo_pkg_name(text)
        if name in ALLOWED:
            continue
        secs = gc.parse_cargo_dep_sections(text)
        for sec, deps in secs.items():
            if MEMFIND in deps:
                hits.append((rel, name, sec))
                break
    if not hits:
        print("✅ memfind-core-gate: 求解核心 crate 均未引用 vxl-phys-memfind（§10 禁令）")
        sys.exit(0)
    print("❌ memfind-core-gate: 求解核心 crate 引用了 vxl-phys-memfind，违反 §10「不进求解核心」")
    for rel, name, sec in hits:
        print(f"   {rel} (pkg={name}) [{sec}]")
    sys.exit(1)


if __name__ == "__main__":
    main()
