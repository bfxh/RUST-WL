#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f64-gate：求解核心禁用 f64（硬判，SPEC §5「严格 f32」）。

§5「严格 f32：禁 fast-math、FTZ/DAZ 关闭、sim 路径无 platform intrinsics、归约有序（岛内定序）」。
物理仿真全程 f32 是确定性回放与跨平台哈希 bit 级一致的前提（§12.1 身份门）。f64 一旦进入
求解核心，会破坏「严格 f32」口径、引入与 f32 路径不同的舍入与序列化差异，威胁跨平台哈希一致。

本门扫求解核心 crate 的全部 .rs（排除 tests/benches/examples/bin 与资产/调试面 crate：
memfind/ffi/replay/gpu——这些非 sim 路径），经 gc.mask 掩码后查 `f64` 类型/字面量，发现即红。
存量须为 0（实测 31 个核心 src 文件 0 命中）。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

# 兼容 `1f64` 这类无空格字面量（\b 在 数字与 f 之间无断词边界）
F64_RE = re.compile(r"(?<![A-Za-z_])f64\b")
# 资产/调试面 crate 豁免（§10：memfind 仅资产/调试面；replay/gpu/ffi 非求解核心）
ASSET = {"vxl-phys-memfind", "vxl-phys-ffi", "vxl-phys-replay", "vxl-phys-gpu"}
EXCL = ("/tests/", "/benches/", "/examples/", "/src/bin/")


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, False):
        p = "/" + rel + "/"
        if any(x in p for x in EXCL):
            continue
        crate = rel.split("/crates/")[1].split("/")[0] if "/crates/" in rel else "root"
        if crate in ASSET:
            continue
        text = open(os.path.join(root, rel), encoding="utf-8").read()
        masked = gc.mask(text)
        for m in F64_RE.finditer(masked):
            s = max(0, m.start() - 30)
            ctx = masked[s:m.start() + 3].replace("\n", " ")
            hits.append((rel, ctx))
            break
    if not hits:
        print("✅ f64-gate: 求解核心无 f64（§5 严格 f32）")
        sys.exit(0)
    print("❌ f64-gate: 求解核心出现 f64，违反 §5「严格 f32」（破坏确定性回放/跨平台哈希一致）")
    for rel, ctx in hits[:30]:
        print(f"   {rel}: ...{ctx}")
    sys.exit(1)


if __name__ == "__main__":
    main()
