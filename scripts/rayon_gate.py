#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rayon-gate：禁止 rayon 依赖（硬判，SPEC §6：「禁止 Rayon 与自研 JobSystem 混用」）。

引擎并发走自有 `schedule::ScopedPool`（std::thread::scope），明确不与 Rayon 混用。
rayon 一旦进依赖树就能被 `use` 进来，破坏该禁令。本门扫全部 crate 的 Cargo.toml
依赖名，发现 `rayon` 即红。存量须为 0。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

FORBID = "rayon"


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, git_tracked=False):
        if not rel.endswith("Cargo.toml"):
            continue
        with open(os.path.join(root, rel), encoding="utf-8") as f:
            text = f.read()
        if FORBID in gc.parse_cargo_deps(text):
            hits.append(rel)
    if not hits:
        print("✅ rayon-gate: 无 rayon 依赖（§6 禁令）")
        sys.exit(0)
    print("❌ rayon-gate: 发现 rayon 依赖，违反 §6「禁止 Rayon 与自研 JobSystem 混用」")
    for h in hits:
        print("   " + h)
    sys.exit(1)


if __name__ == "__main__":
    main()
