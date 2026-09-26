#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dag-gate：crate 依赖图无环 + 遵循声明顺序（硬判，SPEC §1 / §12.4）。

SPEC：「crate DAG 无环检查进 CI」。本门做两件事：
1. 无环：任何 crate 不得（直接/间接）依赖自己；
2. 顺序：每个 crate 的内部依赖必须出现在 workspace members 列表的更早位置
   （members 列表即权威拓扑序：core <- broad <- narrow <- ... <- vxl-phys）。

存量须为 0；任一违例即红（阻断合入）。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

INTERNAL = "vxl-phys"


def main():
    root = gc.repo_root()
    with open(os.path.join(root, "Cargo.toml"), encoding="utf-8") as f:
        root_text = f.read()
    members = gc.parse_members(root_text)            # 路径列表，如 crates/vxl-phys-core
    order = {p: i for i, p in enumerate(members)}
    name_to_path = {os.path.basename(p): p for p in members}

    graph = {}        # 路径 -> set(内部依赖路径)
    order_viol = []
    for p in members:
        ctoml = os.path.join(root, p, "Cargo.toml")
        if not os.path.exists(ctoml):
            continue
        with open(ctoml, encoding="utf-8") as f:
            text = f.read()
        deps = [d for d in gc.parse_cargo_deps(text) if d.startswith(INTERNAL)]
        graph[p] = set()
        for d in deps:
            dp = name_to_path.get(d)
            if dp is None:
                continue  # 非 members 的内部名（理论不应发生）
            if dp == p:
                order_viol.append(f"{p}: 自依赖（环）")
                continue
            graph[p].add(dp)
            if order[dp] >= order[p]:
                order_viol.append(
                    f"{p}: 依赖 {d}（{dp}）但位置 {order[dp]} >= 自身 {order[p]}（须更早）"
                )

    # 环检测（DFS 三色）
    WHITE, GRAY, BLACK = 0, 1, 2
    color = {n: WHITE for n in graph}

    def dfs(u, path):
        color[u] = GRAY
        for v in sorted(graph.get(u, ())):
            cv = color.get(v, WHITE)
            if cv == GRAY:
                if v in path:
                    i = path.index(v)
                    cycles.append(" -> ".join(path[i:] + [v]))
                else:
                    cycles.append(f"{u} -> {v}")
            elif cv == WHITE:
                dfs(v, path + [u])
        color[u] = BLACK

    cycles = []
    for n in graph:
        if color[n] == WHITE:
            dfs(n, [])

    ok = not order_viol and not cycles
    if ok:
        print(f"✅ dag-gate: 无环 + 顺序合规（{len(members)} 个 crate，内部边 {sum(len(v) for v in graph.values())} 条）")
        sys.exit(0)
    print("❌ dag-gate: crate DAG 违例")
    for v in order_viol:
        print("   " + v)
    for c in cycles:
        print("   环: " + c)
    sys.exit(1)


if __name__ == "__main__":
    main()
