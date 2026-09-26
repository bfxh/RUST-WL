#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""engine-dep-gate：禁止以既有游戏引擎/物理引擎作为物理核心（硬判，SPEC §0）。

§0 工程禁令「禁以既有游戏引擎作为物理核心；禁 GPL 传染依赖」。本门扫全部 crate 的
Cargo.toml 的 [dependencies] / [build-dependencies]（非 dev）：发现 rapier/bevy/physx/
godot/amethyst 等引擎 crate → 红。[dev-dependencies] 豁免——gold-sample 对照（如
Rapier 在 benches/tests 做逐帧位姿对照，§0/§12.1 允许），不计入物理核心依赖树。
存量须为 0。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

# 既有游戏/物理引擎 crate —— 不得作为本引擎的 [dependencies]（物理核心）
FORBID = {
    "rapier", "rapier2d", "rapier3d",
    "bevy", "bevy_ecs",
    "physx", "physx-sys",
    "godot", "godot-rs",
    "amethyst",
    "nphysics",
    "salva", "salva2d", "salva3d",
    "parry", "parry2d", "parry3d",
}


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_cargo_toml(root):
        secs = gc.parse_cargo_dep_sections(
            open(os.path.join(root, rel), encoding="utf-8").read()
        )
        bad = (secs.get("dependencies", set()) | secs.get("build-dependencies", set())) & FORBID
        if bad:
            hits.append((rel, sorted(bad)))
    if not hits:
        print("✅ engine-dep-gate: 无游戏/物理引擎 crate 作为物理核心依赖（§0 禁令）")
        sys.exit(0)
    print("❌ engine-dep-gate: 发现既有引擎 crate 作为 [dependencies]，违反 §0「禁以既有游戏引擎作为物理核心」")
    for rel, bad in hits:
        print(f"   {rel}: {', '.join(bad)}")
    sys.exit(1)


if __name__ == "__main__":
    main()
