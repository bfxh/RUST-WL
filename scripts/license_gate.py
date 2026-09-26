#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""license-gate：本仓各 crate 许可须宽松，禁 GPL/AGPL/LGPL 传染，且必须声明（硬判，SPEC §11）。

§11「许可：MIT OR Apache-2.0；依赖审计禁 GPL 传染」。本门查**本仓各 crate** 的 Cargo.toml
license 字段（不查传递依赖——Cargo.lock 无 license 字段，传递依赖审计需 cargo metadata /
cargo-deny，建议另起 CI 作业）。crate 用 `license.workspace = true` 继承根
`[workspace.package]` 许可的，按根许可判定。命中宽松标识（MIT/Apache/BSD/Zlib/ISC/
Unlicense/0BSD/CC0/MIT-0/Boost/MPL-2.0）→ 绿；缺声明或含 GPL/AGPL/LGPL → 红。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

PERMISSIVE = re.compile(
    r"\b(?:MIT|Apache-2\.0|BSD-2-Clause|BSD-3-Clause|Zlib|ISC|Unlicense|0BSD|"
    r"CC0-1\.0|MIT-0|Boost-1\.0|MPL-2\.0)\b"
)
COPYLEFT = re.compile(r"\b(?:GPL-2\.0|GPL-3\.0|AGPL|LGPL-2\.1|LGPL-3\.0)\b")
WS_LIC_RE = re.compile(r"^\s*license\.workspace\s*=\s*true", re.M)
WS_LIC_BRACE = re.compile(r"^\s*license\s*=\s*\{\s*[^}]*workspace\s*=\s*true", re.M)
LIC_LIT = re.compile(r'^\s*license\s*=\s*"([^"]+)"', re.M)


def root_license(root):
    t = open(os.path.join(root, "Cargo.toml"), encoding="utf-8").read()
    m = re.search(r"\[\s*workspace\.package\s*\](.*?)(?:\n\[|\Z)", t, re.S)
    if m:
        lm = re.search(r'license\s*=\s*"([^"]+)"', m.group(1))
        if lm:
            return lm.group(1)
    lm = re.search(r'^\s*license\s*=\s*"([^"]+)"', t, re.M)
    return lm.group(1) if lm else None


def main():
    root = gc.repo_root()
    root_lic = root_license(root)
    hits = []     # GPL/AGPL/LGPL 传染
    miss = []     # 缺声明 / 非宽松
    for rel in gc.list_cargo_toml(root):
        t = open(os.path.join(root, rel), encoding="utf-8").read()
        if WS_LIC_RE.search(t) or WS_LIC_BRACE.search(t):
            lic = root_lic
        else:
            lm = LIC_LIT.search(t)
            lic = lm.group(1) if lm else None
        if lic is None:
            miss.append(rel)
            continue
        up = lic.upper()
        if COPYLEFT.search(up):
            hits.append((rel, lic))
        elif not PERMISSIVE.search(up):
            miss.append(f"{rel} ({lic})")
    if not hits and not miss:
        print(f"✅ license-gate: 各 crate 许可均宽松（继承根 {root_lic}），无 GPL 传染（§11）")
        sys.exit(0)
    if hits:
        print("❌ license-gate: 发现 GPL/AGPL/LGPL 传染许可，违反 §11「禁 GPL 传染」")
        for rel, lic in hits:
            print(f"   {rel}: {lic}")
    if miss:
        print("❌ license-gate: 以下 crate 缺许可声明或许可非宽松（§11 要求 MIT OR Apache-2.0）")
        for m in miss[:40]:
            print(f"   {m}")
    sys.exit(1)


if __name__ == "__main__":
    main()
