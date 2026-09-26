#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate-selftest：门禁自检（防止门被悄悄削弱）。

S1 同源：CI 的 ci.yml 必须显式调用 scripts/local_gate.py（含 selftest 与 probe），
     否则门只在本地跑、CI 不拦 = 门是装饰。
S5 基线在位：每个棘轮门都须有对应的 *.baseline.json，否则棘轮无可比对象。
S2 钩子：本仓以 GitHub Actions 为强制面（无 pre-commit 钩子），仅作信息提示，
     不阻断——CI ratchet 作业已覆盖。

退出码：任一硬性检查失败即 1。
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 与 local_gate.STEPS 同步（避免循环 import，此处单列常量）
RATCHET_BASELINED = [
    "unwrap-gate", "unsafe-gate", "linelen-gate", "glob-gate",
    "todo-gate", "god-gate", "cyc-gate", "nest-gate", "args-gate", "clone-gate",
]
CI_YML = os.path.join(ROOT, ".github", "workflows", "ci.yml")
ANCHOR = "scripts/local_gate.py"  # 单一入口：local_gate 内部再跑 selftest + probe


def check_s1():
    if not os.path.exists(CI_YML):
        print("❌ S1: 找不到 .github/workflows/ci.yml")
        return False
    text = open(CI_YML, encoding="utf-8").read()
    if ANCHOR not in text:
        print(f"❌ S1: ci.yml 未调用 {ANCHOR}（门不在 CI 强制面）")
        return False
    print(f"✅ S1: ci.yml 调用 {ANCHOR}（单一入口，内部跑 selftest + probe，门在 CI 强制面）")
    return True


def check_s5():
    miss = []
    for name in RATCHET_BASELINED:
        p = os.path.join(ROOT, f"{name}.baseline.json")
        if not os.path.exists(p):
            miss.append(name)
    if miss:
        print("❌ S5: 缺基线 " + ", ".join(miss))
        return False
    print(f"✅ S5: {len(RATCHET_BASELINED)} 个棘轮门基线全部在位")
    return True


def check_s2():
    hook = os.path.join(ROOT, ".git", "hooks", "pre-commit")
    if os.path.exists(hook):
        print("✅ S2: pre-commit 钩子在位")
    else:
        print("ℹ️  S2: 无 pre-commit 钩子（本仓以 GitHub Actions ratchet 作业为强制面，信息提示不阻断）")
    return True


def main():
    ok = True
    ok &= check_s1()
    ok &= check_s5()
    ok &= check_s2()
    print("=" * 50)
    if ok:
        print("gate-selftest 通过 ✅")
        sys.exit(0)
    print("gate-selftest 失败 ❌")
    sys.exit(1)


if __name__ == "__main__":
    main()
