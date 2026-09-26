#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate-probe：逐门注入最小违规，必须转红——证明「门是真门」。

静态自检只查门在不在，查不出门是不是空的（正则写错 / 路径过滤没匹配到文件 /
判据名单取空集都会让门空转却绿）。本脚本给每道棘轮门注入一次最小违规文件，
注入后门必须红（退出码 != 0 且打印 ❌）；注入前门已绿（基线通过）。

dag / rayon 为结构性硬判：靠真实数据已验证绿（无环+顺序合规 / 无 rayon 依赖），
其注入需改 Cargo.toml，归手动/后续，本探针跳过并注明。

归 full 档（CI 显式跑），不进提交钩子（每门跑两遍较重）。
"""
import os
import sys
import shutil
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBE_DIR = os.path.join(ROOT, "crates", "_gate_probe")

# 每道棘轮门 -> (临时文件名, 最小违规内容)
PROBES = {
    "unwrap-gate":  ("probe_unwrap.rs",  'fn p(){ let _x: Option<i32> = None; _x.unwrap(); }\n'),
    "unsafe-gate":  ("probe_unsafe.rs",  'fn p(){ unsafe { let _ = 1; } }\n'),
    "linelen-gate": ("probe_linelen.rs", 'fn p() { ' + "a" * 240 + ' }\n'),
    "glob-gate":    ("probe_glob.rs",    'use std::collections::*;\n'),
    "todo-gate":    ("probe_todo.rs",    'fn p(){ todo!(); }\n'),
    "god-gate":     ("probe_god.rs",     'fn p() {}\n'),
    "cyc-gate":     ("probe_cyc.rs",
        'fn p(a:i32,b:i32,c:i32,d:i32,e:i32,f:i32,g:i32,h:i32){'
        ' if a>0{1}else if b>0{2}else if c>0{3}else if d>0{4}'
        ' else if e>0{5}else if f>0{6}else if g>0{7}else if h>0{8} else {9} }\n'),
    "nest-gate":    ("probe_nest.rs",
        'fn p(){ if true { if true { if true { if true { if true { if true { let _=1; } } } } } } }\n'),
    "args-gate":    ("probe_args.rs",
        'fn p(a:i32,b:i32,c:i32,d:i32,e:i32,f:i32,g:i32,h:i32,i:i32){}\n'),
    "fastmath-gate": ("probe_fastmath.rs",
        'fn p(){ let _ = fadd_fast(1.0f32, 2.0f32); }\n'),
    "asm-gate":     ("probe_asm.rs",
        'fn p(){ unsafe { asm!("nop"); } }\n'),
}

SKIP = {
    "dag-gate":   "结构性硬判：真实数据已验证绿（19 crate 无环+顺序合规），注入需改 Cargo.toml，归手动",
    "rayon-gate": "结构性硬判：真实数据已验证绿（无 rayon 依赖），注入需改 Cargo.toml，归手动",
    "cpp-gate":   "结构性硬判：真实数据已验证绿（无 cxx/autocxx/cpp 依赖），注入需改 Cargo.toml，归手动",
}


def run_gate(gate):
    # dict 键用连字符（与门名/基线命名一致），实际脚本文件名用下划线
    script = "scripts/" + gate.replace("-", "_") + ".py"
    return subprocess.run(
        [sys.executable, script], cwd=ROOT,
        capture_output=True, text=True,
    )


def main():
    fails = []
    skipped = []
    try:
        for gate, (fname, content) in PROBES.items():
            os.makedirs(PROBE_DIR, exist_ok=True)
            with open(os.path.join(PROBE_DIR, fname), "w", encoding="utf-8") as f:
                f.write(content)
            out = run_gate(gate)
            os.remove(os.path.join(PROBE_DIR, fname))
            caught = out.returncode != 0 and "❌" in out.stdout
            if caught:
                print(f"✅ {gate}: 注入最小违规后转红（是真门）")
            else:
                print(f"❌ {gate}: 注入后仍绿 / 崩溃 —— 门可能为空门！")
                print("   rc:", out.returncode, "| stdout:", repr(out.stdout[:300]), "| stderr:", repr(out.stderr[:300]))
                fails.append(gate)
    finally:
        if os.path.isdir(PROBE_DIR):
            shutil.rmtree(PROBE_DIR, ignore_errors=True)

    for gate, why in SKIP.items():
        print(f"ℹ️  {gate}: 跳过注入验证 —— {why}")
        skipped.append(gate)

    print("=" * 50)
    if fails:
        print(f"gate-probe 失败：{len(fails)} 道疑似空门 —— {', '.join(fails)}")
        sys.exit(1)
    print(f"gate-probe 通过：{len(PROBES)} 道注入验证均转红；{len(skipped)} 道硬判归手动（{', '.join(skipped)}）")
    sys.exit(0)


if __name__ == "__main__":
    main()
