#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate-probe：逐门注入最小违规，必须转红——证明「门是真门」。

静态自检只查门在不在，查不出门是不是空的（正则写错 / 路径过滤没匹配到文件 /
判据名单取空集都会让门空转却绿）。本脚本给每道棘轮门注入一次最小违规文件，
注入后门必须红（退出码 != 0 且打印 ❌）；注入前门已绿（基线通过）。

.rs 注入：unwrap/unsafe/linelen/glob/todo/god/cyc/nest/args/fastmath/asm/staticmut。
Cargo.toml 注入（结构性硬判）：rayon/cpp/license——注入一个含违例的 manifest 必须转红。
dag 为结构性硬判且依赖 workspace 拓扑布局，注入改动大，靠真实数据已验证绿，归手动。

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
    "staticmut-gate": ("probe_staticmut.rs",
        'static mut PROBE: i32 = 0;\nfn p(){ unsafe { PROBE = 1; } }\n'),
    "dbg-gate":     ("probe_dbg.rs",
        'fn p(){ let _ = dbg!(1 + 1); }\n'),
    "print-gate":   ("probe_print.rs",
        'fn p(){ println!("leak"); }\n'),
    "clone-gate":   ("probe_clone.rs",
        'fn p(){ let v = vec![1i32]; let _ = v.clone(); }\n'),
}

# Cargo.toml 注入探针：结构性硬判，注入含违例的 manifest 必须转红
TOML_PROBES = {
    "rayon-gate":   '[package]\nname = "_gate_probe"\nversion = "0.0.0"\nlicense.workspace = true\n\n[dependencies]\nrayon = "1"\n',
    "cpp-gate":     '[package]\nname = "_gate_probe"\nversion = "0.0.0"\nlicense.workspace = true\n\n[dependencies]\ncxx = "0.7"\n',
    "license-gate": '[package]\nname = "_gate_probe"\nversion = "0.0.0"\nlicense = "GPL-3.0"\n',
    # §10：求解核心不得依赖 vxl-phys-memfind（_gate_probe 不在资产面白名单 → 命中）
    "memfind-core-gate": '[package]\nname = "_gate_probe"\nversion = "0.0.0"\nlicense.workspace = true\n\n[dependencies]\nvxl-phys-memfind = { path = "../vxl-phys-memfind" }\n',
    # §0：既有引擎 crate 不得作为 [dependencies]
    "engine-dep-gate": '[package]\nname = "_gate_probe"\nversion = "0.0.0"\nlicense.workspace = true\n\n[dependencies]\nrapier3d = "0.18"\n',
}

SKIP = {
    "dag-gate":   "结构性硬判：真实数据已验证绿（19 crate 无环+顺序合规），注入需改 workspace members + crate 布局，归手动",
}


def run_gate(gate):
    # dict 键用连字符（与门名/基线命名一致），实际脚本文件名用下划线
    script = "scripts/" + gate.replace("-", "_") + ".py"
    return subprocess.run(
        [sys.executable, script], cwd=ROOT,
        capture_output=True, text=True,
    )


def _probe_rs():
    fails = []
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
    return fails


def _probe_toml():
    fails = []
    try:
        for gate, content in TOML_PROBES.items():
            os.makedirs(PROBE_DIR, exist_ok=True)
            with open(os.path.join(PROBE_DIR, "Cargo.toml"), "w", encoding="utf-8") as f:
                f.write(content)
            out = run_gate(gate)
            shutil.rmtree(PROBE_DIR, ignore_errors=True)
            caught = out.returncode != 0 and "❌" in out.stdout
            if caught:
                print(f"✅ {gate}: 注入最小违例 manifest 后转红（是真门）")
            else:
                print(f"❌ {gate}: 注入违例 manifest 后仍绿 / 崩溃 —— 门可能为空门！")
                print("   rc:", out.returncode, "| stdout:", repr(out.stdout[:300]), "| stderr:", repr(out.stderr[:300]))
                fails.append(gate)
    finally:
        if os.path.isdir(PROBE_DIR):
            shutil.rmtree(PROBE_DIR, ignore_errors=True)
    return fails


def main():
    fails = []
    skipped = []
    fails += _probe_rs()
    fails += _probe_toml()
    for gate, why in SKIP.items():
        print(f"ℹ️  {gate}: 跳过注入验证 —— {why}")
        skipped.append(gate)
    print("=" * 50)
    if fails:
        print(f"gate-probe 失败：{len(fails)} 道疑似空门 —— {', '.join(fails)}")
        sys.exit(1)
    print(f"gate-probe 通过：{len(PROBES) + len(TOML_PROBES)} 道注入验证均转红；{len(skipped)} 道硬判归手动（{', '.join(skipped)}）")
    sys.exit(0)


if __name__ == "__main__":
    main()
