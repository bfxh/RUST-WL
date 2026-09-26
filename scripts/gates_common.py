#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RUST-WL 门禁共用件（零依赖纯 stdlib）。

提供：Rust 感知掩码、函数体遍历（算 args / 嵌套深度 / 圈复杂度 / 行数）、
git-tracked 文件列举、基线读写、棘轮判定，以及两个统一入口
`run_gate`（按文件计数的门）与 `fn_metric_gate`（按函数度量的门）。

掩码纪律：复杂度/形参/嵌套度量必须先把字符串/字符/注释抹掉，否则会数到
format! 里的 `{`、字符字面量 `'{'`、注释里的伪关键字——花括号配平与判据全错。
行长门量原始文本（不过掩码）。Rust 的字符字面量 `'{'` 与生命周期 `'a` 都要认：
字符字面量有收尾 `'`（正则能匹配），生命周期 `'a` 没有收尾 `'`（不会误匹配），
天然区分。
"""
import os
import re
import sys
import json
import subprocess

# --------------------------------------------------------------------------
# 路径
# --------------------------------------------------------------------------
def repo_root():
    d = os.path.dirname(os.path.abspath(__file__))
    while d != os.path.dirname(d):
        if os.path.exists(os.path.join(d, "Cargo.toml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()


def list_rs(root, git_tracked):
    if git_tracked:
        out = subprocess.run(
            ["git", "-C", root, "ls-files", "*.rs"],
            capture_output=True, text=True,
        )
        return [l for l in out.stdout.splitlines() if l.endswith(".rs")]
    res = []
    for dp, _, fns in os.walk(root):
        base = os.path.basename(dp)
        if base in ("target", ".git"):
            continue
        for fn in fns:
            if fn.endswith(".rs"):
                rel = os.path.relpath(os.path.join(dp, fn), root).replace(os.sep, "/")
                res.append(rel)
    return res


# --------------------------------------------------------------------------
# 掩码：把字符串/字符/注释抹成不含花括号的占位
# --------------------------------------------------------------------------
def mask(text):
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        c2 = text[i : i + 2]
        # 行注释（含 /// 与 //! 文档注释）
        if c2 == "//":
            while i < n and text[i] != "\n":
                i += 1
            continue
        # 块注释
        if c2 == "/*":
            i += 2
            while i < n and text[i : i + 2] != "*/":
                i += 1
            i += 2
            continue
        # 字符字面量 'x' / '\n' / '\u{41}' —— 抹成 ''（无花括号占位）
        if c == "'":
            m = re.match(r"^'(?:\\.|\\u\{[0-9a-fA-F]+\}|[^'\\])'", text[i:])
            if m:
                out.append("''")
                i += m.end()
                continue
            # 否则是生命周期 'a（无收尾 '）——原样保留，不含花括号，无害
            out.append(c)
            i += 1
            continue
        # 字符串字面量（含字节串 b"..."；原始串 r"..."/r#"..."# 不含内部 "，
        # 普通串扫描器正好能吃到底，内容被抹掉，开头的 r/b 残留无害）
        if c == '"':
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            out.append('""')
            continue
        out.append(c)
        i += 1
    return "".join(out)


# --------------------------------------------------------------------------
# 函数体遍历
# --------------------------------------------------------------------------
def count_args(span):
    span = span.strip()
    if not span:
        return 0
    depth = 0
    commas = 0
    for ch in span:
        if ch in "<([":
            depth += 1
        elif ch in ">)]":
            depth -= 1
        elif ch == "," and depth == 0:
            commas += 1
    return commas + 1


def nesting_depth(body):
    depth = 0
    mx = 0
    for ch in body:
        if ch == "{":
            depth += 1
            if depth > mx:
                mx = depth
        elif ch == "}":
            depth -= 1
    return mx


def cyclomatic(body):
    c = 1
    c += len(re.findall(r"\bif\b", body))
    c += len(re.findall(r"\bfor\b", body))
    c += len(re.findall(r"\bwhile\b", body))
    c += len(re.findall(r"\bloop\b", body))
    c += len(re.findall(r"\belse\b", body))
    c += len(re.findall(r"\?", body))
    c += len(re.findall(r"&&", body))
    c += len(re.findall(r"\|\|", body))
    c += len(re.findall(r"=>", body))  # match 分支
    return c


_FN_RE = re.compile(r"\bfn\b")


def iter_fns(src):
    """产出每个带函数体的 `fn` 的指标字典。src 已是掩码后文本。"""
    out = []
    for m in _FN_RE.finditer(src):
        j = m.end()
        while j < len(src) and src[j] in " \t":
            j += 1
        nm = re.match(r"[A-Za-z_][A-Za-z0-9_]*", src[j:])
        if not nm:
            continue
        name = nm.group(0)
        j += nm.end()
        # 跳过泛型 <...> 找到参数列表 '('（跟踪 < > 深度）
        k = j
        while k < len(src) and src[k] != "(":
            if src[k] == "<":
                depth = 1
                k += 1
                while k < len(src) and depth:
                    if src[k] == "<":
                        depth += 1
                    elif src[k] == ">":
                        depth -= 1
                    k += 1
                continue
            k += 1
        if k >= len(src):
            continue
        # 找匹配 ')'
        paren = 0
        p = k
        while p < len(src):
            if src[p] == "(":
                paren += 1
            elif src[p] == ")":
                paren -= 1
                if paren == 0:
                    break
            p += 1
        args_span = src[k + 1 : p]
        # 参数列表之后：跳过返回类型 / where，找 '{' 或 ';'
        body_start = None
        r = p + 1
        while r < len(src):
            ch = src[r]
            if ch == "{":
                body_start = r
                break
            if ch == ";":
                break
            r += 1
        if body_start is None:
            continue  # trait 签名 / extern 声明，无函数体
        # 找匹配 '}'
        depth2 = 0
        s = body_start
        while s < len(src):
            if src[s] == "{":
                depth2 += 1
            elif src[s] == "}":
                depth2 -= 1
                if depth2 == 0:
                    break
            s += 1
        body = src[body_start + 1 : s]
        out.append(
            {
                "name": name,
                "args": count_args(args_span),
                "max_nest": nesting_depth(body),
                "cyc": cyclomatic(body),
                "n_lines": body.count("\n") + 1,
            }
        )
    return out


# --------------------------------------------------------------------------
# 基线读写 + 棘轮判定
# --------------------------------------------------------------------------
def read_baseline(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_baseline(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")


def ratchet_check(baseline, current):
    """current 比 baseline 只许减。返回违规清单（字符串）。"""
    viol = []
    for rel, cur in current.items():
        base = baseline.get(rel)
        if base is None:
            if cur and cur > 0:
                viol.append(f"{rel}: 新文件计数 {cur}（基线无）")
            continue
        if cur > base:
            viol.append(f"{rel}: {cur} > 基线 {base}（只许减）")
    return viol


# --------------------------------------------------------------------------
# 统一入口 A：按文件计数的门（unwrap/unsafe/glob/todo/linelen/god）
# --------------------------------------------------------------------------
def run_gate(name, scan_fn, git_tracked, write, top, list_only):
    root = repo_root()
    files = list_rs(root, git_tracked)
    current = scan_fn(files, root)
    if list_only:
        ranked = sorted(current.items(), key=lambda kv: kv[1], reverse=True)
        shown = ranked[: (top or 20)]
        print(f"== {name} 存量（按文件，共 {len(current)} 文件，合计 {sum(current.values())}）==")
        for rel, v in shown:
            if v:
                print(f"  {v:>5}  {rel}")
        return 0
    baseline_path = os.path.join(root, f"{name}.baseline.json")
    if write:
        write_baseline(baseline_path, current)
        print(f"✅ {name}: 基线已写 {baseline_path}（{len(current)} 文件，合计 {sum(current.values())}）")
        return 0
    if not os.path.exists(baseline_path):
        print(f"❌ {name}: 基线缺失，先跑 `--write`")
        return 1
    baseline = read_baseline(baseline_path)
    viol = ratchet_check(baseline, current)
    if viol:
        print(f"❌ {name}: {len(viol)} 处新增/增长")
        for v in viol[:30]:
            print("   " + v)
        return 1
    print(f"✅ {name}: 0 新增（基线合计 {sum(baseline.values())}）")
    return 0


# --------------------------------------------------------------------------
# 统一入口 B：按函数度量的门（cyc/nest/args）
# --------------------------------------------------------------------------
def fn_metric_gate(name, metric_key, soft, hard, git_tracked, write, top, list_only):
    root = repo_root()
    files = list_rs(root, git_tracked)
    per_file = {}      # rel -> 超过 soft 的函数数（棘轮口径）
    maxval = {}        # rel -> 该文件最大 metric（--list 展示）
    hard_viol = []     # (rel, fn, value)
    for rel in files:
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = mask(f.read())
        for fn in iter_fns(src):
            v = fn[metric_key]
            if rel not in maxval or v > maxval[rel]:
                maxval[rel] = v
            if v > soft:
                per_file[rel] = per_file.get(rel, 0) + 1
            if v > hard:
                hard_viol.append((rel, fn["name"], v))
    if list_only:
        ranked = sorted(maxval.items(), key=lambda kv: kv[1], reverse=True)
        shown = ranked[: (top or 20)]
        print(f"== {name} 各文件最大 {metric_key}（soft>{soft} 记棘轮；hard>{hard} 硬禁）==")
        for rel, v in shown:
            print(f"  {v:>5}  {rel}")
        return 0
    baseline_path = os.path.join(root, f"{name}.baseline.json")
    if write:
        write_baseline(baseline_path, per_file)
        print(f"✅ {name}: 基线已写 {baseline_path}（超 soft 函数计数，共 {sum(per_file.values())}）")
        return 0
    if not os.path.exists(baseline_path):
        print(f"❌ {name}: 基线缺失，先跑 `--write`")
        return 1
    baseline = read_baseline(baseline_path)
    if hard_viol:
        print(f"❌ {name}: 硬禁触发（{metric_key} > {hard}）")
        for rel, fn, v in hard_viol[:30]:
            print(f"   {rel}::{fn} = {v}")
        return 1
    viol = ratchet_check(baseline, per_file)
    if viol:
        print(f"❌ {name}: {len(viol)} 个文件超 soft 函数数增长")
        for v in viol[:30]:
            print("   " + v)
        return 1
    print(f"✅ {name}: 0 新增（基线超 soft 计数 {sum(baseline.values())}）")
    return 0


# --------------------------------------------------------------------------
# Cargo.toml 轻量解析（只取依赖名 + workspace members）
# --------------------------------------------------------------------------
_DEP_SECTION_RE = re.compile(
    r'^\[((?:(?:dev|build)-)?dependencies)(?:\.("[^"]*"|[^"\]]*))?\]$'
)


def parse_cargo_deps(text):
    """返回该 Cargo.toml 里出现的全部依赖名（含 dev/build 与 crate 专属表）。"""
    names = []
    mode = None
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("["):
            m = _DEP_SECTION_RE.match(s)
            if m:
                sub = m.group(2)
                if sub is None:
                    mode = "plain"
                else:
                    mode = "subtable"
                    names.append(sub.strip().strip('"'))
            else:
                mode = None
            continue
        if mode == "plain":
            km = re.match(r"^([A-Za-z0-9_-]+)\s*=", s)
            if km:
                names.append(km.group(1))
    return names


def parse_members(text):
    """返回根 Cargo.toml [workspace] members 列表（有序）。"""
    m = re.search(r"members\s*=\s*\[(.*?)\]", text, re.S)
    if not m:
        return []
    return re.findall(r'"([^"]+)"', m.group(1))


# --------------------------------------------------------------------------
# 参数解析
# --------------------------------------------------------------------------
def add_args(p):
    p.add_argument("--git-tracked", action="store_true",
                   help="只扫 git ls-files 跟踪的 *.rs（CI/提交口径）")
    p.add_argument("--write", action="store_true", help="写基线而非校验")
    p.add_argument("--list", action="store_true", help="列出存量（不校验）")
    p.add_argument("--top", type=int, default=20, help="--list 展示条数")


if __name__ == "__main__":
    print("gates_common 是共用件，不是独立门。各门脚本直接 import。")
    sys.exit(0)
