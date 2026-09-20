#!/usr/bin/env python3
"""依赖红线（增量门）：外部依赖 / 构建依赖只准减、不准加。

用法：
  python3 scripts/deps_lock.py [根目录]              # 判门（CI 与本地同款）
  python3 scripts/deps_lock.py [根目录] --write-baseline   # 登记新增后重写基线

退出码：0 = 通过；1 = 命中新增（CI 阻断）。

为什么是「增量门」而不是「恒空」：本仓现状**不是**零依赖——`vxl-phys-replay` 依赖
`xxhash-rust`（xxh3-128 规范化哈希，跨平台位级一致的地基）、`vxl-phys-core` 有
`cfg(loom)` 下的 `loom`（并发原语模型测试）、Cargo.lock 还带它们的传递闭包
（含 `cc` / `find-msvc-tools`——**传递**进来的构建工具，不在本仓 manifest 里）。
把「现状」一刀切成恒空既不真也不可行；改成快照 + 棘轮：数量只准减，新增必须**登记
理由**（`scripts/deps-baseline.json` 的 `why`），否则红。

为什么单独一条门：`cargo-deny` 管「已有外部依赖是否合法/有无公告」，`cargo-machete`
管「有没有没用的」——两条都不拦**新增**一个外部依赖。而本仓的确定性叙事（三编译器 +
aarch64 逐位一致）建立在依赖图可知之上；`[build-dependencies]` 尤其敏感：`cc`/`bindgen`
一旦由本仓 manifest 直接引入，C 工具链就进了构建路径。

白名单（已批准）：
  - `gold-sample/` 自带 [workspace]、不进主 workspace，其对 rapier3d 的依赖是「对标参考」
    而非引擎依赖（独立 job、只在 PR 跑、continue-on-error）⇒ 本脚本不扫它。
  - 主 workspace 内已有的外部依赖见基线；新增按上面的流程登记。
"""
import json
import re
import sys
import tomllib
from pathlib import Path

DEP_TABLES = ("dependencies", "dev-dependencies")


def manifest_paths(root: Path) -> list[Path]:
    ws = tomllib.loads((root / "Cargo.toml").read_text(encoding="utf-8"))
    out = [root / "Cargo.toml"]
    for m in ws.get("workspace", {}).get("members", []):
        p = root / m / "Cargo.toml"
        if p.is_file():
            out.append(p)
    return out


def scan(root: Path) -> tuple[dict, dict, list[str], set[str]]:
    """返回 (外部依赖 {crate: {dep: 形态}}, 构建依赖 {crate: [dep]}, lock 包名, workspace crate 名)"""
    external: dict[str, dict] = {}
    build: dict[str, list[str]] = {}
    names: set[str] = set()

    def walk(path: Path) -> None:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
        crate = data.get("package", {}).get("name")
        if not crate:
            return
        names.add(crate)
        external.setdefault(crate, {})
        for table in DEP_TABLES:
            for dep, spec in (data.get(table) or {}).items():
                if not (isinstance(spec, dict) and ("path" in spec or spec.get("workspace") is True)):
                    external[crate][dep] = "version" if not isinstance(spec, dict) else "registry"
        for dep in (data.get("build-dependencies") or {}):
            build.setdefault(crate, []).append(dep)
        for tgt, cfg in (data.get("target") or {}).items():
            for table in DEP_TABLES:
                for dep, spec in (cfg.get(table) or {}).items():
                    if not (isinstance(spec, dict) and ("path" in spec or spec.get("workspace") is True)):
                        external[crate][dep] = f"target.{tgt}"
            for dep in (cfg.get("build-dependencies") or {}):
                build.setdefault(crate, []).append(f"target.{tgt}:{dep}")

    for p in manifest_paths(root):
        if p.is_file():
            walk(p)
    lock_pkgs = sorted(set(re.findall(r'^name = "([^"]+)"', (root / "Cargo.lock").read_text(encoding="utf-8"), re.M)))
    return external, build, lock_pkgs, names


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    root = Path(args[0] if args else ".").resolve()
    write = "--write-baseline" in sys.argv
    baseline_file = root / "scripts" / "deps-baseline.json"
    external, build, lock_pkgs, names = scan(root)
    baseline = json.loads(baseline_file.read_text(encoding="utf-8")) if baseline_file.is_file() else {
        "deps": {}, "build_deps": {}, "lock_packages": []}

    if write:
        merged: dict = {"deps": {}, "build_deps": {}, "lock_packages": sorted(lock_pkgs)}
        for crate, deps in sorted(external.items()):
            merged["deps"][crate] = {}
            for dep in sorted(deps):
                old = (baseline.get("deps", {}).get(crate) or {}).get(dep)
                merged["deps"][crate][dep] = old or "（待填：为什么需要它——引入它改变了什么契约？）"
        for crate, deps in sorted(build.items()):
            merged["build_deps"][crate] = sorted(deps)
        baseline_file.write_text(json.dumps(merged, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        todo = sum(1 for d in merged["deps"].values() for w in d.values() if "待填" in w)
        print(f"已重写基线：{baseline_file.relative_to(root)}"
              + (f"（⚠️ 有 {todo} 处理由是占位符，提交前必须填实）" if todo else "（理由均已填实）"))
        return 0

    fail = False
    for crate, deps in sorted(external.items()):
        known = baseline.get("deps", {}).get(crate) or {}
        for dep in sorted(deps):
            if dep not in known:
                fail = True
                print(f"❌ 新增外部依赖：{crate} → {dep}（{deps[dep]}）\n"
                      f"   登记流程：确认它改变/不改变确定性契约 → 跑 --write-baseline → "
                      f"把 deps-baseline.json 里的占位理由填实 → 一起提交")
            elif not known[dep] or "待填" in known[dep]:
                fail = True
                print(f"❌ {crate} → {dep} 的登记理由仍是占位符，请填实")
        for dep in sorted(set(known) - set(deps)):
            print(f"NOTE {crate} → {dep} 已不在依赖表里，可顺手把它从基线删掉")
    for crate, deps in sorted(build.items()):
        known = set(baseline.get("build_deps", {}).get(crate) or [])
        for dep in deps:
            if dep not in known:
                fail = True
                print(f"❌ 新增构建依赖：{crate} → {dep}\n"
                      f"   构建工具直接依赖会改变交叉编译与哈希门的成立条件；确需引入先裁决再登记")
    for pkg in lock_pkgs:
        if pkg not in baseline.get("lock_packages", []) and pkg not in names:
            fail = True
            print(f"❌ Cargo.lock 出现未登记包：{pkg}（传递闭包随新增依赖增长；"
                  f"若确属某已登记依赖的传递项，跑 --write-baseline 记账）")
    if len(lock_pkgs) < len(baseline.get("lock_packages", [])):
        print("NOTE 锁文件比基线短了，可顺手收紧基线")

    if not fail:
        direct = sum(len(v) for v in external.values())
        print(f"✅ 依赖红线通过（{len(external)} 个 crate：{direct} 项外部依赖均已在基线内、"
              f"0 项本仓构建依赖；Cargo.lock {len(lock_pkgs)} 项）")
    else:
        print("—— 条款出处：README「依赖 DAG」 + docs/SPEC.md §5（跨平台位级一致的前提）")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
