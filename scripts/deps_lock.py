#!/usr/bin/env python3
"""依赖红线（增量门）：外部依赖 / 构建依赖 / 构建脚本 / 补丁 只准减、不准加。

用法：
  python scripts/deps_lock.py [根目录]                    # 判门（CI 与本地同款）
  python scripts/deps_lock.py [根目录] --write-baseline   # 登记新增后重写基线

退出码：0 = 通过；1 = 命中新增（CI 阻断）；2 = 环境不满足（需 Python 3.11+ 的 tomllib）。

威胁模型（每一条都对应一个曾经能绕过旧版判定的路径，2026-09-21 对抗审核后补齐）：
  ① member 的 `{ workspace = true }` 会继承**根** `[workspace.dependencies]`——只扫 member
     等于让 `cc = "1"` 从正门走进来（旧版漏）。故根 manifest 的 workspace 依赖段单独扫。
  ② `[patch.*]` / `[replace]` 可以换掉已登记依赖的实现（含 path 补丁），名字不变、锁里
     不带 source 行——名字集判定看不见它。故出现即红（白名单为空）。
  ③ `build.rs` 不需要任何 manifest 条目就能 `Command::new("cc"|"protoc"|"nasm")`——
     它就是「构建工具直接依赖」的隐形入口。故按**路径集合**对账，新增即红。
  ④ `path = "…"` 被无条件当「自己人」——指向仓外或未纳入扫描的目录时，其自身依赖图不可见。
     故要求 path 解析后仍在仓内、且其 manifest 在被扫集合里。
  ⑤ 重命名依赖（`xxhash-rust = { package = "bitcode" }`）按 TOML 键记账，理由会挂到不存在的
     crate 上、也能被重指。故按**真实 crate 名**记账（别名只在报告里显示）。
  ⑥ 锁只按名字对账：静默的版本/来源变化看不见。故基线记 `[名字, 版本]`，新增名字红、
     版本变化 **NOTE 提示**（不阻断——`cargo update` 是常规操作，而 xxhash-rust 这类
     与位级一致性相关的版本变化由跨平台哈希门做最终裁决）。

快照 + 棘轮：本仓现状不是零依赖（见基线），数量只准减；新增必须登记理由，占位符也算红。

白名单（已批准，带理由）：
  - `gold-sample/` 自带 [workspace]、不进主 workspace，其对 rapier3d 的依赖是「对标参考」
    而非引擎依赖（独立 job、只在 PR 跑、continue-on-error）⇒ 本脚本不扫它。
  - 主 workspace 内的外部依赖与构建脚本见基线；新增按上面的流程登记。
"""
import glob
import json
import re
import sys

try:
    import tomllib
except ModuleNotFoundError:  # 3.11 以下：如实报「能力缺席」，不静默放行
    print("deps_lock: 需要 Python 3.11+（tomllib 缺席）——CI 用 ubuntu-latest 的 python3，"
          "本机请用 python3.11+ 或 python 3.14", file=sys.stderr)
    sys.exit(2)

from pathlib import Path

DEP_TABLES = ("dependencies", "dev-dependencies")
PATCH_SECTIONS = ("patch", "replace")
PLACEHOLDER = "（待填：为什么需要它——引入它改变了什么契约？）"


def _is_internal(spec) -> bool:
    """path 或 workspace 继承 = 内部；其余（字符串版本、git、registry 表）= 外部。"""
    return isinstance(spec, dict) and ("path" in spec or spec.get("workspace") is True)


def _record(target: dict, crate: str, key: str, spec) -> None:
    name = spec.get("package", key) if isinstance(spec, dict) else key
    target.setdefault(crate, {})[name] = spec


def manifests(root: Path) -> list[Path]:
    """root + 每个 workspace member 的 manifest。member 解析不了就炸（不静默跳过）。"""
    ws = tomllib.loads((root / "Cargo.toml").read_text(encoding="utf-8"))
    out = [root / "Cargo.toml"]
    for m in ws.get("workspace", {}).get("members", []):
        if "*" in m:
            hits = [Path(p) for p in glob.glob(str(root / m))]
            if not hits:
                raise SystemExit(f"deps_lock: member 通配 {m!r} 没匹配到任何目录——门不该静默跳过")
            out += [h / "Cargo.toml" for h in hits]
            continue
        p = root / m / "Cargo.toml"
        if not p.is_file():
            raise SystemExit(f"deps_lock: member {m!r} 的 Cargo.toml 不存在——门不该静默跳过")
        out.append(p)
    return out


def scan(root: Path) -> dict:
    files = manifests(root)
    external: dict[str, dict] = {}
    build: dict[str, dict] = {}
    patches: list[str] = []
    names: set[str] = set()
    path_issues: list[str] = []

    def walk_manifest(path: Path, crate: str, is_root: bool) -> None:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
        # ① 根的 workspace 依赖段：member 的 workspace = true 最终落到这里
        ws_deps = (data.get("workspace") or {}).get("dependencies") or {}
        for key, spec in ws_deps.items():
            if not _is_internal(spec):
                _record(external, f"{crate}·workspace", key, spec)
        for table in DEP_TABLES:
            for key, spec in (data.get(table) or {}).items():
                if not _is_internal(spec):
                    _record(external, crate, key, spec)
        for key in (data.get("build-dependencies") or {}):
            build.setdefault(crate, {})[key] = None
        for tgt, cfg in (data.get("target") or {}).items():
            for table in DEP_TABLES:
                for key, spec in (cfg.get(table) or {}).items():
                    if not _is_internal(spec):
                        _record(external, crate, key, spec)
            for key in (cfg.get("build-dependencies") or {}):
                build.setdefault(crate, {})[f"target.{tgt}:{key}"] = None
        # ② 补丁/替换：出现即登记（白名单当前为空）
        for section in PATCH_SECTIONS:
            for sub in (data.get(section) or {}):
                patches.append(f"{crate}|{section}.{sub}")
        # ④ path 依赖必须留在仓内，且指向被扫到的 manifest
        scanned = {p.resolve() for p in files}
        for table in (*DEP_TABLES, "build-dependencies"):
            for key, spec in (data.get(table) or {}).items():
                if not (isinstance(spec, dict) and "path" in spec):
                    continue
                dest = (path.parent / spec["path"]).resolve()
                if not dest.is_relative_to(root):
                    path_issues.append(f"{crate}·{key} → 仓外：{dest}")
                elif (dest / "Cargo.toml").resolve() not in scanned:
                    path_issues.append(f"{crate}·{key} → 未被扫描的目录：{dest}")

    for p in files:
        data = tomllib.loads(p.read_text(encoding="utf-8"))
        crate = data.get("package", {}).get("name")
        if crate:                      # 根是 virtual manifest，没有 [package]
            names.add(crate)
        else:
            crate = "<root>"
        walk_manifest(p, crate, is_root=not data.get("package"))

    # ③ 构建脚本：不需要任何 manifest 条目
    build_scripts = sorted(
        str(p.relative_to(root)).replace("\\", "/")
        for p in root.glob("crates/**/build.rs")
    )
    # ⑥ 锁：用 tomllib 读，只认 [[package]]（[[patch.unused]] 不再误计）
    lock = tomllib.loads((root / "Cargo.lock").read_text(encoding="utf-8"))
    lock_packages = sorted((p["name"], p["version"]) for p in lock.get("package", []))
    return {"external": external, "build": build, "patches": patches, "names": names,
            "build_scripts": build_scripts, "lock": lock_packages, "path_issues": path_issues}


def main() -> int:
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    root = Path(argv[0] if argv else ".").resolve()
    write = "--write-baseline" in sys.argv
    bfile = root / "scripts" / "deps-baseline.json"
    cur = scan(root)
    base = json.loads(bfile.read_text(encoding="utf-8")) if bfile.is_file() else {
        "deps": {}, "build_deps": {}, "build_scripts": [], "patches": [], "lock_packages": []}
    lock_map = dict(cur["lock"])

    if write:
        merged = {"deps": {}, "build_deps": {}, "build_scripts": cur["build_scripts"],
                  "patches": cur["patches"], "lock_packages": [list(x) for x in cur["lock"]]}
        added = []
        for group, key in ((cur["external"], "deps"), (cur["build"], "build_deps")):
            for crate, deps in sorted(group.items()):
                merged[key][crate] = {}
                for dep in sorted(deps):
                    old = ((base.get(key) or {}).get(crate) or {}).get(dep)
                    why = old if isinstance(old, str) else (old or {}).get("why") if old else None
                    if not why:
                        why = PLACEHOLDER
                        added.append(f"{crate} → {dep}")
                    entry = {"why": why}
                    if dep in lock_map:
                        entry["version"] = lock_map[dep]
                    merged[key][crate][dep] = entry
        bfile.write_text(json.dumps(merged, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        todo = [f"{c}→{d}" for c, dd in merged["deps"].items() for d, e in dd.items()
                if PLACEHOLDER in e["why"]]
        print(f"已重写基线：{bfile.relative_to(root)}")
        print(f"  新增需填理由：{len(added)} 项" + ("（" + "、".join(added) + "）" if added else ""))
        if todo:
            print(f"  ⚠️ 占位符 {len(todo)} 项，提交前必须填实：" + "、".join(todo))
        return 0

    fail = False
    for crate, deps in sorted(cur["external"].items()):
        known = base.get("deps", {}).get(crate) or {}
        for dep in sorted(deps):
            entry = known.get(dep)
            if entry is None:
                fail = True
                print(f"❌ 新增外部依赖：{crate} → {dep}\n"
                      f"   登记流程：确认它是否改变确定性契约 → 跑 --write-baseline → 填实理由 → 一起提交")
            else:
                why = entry if isinstance(entry, str) else entry.get("why", "")
                if not why or PLACEHOLDER in why:
                    fail = True
                    print(f"❌ {crate} → {dep} 的登记理由仍是占位符，请填实")
        for dep in sorted(set(known) - set(deps)):
            print(f"NOTE {crate} → {dep} 已不在依赖表里，可顺手从基线删掉")
    for crate, deps in sorted(cur["build"].items()):
        known = base.get("build_deps", {}).get(crate) or {}
        for dep in sorted(deps):
            if dep not in known:
                fail = True
                print(f"❌ 新增构建依赖：{crate} → {dep}\n"
                      f"   构建工具直接依赖会改变交叉编译与哈希门的成立条件；确需引入先裁决再登记")
    for path in cur["build_scripts"]:
        if path not in base.get("build_scripts", []):
            fail = True
            print(f"❌ 新增构建脚本：{path}——build.rs 可绕过 manifest 直接调用外部工具"
                  f"（cc/protoc/nasm…），属「构建工具直接依赖」的隐形入口；确需引入先裁决再登记")
    for tag in cur["patches"]:
        if tag not in base.get("patches", []):
            fail = True
            print(f"❌ 新增补丁/替换：{tag}——[patch]/[replace] 可在名字不变的情况下换实现")
    for issue in cur["path_issues"]:
        fail = True
        print(f"❌ path 依赖不可见：{issue}\n"
              f"   path 依赖必须落在仓内且指向被扫描的 manifest，否则它的依赖图不在门内")
    known_lock = {tuple(x) for x in base.get("lock_packages", [])}
    lock_names = {n for n, _ in cur["lock"]}
    # ② 补：manifest ↔ 锁一致性。CI 里 cargo 不带 --locked，锁没更新时会**静默拉取**新依赖——
    # 这条纯文本对账替代了「必须给每条 cargo 加 --locked」的侵入式改法。
    for crate, deps in sorted(cur["external"].items()):
        for dep in sorted(deps):
            if dep not in lock_names:
                fail = True
                print(f"❌ {crate} 声明了外部依赖 {dep}，但 Cargo.lock 里没有它——"
                      f"是不是忘了提交锁更新？（CI 的 cargo 不带 --locked，会静默拉取）")
    for name, version in cur["lock"]:
        if name in cur["names"]:
            continue
        if (name, version) not in known_lock:
            if any(n == name for n, _ in known_lock):
                print(f"NOTE 锁内版本变化：{name} {dict(known_lock).get(name)} → {version}"
                      f"（不阻断；与位级一致性相关的依赖由跨平台哈希门裁决）")
            else:
                fail = True
                print(f"❌ Cargo.lock 出现未登记包：{name} {version}"
                      f"（传递闭包随新增依赖增长；确属已登记依赖的传递项就跑 --write-baseline 记账）")
    if len(cur["lock"]) < len(base.get("lock_packages", [])):
        print("NOTE 锁文件比基线短了，可顺手收紧基线")

    if not fail:
        direct = sum(len(v) for v in cur["external"].values())
        bd = sum(len(v) for v in cur["build"].values())
        print(f"✅ 依赖红线通过（{len(cur['names'])} 个 crate：{direct} 项外部依赖均已在基线内、"
              f"{bd} 项本仓构建依赖、{len(cur['build_scripts'])} 个构建脚本、"
              f"{len(cur['patches'])} 项补丁；Cargo.lock {len(cur['lock'])} 项）")
    else:
        print("—— 条款出处：README「依赖 DAG」 + docs/SPEC.md §5（跨平台位级一致的前提）")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
