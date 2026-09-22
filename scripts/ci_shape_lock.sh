#!/usr/bin/env bash
# CI 形状锁：**门禁只许加强，不许悄悄退役**（把 ci.yml 的承重结构变成可执行断言）。
#
# 为什么需要它：ci.yml 是纯文本，一个「顺手删掉几行」的改动就能让 miri/loom/TSan/ASan/
# 确定性哈希/汇总门无声消失，而没人会立刻发现。这里把「必须有」的东西钉住——
# 谁删谁红。与 scripts/local_gate.py 的快档一起跑，也在 CI 的静态层跑。
#
# 用法：scripts/ci_shape_lock.sh [根目录，默认 .]
# 退出码：0 = 通过；1 = 缺东西（CI 被削弱）。
set -u
ROOT="${1:-.}"
CI="$ROOT/.github/workflows/ci.yml"
FAIL=0

[ -f "$CI" ] || { echo "❌ 找不到 $CI"; exit 1; }

need_text() { # 说明 + 字面量（-F 精确匹配）
  if ! grep -qF -- "$2" "$CI"; then
    echo "❌ $1（缺：$2）"
    FAIL=1
  fi
}

# ① 提交门四件套 + 汇总门：job 名一个都不能少
for job in "miri (core 子集)" "loom (并发原语)" "TSan (并行路径)" "ASan (core + 门槛短跑)" \
           "汇总（提交门）" "跨平台哈希比对"; do
  need_text "CI 少了承重 job：$job" "name: $job"
done

# ② 各门的关键命令
need_text "纪律扫描不在 CI 里"            "bash scripts/discipline_scan.sh"
need_text "词汇扫描不在 CI 里"            "bash scripts/vocab_scan.sh"
need_text "依赖红线不在 CI 里"            "python3 scripts/deps_lock.py"
need_text "cargo-deny 不在 CI 里"         "cargo deny check"
need_text "cargo-machete 不在 CI 里"      "cargo machete"
need_text "rustdoc 不再 warnings 即错误"  "RUSTDOCFLAGS: -D warnings"
need_text "miri 不在 CI 里"               "miri test -p vxl-phys-core"
need_text "loom 不在 CI 里"               "--cfg loom"
need_text "TSan 不在 CI 里"               "-Zsanitizer=thread"
need_text "ASan 不在 CI 里"               "-Zsanitizer=address"
need_text "确定性 10 轮不在 CI 里"        "--example determinism"
need_text "门槛场景不在 CI 里"            "--example m0_gates"

# ③ 汇总门必须继续 needs 这些前置门（skipped 被 GitHub 视作通过 ⇒ 漏一个门就漏一片）
for dep in static-text static-deps static-code matrix miri loom tsan asan \
           determinism determinism-arm64 hash-compare; do
  need_text "汇总门的 needs 少了：$dep" "      - $dep"
done

# ④ 安全与成本基线（zizmor 审计过的三条 + 成本控制四条）
need_text "workflow 级最小权限没了"       "permissions:"
need_text "checkout 凭据不落盘没了"       "persist-credentials: false"
need_text "同分支并发取消没了"            "cancel-in-progress: true"
need_text "文档/工具目录跳过没了"          "paths-ignore:"

# ⑤ 第三方 action 必须钉 commit SHA（唯一具名例外：dtolnay/rust-toolchain——它的 ref
#    就是工具链名 stable/nightly，钉 SHA 等于钉错东西，行内注释已写明理由）
floaters="$(grep -nE '^\s*-? *uses: ' "$CI" \
  | grep -v 'dtolnay/rust-toolchain' \
  | grep -vE 'uses: [A-Za-z0-9._/-]+@[0-9a-f]{40}' || true)"
if [ -n "$floaters" ]; then
  echo "❌ 有 action 没钉 commit SHA（浮动 tag 可被上游重指）："
  printf '%s\n' "$floaters"
  FAIL=1
fi

# ⑤b install-action 装的工具必须**钉版本**（`tool: 名@版本`）。默认 @latest ⇒ 工具自己发新版
#     就能把分支判红而代码没动过（2026-09-21 实测：日志里写着 `installing cargo-machete@latest`）。
#     版本号取当次 CI 日志里的实际安装版本；升级是**一次有意动作**，不是被动接受。
#     匹配前先剥掉行尾注释——首版没剥，连 `tool: x@1.30.1  # 说明` 都被判成"没钉"（假红）。
VERPAT="@[0-9]+(\.[0-9]+)*"
unpinned="$(grep -nE '^\s+tool: ' "$CI" | sed 's/#.*$//' \
  | grep -vE "tool: [a-z0-9_-]+$VERPAT(,[a-z0-9_-]+$VERPAT)*[[:space:]]*$" || true)"
if [ -n "$unpinned" ]; then
  echo "❌ install-action 的工具有没钉版本的（改 tool: 名为 名@x.y.z）："
  printf '%s\n' "$unpinned"
  FAIL=1
fi

# ⑥ **本地门链不许退化**：一条命令链里的承重件——三步静态门（discipline / deps_lock /
#    ci_shape）+ 计时门的**机器级独占锁**（2026-09-21 实测：另一个仓的 cargo build 会让
#    计时门假红 ⇒ 那把锁是承重的，谁删谁红）+ 提交通道的两份钩子。
GA="$ROOT/scripts/gate_all.sh"
if [ -f "$GA" ]; then
  for needle in "scripts/discipline_scan.sh" "scripts/deps_lock.py" "scripts/ci_shape_lock.sh" \
                "perf-gate.lock" "acquire_timed"; do
    grep -qF -- "$needle" "$GA" || { echo "❌ 本地门链退化：gate_all.sh 缺 $needle"; FAIL=1; }
  done
else
  echo "❌ 本地门链缺 scripts/gate_all.sh（一条命令跑全量验证的入口）"
  FAIL=1
fi
for hook in .githooks/pre-commit .githooks/pre-push; do
  [ -f "$ROOT/$hook" ] || { echo "❌ 提交通道缺 $hook"; FAIL=1; }
done

if [ "$FAIL" -eq 0 ]; then
  echo "✅ CI 形状锁通过（四提交门 + 汇总门 needs 完整 / 关键命令在 / 安全与成本基线在 /" \
       "action 全钉 SHA / 本地门链与钩子齐全）"
fi
exit "$FAIL"
