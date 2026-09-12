#!/usr/bin/env bash
# 词汇禁令扫描（裁决 61；施工令 T4/T6）——引擎仓源码/文档不得出现宿主业务词汇。
#
# 用法：scripts/vocab_scan.sh [根目录，默认 .]
# 退出码：0 = 通过；1 = 命中违规（CI 阻断）。
#
# 词表与白名单（裁决 61；crate 名豁免已按用户裁决 2026-09-12 删除）：
#   - vehicle / module / cell / 404：独立单词（词边界）形式禁止；
#   - vxl / vxl- 前缀：引擎自身代号，**不在扫描面**（施工令 §3 推荐解释）；
#   - `std::cell`：Rust 标准库路径（非宿主词汇）。

set -u
ROOT="${1:-.}"
FAIL=0

scan() { # $1 = 正则；$2 = 白名单过滤正则（-v）
  local hits
  hits=$(grep -rniE "$1" "$ROOT/crates" "$ROOT/docs" \
      --include='*.rs' --include='*.md' --include='*.toml' 2>/dev/null \
    | grep -vE "$2" || true)
  if [ -n "$hits" ]; then
    echo "❌ 词汇禁令命中：$1"
    echo "$hits"
    FAIL=1
  fi
}

# vehicle：无豁免（原 crate 名已改名 vxl-phys-wheeled / vxl-phys-aero）
scan '\bvehicle\b' '^\s*$'
# module：无豁免
scan '\bmodule\b' '^\s*$'
# cell：豁免 Rust 标准库路径 std::cell
scan '\bcell\b' 'std::cell'
# 404：无豁免
scan '\b404\b' '^\s*$'

if [ "$FAIL" -eq 0 ]; then
  echo "✅ 词汇禁令扫描通过（vehicle/module/cell/404；vxl- 前缀与 std::cell 豁免）"
else
  echo "—— 禁令词表与白名单说明见本脚本头部（施工令 §3）"
fi
exit "$FAIL"
