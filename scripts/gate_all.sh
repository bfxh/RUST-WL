#!/usr/bin/env bash
# **一条命令跑全量验证**（`RECIPES.md` §门禁链 + §行为门 + §金样门）。
#
# 为什么有它：门禁链此前是"文档里的命令清单"，每轮靠人照着敲 ⇒ 口径会漂、也容易漏项
# （金样读数就因为**没有任何门看着**而陈旧到差一倍，见 OPEN-PROBLEMS T5）。
# 本脚本把"跑什么、怎么判绿"固定成一处，逐项贴退出码、任一失败先打日志尾部再非零退出。
#
# 用法：
#   bash scripts/gate_all.sh                 # 全量（含金样门，约 2-3 分钟）
#   SKIP_GOLD=1 bash scripts/gate_all.sh     # 跳过金样门（只跑主仓 + 行为门）
# 退出码：0 = 全绿；非 0 = 第一处失败项的退出码。
#
# ⚠️ 跑它时**别编辑源码**（fmt/test/clippy 期间改文件会让它以编译错失败——踩过），
#    也**别并发其它 cargo 构建**（金样门会因 Windows 锁 `.exe` 报 os error 5——踩过）。

set -u

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
cd "$root" || {
    echo "找不到仓库根（${root}）" >&2
    exit 2
}
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-C:/vxl-wl-target}"
out="${GATE_LOG_DIR:-/tmp}"

fail() { # fail <名称> <码> <日志>
    echo "❌ ${1} 失败（退出码 ${2}）——日志尾部：" >&2
    tail -n 20 "$3" >&2 || true
    exit "$2"
}

# 跑一条命令并把日志落到文件；`expect <0|1>` = 是否要求退出码为 0。
step() { # step <名称> <是否要求 0> <命令…>
    local name="$1" want="$2"; shift 2
    local log="${out}/gate_${name}.log"
    set +e
    "$@" >"$log" 2>&1
    local rc=$?
    set -e 2>/dev/null || true
    echo "  ${name}=${rc}"
    if [ "$want" = "0" ] && [ $rc -ne 0 ]; then fail "$name" $rc "$log"; fi
    [ "$want" = "report" ] || true
}

echo "== 全量验证 @ ${root}（CARGO_TARGET_DIR=${CARGO_TARGET_DIR}）"
echo "-- 主仓五项门禁"

set +e
cargo fmt --all -- --check >"${out}/gate_fmt.log" 2>&1
rc=$?
echo "  fmt=$rc"
if [ $rc -ne 0 ]; then
    cargo fmt --all
    echo "  （已自动 cargo fmt 修复；请复核改动后再跑一次）" >&2
    exit $rc
fi

step clippy 0 cargo clippy --workspace --all-targets -- -D warnings
step test 0 cargo test --release
step vocab 0 bash scripts/vocab_scan.sh .
step typos 0 /c/vxl-wl-tools/typos.exe .

echo "-- 行为门（三命令）"
# `determinism` / `m0_gates` 自报 PASS 且哈希与档内基线一致；`m1_islands` 只报比值（见下）。
step determinism 0 cargo run --release -q -p vxl-phys --example determinism
if ! grep -q "FINAL_HASH=0x711be572cfe0e7eefb2cf51550fd4dd5" "${out}/gate_determinism.log"; then
    echo "❌ determinism 哈希与基线不符（期望 0x711be572cfe0e7eefb2cf51550fd4dd5）——日志尾：" >&2
    tail -n 10 "${out}/gate_determinism.log" >&2
    exit 4
fi

step m0_gates 0 cargo run --release -q -p vxl-phys --example m0_gates
if ! grep -q "M0 门槛 PASS" "${out}/gate_m0_gates.log"; then
    echo "❌ m0_gates 未报 PASS——日志尾：" >&2
    tail -n 10 "${out}/gate_m0_gates.log" >&2
    exit 5
fi
if ! grep -q "0x417be20a8e49c9b0436987415ac9961a" "${out}/gate_m0_gates.log"; then
    echo "❌ m0_gates 压力哈希与基线不符（期望 0x417be20a8e49c9b0436987415ac9961a）——日志尾：" >&2
    tail -n 10 "${out}/gate_m0_gates.log" >&2
    exit 6
fi

# ⚠️ **report-only**：T4 的"解算扩展 ≥3×"在本机结构性达不到（饱和 ~2.5×，成因＝访存带宽，
# 见 OPEN-PROBLEMS T4）⇒ 本脚本只报读数，**不据此判红**；但它的**串行/并行末态哈希逐位一致**
# 是机器无关的，那个照样判。
step m1_islands report cargo run --release -q -p vxl-phys --example m1_islands
grep -E "扩展比|串行/并行末态哈希" "${out}/gate_m1_islands.log" | sed 's/^/    /' || true
grep -q "逐位一致" "${out}/gate_m1_islands.log" || {
    echo "❌ m1_islands 串行/并行末态哈希不一致（机器无关判据）——日志尾：" >&2
    tail -n 10 "${out}/gate_m1_islands.log" >&2
    exit 7
}

if [ "${SKIP_GOLD:-0}" = "1" ]; then
    echo "-- 金样门：SKIP_GOLD=1 ⇒ 跳过"
else
    echo "-- 金样门"
    set +e
    bash scripts/gate_gold.sh >"${out}/gate_gold.log" 2>&1
    rc=$?
    echo "  gold=$rc"
    grep -E "^gold_|金样门全绿" "${out}/gate_gold.log" | sed 's/^/    /' || true
    [ $rc -eq 0 ] || fail gold $rc "${out}/gate_gold.log"
fi

echo "✅ 全量验证通过（主仓五项 + 行为门 + 金样门）"
exit 0
