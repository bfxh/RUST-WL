#!/usr/bin/env bash
# **一条命令跑全量验证**（`RECIPES.md` §门禁链 + §行为门 + §金样门）。
#
# 为什么有它：门禁链此前是"文档里的命令清单"，每轮靠人照着敲 ⇒ 口径会漂、也容易漏项
# （金样读数就因为**没有任何门看着**而陈旧到差一倍，见 OPEN-PROBLEMS T5）。
# 本脚本把"跑什么、怎么判绿"固定成一处，逐项贴退出码、任一失败先打日志尾部再非零退出。
#
# 用法：
#   bash scripts/gate_all.sh                 # 全量（含金样门 + 规模档门，约 3-4 分钟）
#   SKIP_GOLD=1 bash scripts/gate_all.sh     # 跳过金样门（只跑主仓 + 行为门）
# 退出码：0 = 全绿；非 0 = 第一处失败项的退出码。
#
# ⚠️ 跑它时**别编辑源码**（fmt/test/clippy 期间改文件会让它以编译错失败——踩过），
#    也**别并发其它 cargo 构建**（金样门会因 Windows 锁 `.exe` 报 os error 5——踩过）。
#    ⇒ 后半段的计时门由**机器级锁**（~/.rx/perf-gate.lock）兜住：拿不到就等，等到超时
#      以 exit 8 明确报「未能判定」（不是通过）。

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

step clippy 0 cargo clippy --workspace --all-targets -- -D warnings -D clippy::todo -D clippy::unimplemented -D clippy::dbg_macro -D clippy::mem_forget -D clippy::undocumented_unsafe_blocks -D clippy::let_underscore_must_use
step test 0 cargo test --release
step vocab 0 bash scripts/vocab_scan.sh .
# 纪律扫描（forbid 覆盖 / 零 unsafe / 零 f64 / 零 SIMD 内建 / 零 fast-math）——
# 此前只在 CI 里跑，本地漏跑就会「本地绿、CI 红」（本地与 CI 不许漂移）。
step discipline 0 bash scripts/discipline_scan.sh .
# 上帝对象门（文件/函数/类型尺寸；**棘轮只准减**，基线在 god-baseline.json）——
# 脚本可移植（`scripts/god_gate.py` + `god.gate.json`），`--list --top N` 看排行。
# 先跑**门自己的金丝雀**（掩码双向 / 成员数 / 包含面）——判据坏了门就是摆设。
step god_selftest 0 python scripts/god_gate.py --selftest
step god 0 python scripts/god_gate.py --root .
# 依赖红线（外部依赖/构建依赖只准减；构建脚本与补丁单独对账；理由登记在 spec 同目录的基线里）
step deps_lock 0 python scripts/deps_lock.py
# CI 形状锁：硬门、汇总门 needs、安全/成本基线、action 钉 SHA 不被悄悄退役
step ci_shape 0 bash scripts/ci_shape_lock.sh .
step typos 0 /c/vxl-wl-tools/typos.exe .

echo "-- 行为门（三命令）"
# **计时类门必须独占**：determinism / m0_gates / m1_islands / 金样门的判据都含时间，同机
# 别的 cargo 构建会把读数弄脏（上面那条「别并发其它 cargo 构建」的警告，2026-09-21 起由
# 这把**机器级锁**执行：锁在 ~/.rx/perf-gate.lock，跨仓可见，与 scripts/perf_lock 语义一致）。
LOCK="${PERF_LOCK:-$HOME/.rx/perf-gate.lock}"
acquire_timed() {
    mkdir -p "$(dirname "$LOCK")" 2>/dev/null || true
    for _ in $(seq 1 120); do
        if ( set -o noclobber; printf 'pid=%s ts=%s\n' "$$" "$(date +%s)" >"$LOCK" ) 2>/dev/null; then
            trap 'rm -f "$LOCK"' EXIT INT TERM
            return 0
        fi
        # 过期回收：锁文件 30 分钟没动过（持有者崩了也会过期），不让它永久卡住
        if find "$LOCK" -mmin +30 >/dev/null 2>&1; then
            echo "⚠️  回收过期锁：$LOCK" >&2
            rm -f "$LOCK"
            continue
        fi
        echo "⏳ 机器忙：$LOCK 被别的构建占着——计时门等独占（5s 一轮，最多 10 分钟）" >&2
        sleep 5
    done
    echo "❌ 拿不到独占锁（$LOCK）：性能类门的判据是时间，此时判红绿都会被污染——等本机空下来再重跑" >&2
    exit 8
}
acquire_timed

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

# **规模档门**（10 万动态+10 万静态，SPEC §3 最低通过档 / §12.1-1 规模回归）：
# 确定性量**逐项精确断言**（NaN/深穿透/峰值流形/warm 槽/峰值接触点/峰值候选/活跃 tick/末态 awake）
# + 计时**软门**（默认只在 >2× 时红、>1.5× 黄；要按 >10% 严判须 `SCALE_STRICT=1` 且安静机）。
# 为什么加它：`M1-EXIT.md` §2.3 登记过——"劣化 >10% 阻断"在规模档上**此前是空的**
#（会跑的自动门只有金样三场景与默认档 6×6×6）。
step scale 0 bash scripts/gate_scale.sh

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
