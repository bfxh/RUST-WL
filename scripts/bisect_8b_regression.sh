#!/usr/bin/env bash
# **8B 回归二分定位**（一次性诊断脚本；`OPEN-PROBLEMS` P3 那 25%）。
#
# 背景：同机同窗口 A/B 已确立——`5e233eb`（52.6 ms）→ `f41e5b8`/HEAD（≈65.8 ms）**慢 25%**，
# 区间 241 个提交、全部在本会话之前。本脚本按"两臂都有的『平均 ms/tick』"对该区间做二分：
# 每轮取中点 `mid`，跑 `mid`（A 臂）↔ 当前工作树（B 臂，慢端参照）；
# A ≈ 快端 ⇒ 回归在中点之后；A ≈ 慢端 ⇒ 回归在中点或之前。
#
# 用法：`bash scripts/bisect_8b_regression.sh [轮数]`（默认 3 轮；区间 ≤ 8 提交时提前停止）
# ⚠️ 每轮要为一个新提交建 worktree 并全量编译（约 5–8 分钟）⇒ 3 轮约 20–30 分钟。
# ⚠️ 这是**一次性诊断**脚本，不是常设工具（放在 `scripts/` 只是为了让命令可复现）。

set -u
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root" || exit 2

rounds="${1:-3}"
FAST=5e233eb   # 快端（实测 52.6 ms）
SLOW=f41e5b8   # 慢端（实测 ≈65.8 ms，与 HEAD 无差异）
THRESHOLD=59   # 分界：<59 视为"快端"，否则"慢端"（两端差 13 ms，阈值居中留足余量）

echo "== 8B 回归二分：FAST=$FAST  SLOW=$SLOW  轮数=$rounds  分界=${THRESHOLD}ms"
for r in $(seq 1 "$rounds"); do
    mapfile -t list < <(git rev-list --reverse "$FAST..$SLOW")
    n=${#list[@]}
    if [ "$n" -le 8 ]; then
        echo "== 区间已收窄到 $n 个提交（旧→新）："
        printf '   %s\n' "${list[@]}"
        exit 0
    fi
    mid="${list[$((n / 2))]}"
    echo "== 第 $r 轮：区间 $n 提交，中点 $mid"
    out="$(bash scripts/ab_perf.sh "$mid" 3 '平均 +[0-9.]+' \
        cargo run --release -q -p vxl-phys --example m1_scale -- 8 102400 100000 300 16 2>&1)"
    echo "$out" | grep -E '轮 |均值|比值|inconclusive|结论' || true
    a="$(echo "$out" | grep -oE 'A 均值 [0-9.]+' | grep -oE '[0-9.]+' | head -1)"
    if [ -z "$a" ]; then
        echo "⚠️ 第 $r 轮没抓到 A 臂读数（编译失败？）—— 停止，人工看上面输出" >&2
        exit 3
    fi
    # **inconclusive 必须停**：两臂噪声内无差异时，A 的绝对值不足以判定快/慢
    # （本脚本第一版直接按阈值判 ⇒ 在 ±12 ms 噪声的那轮做了错误收窄）。
    if echo "$out" | grep -q 'inconclusive'; then
        echo "⚠️ 第 $r 轮 inconclusive（两臂噪声内无差异）⇒ **就此停止收窄**（区间、候选照旧）" >&2
        exit 4
    fi
    if awk "BEGIN{exit !($a < $THRESHOLD)}"; then
        FAST="$mid"
        echo "  ⇒ A=$a ms（快端）⇒ 回归在 $mid **之后**"
    else
        SLOW="$mid"
        echo "  ⇒ A=$a ms（慢端）⇒ 回归在 $mid **或之前**"
    fi
done
mapfile -t list < <(git rev-list --reverse "$FAST..$SLOW")
echo "== $rounds 轮结束，剩余区间 ${#list[@]} 个提交（旧→新）："
printf '   %s\n' "${list[@]}"
