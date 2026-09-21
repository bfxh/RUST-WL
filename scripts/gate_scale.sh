#!/usr/bin/env bash
# **规模档门**（10 万动态 + 10 万静态，`SPEC.md` §3 最低通过档 / §12.1-1 规模回归）。
#
# 为什么有这个脚本：`M1-EXIT.md` §2.3 登记的条件之一——规模档此前**没有任何冻结基线**，
# 于是"劣化 >10% 即阻断"这条身份门在规模档上是**空的**（会跑的自动门只有金样三场景与
# 默认档 6×6×6）。本脚本把该档的**确定性量**钉住，把**计时量**降为"只报 + 粗门"。
#
# 判据分两类（这是本脚本最重要的设计决定，别改）：
#   ① **确定性量（精确断言）**：NaN / 深穿透 / 峰值流形 / 峰值接触点 / 峰值候选 / warm 槽数 /
#      活跃 tick 数 / 末态 awake。它们**只由场景与码决定** ⇒ 有意改动必须同步改这里的冻结值
#      （走 ADR-0004 换代：记旧值/新值/理由，见 `OPEN-PROBLEMS.md` T5 的同类流程）。
#   ② **计时量（只报 + 粗门）**：全期均 / 最差 tick。按本仓测量协议（`KNOWLEDGE`/记忆：
#      "本机现在测不出 5–10% 级"、"性能类门要机器级独占"），>10% 的判定在本机**不可信**
#      ⇒ 默认只在 **>2×** 时红（抓真正的崩坏），**>1.5×** 时黄；要按 >10% 严判请跑
#      `SCALE_STRICT=1`（须在**安静机**上，且小改动请用 `scripts/ab_perf.sh` 交错 A/B）。
#
# 用法（仓库根或任意目录均可；脚本自己找路径）：
#   bash scripts/gate_scale.sh              # 默认：确定性量精确 + 计时粗门
#   SCALE_STRICT=1 bash scripts/gate_scale.sh   # 计时按 >10% 严判（安静机）
#   SCALE_FREEZE=1 bash scripts/gate_scale.sh   # 换代：打印可直接粘贴的冻结值块
# 退出码：0 = 全绿；1 = 判据不达标；2 = 找不到路径；3 = **没有真正做判定**（解析不到摘要行）。
#
# ⚠️ 跑它时别并发其它构建/门（本仓有并发开发者；计时段会互相污染）。
# ⚠️ Windows/Git-Bash 下 `m1_scale` 的逐 tick 行走 **stderr**，摘要行走 stdout ⇒ 本脚本
#    把两者一起收进日志再解析。
#
# 冻结值出处：2026-09-22 本机 8 线程、`m1_scale 8 102400 100000 600 16`（`M1-EXIT.md` §5）。
#   全期均 35.32 ms/tick ｜ 最差 410.53 ms ｜ 活跃相均 196.08 ms/tick（活跃 103/600 tick）
#   峰值：流形 321132 ｜ warm 槽 400000 ｜ 接触点 1284528 ｜ 候选 1040623

set -u

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
cd "$root" || { echo "找不到仓库根（${root}）" >&2; exit 2; }

# —— 冻结基线（换代时改这里）——
F_NAN=0
F_DEEP=0
F_MF=321132          # 峰值流形
F_WARM=400000        # warm 槽
F_PTS=1284528        # 峰值接触点
F_CAND=1040623       # 峰值候选
F_ACTIVE=103         # 活跃 tick 数（awake>0）
F_AWAKE_END=0        # 末态 awake
T_AVG=35.32          # 计时：全期均 ms/tick（软门）
T_WORST=410.53       # 计时：最差 tick ms（软门）

log="${SCALE_LOG:-C:/vxl-wl-target-gold/gate_scale.log}"
mkdir -p "$(dirname "$log")" 2>/dev/null || true

echo "== 规模档门：m1_scale 8 102400 100000 600 16（日志 ${log}）"
cargo run --release -q -p vxl-phys --example m1_scale -- 8 102400 100000 600 16 >"$log" 2>&1
run_rc=$?
if [ "$run_rc" -ne 0 ]; then
    echo "❌ 场景运行失败（exit ${run_rc}）" >&2
    tail -n 10 "$log" >&2 || true
    exit "$run_rc"
fi

# 解析（缺任一项 = 没真正判定 ⇒ exit 3，宁可红也不要"静默通过"）。
grab() { grep -m1 -oE "$1" "$log" | head -1; }
line_time=$(grep -m1 -E '平均 +[0-9.]+ ms/tick' "$log" || true)
line_act=$(grep -m1 -E '活跃期（awake>0 的' "$log" || true)
[ -n "$line_time" ] && [ -n "$line_act" ] || {
    echo "❌ 解析不到摘要行 —— 没有真正做判定（输出格式变了？）" >&2
    tail -n 8 "$log" >&2 || true
    exit 3
}

avg=$(echo "$line_time" | grep -oE '平均 +[0-9.]+' | grep -oE '[0-9.]+')
worst=$(echo "$line_time" | grep -oE '最差 +[0-9.]+' | grep -oE '[0-9.]+')
nan=$(echo "$line_time" | grep -oE 'NaN +[0-9]+' | grep -oE '[0-9]+')
deep=$(echo "$line_time" | grep -oE 'deep +[0-9]+' | grep -oE '[0-9]+')
active=$(echo "$line_act" | sed -E 's/.*>0[^0-9]*([0-9]+).*/\1/')   # 注意：不能用 grep -oE '[0-9]+' 二次过滤——"awake>0" 里的 0 会被一起取出来
mf=$(echo "$line_act" | grep -oE '流形 +[0-9]+' | grep -oE '[0-9]+')
warm=$(echo "$line_act" | grep -oE 'warm 槽 +[0-9]+' | grep -oE '[0-9]+')
pts=$(echo "$line_act" | grep -oE '接触点 +[0-9]+' | grep -oE '[0-9]+')
cand=$(echo "$line_act" | grep -oE '候选 +[0-9]+' | grep -oE '[0-9]+')
awake_end=$(grep -oE "tick +600: .*awake +[0-9]+" "$log" | grep -oE 'awake +[0-9]+' | grep -oE '[0-9]+' | head -1)

if [ "${SCALE_FREEZE:-0}" = "1" ]; then
    echo "===== 换代用（粘贴回脚本头部的冻结基线）====="
    echo "F_NAN=$nan"
    echo "F_DEEP=$deep"
    echo "F_MF=$mf"
    echo "F_WARM=$warm"
    echo "F_PTS=$pts"
    echo "F_CAND=$cand"
    echo "F_ACTIVE=$active"
    echo "F_AWAKE_END=$awake_end"
    echo "T_AVG=$avg"
    echo "T_WORST=$worst"
    exit 0
fi

fail=0
chk() { # 名 实测 冻结
    if [ "$2" = "$3" ]; then printf '  ✅ %-12s %s\n' "$1" "$2"
    else printf '  ❌ %-12s 实测 %s ≠ 冻结 %s\n' "$1" "$2" "$3"; fail=1; fi
}
echo "—— 确定性量（精确）"
chk NaN "$nan" "$F_NAN"
chk 深穿透 "$deep" "$F_DEEP"
chk 峰值流形 "$mf" "$F_MF"
chk warm槽 "$warm" "$F_WARM"
chk 峰值接触点 "$pts" "$F_PTS"
chk 峰值候选 "$cand" "$F_CAND"
chk 活跃tick "$active" "$F_ACTIVE"
chk 末态awake "$awake_end" "$F_AWAKE_END"

echo "—— 计时量（软门；协议见脚本头）"
ratio_avg=$(awk -v a="$avg" -v b="$T_AVG" 'BEGIN{ if (b+0==0) print "n/a"; else printf "%.3f", a/b }')
ratio_worst=$(awk -v a="$worst" -v b="$T_WORST" 'BEGIN{ if (b+0==0) print "n/a"; else printf "%.3f", a/b }')
printf '  %-12s 实测 %s（冻结 %s，比 %s×）\n' 全期均 "$avg" "$T_AVG" "$ratio_avg"
printf '  %-12s 实测 %s（冻结 %s，比 %s×）\n' 最差tick "$worst" "$T_WORST" "$ratio_worst"

strict="${SCALE_STRICT:-0}"
verdict_time=$(awk -v ra="$ratio_avg" -v rw="$ratio_worst" -v strict="$strict" 'BEGIN{
    r = (ra+0 > rw+0) ? ra+0 : rw+0;
    if (strict == "1")      { if (r > 1.10) print "FAIL"; else print "OK" }
    else                    { if (r > 2.00) print "FAIL"; else if (r > 1.50) print "WARN"; else print "OK" }
}')
case "$verdict_time" in
    FAIL) echo "  ❌ 计时 >$([ "$strict" = 1 ] && echo '10%（严判）' || echo '2×（粗门）') ⇒ 按协议先在**安静机**复测："
          echo "     bash scripts/ab_perf.sh <旧ref> 3 '平均' cargo run --release -p vxl-phys --example m1_scale -- 8 102400 100000 600 16"
          fail=1 ;;
    WARN) echo "  ⚠️ 计时超冻结值 1.5×（未到红）——先看是否机器有负载/并发构建" ;;
    *)    echo "  ✅ 计时在门内" ;;
esac

if [ "$fail" -eq 0 ]; then
    echo "✅ 规模档门全绿（确定性量逐项一致；计时$([ "$verdict_time" = WARN ] && echo '有黄项' || echo '在门内')）"
    exit 0
fi
echo "❌ 规模档门不达标（改动确定性量 ⇒ 按 ADR-0004 换代并记理由；计时超标 ⇒ 安静机 ab_perf 复测）" >&2
exit 1
