#!/usr/bin/env bash
# **交错 A/B 性能对拍**（本仓唯一可信口径的可执行版）。
#
# 为什么有它：这台机器上单跑同码的漂移可达 ±5%，并发开发时同一场景的墙钟方差实测 **±25%**
# ⇒ "改前跑一次、改后跑一次"分辨不出 5–10% 的改动（C4/C7 两条候选就这么被噪声吃掉了）。
# 本仓既定协议是**交错 A/B**：旧 ref 放只读 worktree + 独立 target 目录，两臂**交替**跑 N 轮，
# 用**成对**读数比较（负载漂移在交替中被抵消）。此前靠手工敲，且踩过 worktree 的坑
#（worktree 里要单独编你要测的 example）⇒ 这里固定下来。
#
# 用法：
#   bash scripts/ab_perf.sh <旧ref> <N> <grep 模式> <命令…>
# 例：
#   bash scripts/ab_perf.sh HEAD~1 5 'ns/点' \
#       cargo run --release -q -p vxl-phys --example m1_islands -- 12000 60 8
#
# 说明：
# - `<命令…>` 在**两臂各自的工作目录**里跑（worktree 是旧 ref 的完整检出）；
# - `grep 模式` 用来从输出里抓一个数（取**首个**匹配行里的**最后一个浮点数**）；
# - 打印每轮两臂的读数、各自均值与 **A/B 比值**；比值 >1 表示"当前树更快"…
#   ⚠️ 不是：本脚本的 A 臂 = 旧 ref，B 臂 = 当前树；`B/A` 才是"当前树相对旧 ref 的加速"。
# - **判读纪律**：只有|两臂均值之差| **明显大于**两臂各自的极差时才算有结论；
#   否则打印 `inconclusive` 提示（宁可说测不出，也不要报假收益）。
# - 测完自动删除 worktree（`AB_KEEP=1` 保留供检查）。
#
# ⚠️ **两臂必须各用独立的 target 目录**（默认 `C:/vxl-ab-target/{a,b}`，`AB_TARGET_DIR` 可改根）。
#   工具第一版让两臂共用同一个 ⇒ A 臂先编出的**旧** `vxl_phys_core` rlib 被 B 臂（HEAD）误用，
#   B 直接编不过（`no interop in vxl_phys_core`）——这正是本仓 A/B 协议里
#   "worktree + **独立** target 目录"那条（踩过一次，写死在这里）。

set -u

if [ $# -lt 4 ]; then
    echo "用法：bash scripts/ab_perf.sh <旧ref> <N> <grep 模式> <命令…>" >&2
    exit 2
fi

ref="$1"; rounds="$2"; pattern="$3"; shift 3
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
wt="${AB_WORKTREE:-C:/vxl-ab-${ref//[^A-Za-z0-9]/_}}"
ab_root="${AB_TARGET_DIR:-C:/vxl-ab-target}"
# 两臂各自的 target（互不污染；见文首注）
export CARGO_TARGET_DIR="${ab_root}/a"

cleanup() {
    # **护栏**（安全扫描点）：`wt` 来自环境变量 ⇒ 只删"确实是指向本仓的链接工作树"的目录
    # （链接工作树的 `.git` 是**文件**而非目录），且只走 `git worktree remove`
    # （它会自行拒绝非工作树路径）。**不做** `rm -rf "$wt"` 这类裸删。
    if [ "${AB_KEEP:-0}" != "1" ] && [ -n "$wt" ] && [ -f "$wt/.git" ]; then
        git -C "$root" worktree remove --force "$wt" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

echo "== 交错 A/B：A=旧 ref ${ref}（worktree）｜B=当前工作树（${root}）"
echo "   rounds=${rounds}｜抓取模式=${pattern}｜两臂 target 根=${ab_root}（A=/a  B=/b，互不污染）"
git -C "$root" worktree add "$wt" "$ref" --detach >/dev/null 2>&1 || {
    echo "worktree 建立失败（${wt} 已存在？先 git worktree remove）" >&2
    exit 3
}

# 抓一个数：首个匹配行里的**最后一个浮点数**（`last` 用得上：读数常在行尾）。
extract() { # extract <文件>
    # `-o`：先只取**匹配子串**（否则整行里的数都被算进来，行尾的哈希/其它数字会污染）。
    # 于是 grep 模式可以写成"把要的数框住"的样子，如 `平均 +[0-9.]+` ⇒ 取到 66.00。
    grep -m1 -oE "$pattern" "$1" | grep -oE '[0-9]+\.?[0-9]*' | tail -n1
}

# 每臂先**各编一次**（预热 + 保证比的是编译产物，不是编译时间）。**两臂各用独立 target**。
( cd "$wt"   && CARGO_TARGET_DIR="${ab_root}/a" "$@" >/dev/null 2>&1 )
( cd "$root" && CARGO_TARGET_DIR="${ab_root}/b" "$@" >/dev/null 2>&1 )

aset=""; bset=""
for i in $(seq 1 "$rounds"); do
    ( cd "$wt"   && CARGO_TARGET_DIR="${ab_root}/a" "$@" ) >/tmp/ab_a.log 2>&1
    a="$(extract /tmp/ab_a.log)"
    ( cd "$root" && CARGO_TARGET_DIR="${ab_root}/b" "$@" ) >/tmp/ab_b.log 2>&1
    b="$(extract /tmp/ab_b.log)"
    echo "  轮 ${i}：A=${a:-?}  B=${b:-?}"
    aset="${aset} ${a:-0}"
    bset="${bset} ${b:-0}"
done

python - "$aset" "$bset" <<'PY'
import sys
a = [float(x) for x in sys.argv[1].split()]
b = [float(x) for x in sys.argv[2].split()]
ma, mb = sum(a)/len(a), sum(b)/len(b)
ra, rb = max(a)-min(a), max(b)-min(b)
print(f"  A 均值 {ma:.4f}（极差 {ra:.4f}）｜B 均值 {mb:.4f}（极差 {rb:.4f}）")
if mb > 0 and ma > 0:
    print(f"  B/A 比值 = {ma/mb:.3f}（>1 = 当前树读数更小/更快，取决于该指标方向）")
diff = abs(mb-ma)
if diff <= max(ra, rb):
    print("  ⚠️ inconclusive：两臂均值之差 ≤ 两臂极差 ⇒ 本机噪声内测不出差异，别当结论")
else:
    print("  ✅ 差异超出两臂极差 ⇒ 可作为结论（仍建议与行为门同批核对）")
PY
