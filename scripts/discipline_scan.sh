#!/usr/bin/env bash
# 纪律静态扫描（M1 静态层）——把 §5/§7 的**书面纪律**变成可执行断言。
#
# 用法：scripts/discipline_scan.sh [根目录，默认 .]
# 退出码：0 = 通过；1 = 命中违规（CI 阻断）。
#
# 检查项（全部为项目承重墙纪律，任一失守都会静默破坏确定性契约）：
#   ① 每个引擎 crate 的 lib.rs 必须有 `#![forbid(unsafe_code)]`；
#   ② 引擎源码（crates/*/src/**）不得出现真 unsafe（关键字形态）；
#   ③ 引擎源码不得出现 f64（§5 严格 f32：双精度会改变归约次序与结果）；
#   ④ 引擎源码不得使用 core::arch / std::simd（§7：SIMD 走独立路径，
#      主路径保持标量语义）；
#   ⑤ 构建配置不得引入 fast-math 类开关（禁 FMA 重排契约）。
#
# 白名单：examples/ 与 tests/ 不在扫描面（计时/统计与测试辅助可用 f64）；
# 注释里出现这些词是允许的（本脚本只匹配代码形态与关键字）。

set -u
ROOT="${1:-.}"
FAIL=0

src_files() { # 引擎源码文件清单（含子目录）
  find "$ROOT/crates" -path '*/src/*' -name '*.rs' 2>/dev/null
}

# ① forbid 属性覆盖：逐 crate 检查 src/lib.rs。
missing_forbid=""
for lib in "$ROOT"/crates/*/src/lib.rs; do
  [ -e "$lib" ] || continue
  if ! grep -q 'forbid(unsafe_code)' "$lib"; then
    missing_forbid="$missing_forbid$lib"$'\n'
  fi
done
if [ -n "$missing_forbid" ]; then
  echo "❌ ① 以下 crate 的 lib.rs 缺少 #![forbid(unsafe_code)]："
  printf '%s' "$missing_forbid"
  FAIL=1
fi

# ② 真 unsafe 使用（关键字形态：unsafe { / unsafe fn / unsafe impl / unsafe trait / unsafe extern）。
unsafe_hits=$(src_files | xargs grep -nE 'unsafe[[:space:]]*(\{|fn|impl|trait|extern)' 2>/dev/null || true)
if [ -n "$unsafe_hits" ]; then
  echo "❌ ② 引擎源码出现真 unsafe（承重墙禁令）："
  echo "$unsafe_hits"
  FAIL=1
fi

# ③ f64（类型/字面量）。`f64::` 与 `: f64` 与 `<f64>` 均命中。
f64_hits=$(src_files | xargs grep -nE '(^|[^A-Za-z0-9_])f64([^A-Za-z0-9_]|$)' 2>/dev/null \
  | grep -vE '//' || true)
if [ -n "$f64_hits" ]; then
  echo "❌ ③ 引擎源码出现 f64（§5 严格 f32）："
  echo "$f64_hits"
  FAIL=1
fi

# ④ 平台/向量内建（§7 分层）。
simd_hits=$(src_files | xargs grep -nE 'core::arch|std::simd|core::simd|_mm_|_mm256_|vld1q|is_x86_feature_detected' 2>/dev/null \
  | grep -vE '//' || true)
if [ -n "$simd_hits" ]; then
  echo "❌ ④ 引擎源码使用 platform/SIMD 内建（§7：主路径保持标量语义）："
  echo "$simd_hits"
  FAIL=1
fi

# ⑤ fast-math / 不可复现代码生成开关（RUSTFLAGS/config/CI 文本面）。
#    只看配置文件与工作流；跳过注释行与 YAML 的 `name:` 描述行——文档里写
#    「零 fast-math」是叙述，不是开关（本脚本自身与 CI step 名都会出现该词）。
strip_desc() { grep -vE ':[0-9]+:[[:space:]]*(-[[:space:]]+)?(name|run):' ; }
fm_hits=$(grep -rniE 'fast[-_]math|ffast[-_]math|enable-unsafe-fp-math|target-cpu=native' \
    "$ROOT/.cargo" "$ROOT/.github/workflows" 2>/dev/null \
  | grep -vE ':[[:space:]]*(#|//)' | strip_desc || true)
fm_hits="$fm_hits$(grep -rniE 'fast[-_]math|enable-unsafe-fp-math|target-cpu=native' \
    --include='Cargo.toml' "$ROOT" 2>/dev/null | grep -vE ':[[:space:]]*(#|//)' || true)"
if [ -n "$fm_hits" ]; then
  echo "❌ ⑤ 构建配置引入 fast-math / target-cpu=native（禁 FMA 重排与不可复现代码生成）："
  echo "$fm_hits"
  FAIL=1
fi

if [ "$FAIL" -eq 0 ]; then
  n=$(src_files | wc -l)
  echo "✅ 纪律扫描通过（forbid 全覆盖 / 零 unsafe / 零 f64 / 零 SIMD 内建 / 零 fast-math；源码 $n 文件）"
else
  echo "—— 纪律条款出处：docs/SPEC.md §5（严格 f32 与确定性）、§7（分层与 SIMD 独立路径）"
fi
exit "$FAIL"
