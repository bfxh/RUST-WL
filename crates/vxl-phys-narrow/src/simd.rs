//! SAT 扫描的 SSE2 内核（x86_64 基线 ISA，**无需运行时特性探测**）——**逐位透明**。
//!
//! 只并行「4 条轴」，轴内算术与标量参考 `sat_scan_scalar` 一字不差：
//! - 点积结合序显式 `((x·x + y·y) + z·z)`（与 `Vec3::dot` 的左结合一致）；
//! - `|v|` 用符号位掩码（bit-exact），`sqrt` 不在此模块使用；
//! - **退化轴**（`length_squared < 0.5`）、`sep > skin` 早退、`sep > best` 严格择优
//!   三项规则都在**标量侧按轴序**逐 lane 复刻 ⇒ 与标量实现逐位同结果。
//!
//! 非 x86_64（如 CI 的 aarch64 交叉）自动走标量路径；两条路径的四段哈希
//! （`m0_gates`/`determinism`/status/stress）必须逐位一致——这是跨平台哈希门的要求。
//!
//! SAFETY：`unsafe` 全部限于 `core::arch::x86_64` 的 `_mm_*` 内建调用；
//! `_mm_loadu_ps` 的输入都是长度 4 的**本地数组**（未对齐读取安全），无裸指针运算、
//! 无生命周期擦除、无别名假设；SSE2 是 x86_64 基线 ISA ⇒ 不需要 `target_feature`
//! 门（调用点因此无需 `unsafe fn` 的前置条件）。
#![allow(unsafe_code)]

use vxl_phys_core::Vec3;

/// 标量参考实现（语义基准）：非 x86_64 走它；x86_64 上由测试用它和 SIMD 逐位对照
/// （故 x86_64 的发布构建里它「未使用」，非死代码）。
#[cfg_attr(target_arch = "x86_64", allow(dead_code))]
#[allow(clippy::too_many_arguments)] // 内核签名（轴表 + 两盒的 half/axes/center + skin）
/// 返回 `None` = 存在分离轴（拒）或全轴退化；`Some((best_sep, n, idx))` 同原实现。
pub(crate) fn sat_scan_scalar(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    let mut best = f32::MIN;
    let mut best_n = Vec3::ZERO;
    let mut best_idx = usize::MAX;
    for (idx, &n0) in axes.iter().enumerate() {
        if n0.length_squared() < 0.5 {
            continue;
        }
        let ra =
            ha.x * aa[0].dot(n0).abs() + ha.y * aa[1].dot(n0).abs() + ha.z * aa[2].dot(n0).abs();
        let rb =
            hb.x * ab[0].dot(n0).abs() + hb.y * ab[1].dot(n0).abs() + hb.z * ab[2].dot(n0).abs();
        let ca = pa.dot(n0);
        let cb = pb.dot(n0);
        let (min_a, max_a) = (ca - ra, ca + ra);
        let (min_b, max_b) = (cb - rb, cb + rb);
        let sep1 = min_b - max_a;
        let sep2 = min_a - max_b;
        let (sep, n) = if sep1 >= sep2 {
            (sep1, n0)
        } else {
            (sep2, -n0)
        };
        if sep > skin {
            return None;
        }
        if sep > best {
            best = sep;
            best_n = n;
            best_idx = idx;
        }
    }
    if best == f32::MIN {
        None
    } else {
        Some((best, best_n, best_idx))
    }
}

/// SIMD 路径分发：x86_64 走 SSE2 内核，其余走标量（结果逐位相同）。
#[allow(clippy::too_many_arguments)]
#[cfg(target_arch = "x86_64")]
pub(crate) fn sat_scan(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    sat_scan_sse2(axes, ha, aa, pa, hb, ab, pb, skin)
}

#[cfg(not(target_arch = "x86_64"))]
pub(crate) fn sat_scan(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    sat_scan_scalar(axes, ha, aa, pa, hb, ab, pb, skin)
}

#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
fn sat_scan_sse2(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    use core::arch::x86_64::*;
    let mut best = f32::MIN;
    let mut best_n = Vec3::ZERO;
    let mut best_idx = usize::MAX;
    let n_all = axes.len();
    let mut i = 0usize;
    while i < n_all {
        let m = (n_all - i).min(4);
        // 装 4 条轴的分量行；尾组复制最后一条占位（其 lane 在标量侧不被消费）。
        let mut cx = [0f32; 4];
        let mut cy = [0f32; 4];
        let mut cz = [0f32; 4];
        for k in 0..4 {
            let a = axes[i + k.min(m - 1)];
            cx[k] = a.x;
            cy[k] = a.y;
            cz[k] = a.z;
        }
        let mut sep_arr = [0f32; 4];
        let flags: i32; // bit k: 1 = 该 lane 用 −n（sep2 更大；下面 unsafe 块内赋值）
        let degen: i32; // bit k: 1 = 该 lane 退化（标量侧 continue）
                        // SAFETY: 本块（含内部 `dot_lane`）只调用 x86_64 基线 SSE2 内建——函数在
                        // `#[cfg(target_arch = "x86_64")]` 下编译，故无需运行时特性探测；指针入参只有两处：
                        // `_mm_loadu_ps` 读 `cx/cy/cz`、`_mm_storeu_ps` 写 `sep_arr`，都是长度 4 的本地
                        // `f32` 数组（16 字节未对齐访问仍在界内），其余内建只消费 `__m128` 寄存器值：
                        // 无裸指针运算、无生命周期擦除、无别名假设。
        unsafe {
            let nx = _mm_loadu_ps(cx.as_ptr());
            let ny = _mm_loadu_ps(cy.as_ptr());
            let nz = _mm_loadu_ps(cz.as_ptr());
            let signmask = _mm_set1_ps(-0.0);
            // 退化判定：length_squared < 0.5（结合序同 Vec3::dot）
            let ls = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(nx, nx), _mm_mul_ps(ny, ny)),
                _mm_mul_ps(nz, nz),
            );
            degen = _mm_movemask_ps(_mm_cmplt_ps(ls, _mm_set1_ps(0.5)));
            // lane 版「标量向量 · 4 条轴」：v.x·nx + v.y·ny + v.z·nz（同左结合序）
            #[inline(always)]
            unsafe fn dot_lane(v: Vec3, nx: __m128, ny: __m128, nz: __m128) -> __m128 {
                _mm_add_ps(
                    _mm_add_ps(
                        _mm_mul_ps(_mm_set1_ps(v.x), nx),
                        _mm_mul_ps(_mm_set1_ps(v.y), ny),
                    ),
                    _mm_mul_ps(_mm_set1_ps(v.z), nz),
                )
            }
            let absf = |v: __m128| _mm_andnot_ps(signmask, v);
            let ra = _mm_add_ps(
                _mm_add_ps(
                    _mm_mul_ps(_mm_set1_ps(ha.x), absf(dot_lane(aa[0], nx, ny, nz))),
                    _mm_mul_ps(_mm_set1_ps(ha.y), absf(dot_lane(aa[1], nx, ny, nz))),
                ),
                _mm_mul_ps(_mm_set1_ps(ha.z), absf(dot_lane(aa[2], nx, ny, nz))),
            );
            let rb = _mm_add_ps(
                _mm_add_ps(
                    _mm_mul_ps(_mm_set1_ps(hb.x), absf(dot_lane(ab[0], nx, ny, nz))),
                    _mm_mul_ps(_mm_set1_ps(hb.y), absf(dot_lane(ab[1], nx, ny, nz))),
                ),
                _mm_mul_ps(_mm_set1_ps(hb.z), absf(dot_lane(ab[2], nx, ny, nz))),
            );
            let ca = dot_lane(pa, nx, ny, nz);
            let cb = dot_lane(pb, nx, ny, nz);
            let sep1 = _mm_sub_ps(_mm_sub_ps(cb, rb), _mm_add_ps(ca, ra)); // min_b − max_a
            let sep2 = _mm_sub_ps(_mm_sub_ps(ca, ra), _mm_add_ps(cb, rb)); // min_a − max_b
                                                                           // 标量是 `if sep1 >= sep2 {sep1} else {sep2}`；flip = !(sep1 >= sep2)
            flags = _mm_movemask_ps(_mm_cmpnge_ps(sep1, sep2));
            // 操作数顺序是承重的：标量取 sep1（平局时 `sep1 >= sep2` 为真），而 Intel
            // MAXPS 在相等时返回**第二操作数** ⇒ 必须写成 `max(sep2, sep1)` 才让平局也取
            // sep1。非平局时 max 对称、结果不变；平局带符号零时（sep1 = +0.0、sep2 = −0.0）
            // 顺序写反会让 best_sep 的符号位与标量分叉，而那个值进确定性哈希。
            // 见 tests::sat_tie_break_matches_scalar_on_signed_zero（随机采样撞不到这个平局）。
            let sep = _mm_max_ps(sep2, sep1);
            _mm_storeu_ps(sep_arr.as_mut_ptr(), sep);
        }
        // 标量侧：按轴序复刻三条规则（退化跳过 / 早退 / 严格择优）。
        for k in 0..m {
            if degen & (1 << k) != 0 {
                continue;
            }
            let s = sep_arr[k];
            if s > skin {
                return None;
            }
            if s > best {
                best = s;
                best_idx = i + k;
                let n0 = axes[i + k];
                best_n = if flags & (1 << k) != 0 { -n0 } else { n0 };
            }
        }
        i += m;
    }
    if best == f32::MIN {
        None
    } else {
        Some((best, best_n, best_idx))
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use super::*;

    fn v(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3::new(x, y, z)
    }

    /// 轴表（盒对形态：12 面轴 + 至多 9 棱叉积轴）——与 `sat()` 的构造同序。
    fn axes_of(aa: &[Vec3; 3], ab: &[Vec3; 3]) -> Vec<Vec3> {
        let mut out = Vec::new();
        for s in [
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        ] {
            out.push(aa[0] * s.0 + aa[1] * s.1 + aa[2] * s.2);
        }
        for s in [
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        ] {
            out.push(ab[0] * s.0 + ab[1] * s.1 + ab[2] * s.2);
        }
        let (ea, eb) = ([aa[1], aa[2], aa[0]], [ab[1], ab[2], ab[0]]);
        for x in ea {
            for y in eb {
                let c = x.cross(y);
                let l2 = c.length_squared();
                if l2 > 1e-8 {
                    out.push(c * (1.0 / l2.sqrt()));
                }
            }
        }
        out
    }

    /// 随机**正交基**（Gram-Schmidt）——模拟真实盒姿态；随机基会让 SAT 几乎总是
    /// 分离，样本覆盖不到接触分支。
    fn basis(rnd: &mut impl FnMut() -> f32) -> [Vec3; 3] {
        let (mut a, mut b, mut c) = (
            v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
            v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
            v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
        );
        a = a * (1.0 / a.length());
        b = b - a * b.dot(a);
        b = b * (1.0 / b.length());
        c = c - a * c.dot(a) - b * c.dot(b);
        c = c * (1.0 / c.length());
        [a, b, c]
    }

    /// SIMD 与标量参考在随机姿态/尺寸/位移上**逐位一致**（含退化与分离两类）。
    #[test]
    fn simd_matches_scalar_bitwise() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f32 / (1u64 << 53) as f32
        };
        let mut sep_cnt = 0usize;
        let mut hit_cnt = 0usize;
        for _ in 0..4000 {
            let ax = basis(&mut rnd);
            let bx = basis(&mut rnd);
            let ha = v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3);
            let hb = v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3);
            let pa = v(rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5);
            let pb = pa + v(rnd() * 1.6 - 0.8, rnd() * 1.6 - 0.8, rnd() * 1.6 - 0.8);
            let skin = 0.01;
            let axes = axes_of(&ax, &bx);
            let s = sat_scan_scalar(&axes, ha, &ax, pa, hb, &bx, pb, skin);
            let d = sat_scan_sse2(&axes, ha, &ax, pa, hb, &bx, pb, skin);
            match (s, d) {
                (None, None) => sep_cnt += 1,
                (Some((s0, sn, si)), Some((d0, dn, di))) => {
                    hit_cnt += 1;
                    assert_eq!(s0.to_bits(), d0.to_bits(), "best sep 不等");
                    assert_eq!(si, di, "best idx 不等");
                    assert_eq!(sn.x.to_bits(), dn.x.to_bits(), "法线 x 不等");
                    assert_eq!(sn.y.to_bits(), dn.y.to_bits(), "法线 y 不等");
                    assert_eq!(sn.z.to_bits(), dn.z.to_bits(), "法线 z 不等");
                }
                (a, b) => panic!(
                    "分支不一致：标量 {:?} vs SIMD {:?}",
                    a.is_some(),
                    b.is_some()
                ),
            }
        }
        // 两类都必须被覆盖到（否则测试没有鉴别力）。
        assert!(sep_cnt > 100, "分离样本太少：{sep_cnt}");
        assert!(hit_cnt > 100, "命中样本太少：{hit_cnt}");
    }

    /// **符号零平局**：两条 sep 恰好都为零、符号相反时，`_mm_max_ps` 的平局规则
    /// （Intel：相等取**第二操作数**）与标量参考的 `if sep1 >= sep2 { sep1 }` 会分叉，
    /// 使 `best_sep` 的符号位不同——而它进确定性哈希。
    ///
    /// 随机采样**撞不到**这个平局（`simd_matches_scalar_bitwise` 的 4000 个样本里概率 ~0），
    /// 所以这里给确定性的构造输入把它钉住：轴取 +X、两盒半宽均为零、A 心 x = −0.0、B 心 x = 0.0 ⇒
    /// `sep1 = (cb − rb) − (ca + ra) = 0.0 − (−0.0) = +0.0`，
    /// `sep2 = (ca − ra) − (cb + rb) = (−0.0 − 0.0) − 0.0 = −0.0`（IEEE：混号零相加归 +0.0，
    /// 但 `(−0.0) − (+0.0)` 仍是 −0.0）。两条路径必须逐位同结果。
    #[test]
    fn sat_tie_break_matches_scalar_on_signed_zero() {
        let ax = [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)];
        let axes = vec![v(1.0, 0.0, 0.0)];
        let zero = v(0.0, 0.0, 0.0);
        let neg_zero_x = v(-0.0, 0.0, 0.0);
        let s = sat_scan_scalar(&axes, zero, &ax, neg_zero_x, zero, &ax, zero, 0.01)
            .expect("应有分离解（sep 为零、不被 skin 早退）");
        let d = sat_scan_sse2(&axes, zero, &ax, neg_zero_x, zero, &ax, zero, 0.01)
            .expect("应有分离解（sep 为零、不被 skin 早退）");
        assert_eq!(
            s.0.to_bits(),
            d.0.to_bits(),
            "平局时 best_sep 的符号位分叉：标量 {}（{:#010x}）vs SIMD {}（{:#010x}）",
            s.0,
            s.0.to_bits(),
            d.0,
            d.0.to_bits()
        );
        assert_eq!(s.2, d.2, "平局时 best_idx 不等");
        // 构造前提自检：两条路径都必须给 **+0.0**。若构造失效（例如两个 sep 同号），
        // 上面那条等式就退化成恒真、失去鉴别力——所以把期望值也钉死。
        // 若 MAXPS 的操作数顺序写反（平局取 sep2 = −0.0），这里会以符号位不同报红。
        assert_eq!(
            s.0.to_bits(),
            0.0f32.to_bits(),
            "标量平局应取 sep1（+0.0），实为 {}",
            s.0
        );
        assert_eq!(
            d.0.to_bits(),
            0.0f32.to_bits(),
            "SIMD 平局必须与标量同取 +0.0，实为 {}",
            d.0
        );
    }

    /// 轴数不是 4 的倍数（21、12、13）时尾组 lane 不得被消费——与标量对比。
    #[test]
    fn simd_tail_lanes_are_ignored() {
        let ax = [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)];
        let bx = ax;
        let ha = v(0.5, 0.5, 0.5);
        let pa = v(0.0, 0.0, 0.0);
        let pb = v(0.95, 0.0, 0.0);
        for take in [1usize, 2, 3, 4, 5, 9, 12, 13, 21] {
            let axes: Vec<Vec3> = (0..take).map(|k| ax[k % 3]).collect();
            let s = sat_scan_scalar(&axes, ha, &ax, pa, ha, &bx, pb, 0.01);
            let d = sat_scan_sse2(&axes, ha, &ax, pa, ha, &bx, pb, 0.01);
            assert_eq!(s.is_some(), d.is_some(), "take={take} 分支不一致");
            if let (Some((s0, _, si)), Some((d0, _, di))) = (s, d) {
                assert_eq!(s0.to_bits(), d0.to_bits(), "take={take} sep 不等");
                assert_eq!(si, di, "take={take} idx 不等");
            }
        }
    }
}
