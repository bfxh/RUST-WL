// **窄相（卡上）：固定槽**。字数/偏移/种类值与主机 `crate::narrow` 的 `SLOT_WORDS` /
// `BODY_WORDS` / `KIND_*` **逐字对齐**（改了这边必须同步那边，否则槽会错位）。
//
// 槽 = 26 个 u32（104 B）/对，**对序即槽序**（不需要重排 ⇒ 流形表结构天然确定）：
//   w0 a | w1 b | w2..4 normal.xyz | w5 count | 4×(point.xyz | depth | feature)
//   `count` = 0（无流形）/ 1..4（点数）/ `NOT_HANDLED`(0xffffffff ⇒ 本档不接手该对，
//   调用方按**主机回填**处理并写进同一槽位）。
//
// 逐体输入 = 12 个 u32（48 B）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`
//   —— 浮点走 f32 位模式（`as_f32`），`kind` 是**裸整数**（1 = 球、2 = 盒；0 = 不接手）。
//
// **已接的族**（几何都**逐字照抄** CPU `pair_shaped.rs` / `sat.rs` 对应分支）：
//   ① 球×球（含同心球 `dist < 1e-9` 特殊分支）；
//   ② 盒×盒：21 轴 SAT（6+6 面轴 + 9 棱叉积，**压缩序照抄**）→ 参考/入射面 → 4 次侧平面裁剪
//      → 主平面过滤 → 稳定排序 + 去重选 ≤4 点。`feature` 位域与 CPU 逐位同源。
// **不实现 `predict_inflate`**（CPU 那条 `inflate` 只在 `predict_dt > 0` 时非零，默认档 = 0）——
//   这是**前提**：接线时若 `World` 给窄相设了 `predict_dt > 0`，卡上路径必须让位或补这一项。

const SLOT_WORDS: u32 = 26u;
const PT_BASE: u32 = 6u;
const PT_WORDS: u32 = 5u;
const W_NX: u32 = 2u;
const W_CNT: u32 = 5u;
const NOT_HANDLED: u32 = 0xffffffffu;

const BODY_WORDS: u32 = 12u;
const KIND_AT: u32 = 7u;
const P0_AT: u32 = 8u;
const KIND_SPHERE: u32 = 1u;
const KIND_BOX: u32 = 2u;

/// 特征位（与 CPU `types.rs` 同值）：bit31 = 入射侧 B、bit30 = 裁剪交点。
const FEAT_SIDE_B: u32 = 0x80000000u;
const FEAT_CLIPPED: u32 = 0x40000000u;

/// 分离轴来源（与 CPU `AxisSrc` 同序）。
const SRC_FACE_A: u32 = 0u;
const SRC_FACE_B: u32 = 1u;
const SRC_EDGE: u32 = 2u;

/// `f32::MIN` / `f32::MAX`（CPU 那边用的哨兵）。
const F32_MIN: f32 = -3.402823466e38;
const F32_MAX: f32 = 3.402823466e38;

/// 裁剪多边形上限：凸多边形被半平面裁一次至多 +1 顶点 ⇒ 4 → 5 → 6 → 7 → 8，取 16（一倍余量）。
/// 超了不静默截断：`atomicAdd(&diag[0], 1)`（主机侧断言它必须是 0）。
const CLIP_MAX: u32 = 16u;

/// 6 张面 × 4 顶点：把 CPU `BOX_FACES` 的「顶点符号三元组」压成整数
/// （bit0 = sx>0、bit1 = sy>0、bit2 = sz>0；4 顶点 × 3 bit 打包）。**面序与面内顶点序都照抄**：
/// +X [1,3,7,5] / −X [0,4,6,2] / +Y [2,6,7,3] / −Y [0,1,5,4] / +Z [4,5,7,6] / −Z [0,2,3,1]。
/// 面法线由面号直接给：`axis = f / 2`、`f & 1` = 该轴取负（+X,−X,+Y,−Y,+Z,−Z）。
const FACE_V: array<u32, 6> = array<u32, 6>(3033u, 1440u, 2034u, 2376u, 3564u, 720u);

struct Params {
    n_bodies: u32,
    n_pairs: u32,
    /// 接触 skin（§4.3 投机带）——SAT 的分离阈与主平面过滤都用它。
    skin: f32,
    /// 选点去重间距（CPU `min_point_sep` = `max(skin*2, 0.01)`）。
    min_sep: f32,
};

@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> bodies: array<u32>;
@group(0) @binding(2) var<storage, read> pairs: array<u32>;
@group(0) @binding(3) var<storage, read_write> slots: array<u32>;
/// 诊断（主机侧断言全 0）：`[0]` = 裁剪多边形越界次数。
@group(0) @binding(4) var<storage, read_write> diag: array<atomic<u32>>;

fn as_f32(w: u32) -> f32 {
    return bitcast<f32>(w);
}

fn body_pos(base: u32) -> vec3<f32> {
    return vec3<f32>(
        as_f32(bodies[base]),
        as_f32(bodies[base + 1u]),
        as_f32(bodies[base + 2u]),
    );
}

fn body_quat(base: u32) -> vec4<f32> {
    return vec4<f32>(
        as_f32(bodies[base + 3u]),
        as_f32(bodies[base + 4u]),
        as_f32(bodies[base + 5u]),
        as_f32(bodies[base + 6u]),
    );
}

fn body_half(base: u32) -> vec3<f32> {
    return vec3<f32>(
        as_f32(bodies[base + P0_AT]),
        as_f32(bodies[base + P0_AT + 1u]),
        as_f32(bodies[base + P0_AT + 2u]),
    );
}

/// 体轴 `[R·X, R·Y, R·Z]`：与 CPU `Quat::rotate_vec3` **同式**
/// （`t = qv×v·2`、`v + qv×t + t·w`，结合序也照抄）。
fn rotate_axis(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let qv = q.xyz;
    let t = cross(qv, v) * 2.0;
    return v + cross(qv, t) + t * q.w;
}

fn box_axes_of(base: u32) -> array<vec3<f32>, 3> {
    let q = body_quat(base);
    return array<vec3<f32>, 3>(
        rotate_axis(q, vec3<f32>(1.0, 0.0, 0.0)),
        rotate_axis(q, vec3<f32>(0.0, 1.0, 0.0)),
        rotate_axis(q, vec3<f32>(0.0, 0.0, 1.0)),
    );
}

/// 面法线：`axis = f/2`、`f & 1` = 取负（与 CPU `face_normal_of` 逐位同源：±v 是精确运算）。
fn face_normal(ax: array<vec3<f32>, 3>, f: u32) -> vec3<f32> {
    let v = ax[f / 2u];
    return select(v, -v, (f & 1u) != 0u);
}

/// 面顶点：`pos + (t0 + t1) + t2`（展开式与结合序都照抄 CPU `face_vertex`）。
fn face_vertex(
    pos: vec3<f32>,
    ax: array<vec3<f32>, 3>,
    half: vec3<f32>,
    f: u32,
    i: u32,
) -> vec3<f32> {
    let bits = (FACE_V[f] >> (i * 3u)) & 7u;
    let sx = select(-1.0, 1.0, (bits & 1u) != 0u);
    let sy = select(-1.0, 1.0, (bits & 2u) != 0u);
    let sz = select(-1.0, 1.0, (bits & 4u) != 0u);
    let t0 = ax[0] * (half.x * sx);
    let t1 = ax[1] * (half.y * sy);
    let t2 = ax[2] * (half.z * sz);
    return pos + (t0 + t1) + t2;
}

/// 裁剪交点特征 = 入射棱 (fa→fb) × 参考侧平面 k 的确定性混合（与 CPU `feat_intersect` 同式；
/// u32 乘在 WGSL 里就是**回绕**乘 ⇒ 无需 wrapping 前缀）。
fn feat_intersect(fa: u32, fb: u32, k: u32) -> u32 {
    let h = (fa * 73856093u) ^ (fb * 19349663u) ^ (k * 83492791u);
    return FEAT_CLIPPED | (h & 0x3fffffffu);
}

fn put_normal(base: u32, n: vec3<f32>) {
    slots[base + W_NX] = bitcast<u32>(n.x);
    slots[base + W_NX + 1u] = bitcast<u32>(n.y);
    slots[base + W_NX + 2u] = bitcast<u32>(n.z);
}

fn put_point(base: u32, k: u32, p: vec3<f32>, depth: f32, feature: u32) {
    let o = base + PT_BASE + k * PT_WORDS;
    slots[o] = bitcast<u32>(p.x);
    slots[o + 1u] = bitcast<u32>(p.y);
    slots[o + 2u] = bitcast<u32>(p.z);
    slots[o + 3u] = bitcast<u32>(depth);
    slots[o + 4u] = feature;
}

// ===== ① 球×球 =====
// 与 CPU 同式（`d = pb − pa`、`dist = √(d·d)`、`rr = ra + rb`、`n = d·(1/dist)`、
// `point = pa + n·(ra − (rr − dist)·0.5)`、`depth = rr − dist`）。
// 分支序也照抄：`dist ≥ rr` 先返（槽留 0 = 无流形），再判同心（`dist < 1e-9`）。
fn sphere_pair(base: u32, ba: u32, bb: u32) {
    let pa = body_pos(ba);
    let pb = body_pos(bb);
    let ra = as_f32(bodies[ba + P0_AT]);
    let rb = as_f32(bodies[bb + P0_AT]);
    let d = pb - pa;
    let dist = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    let rr = ra + rb;
    if (dist >= rr) {
        return;
    }
    if (dist < 1e-9) {
        // 同心球（CPU 同分支）：法线恒定 +Y、点 = a 心、深度 = rr。
        put_normal(base, vec3<f32>(0.0, 1.0, 0.0));
        put_point(base, 0u, pa, rr, 0u);
        slots[base + W_CNT] = 1u;
        return;
    }
    let inv = 1.0 / dist;
    let n = vec3<f32>(d.x * inv, d.y * inv, d.z * inv);
    let point = pa + n * (ra - (rr - dist) * 0.5);
    put_normal(base, n);
    put_point(base, 0u, point, rr - dist, 0u);
    slots[base + W_CNT] = 1u;
}

// ===== ② 盒×盒：SAT =====

struct Sat {
    /// 0 = 分离（无接触）。
    ok: u32,
    sep: f32,
    n: vec3<f32>,
    src: u32,
};

/// 21 轴扫描（6 面轴 A + 6 面轴 B + 9 棱叉积）。**语义逐条照抄 CPU `sat_scan_scalar`**：
/// ① 棱叉积只收 `l2 > 1e-8` 的（**压缩后的下标序**决定平局时谁先到 ⇒ 影响 `src`）；
/// ② 逐轴跳过 `|n|² < 0.5`；③ 双侧测分离取较大者；④ **任一轴 `sep > skin` 立即返回分离**；
/// ⑤ `sep > best` 是**严格大于** ⇒ 平局保留**先到的轴**。
fn sat_boxes(
    pa: vec3<f32>,
    ha: vec3<f32>,
    axa: array<vec3<f32>, 3>,
    pb: vec3<f32>,
    hb: vec3<f32>,
    axb: array<vec3<f32>, 3>,
    skin: f32,
) -> Sat {
    var axes: array<vec3<f32>, 21>;
    var n_ax = 0u;
    for (var f = 0u; f < 6u; f = f + 1u) {
        axes[n_ax] = face_normal(axa, f);
        n_ax = n_ax + 1u;
    }
    for (var f = 0u; f < 6u; f = f + 1u) {
        axes[n_ax] = face_normal(axb, f);
        n_ax = n_ax + 1u;
    }
    // 棱序照抄 CPU：`ea = [ax1, ax2, ax0]`（× eb 同构），双层循环下标序也照抄。
    let ea = array<vec3<f32>, 3>(axa[1], axa[2], axa[0]);
    let eb = array<vec3<f32>, 3>(axb[1], axb[2], axb[0]);
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            let c = cross(ea[i], eb[j]);
            let l2 = dot(c, c);
            if (l2 > 1e-8) {
                axes[n_ax] = c * (1.0 / sqrt(l2));
                n_ax = n_ax + 1u;
            }
        }
    }
    var best = F32_MIN;
    var best_n = vec3<f32>(0.0, 0.0, 0.0);
    var best_src = SRC_EDGE;
    var out: Sat;
    for (var idx = 0u; idx < n_ax; idx = idx + 1u) {
        let n0 = axes[idx];
        if (dot(n0, n0) < 0.5) {
            continue;
        }
        let ra = ha.x * abs(dot(axa[0], n0)) + ha.y * abs(dot(axa[1], n0))
            + ha.z * abs(dot(axa[2], n0));
        let rb = hb.x * abs(dot(axb[0], n0)) + hb.y * abs(dot(axb[1], n0))
            + hb.z * abs(dot(axb[2], n0));
        let ca = dot(pa, n0);
        let cb = dot(pb, n0);
        let sep1 = (cb - rb) - (ca + ra);
        let sep2 = (ca - ra) - (cb + rb);
        var sep = sep2;
        var n = -n0;
        if (sep1 >= sep2) {
            sep = sep1;
            n = n0;
        }
        if (sep > skin) {
            out.ok = 0u;
            out.sep = 0.0;
            out.n = vec3<f32>(0.0, 0.0, 0.0);
            out.src = SRC_EDGE;
            return out;
        }
        if (sep > best) {
            best = sep;
            best_n = n;
            best_src = SRC_EDGE;
            if (idx < 6u) {
                best_src = SRC_FACE_A;
            } else if (idx < 12u) {
                best_src = SRC_FACE_B;
            }
        }
    }
    out.ok = select(0u, 1u, best != F32_MIN);
    out.sep = best;
    out.n = best_n;
    out.src = best_src;
    return out;
}

/// 参考面裁剪 → 接触点（≤4）→ 写槽。**逐条照抄 CPU `clip()`**：
/// 参考盒按 `src` 选（FaceB ⇒ B，否则 A）、参考面按「ref → incident」方向重选（严格大于 ⇒ 平局取面序在前）、
/// 入射面取与 `n_ref` 最逆平行（严格小于）、4 次侧平面裁剪（`|s|² < 1e-16` 的平面**整条跳过**）、
/// 主平面过滤 `d <= skin`（`depth = −d`）、稳定排序 + 去重选 ≤4。
fn boxes_pair(
    base: u32,
    ba: u32,
    bb: u32,
    pa: vec3<f32>,
    ha: vec3<f32>,
    axa: array<vec3<f32>, 3>,
    pb: vec3<f32>,
    hb: vec3<f32>,
    axb: array<vec3<f32>, 3>,
) {
    let sat = sat_boxes(pa, ha, axa, pb, hb, axb, prm.skin);
    if (sat.ok == 0u) {
        return;
    }
    let ref_is_a = sat.src != SRC_FACE_B;
    // 参考盒 / 入射盒（数组不能 select ⇒ 用分支赋值）。
    var rax: array<vec3<f32>, 3>;
    var iax: array<vec3<f32>, 3>;
    var rpos: vec3<f32>;
    var rhalf: vec3<f32>;
    var ipos: vec3<f32>;
    var ihalf: vec3<f32>;
    if (ref_is_a) {
        rax = axa;
        iax = axb;
        rpos = pa;
        rhalf = ha;
        ipos = pb;
        ihalf = hb;
    } else {
        rax = axb;
        iax = axa;
        rpos = pb;
        rhalf = hb;
        ipos = pa;
        ihalf = ha;
    }
    let dir = select(-sat.n, sat.n, ref_is_a);
    // 参考面：与 dir 最对齐（面序 0..5，严格大于 ⇒ 平局取先）。
    var ref_face = 0u;
    var bd = F32_MIN;
    for (var f = 0u; f < 6u; f = f + 1u) {
        let d = dot(face_normal(rax, f), dir);
        if (d > bd) {
            bd = d;
            ref_face = f;
        }
    }
    let n_ref = face_normal(rax, ref_face);
    let ref_base = ref_face * 4u;
    var ref_v: array<vec3<f32>, 4>;
    for (var i = 0u; i < 4u; i = i + 1u) {
        ref_v[i] = face_vertex(rpos, rax, rhalf, ref_face, i);
    }
    // 入射面：与 n_ref 最逆平行（严格小于 ⇒ 平局取先）。
    var inc_face = 0u;
    var inc_dot = F32_MAX;
    for (var f = 0u; f < 6u; f = f + 1u) {
        let d = dot(face_normal(iax, f), n_ref);
        if (d < inc_dot) {
            inc_dot = d;
            inc_face = f;
        }
    }
    let side_bit = select(0u, FEAT_SIDE_B, ref_is_a);
    var clip_pt: array<vec3<f32>, CLIP_MAX>;
    var clip_ft: array<u32, CLIP_MAX>;
    for (var i = 0u; i < 4u; i = i + 1u) {
        clip_pt[i] = face_vertex(ipos, iax, ihalf, inc_face, i);
        clip_ft[i] = side_bit | (inc_face * 4u + i);
    }
    var m = 4u;
    var o_pt: array<vec3<f32>, CLIP_MAX>;
    var o_ft: array<u32, CLIP_MAX>;
    var centroid = vec3<f32>(0.0, 0.0, 0.0);
    for (var i = 0u; i < 4u; i = i + 1u) {
        centroid = centroid + ref_v[i];
    }
    centroid = centroid * (1.0 / 4.0);
    for (var k = 0u; k < 4u; k = k + 1u) {
        var k1 = k + 1u;
        if (k1 == 4u) {
            k1 = 0u;
        }
        let w0 = ref_v[k];
        let e = ref_v[k1] - w0;
        var s = cross(e, n_ref);
        if (dot(s, s) < 1e-16) {
            continue;
        }
        if (dot(s, centroid - w0) > 0.0) {
            s = -s;
        }
        var o = 0u;
        for (var i = 0u; i < m; i = i + 1u) {
            let j = (i + 1u) % m;
            let va = clip_pt[i];
            let vb = clip_pt[j];
            let da = dot(va - w0, s);
            let db = dot(vb - w0, s);
            if (da <= 0.0) {
                if (o < CLIP_MAX) {
                    o_pt[o] = va;
                    o_ft[o] = clip_ft[i];
                    o = o + 1u;
                } else {
                    atomicAdd(&diag[0], 1u);
                }
            }
            if (da * db < 0.0) {
                let t = da / (da - db);
                if (o < CLIP_MAX) {
                    o_pt[o] = va + (vb - va) * t;
                    o_ft[o] = feat_intersect(clip_ft[i], clip_ft[j], ref_base + k);
                    o = o + 1u;
                } else {
                    atomicAdd(&diag[0], 1u);
                }
            }
        }
        m = o;
        for (var i = 0u; i < o; i = i + 1u) {
            clip_pt[i] = o_pt[i];
            clip_ft[i] = o_ft[i];
        }
        if (m == 0u) {
            return; // 裁空 ⇒ 无流形（槽留 0）
        }
    }
    // 主平面过滤（`depth = −d`；`inflate` 恒 0 —— 见档头那条前提）。
    let p0 = ref_v[0];
    var c_pt: array<vec3<f32>, CLIP_MAX>;
    var c_dep: array<f32, CLIP_MAX>;
    var c_ft: array<u32, CLIP_MAX>;
    var cn = 0u;
    for (var i = 0u; i < m; i = i + 1u) {
        let d = dot(clip_pt[i] - p0, n_ref);
        if (d <= prm.skin) {
            c_pt[cn] = clip_pt[i];
            c_dep[cn] = -d;
            c_ft[cn] = clip_ft[i];
            cn = cn + 1u;
        }
    }
    if (cn == 0u) {
        return;
    }
    select_and_emit(base, sat.n, c_pt, c_dep, c_ft, cn);
}

/// 选点：**稳定**插入排序（深度降序 → x → y → z 升序，与 CPU 的 `sort_by` 同键同序）
/// + 贪心去重（`min_sep` 内视为同点）+ 截断 ≤4 —— 与 CPU `select_contacts` 逐条对应。
/// ⚠️ 比较用普通 `<`（非 `total_cmp`）：两者只在 NaN 与 ±0.0 上不同，而这里的量都是有限非零差异；
/// 真出现 ±0.0 平局时会落到下一个键，判据（逐对容差比 CPU）会立刻暴露。
fn select_and_emit(
    base: u32,
    n_ab: vec3<f32>,
    c_pt: array<vec3<f32>, CLIP_MAX>,
    c_dep: array<f32, CLIP_MAX>,
    c_ft: array<u32, CLIP_MAX>,
    cn: u32,
) {
    var pt = c_pt;
    var dep = c_dep;
    var ft = c_ft;
    for (var i = 1u; i < cn; i = i + 1u) {
        let kp = pt[i];
        let kd = dep[i];
        let kf = ft[i];
        var j = i;
        loop {
            if (j == 0u) {
                break;
            }
            let pd = dep[j - 1u];
            let pp = pt[j - 1u];
            let before = (kd > pd)
                || (kd == pd && kp.x < pp.x)
                || (kd == pd && kp.x == pp.x && kp.y < pp.y)
                || (kd == pd && kp.x == pp.x && kp.y == pp.y && kp.z < pp.z);
            if (!before) {
                break;
            }
            pt[j] = pp;
            dep[j] = pd;
            ft[j] = ft[j - 1u];
            j = j - 1u;
        }
        pt[j] = kp;
        dep[j] = kd;
        ft[j] = kf;
    }
    var kept_pt: array<vec3<f32>, 4>;
    var kept_dep: array<f32, 4>;
    var kept_ft: array<u32, 4>;
    let min2 = prm.min_sep * prm.min_sep;
    var kn = 0u;
    for (var i = 0u; i < cn; i = i + 1u) {
        if (kn >= 4u) {
            break;
        }
        var dup = false;
        for (var j = 0u; j < kn; j = j + 1u) {
            let dd = kept_pt[j] - pt[i];
            if (dot(dd, dd) < min2) {
                dup = true;
            }
        }
        if (!dup) {
            kept_pt[kn] = pt[i];
            kept_dep[kn] = dep[i];
            kept_ft[kn] = ft[i];
            kn = kn + 1u;
        }
    }
    if (kn == 0u) {
        return;
    }
    put_normal(base, n_ab);
    for (var k = 0u; k < kn; k = k + 1u) {
        put_point(base, k, kept_pt[k], kept_dep[k], kept_ft[k]);
    }
    slots[base + W_CNT] = kn;
}

// ===== ③ 球×盒（CPU 走 `sphere_convex_ab` + `closest_point_on_poly`）=====
//
// ⚠️ **顶点算法与盒×盒不同源，别混**：盒×盒那条路用 `face_vertex`（由**体轴 × 半长**拼世界顶点），
// 而球×凸体这条路用 `WorldPoly::fill`（`rotate_vec3(局部顶点) + pos`）——两者在浮点上**不等价**
// （`R·(a+b+c) ≠ R·a+R·b+R·c` 的末位）。这里必须照抄后者。
//
// 局部顶点按 `BoxPolytope::box_polytope` 的位约定：bit0=+x、bit1=+y、bit2=+z ⇒ 三比特就是
// `FACE_V` 里的那三组（面序/面内序都与 `BOX_FACES` 一致）。面法线 = 局部 ±X/±Y/±Z（axis=f/2）。

struct Closest {
    pt: vec3<f32>,
    d2: f32,
    inside: bool,
    in_n: vec3<f32>,
};

fn face_normal_local(f: u32) -> vec3<f32> {
    let a = f / 2u;
    var v = vec3<f32>(0.0, 0.0, 1.0);
    if (a == 0u) {
        v = vec3<f32>(1.0, 0.0, 0.0);
    } else if (a == 1u) {
        v = vec3<f32>(0.0, 1.0, 0.0);
    }
    return select(v, -v, (f & 1u) != 0u);
}

fn local_vert(half: vec3<f32>, f: u32, k: u32) -> vec3<f32> {
    let bits = (FACE_V[f] >> (k * 3u)) & 7u;
    let sx = select(-half.x, half.x, (bits & 1u) != 0u);
    let sy = select(-half.y, half.y, (bits & 2u) != 0u);
    let sz = select(-half.z, half.z, (bits & 4u) != 0u);
    return vec3<f32>(sx, sy, sz);
}

/// 世界顶点 = `rotate_vec3(局部顶点) + pos`（与 `WorldPoly::fill` 逐字同源）。
fn world_vert(q: vec4<f32>, pos: vec3<f32>, half: vec3<f32>, f: u32, k: u32) -> vec3<f32> {
    return rotate_axis(q, local_vert(half, f, k)) + pos;
}

/// 点到三角形最近点（标准区域分解；与 CPU `closest_point_on_triangle` 逐分支照抄）。
fn tri_closest(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, c: vec3<f32>) -> vec3<f32> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }
    let bp = p - b;
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        let denom = d1 - d3;
        if (abs(denom) > 1e-12) {
            return a + ab * (d1 / denom);
        }
        return a;
    }
    let cp = p - c;
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        let denom = d2 - d6;
        if (abs(denom) > 1e-12) {
            return a + ac * (d2 / denom);
        }
        return a;
    }
    let va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        let denom = (d4 - d3) + (d5 - d6);
        if (abs(denom) > 1e-12) {
            return b + (c - b) * ((d4 - d3) / denom);
        }
        return b;
    }
    let denom = va + vb + vc;
    if (abs(denom) > 1e-12) {
        let inv = 1.0 / denom;
        return a + ab * (vb * inv) + ac * (vc * inv);
    }
    return a;
}

/// 凸体（**只做盒**）最近点查询：与 CPU `closest_point_on_poly` 同序 —— 逐面判内外 + 记最大平面距，
/// 并按**扇形三角化** `(v0,v1,v2)`/`(v0,v2,v3)` 取最近点（严格 `<` ⇒ 平局取先到的面）。
fn closest_on_box(q: vec4<f32>, pos: vec3<f32>, half: vec3<f32>, p: vec3<f32>) -> Closest {
    var out: Closest;
    out.pt = vec3<f32>(0.0, 0.0, 0.0);
    out.d2 = F32_MAX;
    out.inside = true;
    out.in_n = vec3<f32>(0.0, 1.0, 0.0);
    var max_plane_d = F32_MIN;
    for (var f = 0u; f < 6u; f = f + 1u) {
        let n = rotate_axis(q, face_normal_local(f));
        let v0 = world_vert(q, pos, half, f, 0u);
        let d = dot(p - v0, n);
        if (d > 0.0) {
            out.inside = false;
        } else if (d > max_plane_d) {
            max_plane_d = d;
            out.in_n = n;
        }
        for (var k = 1u; k < 3u; k = k + 1u) {
            let tri = tri_closest(
                p,
                v0,
                world_vert(q, pos, half, f, k),
                world_vert(q, pos, half, f, k + 1u),
            );
            let dd = tri - p;
            let d2 = dot(dd, dd);
            if (d2 < out.d2) {
                out.d2 = d2;
                out.pt = tri;
            }
        }
    }
    return out;
}

/// 球×盒：与 CPU `sphere_convex_ab` 同分支（内部走 `max_plane_d`、外部走"最近点距离 < 半径"）。
/// 流形只有 1 点、`feature = 0`；`sphere_is_a` = false 时法线取反（CPU 的 `(convex, Sphere)` 臂）。
fn sphere_box(base: u32, sb: u32, bb: u32, sphere_is_a: bool) {
    let radius = as_f32(bodies[sb + P0_AT]);
    let center = body_pos(sb);
    let bq = body_quat(bb);
    let bpos = body_pos(bb);
    let bhalf = body_half(bb);
    let cl = closest_on_box(bq, bpos, bhalf, center);
    var n: vec3<f32>;
    var depth: f32;
    var point: vec3<f32>;
    if (cl.inside) {
        // 球心在盒内：max_plane_d < 0、表面距 = −max_plane_d。
        // ⚠️ CPU 在 `inside` 分支里**再算一次** `max_plane_d_of`（独立的一遍循环，顺序/平局规则同）。
        var max_d = F32_MIN;
        var in_n = vec3<f32>(0.0, 1.0, 0.0);
        for (var f = 0u; f < 6u; f = f + 1u) {
            let nf = rotate_axis(bq, face_normal_local(f));
            let d = dot(center - world_vert(bq, bpos, bhalf, f, 0u), nf);
            if (d > max_d) {
                max_d = d;
                in_n = nf;
            }
        }
        depth = radius + max_d;
        if (depth <= 0.0) {
            return; // 无流形
        }
        n = -in_n;
        point = center - in_n * max_d;
    } else {
        let dist = sqrt(cl.d2);
        if (dist >= radius) {
            return;
        }
        n = select((cl.pt - center) * (1.0 / dist), vec3<f32>(0.0, 1.0, 0.0), dist <= 1e-9);
        depth = radius - dist;
        point = cl.pt;
    }
    if (!sphere_is_a) {
        n = -n; // CPU 的 `(convex, Sphere)` 臂用同一路径后**翻转法线**
    }
    put_normal(base, n);
    put_point(base, 0u, point, depth, 0u);
    slots[base + W_CNT] = 1u;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= prm.n_pairs) {
        return;
    }
    let base = i * SLOT_WORDS;
    // 先整槽清零 ⇒ 未写的部分**确定性**（自证"连跑两次逐字相同"不受未初始化内存影响）。
    for (var k = 0u; k < SLOT_WORDS; k = k + 1u) {
        slots[base + k] = 0u;
    }
    let a = pairs[i * 2u];
    let b = pairs[i * 2u + 1u];
    slots[base] = a;
    slots[base + 1u] = b;
    if (a >= prm.n_bodies || b >= prm.n_bodies) {
        slots[base + W_CNT] = NOT_HANDLED;
        return;
    }
    let ba = a * BODY_WORDS;
    let bb = b * BODY_WORDS;
    let ka = bodies[ba + KIND_AT];
    let kb = bodies[bb + KIND_AT];
    if (ka == KIND_SPHERE && kb == KIND_SPHERE) {
        sphere_pair(base, ba, bb);
        return;
    }
    if (ka == KIND_BOX && kb == KIND_BOX) {
        boxes_pair(
            base,
            ba,
            bb,
            body_pos(ba),
            body_half(ba),
            box_axes_of(ba),
            body_pos(bb),
            body_half(bb),
            box_axes_of(bb),
        );
        return;
    }
    // 球×盒 / 盒×球（CPU 都走 `sphere_convex_ab`；盒×球那条臂算完再翻转法线）。
    if (ka == KIND_SPHERE && kb == KIND_BOX) {
        sphere_box(base, ba, bb, true);
        return;
    }
    if (ka == KIND_BOX && kb == KIND_SPHERE) {
        sphere_box(base, bb, ba, false);
        return;
    }
    slots[base + W_CNT] = NOT_HANDLED;
}
