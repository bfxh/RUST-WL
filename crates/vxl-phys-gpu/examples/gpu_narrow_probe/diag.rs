//! 窄相探针的**判据与诊断**：容差 / 比较 / 双射匹配 / 软差分类 / dump。
//! （主文件 `main.rs` 只留编排与打印；本文件被 `mod diag;` 收进来 ⇒ 条目对父模块可见即可。）
//!
//! **判据形态见 `PLAN-gpu.md` §17.7**：逐元素「次序 + 逐位」在本栈**不可达**——
//! `select_contacts` 的排序 / 去重 / 截断是**离散选择**，ulp 级差异会翻它（实测：同位次 1 ulp 的
//! 坐标差 + `-0.5` vs `-0.49999997`）。⇒ 改**集合口径**：逐对结构 + **`feature` 多重集逐位** +
//! **点集双射**（容差内一对一）+ 法线容差；点的次序与同位次 ulp 噪声**只报不判**。

use super::*;

/// 判据的三项容差（按场景尺度算出的实际值）。
#[derive(Clone, Copy)]
pub(super) struct Tol {
    pub(super) pt: f32,
    pub(super) depth: f32,
    pub(super) nrm: f32,
}

/// 场景坐标尺度 = 最大 `|pos|` 分量 —— ulp 级残差的上界尺度。
pub(super) fn coord_scale(w: &World) -> f32 {
    (0..w.bodies.len())
        .map(|i| {
            let p = w.bodies.position[i];
            p.x.abs().max(p.y.abs()).max(p.z.abs())
        })
        .fold(0.0f32, f32::max)
}

/// 按尺度放宽后的容差：地板 ⊕ `2ε·尺度`（实测残差 ≲ 1 ulp 的分数 —— 见 §17.7 读数）。
pub(super) fn tol_of(w: &World) -> Tol {
    let s = 2.0 * f32::EPSILON * coord_scale(w);
    Tol {
        pt: TOL_PT.max(s),
        depth: TOL_DEPTH.max(s),
        nrm: TOL_NRM.max(s),
    }
}

/// 分族统计「逐位相同 / 总数」的流形数：`[球×球, 盒×盒(两者都不转), 盒×盒(有转)]`。
/// **证据用途**：对齐盒的坐标全可精确表示（整数/半整数），若那一族 100% 逐位相同 ⇒ 公式链本身
/// 没错，剩下的差只可能来自**旋转带来的舍入/收缩**（口径 B）——比"总残差小"更有说服力。
pub(super) fn per_family(cpu: &[Manifold], gpu: &[Manifold], w: &World) -> [(u32, u32); 3] {
    let mut out = [(0u32, 0u32); 3];
    let idq = |i: u32| w.bodies.rot(i as usize) == Quat::IDENTITY;
    for (c, g) in cpu.iter().zip(gpu.iter()) {
        let (sa, sb) = (w.bodies.shape[c.a as usize], w.bodies.shape[c.b as usize]);
        let k = if matches!(sa, Shape::Sphere { .. }) && matches!(sb, Shape::Sphere { .. }) {
            0
        } else if idq(c.a) && idq(c.b) {
            1
        } else {
            2
        };
        out[k].1 += 1;
        if point_multiset_bits_eq(c, g) && order_same(c, g) {
            out[k].0 += 1;
        }
    }
    out
}

/// 不符分解（条数）：对号 / 点数 / space / **特征多重集** / 法线超差 / 点集双射失败 / **软差**。
#[derive(Default)]
pub(super) struct Delta {
    /// **判定量**：配对后的最大点距/深度差（次序无关）。
    pub(super) max_pt: f32,
    pub(super) max_depth: f32,
    pub(super) max_nrm: f32,
    /// 逐位对齐口径的最大点距（**含次序噪声**，只报不判）。
    pub(super) pt_idx: f32,
    pub(super) n_pt: usize,
    pub(super) bit_pt: usize,
    pub(super) k_ab: u32,
    pub(super) k_len: u32,
    pub(super) k_space: u32,
    pub(super) k_feat: u32,
    pub(super) k_nrm: u32,
    pub(super) k_bij: u32,
    /// 硬项全过、只是**点的次序**不同（**纯置换**：点多重集逐位相同）。
    pub(super) k_perm: u32,
    /// 硬项全过，同位次上有点差 ≤ 容差（**ulp 噪声**，口径 B 的常规残差）。
    pub(super) k_ulp: u32,
    pub(super) ord_at: Option<usize>,
    pub(super) ulp_at: Option<usize>,
    /// 首次不符的流形下标（用来 dump 双方数据）。
    pub(super) mism_at: Option<usize>,
    /// 不符的**对**（去重后；用来与"CPU 自敏感"命中的对取交集）。
    pub(super) mism_pairs: Vec<(u32, u32)>,
    pub(super) mism: Option<String>,
}

/// 特征**多重集**是否相同（升序逐位比）—— 点的次序不算，特征集合必须严丝合缝。
fn feat_multiset_eq(c: &Manifold, g: &Manifold) -> bool {
    let mut cf: Vec<u32> = c.points.iter().map(|p| p.feature).collect();
    let mut gf: Vec<u32> = g.points.iter().map(|p| p.feature).collect();
    cf.sort_unstable();
    gf.sort_unstable();
    cf == gf
}

/// 点集**双射匹配**（n ≤ 4 ⇒ O(n²) 贪心最近邻）：每个 CPU 点配一个互不重复的卡上点，
/// 配对距离与深度差都要在容差内。返回 `(是否配得上, 配对里的最大点距, 最大深度差)`。
fn bij_match(c: &Manifold, g: &Manifold, tol: &Tol) -> (bool, f32, f32) {
    let n = c.points.len();
    let mut used = [false; 4];
    let (mut mp, mut md) = (0.0f32, 0.0f32);
    for i in 0..n {
        let mut best = f32::MAX;
        let mut bi = usize::MAX;
        for (j, u) in used.iter().enumerate().take(n) {
            if *u {
                continue;
            }
            let d = (c.points[i].point - g.points[j].point).length();
            let dd = (c.points[i].depth - g.points[j].depth).abs();
            if d <= tol.pt && dd <= tol.depth && d < best {
                best = d;
                bi = j;
            }
        }
        if bi == usize::MAX {
            return (false, mp, md);
        }
        used[bi] = true;
        mp = mp.max(best);
        let dd = (c.points[i].depth - g.points[bi].depth).abs();
        md = md.max(dd);
    }
    (true, mp, md)
}

/// 点**多重集**是否逐位相同（按位模式排序后逐位比）—— 相同而次序不同 = **纯置换**（接触内容一模一样）。
fn point_multiset_bits_eq(c: &Manifold, g: &Manifold) -> bool {
    let key = |m: &Manifold| {
        let mut v: Vec<[u32; 5]> = m
            .points
            .iter()
            .map(|p| {
                [
                    p.point.x.to_bits(),
                    p.point.y.to_bits(),
                    p.point.z.to_bits(),
                    p.depth.to_bits(),
                    p.feature,
                ]
            })
            .collect();
        v.sort_unstable();
        v
    };
    key(c) == key(g)
}

/// 逐点**次序**是否一致（不含容差：位置/深度按位比）。
fn order_same(c: &Manifold, g: &Manifold) -> bool {
    c.points.len() == g.points.len()
        && (0..c.points.len()).all(|j| {
            let (cp, gp) = (&c.points[j], &g.points[j]);
            cp.point.x.to_bits() == gp.point.x.to_bits()
                && cp.point.y.to_bits() == gp.point.y.to_bits()
                && cp.point.z.to_bits() == gp.point.z.to_bits()
                && cp.depth.to_bits() == gp.depth.to_bits()
                && cp.feature == gp.feature
        })
}

fn cmp_one(k: usize, c: &Manifold, g: &Manifold, tol: &Tol, d: &mut Delta) {
    let mark = |d: &mut Delta| {
        if d.mism_at.is_none() {
            d.mism_at = Some(k);
        }
        d.mism_pairs.push((c.a, c.b));
    };
    if c.a != g.a || c.b != g.b {
        d.k_ab += 1;
        mark(d);
        d.mism.get_or_insert(format!(
            "第 {k} 条流形对号不同：CPU ({}, {}) vs 卡上 ({}, {})",
            c.a, c.b, g.a, g.b
        ));
        return;
    }
    if c.points.len() != g.points.len() {
        d.k_len += 1;
        mark(d);
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）点数不同：CPU {} vs 卡上 {}",
            c.a,
            c.b,
            c.points.len(),
            g.points.len()
        ));
        return;
    }
    if c.points.space() != g.points.space() {
        d.k_space += 1;
        mark(d);
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）space 不同：{} vs {}",
            c.a,
            c.b,
            c.points.space(),
            g.points.space()
        ));
    }
    if !feat_multiset_eq(c, g) {
        d.k_feat += 1;
        mark(d);
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）**特征多重集**不同",
            c.a, c.b
        ));
    }
    let dn = (c.normal - g.normal).abs();
    d.max_nrm = d.max_nrm.max(dn.x).max(dn.y).max(dn.z);
    if dn.x > tol.nrm || dn.y > tol.nrm || dn.z > tol.nrm {
        d.k_nrm += 1;
        mark(d);
    }
    let (bij, bpt, bdep) = bij_match(c, g, tol);
    if !bij {
        d.k_bij += 1;
        mark(d);
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）**点集双射匹配失败**（不是次序问题）",
            c.a, c.b
        ));
    }
    // 判定量 = **配对后**的最大点距/深度差（次序无关）；逐位对齐口径另报（含次序噪声）。
    d.max_pt = d.max_pt.max(bpt);
    d.max_depth = d.max_depth.max(bdep);
    for j in 0..c.points.len() {
        let (cp, gp) = (&c.points[j], &g.points[j]);
        d.n_pt += 1;
        d.pt_idx = d.pt_idx.max((cp.point - gp.point).length());
        let same = cp.point.x.to_bits() == gp.point.x.to_bits()
            && cp.point.y.to_bits() == gp.point.y.to_bits()
            && cp.point.z.to_bits() == gp.point.z.to_bits()
            && cp.depth.to_bits() == gp.depth.to_bits();
        if same {
            d.bit_pt += 1;
        }
    }
    // 硬项全过后还剩两种"软差"（都不计入判定，但分开数清楚）：
    //   ① 纯置换：点**多重集逐位相同**、只是次序不同；
    //   ② ulp 噪声：同位次上有个点差 ≤ 容差（口径 B 的常规残差）。
    if d.mism_at != Some(k) && !order_same(c, g) && c.points.len() > 1 {
        if point_multiset_bits_eq(c, g) {
            d.k_perm += 1;
            if d.ord_at.is_none() {
                d.ord_at = Some(k);
            }
        } else {
            d.k_ulp += 1;
            if d.ulp_at.is_none() {
                d.ulp_at = Some(k);
            }
        }
    }
}

/// 全表比较 ⇒ `(通过?, 残差)`。判据 = 逐对结构 + **特征多重集逐位** + **点集双射（容差内）** +
/// 法线容差；点的**次序**不计入判定（见档头），但单独报出来。
pub(super) fn compare(cpu: &[Manifold], gpu: &[Manifold], tol: &Tol) -> (bool, Delta) {
    let mut d = Delta::default();
    if cpu.len() != gpu.len() {
        d.mism = Some(format!(
            "流形总数不同：CPU {} vs 卡上档 {}",
            cpu.len(),
            gpu.len()
        ));
        return (false, d);
    }
    for (k, (c, g)) in cpu.iter().zip(gpu.iter()).enumerate() {
        cmp_one(k, c, g, tol, &mut d);
    }
    d.mism_pairs.sort_unstable();
    d.mism_pairs.dedup();
    let pass =
        d.mism.is_none() && d.max_pt <= tol.pt && d.max_depth <= tol.depth && d.max_nrm <= tol.nrm;
    (pass, d)
}

/// 位置扰动：**每轴 ±1 ulp 的相对量**（`p·(1+1.2e-7·k)`，k 逐体/逐轴取 −1/0/+1）——**只动位置**，
/// 配对表由调用方保持不变。为什么不用绝对量：绝对扰动（如 4e-6）在 1 m 量级上等于 30+ ulp，
/// 会把"精确平局"直接破坏掉（于是测不出刀刃），必须压到 1 ulp 才是同一把尺。
fn perturb(w: &mut World, _base: f32) {
    let ulp = 1.2e-7f32;
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        let k = |m: usize| ((i / m) % 3) as f32 - 1.0;
        w.bodies.position[i] = Vec3::new(
            p.x * (1.0 + ulp * k(1)),
            p.y * (1.0 + ulp * k(3)),
            p.z * (1.0 + ulp * k(9)),
        );
    }
}

/// **CPU 自敏感**（本仓的老办法，见 `vxl-phys-measurement-protocol`）：另建一个同布局场景、把位置
/// 扰动 **1 ulp**，**配对表用同一份**（配对不动）⇒ 若某些对的答案跟着翻（次序也算），说明那些对
/// 本来就落在刀刃上。返回 `(CPU 自敏感残差, 与「CPU↔卡上不符」的交集对数)`。
///
/// ⚠️ 实测**它对本片的软差给 0**（旋转路径不灵）——**0 不等于"软差没事"**，别把它当万能尺；
/// 机制证据看 `per_family` 的分族逐位率。
pub(super) fn cpu_self_sensitivity(
    pairs: &[(u32, u32)],
    cpu_ref: &[Manifold],
    np: &mut DefaultNarrowPhase,
    jobs: &ScopedPool,
    mism_pairs: &[(u32, u32)],
    tol: &Tol,
) -> (Delta, usize) {
    let mut sc2 = scene();
    perturb(&mut sc2.w, 4e-6);
    let mut cpu_eps: Vec<Manifold> = Vec::new();
    np.collide(&sc2.w.bodies, pairs, &[], &NoProviders, &mut cpu_eps, jobs);
    let (_p, d_eps) = compare(cpu_ref, &cpu_eps, tol);
    let both = mism_pairs
        .iter()
        .filter(|p| d_eps.mism_pairs.binary_search(p).is_ok())
        .count();
    (d_eps, both)
}

/// 首处不符的**双份数据**（CPU 参考 vs 卡上槽）+ 两个体的位姿/形状 —— 定位差异的起点。
/// 坐标/深度用 `{:?}`（往返最短表示）⇒ ulp 级差异看得见。
pub(super) fn dump_pair(k: usize, c: &Manifold, g: &Manifold, w: &World) {
    let f6 = |v: Vec3| format!("({:?},{:?},{:?})", v.x, v.y, v.z);
    println!("       ⚠️ 差异样本 第 {k} 条流形（对 {}、{}）：", c.a, c.b);
    println!(
        "         CPU  n={} 点数 {} | 点 {} 深度 {:?} 特征 {:#010x}",
        f6(c.normal),
        c.points.len(),
        f6(c.points[0].point),
        c.points[0].depth,
        c.points[0].feature
    );
    println!(
        "         卡上 n={} 点数 {} | 点 {} 深度 {:?} 特征 {:#010x}",
        f6(g.normal),
        g.points.len(),
        f6(g.points[0].point),
        g.points[0].depth,
        g.points[0].feature
    );
    for j in 1..c.points.len().min(g.points.len()) {
        println!(
            "         第 {j} 点 CPU {} d={:?} f={:#010x} | 卡上 {} d={:?} f={:#010x}",
            f6(c.points[j].point),
            c.points[j].depth,
            c.points[j].feature,
            f6(g.points[j].point),
            g.points[j].depth,
            g.points[j].feature
        );
    }
    for (tag, i) in [("a", c.a), ("b", c.b)] {
        println!(
            "         体 {tag}={i} pos={} rot={:?} 形状={:?}",
            f6(w.bodies.position[i as usize]),
            w.bodies.rot(i as usize),
            w.bodies.shape[i as usize]
        );
    }
}
