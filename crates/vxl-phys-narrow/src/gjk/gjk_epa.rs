//! gjk_epa：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// EPA：穿透深度与法线（**仅在 GJK 判定相交时调用**）。
/// 返回 `(法线, 深度, 见证点)`：法线由 b 指向 a（沿它平移 a 可分离）。
pub fn epa(a: &dyn Support, b: &dyn Support, iters: usize) -> Option<(Vec3, f32, Vec3)> {
    let simplex = match gjk_inner(a, b) {
        Err(s) => s,
        Ok(_) => return None,
    };
    epa_from_simplex(a, b, simplex, iters)
}

/// 已持有 GJK 终止单纯形时的 EPA（避免重复跑 GJK）。
pub fn epa_from_simplex(
    a: &dyn Support,
    b: &dyn Support,
    simplex: Vec<(Vec3, Vec3, Vec3)>,
    iters: usize,
) -> Option<(Vec3, f32, Vec3)> {
    // 初始多面体：4 个非退化顶点；退化（切向接触等）⇒ 保守估计
    let mut verts: Vec<Vec3> = Vec::new();
    let mut wit: Vec<(Vec3, Vec3)> = Vec::new();
    for v in &simplex {
        if verts.iter().all(|q| (*q - v.0).length_squared() > 1e-12) {
            verts.push(v.0);
            wit.push((v.1, v.2));
        }
    }
    // 退化单纯形（轴对齐对称接触 / 真贴面：原点正落在点/线/面上）⇒ 6 轴 SAT。
    // **必须取三轴最小重叠**（贴面时某轴重叠恰为 0 ⇒ 深度 0；负值 ⇒ 分离），
    // 只看「任一轴重叠」会把相邻体（X 轴重叠、Z 轴不重叠）误判成 0.5 深穿透。
    let coplanar = if verts.len() == 4 {
        let d = (verts[1] - verts[0])
            .cross(verts[2] - verts[0])
            .dot(verts[3] - verts[0])
            .abs();
        d < 1e-9
    } else {
        false
    };
    if verts.len() < 4 || coplanar {
        return axis_sat(a, b);
    }
    verts.truncate(4);
    wit.truncate(4);
    let mut faces: Vec<[usize; 3]> = vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];
    let c = (verts[0] + verts[1] + verts[2] + verts[3]) * 0.25;
    for f in faces.iter_mut() {
        if face_n(&verts, f).dot(c - verts[f[0]]) > 0.0 {
            f.swap(1, 2);
        }
    }
    let mut best_n = Vec3::X;
    let mut best_d = 0.0f32;
    let mut best_p = wit[0].0;
    for _ in 0..iters {
        // 最近面（外法向 ⇒ n·p ≥ 0）
        let mut bi = usize::MAX;
        let mut bd = f32::INFINITY;
        let mut bn = Vec3::X;
        for (i, fc) in faces.iter().enumerate() {
            let n = face_n(&verts, fc);
            if n.length_squared() < 1e-18 {
                continue;
            }
            let n = n.normalize();
            let d = n.dot(verts[fc[0]]);
            if d < bd {
                bd = d;
                bi = i;
                bn = n;
            }
        }
        if bi == usize::MAX {
            break;
        }
        best_n = bn;
        best_d = bd.max(0.0);
        let f = faces[bi];
        // 面内投影重心坐标 ⇒ 见证点（两侧见证点中点）
        let (p0, p1, p2) = (verts[f[0]], verts[f[1]], verts[f[2]]);
        let bar = bary3(p0, p1, p2, bn * bd);
        best_p = (wit[f[0]].0 * bar[0]
            + wit[f[1]].0 * bar[1]
            + wit[f[2]].0 * bar[2]
            + wit[f[0]].1 * bar[0]
            + wit[f[1]].1 * bar[1]
            + wit[f[2]].1 * bar[2])
            * 0.5;
        // 支撑点：若未越过该面 ⇒ 收敛
        let sa = a.support(bn);
        let sb = b.support(-bn);
        let w = sa - sb;
        if w.dot(bn) - bd < 1e-4 {
            break;
        }
        // 剔除可见面 → 重建地平线
        let mut visible: Vec<usize> = Vec::new();
        for (i, fc) in faces.iter().enumerate() {
            let n = face_n(&verts, fc);
            if n.length_squared() < 1e-18 {
                visible.push(i);
                continue;
            }
            if n.normalize().dot(w - verts[fc[0]]) > 1e-9 {
                visible.push(i);
            }
        }
        if visible.is_empty() {
            break;
        }
        let mut horizon: Vec<(usize, usize)> = Vec::new();
        for &i in &visible {
            let fc = faces[i];
            for e in [(fc[0], fc[1]), (fc[1], fc[2]), (fc[2], fc[0])] {
                let k = (e.0.min(e.1), e.0.max(e.1));
                match horizon
                    .iter()
                    .position(|&(x, y)| x.min(y) == k.0 && x.max(y) == k.1)
                {
                    Some(pos) => {
                        horizon.remove(pos);
                    }
                    None => horizon.push(k),
                }
            }
        }
        if horizon.is_empty() {
            break;
        }
        let wi = verts.len();
        verts.push(w);
        wit.push((sa, sb));
        for i in visible.iter().rev() {
            faces.remove(*i);
        }
        for (e0, e1) in horizon {
            faces.push([e0, e1, wi]);
        }
        if faces.len() > 64 {
            break;
        }
    }
    Some((best_n, best_d, best_p))
}

/// 6 轴 SAT 回退（轴对齐构型即解析深度；三轴取**最小重叠**）。
/// `None` = 沿某轴已分离（分离距离 ≤ -1e-4）；返回 `(法线 b→a, 深度, 见证点)`。
pub(crate) fn axis_sat(a: &dyn Support, b: &dyn Support) -> Option<(Vec3, f32, Vec3)> {
    let mut best_n = Vec3::X;
    let mut best_over = f32::INFINITY;
    let mut best_p = Vec3::ZERO;
    for &ax in &[Vec3::X, Vec3::Y, Vec3::Z] {
        let a_min = a.support(-ax).dot(ax);
        let a_max = a.support(ax).dot(ax);
        let b_min = b.support(-ax).dot(ax);
        let b_max = b.support(ax).dot(ax);
        let over = a_max.min(b_max) - a_min.max(b_min);
        if over < best_over {
            best_over = over;
            // a 在低侧 ⇒ 把 a 推离 b 的方向为 −ax
            best_n = if a_max <= b_max { -ax } else { ax };
            best_p = (a.support(best_n) + b.support(-best_n)) * 0.5;
        }
    }
    if best_over < -1e-4 {
        return None;
    }
    Some((best_n, best_over.max(0.0), best_p))
}

/// 面外法向（未归一化）。
pub(crate) fn face_n(v: &[Vec3], f: &[usize; 3]) -> Vec3 {
    (v[f[1]] - v[f[0]]).cross(v[f[2]] - v[f[0]])
}

/// 点到三角形的重心坐标。
pub(crate) fn bary3(p0: Vec3, p1: Vec3, p2: Vec3, q: Vec3) -> [f32; 3] {
    let v0 = p1 - p0;
    let v1 = p2 - p0;
    let v2 = q - p0;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let den = d00 * d11 - d01 * d01;
    if den.abs() < 1e-18 {
        return [1.0, 0.0, 0.0];
    }
    let v = (d11 * d20 - d01 * d21) / den;
    let w = (d00 * d21 - d01 * d20) / den;
    [1.0 - v - w, v, w]
}

/// 便捷：外壳 vs 盒 的穿透（相交时）；未相交返回 None。
/// 返回 `(法线 b→a, 深度, 见证点)`。
pub fn hull_box_penetration(
    hull: &ConvexHull,
    hpos: Vec3,
    hrot: Quat,
    half: Vec3,
    bpos: Vec3,
    brot: Quat,
    iters: usize,
) -> Option<(Vec3, f32, Vec3)> {
    let ha = HullSupport {
        hull,
        pos: hpos,
        rot: Mat3::from_quat(hrot),
    };
    let bs = BoxSupport {
        half,
        pos: bpos,
        rot: Mat3::from_quat(brot),
    };
    match gjk_inner(&ha, &bs) {
        Err(s) => epa_from_simplex(&ha, &bs, s, iters),
        Ok(_) => None,
    }
}
