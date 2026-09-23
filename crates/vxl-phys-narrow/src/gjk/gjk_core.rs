//! gjk_core：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// GJK 结果。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gjk {
    /// 分离：给最近点对与距离（均在世界系）。
    Separated {
        point_a: Vec3,
        point_b: Vec3,
        dist: f32,
    },
    /// 相交：交给 EPA 求穿透。
    Intersecting,
}

/// GJK 距离/相交判定（标准「单纯形最近点 + Voronoi 区域分类」；迭代上限固定 ⇒ 确定性）。
pub fn gjk(a: &dyn Support, b: &dyn Support) -> Gjk {
    match gjk_inner(a, b) {
        Ok(g) => g,
        Err(_) => Gjk::Intersecting,
    }
}

/// GJK 主循环：分离 ⇒ `Ok(Separated)`（含距离与见证点）；相交 ⇒ `Err(终止单纯形)`（供 EPA 使用）。
pub(crate) fn gjk_inner(a: &dyn Support, b: &dyn Support) -> Result<Gjk, Vec<(Vec3, Vec3, Vec3)>> {
    let mut dir = a.interior() - b.interior();
    if dir.length_squared() < 1e-12 {
        dir = Vec3::X;
    }
    let mut simplex: Vec<(Vec3, Vec3, Vec3)> = Vec::with_capacity(4);
    let mut prev = f32::INFINITY;
    let mut last = (Vec3::ZERO, [0.0f32; 4]);
    for _ in 0..32 {
        let sa = a.support(dir);
        let sb = b.support(-dir);
        let w = sa - sb;
        if simplex.len() < 4 {
            simplex.push((w, sa, sb));
        }
        let (closest, bary) = closest_simplex(&mut simplex);
        last = (closest, bary);
        let d2 = closest.length_squared();
        if d2 < 1e-12 {
            return Err(simplex); // 原点落在单纯形上 ⇒ 相交
        }
        if d2 >= prev {
            // 距离无改善 ⇒ 收敛（分离）；见证点按重心坐标插值
            let (wa, wb) = interp(&simplex, &bary);
            return Ok(Gjk::Separated {
                point_a: wa,
                point_b: wb,
                dist: d2.sqrt(),
            });
        }
        prev = d2;
        dir = -closest;
    }
    let (wa, wb) = interp(&simplex, &last.1);
    Ok(Gjk::Separated {
        point_a: wa,
        point_b: wb,
        dist: last.0.length(),
    })
}

/// 见证点插值（重心坐标权重定义在**缩减后**的单纯形上）。
pub(crate) fn interp(s: &[(Vec3, Vec3, Vec3)], bary: &[f32; 4]) -> (Vec3, Vec3) {
    let mut wa = Vec3::ZERO;
    let mut wb = Vec3::ZERO;
    for (i, v) in s.iter().enumerate() {
        let w = bary[i];
        wa += v.1 * w;
        wb += v.2 * w;
    }
    (wa, wb)
}

/// 单纯形上离原点最近的点（顺带把单纯形**缩减到承载该点的最小子集**）。
/// 教科书 Voronoi 区域分类（点/线段/三角形/四面体）。
pub(crate) fn closest_simplex(s: &mut Vec<(Vec3, Vec3, Vec3)>) -> (Vec3, [f32; 4]) {
    match s.len() {
        1 => (s[0].0, [1.0, 0.0, 0.0, 0.0]),
        2 => {
            let (a, b) = (s[0].0, s[1].0);
            let ab = b - a;
            let t = (-a.dot(ab) / ab.length_squared().max(1e-18)).clamp(0.0, 1.0);
            if t <= 0.0 {
                (a, [1.0, 0.0, 0.0, 0.0])
            } else if t >= 1.0 {
                let p = s[1];
                s.truncate(1);
                s[0] = p;
                (b, [1.0, 0.0, 0.0, 0.0])
            } else {
                (a + ab * t, [1.0 - t, t, 0.0, 0.0])
            }
        }
        3 => {
            let (a, b, c) = (s[0].0, s[1].0, s[2].0);
            let ab = b - a;
            let ac = c - a;
            let n = ab.cross(ac);
            let nn = n.length_squared();
            if nn < 1e-18 {
                // 退化三角形：按线段处理
                let p = s[1];
                s.truncate(1);
                s[0] = p;
                return closest_simplex(s);
            }
            // 三个边的外法向（在三角形平面内、背离第三点）
            let mut best = (f32::INFINITY, Vec3::ZERO, [0.0f32; 4]);
            let edges = [(0usize, 1usize, 2usize), (0, 2, 1), (1, 2, 0)];
            for &(i, j, k) in &edges {
                let (p, q, r) = (s[i].0, s[j].0, s[k].0);
                let e = q - p;
                let mut en = e.cross(n); // 平面内垂直于 e
                if en.dot(r - p) > 0.0 {
                    en = -en;
                }
                let d = p.dot(en);
                if d >= 0.0 {
                    continue; // 原点在该边内侧
                }
                // 原点在外侧 ⇒ 最近点落在该边（或其端点）：按线段求
                let t = (-p.dot(e) / e.length_squared().max(1e-18)).clamp(0.0, 1.0);
                let proj = p + e * t;
                let dist = proj.length_squared();
                if dist < best.0 {
                    let mut bary = [0.0f32; 4];
                    bary[i] = 1.0 - t;
                    bary[j] = t;
                    best = (dist, proj, bary);
                }
            }
            if best.0.is_finite() {
                // 缩减到承载边/点
                rebuild(s, &best.2);
                return (best.1, best.2);
            }
            // 原点在三角形内（平面内）
            let d = -a.dot(n) / nn.sqrt();
            let _ = d;
            (Vec3::ZERO + n * 0.0, [0.0; 4])
        }
        _ => {
            // 四面体：四个面的外法向，原点在全部内侧 ⇒ 相交（closest ≈ 0）
            let idx = [[0usize, 1, 2, 3], [0, 3, 1, 2], [0, 2, 3, 1], [1, 3, 2, 0]];
            for f in &idx {
                let (p0, p1, p2, p3) = (s[f[0]].0, s[f[1]].0, s[f[2]].0, s[f[3]].0);
                let mut n = (p1 - p0).cross(p2 - p0);
                if n.length_squared() < 1e-18 {
                    continue;
                }
                n = n.normalize();
                if n.dot(p3 - p0) > 0.0 {
                    n = -n; // 外向（背离第四点）
                }
                if p0.dot(n) > 1e-9 {
                    // 原点在该面外侧 ⇒ 最近点落在该面（三点单纯形）
                    let keep = [f[0], f[1], f[2]];
                    let pts: Vec<(Vec3, Vec3, Vec3)> = keep.iter().map(|&i| s[i]).collect();
                    *s = pts;
                    return closest_simplex(s);
                }
            }
            (Vec3::ZERO, [0.25, 0.25, 0.25, 0.25])
        }
    }
}

/// 把单纯形缩减到「重心坐标非零」的子集（保持原顺序 ⇒ 确定性）。
pub(crate) fn rebuild(s: &mut Vec<(Vec3, Vec3, Vec3)>, bary: &[f32; 4]) {
    let keep: Vec<(Vec3, Vec3, Vec3)> = s
        .iter()
        .enumerate()
        .filter(|(i, _)| bary[*i] > 0.0)
        .map(|(_, v)| *v)
        .collect();
    *s = keep;
}
