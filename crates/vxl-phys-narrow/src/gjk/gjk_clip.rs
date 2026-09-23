//! gjk_clip：从 gjk.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 半空间裁剪（保留 `n·p ≤ d` 侧）。
///
/// 保留侧内点 + 跨立线段与平面的交点；**交点只保留其在切平面内的 2D 凸包顶点**
/// —— 其余交点都是这些顶点的凸组合（共面）⇒ 对 `conv()` 无贡献（数学精确），
/// 但把每轮点云规模从 O(n²) 爆炸钉在 O(n)（否则下一轮两两枚举再放大）。
pub fn clip_halfspace(points: &[Vec3], n: Vec3, d: f32) -> Vec<Vec3> {
    let m = points.len();
    let mut keep: Vec<Vec3> = Vec::with_capacity(m + 8);
    for p in points {
        if n.dot(*p) <= d {
            keep.push(*p);
        }
    }
    let mut cross: Vec<Vec3> = Vec::new();
    for i in 0..m {
        for j in (i + 1)..m {
            let (a, b) = (points[i], points[j]);
            let da = n.dot(a) - d;
            let db = n.dot(b) - d;
            if (da > 0.0) != (db > 0.0) {
                let t = da / (da - db);
                cross.push(a + (b - a) * t);
            }
        }
    }
    if cross.len() > 3 {
        cross = hull_2d_on_plane(&cross, n);
    }
    dedup_by_dist(&mut keep);
    keep.extend(cross);
    dedup_by_dist(&mut keep);
    keep
}

/// 共面点集的 2D 凸包顶点（单调链；确定性：投影基与排序全由输入决定）。
pub(crate) fn hull_2d_on_plane(pts: &[Vec3], n: Vec3) -> Vec<Vec3> {
    // 平面内正交基：取与 n 最不平行的坐标轴叉乘（确定性）。
    let ax = if n.x.abs() <= n.y.abs() && n.x.abs() <= n.z.abs() {
        Vec3::X
    } else if n.y.abs() <= n.z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let u = ax.cross(n).normalize();
    let v = n.cross(u); // 右手系（u, v, n）
    let mut q: Vec<(f32, f32, Vec3)> = pts.iter().map(|p| (p.dot(u), p.dot(v), *p)).collect();
    q.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(core::cmp::Ordering::Equal)
            .then(a.1.partial_cmp(&b.1).unwrap_or(core::cmp::Ordering::Equal))
    });
    let cross2 = |o: (f32, f32), a: (f32, f32), b: (f32, f32)| -> f32 {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut lower: Vec<(f32, f32, Vec3)> = Vec::new();
    for &p in &q {
        while lower.len() >= 2
            && cross2(
                (lower[lower.len() - 2].0, lower[lower.len() - 2].1),
                (lower[lower.len() - 1].0, lower[lower.len() - 1].1),
                (p.0, p.1),
            ) <= 0.0
        {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<(f32, f32, Vec3)> = Vec::new();
    for &p in q.iter().rev() {
        while upper.len() >= 2
            && cross2(
                (upper[upper.len() - 2].0, upper[upper.len() - 2].1),
                (upper[upper.len() - 1].0, upper[upper.len() - 1].1),
                (p.0, p.1),
            ) <= 0.0
        {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower.into_iter().map(|t| t.2).collect()
}

/// 近重复点去重（1e-5 容差；保序扫描 ⇒ 确定性）。（1e-5 容差；保序扫描 ⇒ 确定性）。
pub(crate) fn dedup_by_dist(v: &mut Vec<Vec3>) {
    let mut i = 0;
    while i < v.len() {
        let mut j = i + 1;
        while j < v.len() {
            if (v[j] - v[i]).length_squared() < 1e-10 {
                v.remove(j);
            } else {
                j += 1;
            }
        }
        i += 1;
    }
}
