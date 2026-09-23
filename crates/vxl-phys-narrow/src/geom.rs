//! geom：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) fn poly_key(s: &Shape) -> u64 {
    pub(crate) fn mix(tag: u64, a: f32, b: f32, c: f32) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64 ^ tag;
        for v in [a, b, c] {
            h ^= (v.to_bits() as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15);
            h = h.wrapping_mul(0x100_0000_01b3);
        }
        h
    }
    match *s {
        Shape::Box { half } => mix(1, half.x, half.y, half.z),
        Shape::Cylinder {
            half_height,
            radius,
        } => mix(2, half_height, radius, CYLINDER_SEGMENTS as f32),
        Shape::Cone {
            half_height,
            radius,
        } => mix(3, half_height, radius, CYLINDER_SEGMENTS as f32),
        _ => 0,
    }
}

/// 点到三角形最近点（标准区域分解）。
pub(crate) fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        if denom.abs() > 1e-12 {
            return a + ab * (d1 / denom);
        }
        return a;
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        if denom.abs() > 1e-12 {
            return a + ac * (d2 / denom);
        }
        return a;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        if denom.abs() > 1e-12 {
            return b + (c - b) * ((d4 - d3) / denom);
        }
        return b;
    }
    let denom = va + vb + vc;
    if denom.abs() > 1e-12 {
        let inv = 1.0 / denom;
        let v = vb * inv;
        let w = vc * inv;
        a + ab * v + ac * w
    } else {
        a
    }
}

/// 凸体最近点查询：返回 (最近点, dist², 是否内部, 内部时最大平面距的面法线)。
pub(crate) fn closest_point_on_poly(poly: &WorldPoly, p: Vec3) -> (Vec3, f32, bool, Vec3) {
    let mut best = Vec3::ZERO;
    let mut best_d2 = f32::MAX;
    let faces = poly.face_normal.len();
    let mut inside = true;
    let mut max_plane_d = f32::MIN;
    let mut max_plane_n = Vec3::Y;
    for f in 0..faces {
        let s = poly.face_start[f] as usize;
        let e = poly.face_start[f + 1] as usize;
        let n = poly.face_normal[f];
        let v0 = poly.verts[s];
        let d = (p - v0).dot(n);
        if d > 0.0 {
            inside = false;
        } else if d > max_plane_d {
            max_plane_d = d;
            max_plane_n = n;
        }
        // 扇形三角化最近点。
        for k in (s + 1)..(e - 1) {
            let q = closest_point_on_triangle(p, v0, poly.verts[k], poly.verts[k + 1]);
            let d2 = (p - q).length_squared();
            if d2 < best_d2 {
                best_d2 = d2;
                best = q;
            }
        }
    }
    (best, best_d2, inside, max_plane_n)
}
