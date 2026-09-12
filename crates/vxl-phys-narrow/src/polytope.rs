//! 凸多面体（碰撞近似）：盒与圆柱（N 边内接棱柱，§2.4）。
//!
//! 面顶点「按面展开」存储（同一顶点在多面重复），使 SAT 裁剪与入射面提取
//! 都在连续切片上进行。绕向统一为：从外侧看逆时针（Newell 法线 = 面外法线）。

#![forbid(unsafe_code)]

use vxl_phys_core::Vec3;

#[derive(Clone, Debug, Default)]
pub struct ConvexPolytope {
    /// 展开顶点：面 i 的顶点 = verts[face_start[i]..face_start[i+1]]。
    pub verts: Vec<Vec3>,
    pub face_normal: Vec<Vec3>,
    pub face_start: Vec<u32>,
    /// 去重后的棱方向（SAT 边轴用）。
    pub edge_dirs: Vec<Vec3>,
}

/// Newell 法线（对任意平面多边形稳健）。
pub fn newell_normal(verts: &[Vec3]) -> Vec3 {
    let n = verts.len();
    let mut nx = 0.0f32;
    let mut ny = 0.0f32;
    let mut nz = 0.0f32;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
    }
    Vec3::new(nx, ny, nz).normalize()
}

impl ConvexPolytope {
    fn finish(mut self) -> Self {
        // 去重棱方向：|dot| ≥ 0.999 视为同向（符号无关，轴取向由 hint 决定）。
        let mut dirs: Vec<Vec3> = Vec::new();
        let fs = self.face_start.clone();
        for f in 0..fs.len() - 1 {
            let s = fs[f] as usize;
            let e = fs[f + 1] as usize;
            for i in s..e {
                let a = self.verts[i];
                let b = self.verts[s + (i - s + 1) % (e - s)];
                let d = (b - a).normalize();
                if d.length_squared() < 0.5 {
                    continue;
                }
                let dup = dirs.iter().any(|&x| x.dot(d).abs() > 0.999);
                if !dup {
                    dirs.push(d);
                }
            }
        }
        self.edge_dirs = dirs;
        self
    }

    pub fn box_polytope(half: Vec3) -> Self {
        let hx = half.x;
        let hy = half.y;
        let hz = half.z;
        // 顶点索引位约定：bit0=+x, bit1=+y, bit2=+z。
        let c = |i: u32| -> Vec3 {
            Vec3::new(
                if i & 1 != 0 { hx } else { -hx },
                if i & 2 != 0 { hy } else { -hy },
                if i & 4 != 0 { hz } else { -hz },
            )
        };
        let mut p = ConvexPolytope::default();
        let face = |p: &mut ConvexPolytope, normal: Vec3, idx: [u32; 4]| {
            p.face_start.push(p.verts.len() as u32);
            p.face_normal.push(normal);
            for i in idx {
                p.verts.push(c(i));
            }
        };
        face(&mut p, Vec3::X, [1, 3, 7, 5]);
        face(&mut p, -Vec3::X, [0, 4, 6, 2]);
        face(&mut p, Vec3::Y, [2, 6, 7, 3]);
        face(&mut p, -Vec3::Y, [0, 1, 5, 4]);
        face(&mut p, Vec3::Z, [4, 5, 7, 6]);
        face(&mut p, -Vec3::Z, [0, 2, 3, 1]);
        p.face_start.push(p.verts.len() as u32);
        p.finish()
    }

    pub fn cylinder_polytope(radius: f32, half_height: f32, segments: u32) -> Self {
        let n = segments.max(8);
        let mut p = ConvexPolytope::default();
        let mut b: Vec<Vec3> = Vec::with_capacity(n as usize);
        let mut t: Vec<Vec3> = Vec::with_capacity(n as usize);
        for i in 0..n {
            let th = 2.0 * core::f32::consts::PI * (i as f32) / (n as f32);
            let (s, c) = th.sin_cos();
            b.push(Vec3::new(radius * c, -half_height, radius * s));
            t.push(Vec3::new(radius * c, half_height, radius * s));
        }
        let ni = n as i32;
        // 底盖（外法线 -Y）：Newell 推导，角度递增序给出 -Y 法线。
        p.face_start.push(0);
        p.face_normal.push(-Vec3::Y);
        for k in 0..ni {
            p.verts.push(b[k as usize]);
        }
        // 顶盖（+Y）：角度递减序。
        p.face_start.push(p.verts.len() as u32);
        p.face_normal.push(Vec3::Y);
        for k in 0..ni {
            let i = ((-k).rem_euclid(ni)) as usize;
            p.verts.push(t[i]);
        }
        // 侧面：[b_i, t_i, t_{i+1}, b_{i+1}]。
        for i in 0..n {
            let j = (i + 1) % n;
            let center = Vec3::new(
                (b[i as usize].x + b[j as usize].x) * 0.25,
                0.0,
                (b[i as usize].z + b[j as usize].z) * 0.25,
            )
            .normalize();
            p.face_start.push(p.verts.len() as u32);
            p.face_normal.push(center);
            p.verts.push(b[i as usize]);
            p.verts.push(t[i as usize]);
            p.verts.push(t[j as usize]);
            p.verts.push(b[j as usize]);
        }
        p.face_start.push(p.verts.len() as u32);
        p.finish()
    }

    pub fn face_range(&self, f: usize) -> (usize, usize) {
        (self.face_start[f] as usize, self.face_start[f + 1] as usize)
    }

    pub fn face_vert_count(&self, f: usize) -> usize {
        let (s, e) = self.face_range(f);
        e - s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_newell_normals_match_declared() {
        let p = ConvexPolytope::box_polytope(Vec3::new(0.5, 1.0, 2.0));
        for f in 0..p.face_normal.len() {
            let (s, e) = p.face_range(f);
            let n = newell_normal(&p.verts[s..e]);
            assert!(n.dot(p.face_normal[f]) > 0.999, "face {f} winding broken");
        }
    }

    #[test]
    fn cylinder_newell_normals_match_declared() {
        let p = ConvexPolytope::cylinder_polytope(0.5, 1.0, 16);
        for f in 0..p.face_normal.len() {
            let (s, e) = p.face_range(f);
            let n = newell_normal(&p.verts[s..e]);
            assert!(n.dot(p.face_normal[f]) > 0.99, "face {f} winding broken");
        }
    }

    #[test]
    fn box_has_three_unique_edge_dirs() {
        let p = ConvexPolytope::box_polytope(Vec3::splat(0.5));
        assert_eq!(p.edge_dirs.len(), 3);
    }
}
