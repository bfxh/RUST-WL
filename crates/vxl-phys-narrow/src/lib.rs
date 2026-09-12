//! # vxl-phys-narrow
//!
//! 窄相（§2.4）：
//! - 凸-凸：SAT（面法线 + 棱叉积轴）+ 参考面 Sutherland–Hodgman 裁剪 → ≤4 点流形；
//! - 球：解析（球-球 / 球-凸体最近点）；
//! - 高度场：列采样特化（§2.4「列裁剪 + 局部采样」的 M0 版）；
//! - GJK/EPA 通用路径在 M1 接入（`NarrowPhase` trait 不变）。
//!
//! 流形法线约定：`normal` 从 a 指向 b；求解器把 +n 冲量施加给 b、−n 施加给 a。
//! skin = speculative margin（§4.3）：分离距离 ≤ skin 仍生成「预期接触」。

#![forbid(unsafe_code)]

pub mod heightfield;
pub mod polytope;

use std::collections::HashMap;

use heightfield::HeightField;
use polytope::ConvexPolytope;
use vxl_phys_core::{Quat, Shape, Vec3, CYLINDER_SEGMENTS};

/// 单个接触点：世界坐标 + 穿透深度（可为小的负值 = 预期接触，供 warm starting 续接）。
#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub point: Vec3,
    pub depth: f32,
}

/// 接触流形。
#[derive(Clone, Debug)]
pub struct Manifold {
    pub a: u32,
    pub b: u32,
    /// 从 a 指向 b。
    pub normal: Vec3,
    pub points: Vec<ContactPoint>,
}

pub trait NarrowPhase {
    fn collide(
        &mut self,
        bodies: &vxl_phys_core::BodySet,
        pairs: &[(u32, u32)],
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    );
}

/// 参考面来源（决定裁剪参考多面体；参考面本身按法线对齐重选）。
#[derive(Clone, Copy, Debug)]
enum AxisSrc {
    FaceA,
    FaceB,
    Edge,
}

/// 世界空间多面体（局部多面体经刚体变换填充）。
#[derive(Clone, Debug, Default)]
pub struct WorldPoly {
    pub verts: Vec<Vec3>,
    pub face_normal: Vec<Vec3>,
    pub face_start: Vec<u32>,
    pub edge_dirs: Vec<Vec3>,
}

impl WorldPoly {
    fn fill(&mut self, poly: &ConvexPolytope, pos: Vec3, rot: Quat) {
        self.verts.clear();
        self.face_normal.clear();
        self.edge_dirs.clear();
        for v in &poly.verts {
            self.verts.push(rot.rotate_vec3(*v) + pos);
        }
        for n in &poly.face_normal {
            self.face_normal.push(rot.rotate_vec3(*n));
        }
        for d in &poly.edge_dirs {
            self.edge_dirs.push(rot.rotate_vec3(*d));
        }
        self.face_start.clone_from(&poly.face_start);
    }
}

/// 默认窄相（SAT + 解析球 + 高度场采样）。工作缓冲全程复用（无逐步分配）。
pub struct DefaultNarrowPhase {
    skin: f32,
    /// 接触点空间去重最小间距（m）：2×skin，且 ≥ 1 cm。
    min_point_sep: f32,
    polys: Vec<ConvexPolytope>,
    poly_index: HashMap<u64, usize>,
    poly_a: WorldPoly,
    poly_b: WorldPoly,
    axes: Vec<Vec3>,
    clip_in: Vec<Vec3>,
    clip_out: Vec<Vec3>,
    cand: Vec<ContactPoint>,
}

fn poly_key(s: &Shape) -> u64 {
    fn mix(tag: u64, a: f32, b: f32, c: f32) -> u64 {
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
        _ => 0,
    }
}

/// 点到三角形最近点（标准区域分解）。
fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
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
fn closest_point_on_poly(poly: &WorldPoly, p: Vec3) -> (Vec3, f32, bool, Vec3) {
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

impl DefaultNarrowPhase {
    pub fn new(skin: f32) -> Self {
        Self {
            skin,
            min_point_sep: (skin * 2.0).max(0.01),
            polys: Vec::new(),
            poly_index: HashMap::new(),
            poly_a: WorldPoly::default(),
            poly_b: WorldPoly::default(),
            axes: Vec::new(),
            clip_in: Vec::new(),
            clip_out: Vec::new(),
            cand: Vec::new(),
        }
    }

    fn poly_for(&mut self, shape: &Shape) -> Option<usize> {
        let key = poly_key(shape);
        if key == 0 {
            return None;
        }
        if let Some(&idx) = self.poly_index.get(&key) {
            return Some(idx);
        }
        let poly = match *shape {
            Shape::Box { half } => ConvexPolytope::box_polytope(half),
            Shape::Cylinder {
                half_height,
                radius,
            } => ConvexPolytope::cylinder_polytope(radius, half_height, CYLINDER_SEGMENTS),
            _ => return None,
        };
        self.polys.push(poly);
        let idx = self.polys.len() - 1;
        self.poly_index.insert(key, idx);
        Some(idx)
    }

    /// SAT：双侧分离判定，返回 (分离距离 ≤ skin, 轴 a→b, 来源)。
    ///
    /// 对每根轴同时测两个方向（A 在负侧 / B 在负侧），取较大分离度；
    /// 法线统一取向为 a→b。此前单侧公式的取向错误会造成深度失真（能量泵）。
    fn sat(&mut self, _hint: Vec3) -> Option<(f32, Vec3, AxisSrc)> {
        let na = self.poly_a.face_normal.len();
        let nb = self.poly_b.face_normal.len();
        self.axes.clear();
        for i in 0..na {
            self.axes.push(self.poly_a.face_normal[i]);
        }
        for i in 0..nb {
            self.axes.push(self.poly_b.face_normal[i]);
        }
        for &ea in &self.poly_a.edge_dirs {
            for &eb in &self.poly_b.edge_dirs {
                let c = ea.cross(eb);
                let l2 = c.length_squared();
                if l2 > 1e-8 {
                    self.axes.push(c * (1.0 / l2.sqrt()));
                }
            }
        }
        let mut best = f32::MIN;
        let mut best_n = Vec3::ZERO;
        let mut best_src = AxisSrc::Edge;
        for (idx, &n0) in self.axes.iter().enumerate() {
            if n0.length_squared() < 0.5 {
                continue;
            }
            let mut min_a = f32::MAX;
            let mut max_a = f32::MIN;
            for &v in &self.poly_a.verts {
                let d = v.dot(n0);
                if d < min_a {
                    min_a = d;
                }
                if d > max_a {
                    max_a = d;
                }
            }
            let mut min_b = f32::MAX;
            let mut max_b = f32::MIN;
            for &v in &self.poly_b.verts {
                let d = v.dot(n0);
                if d < min_b {
                    min_b = d;
                }
                if d > max_b {
                    max_b = d;
                }
            }
            // 两个分离方向：A 在负侧（n 指向 a→b）或 B 在负侧（翻转）。
            let sep1 = min_b - max_a;
            let sep2 = min_a - max_b;
            let (sep, n) = if sep1 >= sep2 {
                (sep1, n0)
            } else {
                (sep2, -n0)
            };
            if sep > self.skin {
                return None;
            }
            if sep > best {
                best = sep;
                best_n = n;
                best_src = if idx < na {
                    AxisSrc::FaceA
                } else if idx < na + nb {
                    AxisSrc::FaceB
                } else {
                    AxisSrc::Edge
                };
            }
        }
        if best == f32::MIN {
            return None;
        }
        Some((best, best_n, best_src))
    }

    /// 参考面裁剪：生成接触点（深度可为小负值）。
    /// 参考面按「ref → incident 方向」与外法线对齐重选（与轴来源无关，稳健）。
    fn clip(&mut self, normal_ab: Vec3, src: AxisSrc) -> bool {
        let ref_is_a = !matches!(src, AxisSrc::FaceB);
        let dir_to_incident = if ref_is_a { normal_ab } else { -normal_ab };
        let (ref_face_idx, n_ref) = {
            let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
            let mut bi = 0;
            let mut bd = f32::MIN;
            for (i, &n) in p.face_normal.iter().enumerate() {
                let d = n.dot(dir_to_incident);
                if d > bd {
                    bd = d;
                    bi = i;
                }
            }
            (bi, p.face_normal[bi])
        };

        // 入射面：入射多面体中与 n_ref 最逆平行的面。
        let inc_poly = if ref_is_a { &self.poly_b } else { &self.poly_a };
        let mut inc_face = 0;
        let mut inc_dot = f32::MAX;
        for (i, &n) in inc_poly.face_normal.iter().enumerate() {
            let d = n.dot(n_ref);
            if d < inc_dot {
                inc_dot = d;
                inc_face = i;
            }
        }
        let (is_, ie) = {
            let p = inc_poly;
            (
                p.face_start[inc_face] as usize,
                p.face_start[inc_face + 1] as usize,
            )
        };

        self.clip_in.clear();
        for k in is_..ie {
            self.clip_in.push(inc_poly.verts[k]);
        }

        // 参考面世界顶点。
        let (rs, re) = {
            let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
            (
                p.face_start[ref_face_idx] as usize,
                p.face_start[ref_face_idx + 1] as usize,
            )
        };
        // 参考面质心（侧面朝向判定）。
        let mut centroid = Vec3::ZERO;
        {
            let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
            for k in rs..re {
                centroid += p.verts[k];
            }
            centroid *= 1.0 / (re - rs) as f32;
        }

        // 逐侧平面裁剪入射多边形。
        for k in rs..re {
            let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
            let w0 = p.verts[k];
            let w1 = p.verts[if k + 1 == re { rs } else { k + 1 }];
            let e = w1 - w0;
            let mut s = e.cross(n_ref);
            if s.length_squared() < 1e-16 {
                continue;
            }
            s = s.normalize();
            if s.dot(centroid - w0) > 0.0 {
                s = -s;
            }
            // keep: dot(v - w0, s) <= 0
            self.clip_out.clear();
            let m = self.clip_in.len();
            for i in 0..m {
                let va = self.clip_in[i];
                let vb = self.clip_in[(i + 1) % m];
                let da = (va - w0).dot(s);
                let db = (vb - w0).dot(s);
                if da <= 0.0 {
                    self.clip_out.push(va);
                }
                if da * db < 0.0 {
                    let t = da / (da - db);
                    self.clip_out.push(va + (vb - va) * t);
                }
            }
            core::mem::swap(&mut self.clip_in, &mut self.clip_out);
            if self.clip_in.is_empty() {
                return false;
            }
        }

        // 主平面过滤：保留 n_ref 方向距离 ≤ skin 的点（depth = -dist）。
        let p0 = {
            let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
            p.verts[rs]
        };
        self.cand.clear();
        for &v in &self.clip_in {
            let d = (v - p0).dot(n_ref);
            if d <= self.skin {
                self.cand.push(ContactPoint {
                    point: v,
                    depth: -d,
                });
            }
        }
        if self.cand.is_empty() {
            return false;
        }

        // 去重 + 取最深 ≤4 点（确定性排序见 select_contacts）。
        self.select_contacts(self.min_point_sep)
    }

    /// 球-凸体：球心 center、半径 radius；凸体形状 convex（世界变换 cpos/crot）。
    /// 返回 (球→凸体方向法线, 深度, 接触点)。
    fn sphere_convex_ab(
        &mut self,
        center: Vec3,
        radius: f32,
        convex: &Shape,
        cpos: Vec3,
        crot: Quat,
    ) -> Option<(Vec3, f32, Vec3)> {
        let idx = self.poly_for(convex)?;
        self.poly_b.fill(&self.polys[idx], cpos, crot);
        let (closest, d2, inside, in_n) = closest_point_on_poly(&self.poly_b, center);
        if inside {
            // 球心在凸体内部：max_plane_d < 0，表面距 = -max_plane_d。
            let max_d = max_plane_d_of(&self.poly_b, center);
            let depth = radius + max_d;
            if depth <= 0.0 {
                return None;
            }
            let surface = center - in_n * max_d;
            Some((-in_n, depth, surface))
        } else {
            let dist = d2.sqrt();
            if dist >= radius {
                return None;
            }
            let n = if dist > 1e-9 {
                (closest - center) * (1.0 / dist)
            } else {
                Vec3::Y
            };
            Some((n, radius - dist, closest))
        }
    }

    /// 接触点统一选点：按深度降序 + 空间去重（min_sep 内视为同一点）+ 截断 ≤4。
    /// 展平多面体的同一顶点会以浮点噪声级差异重复出现，不去重会造成
    /// 单角多倍冲量 → 永续 rocking（能量泵）。
    fn select_contacts(&mut self, min_sep: f32) -> bool {
        if self.cand.is_empty() {
            return false;
        }
        self.cand.sort_by(|x, y| {
            y.depth
                .total_cmp(&x.depth)
                .then(x.point.x.total_cmp(&y.point.x))
                .then(x.point.y.total_cmp(&y.point.y))
                .then(x.point.z.total_cmp(&y.point.z))
        });
        let min2 = min_sep * min_sep;
        let mut kept: Vec<ContactPoint> = Vec::with_capacity(4);
        for &c in self.cand.iter() {
            if kept.len() >= 4 {
                break;
            }
            let dup = kept
                .iter()
                .any(|k| (k.point - c.point).length_squared() < min2);
            if !dup {
                kept.push(c);
            }
        }
        self.cand = kept;
        !self.cand.is_empty()
    }

    /// 球-高度场：采样 = 投影点双线性 + 所在格 3×3 邻域节点，取最深 ≤4。
    fn sphere_heightfield(&mut self, center: Vec3, radius: f32, hf: &HeightField) -> bool {
        self.cand.clear();
        let r2 = radius * radius;
        let try_point = |cand: &mut Vec<ContactPoint>, px: f32, pz: f32, h: f32| {
            let dx = px - center.x;
            let dz = pz - center.z;
            let d2 = dx * dx + dz * dz;
            if d2 >= r2 {
                return;
            }
            let sy = center.y - (r2 - d2).sqrt();
            // skin 预期接触：允许深度小负值（speculative margin，§4.3），静止时不抖。
            if sy < h + self.skin {
                cand.push(ContactPoint {
                    point: Vec3::new(px, h, pz),
                    depth: h - sy,
                });
            }
        };
        // 1) 投影点（双线性高度；覆盖球心位于格心/格间的一切情形）。
        if let Some((h, _)) = hf.sample(center.x, center.z) {
            try_point(&mut self.cand, center.x, center.z, h);
        }
        // 2) 所在格 + 邻域 3×3 网格节点。
        let ix0 = ((center.x - hf.origin_x) / hf.cell).floor() as i64;
        let iz0 = ((center.z - hf.origin_z) / hf.cell).floor() as i64;
        for dix in -1i64..=1 {
            for diz in -1i64..=1 {
                let ix = ix0 + dix;
                let iz = iz0 + diz;
                if ix < 0 || iz < 0 || ix >= hf.nx as i64 || iz >= hf.nz as i64 {
                    continue;
                }
                let h = hf.height_ix(ix as u32, iz as u32);
                let px = hf.origin_x + ix as f32 * hf.cell;
                let pz = hf.origin_z + iz as f32 * hf.cell;
                try_point(&mut self.cand, px, pz, h);
            }
        }
        if self.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }

    /// 多面体顶点-高度场：逐顶点采样（skin 预期接触），最深 ≤4。
    fn poly_heightfield(
        &mut self,
        poly_idx: usize,
        pos: Vec3,
        rot: Quat,
        hf: &HeightField,
    ) -> bool {
        self.poly_a.fill(&self.polys[poly_idx], pos, rot);
        self.cand.clear();
        for &v in &self.poly_a.verts {
            if let Some((h, _)) = hf.sample(v.x, v.z) {
                let depth = h - v.y;
                if depth > -self.skin {
                    self.cand.push(ContactPoint {
                        point: Vec3::new(v.x, h, v.z),
                        depth,
                    });
                }
            }
        }
        if self.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }
}

/// 内部时「最大平面距」（全部为负；面数小，代价可忽略）。
fn max_plane_d_of(poly: &WorldPoly, p: Vec3) -> f32 {
    let mut max_d = f32::MIN;
    for f in 0..poly.face_normal.len() {
        let s = poly.face_start[f] as usize;
        let d = (p - poly.verts[s]).dot(poly.face_normal[f]);
        if d > max_d {
            max_d = d;
        }
    }
    max_d
}

impl NarrowPhase for DefaultNarrowPhase {
    fn collide(
        &mut self,
        bodies: &vxl_phys_core::BodySet,
        pairs: &[(u32, u32)],
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    ) {
        out.clear();
        for &(a, b) in pairs {
            let (sa, sb) = (&bodies.shape[a as usize], &bodies.shape[b as usize]);
            let pa = bodies.position[a as usize];
            let pb = bodies.position[b as usize];
            let ra = bodies.rotation[a as usize];
            let rb = bodies.rotation[b as usize];

            // 高度场参与的对。
            let hf_a = match sa {
                Shape::HeightField(id) => Some(*id as usize),
                _ => None,
            };
            let hf_b = match sb {
                Shape::HeightField(id) => Some(*id as usize),
                _ => None,
            };
            if hf_a.is_some() && hf_b.is_some() {
                continue;
            }
            if hf_a.is_some() || hf_b.is_some() {
                let (body_shape, bpos, brot, hf_is_a) = if hf_a.is_some() {
                    (sb, pb, rb, true)
                } else {
                    (sa, pa, ra, false)
                };
                let hf = match heightfields.get(hf_a.or(hf_b).unwrap()) {
                    Some(h) => h,
                    None => continue,
                };
                let ok = match *body_shape {
                    Shape::Sphere { radius } => self.sphere_heightfield(bpos, radius, hf),
                    Shape::Box { .. } | Shape::Cylinder { .. } => {
                        let idx = match self.poly_for(body_shape) {
                            Some(i) => i,
                            None => continue,
                        };
                        self.poly_heightfield(idx, bpos, brot, hf)
                    }
                    Shape::HeightField(_) => continue,
                };
                if !ok {
                    continue;
                }
                // 地形法线（取最深接触的采样法线）。
                let deepest = self.cand[0];
                let n_t = hf
                    .sample(deepest.point.x, deepest.point.z)
                    .map(|(_, n)| n)
                    .unwrap_or(Vec3::Y);
                // 流形法线 a→b：a=地形 → +n_t（推向 b）；b=地形 → -n_t。
                let normal = if hf_is_a { n_t } else { -n_t };
                out.push(Manifold {
                    a,
                    b,
                    normal,
                    points: self.cand.clone(),
                });
                continue;
            }

            // 非 heightfield 对。
            match (*sa, *sb) {
                (Shape::Sphere { radius: ra_ }, Shape::Sphere { radius: rb_ }) => {
                    let d = pb - pa;
                    let dist = d.length();
                    let rr = ra_ + rb_;
                    if dist >= rr || dist < 1e-9 {
                        if dist < 1e-9 {
                            out.push(Manifold {
                                a,
                                b,
                                normal: Vec3::Y,
                                points: vec![ContactPoint {
                                    point: pa,
                                    depth: rr,
                                }],
                            });
                        }
                        continue;
                    }
                    let n = d * (1.0 / dist);
                    let point = pa + n * (ra_ - (rr - dist) * 0.5);
                    out.push(Manifold {
                        a,
                        b,
                        normal: n,
                        points: vec![ContactPoint {
                            point,
                            depth: rr - dist,
                        }],
                    });
                }
                (Shape::Sphere { radius }, convex) => {
                    if let Some((n, depth, point)) =
                        self.sphere_convex_ab(pa, radius, &convex, pb, rb)
                    {
                        out.push(Manifold {
                            a,
                            b,
                            normal: n,
                            points: vec![ContactPoint { point, depth }],
                        });
                    }
                }
                (convex, Shape::Sphere { radius }) => {
                    // 球在 b：convex = a。用球-凸路径后翻转法线。
                    if let Some((n_ba, depth, point)) =
                        self.sphere_convex_ab(pb, radius, &convex, pa, ra)
                    {
                        out.push(Manifold {
                            a,
                            b,
                            normal: -n_ba,
                            points: vec![ContactPoint { point, depth }],
                        });
                    }
                }
                (
                    Shape::Box { .. } | Shape::Cylinder { .. },
                    Shape::Box { .. } | Shape::Cylinder { .. },
                ) => {
                    let ia = match self.poly_for(sa) {
                        Some(i) => i,
                        None => continue,
                    };
                    let ib = match self.poly_for(sb) {
                        Some(i) => i,
                        None => continue,
                    };
                    self.poly_a.fill(&self.polys[ia], pa, ra);
                    self.poly_b.fill(&self.polys[ib], pb, rb);
                    if let Some((sep, n, src)) = self.sat(pb - pa) {
                        if sep > self.skin {
                            continue;
                        }
                        if self.clip(n, src) {
                            out.push(Manifold {
                                a,
                                b,
                                normal: n,
                                points: self.cand.clone(),
                            });
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::BodySet;

    fn manifolds_for(b: &BodySet, hf: &[HeightField]) -> Vec<Manifold> {
        let mut np = DefaultNarrowPhase::new(0.01);
        // 全对暴力（测试用）。
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(b, &pairs, hf, &mut out);
        out
    }

    #[test]
    fn sphere_sphere_touch() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
        );
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.9, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let m = manifolds_for(&b, &[]);
        assert_eq!(m.len(), 1);
        assert!((m[0].normal.x - 1.0).abs() < 1e-5);
        assert!((m[0].points[0].depth - 0.1).abs() < 1e-5);
    }

    #[test]
    fn sphere_above_box_normal_points_down() {
        let mut b = BodySet::new();
        // 球心在盒顶上方 0.4 → 穿透深度 = r - 0.4 = 0.1。
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 0.9, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::new(2.0, 0.5, 2.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let m = manifolds_for(&b, &[]);
        assert_eq!(m.len(), 1);
        // a=球 在上，b=盒 → 法线 a→b 朝下。
        assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
        assert!((m[0].points[0].depth - 0.1).abs() < 1e-4);
    }

    #[test]
    fn box_box_resting_manifold() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 0.95, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::new(2.0, 0.5, 2.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let m = manifolds_for(&b, &[]);
        assert_eq!(m.len(), 1);
        assert!(m[0].normal.y < -0.99);
        assert!(!m[0].points.is_empty() && m[0].points.len() <= 4);
        assert!(m[0].points[0].depth > 0.0 && m[0].points[0].depth < 0.06);
    }

    #[test]
    fn box_penetrating_deep_gives_points() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.7, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::new(2.0, 0.5, 2.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let m = manifolds_for(&b, &[]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].points.len(), 4);
    }

    #[test]
    fn cylinder_on_ground() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Cylinder {
                half_height: 0.5,
                radius: 0.3,
            },
            Vec3::new(0.0, 0.9, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::new(2.0, 0.5, 2.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let m = manifolds_for(&b, &[]);
        assert_eq!(m.len(), 1);
        assert!(m[0].normal.y < -0.99);
    }

    #[test]
    fn sphere_on_heightfield() {
        let mut b = BodySet::new();
        let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 0.45, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let m = manifolds_for(&b, &[hf]);
        assert_eq!(m.len(), 1);
        // a=球(0) 在上，b=marker(1) → 法线 a→b = -Y（推向地面）。
        assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
        assert!((m[0].points[0].depth - 0.05).abs() < 0.02);
    }

    #[test]
    fn box_on_heightfield_slope() {
        let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        for iz in 0..11 {
            for ix in 0..11 {
                // 以 x=0 为零点、沿 x 抬升的斜坡（h(0)=0）。
                hf.set_height(ix, iz, (ix as f32 - 5.0) * 0.2);
            }
        }
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(0.0, 0.35, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let m = manifolds_for(&b, &[hf]);
        assert_eq!(m.len(), 1);
        // a=盒(0) 在上，b=marker(1) → 流形法线 a→b 指向地面（-y 分量为主），
        // 且因地面沿 +x 抬升而偏向 +x；求解器给盒子的推力 = -n = 朝上偏 -x。
        assert!(m[0].normal.y < -0.9, "normal {:?}", m[0].normal);
        assert!(m[0].normal.x > 0.1, "normal {:?}", m[0].normal);
    }

    #[test]
    fn separated_boxes_no_manifold() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        assert!(manifolds_for(&b, &[]).is_empty());
    }
}
