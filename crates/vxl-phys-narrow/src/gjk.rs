//! 凸体外壳（Convex Hull）+ GJK / EPA——凸体通用窄相（SPEC §2.4）。
//!
//! 设计：**支撑映射抽象**（`Support`）——任何凸体只需「沿方向的最远点」即可参与
//! GJK（距离/相交）与 EPA（穿透深度/法线）。盒/球/圆柱可复用既有特化路径；**外壳
//! （多边形域）**走本模块。
//!
//! 确定性：迭代次数固定、退化分支显式短路、无随机（SPEC §5）。
//! 规模：顶点数 O(10~10³) 的外壳够用（Voronoi 碎块/凸分解件）；更大建议先凸分解。

#![deny(unsafe_code)]

use vxl_phys_core::{Mat3, Quat, Vec3};

/// 凸体（外壳）：顶点集 + 预计算质心（供 EPA 剥离用）。
#[derive(Clone, Debug, Default)]
pub struct ConvexHull {
    pub points: Vec<Vec3>,
    /// 质心（顶点均值；作为「体内点」用于 GJK/EPA 的剥离方向）。
    pub centroid: Vec3,
}

impl ConvexHull {
    pub fn new(points: Vec<Vec3>) -> Self {
        let n = points.len().max(1) as f32;
        let mut c = Vec3::ZERO;
        for p in &points {
            c += *p;
        }
        Self {
            points,
            centroid: c * (1.0 / n),
        }
    }

    /// 沿 `dir`（单位，非零）最远的顶点。
    #[inline]
    pub fn support_point(&self, dir: Vec3) -> Vec3 {
        let mut best = self.points[0];
        let mut best_d = best.dot(dir);
        for &p in &self.points[1..] {
            let d = p.dot(dir);
            if d > best_d {
                best_d = d;
                best = p;
            }
        }
        best
    }
}

/// 支撑映射（世界系）：凸体沿 `dir` 的最远点 + 一个内部点。
pub trait Support {
    fn support(&self, dir: Vec3) -> Vec3;
    /// 内部点（用于 GJK 的初始方向与 EPA 的剥离）。
    fn interior(&self) -> Vec3;
}

/// 外壳体的支撑映射（世界位姿）。
pub struct HullSupport<'a> {
    pub hull: &'a ConvexHull,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for HullSupport<'_> {
    fn support(&self, dir: Vec3) -> Vec3 {
        let local_dir = self.rot.transpose_mul_vec3(dir);
        self.pos + self.rot.mul_vec3(self.hull.support_point(local_dir))
    }

    fn interior(&self) -> Vec3 {
        self.pos + self.rot.mul_vec3(self.hull.centroid)
    }
}

/// 盒体的支撑映射（与外壳同款接口；供 GJK 通用路径复用）。
pub struct BoxSupport {
    pub half: Vec3,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for BoxSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let ld = self.rot.transpose_mul_vec3(dir);
        let local = Vec3::new(
            if ld.x >= 0.0 {
                self.half.x
            } else {
                -self.half.x
            },
            if ld.y >= 0.0 {
                self.half.y
            } else {
                -self.half.y
            },
            if ld.z >= 0.0 {
                self.half.z
            } else {
                -self.half.z
            },
        );
        self.pos + self.rot.mul_vec3(local)
    }

    fn interior(&self) -> Vec3 {
        self.pos
    }
}

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
fn gjk_inner(a: &dyn Support, b: &dyn Support) -> Result<Gjk, Vec<(Vec3, Vec3, Vec3)>> {
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
fn interp(s: &[(Vec3, Vec3, Vec3)], bary: &[f32; 4]) -> (Vec3, Vec3) {
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
fn closest_simplex(s: &mut Vec<(Vec3, Vec3, Vec3)>) -> (Vec3, [f32; 4]) {
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
fn rebuild(s: &mut Vec<(Vec3, Vec3, Vec3)>, bary: &[f32; 4]) {
    let keep: Vec<(Vec3, Vec3, Vec3)> = s
        .iter()
        .enumerate()
        .filter(|(i, _)| bary[*i] > 0.0)
        .map(|(_, v)| *v)
        .collect();
    *s = keep;
}

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
fn axis_sat(a: &dyn Support, b: &dyn Support) -> Option<(Vec3, f32, Vec3)> {
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
fn face_n(v: &[Vec3], f: &[usize; 3]) -> Vec3 {
    (v[f[1]] - v[f[0]]).cross(v[f[2]] - v[f[0]])
}

/// 点到三角形的重心坐标。
fn bary3(p0: Vec3, p1: Vec3, p2: Vec3, q: Vec3) -> [f32; 3] {
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

/// 球支撑（解析；`dir = 0` 退回球心 ⇒ 确定性）。
pub struct SphereSupport {
    pub radius: f32,
    pub pos: Vec3,
}

impl Support for SphereSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let l = dir.length();
        if l < 1e-12 {
            self.pos
        } else {
            self.pos + dir * (self.radius / l)
        }
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 胶囊体支撑（解析）：局部 **Y 轴线段 `±half_height` ⊕ 半径球**。
///
/// 关键：**不做多边形化**——多边形化的圆弧在滚动/倾斜承载时会"打摆"（每过一个面片
/// 一次冲击），这正是当前 cylinder 走多面体路径的代价。支撑函数式接入 GJK/EPA 即可拿到
/// 光滑外形的精确最远点（`TECH-SURVEY.md` A9）。
pub struct CapsuleSupport {
    pub half_height: f32,
    pub radius: f32,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for CapsuleSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        // 线段端点：局部 Y 方向取号（`dir = 0` 取 +h ⇒ 确定性）。
        let ld = self.rot.transpose_mul_vec3(dir);
        let sy = if ld.y >= 0.0 {
            self.half_height
        } else {
            -self.half_height
        };
        let seg = self.rot.mul_vec3(Vec3::new(0.0, sy, 0.0));
        // 球帽：沿世界方向偏移 radius（`dir = 0` 退回线段端点 ⇒ 确定性）。
        let l = dir.length();
        let n = if l < 1e-12 {
            Vec3::ZERO
        } else {
            dir * (1.0 / l)
        };
        self.pos + seg + n * self.radius
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 圆柱/圆锥（**多面化表示**）的支撑：与 `ConvexPolytope::cylinder_polytope` /
/// `cone_polytope` **同一套 `segments` 边内接多边形**（角度量化到最近顶点 ⇒ 支撑 = 顶点取极值）。
///
/// ⚠️ **与胶囊的对比（别混淆）**：这里是**多面体**支撑（顶点有限）⇒ 喂 EPA 良态；胶囊是
/// **光滑**面 ⇒ EPA 不适定（`EXPERIMENTS.md` R.2/R.3）。**不要**为了"更圆"去掉角度量化，
/// 那会退化成光滑支撑、复现胶囊那类 45° 假法线。
pub struct PrismSupport {
    pub half_height: f32,
    pub radius: f32,
    pub segments: u32,
    /// `true` = 圆锥（顶点在 +Y、底面在 −Y）；`false` = 圆柱（两端圆盘）。
    pub cone: bool,
    pub pos: Vec3,
    pub rot: Mat3,
}

impl Support for PrismSupport {
    fn support(&self, dir: Vec3) -> Vec3 {
        let ld = self.rot.transpose_mul_vec3(dir);
        let n = self.segments.max(8) as f32;
        let two_pi = 2.0 * core::f32::consts::PI;
        // 角度量化到**最近顶点**（等分 ⇒ 最大点积即最近角；`dir = 0` 取 0 角 ⇒ 确定性）。
        let ang = if ld.x == 0.0 && ld.z == 0.0 {
            0.0
        } else {
            ld.z.atan2(ld.x)
        };
        let k = (ang / two_pi * n).round().rem_euclid(n);
        let th = two_pi * k / n;
        let (s, c) = th.sin_cos();
        let (rx, rz) = (self.radius * c, self.radius * s);
        let rim_dot = self.radius * (c * ld.x + s * ld.z);
        if self.cone {
            // 锥：`max(顶点·dir, 底圈最近顶点·dir)`——不能只看 `ld.y` 的符号（近水平方向时底圈更大）。
            let apex_dot = self.half_height * ld.y;
            let base_dot = -self.half_height * ld.y + rim_dot;
            let local = if apex_dot >= base_dot {
                Vec3::new(0.0, self.half_height, 0.0)
            } else {
                Vec3::new(rx, -self.half_height, rz)
            };
            self.pos + self.rot.mul_vec3(local)
        } else {
            // 圆柱：两盘同角度的最近顶点取极值 ⇒ 盘心沿 ±Y 取号即最大。
            let y = if ld.y >= 0.0 {
                self.half_height
            } else {
                -self.half_height
            };
            self.pos + self.rot.mul_vec3(Vec3::new(rx, y, rz))
        }
    }
    fn interior(&self) -> Vec3 {
        self.pos
    }
}

/// 形状 → 支撑体（窄相统一入口）。
pub enum ShapeSupport<'a> {
    Hull(HullSupport<'a>),
    Box(BoxSupport),
    Sphere(SphereSupport),
    Capsule(CapsuleSupport),
    /// 圆柱/圆锥（多面化表示；见 `PrismSupport`）。
    Prism(PrismSupport),
}

impl Support for ShapeSupport<'_> {
    fn support(&self, dir: Vec3) -> Vec3 {
        match self {
            ShapeSupport::Hull(s) => s.support(dir),
            ShapeSupport::Box(s) => s.support(dir),
            ShapeSupport::Sphere(s) => s.support(dir),
            ShapeSupport::Capsule(s) => s.support(dir),
            ShapeSupport::Prism(s) => s.support(dir),
        }
    }
    fn interior(&self) -> Vec3 {
        match self {
            ShapeSupport::Hull(s) => s.interior(),
            ShapeSupport::Box(s) => s.interior(),
            ShapeSupport::Sphere(s) => s.interior(),
            ShapeSupport::Capsule(s) => s.interior(),
            ShapeSupport::Prism(s) => s.interior(),
        }
    }
}

// ---------------------------------------------------------------------------
// 凸体切割（半空间裁剪 + Voronoi 预断裂）——「更一般的凸体/网格切割」的凸体侧。
// ---------------------------------------------------------------------------

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
fn hull_2d_on_plane(pts: &[Vec3], n: Vec3) -> Vec<Vec3> {
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
fn dedup_by_dist(v: &mut Vec<Vec3>) {
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

/// 点是否落在格 `i` 的 Voronoi 半空间约束内（`tol` 吸收边界浮点误差）。
#[cfg(test)]
fn in_voronoi_cell(seeds: &[Vec3], i: usize, q: Vec3, tol: f32) -> bool {
    for (j, s) in seeds.iter().enumerate() {
        if j == i {
            continue;
        }
        let n = *s - seeds[i];
        let mid = (seeds[i] + *s) * 0.5;
        if n.dot(q) > n.dot(mid) + tol {
            return false;
        }
    }
    true
}

/// **Voronoi 预断裂（凸体版）**：种子点最近邻分区 → 逐格 = 原凸壳 ∩ 各平分半空间。
/// 返回每格顶点云（空格 = 种子在体外 ⇒ 空 Vec ⇒ 该格无碎块）。
/// 性质：各格凸、互不重叠、并集 = 原体（浮点容差内）。
pub fn fracture_voronoi_hull(hull: &ConvexHull, seeds: &[Vec3]) -> Vec<Vec<Vec3>> {
    let mut out = Vec::with_capacity(seeds.len());
    for i in 0..seeds.len() {
        let mut region: Vec<Vec3> = hull.points.clone();
        for (j, s) in seeds.iter().enumerate() {
            if i == j {
                continue;
            }
            let n = *s - seeds[i];
            let l2 = n.length_squared();
            if l2 < 1e-12 {
                continue;
            }
            let mid = (seeds[i] + *s) * 0.5;
            region = clip_halfspace(&region, n, n.dot(mid));
            if region.is_empty() {
                break;
            }
        }
        out.push(region);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box_hull() -> ConvexHull {
        let mut pts = Vec::new();
        for &x in &[-0.5f32, 0.5] {
            for &y in &[-0.5f32, 0.5] {
                for &z in &[-0.5f32, 0.5] {
                    pts.push(Vec3::new(x, y, z));
                }
            }
        }
        ConvexHull::new(pts)
    }

    #[test]
    fn gjk_separated_distance_matches_analytic() {
        // 两个盒心相距 1.5（半长 0.5+0.5=1.0 ⇒ 缝 0.5）
        let a = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::ZERO,
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        let b = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::new(1.5, 0.0, 0.0),
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        match gjk(&a, &b) {
            Gjk::Separated { dist, .. } => {
                assert!((dist - 0.5).abs() < 1e-3, "dist={dist}");
            }
            Gjk::Intersecting => panic!("应分离"),
        }
    }

    #[test]
    fn epa_penetration_matches_analytic() {
        // 盒沿 X 重叠 0.2：深度应 ≈ 0.2、法线 ≈ ±X
        let a = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::ZERO,
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        let b = BoxSupport {
            half: Vec3::splat(0.5),
            pos: Vec3::new(0.8, 0.0, 0.0),
            rot: Mat3::from_quat(Quat::IDENTITY),
        };
        assert!(matches!(gjk(&a, &b), Gjk::Intersecting));
        let (n, d, _p) = epa(&a, &b, 32).expect("应相交");
        assert!((d - 0.2).abs() < 0.02, "depth={d}");
        assert!(n.x.abs() > 0.99, "normal={n:?}");
    }

    #[test]
    fn hull_box_intersection_and_separation() {
        let hull = unit_box_hull();
        // 相交（盒心距 0.8 ⇒ 穿透 0.2）
        let hit = hull_box_penetration(
            &hull,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::splat(0.5),
            Vec3::new(0.8, 0.0, 0.0),
            Quat::IDENTITY,
            32,
        );
        let (n, d, _) = hit.expect("应相交");
        assert!(d > 0.05 && d < 0.35, "depth={d}");
        assert!(n.x.abs() > 0.9, "normal={n:?}");
        // 分离
        assert!(hull_box_penetration(
            &hull,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::splat(0.5),
            Vec3::new(1.5, 0.0, 0.0),
            Quat::IDENTITY,
            32
        )
        .is_none());
    }

    #[test]
    fn halfspace_clip_is_exact_half_cube() {
        let h = unit_box_hull();
        let pts = clip_halfspace(&h.points, Vec3::X, 0.0);
        // 裁剪体内：全体 x ≤ 0；且裁切面四角都在（凸包 = 半个立方体）
        assert!(pts.iter().all(|p| p.x <= 1e-6), "{pts:?}");
        for &y in &[-0.5f32, 0.5] {
            for &z in &[-0.5f32, 0.5] {
                assert!(
                    pts.iter()
                        .any(|p| (*p - Vec3::new(0.0, y, z)).length() < 1e-5),
                    "缺裁切面角 ({y},{z})"
                );
            }
        }
    }

    #[test]
    fn voronoi_fracture_cells_tile_convex_body() {
        // 立方体内 8 个种子（偏移去对称 ⇒ 无双胞边界歧义）
        let h = unit_box_hull();
        let mut seeds = Vec::new();
        for (i, &x) in [-0.3f32, 0.3].iter().enumerate() {
            for (j, &y) in [-0.25f32, 0.35].iter().enumerate() {
                for (k, &z) in [-0.35f32, 0.25].iter().enumerate() {
                    let _ = (i, j, k);
                    seeds.push(Vec3::new(x, y, z));
                }
            }
        }
        let cells = fracture_voronoi_hull(&h, &seeds);
        assert_eq!(cells.len(), seeds.len());
        // 抽样：体内每个点恰属 1 格（Voronoi 分区互斥 + 覆盖）
        let mut checked = 0;
        for i in 0..9 {
            for j in 0..9 {
                for k in 0..9 {
                    let q = Vec3::new(
                        -0.45 + 0.9 * i as f32 / 8.0,
                        -0.45 + 0.9 * j as f32 / 8.0,
                        -0.45 + 0.9 * k as f32 / 8.0,
                    );
                    // 跳过贴近平分面的样本（浮点边界归属不判）
                    let mut near = false;
                    for (a, sa) in seeds.iter().enumerate() {
                        for sb in seeds.iter().skip(a + 1) {
                            let dv = *sb - *sa;
                            if dv.length_squared() < 1e-12 {
                                continue;
                            }
                            let n = dv.normalize();
                            let mid = (*sa + *sb) * 0.5;
                            if (n.dot(q) - n.dot(mid)).abs() < 5e-3 {
                                near = true;
                            }
                        }
                    }
                    if near {
                        continue;
                    }
                    let cnt = (0..seeds.len())
                        .filter(|&ci| in_voronoi_cell(&seeds, ci, q, 1e-6))
                        .count();
                    assert_eq!(cnt, 1, "q={q:?} 属 {cnt} 格");
                    checked += 1;
                }
            }
        }
        assert!(checked > 200, "样本太少: {checked}");
    }

    #[test]
    fn support_point_is_extremal() {
        let h = unit_box_hull();
        let p = h.support_point(Vec3::new(1.0, -1.0, 1.0));
        assert_eq!(p, Vec3::new(0.5, -0.5, 0.5));
        // 旋转 90°（绕 Y）后，+X 支撑点应来自原 +Z 方向
        let rot = Mat3::from_quat(Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2));
        let hs = HullSupport {
            hull: &h,
            pos: Vec3::ZERO,
            rot,
        };
        let p = hs.support(Vec3::X);
        // 绕 Y 转 90°：世界 +X 支撑应由原 +Z 面给出（x=0.5）；并列点不唯一 ⇒ 只验极值性
        assert!((p.x - 0.5).abs() < 1e-5, "{p:?}");
        for q in &h.points {
            assert!(rot.mul_vec3(*q).x <= p.x + 1e-5, "非极值: {q:?}");
        }
    }
}
