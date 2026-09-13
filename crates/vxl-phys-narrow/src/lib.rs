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
use vxl_phys_core::{JobSystem, Quat, Shape, Vec3, CYLINDER_SEGMENTS};

/// 单个接触点：世界坐标 + 穿透深度（可为小的负值 = 预期接触，供 warm starting 续接）。
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactPoint {
    pub point: Vec3,
    pub depth: f32,
    /// 接触特征 ID（跨帧稳定的点命名，M1 接触 ID）：warm starting 以 ID
    /// 精确匹配替代近邻匹配——面-面接触的裁剪点集逐帧微动/翻面时，
    /// 冲量缓存不丢（Rapier `contact_recycling` / Box2D `b2ContactFeature`
    /// 同族）。位域：bit31 = 入射侧（1 = B），bit30 = 裁剪交点（棱×侧平面
    /// 哈希）、否则为入射面顶点序号。0 = 无特征（回退近邻匹配）。
    pub feature: u32,
}

/// 特征 ID 位域常量（见 `ContactPoint::feature`）。
pub(crate) const FEAT_SIDE_B: u32 = 1 << 31;
pub(crate) const FEAT_CLIPPED: u32 = 1 << 30;

/// 裁剪交点特征 = 入射棱 (fa→fb) × 参考侧平面 k 的确定性混合（同三元组
/// 重现同 ID）。碰撞概率对 4 点流形可忽略；miss 时求解器回退近邻匹配。
#[inline]
pub(crate) fn feat_intersect(fa: u32, fb: u32, k: usize) -> u32 {
    FEAT_CLIPPED
        | (fa.wrapping_mul(73_856_093)
            ^ fb.wrapping_mul(19_349_663)
            ^ (k as u32).wrapping_mul(83_492_791))
            & !(FEAT_SIDE_B | FEAT_CLIPPED)
}

/// 内联接触点集（≤4 点；T3：免每次流形一次堆分配——8B 场景 ~22 万接触/帧，
/// 旧 `Vec` 构造 = 数十万次分配/帧）。Deref 到 `&[ContactPoint]`：读取面
/// （len/index/iter）零改动。构造：`[..].into()` 或 `ContactPoints::empty()`。
#[derive(Clone, Debug, Default)]
pub struct ContactPoints {
    buf: [ContactPoint; 4],
    len: u8,
}

impl ContactPoints {
    pub fn empty() -> Self {
        Self::default()
    }

    /// 从切片复制（len > 4 时取前 4——本引擎流形按 ≤4 点构造，防御性截断）。
    pub fn from_slice(s: &[ContactPoint]) -> Self {
        let mut out = Self::default();
        let n = s.len().min(4);
        out.buf[..n].copy_from_slice(&s[..n]);
        out.len = n as u8;
        out
    }
}

impl core::ops::Deref for ContactPoints {
    type Target = [ContactPoint];
    fn deref(&self) -> &[ContactPoint] {
        &self.buf[..self.len as usize]
    }
}

impl<'a> IntoIterator for &'a ContactPoints {
    type Item = &'a ContactPoint;
    type IntoIter = core::slice::Iter<'a, ContactPoint>;
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<const N: usize> From<[ContactPoint; N]> for ContactPoints {
    fn from(a: [ContactPoint; N]) -> Self {
        Self::from_slice(&a)
    }
}

/// 接触流形。
#[derive(Clone, Debug)]
pub struct Manifold {
    pub a: u32,
    pub b: u32,
    /// 从 a 指向 b。
    pub normal: Vec3,
    pub points: ContactPoints,
}

pub trait NarrowPhase {
    fn collide(
        &mut self,
        bodies: &vxl_phys_core::BodySet,
        pairs: &[(u32, u32)],
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
        jobs: &dyn JobSystem,
    );
}

/// 参考面来源（决定裁剪参考多面体；参考面本身按法线对齐重选）。
#[derive(Clone, Copy, Debug, PartialEq)]
enum AxisSrc {
    FaceA,
    FaceB,
    Edge,
}

/// 盒面表：顺序与 `box_polytope` 逐条一致（面序 + 每面 4 顶点的环绕序），
/// 由 `polytope::tests::box_axis_order_and_orientation_is_pinned` 钉死。
///
/// 每项 = (法线所在轴与正负, 4 个顶点的 (sx, sy, sz) 符号)，顶点符号位约定
/// 与 `box_polytope` 相同（bit0=+x, bit1=+y, bit2=+z）；全局顶点号 =
/// `面序 × 4 + 面内序号`（即多面体 `verts` 的下标 ⇒ 特征号逐位一致）。
type BoxSign = (i8, i8, i8);
const BOX_FACES: [((i8, i8, i8), [BoxSign; 4]); 6] = [
    // +X: [1, 3, 7, 5]
    ((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
    // -X: [0, 4, 6, 2]
    (
        (-1, 0, 0),
        [(-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1)],
    ),
    // +Y: [2, 6, 7, 3]
    ((0, 1, 0), [(-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1)]),
    // -Y: [0, 1, 5, 4]
    (
        (0, -1, 0),
        [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)],
    ),
    // +Z: [4, 5, 7, 6]
    ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
    // -Z: [0, 2, 3, 1]
    (
        (0, 0, -1),
        [(-1, -1, -1), (-1, 1, -1), (1, 1, -1), (1, -1, -1)],
    ),
];

/// 盒的体轴：`[R·X, R·Y, R·Z]`（与多面体填充逐位同源——同一 `rotate_vec3`）。
#[inline]
fn box_axes(rot: Quat) -> [Vec3; 3] {
    [
        rot.rotate_vec3(Vec3::X),
        rot.rotate_vec3(Vec3::Y),
        rot.rotate_vec3(Vec3::Z),
    ]
}

/// 面法线：轴分量带符号（±v 逐位精确，`rotate(-v) == -rotate(v)` 在
/// 双叉积形式下成立）。
#[inline]
fn face_normal_of(axes: &[Vec3; 3], n: (i8, i8, i8)) -> Vec3 {
    let s = |a: i8, v: Vec3| if a > 0 { v } else { -v };
    let mut out = Vec3::ZERO;
    if n.0 != 0 {
        out = s(n.0, axes[0]);
    }
    if n.1 != 0 {
        out = s(n.1, axes[1]);
    }
    if n.2 != 0 {
        out = s(n.2, axes[2]);
    }
    out
}

/// 面上第 i 个顶点的世界坐标：pos + Σ 符号·half·轴（沿三轴展开，含零项）。
#[inline]
fn face_vertex(pos: Vec3, axes: &[Vec3; 3], half: Vec3, sg: BoxSign) -> Vec3 {
    let t0 = axes[0] * (half.x * sg.0 as f32);
    let t1 = axes[1] * (half.y * sg.1 as f32);
    let t2 = axes[2] * (half.z * sg.2 as f32);
    pos + (t0 + t1) + t2
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
#[derive(Clone)]
pub struct DefaultNarrowPhase {
    skin: f32,
    /// 接触点空间去重最小间距（m）：2×skin，且 ≥ 1 cm。
    min_point_sep: f32,
    polys: Vec<ConvexPolytope>,
    poly_index: HashMap<u64, usize>,
    poly_a: WorldPoly,
    poly_b: WorldPoly,
    /// 世界多面体填充缓存（T3 快路径）：pair 按 (a,b) 排序 ⇒ 同一体的对连续，
    /// 单条缓存即可让每体每帧只填一次（大场景实测同一体每帧被填 ~50 次）。
    /// 纯函数（poly, pos, rot）→ 命中即跳过 33 次旋转；逐位一致。
    cached_a: (u32, u64),
    cached_b: (u32, u64),
    axes: Vec<Vec3>,
    clip_in: Vec<(Vec3, u32)>,
    clip_out: Vec<(Vec3, u32)>,
    cand: Vec<ContactPoint>,
    /// select_contacts 去重取点 scratch（T3：免每接触一次堆分配）。
    kept_buf: Vec<ContactPoint>,
    /// 盒对 SAT 快路径参数（half, 世界中心）：两 poly 均为盒时用 extents
    /// 投影公式（O(1)/体/轴）替代逐顶点 min/max（24 点/体/轴）。由盒-盒
    /// 分支设置；圆柱等其它形状为 None（走通用逐顶点路径）。
    box_a: Option<(Vec3, Vec3)>,
    box_b: Option<(Vec3, Vec3)>,
    /// 盒对专属体轴（T3 专用路径）：由 `(rot, half)` 直生（每体 3 次旋转），
    /// 与预筛共用；置位时 SAT/clip 走免填充路径，其余形状为 None。
    box_axes_a: Option<[Vec3; 3]>,
    box_axes_b: Option<[Vec3; 3]>,
    /// 裁剪顶点 scratch（参考面顶点）：通用与专用路径共用同一裁剪实现，
    /// 避免两份易漂移的裁剪代码。
    ref_v: Vec<Vec3>,
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
            cached_a: (u32::MAX, u64::MAX),
            cached_b: (u32::MAX, u64::MAX),
            axes: Vec::new(),
            clip_in: Vec::new(),
            clip_out: Vec::new(),
            cand: Vec::new(),
            kept_buf: Vec::new(),
            box_a: None,
            box_b: None,
            box_axes_a: None,
            box_axes_b: None,
            ref_v: Vec::new(),
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
        // 轴表：盒对专用路径由体轴直生（面 6 轴 + 棱叉积 9 轴，顺序与通用
        // 路径逐条对应——面序 [±X,±Y,±Z]、棱序 [+Y,+Z,+X]×[+Y,+Z,+X]，后者由
        // polytope 测试钉死）；其余形状读多面体。
        let box_axes = match (self.box_axes_a, self.box_axes_b) {
            (Some(aa), Some(ab)) => Some((aa, ab)),
            _ => None,
        };
        if let Some((aa, ab)) = box_axes {
            for f in &BOX_FACES {
                self.axes.push(face_normal_of(&aa, f.0));
            }
            for f in &BOX_FACES {
                self.axes.push(face_normal_of(&ab, f.0));
            }
            let ea = [aa[1], aa[2], aa[0]];
            let eb = [ab[1], ab[2], ab[0]];
            for x in ea {
                for y in eb {
                    let c = x.cross(y);
                    let l2 = c.length_squared();
                    if l2 > 1e-8 {
                        self.axes.push(c * (1.0 / l2.sqrt()));
                    }
                }
            }
        } else {
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
        }
        debug_assert!(box_axes.is_some() || (na == 6 && nb == 6));
        // 面轴计数：专用路径恒 6+6（体轴直生，未填多面体），通用路径读多面体。
        let (n_face_a, n_face_b) = match box_axes {
            Some(_) => (6usize, 6usize),
            None => (na, nb),
        };
        let mut best = f32::MIN;
        let mut best_n = Vec3::ZERO;
        let mut best_src = AxisSrc::Edge;
        for (idx, &n0) in self.axes.iter().enumerate() {
            if n0.length_squared() < 0.5 {
                continue;
            }
            let mut min_a = f32::MAX;
            let mut max_a = f32::MIN;
            let mut min_b = f32::MAX;
            let mut max_b = f32::MIN;
            // T3 快路径：盒对用 extents 投影公式（面法线 [0]/[2]/[4] 即体轴，
            // 与逐顶点 min/max 数学等价），轴序/取向/来源分类与通用路径同。
            if let (Some((ha, pa)), Some((hb, pb)), Some((aa, ab))) =
                (self.box_a, self.box_b, box_axes)
            {
                let ra = ha.x * aa[0].dot(n0).abs()
                    + ha.y * aa[1].dot(n0).abs()
                    + ha.z * aa[2].dot(n0).abs();
                let rb = hb.x * ab[0].dot(n0).abs()
                    + hb.y * ab[1].dot(n0).abs()
                    + hb.z * ab[2].dot(n0).abs();
                let ca = pa.dot(n0);
                let cb = pb.dot(n0);
                min_a = ca - ra;
                max_a = ca + ra;
                min_b = cb - rb;
                max_b = cb + rb;
            } else if let (Some((ha, pa)), Some((hb, pb))) = (self.box_a, self.box_b) {
                let ax = &self.poly_a.face_normal;
                let bx = &self.poly_b.face_normal;
                let ra = ha.x * ax[0].dot(n0).abs()
                    + ha.y * ax[2].dot(n0).abs()
                    + ha.z * ax[4].dot(n0).abs();
                let rb = hb.x * bx[0].dot(n0).abs()
                    + hb.y * bx[2].dot(n0).abs()
                    + hb.z * bx[4].dot(n0).abs();
                let ca = pa.dot(n0);
                let cb = pb.dot(n0);
                min_a = ca - ra;
                max_a = ca + ra;
                min_b = cb - rb;
                max_b = cb + rb;
            } else {
                for &v in &self.poly_a.verts {
                    let d = v.dot(n0);
                    if d < min_a {
                        min_a = d;
                    }
                    if d > max_a {
                        max_a = d;
                    }
                }
                for &v in &self.poly_b.verts {
                    let d = v.dot(n0);
                    if d < min_b {
                        min_b = d;
                    }
                    if d > max_b {
                        max_b = d;
                    }
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
                best_src = if idx < n_face_a {
                    AxisSrc::FaceA
                } else if idx < n_face_a + n_face_b {
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
        // 面轴来源：盒对专用路径（体轴直生，免多面体填充）或通用多面体。
        // 两者给出的面法线序列逐位相同（BOX_FACES 顺序 = 多面体面序，
        // ±v 精确），故参考/入射面选择与平局分解不变。
        let box_axes = match (self.box_axes_a, self.box_axes_b) {
            (Some(aa), Some(ab)) => Some((aa, ab)),
            _ => None,
        };
        let (ref_face_idx, n_ref) = match box_axes {
            Some((aa, ab)) => {
                let axes = if ref_is_a { aa } else { ab };
                let mut bi = 0;
                let mut bd = f32::MIN;
                for (i, f) in BOX_FACES.iter().enumerate() {
                    let d = face_normal_of(&axes, f.0).dot(dir_to_incident);
                    if d > bd {
                        bd = d;
                        bi = i;
                    }
                }
                (bi, face_normal_of(&axes, BOX_FACES[bi].0))
            }
            None => {
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
            }
        };

        // 参考面世界顶点入 scratch（含全局基准号，供侧平面特征号复用）。
        self.ref_v.clear();
        let ref_base: u32 = match box_axes {
            Some((aa, ab)) => {
                let (axes, half, pos) = if ref_is_a {
                    let (h, p) = self.box_a.expect("盒对专用路径必置 box_a");
                    (aa, h, p)
                } else {
                    let (h, p) = self.box_b.expect("盒对专用路径必置 box_b");
                    (ab, h, p)
                };
                for &sg in BOX_FACES[ref_face_idx].1.iter() {
                    self.ref_v.push(face_vertex(pos, &axes, half, sg));
                }
                ref_face_idx as u32 * 4
            }
            None => {
                let (s, e) = if ref_is_a {
                    let p = &self.poly_a;
                    (
                        p.face_start[ref_face_idx] as usize,
                        p.face_start[ref_face_idx + 1] as usize,
                    )
                } else {
                    let p = &self.poly_b;
                    (
                        p.face_start[ref_face_idx] as usize,
                        p.face_start[ref_face_idx + 1] as usize,
                    )
                };
                if ref_is_a {
                    self.ref_v.extend_from_slice(&self.poly_a.verts[s..e]);
                } else {
                    self.ref_v.extend_from_slice(&self.poly_b.verts[s..e]);
                }
                s as u32
            }
        };

        // 入射面：与 n_ref 最逆平行的面；顶点直接入裁剪多边形（带特征号）。
        self.clip_in.clear();
        let side_bit = if ref_is_a { FEAT_SIDE_B } else { 0 };
        match box_axes {
            Some((aa, ab)) => {
                let (axes, half, pos) = if ref_is_a {
                    let (h, p) = self.box_b.expect("盒对专用路径必置 box_b");
                    (ab, h, p)
                } else {
                    let (h, p) = self.box_a.expect("盒对专用路径必置 box_a");
                    (aa, h, p)
                };
                let mut inc_face = 0;
                let mut inc_dot = f32::MAX;
                for (i, f) in BOX_FACES.iter().enumerate() {
                    let d = face_normal_of(&axes, f.0).dot(n_ref);
                    if d < inc_dot {
                        inc_dot = d;
                        inc_face = i;
                    }
                }
                let base = inc_face as u32 * 4;
                for (i, &sg) in BOX_FACES[inc_face].1.iter().enumerate() {
                    let v = face_vertex(pos, &axes, half, sg);
                    self.clip_in.push((v, side_bit | (base + i as u32)));
                }
            }
            None => {
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
                for k in is_..ie {
                    let v = inc_poly.verts[k];
                    self.clip_in.push((v, side_bit | (k as u32)));
                }
            }
        }

        // 参考面质心（侧面朝向判定）。
        let mut centroid = Vec3::ZERO;
        for &v in &self.ref_v {
            centroid += v;
        }
        centroid *= 1.0 / self.ref_v.len() as f32;

        // 逐侧平面裁剪入射多边形。侧平面向量**不归一化**：保留判定
        // （`da <= 0`）、穿越判定（`da*db < 0`）与插值参数
        // （`t = da/(da−db)`）全部与平面向量尺度无关，故省掉每面一次
        // sqrt + 除法（旧实现每 clip 4 次）。
        let nref_v = self.ref_v.len();
        for k in 0..nref_v {
            let w0 = self.ref_v[k];
            let w1 = self.ref_v[if k + 1 == nref_v { 0 } else { k + 1 }];
            let e = w1 - w0;
            let mut s = e.cross(n_ref);
            if s.length_squared() < 1e-16 {
                continue;
            }
            if s.dot(centroid - w0) > 0.0 {
                s = -s;
            }
            // keep: dot(v - w0, s) <= 0
            self.clip_out.clear();
            let m = self.clip_in.len();
            for i in 0..m {
                let (va, fa) = self.clip_in[i];
                let (vb, fb) = self.clip_in[(i + 1) % m];
                let da = (va - w0).dot(s);
                let db = (vb - w0).dot(s);
                if da <= 0.0 {
                    self.clip_out.push((va, fa));
                }
                if da * db < 0.0 {
                    let t = da / (da - db);
                    self.clip_out.push((
                        va + (vb - va) * t,
                        feat_intersect(fa, fb, ref_base as usize + k),
                    ));
                }
            }
            core::mem::swap(&mut self.clip_in, &mut self.clip_out);
            if self.clip_in.is_empty() {
                return false;
            }
        }

        // 主平面过滤：保留 n_ref 方向距离 ≤ skin 的点（depth = -dist）。
        let p0 = self.ref_v[0];
        self.cand.clear();
        for &(v, feat) in &self.clip_in {
            let d = (v - p0).dot(n_ref);
            if d <= self.skin {
                self.cand.push(ContactPoint {
                    point: v,
                    depth: -d,
                    feature: feat,
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
        // 复用 scratch（T3：旧实现每调用一次 Vec::with_capacity(4) —— 8B 场景
        // ~22 万次/帧的堆分配）。语义逐位不变：按深度序取 ≤4 个非重复点。
        let mut kept = std::mem::take(&mut self.kept_buf);
        kept.clear();
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
        self.cand.clear();
        self.cand.extend_from_slice(&kept);
        self.kept_buf = kept;
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
                    feature: 0,
                });
            }
        };
        // 1) 投影点（双线性高度；覆盖球心位于格心/格间的一切情形）。
        if let Some((h, _)) = hf.sample(center.x, center.z) {
            try_point(&mut self.cand, center.x, center.z, h);
        }
        // 2) 所在格 + 邻域 3×3 网格节点。
        let ix0 = ((center.x - hf.origin_x) / hf.spacing).floor() as i64;
        let iz0 = ((center.z - hf.origin_z) / hf.spacing).floor() as i64;
        for dix in -1i64..=1 {
            for diz in -1i64..=1 {
                let ix = ix0 + dix;
                let iz = iz0 + diz;
                if ix < 0 || iz < 0 || ix >= hf.nx as i64 || iz >= hf.nz as i64 {
                    continue;
                }
                let h = hf.height_ix(ix as u32, iz as u32);
                let px = hf.origin_x + ix as f32 * hf.spacing;
                let pz = hf.origin_z + iz as f32 * hf.spacing;
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
        for (idx, &v) in self.poly_a.verts.iter().enumerate() {
            if let Some((h, _)) = hf.sample(v.x, v.z) {
                let depth = h - v.y;
                if depth > -self.skin {
                    self.cand.push(ContactPoint {
                        point: Vec3::new(v.x, h, v.z),
                        depth,
                        // 特征 = 盒顶点序号（跨帧稳定；盒侧不置侧位）。
                        feature: idx as u32,
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
        jobs: &dyn JobSystem,
    ) {
        out.clear();
        // 世界多面体填充缓存跨帧失效（体在帧间移动；键只含体号+形状）。
        self.cached_a = (u32::MAX, u64::MAX);
        self.cached_b = (u32::MAX, u64::MAX);
        let threads = jobs.threads();
        // 小规模串行（线程启动开销 > 收益）；并行 = 每块独立 clone（含自有
        // scratch），结果按块序拼接 = pair 序（§5 确定性契约）。
        if threads <= 1 || pairs.len() < 2048 {
            for &(a, b) in pairs {
                self.process_pair(a, b, bodies, heightfields, out);
            }
            return;
        }
        let this = &*self;
        let n_chunks = threads.min(pairs.len().div_ceil(2048));
        let chunk = pairs.len().div_ceil(n_chunks);
        let mut outs: Vec<Vec<Manifold>> = vec![Vec::new(); n_chunks];
        vxl_phys_core::schedule::for_each_chunk_mut(
            &mut outs,
            threads,
            2,
            |start_slot, _len, slots| {
                // slots[k] = outs[start_slot + k]（块内逐槽对应各自的 pair 区间）。
                for (k, co) in slots.iter_mut().enumerate() {
                    let oi = start_slot + k;
                    let range = oi * chunk..((oi + 1) * chunk).min(pairs.len());
                    let mut np = this.clone();
                    for &(a, b) in &pairs[range] {
                        np.process_pair(a, b, bodies, heightfields, co);
                    }
                }
            },
        );
        for mut co in outs {
            out.append(&mut co);
        }
    }
}

impl DefaultNarrowPhase {
    fn process_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    ) {
        let (sa, sb) = (&bodies.shape[a as usize], &bodies.shape[b as usize]);
        let pa = bodies.position[a as usize];
        let pb = bodies.position[b as usize];
        let ra = bodies.rot(a as usize);
        let rb = bodies.rot(b as usize);
        // 盒对专用路径开关：每对先复位（非盒对 / 圆柱对一律走通用路径）。
        self.box_axes_a = None;
        self.box_axes_b = None;

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
            return;
        }
        if hf_a.is_some() || hf_b.is_some() {
            let (body_shape, bpos, brot, hf_is_a) = if hf_a.is_some() {
                (sb, pb, rb, true)
            } else {
                (sa, pa, ra, false)
            };
            let hf = match heightfields.get(hf_a.or(hf_b).unwrap()) {
                Some(h) => h,
                None => return,
            };
            let ok = match *body_shape {
                Shape::Sphere { radius } => self.sphere_heightfield(bpos, radius, hf),
                Shape::Box { .. } | Shape::Cylinder { .. } => {
                    let idx = match self.poly_for(body_shape) {
                        Some(i) => i,
                        None => return,
                    };
                    self.poly_heightfield(idx, bpos, brot, hf)
                }
                Shape::HeightField(_) => return,
            };
            if !ok {
                return;
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
                points: ContactPoints::from_slice(&self.cand),
            });
            return;
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
                            points: [ContactPoint {
                                point: pa,
                                depth: rr,
                                feature: 0,
                            }]
                            .into(),
                        });
                    }
                    return;
                }
                let n = d * (1.0 / dist);
                let point = pa + n * (ra_ - (rr - dist) * 0.5);
                out.push(Manifold {
                    a,
                    b,
                    normal: n,
                    points: [ContactPoint {
                        point,
                        depth: rr - dist,
                        feature: 0,
                    }]
                    .into(),
                });
            }
            (Shape::Sphere { radius }, convex) => {
                if let Some((n, depth, point)) = self.sphere_convex_ab(pa, radius, &convex, pb, rb)
                {
                    out.push(Manifold {
                        a,
                        b,
                        normal: n,
                        points: [ContactPoint {
                            point,
                            depth,
                            feature: 0,
                        }]
                        .into(),
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
                        points: [ContactPoint {
                            point,
                            depth,
                            feature: 0,
                        }]
                        .into(),
                    });
                }
            }
            (Shape::Box { half: ha }, Shape::Box { half: hb }) => {
                // ===== T3 盒对专用路径 =====
                // 轴由 (rot, half) 直生（每体 3 次旋转），供 SAT/clip 直接消费。
                // 【实验】不再调用分离预筛：其 15 轴是 SAT 21 轴的子集（面轴 ± 同解），
                // 预筛不拒的对必然要再做一遍同样的 15 轴测试 ⇒ 对真接触对是纯重复。
                self.box_a = Some((ha, pa));
                self.box_b = Some((hb, pb));
                self.box_axes_a = Some(box_axes(ra));
                self.box_axes_b = Some(box_axes(rb));
                if let Some((sep, n, src)) = self.sat(pb - pa) {
                    if sep > self.skin {
                        return;
                    }
                    if self.clip(n, src) {
                        out.push(Manifold {
                            a,
                            b,
                            normal: n,
                            points: ContactPoints::from_slice(&self.cand),
                        });
                    }
                }
            }
            (
                Shape::Box { .. } | Shape::Cylinder { .. },
                Shape::Box { .. } | Shape::Cylinder { .. },
            ) => {
                // 圆柱参与的对：走通用路径（多面体填充 + 逐顶点/通用 SAT）。
                let ia = match self.poly_for(sa) {
                    Some(i) => i,
                    None => return,
                };
                let ib = match self.poly_for(sb) {
                    Some(i) => i,
                    None => return,
                };
                // 世界多面体填充缓存（T3）：键 = (体号, 多面体序号)；pair 按
                // (a,b) 排序 ⇒ 同一体连续命中，每体每帧只填一次（纯函数）。
                if self.cached_a != (a, ia as u64) {
                    self.poly_a.fill(&self.polys[ia], pa, ra);
                    self.cached_a = (a, ia as u64);
                }
                if self.cached_b != (b, ib as u64) {
                    self.poly_b.fill(&self.polys[ib], pb, rb);
                    self.cached_b = (b, ib as u64);
                }
                // 盒对 SAT 快路径参数（圆柱 → None，走通用逐顶点路径）。
                self.box_a = match sa {
                    Shape::Box { half } => Some((*half, pa)),
                    _ => None,
                };
                self.box_b = match sb {
                    Shape::Box { half } => Some((*half, pb)),
                    _ => None,
                };
                if let Some((sep, n, src)) = self.sat(pb - pa) {
                    if sep > self.skin {
                        return;
                    }
                    if self.clip(n, src) {
                        out.push(Manifold {
                            a,
                            b,
                            normal: n,
                            points: ContactPoints::from_slice(&self.cand),
                        });
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{BodySet, SerialJobSystem};

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
        np.collide(b, &pairs, hf, &mut out, &SerialJobSystem);
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

    /// T3 盒对 SAT extents 快路径 vs 通用逐顶点路径：300 对随机盒
    /// （重叠/分离/极端姿态混合）同帧对拍，断言 None/Some 类别一致、
    /// sep 差 ≤1e-4、法线对齐 >0.999、来源分类一致。守门对象：`sat()`
    /// 内盒对分支（extents 公式）与通用顶点 min/max 的等价性。
    #[test]
    fn box_sat_fast_matches_vertex_reference() {
        let mut np = DefaultNarrowPhase::new(0.01);
        let ha = Vec3::new(0.5, 0.3, 0.7);
        let hb = Vec3::new(0.4, 0.6, 0.2);
        let ia = np.poly_for(&Shape::Box { half: ha }).unwrap();
        let ib = np.poly_for(&Shape::Box { half: hb }).unwrap();
        let mut rng: u32 = 0x1234_5678;
        let mut next = move || {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (rng >> 8) as f32 / 16_777_216.0
        };
        let mut max_delta = 0.0f32;
        for k in 0..300 {
            let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
            let pb = pa + Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0);
            let ax = Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize();
            let bx = Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize();
            let qa = Quat::from_axis_angle(ax, next() * core::f32::consts::TAU);
            let qb = Quat::from_axis_angle(bx, next() * core::f32::consts::TAU);
            np.poly_a.fill(&np.polys[ia], pa, qa);
            np.poly_b.fill(&np.polys[ib], pb, qb);
            let d = pb - pa;
            // 快路径（盒对 extents 公式）。
            np.box_a = Some((ha, pa));
            np.box_b = Some((hb, pb));
            let fast = np.sat(d);
            // 通用路径（逐顶点 min/max；轴序、取向、平局规则完全相同，
            // 唯一差异即投影计算方式）。
            np.box_a = None;
            np.box_b = None;
            let slow = np.sat(d);
            match (fast, slow) {
                (None, None) => {}
                (Some((s1, n1, r1)), Some((s2, n2, r2))) => {
                    let d = (s1 - s2).abs();
                    if d > max_delta {
                        max_delta = d;
                    }
                    assert!(d <= 1e-6, "k{k}: sep {s1} vs {s2}（Δ {d}，非 ULP 级）");
                    assert!(n1.dot(n2).abs() > 0.999, "k{k}: normal {n1:?} vs {n2:?}");
                    assert_eq!(r1, r2, "k{k}: src {r1:?} vs {r2:?}");
                }
                (f, s) => panic!(
                    "k{k}: 类别不一致 fast={:?} slow={:?}",
                    f.is_some(),
                    s.is_some()
                ),
            }
        }
        eprintln!("快/通用路径 sep 最大 Δ = {max_delta:.3e}（f32 ULP 级；>0 表示快路径确被拉到）");
        assert!(max_delta > 0.0, "两路径逐位相同 ⇒ 快路径未被真正测到");
    }

    /// T3 盒对专用路径 vs 通用路径**全链对拍**：同一姿态下，唯一变量是
    /// 「体轴直生（专用）还是多面体填充（通用）」，断言 SAT 的 sep/法线/来源
    /// 三者一致，且 `clip` 的接触点集合逐点对应（点数相同、特征号逐位相同、
    /// 位置差 ≤1e-5——两条路径的顶点算术序不同，只保证到 ULP 级）。
    /// 守门对象：专用路径的轴序 / 面表环绕序 / 特征号编号。
    #[test]
    fn box_dedicated_matches_generic_full_chain() {
        let mut np = DefaultNarrowPhase::new(0.02);
        let ha = Vec3::new(0.5, 0.3, 0.7);
        let hb = Vec3::new(0.4, 0.6, 0.2);
        let ia = np.poly_for(&Shape::Box { half: ha }).unwrap();
        let ib = np.poly_for(&Shape::Box { half: hb }).unwrap();
        let mut rng: u32 = 0x51ED_2701;
        let mut next = move || {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (rng >> 8) as f32 / 16_777_216.0
        };
        let mut contacts_seen = 0usize;
        for k in 0..300 {
            let qa = Quat::from_axis_angle(
                Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize(),
                next() * core::f32::consts::TAU,
            );
            let qb = Quat::from_axis_angle(
                Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize(),
                next() * core::f32::consts::TAU,
            );
            let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
            // 近半样本深度重叠（保证 clip 真的跑出接触点），其余随机。
            let d = if k % 2 == 0 {
                Vec3::new(
                    next() * 0.3 - 0.15,
                    next() * 0.3 - 0.15,
                    next() * 0.3 - 0.15,
                )
            } else {
                Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0)
            };
            let pb = pa + d;

            // 通用路径：多面体填充 + 盒对 extents 快路径。
            np.poly_a.fill(&np.polys[ia], pa, qa);
            np.poly_b.fill(&np.polys[ib], pb, qb);
            np.box_axes_a = None;
            np.box_axes_b = None;
            np.box_a = Some((ha, pa));
            np.box_b = Some((hb, pb));
            let generic = match np.sat(d) {
                Some((sep, n, src)) if sep <= np.skin => {
                    if np.clip(n, src) {
                        Some((sep, n, src, np.cand.clone()))
                    } else {
                        Some((sep, n, src, Vec::new()))
                    }
                }
                _ => None,
            };

            // 专用路径：体轴直生（不填多面体——与生产路径一致）。
            let aa = box_axes(qa);
            let ab = box_axes(qb);
            np.box_axes_a = Some(aa);
            np.box_axes_b = Some(ab);
            let dedicated = match np.sat(d) {
                Some((sep, n, src)) if sep <= np.skin => {
                    if np.clip(n, src) {
                        Some((sep, n, src, np.cand.clone()))
                    } else {
                        Some((sep, n, src, Vec::new()))
                    }
                }
                _ => None,
            };

            match (generic, dedicated) {
                (None, None) => {}
                (Some((s1, n1, r1, c1)), Some((s2, n2, r2, c2))) => {
                    assert_eq!(s1.to_bits(), s2.to_bits(), "k{k}: sep 位不等");
                    assert!(n1.dot(n2).abs() > 0.9999, "k{k}: 法线不一致");
                    assert_eq!(r1, r2, "k{k}: 来源分类不一致");
                    assert_eq!(c1.len(), c2.len(), "k{k}: 接触点数不一致");
                    for p in &c1 {
                        let hit = c2.iter().any(|q| {
                            q.feature == p.feature && (q.point - p.point).length() <= 1e-5
                        });
                        assert!(hit, "k{k}: 通用点 {:?} 在专用路径无对应", p.point);
                    }
                    if !c1.is_empty() {
                        contacts_seen += 1;
                    }
                }
                (g, s) => panic!(
                    "k{k}: 类别不一致 通用={:?} 专用={:?}",
                    g.is_some(),
                    s.is_some()
                ),
            }
        }
        assert!(
            contacts_seen >= 100,
            "接触样本仅 {contacts_seen}，鉴别力不足"
        );
    }
}
