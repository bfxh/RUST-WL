//! # vxl-phys-narrow
//!
//! 窄相（§2.4）：
//! - 凸-凸：SAT（面法线 + 棱叉积轴）+ 参考面 Sutherland–Hodgman 裁剪 → ≤4 点流形；
//! - 球：解析（球-球 / 球-凸体最近点）；
//! - 高度场：列采样特化（§2.4「列裁剪 + 局部采样」的 M0 版）；
//! - GJK/EPA 通用凸路径：**凸体外壳 × {盒|球|外壳}**（`gjk.rs`，多点流形由外壳面顶点细化）；
//!   **外壳 × 提供者**走顶点采样（逐顶点 SDF 解析）。
//!
//! 流形法线约定：`normal` 从 a 指向 b；求解器把 +n 冲量施加给 b、−n 施加给 a。
//! skin = speculative margin（§4.3）：分离距离 ≤ skin 仍生成「预期接触」。

// 注：本 crate 不再是 `forbid`——SAT 扫描有 SSE2 内核（`simd.rs`，用户已批准
// unsafe；SAFETY 注释见该模块），其余代码仍 `deny`（模块级 `allow` 仅限那里）。
#![deny(unsafe_code)]

pub mod gjk;
pub mod heightfield;
pub mod polytope;

use std::collections::HashMap;

use heightfield::HeightField;
use polytope::ConvexPolytope;
mod simd;

use vxl_phys_core::{JobSystem, Mat3, Quat, Shape, Vec3, CYLINDER_SEGMENTS};

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
    /// **特征空间**：复合体子形状序号 + 1（非复合体恒 0）。求解器用它区分**同一体对**上的
    /// 多条流形（复合体每个子形状一条）。**不借 `feature` 的任何位**——窄相特征号的高位被
    /// "侧别/裁剪路/哈希"编码占用，哈希逐帧漂移 ⇒ 借用会让普通场景暖启动随机失效（实测见
    /// `TECH-SURVEY.md` A9 ④ 的那次回退）。
    space: u16,
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

    /// 可变逐点视图（窄相内部用：复合体要给特征号并入子形状序号）。
    pub fn as_mut_slice(&mut self) -> &mut [ContactPoint] {
        &mut self.buf[..self.len as usize]
    }

    /// 特征空间（复合体子形状序号 + 1；非复合体恒 0）。
    pub fn space(&self) -> u16 {
        self.space
    }

    /// 设特征空间（窄相复合体展开时按子序号填；求解器读它做 warm 键的第三维）。
    pub fn set_space(&mut self, s: u16) {
        self.space = s;
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
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
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
/// provider 对的「接近面」判据下限（m/s）：闭合速度超过它才按接近方向选面，
/// 否则退回"点数最多、并列取最深"（静置/无明显接近时的原规则）。
const CLOSING_MIN: f32 = 0.1;

/// 特征 ID 的**种类**（诊断用）：`(入射侧是否为 B, 是否为裁剪交点)`。
///
/// 用途（`EXPERIMENTS.md` 末节 K 的后续）：求解器里"回退命中"的点是"特征 ID 变了但
/// 材料点还在附近"，把两个 ID 异或再按位域拆开即可判**变了哪一种**：
/// - 只有 bit31 变 ⇒ **入射侧翻转**（参考面选择在 A/B 之间跳）——最可修的一种；
/// - 只有 bit30 变 ⇒ 裁剪交点 ↔ 入射面顶点互换（裁剪路变化）；
/// - 哈希部分变 ⇒ 参考侧平面索引 k 或入射棱对变（`ref_base` 变化）。
pub fn feature_kind(f: u32) -> (bool, bool) {
    (f & FEAT_SIDE_B != 0, f & FEAT_CLIPPED != 0)
}

/// 特征 ID 的**哈希部分**（诊断用；去掉种类位）。
pub fn feature_hash_part(f: u32) -> u32 {
    f & !(FEAT_SIDE_B | FEAT_CLIPPED)
}

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

/// 姿态指纹（盒对轴缓存的键）：4×f32 位模式左旋混合——比 3 次四元数旋转廉价得多，
/// 且键含体号 ⇒ 不同体不会互撞；同体同姿态必然同值 ⇒ **逐位透明**。
#[inline]
fn rot_fp(rot: Quat) -> u64 {
    let mut h = rot.x.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.y.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.z.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.w.to_bits() as u64;
    h
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
    /// 凸体外壳仓库（多边形域；点云注册后由 shape 引用）。
    hulls: HullStore,
    /// 复合体仓库（子形状表；由 `Shape::Compound { compound, .. }` 引用）。
    compounds: CompoundStore,
    /// 子形状表 scratch（`kids_take`/`kids_put` 借出，避开 `&self`/`&mut self` 借用冲突）。
    kids_buf: Vec<CompoundChild>,
    skin: f32,
    /// **速度充气视野的预测时长**（s；0 = 不预测 ＝ 现行行为）。由 `World` 每子步设为
    /// **检测间隔**（每子步检测时为 `dt`、每 tick 检测时为整 tick）：窄相的接受判据
    /// 从 `sep ≤ skin` 放宽为 `sep ≤ skin + max(0, 接近速度)·predict_dt`，使
    /// "**下一次检测之前会碰上的接触**"提前成流形——求解器的 spec 项
    /// （`sep·inv_dt`）负责把逼近平滑拦停，不需要额外机制。
    ///
    /// 依据（`EXPERIMENTS.md` 末节 L/M）：检测每步一次时塔崩（KE 30 万）的真实机理是
    /// **逼近中的接触晚生**；把 skin 从 0.01 加到 0.02/0.04/0.08 即可让 KE 降到
    /// 2 386/1 992/1 271 且 y 带完整 ⇒ 视野不足而非语义不可行。
    /// `0` 时整条路径逐位不变（含三哈希）。
    predict_dt: f32,
    /// 本对的充气量（scratch）：由 SAT 预筛处按 (a, b, n) 计算，`clip` 的逐点过滤共用。
    inflate: f32,
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
    /// 体轴缓存（T3）：键 = (体号, 姿态指纹)。盒对路径原先**每对**都重算两体的
    /// **外壳世界点缓存**（两侧各一份）：键 = (体号, 姿态指纹)，与 `cached_ax_*`
    /// 同机制 ⇒ 同体同帧多对时只做一次 O(n) 世界变换（顶点采样/EPA 细化共用）。
    hull_pts: [Vec<Vec3>; 2],
    cached_hull: [(u32, u64); 2],
    /// 3 次旋转（同体在 (a,b) 序下连续出现，与 `cached_a/cached_b` 同机制）
    /// ⇒ 纯函数、命中即同值 ⇒ **逐位透明**。两槽分别给对的两个侧别。
    cached_ax_a: (u32, u64, [Vec3; 3]),
    cached_ax_b: (u32, u64, [Vec3; 3]),
    /// 裁剪顶点 scratch（参考面顶点）：通用与专用路径共用同一裁剪实现，
    /// 避免两份易漂移的裁剪代码。
    ref_v: Vec<Vec3>,
    /// 上一帧产出的流形数（下一帧并行块输出缓冲的容量提示）。纯性能提示：
    /// 不参与任何判定，故与确定性无关。
    out_hint: usize,
    /// 诊断计数器（**零开销**，仅整数自增）：裁剪调用数 / 内层顶点迭代总数 /
    /// 交点插值总数 / 进 `select_contacts` 前的候选点数。用途：用"每次调用的
    /// 迭代数"反推成本落在**固定开销**（面选择 + 装配 + 归约）还是**内层行走**
    /// ——计时探针在 1361 次/步下自身就要 ~200 µs/步，会淹没被测段。
    pub probe_clip_calls: u64,
    pub probe_clip_iters: u64,
    pub probe_clip_xings: u64,
    pub probe_cand_pts: u64,
    /// 观测到的**最大裁剪多边形长度**（`clip_in` 峰值）。用途：为"把
    /// `clip_in`/`clip_out` 换成定长数组"提供**实测上界**——盒对快路径理论上界
    /// 是 8（4 顶点入射面 + 4 次半平面裁剪，每次凸多边形裁剪 ≤ m+1），但通用
    /// 路径的面顶点数无小常数上界（圆柱面 = n 边形、凸包面可达数十顶点），
    /// 只有实测峰值才能支撑"定长 16 是否安全"的判断。
    pub probe_clip_max: u64,
}

impl DefaultNarrowPhase {
    /// 诊断读数：(裁剪调用数, 内层迭代总数, 插值总数, 候选点总数, 裁剪多边形峰值)。
    pub fn probe_stats(&self) -> (u64, u64, u64, u64, u64) {
        (
            self.probe_clip_calls,
            self.probe_clip_iters,
            self.probe_clip_xings,
            self.probe_cand_pts,
            self.probe_clip_max,
        )
    }
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
        Shape::Cone {
            half_height,
            radius,
        } => mix(3, half_height, radius, CYLINDER_SEGMENTS as f32),
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

/// **复合体子形状**：形状 + 相对复合体原点的局部平移/旋转（顺序即特征序）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundChild {
    pub shape: Shape,
    pub offset: Vec3,
    pub rot: Quat,
}

/// **复合体仓库**（刚性多形状体）：注册后由 `Shape::Compound { compound, .. }` 引用。
///
/// 窄相按子形状展开为**子对**并递归复用 `process_pair`；子序号左移 16 位并入 `feature`
/// （同一体对同时存在多条流形，不编码会让不同子形状的接触点在暖启动缓存上互相顶替）。
#[derive(Clone, Default)]
pub struct CompoundStore {
    items: Vec<Vec<CompoundChild>>,
}

impl CompoundStore {
    /// 注册一个复合体；返回 id。**嵌套复合体在此丢弃**（防递归；顺序即特征序，不去重）。
    pub fn add(&mut self, children: Vec<CompoundChild>) -> u32 {
        let id = self.items.len() as u32;
        self.items.push(
            children
                .into_iter()
                .filter(|c| !matches!(c.shape, Shape::Compound { .. }))
                .collect(),
        );
        id
    }

    #[inline]
    pub fn get(&self, id: u32) -> Option<&[CompoundChild]> {
        self.items.get(id as usize).map(|v| v.as_slice())
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// 子形状局部 AABB 并集半长（保守：子半长取**包围球**；空复合体返回 ZERO）。
    pub fn half_extents(&self, id: u32) -> Vec3 {
        let Some(kids) = self.get(id) else {
            return Vec3::ZERO;
        };
        if kids.is_empty() {
            return Vec3::ZERO;
        }
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for c in kids {
            let e = Vec3::splat(c.shape.bounding_sphere_radius());
            lo = lo.min(c.offset - e);
            hi = hi.max(c.offset + e);
        }
        (hi - lo) * 0.5
    }
}

/// 把新产出的一段流形里的接触点特征号打上**子形状序号**（`(ci+1) << 16`）。
///
/// 对**全部**点位生效（含原本 `feature == 0` 的）：球类接触的特征号恒为 0（"无特征"哨兵），
/// 而复合体里两个球子形状的接触点在同一个体对上会互相顶替 ⇒ 子序号必须成为可区分信息。
/// 低 16 位保持窄相原编码不变（`0` 时置为纯 tag）。
fn tag_child_features(ms: &mut [Manifold], ci: usize) {
    let tag = ((ci as u32) + 1) << 16;
    for m in ms.iter_mut() {
        // **显式空间通道**（求解器 warm 键的第三维）：不借 `feature` 的位（高位是编码/哈希）。
        m.points.set_space((ci + 1) as u16);
        for p in m.points.as_mut_slice() {
            p.feature = if p.feature == 0 { tag } else { p.feature | tag };
        }
    }
}

/// **凸体外壳仓库**（多边形域；窄相自持 ⇒ 零签名改动）。
///
/// 外壳点云注册后由 `Shape::ConvexHull { hull, .. }` 引用；查询按 id 直取。
/// 点云顺序即特征序（流形 `feature = 顶点序号+1`，跨帧稳定 ⇒ warm 缓存可续接）。
#[derive(Clone, Default)]
pub struct HullStore {
    hulls: Vec<gjk::ConvexHull>,
}

impl HullStore {
    /// 注册一个外壳（点云，局部坐标）；返回 id。点云顺序决定确定性（不去重）。
    pub fn add(&mut self, points: Vec<Vec3>) -> u32 {
        let id = self.hulls.len() as u32;
        self.hulls.push(gjk::ConvexHull::new(points));
        id
    }

    #[inline]
    pub fn get(&self, id: u32) -> Option<&gjk::ConvexHull> {
        self.hulls.get(id as usize)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.hulls.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.hulls.is_empty()
    }

    /// 局部 AABB 半长（宽相/惯量近似用；空壳返回 ZERO）。
    pub fn half_extents(&self, id: u32) -> Vec3 {
        let Some(h) = self.get(id) else {
            return Vec3::ZERO;
        };
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for p in &h.points {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        (hi - lo) * 0.5
    }
}

impl DefaultNarrowPhase {
    /// 注册凸体外壳（点云，局部坐标）→ id；配 `Shape::ConvexHull { hull, .. }` 使用。
    pub fn add_hull(&mut self, points: Vec<Vec3>) -> u32 {
        self.hulls.add(points)
    }

    /// 外壳点云（局部坐标；空切片 = id 无效）。
    pub fn hull_points(&self, id: u32) -> &[Vec3] {
        self.hulls
            .get(id)
            .map(|h| h.points.as_slice())
            .unwrap_or(&[])
    }

    /// 注册复合体（子形状 = 形状 + 局部平移/旋转）→ id；配 `Shape::Compound { compound, .. }`。
    pub fn add_compound(&mut self, children: Vec<CompoundChild>) -> u32 {
        self.compounds.add(children)
    }

    /// 复合体子形状表（局部；空切片 = id 无效）。
    pub fn compound_children(&self, id: u32) -> &[CompoundChild] {
        self.compounds.get(id).unwrap_or(&[])
    }

    /// 子形状局部 AABB 并集半长（宽相/惯量近似用；空复合体 = ZERO）。
    pub fn compound_half_extents(&self, id: u32) -> Vec3 {
        self.compounds.half_extents(id)
    }

    /// 子形状表借出到 scratch（递归前必须归还；嵌套复合体已在注册期丢弃 ⇒ 不会重入）。
    fn kids_take(&mut self, id: u32) -> Option<Vec<CompoundChild>> {
        let mut buf = std::mem::take(&mut self.kids_buf);
        buf.clear();
        match self.compounds.get(id) {
            Some(kids) => {
                buf.extend_from_slice(kids);
                Some(buf)
            }
            None => {
                self.kids_buf = buf;
                None
            }
        }
    }

    fn kids_put(&mut self, buf: Vec<CompoundChild>) {
        self.kids_buf = buf;
    }

    /// 外壳点云 → 局部 AABB 半长（门面构 `Shape::ConvexHull` 用）。
    pub fn hull_half_extents(&self, id: u32) -> Vec3 {
        self.hulls.half_extents(id)
    }

    /// **凸体 Voronoi 预断裂**：原壳 ✕ 种子 ⇒ 逐格点云（凸、互斥、并集 = 原体）。
    /// 局部坐标（种子也给局部坐标）。`None` = 外壳 id 无效。
    pub fn fracture_hull(&self, id: u32, seeds: &[Vec3]) -> Option<Vec<Vec<Vec3>>> {
        self.hulls
            .get(id)
            .map(|h| gjk::fracture_voronoi_hull(h, seeds))
    }

    /// 填充「外壳世界点缓存」（side：0 = 对侧 a、1 = 侧 b）。
    /// 返回 true = 该形状是外壳且缓存已就绪（点列在 `self.hull_pts[side]`）。
    fn fill_hull_world(
        &mut self,
        side: usize,
        body: u32,
        shape: &Shape,
        pos: Vec3,
        rot: Quat,
    ) -> bool {
        let Shape::ConvexHull { hull, .. } = *shape else {
            return false;
        };
        let fp = rot_fp(rot);
        if self.cached_hull[side] != (body, fp) {
            let m = Mat3::from_quat(rot);
            let out = &mut self.hull_pts[side];
            out.clear();
            if let Some(h) = self.hulls.get(hull) {
                out.extend(h.points.iter().map(|p| pos + m.mul_vec3(*p)));
            }
            self.cached_hull[side] = (body, fp);
        }
        true
    }

    /// 形状 → 支撑体（不支持的形状返回 None）。
    fn support_of(&self, shape: &Shape, pos: Vec3, rot: Quat) -> Option<gjk::ShapeSupport<'_>> {
        match *shape {
            Shape::ConvexHull { hull, .. } => self.hulls.get(hull).map(|h| {
                gjk::ShapeSupport::Hull(gjk::HullSupport {
                    hull: h,
                    pos,
                    rot: Mat3::from_quat(rot),
                })
            }),
            Shape::Box { half } => Some(gjk::ShapeSupport::Box(gjk::BoxSupport {
                half,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            Shape::Sphere { radius } => Some(gjk::ShapeSupport::Sphere(gjk::SphereSupport {
                radius,
                pos,
            })),
            Shape::Capsule {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Capsule(gjk::CapsuleSupport {
                half_height,
                radius,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            // 圆柱/圆锥：**多面化表示**的支撑（顶点有限 ⇒ EPA 良态）。此前缺这两支 ⇒
            // 「外壳 × 圆柱/锥」这类组合**静默无接触**（见 `TECH-SURVEY.md` A9 ④ 留档）。
            Shape::Cylinder {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Prism(gjk::PrismSupport {
                half_height,
                radius,
                segments: CYLINDER_SEGMENTS,
                cone: false,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            Shape::Cone {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Prism(gjk::PrismSupport {
                half_height,
                radius,
                segments: CYLINDER_SEGMENTS,
                cone: true,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            _ => None,
        }
    }

    /// **外壳 × {盒|球|外壳}**：GJK/EPA 求穿透 → 用**外壳近接触面顶点**细化成多点流形。
    ///
    /// - 法线一次求解（EPA，轴对齐退化时退 6 轴 SAT 解析）；
    /// - 流形点 = 外壳点云中落在对方支撑面 `plane ± skin` 带内的顶点，
    ///   逐点深度 `plane − n̂·v`（n̂ = 对方 → 外壳）；取最深 4 点。
    /// - `feature = 顶点序号 + 1`（点云序稳定 ⇒ 跨帧可续接）。
    /// - 外壳 × 高度场：**已支持**——走 `hull_heightfield`（逐顶点采样，与 `poly_heightfield`
    ///   同款），不再走本函数。
    #[allow(clippy::too_many_arguments)] // 与 process_pair 同形（两侧位姿 + 形状 + 出参）
    fn hull_pair(
        &mut self,
        a: u32,
        b: u32,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    ) {
        let a_is_hull = matches!(*sa, Shape::ConvexHull { .. });
        let (side, body_id, hshape, hpos, hrot) = if a_is_hull {
            (0usize, a, sa, pa, ra)
        } else {
            (1usize, b, sb, pb, rb)
        };
        let other_shape = if a_is_hull { sb } else { sa };
        // 世界点缓存（早于借支撑体：填充需要 &mut self）
        if !self.fill_hull_world(side, body_id, hshape, hpos, hrot) {
            return;
        }

        // —— 对方是高度场（L1）：顶点采样，与盒/圆柱版同构 ——
        if let Shape::HeightField(hf_id) = *other_shape {
            let Some(hf) = heightfields.get(hf_id as usize) else {
                return;
            };
            self.cand.clear();
            let n_pts = self.hull_pts[side].len();
            for idx in 0..n_pts {
                let v = self.hull_pts[side][idx];
                if let Some((h, _)) = hf.sample(v.x, v.z) {
                    let depth = h - v.y;
                    if depth > -self.skin {
                        self.cand.push(ContactPoint {
                            point: Vec3::new(v.x, h, v.z),
                            depth,
                            feature: idx as u32,
                        });
                    }
                }
            }
            if self.cand.is_empty() || !self.select_contacts(self.min_point_sep) {
                return;
            }
            let deepest = self.cand[0];
            let n_t = hf
                .sample(deepest.point.x, deepest.point.z)
                .map(|(_, n)| n)
                .unwrap_or(Vec3::Y);
            // 法线约定 a→b：外壳在 a（地形在 b）⇒ −n_t；否则 +n_t
            let normal = if a_is_hull { -n_t } else { n_t };
            out.push(Manifold {
                a,
                b,
                normal,
                points: ContactPoints::from_slice(&self.cand),
            });
            return;
        }

        // —— 对方是盒/球/外壳：GJK/EPA 一次法线 + 外壳近面顶点细化 ——
        let (Some(ua), Some(ub)) = (self.support_of(sa, pa, ra), self.support_of(sb, pb, rb))
        else {
            return; // 对方形状不受理
        };
        let Some((n_p, _depth, _p)) = gjk::epa(&ua, &ub, 32) else {
            return; // 未相交（宽相 fat 边距会给出近邻对）
        };
        let n = if a_is_hull { n_p } else { -n_p }; // 对方 → 外壳
        let other: &dyn gjk::Support = if a_is_hull { &ub } else { &ua };
        let plane = n.dot(other.support(n));
        let mut cand: Vec<(f32, usize, Vec3)> = Vec::new();
        let n_pts = self.hull_pts[side].len();
        for i in 0..n_pts {
            let w = self.hull_pts[side][i];
            let d = plane - n.dot(w);
            if d > -self.skin {
                cand.push((d, i, w));
            }
        }
        if cand.is_empty() {
            return;
        }
        // 最深 4 点（深度降序；并列按顶点序 ⇒ 确定性）
        cand.sort_by(|x, y| {
            y.0.partial_cmp(&x.0)
                .unwrap_or(core::cmp::Ordering::Equal)
                .then(x.1.cmp(&y.1))
        });
        cand.truncate(4);
        let pts: Vec<ContactPoint> = cand
            .iter()
            .map(|&(d, i, w)| ContactPoint {
                point: w,
                depth: d,
                feature: (i as u32) + 1,
            })
            .collect();
        out.push(Manifold {
            a,
            b,
            normal: -n_p, // 流形约定：a → b
            points: ContactPoints::from_slice(&pts),
        });
    }

    /// **设置速度充气视野的预测时长**（0 = 关闭；见 `predict_dt` 字段注）。
    /// 由 `World` 在每次 `collide` 前设置：检测间隔 = 距下一次窄相的时间。
    pub fn set_predict_dt(&mut self, dt: f32) {
        self.predict_dt = if dt > 0.0 { dt } else { 0.0 };
    }

    /// 本对的充气量＝`max(0, 接近速度)·predict_dt`（`predict_dt = 0` ⇒ 恒 0）。
    /// 接近速度取**质心**相对速度在法向的投影（忽略角速度贡献：角项在贴面接触上是一阶小量）。
    fn predict_inflate(&self, a: u32, b: u32, bodies: &vxl_phys_core::BodySet, n: Vec3) -> f32 {
        if self.predict_dt <= 0.0 {
            return 0.0;
        }
        let vrel = bodies.linvel[b as usize] - bodies.linvel[a as usize];
        (-vrel.dot(n)).max(0.0) * self.predict_dt
    }

    pub fn new(skin: f32) -> Self {
        Self {
            hulls: HullStore::default(),
            compounds: CompoundStore::default(),
            kids_buf: Vec::new(),
            skin,
            predict_dt: 0.0,
            inflate: 0.0,
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
            hull_pts: [Vec::new(), Vec::new()],
            cached_hull: [(u32::MAX, u64::MAX), (u32::MAX, u64::MAX)],
            cached_ax_a: (u32::MAX, u64::MAX, [Vec3::ZERO; 3]),
            cached_ax_b: (u32::MAX, u64::MAX, [Vec3::ZERO; 3]),
            ref_v: Vec::new(),
            out_hint: 256,
            probe_clip_calls: 0,
            probe_clip_iters: 0,
            probe_clip_xings: 0,
            probe_cand_pts: 0,
            probe_clip_max: 0,
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
            Shape::Cone {
                half_height,
                radius,
            } => ConvexPolytope::cone_polytope(radius, half_height, CYLINDER_SEGMENTS),
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
        // 契约（写清楚，因为原断言把它写反了）：**通用路径的多面体必须已填**
        // （`na/nb` 即面轴条数）。分派侧保证这件事：盒对走 T3 专用路径、**不填**多面体
        // （轴由 (rot, half) 直生）；圆柱/圆锥参与的对走通用分支，进 `sat` 前已
        // `poly_a/poly_b.fill`（世界多面体缓存命中时用的是上一次同 (体, 多面体) 的填充，
        // 长度同源）。⇒ 通用路径下 `na/nb` 恒 > 0；读它们当"面轴数"是对的。
        // ⚠️ 原文是 `debug_assert!(box_axes.is_some() || (na == 6 && nb == 6))`——
        // 第二个析取项与紧随其后的 `match box_axes { None => (na, nb) }` 自相矛盾：
        // 圆柱是 **18 面**（`CYLINDER_SEGMENTS=16` + 两端面），盒×柱是正当组合。
        // 实测（2026-09-22）：debug 档把 `tests/rolling_probe.rs` 打成 panic，
        // 而 release 因 `debug_assert` 被编译掉**掩盖**了它（CI 跑 release ⇒ 全绿）。
        // 现在断言的是真契约——"进了通用路径却没填多面体"才是要抓的 bug（会静默出垃圾接触）。
        debug_assert!(
            box_axes.is_some() || (na > 0 && nb > 0),
            "通用路径的多面体必须已填：na={na}, nb={nb}"
        );
        // 面轴计数：专用路径恒 6+6（体轴直生，未填多面体），通用路径读多面体。
        let (n_face_a, n_face_b) = match box_axes {
            Some(_) => (6usize, 6usize),
            None => (na, nb),
        };
        // ===== 盒对快路径：SIMD 4 轴并行扫描（x86_64 SSE2；非 x86_64 走标量）=====
        // 与下方通用标量循环**逐位等价**（轴内算术/结合序/三条规则全同），
        // `simd::tests::simd_matches_scalar_bitwise` 逐位对照守门。
        // 注：曾试「面轴/棱轴分段扫描（分离时跳过棱轴构建）」——实测**更慢**
        // （窄相峰 26.51 → 28.79，棱轴构建不是瓶颈、分段徒增重入）⇒ 已回退。
        if let Some((aa, ab)) = box_axes {
            let (ha, pa) = self.box_a.expect("盒对快路径必置 box_a");
            let (hb, pb) = self.box_b.expect("盒对快路径必置 box_b");
            let scanned = simd::sat_scan(&self.axes, ha, &aa, pa, hb, &ab, pb, self.skin);
            return scanned.map(|(sep, n, idx)| {
                let src = if idx < n_face_a {
                    AxisSrc::FaceA
                } else if idx < n_face_a + n_face_b {
                    AxisSrc::FaceB
                } else {
                    AxisSrc::Edge
                };
                (sep, n, src)
            });
        }
        // ===== 非盒对（球/圆柱/高度场参与）：通用标量扫描 =====
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
            self.probe_clip_iters += m as u64;
            self.probe_clip_max = self.probe_clip_max.max(m as u64);
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
                    self.probe_clip_xings += 1;
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

        // 主平面过滤：保留 n_ref 方向距离 ≤ skin（+本对充气量）的点（depth = -dist）。
        let p0 = self.ref_v[0];
        self.cand.clear();
        for &(v, feat) in &self.clip_in {
            let d = (v - p0).dot(n_ref);
            if d <= self.skin + self.inflate {
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
        self.probe_clip_calls += 1;
        self.probe_cand_pts += self.cand.len() as u64;

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

    /// **胶囊中心线 ↔ 凸体**：中心线取最近点后复用球的解析路径（`sphere_convex_ab`）。
    /// 返回 `(中心线最近点, 对方表面点)`；无接触返回 `None`。
    ///
    /// 为什么不用 GJK/EPA（`EXPERIMENTS.md` R.1/R.2 两次实测）：EPA 对**光滑**支撑不适定
    /// （临界接触给 45° 假法线 + 42 m 假深度）；"收成芯再跑 GJK"又踩 GJK 的**面平局**退化
    /// （盒顶面对 ±Y 时支撑解到角点 + 无进展提前退出，实测 `d=7.077 / p_other=(5,0,5)`）。
    /// 解析最近点两条坑都不碰。
    ///
    /// 采样点为什么就是段上最近点：点到**凸**集的距离沿线段是凸函数 ⇒ 迭代
    /// `q ← 体上最近点(p)`、`p ← 段上最近点(q)` 的驻点即全局最近点对，且距离单调不增。
    fn capsule_axis_reach(
        &mut self,
        s0: Vec3,
        s1: Vec3,
        radius: f32,
        other: &Shape,
        opos: Vec3,
        orot: Quat,
    ) -> Option<(Vec3, Vec3)> {
        let idx = self.poly_for(other)?;
        self.poly_b.fill(&self.polys[idx], opos, orot);
        let seg = s1 - s0;
        let seg_len2 = seg.length_squared();
        let mut p = (s0 + s1) * 0.5;
        for _ in 0..4 {
            let (q, _d2, inside, in_n) = closest_point_on_poly(&self.poly_b, p);
            // 内部时同样把 q 拉到"朝最近面"的表面上，迭代方向才有效。
            let q = if inside {
                p + in_n * -max_plane_d_of(&self.poly_b, p)
            } else {
                q
            };
            let t = if seg_len2 > 1e-18 {
                ((q - s0).dot(seg) / seg_len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let p_new = s0 + seg * t;
            let done = (p_new - p).length_squared() <= 1e-12;
            p = p_new;
            if done {
                break;
            }
        }
        // 采样点交给球的解析路径（其内部会再填一次 `poly_b`，成本可接受）。
        let (_n, _depth, surf) = self.sphere_convex_ab(p, radius, other, opos, orot)?;
        Some((p, surf))
    }

    /// **胶囊体 × 凸体：GJK 距离**。返回 `(n_cap→other, dist, p_cap, p_other)`；
    /// 线段与对方重叠（`Intersecting`）时改用 EPA 求穿透法线（返回 `dist = -depth`）。
    ///
    /// 为什么不能用 SAT/裁剪：胶囊是**光滑**外形，SAT 需要有限面集。为什么不能直接用
    /// EPA：EPA 只在**重叠**时可用，而"胶囊接触"的定义是**线段到对方的距离 < radius**
    /// （此时线段本身可能完全没碰到对方）。GJK 距离恰好给出这个量。
    fn capsule_reach(
        cap: &gjk::CapsuleSupport,
        other: &dyn gjk::Support,
    ) -> Option<(Vec3, f32, Vec3, Vec3)> {
        match gjk::gjk(cap, other) {
            gjk::Gjk::Separated {
                point_a,
                point_b,
                dist,
            } => {
                let n = if dist > 1e-9 {
                    (point_b - point_a) * (1.0 / dist)
                } else {
                    Vec3::Y
                };
                Some((n, dist, point_a, point_b))
            }
            gjk::Gjk::Intersecting => {
                let (n, depth, p) = gjk::epa(cap, other, 32)?;
                Some((n, -depth, p, p))
            }
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

    /// 胶囊 × 高度场：**沿中心线取 N 个样本**，每个样本按半径 r 的球处理
    /// （候选点 `depth = h − y + r`、接触点 `(x, h, z)`、法线取地形法线）。
    /// 覆盖：两端 + 等分中间点（平躺胶囊靠两端、竖直靠底端；`select_contacts` 取最深 ≤4）。
    /// `feature = 样本序号 + 1`（等分序稳定 ⇒ 跨帧可续接）。
    fn capsule_heightfield(
        &mut self,
        seg_a: Vec3,
        seg_b: Vec3,
        radius: f32,
        hf: &HeightField,
    ) -> bool {
        const SAMPLES: u32 = 5;
        self.cand.clear();
        let denom = (SAMPLES - 1) as f32;
        for k in 0..SAMPLES {
            let t = k as f32 / denom;
            let s = seg_a + (seg_b - seg_a) * t;
            if let Some((hgt, _)) = hf.sample(s.x, s.z) {
                let depth = hgt - s.y + radius;
                if depth > -self.skin {
                    self.cand.push(ContactPoint {
                        point: Vec3::new(s.x, hgt, s.z),
                        depth,
                        feature: k + 1,
                    });
                }
            }
        }
        if self.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }

    /// 外壳 × 高度场：**逐顶点采样**（与 `poly_heightfield` 同款，只是顶点来自外壳点云）。
    /// 此前该组合**不受理**（`hull_pair` 的注："列裁剪对任意凸壳未实现"）。
    /// 特征 = 顶点序号 + 1（点云序稳定 ⇒ 跨帧可续接）。点云较密时接触点靠 `select_contacts`
    /// 截断到 ≤4。
    fn hull_heightfield(&mut self, hull: u32, pos: Vec3, rot: Quat, hf: &HeightField) -> bool {
        self.cand.clear();
        let r = Mat3::from_quat(rot);
        let Some(h) = self.hulls.get(hull) else {
            return false;
        };
        for (idx, &p) in h.points.iter().enumerate() {
            let v = pos + r.mul_vec3(p);
            if let Some((hgt, _)) = hf.sample(v.x, v.z) {
                let depth = hgt - v.y;
                if depth > -self.skin {
                    self.cand.push(ContactPoint {
                        point: Vec3::new(v.x, hgt, v.z),
                        depth,
                        feature: idx as u32 + 1,
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
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
        jobs: &dyn JobSystem,
    ) {
        out.clear();
        // 每对各自的充气量（`clip` 逐点过滤复用）；默认 0 ⇒ 逐位同现行。
        self.inflate = 0.0;
        // 世界多面体填充缓存跨帧失效（体在帧间移动；键只含体号+形状）。
        self.cached_a = (u32::MAX, u64::MAX);
        self.cached_b = (u32::MAX, u64::MAX);
        self.cached_ax_a = (u32::MAX, u64::MAX, [Vec3::ZERO; 3]);
        self.cached_ax_b = (u32::MAX, u64::MAX, [Vec3::ZERO; 3]);
        self.cached_hull = [(u32::MAX, u64::MAX), (u32::MAX, u64::MAX)];
        let threads = jobs.threads();
        // 小规模串行（线程启动开销 > 收益）；并行 = 每块独立 clone（含自有
        // scratch），结果按块序拼接 = pair 序（§5 确定性契约）。
        if threads <= 1 || pairs.len() < 2048 {
            for &(a, b) in pairs {
                self.process_pair(a, b, bodies, heightfields, providers, out);
            }
            return;
        }
        let this = &*self;
        let n_chunks = threads.min(pairs.len().div_ceil(2048));
        let chunk = pairs.len().div_ceil(n_chunks);
        // 输出缓冲预分配（T3 结构项②的第一片）：8B 场景实测对/流形 ≈ 15:1，
        // 旧实现每块从空 Vec 逐次增长（每块 ~8 次重分配 + memcpy），且最终
        // 拼接要把全部流形再搬一遍。按下界预留容量即免掉这一段。
        let out_hint = self.out_hint.max(16);
        let mut outs: Vec<Vec<Manifold>> = (0..n_chunks)
            .map(|_| Vec::with_capacity(out_hint / n_chunks + 8))
            .collect();
        out.reserve(out_hint);
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
                        np.process_pair(a, b, bodies, heightfields, providers, co);
                    }
                }
            },
        );
        let mut produced = 0usize;
        for mut co in outs {
            produced += co.len();
            out.append(&mut co);
        }
        // 下一帧的容量提示（纯性能提示，不参与任何判定 ⇒ 确定性无关）。
        self.out_hint = produced;
    }
}

impl DefaultNarrowPhase {
    /// 薄入口：从 `bodies` 取形状/位姿后交给 `process_pair_shaped`（复合体递归走后者）。
    fn process_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        heightfields: &[HeightField],
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
    ) {
        let (sa, sb) = (&bodies.shape[a as usize], &bodies.shape[b as usize]);
        let pa = bodies.position[a as usize];
        let pb = bodies.position[b as usize];
        let ra = bodies.rot(a as usize);
        let rb = bodies.rot(b as usize);
        self.process_pair_shaped(
            a,
            b,
            bodies,
            sa,
            sb,
            pa,
            ra,
            pb,
            rb,
            heightfields,
            providers,
            out,
        );
    }

    /// **配对主入口**（形状 + 世界位姿已给定）：复合体在此按子形状展开并**递归**本函数。
    #[allow(clippy::too_many_arguments)] // 两侧形状 + 位姿 + 上下文 + 出参
    fn process_pair_shaped(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        heightfields: &[HeightField],
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
    ) {
        // 盒对专用路径开关：每对先复位（非盒对 / 圆柱对一律走通用路径）。
        self.box_axes_a = None;
        self.box_axes_b = None;

        // —— 复合体：子形状展开为**子对**并递归本函数 ——
        //
        // 放在**最前**（先于地形/提供者分支）⇒ 子形状各自走完整配对路径（含地形）。
        // 同一体对会**同时**存在多条流形（每个子形状各一条）：`feature` 是暖启动缓存键的
        // 一部分，不并入子序号会让不同子形状的接触点互相顶替（`0` 是"无特征"哨兵，保持 0）。
        if let Shape::Compound { compound, .. } = *sa {
            let Some(kids) = self.kids_take(compound) else {
                return;
            };
            for (ci, kid) in kids.iter().enumerate() {
                let before = out.len();
                let cpos = pa + Mat3::from_quat(ra).mul_vec3(kid.offset);
                let crot = ra * kid.rot;
                self.process_pair_shaped(
                    a,
                    b,
                    bodies,
                    &kid.shape,
                    sb,
                    cpos,
                    crot,
                    pb,
                    rb,
                    heightfields,
                    providers,
                    out,
                );
                tag_child_features(&mut out[before..], ci);
            }
            self.kids_put(kids);
            return;
        }
        if let Shape::Compound { compound, .. } = *sb {
            let Some(kids) = self.kids_take(compound) else {
                return;
            };
            for (ci, kid) in kids.iter().enumerate() {
                let before = out.len();
                let cpos = pb + Mat3::from_quat(rb).mul_vec3(kid.offset);
                let crot = rb * kid.rot;
                self.process_pair_shaped(
                    a,
                    b,
                    bodies,
                    sa,
                    &kid.shape,
                    pa,
                    ra,
                    cpos,
                    crot,
                    heightfields,
                    providers,
                    out,
                );
                tag_child_features(&mut out[before..], ci);
            }
            self.kids_put(kids);
            return;
        }

        // **外部碰撞提供者参与的对**（体素/网格/喷溅场…；ROUTE §2.1 兼容轴）。
        // 只有「盒 vs provider」走此路径（其余形状待 provider 专用解法补齐）；
        // 法线约定与高度场一致：provider 在 a → +n_s（外向）；在 b → −n_s。
        let pr_a = match sa {
            Shape::Provider(id) => Some(*id),
            _ => None,
        };
        let pr_b = match sb {
            Shape::Provider(id) => Some(*id),
            _ => None,
        };
        if pr_a.is_some() || pr_b.is_some() {
            if pr_a.is_some() && pr_b.is_some() {
                return; // provider-provider 暂不支持（需要 provider 对偶解法）
            }
            let (body_shape, bpos, brot, pr_is_a) = if pr_a.is_some() {
                (sb, pb, rb, true)
            } else {
                (sa, pa, ra, false)
            };
            let id = pr_a.or(pr_b).unwrap();
            let mut buf: Vec<vxl_phys_core::interop::InteropContact> = Vec::new();
            // **接触带按相对速度自适应**（实测修复）：固定 skin（0.02 m）小于每 tick
            // 位移（8 m/s ⇒ 0.13 m）时，体**跨过皮肤带** ⇒ 进入体内才建接触，此时
            // 接近速度已≈0 ⇒ 冲击判据不触发、且无 spec 减速（实测：8 m/s 弹体无声
            // 停在墙前 0.04 m、零破坏）。带 = max(skin, |v_rel|·dt·1.5)；dt 取引擎
            // 固定基础步 1/60（子步更小 ⇒ 该带偏保守、安全）。
            let vrel = if pr_is_a {
                bodies.linvel[b as usize] - bodies.linvel[a as usize]
            } else {
                bodies.linvel[a as usize] - bodies.linvel[b as usize]
            };
            // **+ skin 余量**（2026-09-15 修复）：带恰等于「样点到表面距离」时
            // 接触的出现与否只在速度上差 0.5%（实测基准 wall_provider：2 子步下
            // 0.1398 vs 0.14 的刀锋 ⇒ 墙面接触整整晚一个子步出现，期间水平动量
            // 被倾斜的地板法线吸收、撞墙事件不登记）。加一个 skin 把刀锋推开，
            // 使「带内即建（预测）接触」在速度上连续；预测接触由求解器按
            // distance/dt 限接近速度，不会造成假制动。
            let band = self
                .skin
                .max(vrel.length() * (1.0 / 60.0) * 1.5 + self.skin);
            let ok = match *body_shape {
                Shape::Box { half } => providers.contacts_box(id, half, bpos, brot, band, &mut buf),
                // 球：SDF 类提供者解析求解（`depth = r − sdf(center)`）
                Shape::Sphere { radius } => {
                    providers.contacts_sphere(id, bpos, radius, band, &mut buf)
                }
                // 外壳 vs 提供者：**顶点采样**（逐顶点按 SDF 解析求深度/法线；
                // 多点 ⇒ 面接触稳定）。顶点序即特征序之外的 provider 特征由各点给。
                Shape::ConvexHull { .. } => {
                    // 世界点走缓存（同体同帧多对时只做一次 O(n) 变换）
                    let side = if pr_is_a { 1 } else { 0 };
                    let body = if pr_is_a { b } else { a };
                    if !self.fill_hull_world(side, body, body_shape, bpos, brot) {
                        return;
                    }
                    let mut supported = false;
                    for k in 0..self.hull_pts[side].len() {
                        supported |=
                            providers.contacts_point(id, self.hull_pts[side][k], band, &mut buf);
                    }
                    supported
                }
                _ => false, // 其余形状 vs provider：待专用查询
            };
            if !ok || buf.is_empty() {
                return; // 不支持 / 全部顶点都不在接触带内
            }
            let sgn = if pr_is_a { 1.0 } else { -1.0 };
            // 流形法线 = 多面候选里选**主导接触面**。候选来自 provider 的
            // **逐面发射**（每张面各自发带内样本；共享角点会在相邻面里重复出现
            // ——只有「面心样本」（feature % 16 == 0）能证明该面真的贴着）。
            // 选择次序（2026-09-15 修复，取代旧的"点数最多、并列取最深"单一规则）：
            //   ① 有**闭合速度**的面（法线逆着相对速度 = 正在撞上去）优先，取最大者；
            //   ② 否则只考虑**带面心样本**的组（排除只有角点的"伪面"）；
            //   ③ 组内仍按点数最多、并列取最深。
            // 实测：旧的单一规则在墙角按深度选中**地板面**、丢掉墙面 ⇒ 体的水平
            // 动量被倾斜地板法线吸收、撞墙事件不登记（bench `wall_provider`：
            // 12 m/s 弹体停在墙前 0.14 m、vx 11.4→−0.02）。
            let quant = |v: Vec3| -> (i32, i32, i32) {
                (
                    (v.x * 10.0).round() as i32,
                    (v.y * 10.0).round() as i32,
                    (v.z * 10.0).round() as i32,
                )
            };
            // 小组统计（确定性；组数 ≤ 10）
            struct Group {
                key: (i32, i32, i32),
                count: usize,
                deepest: f32,
                closing: f32,
                has_center: bool,
            }
            let mut groups: Vec<Group> = Vec::with_capacity(8);
            for c in &buf {
                let key = quant(c.normal);
                let idx = match groups.iter().position(|g| g.key == key) {
                    Some(i) => i,
                    None => {
                        groups.push(Group {
                            key,
                            count: 0,
                            deepest: f32::NEG_INFINITY,
                            closing: f32::NEG_INFINITY,
                            has_center: false,
                        });
                        groups.len() - 1
                    }
                };
                let g = &mut groups[idx];
                g.count += 1;
                g.deepest = g.deepest.max(c.depth);
                g.closing = g.closing.max(-(c.normal * sgn).dot(vrel));
                if c.feature % 16 == 0 {
                    g.has_center = true;
                }
            }
            let closing_group = groups
                .iter()
                .filter(|g| g.closing > CLOSING_MIN)
                .max_by(|x, y| x.closing.total_cmp(&y.closing));
            let pick = closing_group.or_else(|| {
                let with_center: Vec<&Group> = groups.iter().filter(|g| g.has_center).collect();
                let pool: Vec<&Group> = if with_center.is_empty() {
                    groups.iter().collect()
                } else {
                    with_center
                };
                pool.into_iter()
                    .max_by(|x, y| x.count.cmp(&y.count).then(x.deepest.total_cmp(&y.deepest)))
            });
            let (best_key, best_normal) = match pick {
                Some(g) => (
                    g.key,
                    buf.iter()
                        .find(|c| quant(c.normal) == g.key)
                        .map(|c| c.normal * sgn)
                        .unwrap_or(Vec3::Y),
                ),
                None => return,
            };
            let normal = best_normal;
            let pts: Vec<ContactPoint> = buf
                .iter()
                .filter(|c| quant(c.normal) == best_key)
                .take(4)
                .map(|c| ContactPoint {
                    point: c.point,
                    depth: c.depth,
                    feature: c.feature,
                })
                .collect();
            out.push(Manifold {
                a,
                b,
                normal,
                points: ContactPoints::from_slice(&pts),
            });
            return;
        }

        // **凸体外壳参与的对**（多边形域）：外壳 × {盒|球|外壳} → GJK/EPA。
        // 与提供者的组合已在上面的 provider 分支处理；与高度场暂不受理。
        if matches!(*sa, Shape::ConvexHull { .. }) || matches!(*sb, Shape::ConvexHull { .. }) {
            self.hull_pair(a, b, sa, sb, pa, ra, pb, rb, heightfields, out);
            return;
        }

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
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. } => {
                    let idx = match self.poly_for(body_shape) {
                        Some(i) => i,
                        None => return,
                    };
                    self.poly_heightfield(idx, bpos, brot, hf)
                }
                Shape::ConvexHull { hull, .. } => self.hull_heightfield(hull, bpos, brot, hf),
                Shape::HeightField(_) | Shape::Provider(_) => return,
                // 复合体已在上游按子形状展开（本臂不可达，留作穷尽性）。
                Shape::Compound { .. } => return,
                // 胶囊体 × 地形：沿中心线取 N 个样本（每个样本按球处理）。
                Shape::Capsule {
                    half_height,
                    radius,
                } => {
                    let axis = Mat3::from_quat(brot).mul_vec3(Vec3::Y);
                    self.capsule_heightfield(
                        bpos - axis * half_height,
                        bpos + axis * half_height,
                        radius,
                        hf,
                    )
                }
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
                // 体轴缓存（T3）：同体连续出现 ⇒ 每体每帧只算一次 3 次旋转。
                let fp_a = rot_fp(ra);
                self.box_axes_a = Some(if self.cached_ax_a.0 == a && self.cached_ax_a.1 == fp_a {
                    self.cached_ax_a.2
                } else {
                    let ax = box_axes(ra);
                    self.cached_ax_a = (a, fp_a, ax);
                    ax
                });
                let fp_b = rot_fp(rb);
                self.box_axes_b = Some(if self.cached_ax_b.0 == b && self.cached_ax_b.1 == fp_b {
                    self.cached_ax_b.2
                } else {
                    let ax = box_axes(rb);
                    self.cached_ax_b = (b, fp_b, ax);
                    ax
                });
                if let Some((sep, n, src)) = self.sat(pb - pa) {
                    // 速度充气视野（见 `predict_dt` 字段注）：`0` ⇒ 逐位同现行。
                    self.inflate = self.predict_inflate(a, b, bodies, n);
                    if sep > self.skin + self.inflate {
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
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. },
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. },
            ) => {
                // 圆柱/圆锥参与的对：走通用路径（多面体填充 + 逐顶点/通用 SAT）。
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
                    // 速度充气视野（同盒对路径；见 `predict_dt` 字段注）。
                    self.inflate = self.predict_inflate(a, b, bodies, n);
                    if sep > self.skin + self.inflate {
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
            // —— 胶囊体 × {盒|球|外壳|胶囊|圆柱}：**GJK 距离路径** ——
            // 光滑外形不能走 SAT/裁剪；接触判据 = **线段到对方的距离 < radius**
            // （此时线段本身可能并未碰到对方，故 EPA 也不适用）。流形点 = 线段两端
            // 各自的表面点（贴平躺给 2 点支撑，竖直时只有一端落进 skin 带 ⇒ 退化为 1 点）。
            (
                Shape::Capsule {
                    half_height,
                    radius,
                },
                _,
            ) => {
                let axis = Mat3::from_quat(ra).mul_vec3(Vec3::Y);
                // 凸体对方走**解析最近点**（`EXPERIMENTS.md` R.2）；对方不是多面体（胶囊/球）
                // 时退回 GJK 距离老路。两条路都给出对方**表面点** `p_other`。
                let (n_raw, p_other) = if self.poly_for(sb).is_some() {
                    let Some((p_sample, surf)) = self.capsule_axis_reach(
                        pa - axis * half_height,
                        pa + axis * half_height,
                        radius,
                        sb,
                        pb,
                        rb,
                    ) else {
                        return;
                    };
                    (p_sample - surf, surf)
                } else {
                    let cap = gjk::CapsuleSupport {
                        half_height,
                        radius,
                        pos: pa,
                        rot: Mat3::from_quat(ra),
                    };
                    // 借用作用域：`support_of` 借 `self.hulls` ⇒ 先把结论算成局部值。
                    let reach = match self.support_of(sb, pb, rb) {
                        Some(other) => Self::capsule_reach(&cap, &other),
                        None => None,
                    };
                    let Some((_n, _dist, p_cap, p_other)) = reach else {
                        return;
                    };
                    (p_other - p_cap, p_other)
                };
                // 法线 a→b：初始符号不重要——**一律用两体中心定号**（GJK 穿透时见证点会换序，
                // 这正是要抹平的不确定性）。
                let mut n_ab = n_raw;
                if n_ab.length_squared() < 1e-18 {
                    n_ab = pb - pa;
                }
                if n_ab.length_squared() < 1e-18 {
                    return;
                }
                let mut n_ab = n_ab.normalize();
                if n_ab.dot(pb - pa) < 0.0 {
                    n_ab = -n_ab;
                }
                // 压入量用**见证点平面**（对方表面点沿 n 的投影），不是对方的支撑平面：大盒配
                // 微倾法线时支撑平面会给出数十米偏移（R.1 实测的 42 m）。两条路的 `p_other`
                // 都是真表面点，此式统一适用。
                let plane = n_ab.dot(p_other);
                let mut local: Vec<ContactPoint> = Vec::with_capacity(2);
                for (i, s) in [pa - axis * half_height, pa + axis * half_height]
                    .into_iter()
                    .enumerate()
                {
                    // 该端帽压入量（n 由胶囊指向对方）：`n·端点 + radius − plane`；
                    // 平面取对方朝胶囊那侧，故压入为正。
                    let depth = n_ab.dot(s) + radius - plane;
                    if depth > -self.skin {
                        local.push(ContactPoint {
                            // 接触点 = 朝向对方那侧的帽面（+n 侧）。
                            point: s + n_ab * radius,
                            depth,
                            feature: i as u32 + 1,
                        });
                    }
                }
                if local.is_empty() {
                    return;
                }
                self.cand.clear();
                self.cand.extend_from_slice(&local);
                if !self.select_contacts(self.min_point_sep) {
                    return;
                }
                out.push(Manifold {
                    a,
                    b,
                    normal: n_ab,
                    points: ContactPoints::from_slice(&self.cand),
                });
            }
            (
                _,
                Shape::Capsule {
                    half_height,
                    radius,
                },
            ) => {
                let axis = Mat3::from_quat(rb).mul_vec3(Vec3::Y);
                // 同臂 1：凸体对方走解析最近点，非多面体（胶囊/球）退回 GJK 老路。
                let (n_raw, p_other) = if self.poly_for(sa).is_some() {
                    let Some((p_sample, surf)) = self.capsule_axis_reach(
                        pb - axis * half_height,
                        pb + axis * half_height,
                        radius,
                        sa,
                        pa,
                        ra,
                    ) else {
                        return;
                    };
                    (surf - p_sample, surf)
                } else {
                    let cap = gjk::CapsuleSupport {
                        half_height,
                        radius,
                        pos: pb,
                        rot: Mat3::from_quat(rb),
                    };
                    let reach = match self.support_of(sa, pa, ra) {
                        Some(other) => Self::capsule_reach(&cap, &other),
                        None => None,
                    };
                    let Some((_n, _dist, p_cap, p_other)) = reach else {
                        return;
                    };
                    (p_cap - p_other, p_other)
                };
                // 法线 a→b：初始符号不重要（同上：中心定号，别信见证点序）。
                let mut n_ab = n_raw;
                if n_ab.length_squared() < 1e-18 {
                    n_ab = pb - pa;
                }
                if n_ab.length_squared() < 1e-18 {
                    return;
                }
                let mut n_ab = n_ab.normalize();
                if n_ab.dot(pb - pa) < 0.0 {
                    n_ab = -n_ab;
                }
                // 压入量用见证点平面（理由同臂 1）。
                let plane = n_ab.dot(p_other);
                let mut local: Vec<ContactPoint> = Vec::with_capacity(2);
                for (i, s) in [pb - axis * half_height, pb + axis * half_height]
                    .into_iter()
                    .enumerate()
                {
                    // 该端帽压入量（n 由对方指向胶囊）：`plane − (n·端点 − radius)`。
                    let depth = plane - (n_ab.dot(s) - radius);
                    if depth > -self.skin {
                        local.push(ContactPoint {
                            // 接触点 = 朝向对方那侧的帽面（−n 侧）。
                            point: s - n_ab * radius,
                            depth,
                            feature: i as u32 + 1,
                        });
                    }
                }
                if local.is_empty() {
                    return;
                }
                self.cand.clear();
                self.cand.extend_from_slice(&local);
                if !self.select_contacts(self.min_point_sep) {
                    return;
                }
                out.push(Manifold {
                    a,
                    b,
                    normal: n_ab,
                    points: ContactPoints::from_slice(&self.cand),
                });
            }
            _ => {}
        }
    }
}

#[cfg(test)]
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
        np.collide(
            b,
            &pairs,
            hf,
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        out
    }

    /// 胶囊 × 盒地板（竖直）：接触判据 = **线段端点到地面的距离 < radius**（线段本身
    /// 并未碰到地面）；应给出 1 个接触点、法线向上、深度 ≈ 压入量。
    #[test]
    fn capsule_vs_box_floor_vertical() {
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::new(5.0, 0.5, 5.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        // 下帽端点 y = y0 − 0.4；下帽表面 = 端点 − 0.3 ⇒ 压住地面 1 cm 时 y0 = 0.69。
        b.push_dynamic(
            Shape::Capsule {
                half_height: 0.4,
                radius: 0.3,
            },
            Vec3::new(0.0, 0.69, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
        let out = manifolds_for(&b, &[]);
        assert_eq!(
            out.len(),
            1,
            "应有 1 个流形（胶囊 × 地板），实得 {}",
            out.len()
        );
        let m = &out[0];
        // 法线约定 **a→b**：期望符号由流形自身推导（别假定地板一定是 `a`——
        // 配对索引是 `(i, j)` 生成序，而本测试的 push 序不保证与之对应）。
        let floor_is_a = m.a == 0;
        let want = if floor_is_a { 1.0 } else { -1.0 };
        assert!(
            m.normal.y * want > 0.99,
            "法线应指向 a→b（地板→胶囊），a={} b={} 实得 {:?}",
            m.a,
            m.b,
            m.normal
        );
        assert_eq!(
            m.points.len(),
            1,
            "竖直胶囊只有下帽接触，实得 {} 点",
            m.points.len()
        );
        let d = m.points[0].depth;
        assert!(
            (d - 0.01).abs() < 2e-3,
            "深度应 ≈1 cm（0.3 − 端点距 0.29），实得 {d}"
        );
    }

    /// 胶囊 × 地形：此前高度场分支**显式拒绝**本组合 ⇒ 静默无接触。
    /// 现沿中心线取 5 个样本、每个按球处理。判据：有接触、法线竖直、压入 ≈1 cm。
    #[test]
    fn capsule_on_heightfield() {
        let mut b = BodySet::new();
        let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        // 竖直胶囊：下端点 y0 − 0.4、帽面再 −0.3 ⇒ 压入 1 cm 时 y0 = 0.69。
        b.push_dynamic(
            Shape::Capsule {
                half_height: 0.4,
                radius: 0.3,
            },
            Vec3::new(0.0, 0.69, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let m = manifolds_for(&b, &[hf]);
        assert!(!m.is_empty(), "竖直胶囊 × 地形应有接触（此前为静默无接触）");
        assert!(
            m[0].normal.y.abs() > 0.99,
            "法线应竖直，实得 {:?}",
            m[0].normal
        );
        let dmax = m[0].points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!((dmax - 0.01).abs() < 5e-3, "压入应 ≈1 cm，实得 {dmax}");
    }

    /// 外壳 × 地形：此前 `hull_pair` 明确不受理（"列裁剪对任意凸壳未实现"）⇒ 静默无接触。
    /// 现走**逐顶点采样**（与 `poly_heightfield` 同款）。判据：有接触、法线竖直、正压入。
    #[test]
    fn hull_on_heightfield() {
        let mut np = DefaultNarrowPhase::new(0.01);
        // 外壳：3×3×3 立方点云（半 0.3）。
        let mut pts: Vec<Vec3> = Vec::with_capacity(27);
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
                }
            }
        }
        let hid = np.add_hull(pts);
        let mut b = BodySet::new();
        let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        // 底面压入地面（y=0）1 cm ⇒ 中心 y = 0.29。
        b.push_dynamic(
            Shape::ConvexHull {
                hull: hid,
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 0.29, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(
            &b,
            &pairs,
            &[hf],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        assert!(!out.is_empty(), "外壳 × 地形应有接触（此前为静默无接触）");
        let m = &out[0];
        assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
        let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
    }

    /// 外壳 × 圆柱：曾因 `support_of` 缺圆柱/锥分支而**静默无接触**（`TECH-SURVEY.md` A9 ④ 留档）。
    /// 判据：有接触、法线竖直、有正压入。
    #[test]
    fn hull_on_cylinder_cap() {
        let mut np = DefaultNarrowPhase::new(0.01);
        // 外壳：3×3×3 立方点云（半 0.3）。
        let mut pts: Vec<Vec3> = Vec::with_capacity(27);
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
                }
            }
        }
        let hid = np.add_hull(pts);
        let mut b = BodySet::new();
        // 静立圆柱（半高 0.5、半径 0.4，中心 y=-0.5 ⇒ 顶面 y=0）。
        b.push_static(
            Shape::Cylinder {
                half_height: 0.5,
                radius: 0.4,
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        // 外壳落在顶面（底面压入 1 cm ⇒ 中心 y = 0.29）。
        b.push_dynamic(
            Shape::ConvexHull {
                hull: hid,
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 0.29, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(
            &b,
            &pairs,
            &[],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        assert!(!out.is_empty(), "外壳 × 圆柱应有接触（此前为静默无接触）");
        let m = &out[0];
        assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
        let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
    }

    /// 圆锥 × 盒地板（坐底）：底圆盘多面化 ⇒ 应给出**多点**支撑（单点会晃）、法线竖直、
    /// 最深压入 ≈ 压入量。
    #[test]
    fn cone_on_box_floor() {
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::new(5.0, 0.5, 5.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        // 底面在 y0 − h；要让底圆盘压入地面（y=0）1 cm ⇒ y0 = 0.5 − 0.01 = 0.49。
        b.push_dynamic(
            Shape::Cone {
                half_height: 0.5,
                radius: 0.4,
            },
            Vec3::new(0.0, 0.49, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
        let out = manifolds_for(&b, &[]);
        assert!(!out.is_empty(), "圆锥坐底应有接触");
        let m = &out[0];
        let floor_is_a = m.a == 0;
        let want = if floor_is_a { 1.0 } else { -1.0 };
        assert!(
            m.normal.y * want > 0.99,
            "法线应竖直（a→b），实得 {:?}",
            m.normal
        );
        let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!((dmax - 0.01).abs() < 3e-3, "最深压入应 ≈1 cm，实得 {dmax}");
        assert!(
            m.points.len() >= 3,
            "坐底应为多点支撑（多点才不摇），实得 {} 点",
            m.points.len()
        );
    }

    /// 复合体（哑铃：两端盒 + 中间横杆）坐地：**每个接触的子形状各出一条流形**（≥2 条），
    /// 且特征号按**子序号左移 16 位**编码 ⇒ 不同子形状的特征空间互不重叠（暖缓存不串号）。
    #[test]
    fn compound_dumbbell_on_floor() {
        let mut np = DefaultNarrowPhase::new(0.01);
        // 两端用**盒**（而非球）：盒-盒接触带 `feature`，才能验到"子序号并入特征号"这条路径
        // （球接触的 `feature` 恒为 0 = "无特征"哨兵，标记不碰它）。
        let cid = np.add_compound(vec![
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::splat(0.3),
                },
                offset: Vec3::new(-0.6, 0.0, 0.0),
                rot: Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::splat(0.3),
                },
                offset: Vec3::new(0.6, 0.0, 0.0),
                rot: Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::splat(0.3),
                },
                offset: Vec3::new(0.6, 0.0, 0.0),
                rot: Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::new(0.6, 0.1, 0.1),
                },
                offset: Vec3::ZERO,
                rot: Quat::IDENTITY,
            },
        ]);
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::new(5.0, 0.5, 5.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        // 两球半径 0.3、横杆 1.2×0.2×0.2；球压入地面（y=0）1 cm ⇒ y0 = 0.29。
        b.push_dynamic(
            Shape::Compound {
                compound: cid,
                half: np.compound_half_extents(cid),
            },
            Vec3::new(0.0, 0.29, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(
            &b,
            &pairs,
            &[],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        assert!(
            out.len() >= 2,
            "两个球应各出一条流形（同体对多条），实得 {} 条",
            out.len()
        );
        let mut tags = std::collections::BTreeSet::new();
        for m in &out {
            assert!(
                m.normal.y.abs() > 0.99,
                "地面接触法线应竖直，实得 {:?}",
                m.normal
            );
            let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
            assert!((dmax - 0.01).abs() < 3e-3, "压入应 ≈1 cm，实得 {dmax}");
            for p in m.points.iter() {
                assert!(
                    p.feature >> 16 != 0,
                    "特征号应带子序号标记，实得 {}",
                    p.feature
                );
                tags.insert(p.feature >> 16);
            }
        }
        assert!(
            tags.len() >= 2,
            "不同子形状的特征空间应互不相同，实得 {tags:?}"
        );
    }

    /// 平躺胶囊：应给出**两个**接触点（两帽各一）——这是它稳定静置（不摇）的前提。
    #[test]
    fn capsule_flat_gives_two_points() {
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::new(5.0, 0.5, 5.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        // 绕 Z 转 90° ⇒ 局部 +Y 变成世界 +X ⇒ 胶囊水平平躺，压入 1 cm（0.3 − 0.29）。
        let rot = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
        b.push_dynamic(
            Shape::Capsule {
                half_height: 0.4,
                radius: 0.3,
            },
            Vec3::new(0.0, 0.29, 0.0),
            rot,
            1000.0,
        );
        let out = manifolds_for(&b, &[]);
        assert_eq!(out.len(), 1, "应有 1 个流形，实得 {}", out.len());
        assert_eq!(
            out[0].points.len(),
            2,
            "平躺胶囊应给 2 点支撑，实得 {}",
            out[0].points.len()
        );
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

    /// 复合体 × 地形：子形状各自走地形路径（此前该组合在高度场分支被**显式拒绝**）。
    /// 判据：两个球子形状各给出一条流形、法线竖直（含接触）。
    #[test]
    fn compound_on_heightfield() {
        let mut np = DefaultNarrowPhase::new(0.01);
        let cid = np.add_compound(vec![
            CompoundChild {
                shape: Shape::Sphere { radius: 0.3 },
                offset: Vec3::new(-0.6, 0.0, 0.0),
                rot: Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Sphere { radius: 0.3 },
                offset: Vec3::new(0.6, 0.0, 0.0),
                rot: Quat::IDENTITY,
            },
        ]);
        let mut b = BodySet::new();
        let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        b.push_dynamic(
            Shape::Compound {
                compound: cid,
                half: np.compound_half_extents(cid),
            },
            Vec3::new(0.0, 0.29, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(
            &b,
            &pairs,
            &[hf],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        assert!(
            out.len() >= 2,
            "两个球子形状应各出一条地形流形，实得 {} 条",
            out.len()
        );
        for m in &out {
            assert!(
                m.normal.y.abs() > 0.99,
                "地形法线应竖直，实得 {:?}",
                m.normal
            );
            // 球子形状的特征号恒为 0 ⇒ 子序号标记必须对**全部**点位生效，否则两个子形状
            // 的接触点在同一个体对（暖启动缓存键）上无法区分。
            for p in m.points.iter() {
                assert!(
                    p.feature >> 16 != 0,
                    "地形接触也应带子序号标记，实得 {}",
                    p.feature
                );
            }
        }
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

    /// 内边（折痕）幽灵接触判据：盒正中骑在「平地面 / 斜坡」的折痕上时，
    /// 采样法线**不许是两块面法线的混合**——混合即内边假接触（幽灵推力）。
    /// 高度场路径取「最深点采样法线」作整条流形法线，折痕处的双线性采样
    /// 天然会把两块面混在一起，故这里是该缺陷的天然复现位。
    #[test]
    fn heightfield_crease_normal_is_not_blended() {
        // ix≤5 平（h=0），ix>5 沿 +x 抬升 0.5/格 ⇒ 折痕在 x=0（spacing=1，x0=-5）。
        let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
        for iz in 0..11 {
            for ix in 0..11 {
                hf.set_height(ix, iz, (ix as f32 - 5.0).max(0.0) * 0.5);
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
        let _ = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
        let m = manifolds_for(&b, &[hf]);
        assert_eq!(m.len(), 1);
        let n = m[0].normal; // 约定：a=盒 → b=地面（指向地面者为主）
        let floor = Vec3::new(0.0, -1.0, 0.0);
        let ramp = -Vec3::new(-0.5, 1.0, 0.0).normalize();
        let d_floor = n.dot(floor);
        let d_ramp = n.dot(ramp);
        assert!(n.z.abs() < 1e-3, "折痕法线出现 z 分量（邻格串扰）：{n:?}");
        assert!(
            d_floor.max(d_ramp) > 0.995,
            "折痕法线是两块面法线的混合 ⇒ 内边幽灵接触：{n:?}（floor 对齐 {d_floor}，ramp 对齐 {d_ramp}）"
        );
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
