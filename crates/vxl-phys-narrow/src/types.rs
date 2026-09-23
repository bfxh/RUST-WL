//! types：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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
    pub(crate) buf: [ContactPoint; 4],
    pub(crate) len: u8,
    /// **特征空间**：复合体子形状序号 + 1（非复合体恒 0）。求解器用它区分**同一体对**上的
    /// 多条流形（复合体每个子形状一条）。**不借 `feature` 的任何位**——窄相特征号的高位被
    /// "侧别/裁剪路/哈希"编码占用，哈希逐帧漂移 ⇒ 借用会让普通场景暖启动随机失效（实测见
    /// `TECH-SURVEY.md` A9 ④ 的那次回退）。
    pub(crate) space: u16,
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
pub(crate) enum AxisSrc {
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
pub(crate) type BoxSign = (i8, i8, i8);
/// provider 对的「接近面」判据下限（m/s）：闭合速度超过它才按接近方向选面，
/// 否则退回"点数最多、并列取最深"（静置/无明显接近时的原规则）。
pub(crate) const CLOSING_MIN: f32 = 0.1;

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

pub(crate) const BOX_FACES: [((i8, i8, i8), [BoxSign; 4]); 6] = [
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
pub(crate) fn box_axes(rot: Quat) -> [Vec3; 3] {
    [
        rot.rotate_vec3(Vec3::X),
        rot.rotate_vec3(Vec3::Y),
        rot.rotate_vec3(Vec3::Z),
    ]
}

/// 姿态指纹（盒对轴缓存的键）：4×f32 位模式左旋混合——比 3 次四元数旋转廉价得多，
/// 且键含体号 ⇒ 不同体不会互撞；同体同姿态必然同值 ⇒ **逐位透明**。
#[inline]
pub(crate) fn rot_fp(rot: Quat) -> u64 {
    let mut h = rot.x.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.y.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.z.to_bits() as u64;
    h = h.rotate_left(13) ^ rot.w.to_bits() as u64;
    h
}

/// 面法线：轴分量带符号（±v 逐位精确，`rotate(-v) == -rotate(v)` 在
/// 双叉积形式下成立）。
#[inline]
pub(crate) fn face_normal_of(axes: &[Vec3; 3], n: (i8, i8, i8)) -> Vec3 {
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
pub(crate) fn face_vertex(pos: Vec3, axes: &[Vec3; 3], half: Vec3, sg: BoxSign) -> Vec3 {
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
    pub(crate) fn fill(&mut self, poly: &ConvexPolytope, pos: Vec3, rot: Quat) {
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
