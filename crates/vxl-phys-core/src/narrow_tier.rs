//! 窄相**卡上档**的接口契约（`PLAN-gpu.md` §17.9）。
//!
//! **为什么落在 core**（与 `interop::FluidStepper` 同款）：两端都已依赖 core ⇒ **不动任何依赖边**。
//! 而窄相的物理类型（`Manifold`）在 `vxl-phys-narrow`，core **不能**依赖它（分层反了）⇒ 这里只过
//! **裸字**：主机打包好的逐体/逐对表进、**对序**的槽字表出；由**门面**（`vxl-phys`）解成流形、
//! 做主机回填与装配。⇒ 卡上档不引入任何几何知识到 core。
//!
//! **布局三处同源**：本文件的常量 = `narrow.wgsl` 的布局 = `vxl-phys-gpu::narrow` 的常量
//! （后者直接 `pub use` 本文件）。改一处要同步另两处。

use crate::{BodySet, Shape};

/// 槽字数（u32）：`a|b|normal.xyz|count` + 4×`(point.xyz|depth|feature)` = 6 + 20。
pub const SLOT_WORDS: usize = 26;
/// 槽字节数（104 B/对）。
pub const SLOT_BYTES: usize = SLOT_WORDS * 4;
/// 逐体输入字数（48 B/体）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`。
pub const BODY_WORDS: usize = 12;
/// `count` 哨兵：卡上**不接手**该对 ⇒ 调用方按主机回填处理（别当成"无流形"）。
pub const NOT_HANDLED: u32 = u32::MAX;
/// 体种类：0 = 不接手；1 = 球（`p0` = 半径）；2 = 盒（`p0..p2` = `half.xyz`）。
pub const KIND_NONE: u32 = 0;
pub const KIND_SPHERE: u32 = 1;
pub const KIND_BOX: u32 = 2;

/// 一趟读数：**对序**的槽字表（长度 = 对数 × `SLOT_WORDS`）+ 诊断字。
pub struct NarrowSlots {
    pub words: Vec<u32>,
    /// `[0]` = 裁剪多边形越界次数（**必须 0**；>0 表示核与主机的槽布局漂移或护栏触发）。
    pub diag: [u32; 2],
}

impl NarrowSlots {
    #[inline]
    fn f(&self, w: usize) -> f32 {
        f32::from_bits(self.words[w])
    }

    /// 第 `i` 个槽的点数：0 = 无流形、1..=4 = 点数、`NOT_HANDLED` = 卡上不接手。
    #[inline]
    pub fn count(&self, i: usize) -> u32 {
        self.words[i * SLOT_WORDS + 5]
    }

    /// 第 `i` 个槽的两个体号。
    #[inline]
    pub fn pair(&self, i: usize) -> (u32, u32) {
        let o = i * SLOT_WORDS;
        (self.words[o], self.words[o + 1])
    }

    /// 第 `i` 个槽的流形法线（a→b）。
    #[inline]
    pub fn normal(&self, i: usize) -> [f32; 3] {
        let o = i * SLOT_WORDS + 2;
        [self.f(o), self.f(o + 1), self.f(o + 2)]
    }

    /// 第 `i` 个槽的第 `k` 个接触点 = `(point, depth, feature)`。
    #[inline]
    pub fn point(&self, i: usize, k: usize) -> ([f32; 3], f32, u32) {
        let o = i * SLOT_WORDS + 6 + k * 5;
        (
            [self.f(o), self.f(o + 1), self.f(o + 2)],
            self.f(o + 3),
            self.words[o + 4],
        )
    }
}

/// **卡上窄相后端**：吃"主机打包好的逐体表/对表"，回**对序**的槽字表。
/// 失败（无适配器 / 超容量 / 核跑不通）⇒ `Err`，调用方**回退 CPU**（不得静默用半张表）。
pub trait NarrowTierBackend {
    fn narrow_run(&self, bodies: &[u32], pairs: &[u32]) -> Result<NarrowSlots, String>;
}

/// 逐体打包为卡上布局：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`（浮点走 f32 位模式、
/// `kind` 是**裸整数**）。**主机唯一一份**（门面与探针都调它，免得布局在多处各写一遍）。
pub fn pack_bodies(bodies: &BodySet) -> Vec<u32> {
    let mut out = Vec::with_capacity(bodies.len() * BODY_WORDS);
    for i in 0..bodies.len() {
        let p = bodies.position[i];
        let r = bodies.rot(i);
        out.extend_from_slice(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        out.extend_from_slice(&[r.x.to_bits(), r.y.to_bits(), r.z.to_bits(), r.w.to_bits()]);
        // 卡上已接的族 = 球×球、球×盒、盒×盒；其余（圆柱/圆锥/外壳/胶囊/高度场/provider/复合体）
        // 一律 `KIND_NONE` ⇒ 由主机回填。
        let (kind, params) = match bodies.shape[i] {
            Shape::Sphere { radius } => (KIND_SPHERE, [radius, 0.0, 0.0]),
            Shape::Box { half } => (KIND_BOX, [half.x, half.y, half.z]),
            _ => (KIND_NONE, [0.0; 3]),
        };
        out.push(kind);
        for v in params {
            out.push(v.to_bits());
        }
        out.push(0); // pad
    }
    out
}

/// 对表 → 卡上布局（2 字/对）。
pub fn flat_pairs(pairs: &[(u32, u32)]) -> Vec<u32> {
    pairs.iter().flat_map(|&(a, b)| [a, b]).collect()
}
