//! store：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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
    pub(crate) items: Vec<Vec<CompoundChild>>,
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
pub(crate) fn tag_child_features(ms: &mut [Manifold], ci: usize) {
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
    pub(crate) hulls: Vec<gjk::ConvexHull>,
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
