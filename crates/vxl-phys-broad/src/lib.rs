//! # vxl-phys-broad
//!
//! 宽相（§2.3）：「增量 AABB 树（bvh2 风格）为主 + 大世界空间哈希网格」。
//! M1 落地 `BvhBroadPhase`（增量动态 BVH，`bvh::DynamicBvh`）为主实现；
//! M0 的 `GridBroadPhase`（均匀空间哈希网格）保留作对照/大世界叠加。
//!
//! 确定性（§5）：树插入/移动按体索引升序；查询显式栈、先左后右；
//! 输出对 `(a, b)`（a < b）按字典序排序去重。两者绝不迭代哈希结构本身。

#![forbid(unsafe_code)]

pub mod bvh;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, Quat, Shape, Vec3};

pub use bvh::DynamicBvh;

/// 轴对齐包围盒。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    #[inline]
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }
}

/// 形状 → 世界 AABB（含 margin 膨胀）。
/// 高度场体的包围盒由调用方经 `hf_bounds[id]` 提供（高度场数据在 narrow/terrain 层）。
pub fn shape_aabb(shape: &Shape, pos: Vec3, rot: Quat, margin: f32, hf_bounds: &[Aabb]) -> Aabb {
    let half = match *shape {
        Shape::Box { half } => {
            // 精确：|R|·half（旋转矩阵逐元素绝对值作用于半长）。
            let r = vxl_phys_core::Mat3::from_quat(rot);
            Vec3::new(
                r.m[0][0].abs() * half.x + r.m[0][1].abs() * half.y + r.m[0][2].abs() * half.z,
                r.m[1][0].abs() * half.x + r.m[1][1].abs() * half.y + r.m[1][2].abs() * half.z,
                r.m[2][0].abs() * half.x + r.m[2][1].abs() * half.y + r.m[2][2].abs() * half.z,
            )
        }
        Shape::Sphere { radius } => Vec3::splat(radius),
        // 圆柱 ⊆ 外接盒（r, hh, r），经 |R| 变换保守。
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let r = vxl_phys_core::Mat3::from_quat(rot);
            let ext = Vec3::new(radius, half_height, radius);
            Vec3::new(
                r.m[0][0].abs() * ext.x + r.m[0][1].abs() * ext.y + r.m[0][2].abs() * ext.z,
                r.m[1][0].abs() * ext.x + r.m[1][1].abs() * ext.y + r.m[1][2].abs() * ext.z,
                r.m[2][0].abs() * ext.x + r.m[2][1].abs() * ext.y + r.m[2][2].abs() * ext.z,
            )
        }
        Shape::HeightField(id) => {
            let b = hf_bounds.get(id as usize).copied().unwrap_or(Aabb {
                min: Vec3::splat(0.0),
                max: Vec3::splat(0.0),
            });
            return Aabb {
                min: b.min - Vec3::splat(margin),
                max: b.max + Vec3::splat(margin),
            };
        }
    };
    Aabb {
        min: pos - half - Vec3::splat(margin),
        max: pos + half + Vec3::splat(margin),
    }
}

pub trait BroadPhase {
    /// 输出确定性有序对（a < b，字典序升序，去重）。
    fn compute_pairs(&mut self, bodies: &BodySet, hf_bounds: &[Aabb]) -> &[(u32, u32)];

    /// 任意 AABB 命中查询（CCD 扫掠区域用，§4.12）；输出体 id 升序去重。
    /// 默认空实现（不需要该能力的宽相可直接忽略）。
    fn query_aabb(&mut self, _aabb: &Aabb, _out: &mut Vec<u32>) {}
}

type CellKey = (i32, i32, i32);

/// 均匀空间哈希网格宽相。
pub struct GridBroadPhase {
    /// 网格边长（m）；应 ≥ 最大物体直径。
    pub cell_size: f32,
    skin: f32,
    cells_static: HashMap<CellKey, Vec<u32>>,
    cells_dynamic: HashMap<CellKey, Vec<u32>>,
    aabbs: Vec<Aabb>,
    pairs: Vec<(u32, u32)>,
}

impl GridBroadPhase {
    pub fn new(cell_size: f32, skin: f32) -> Self {
        Self {
            cell_size: if cell_size > 1e-4 { cell_size } else { 1.0 },
            skin,
            cells_static: HashMap::new(),
            cells_dynamic: HashMap::new(),
            aabbs: Vec::new(),
            pairs: Vec::new(),
        }
    }

    #[inline]
    fn cell_of(&self, v: Vec3) -> CellKey {
        const LIMIT: f32 = 1_000_000.0;
        let inv = 1.0 / self.cell_size;
        let x = (v.x * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let y = (v.y * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let z = (v.z * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        (x, y, z)
    }

    fn insert(
        cells: &mut HashMap<CellKey, Vec<u32>>,
        aabb: &Aabb,
        cell_min: CellKey,
        cell_max: CellKey,
        id: u32,
    ) {
        let mut x = cell_min.0;
        while x <= cell_max.0 {
            let mut y = cell_min.1;
            while y <= cell_max.1 {
                let mut z = cell_min.2;
                while z <= cell_max.2 {
                    cells.entry((x, y, z)).or_default().push(id);
                    z += 1;
                }
                y += 1;
            }
            x += 1;
        }
        let _ = aabb;
    }
}

impl BroadPhase for GridBroadPhase {
    fn compute_pairs(&mut self, bodies: &BodySet, hf_bounds: &[Aabb]) -> &[(u32, u32)] {
        self.cells_static.clear();
        self.cells_dynamic.clear();
        self.pairs.clear();

        let n = bodies.len();
        self.aabbs.clear();
        self.aabbs.reserve(n);
        for i in 0..n {
            let aabb = shape_aabb(
                &bodies.shape[i],
                bodies.position[i],
                bodies.rotation[i],
                self.skin,
                hf_bounds,
            );
            self.aabbs.push(aabb);
        }

        // 静态：仅插入。
        for i in 0..n {
            if bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            Self::insert(&mut self.cells_static, &self.aabbs[i], cmin, cmax, i as u32);
        }
        // 动态：先全部插入（dyn-dyn 去重靠 j > i），再统一查询。
        for i in 0..n {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            Self::insert(
                &mut self.cells_dynamic,
                &self.aabbs[i],
                cmin,
                cmax,
                i as u32,
            );
        }
        // 查询。
        for i in 0..n {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let (cmin, cmax) = (
                self.cell_of(self.aabbs[i].min),
                self.cell_of(self.aabbs[i].max),
            );
            let mut x = cmin.0;
            while x <= cmax.0 {
                let mut y = cmin.1;
                while y <= cmax.1 {
                    let mut z = cmin.2;
                    while z <= cmax.2 {
                        let key = (x, y, z);
                        if let Some(list) = self.cells_static.get(&key) {
                            for &j in list {
                                // 输出约定：对内 a < b（下游窄相/求解器的法线方向依赖它）。
                                let (a, b) = if (i as u32) < j {
                                    (i as u32, j)
                                } else {
                                    (j, i as u32)
                                };
                                if self.aabbs[a as usize].overlaps(&self.aabbs[b as usize]) {
                                    self.pairs.push((a, b));
                                }
                            }
                        }
                        if let Some(list) = self.cells_dynamic.get(&key) {
                            for &j in list {
                                if j as usize > i && self.aabbs[i].overlaps(&self.aabbs[j as usize])
                                {
                                    self.pairs.push((i as u32, j));
                                }
                            }
                        }
                        z += 1;
                    }
                    y += 1;
                }
                x += 1;
            }
        }

        self.pairs.sort_unstable();
        self.pairs.dedup();
        &self.pairs
    }

    fn query_aabb(&mut self, aabb: &Aabb, out: &mut Vec<u32>) {
        out.clear();
        for cells in [&self.cells_static, &self.cells_dynamic] {
            for list in cells.values() {
                for &j in list {
                    if self.aabbs.get(j as usize).is_some_and(|b| b.overlaps(aabb)) {
                        out.push(j);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

/// 增量 BVH 宽相（§2.3 主路径）。
///
/// - 代理更新按体索引升序（确定性）；树跨帧保持（增量 refit/平衡）；
/// - fat margin = 2×skin（≥ 2cm），静置帧零树操作；
/// - M1 不提供删体 API（`World` 无删体），叶子与体索引一一对应。
pub struct BvhBroadPhase {
    tree: DynamicBvh,
    skin: f32,
    leaves: Vec<u32>,
    /// 精确 AABB（含 skin 膨胀），与树内 fat AABB 分离。
    aabbs: Vec<Aabb>,
    query_buf: Vec<u32>,
    pairs: Vec<(u32, u32)>,
}

impl BvhBroadPhase {
    pub fn new(skin: f32) -> Self {
        Self {
            tree: DynamicBvh::new((skin * 2.0).max(0.02)),
            skin,
            leaves: Vec::new(),
            aabbs: Vec::new(),
            query_buf: Vec::new(),
            pairs: Vec::new(),
        }
    }

    /// 树高（诊断/负载审计：健康树 ≈ 1.4·log2(n)）。
    pub fn tree_height(&self) -> u32 {
        self.tree.root_height()
    }
}

impl BroadPhase for BvhBroadPhase {
    fn compute_pairs(&mut self, bodies: &BodySet, hf_bounds: &[Aabb]) -> &[(u32, u32)] {
        self.pairs.clear();
        let n = bodies.len();
        self.aabbs.clear();
        self.aabbs.reserve(n);
        for i in 0..n {
            let aabb = shape_aabb(
                &bodies.shape[i],
                bodies.position[i],
                bodies.rotation[i],
                self.skin,
                hf_bounds,
            );
            self.aabbs.push(aabb);
        }
        // 1) 代理更新（首帧大场景 = 中位分裂全量重建；增量后链化超限再重建）。
        //    面积启发式对网格行优先等结构化插入序会链化（树高 ≈ n/2），
        //    阈值 = 3·log2(n) + 16（确定性纯函数，不依赖时序）。
        let rebuild_due = n >= 32 && {
            let limit = 3.0 * (n as f32).log2() + 16.0;
            self.tree.root_height() as f32 > limit
        };
        if n >= 32 && (self.leaves.is_empty() || rebuild_due) {
            let items: Vec<(u32, Aabb)> = (0..n).map(|i| (i as u32, self.aabbs[i])).collect();
            self.leaves = self.tree.rebuild(&items);
        } else {
            for i in 0..n {
                if (i as u32) < self.leaves.len() as u32 {
                    self.leaves[i] = self.tree.move_proxy(self.leaves[i], self.aabbs[i]);
                } else {
                    self.leaves.push(self.tree.insert(i as u32, self.aabbs[i]));
                }
            }
        }
        // 2) 动体查询（dyn-dyn 靠 j > i 去重；dyn-static 只由动体侧发起）。
        for i in 0..n {
            if !bodies.is_dynamic(i) {
                continue;
            }
            self.tree.query(&self.aabbs[i], &mut self.query_buf);
            for &j in &self.query_buf {
                let j = j as usize;
                if j == i || (bodies.is_dynamic(j) && j <= i) {
                    continue;
                }
                if self.aabbs[i].overlaps(&self.aabbs[j]) {
                    let (a, b) = if i < j {
                        (i as u32, j as u32)
                    } else {
                        (j as u32, i as u32)
                    };
                    self.pairs.push((a, b));
                }
            }
        }
        self.pairs.sort_unstable();
        self.pairs.dedup();
        &self.pairs
    }

    fn query_aabb(&mut self, aabb: &Aabb, out: &mut Vec<u32>) {
        self.tree.query(aabb, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> BodySet {
        BodySet::new()
    }

    /// BVH 与网格宽相在同场景下输出完全一致（确定性交叉验证）。
    #[test]
    fn bvh_matches_grid_across_frames() {
        let build = || {
            let mut b = world();
            // 地面静态瓦片 5×5。
            for gx in -2i32..=2 {
                for gz in -2i32..=2 {
                    b.push_static(
                        Shape::Box {
                            half: Vec3::new(0.5, 0.25, 0.5),
                        },
                        Vec3::new(gx as f32, -0.25, gz as f32),
                        Quat::IDENTITY,
                    );
                }
            }
            // 动体：确定性伪随机散布。
            for k in 0..40u32 {
                let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let z = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 8.0 - 4.0;
                let y = 0.5 + ((k.wrapping_mul(97)) % 400) as f32 / 100.0;
                let shape = if k % 2 == 0 {
                    Shape::Sphere { radius: 0.3 }
                } else {
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    }
                };
                b.push_dynamic(shape, Vec3::new(x, y, z), Quat::IDENTITY, 1.0);
            }
            b
        };
        let mut grid = GridBroadPhase::new(2.0, 0.01);
        let mut bvh = BvhBroadPhase::new(0.01);
        for frame in 0..5u32 {
            let mut b = build();
            // 每帧给动体一个确定性位移（模拟增量移动）。
            for i in 0..b.len() {
                if b.is_dynamic(i) {
                    b.position[i].x += frame as f32 * 0.17;
                    b.position[i].y -= frame as f32 * 0.09;
                }
            }
            let p_grid = grid.compute_pairs(&b, &[]).to_vec();
            let p_bvh = bvh.compute_pairs(&b, &[]).to_vec();
            assert_eq!(p_grid, p_bvh, "frame {frame}");
            let _ = &mut b;
        }
    }

    #[test]
    fn overlapping_pair_found_once() {
        let mut b = world();
        let g = b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let d = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let far = b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(50.0, 0.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let pairs = bp.compute_pairs(&b, &[]);
        assert_eq!(pairs, &[(d.min(g), d.max(g))]);
        assert!(pairs.iter().all(|&p| p.1 != far && p.0 != far));
    }

    #[test]
    fn no_static_static_pairs() {
        let mut b = world();
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.1, 0.0, 0.0),
            Quat::IDENTITY,
        );
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        assert!(bp.compute_pairs(&b, &[]).is_empty());
    }

    #[test]
    fn deterministic_output() {
        let mut b = world();
        for k in 0..50 {
            let x = (k % 10) as f32 * 1.1;
            let z = (k / 10) as f32 * 1.1;
            b.push_dynamic(
                Shape::Sphere { radius: 0.5 },
                Vec3::new(x, 1.0, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        let mut bp = GridBroadPhase::new(2.0, 0.01);
        let p1 = bp.compute_pairs(&b, &[]).to_vec();
        let p2 = bp.compute_pairs(&b, &[]).to_vec();
        assert_eq!(p1, p2);
    }
}
