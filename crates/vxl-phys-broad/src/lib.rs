//! # vxl-phys-broad
//!
//! 宽相（§2.3）：目标是「增量 AABB 树（bvh2 风格）为主 + 大世界空间哈希网格」。
//! M0 落地均匀空间哈希网格（1 万静态 + 1 千动态达标），BVH 在 M1 接入
//! （`BroadPhase` trait 不变，替换实现即可）。
//!
//! 确定性（§5）：网格只做 lookup/insert（按体索引遍历），绝不迭代哈希桶本身；
//! 输出对 `(a, b)`（a < b）按字典序排序去重。

#![forbid(unsafe_code)]

use std::collections::HashMap;

use vxl_phys_core::{BodySet, Quat, Shape, Vec3};

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
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> BodySet {
        BodySet::new()
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
