//! grid_phase：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) type CellKey = (i32, i32, i32);

/// 均匀空间哈希网格宽相。
pub struct GridBroadPhase {
    /// 网格边长（m）；应 ≥ 最大物体直径。
    pub cell_size: f32,
    pub(crate) skin: f32,
    pub(crate) cells_static: HashMap<CellKey, Vec<u32>>,
    pub(crate) cells_dynamic: HashMap<CellKey, Vec<u32>>,
    pub(crate) aabbs: Vec<Aabb>,
    pub(crate) pairs: Vec<(u32, u32)>,
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
    pub(crate) fn cell_of(&self, v: Vec3) -> CellKey {
        const LIMIT: f32 = 1_000_000.0;
        let inv = 1.0 / self.cell_size;
        let x = (v.x * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let y = (v.y * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        let z = (v.z * inv).floor().clamp(-LIMIT, LIMIT) as i32;
        (x, y, z)
    }

    pub(crate) fn insert(
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
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        provider_bounds: &[Aabb],
        _jobs: &dyn JobSystem,
    ) -> &[(u32, u32)] {
        // 网格宽相对外部 provider 体无特化；仅保证签名一致（其 AABB 走
        // `shape_aabb` 的 Provider 分支，由 provider_bounds 供给）。
        let _ = provider_bounds;
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
                bodies.rot(i),
                self.skin,
                hf_bounds,
                provider_bounds,
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
