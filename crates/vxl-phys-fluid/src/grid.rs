//! grid：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 均匀网格：格边 = h（或粗化后），27 邻域。
/// **格内序 = 登记序 = 粒子索引序** —— 确定性求和的根。
///
/// **存储 = 计数排序**（2026-09-23，规模档第二刀）：`items` 是"按格分组、格内按索引序"的
/// 紧凑 `u32` 数组，`start` 是每格起点（长度 total+1）。相对旧的 `Vec<Vec<u32>>`：
/// ① 免掉每格一次 `Vec` 的分配/增长/清空（30 万粒档实测重建 25.8 ms ⇒ 换后见 §6.2）；
/// ② 邻居遍历变成**连续切片扫描**（旧版是 27 条指针链各自追堆块）⇒ 缓存友好。
/// **逐位一致**：散射按粒子**索引升序**写入 ⇒ 每格内序与旧实现（同样索引序 push）**相同**。
#[derive(Default)]
pub(crate) struct UniformGrid {
    pub(crate) bin: f32,
    pub(crate) inv: f32,
    pub(crate) min: Vec3,
    pub(crate) nx: u32,
    pub(crate) ny: u32,
    pub(crate) nz: u32,
    /// 按格分组的粒子索引（格内索引升序）。
    pub(crate) items: Vec<u32>,
    /// 每格起点（`start[c]..start[c+1]` = 格 c 的粒子）；长度 = 格数 + 1。
    pub(crate) start: Vec<u32>,
    /// 计数 scratch（复用，免每帧分配）。
    pub(crate) counts: Vec<u32>,
}

impl UniformGrid {
    pub(crate) fn rebuild(&mut self, pos: &[Vec3], h: f32) {
        let n = pos.len();
        if n == 0 {
            self.nx = 0;
            self.ny = 0;
            self.nz = 0;
            self.items.clear();
            self.start.clear();
            self.start.push(0);
            return;
        }
        let mut lo = pos[0];
        let mut hi = pos[0];
        for p in &pos[1..] {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        // 箱子三件套走**单一来源**（`vxl_phys_core::grid::grid_box`）：GPU 常驻管线用同一条
        // 规则，保证两侧 `(min, bin, dims)` 逐位相同（规则的注释与边界用例在同名模块）。
        let (min, bin, dims) = vxl_phys_core::grid::grid_box(lo, hi, h, GRID_MAX_BINS);
        self.bin = bin;
        self.inv = 1.0 / bin;
        self.min = min;
        self.nx = dims[0];
        self.ny = dims[1];
        self.nz = dims[2];
        let total = self.nx as usize * self.ny as usize * self.nz as usize;
        // —— 计数排序（两遍扫 + 前缀和；无每格分配）——
        self.counts.clear();
        self.counts.resize(total + 1, 0);
        for p in pos {
            let c = self.bin_index_of(*p, total);
            self.counts[c + 1] += 1;
        }
        for c in 0..total {
            self.counts[c + 1] += self.counts[c];
        }
        self.start.clear();
        self.start.extend_from_slice(&self.counts);
        self.items.clear();
        self.items.resize(n, 0);
        for (i, p) in pos.iter().enumerate() {
            let c = self.bin_index_of(*p, total);
            let slot = self.counts[c] as usize;
            self.items[slot] = i as u32;
            self.counts[c] += 1; // 游标：索引升序写入 ⇒ 格内序不变
        }
    }

    /// 粒子所在格的**线性下标**（钳到网格内；`total` 由调用方给以省一次乘）。
    #[inline]
    pub(crate) fn bin_index_of(&self, p: Vec3, total: usize) -> usize {
        let f = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * self.inv).floor().max(0.0) as u32).min(n - 1)
        };
        let idx = (f(self.min.x, p.x, self.nx) * self.ny + f(self.min.y, p.y, self.ny)) * self.nz
            + f(self.min.z, p.z, self.nz);
        (idx as usize).min(total.saturating_sub(1))
    }

    /// 粒子所在格坐标（钳到网格内）。
    #[inline]
    pub(crate) fn bin_of(&self, p: Vec3) -> (u32, u32, u32) {
        let f = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * self.inv).floor().max(0.0) as u32).min(n - 1)
        };
        (
            f(self.min.x, p.x, self.nx),
            f(self.min.y, p.y, self.ny),
            f(self.min.z, p.z, self.nz),
        )
    }

    /// 格 `c` 的粒子切片（格内索引升序）。
    #[inline]
    pub(crate) fn bin_items(&self, c: usize) -> &[u32] {
        let a = self.start[c] as usize;
        let b = self.start[c + 1] as usize;
        &self.items[a..b]
    }

    /// 邻域公共体（**自由函数形态**，供并行相位在分块闭包里调用）：粒子 `i` 的
    /// 27 邻域格（钳边）内、`r ≤ h` 的 `j` 交给 `f`。访问序 = 格坐标序
    /// （dz, dy, dx 固定）× 格内索引序 ⇒ 求和序是位置的确定函数
    /// ⇒ **并行分块不改变任一粒子的求和序**（这就是"并行 = 串行逐位一致"的根据）。
    #[inline]
    pub(crate) fn for_neighbors_in(
        &self,
        pos: &[Vec3],
        h2: f32,
        i: usize,
        mut f: impl FnMut(usize, Vec3, f32),
    ) {
        let pi = pos[i];
        let (cx, cy, cz) = self.bin_of(pi);
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        for dz in -1i32..=1 {
            let z = cz as i32 + dz;
            if z < 0 || z >= nz as i32 {
                continue;
            }
            for dy in -1i32..=1 {
                let y = cy as i32 + dy;
                if y < 0 || y >= ny as i32 {
                    continue;
                }
                for dx in -1i32..=1 {
                    let x = cx as i32 + dx;
                    if x < 0 || x >= nx as i32 {
                        continue;
                    }
                    let idx = ((x as u32 * ny + y as u32) * nz + z as u32) as usize;
                    for &j in self.bin_items(idx) {
                        let j = j as usize;
                        if j == i {
                            continue;
                        }
                        let d = pi - pos[j];
                        let r2 = d.length_squared();
                        if r2 <= h2 {
                            f(j, d, r2);
                        }
                    }
                }
            }
        }
    }
}

/// 邻域网格的**只读导出**（见 `FluidSystem::neighbor_grid`）：`start` 长 `total+1`、
/// `items` 长 `n`；`min/inv/dims` 供调用方复算粒子所在格（与 CPU 同一公式）。
#[derive(Clone, Copy, Debug)]
pub struct NeighborGrid<'a> {
    pub min: Vec3,
    pub inv: f32,
    pub dims: (u32, u32, u32),
    pub start: &'a [u32],
    pub items: &'a [u32],
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 箱子规则的**单一来源**由这条测试锁住：`rebuild` 算出的箱子必须与
    /// `vxl_phys_core::grid::grid_box` **逐位相同**（GPU 常驻管线的每 tick 重算走的就是后者）。
    /// 一旦有人在任一侧改规则，这条会红。
    #[test]
    fn rebuild_box_matches_core_grid_box() {
        let sets: Vec<Vec<Vec3>> = vec![
            // 单粒子：ext = 0 ⇒ 每维 1 格
            vec![Vec3::new(0.05, 0.05, 0.05)],
            // 常规散布（含负坐标）
            (0..500)
                .map(|k| {
                    let f = k as f32 * 0.037;
                    Vec3::new(f.sin() * 0.3, 0.1 + f.cos() * 0.2, f * 0.001)
                })
                .collect(),
            // 超预算档：范围极大 ⇒ 必然发生翻倍
            vec![Vec3::new(-40.0, 0.0, -40.0), Vec3::new(40.0, 30.0, 40.0)],
        ];
        for h in [0.05f32, 0.1] {
            for pos in &sets {
                let mut g = UniformGrid::default();
                g.rebuild(pos, h);
                let mut lo = pos[0];
                let mut hi = pos[0];
                for p in &pos[1..] {
                    lo = lo.min(*p);
                    hi = hi.max(*p);
                }
                let (min, bin, dims) = vxl_phys_core::grid::grid_box(lo, hi, h, GRID_MAX_BINS);
                assert_eq!(g.min, min, "min 角不同（h={h}）");
                assert_eq!(g.bin, bin, "bin 不同（h={h}）");
                assert_eq!(g.inv, 1.0 / bin, "inv 必须逐位相同（h={h}）");
                assert_eq!([g.nx, g.ny, g.nz], dims, "dims 不同（h={h}）");
            }
        }
    }
}
