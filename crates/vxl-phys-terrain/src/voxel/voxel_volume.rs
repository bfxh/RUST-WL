//! voxel_volume：从 voxel.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 均匀网格体素体（占据位图）。
#[derive(Clone, Debug)]
pub struct VoxelVolume {
    pub(crate) origin: Vec3,
    pub(crate) step: f32,
    pub(crate) nx: u32,
    pub(crate) ny: u32,
    pub(crate) nz: u32,
    /// 占据位图（`nx*ny*nz` 位，行主序 ix + nx*(iy + ny*iz)）。
    pub(crate) bits: Vec<u64>,
    /// 占据格数（诊断用）。
    pub(crate) filled: usize,
    /// 占据格的 AABB（格索引闭区间；空体为 None）。
    pub(crate) occ_min: (u32, u32, u32),
    pub(crate) occ_max: (u32, u32, u32),
    pub(crate) any: bool,
}

impl VoxelVolume {
    /// 空体：`origin` 为体素网格原点，`step` 为边长，`nx/ny/nz` 为格数。
    pub fn new(origin: Vec3, step: f32, nx: u32, ny: u32, nz: u32) -> Self {
        assert!(step > 0.0 && nx > 0 && ny > 0 && nz > 0);
        let words = (nx as usize * ny as usize * nz as usize).div_ceil(64);
        Self {
            origin,
            step,
            nx,
            ny,
            nz,
            bits: vec![0; words],
            filled: 0,
            occ_min: (u32::MAX, u32::MAX, u32::MAX),
            occ_max: (0, 0, 0),
            any: false,
        }
    }

    #[inline]
    pub(crate) fn index(&self, ix: u32, iy: u32, iz: u32) -> usize {
        (ix + self.nx * (iy + self.ny * iz)) as usize
    }

    #[inline]
    pub fn in_range(&self, ix: i32, iy: i32, iz: i32) -> bool {
        ix >= 0
            && iy >= 0
            && iz >= 0
            && (ix as u32) < self.nx
            && (iy as u32) < self.ny
            && (iz as u32) < self.nz
    }

    #[inline]
    pub fn get(&self, ix: u32, iy: u32, iz: u32) -> bool {
        if !self.in_range(ix as i32, iy as i32, iz as i32) {
            return false;
        }
        let i = self.index(ix, iy, iz);
        (self.bits[i >> 6] >> (i & 63)) & 1 == 1
    }

    /// 写入/清除单格（就地账本；确定性：单格单写）。
    pub fn set(&mut self, ix: u32, iy: u32, iz: u32, on: bool) {
        if !self.in_range(ix as i32, iy as i32, iz as i32) {
            return;
        }
        let i = self.index(ix, iy, iz);
        let w = i >> 6;
        let b = 1u64 << (i & 63);
        let was = self.bits[w] & b != 0;
        if was == on {
            return;
        }
        if on {
            self.bits[w] |= b;
            self.filled += 1;
            self.any = true;
            self.occ_min = (
                self.occ_min.0.min(ix),
                self.occ_min.1.min(iy),
                self.occ_min.2.min(iz),
            );
            self.occ_max = (
                self.occ_max.0.max(ix),
                self.occ_max.1.max(iy),
                self.occ_max.2.max(iz),
            );
        } else {
            self.bits[w] &= !b;
            self.filled -= 1;
        }
    }

    /// 填充世界系盒域内的格（`min`/`max` 为世界坐标；**max 侧开区间**：
    /// 只填完全落在 `[min, max)` 内的格——`max = 1.5` 且格边长 0.5 时填到
    /// 顶面 1.5，而不是把 [1.5,2.0) 也填上。闭区间语义曾让场景「地板比预期
    /// 厚一格」（实测：炮弹开局嵌在地板里、tick 1 就触发破坏）。
    pub fn fill_box(&mut self, min: Vec3, max: Vec3) {
        let lo = self.grid_of(min);
        let inv = 1.0 / self.step;
        let hi = (
            ((max.x - self.origin.x) * inv).ceil() as i32 - 1,
            ((max.y - self.origin.y) * inv).ceil() as i32 - 1,
            ((max.z - self.origin.z) * inv).ceil() as i32 - 1,
        );
        for iz in lo.2.max(0)..=hi.2.min(self.nz as i32 - 1) {
            for iy in lo.1.max(0)..=hi.1.min(self.ny as i32 - 1) {
                for ix in lo.0.max(0)..=hi.0.min(self.nx as i32 - 1) {
                    self.set(ix as u32, iy as u32, iz as u32, true);
                }
            }
        }
    }

    /// 世界坐标 → 格索引（可越界；调用方自行判 `in_range`）。
    #[inline]
    pub fn grid_of(&self, p: Vec3) -> (i32, i32, i32) {
        let inv = 1.0 / self.step;
        let f = (p - self.origin) * inv;
        (f.x.floor() as i32, f.y.floor() as i32, f.z.floor() as i32)
    }

    /// 格索引 → 格中心世界坐标。
    #[inline]
    pub fn grid_center(&self, ix: u32, iy: u32, iz: u32) -> Vec3 {
        self.origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * self.step
    }

    /// 格数 (nx, ny, nz)。
    pub fn dims(&self) -> (u32, u32, u32) {
        (self.nx, self.ny, self.nz)
    }

    /// 体原点（最小角）。
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    /// 格边长。
    pub fn step(&self) -> f32 {
        self.step
    }

    pub fn filled_count(&self) -> usize {
        self.filled
    }

    /// 局域 SDF：`±1` 格邻域内到最近**占据格盒子**的带符号距离。
    /// 正 = 体外、负 = 体内；邻域内无占据格时返回 `2×step`（粗远界）。
    pub fn sdf(&self, p: Vec3) -> f32 {
        if !self.any {
            return self.step * 2.0;
        }
        let (cx, cy, cz) = self.grid_of(p);
        let mut best = self.step * 2.0;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let (ix, iy, iz) = (cx + dx, cy + dy, cz + dz);
                    if !self.in_range(ix, iy, iz) {
                        continue;
                    }
                    if !self.get(ix as u32, iy as u32, iz as u32) {
                        continue;
                    }
                    // 到该格盒子的距离（盒内 → 负：到最近面的距离取负）。
                    let lo = self.grid_center(ix as u32, iy as u32, iz as u32)
                        - Vec3::splat(self.step * 0.5);
                    let hi = lo + Vec3::splat(self.step);
                    let q = Vec3::new(
                        (lo.x - p.x).max(p.x - hi.x).max(0.0),
                        (lo.y - p.y).max(p.y - hi.y).max(0.0),
                        (lo.z - p.z).max(p.z - hi.z).max(0.0),
                    );
                    let outside = q.length();
                    let d = if outside > 0.0 {
                        outside
                    } else {
                        // 盒内：到最近面（各轴穿透量的最大值取负）
                        let inx = (p.x - lo.x).min(hi.x - p.x);
                        let iny = (p.y - lo.y).min(hi.y - p.y);
                        let inz = (p.z - lo.z).min(hi.z - p.z);
                        -inx.min(iny).min(inz)
                    };
                    if d < best {
                        best = d;
                    }
                }
            }
        }
        best
    }

    /// **破坏提取**（M3 第一块）：把盒域内的占据格合并成若干**轴对齐盒**
    /// （贪心：先沿 +X 拉长，再沿 +Y、+Z 按整行/整片匹配），并从体积里清除。
    /// 返回 `(中心, 半长)` 列表——供门面转成刚体碎块。
    ///
    /// 确定性：扫描序固定 `(iz, iy, ix)`、扩展方向固定；提取顺序 = 扫描序。
    /// 注意：占据包围盒只增不减 ⇒ 提取后 `bounds()` 可能偏保守（宽相多做工作，
    /// 不影响正确性；需要紧盒时调用方重建体积或加紧盒接口）。
    pub fn extract_boxes(&mut self, min: Vec3, max: Vec3) -> Vec<(Vec3, Vec3)> {
        self.extract_where(min, max, |_ix, _iy, _iz| true)
    }

    /// **球域挖洞**（任意形状切割第一步；爆炸/弹坑形态）：提取**格心落在球内**的
    /// 占据格，合并成轴对齐盒（贪心同 `extract_where`）。确定性：格心判据 +
    /// 固定扫描序。返回 `(中心, 半长)` 列表；半径 ≤ 0 时为 0 个。
    pub fn extract_sphere(&mut self, center: Vec3, radius: f32) -> Vec<(Vec3, Vec3)> {
        if radius <= 0.0 {
            return Vec::new();
        }
        let r2 = radius * radius;
        // 复制到局部：闭包不得借用 self（调用处已 &mut self 借出）
        let (origin, step) = (self.origin, self.step);
        self.extract_where(
            center - Vec3::splat(radius),
            center + Vec3::splat(radius),
            move |ix, iy, iz| {
                let c =
                    origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * step;
                (c - center).length_squared() <= r2
            },
        )
    }

    /// 通用提取：在 `[min, max]` 的格范围内，取「占据 且 谓词为真」的格，
    /// 贪心合并成轴对齐盒并清除（+X 拉长 → +Y 整行 → +Z 整片）。
    /// 谓词签名 `(ix, iy, iz) -> bool`；**确定性**：扫描序与扩展方向固定。
    pub fn extract_where<F>(&mut self, min: Vec3, max: Vec3, inside: F) -> Vec<(Vec3, Vec3)>
    where
        F: Fn(i32, i32, i32) -> bool,
    {
        let mut out = Vec::new();
        let lo = self.grid_of(min);
        let hi = self.grid_of(max);
        let x0 = lo.0.max(0);
        let y0 = lo.1.max(0);
        let z0 = lo.2.max(0);
        let x1 = hi.0.min(self.nx as i32 - 1);
        let y1 = hi.1.min(self.ny as i32 - 1);
        let z1 = hi.2.min(self.nz as i32 - 1);
        if x0 > x1 || y0 > y1 || z0 > z1 {
            return out;
        }
        for iz in z0..=z1 {
            for iy in y0..=y1 {
                for ix in x0..=x1 {
                    if !self.get(ix as u32, iy as u32, iz as u32) || !inside(ix, iy, iz) {
                        continue;
                    }
                    // +X 连续
                    let mut ex = ix;
                    while ex < x1
                        && self.get((ex + 1) as u32, iy as u32, iz as u32)
                        && inside(ex + 1, iy, iz)
                    {
                        ex += 1;
                    }
                    // +Y 整行匹配
                    let mut ey = iy;
                    'ey: while ey < y1 {
                        for kx in ix..=ex {
                            if !self.get(kx as u32, (ey + 1) as u32, iz as u32)
                                || !inside(kx, ey + 1, iz)
                            {
                                break 'ey;
                            }
                        }
                        ey += 1;
                    }
                    // +Z 整片匹配
                    let mut ez = iz;
                    'ez: while ez < z1 {
                        for ky in iy..=ey {
                            for kx in ix..=ex {
                                if !self.get(kx as u32, ky as u32, (ez + 1) as u32)
                                    || !inside(kx, ky, ez + 1)
                                {
                                    break 'ez;
                                }
                            }
                        }
                        ez += 1;
                    }
                    // 消费（清除）+ 记录盒
                    for kz in iz..=ez {
                        for ky in iy..=ey {
                            for kx in ix..=ex {
                                self.set(kx as u32, ky as u32, kz as u32, false);
                            }
                        }
                    }
                    let bmin = self.grid_center(ix as u32, iy as u32, iz as u32)
                        - Vec3::splat(self.step * 0.5);
                    let bmax = self.grid_center(ex as u32, ey as u32, ez as u32)
                        + Vec3::splat(self.step * 0.5);
                    out.push(((bmin + bmax) * 0.5, (bmax - bmin) * 0.5));
                }
            }
        }
        out
    }

    /// **体素 Voronoi 分区（预断裂）**：把 `[min, max]` 域内的占据格按「距最近种子」
    /// 归属到 `seeds` 的某一格，逐种子提取成碎块盒（贪心同 `extract_where`）。
    /// 返回 `(种子序号, 碎块盒列表)`（顺序 = 种子序；无格的种子不出现）。
    ///
    /// **确定性**：归属判据 = 格心到种子的平方距离，**并列取序号小的种子**；
    /// 提取顺序 = 种子序，内部扫描序固定。**守恒**：分区是对域内占据格的**划分**
    /// （每格恰好被提取一次）⇒ 提取总格数 = 域内原占据格数（测试守门）。
    pub fn fracture_voronoi(
        &mut self,
        min: Vec3,
        max: Vec3,
        seeds: &[Vec3],
    ) -> Vec<(usize, Vec<(Vec3, Vec3)>)> {
        if seeds.is_empty() {
            return Vec::new();
        }
        let (origin, step) = (self.origin, self.step);
        let mut out = Vec::new();
        for (si, &seed) in seeds.iter().enumerate() {
            let boxes = self.extract_where(min, max, |ix, iy, iz| {
                let c =
                    origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * step;
                let d_me = (c - seed).length_squared();
                // 并列取序号小者：只有「更近」或「并列且序号更小」才归我
                for (sj, &other) in seeds.iter().enumerate() {
                    if sj == si {
                        continue;
                    }
                    let d_o = (c - other).length_squared();
                    if d_o < d_me || (d_o == d_me && sj < si) {
                        return false;
                    }
                }
                true
            });
            if !boxes.is_empty() {
                out.push((si, boxes));
            }
        }
        out
    }

    /// **确定性抖动种子**（Voronoi 预断裂用）：`n` 个种子按立方根网格铺开 + 整数
    /// 哈希抖动（无外部 RNG；同参数 ⇒ 同结果）。`jitter` ∈ `[0,1]` 为格内抖动比例。
    ///
    /// 网格边 = `ceil(n^(1/3))`：用**整数搜索**求（`side³ ≥ n` 的最小 side）——
    /// 与 `f64::cbrt().ceil()` 同结果，但不引入 f64（§5 严格 f32 纪律）。
    pub fn seeds_jittered(min: Vec3, max: Vec3, n: usize, jitter: f32) -> Vec<Vec3> {
        let n = n.max(1);
        let mut side = 1usize;
        while side * side * side < n {
            side += 1;
        }
        let inv = 1.0 / side as f32;
        let mut out = Vec::with_capacity(side * side * side);
        let mut k = 0usize;
        for iz in 0..side {
            for iy in 0..side {
                for ix in 0..side {
                    if out.len() >= n {
                        break;
                    }
                    // 整数哈希（确定性；splitmix 尾步）
                    let mut h = (k as u64)
                        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                        .wrapping_add(0x1234_5678_9ABC_DEF0);
                    h ^= h >> 30;
                    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
                    h ^= h >> 27;
                    let jx = ((h & 0xFFFF) as f32 / 65535.0 - 0.5) * jitter;
                    let jy = (((h >> 16) & 0xFFFF) as f32 / 65535.0 - 0.5) * jitter;
                    let jz = (((h >> 32) & 0xFFFF) as f32 / 65535.0 - 0.5) * jitter;
                    let t = Vec3::new(
                        (ix as f32 + 0.5 + jx) * inv,
                        (iy as f32 + 0.5 + jy) * inv,
                        (iz as f32 + 0.5 + jz) * inv,
                    );
                    // 分量式：`Vec3` 无逐分量乘法
                    out.push(Vec3::new(
                        min.x + (max.x - min.x) * t.x,
                        min.y + (max.y - min.y) * t.y,
                        min.z + (max.z - min.z) * t.z,
                    ));
                    k += 1;
                }
            }
        }
        out.truncate(n);
        out
    }

    /// SDF 的最小格（用于测试/诊断）。
    pub fn occupied_bounds(&self) -> Option<Aabb> {
        if !self.any {
            return None;
        }
        let lo = self.grid_center(self.occ_min.0, self.occ_min.1, self.occ_min.2)
            - Vec3::splat(self.step * 0.5);
        let hi = self.grid_center(self.occ_max.0, self.occ_max.1, self.occ_max.2)
            + Vec3::splat(self.step * 0.5);
        Some(Aabb { min: lo, max: hi })
    }
}
