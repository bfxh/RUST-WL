//! 稀疏体素体（可破坏地形的**物理表示**；ROUTE §3 体素域的第一步）。
//!
//! 表示：均匀网格 + 占据位图（1 bit/格）+ 占据包围盒增量维护。查询：
//! **局域 SDF**（±1 格邻域到最近占据格盒子的带符号距离）+ 有限差分法线
//! ⇒ 实现 `vxl_phys_core::interop::CollisionProvider`，即可作为**碰撞提供者**
//! 被刚体接触消费（跨域唯一通道，见 ROUTE §2.1/§5）。
//!
//! 确定性：查询只读位图与常数；无哈希迭代、无浮点归约顺序问题（逐格循环固定序）。
//! 性能：局域扫描是 O(27)/查询——первый版够用；上量时应换距离场或 BVH
//! （与「体素→SDF→Provider」的专用解法一并做，见 ROUTE §3 体素行）。

use vxl_phys_core::interop::{CollisionProvider, SurfaceHit};
use vxl_phys_core::{Aabb, Quat, Vec3};

/// 均匀网格体素体（占据位图）。
#[derive(Clone, Debug)]
pub struct VoxelVolume {
    origin: Vec3,
    step: f32,
    nx: u32,
    ny: u32,
    nz: u32,
    /// 占据位图（`nx*ny*nz` 位，行主序 ix + nx*(iy + ny*iz)）。
    bits: Vec<u64>,
    /// 占据格数（诊断用）。
    filled: usize,
    /// 占据格的 AABB（格索引闭区间；空体为 None）。
    occ_min: (u32, u32, u32),
    occ_max: (u32, u32, u32),
    any: bool,
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
    fn index(&self, ix: u32, iy: u32, iz: u32) -> usize {
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

impl CollisionProvider for VoxelVolume {
    fn bounds(&self) -> Aabb {
        // 有占据格 → 用占据包围盒（更紧）；否则退化为网格范围。
        self.occupied_bounds().unwrap_or(Aabb {
            min: self.origin,
            max: self.origin
                + Vec3::new(self.nx as f32, self.ny as f32, self.nz as f32) * self.step,
        })
    }

    /// 表面最近点：SDF + 有限差分法线（步长 = 半格，确定性固定序）。
    fn closest_point(&self, p: Vec3) -> Option<SurfaceHit> {
        let d = self.sdf(p);
        if d > self.step * 1.5 {
            return None; // 远离表面：视为无碰撞（与高度场「范围外 None」同语义）
        }
        let e = self.step * 0.5;
        let gx = self.sdf(p + Vec3::new(e, 0.0, 0.0)) - self.sdf(p - Vec3::new(e, 0.0, 0.0));
        let gy = self.sdf(p + Vec3::new(0.0, e, 0.0)) - self.sdf(p - Vec3::new(0.0, e, 0.0));
        let gz = self.sdf(p + Vec3::new(0.0, 0.0, e)) - self.sdf(p - Vec3::new(0.0, 0.0, e));
        let g = Vec3::new(gx, gy, gz);
        let n = if g.length_squared() > 1e-12 {
            g.normalize()
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        Some(SurfaceHit {
            point: p - n * d,
            normal: n,
            signed_dist: d,
        })
    }
}

/// 盒的世界 AABB 半长：`|R|·half`（与宽相同式）。
fn box_aabb_half(half: Vec3, rot: Quat) -> Vec3 {
    let m = vxl_phys_core::Mat3::from_quat(rot);
    Vec3::new(
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).x.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).x.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).x.abs(),
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).y.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).y.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).y.abs(),
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).z.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).z.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).z.abs(),
    )
}

/// 盒形包络的体素专用接触（**按盒的面聚合**，标准「参考面」做法）。
///
/// 采样 = 6 个面的面中心 + 4 角（每面 5 点，共 30 点）。**主导面** = skin 带内
/// 采样数最多的面（并列取最深的）；法线取**该面翻转**（= 表面外向法线；平面
/// 接触精确，斜面/棱边站立时是标准近似——球路径仍用精确 SDF 梯度）。
///
/// 实测教训：早先版本按「采样点的 SDF 梯度」逐点出法线，块体对齐落在柱顶时
/// 4 个角都落在柱角上 ⇒ 梯度是斜向的 ⇒ 窄相取到斜法线，盒沿斜向滑走。
pub fn contacts_box_voxel(
    v: &VoxelVolume,
    half: Vec3,
    pos: Vec3,
    rot: Quat,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let m = vxl_phys_core::Mat3::from_quat(rot);
    // **自适应查询范围（本轮修复）**：先收集「盒 AABB（±1 格）范围内的占据格」，
    // 采样点对这批格求最近距离——而不是只扫点周围 ±1 格（后者会让大碎块的采样点
    // 找不到最近的体素格、拿不到接触 ⇒ 自由落体穿地，实测 74/79 逃逸）。
    let ah = box_aabb_half(half, rot);
    let lo = v.grid_of(pos - ah - Vec3::splat(v.step));
    let hi = v.grid_of(pos + ah + Vec3::splat(v.step));
    let mut cells: Vec<(i32, i32, i32)> = Vec::new();
    for iz in lo.2.max(0)..=hi.2.min(v.nz as i32 - 1) {
        for iy in lo.1.max(0)..=hi.1.min(v.ny as i32 - 1) {
            for ix in lo.0.max(0)..=hi.0.min(v.nx as i32 - 1) {
                if v.get(ix as u32, iy as u32, iz as u32) {
                    cells.push((ix, iy, iz));
                }
            }
        }
    }
    if cells.is_empty() {
        return false;
    }
    // 到「这批格」的带符号距离（同 `sdf` 公式，但遍历给定列表）
    let sd = |p: Vec3| -> f32 {
        let mut best = v.step * 2.0;
        for &(ix, iy, iz) in &cells {
            let lo = v.grid_center(ix as u32, iy as u32, iz as u32) - Vec3::splat(v.step * 0.5);
            let hi = lo + Vec3::splat(v.step);
            let q = Vec3::new(
                (lo.x - p.x).max(p.x - hi.x).max(0.0),
                (lo.y - p.y).max(p.y - hi.y).max(0.0),
                (lo.z - p.z).max(p.z - hi.z).max(0.0),
            );
            let outside = q.length();
            let d = if outside > 0.0 {
                outside
            } else {
                let inx = (p.x - lo.x).min(hi.x - p.x);
                let iny = (p.y - lo.y).min(hi.y - p.y);
                let inz = (p.z - lo.z).min(hi.z - p.z);
                -inx.min(iny).min(inz)
            };
            if d < best {
                best = d;
            }
        }
        best
    };
    // 6 个面（局部轴向外法线）：±X/±Y/±Z
    let dirs = [
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let h = [half.x, half.y, half.z];
    let d3 = [
        [
            dirs[0].x, dirs[1].x, dirs[2].x, dirs[3].x, dirs[4].x, dirs[5].x,
        ],
        [
            dirs[0].y, dirs[1].y, dirs[2].y, dirs[3].y, dirs[4].y, dirs[5].y,
        ],
        [
            dirs[0].z, dirs[1].z, dirs[2].z, dirs[3].z, dirs[4].z, dirs[5].z,
        ],
    ];
    // 每面：(skin 带内采样数, 最深 signed_dist)
    let mut count = [0usize; 6];
    let mut deepest = [f32::INFINITY; 6];
    // 逐面采样（面中心 + 4 角）
    for k in 0..6 {
        let (fcx, fcy, fcz) = (d3[0][k] * h[0], d3[1][k] * h[1], d3[2][k] * h[2]);
        let axes = match k {
            0 | 1 => [1usize, 2],
            2 | 3 => [0, 2],
            _ => [0, 1],
        };
        // 采样点（局部）：中心 + 4 角
        let mut samples: [(f32, f32, f32); 5] = [(fcx, fcy, fcz); 5];
        for (ci, (su, sv)) in [(-1.0f32, -1.0f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
            .into_iter()
            .enumerate()
        {
            let mut l = [fcx, fcy, fcz];
            l[axes[0]] = su * h[axes[0]];
            l[axes[1]] = sv * h[axes[1]];
            samples[ci + 1] = (l[0], l[1], l[2]);
        }
        for s in &samples {
            let p = pos + m.mul_vec3(Vec3::new(s.0, s.1, s.2));
            let d = sd(p);
            if d < skin {
                count[k] += 1;
                deepest[k] = deepest[k].min(d);
            }
        }
    }
    // 主导面：skin 带内采样数最多 → 并列取最深
    let mut best = usize::MAX;
    for k in 0..6 {
        if count[k] == 0 {
            continue;
        }
        if best == usize::MAX
            || count[k] > count[best]
            || (count[k] == count[best] && deepest[k] < deepest[best])
        {
            best = k;
        }
    }
    if best == usize::MAX {
        return false;
    }
    // **逐面发射**（2026-09-15 修复）：此前只发「主导面」（按带内采样数、并列取
    // 最深选一张），墙角下会按深度选中**地板面**而把**墙面整张丢掉**——体的水平
    // 动量被倾斜的地板法线吸收、撞墙事件不登记（实测 `arena_bench wall_provider`：
    // 12 m/s 弹体在墙前 0.14 m 处 vx 11.4 → −0.02，墙面法线缺失 ⇒ 破坏管线无冲击，
    // 2 子步档下更明显）。现在 6 张面各自「带内即发」（法线按面、特征 = 面号×16 +
    // 采样号），主导面选择从 provider 内移到窄相，依据从"深度"换成"闭合速度"
    // （见 narrow 的 CLOSING_MIN 逻辑）——两处合起来才修好"墙角丢面"。
    let _ = best;
    let mut any = false;
    for k in 0..6usize {
        if count[k] == 0 {
            continue;
        }
        let n_world = m.mul_vec3(dirs[k] * -1.0);
        let (fcx, fcy, fcz) = (d3[0][k] * h[0], d3[1][k] * h[1], d3[2][k] * h[2]);
        let axes = match k {
            0 | 1 => [1usize, 2],
            2 | 3 => [0, 2],
            _ => [0, 1],
        };
        for ci in 0..5usize {
            let (lx, ly, lz) = if ci == 0 {
                (fcx, fcy, fcz)
            } else {
                let (su, sv) = match ci {
                    1 => (-1.0f32, -1.0f32),
                    2 => (-1.0, 1.0),
                    3 => (1.0, -1.0),
                    _ => (1.0, 1.0),
                };
                let mut l = [fcx, fcy, fcz];
                l[axes[0]] = su * h[axes[0]];
                l[axes[1]] = sv * h[axes[1]];
                (l[0], l[1], l[2])
            };
            let p = pos + m.mul_vec3(Vec3::new(lx, ly, lz));
            let d = sd(p);
            if d >= skin {
                continue;
            }
            out.push(vxl_phys_core::interop::InteropContact {
                point: p,
                normal: n_world,
                depth: -d,
                // 面号(1..6)×16 + 采样号(0 = 中心, 1..4 = 角)：跨帧稳定，供 warm 匹配
                feature: (k as u32 + 1) * 16 + ci as u32,
            });
            any = true;
        }
    }
    any
}

/// 球 vs 体素的**解析**接触（SDF 语义）：`depth = r − sdf(center)`，法线取
/// SDF 梯度；接触点取「球面点与 provider 表面点的中点」。只保留 `depth ≥ −skin`。
pub fn contacts_sphere_voxel(
    v: &VoxelVolume,
    center: Vec3,
    radius: f32,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let d = v.sdf(center);
    let depth = radius - d;
    if depth < -skin {
        return false;
    }
    let e = v.step * 0.5;
    let gx = v.sdf(center + Vec3::new(e, 0.0, 0.0)) - v.sdf(center - Vec3::new(e, 0.0, 0.0));
    let gy = v.sdf(center + Vec3::new(0.0, e, 0.0)) - v.sdf(center - Vec3::new(0.0, e, 0.0));
    let gz = v.sdf(center + Vec3::new(0.0, 0.0, e)) - v.sdf(center - Vec3::new(0.0, 0.0, e));
    let g = Vec3::new(gx, gy, gz);
    let n = if g.length_squared() > 1e-12 {
        g.normalize()
    } else {
        Vec3::Y
    };
    out.push(vxl_phys_core::interop::InteropContact {
        point: center - n * ((radius + d) * 0.5),
        normal: n,
        depth,
        feature: 1,
    });
    true
}

/// **点查询**：`depth = −sdf(p)`，即 **depth = 穿透量**（正 = 点已在固体里侧）；
/// 带内判据 `depth > −skin`（等价 `sdf < skin`）⇒ 表面外 `skin` 内仍生成**预期接触**。
/// 与 `contacts_box_voxel`（`depth = −d`）和网格路径（2026-09-21 起 `depth = −sd`）**同一口径**。
///
/// **偏置修正**（2026-09-21，P5 ⑳ 同族）：旧式 `depth = skin − sdf` 在**外侧**也给正 depth
/// ⇒ 走点查询的**外壳顶点采样**把体顶到 `sdf ≈ skin` ⇒ **包体在体素地面上悬空 ≈ 0.022 m**
/// （实测 `voxel_rest_probe`：同一块板上盒 −0.0004、球 −0.0000、**包 +0.0220**）。
/// 流体不受影响：它**不读 `depth`**，只用返回点的 `point`/`normal` 自算 sdf，
/// 且它过滤的上界就是 `self.h`（新口径的带正好等于管道半径 h，旧口径是 2h 后又被它滤掉）。
pub fn contacts_point_voxel(
    v: &VoxelVolume,
    p: Vec3,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let d = v.sdf(p);
    let depth = -d;
    if depth < -skin {
        return true; // 支持查询，但该点不在接触带内（无接触）
    }
    let e = v.step * 0.5;
    let gx = v.sdf(p + Vec3::new(e, 0.0, 0.0)) - v.sdf(p - Vec3::new(e, 0.0, 0.0));
    let gy = v.sdf(p + Vec3::new(0.0, e, 0.0)) - v.sdf(p - Vec3::new(0.0, e, 0.0));
    let gz = v.sdf(p + Vec3::new(0.0, 0.0, e)) - v.sdf(p - Vec3::new(0.0, 0.0, e));
    let g = Vec3::new(gx, gy, gz);
    let n = if g.length_squared() > 1e-12 {
        g.normalize()
    } else {
        Vec3::Y
    };
    out.push(vxl_phys_core::interop::InteropContact {
        point: p - n * d,
        normal: n,
        depth,
        feature: 0,
    });
    true
}

/// **点查询·流体边界口径**（`contacts_point_voxel` 的内点鲁棒变体）。
///
/// 所在格为空或出界（域外/腔内驻留点）与 [`contacts_point_voxel`] 逐位
/// 一致——外点的截断 SDF 梯度可信（刚体通道既有行为不变）。内点判定按
/// **所在格占据**而非 sdf 符号：截断 SDF 把出界当「空」，靠近网格外壳的
/// 内点（距外壳面不足一格）sdf 会误报为正 ⇒ 若按符号分流，会走旧外点
/// 路径沿截断梯度被持续外推——穿壁隧逃引信之一（实测 escapee 钉在
/// x≈−0.385 外壳驻留带上）。
///
/// 内点**不用**截断 SDF 的内部梯度：±1 格截断在薄壁/角部内部被格间内面
/// （占据格之间的共享面）主导，中心差分符号可翻转——轻则沿壁面滑推
/// （穿透轴永不修正），重则把粒子推出远侧表面（穿壁隧逃）。改为在 ±1 格
/// 邻域内搜最近**真表面面片**（占据格朝空邻格/出界的面）：距离² = 轴向
/// 差² + 两径向轴**各归各轴**的钳制距离²；法线 = 面轴外向，
/// `depth = skin − (−距离)` 与外点同口径。
///
/// 推回侧：内点一律推往最近的**流体可达面**（邻格在网格内且空）——外壳
/// 面通向域外，推过去等于把粒子逐出模拟域（隧逃引信之二：距外壳面不足
/// 0.25·step 的「新穿透原路推回」在壁内命中外壳面，与 sdf 误报叠加成
/// 外推棘轮，故不作「原路推回」特判）。邻域内无流体可达面才退外壳面；
/// 再无（深陷大固体，流体浅穿透机制下不会发生）退回截断 SDF 梯度路径
/// （次优但有限）。扫描序 (dz,dy,dx) × 面序 (轴,±) 固定，严格 `<` 取
/// 最近 ⇒ 确定。
pub fn contacts_point_voxel_solid(
    v: &VoxelVolume,
    p: Vec3,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let (cx, cy, cz) = v.grid_of(p);
    // 内点判定：所在格被占据 ⇒ 固体内部（格占据是精确判据，不受截断
    // SDF 在外壳附近的符号噪声影响）；否则（空格/出界）走旧外点路径。
    if !v.in_range(cx, cy, cz) || !v.get(cx as u32, cy as u32, cz as u32) {
        return contacts_point_voxel(v, p, skin, out);
    }
    let step = v.step;
    // 两轨最近真表面面片：open = 邻格在网格内且空（流体可达面）；
    // shell = 邻格出界（网格外壳面）。扫描序 (dz,dy,dx) × 面序 (轴,±)
    // 固定 + 严格 `<` 取最近 ⇒ 确定。
    let mut open_d2 = f32::INFINITY;
    let mut open_n = Vec3::Y;
    let mut shell_d2 = f32::INFINITY;
    let mut shell_n = Vec3::Y;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (ix, iy, iz) = (cx + dx, cy + dy, cz + dz);
                if !v.in_range(ix, iy, iz) || !v.get(ix as u32, iy as u32, iz as u32) {
                    continue;
                }
                let lo = v.grid_center(ix as u32, iy as u32, iz as u32) - Vec3::splat(step * 0.5);
                let hi = lo + Vec3::splat(step);
                // 两径向轴到格区间外的钳制距离（各归各轴；面片矩形距离的径向项）。
                let qx = (lo.x - p.x).max(p.x - hi.x).max(0.0);
                let qy = (lo.y - p.y).max(p.y - hi.y).max(0.0);
                let qz = (lo.z - p.z).max(p.z - hi.z).max(0.0);
                // 6 面：面为「真表面」⇔ 该方向邻格空或出界。面序固定
                // （−x,+x,−y,+y,−z,+z），法线 = 外向；到面片矩形的距离²
                // = 轴向差² + 两径向项²。
                let faces = [
                    (Vec3::new(-1.0, 0.0, 0.0), (lo.x - p.x).abs(), qy, qz),
                    (Vec3::new(1.0, 0.0, 0.0), (hi.x - p.x).abs(), qy, qz),
                    (Vec3::new(0.0, -1.0, 0.0), (lo.y - p.y).abs(), qx, qz),
                    (Vec3::new(0.0, 1.0, 0.0), (hi.y - p.y).abs(), qx, qz),
                    (Vec3::new(0.0, 0.0, -1.0), (lo.z - p.z).abs(), qx, qy),
                    (Vec3::new(0.0, 0.0, 1.0), (hi.z - p.z).abs(), qx, qy),
                ];
                for (dir, da, dr1, dr2) in faces {
                    let (nx, ny, nz) = (ix + dir.x as i32, iy + dir.y as i32, iz + dir.z as i32);
                    if v.in_range(nx, ny, nz) && v.get(nx as u32, ny as u32, nz as u32) {
                        continue; // 内面（贴着占据格）：截断 SDF 的噪声源，跳过
                    }
                    let d2 = da * da + dr1 * dr1 + dr2 * dr2;
                    if v.in_range(nx, ny, nz) {
                        if d2 < open_d2 {
                            open_d2 = d2;
                            open_n = dir;
                        }
                    } else if d2 < shell_d2 {
                        shell_d2 = d2;
                        shell_n = dir;
                    }
                }
            }
        }
    }
    // 推回侧：一律优先流体可达面（open），外壳面仅作邻域内无 open 时的
    // 兜底（见函数文档——「原路推回」特判在外壳附近会与 sdf 误报叠加成
    // 外推棘轮，已删除）；两者皆无退回截断 SDF 梯度路径（次优但有限）。
    let (face_n, dist) = if open_d2.is_finite() {
        (open_n, open_d2.sqrt())
    } else if shell_d2.is_finite() {
        (shell_n, shell_d2.sqrt())
    } else {
        return contacts_point_voxel(v, p, skin, out);
    };
    let depth = skin + dist; // 内点：sdf = −dist ⇒ depth = skin − (−dist)
    out.push(vxl_phys_core::interop::InteropContact {
        point: p + face_n * dist,
        normal: face_n,
        depth,
        feature: 0,
    });
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floor_volume() -> VoxelVolume {
        // 8×2×8 格、边长 0.5：填充 y ∈ [0,1) 两层 ⇒ 顶面 y = 1.0
        let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
        v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
        v
    }

    #[test]
    fn sdf_signs_and_surface() {
        let v = floor_volume();
        // 地面上方 0.25 ⇒ +0.25
        let d = v.sdf(Vec3::new(0.0, 1.25, 0.0));
        assert!((d - 0.25).abs() < 1e-5, "d={d}");
        // 地面内一格的**中心**（0.25,0.75,0.25 ⇒ 格 y=[0.5,1.0] 的中心）⇒ 负；
        // 到最近面 = 半格 0.25（注意别取在格边界上：那里距离恰为 0）
        let d = v.sdf(Vec3::new(0.25, 0.75, 0.25));
        assert!(d < 0.0, "d={d}");
        assert!((d + 0.25).abs() < 1e-5, "d={d}");
        // 表面最近点与法线
        let hit = v.closest_point(Vec3::new(0.25, 1.25, 0.25)).unwrap();
        assert!((hit.point.y - 1.0).abs() < 1e-5, "py={}", hit.point.y);
        assert!((hit.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-4);
        assert!((hit.signed_dist - 0.25).abs() < 1e-5);
    }

    #[test]
    fn dug_hole_reads_as_empty_with_walls() {
        let mut v = floor_volume();
        // 挖掉 (0,0,0) 与 (0,1,0) 两格（x∈[0,0.5), y∈[0,1), z∈[0,0.5)）
        v.set(4, 0, 4, false);
        v.set(4, 1, 4, false);
        assert_eq!(v.filled_count(), 8 * 2 * 8 - 2);
        // 洞里（y=0.5, x=z=0.25）现在是空：SDF 应为正（到洞壁/洞底的最近距离）
        let d = v.sdf(Vec3::new(0.25, 0.5, 0.25));
        assert!(d > 0.0, "d={d}");
        // 洞底 y=0.25 处在洞里；正下方无占据格（挖穿到底）⇒ 仍是空
        assert!(v.sdf(Vec3::new(0.25, 0.1, 0.25)) > 0.0);
    }

    #[test]
    fn box_contacts_on_voxel_floor() {
        let v = floor_volume();
        // 盒（半 0.5）落在地面 y=1.0 上、穿透 0.1 ⇒ 底面 4 角 depth ≈ 0.1
        let mut out = Vec::new();
        let any = contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 1.4, 0.25),
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any);
        // **逐面发射**（2026-09-15 起）：每张「带内样本数 > 0」的面各自发点；
        // 共享角点会在相邻面里重复出现，因此断言改为：
        //  ① 贴地面的 5 点（面号 3 ⇒ feature 48..52）必须在，且深度 ≈0.1、法线 +Y；
        //  ② 其它面只允许出现**角点**（feature % 16 != 0）——面心样本只有真贴着
        //     的底面才有（窄相据此在多面候选里排除"只有角点的伪面"）。
        let bottom: Vec<_> = out
            .iter()
            .filter(|c| (48..53).contains(&c.feature))
            .collect();
        assert_eq!(bottom.len(), 5, "贴地面应有 5 点；out={}", out.len());
        for c in &bottom {
            assert!((c.depth - 0.1).abs() < 1e-4, "depth={}", c.depth);
            assert!((c.normal.y - 1.0).abs() < 1e-4, "normal={:?}", c.normal);
        }
        for c in out.iter().filter(|c| !(48..53).contains(&c.feature)) {
            assert_ne!(
                c.feature % 16,
                0,
                "非贴地面不得有面心样本：feature={}",
                c.feature
            );
        }
        // 面中心点（feature 48）也应在（对齐落面时中心才给得出正确法线语义）
        assert!(out.iter().any(|c| c.feature == 48));
        // 提升到 y=2.5（远离表面）⇒ 无接触
        let mut out2 = Vec::new();
        assert!(!contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 2.5, 0.25),
            Quat::IDENTITY,
            0.02,
            &mut out2
        ));
    }

    #[test]
    fn box_fully_embedded_reports_contact() {
        // 回归：盒**完全嵌入**体积内部时也必须报接触——否则碎块会「自由落体
        // 穿地」（实测逃逸机制：体素挖出的碎块嵌在残余结构里却拿不到接触）。
        let v = floor_volume(); // 顶面 y=1.0
        let mut out = Vec::new();
        let any = contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 0.75, 0.25), // 底 0.25、顶 1.25 ⇒ 完全在体内
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any, "完全嵌入的盒必须报接触（out={}）", out.len());
        assert!(!out.is_empty());
        // 深度应为正（穿透）
        assert!(out[0].depth > 0.0, "depth={}", out[0].depth);
    }

    #[test]
    fn sphere_sdf_contact_depth_and_normal() {
        let v = floor_volume();
        // 球心在 y=1.3（地面顶 1.0）、半径 0.4 ⇒ 穿透 0.1
        let mut out = Vec::new();
        assert!(contacts_sphere_voxel(
            &v,
            Vec3::new(0.25, 1.3, 0.25),
            0.4,
            0.02,
            &mut out
        ));
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.1).abs() < 1e-4, "depth={}", out[0].depth);
        assert!((out[0].normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-4);
        // 球心在 y=1.5、半径 0.4 ⇒ 缝 0.1 > skin ⇒ 无接触
        let mut out2 = Vec::new();
        assert!(!contacts_sphere_voxel(
            &v,
            Vec3::new(0.25, 1.5, 0.25),
            0.4,
            0.02,
            &mut out2
        ));
    }

    #[test]
    fn sphere_extract_carves_crater() {
        // 8×8×8 实体块（格边长 0.5、origin 0）：球域挖洞 ⇒ 洞内变空、盒数>0、
        // 移除格数 ≈ 球体积/格体积（格心判据 ⇒ 数量级一致即可）
        let mut v = VoxelVolume::new(Vec3::ZERO, 0.5, 8, 8, 8);
        v.fill_box(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0));
        let filled0 = v.filled_count();
        let boxes = v.extract_sphere(Vec3::new(2.0, 2.0, 2.0), 1.0);
        let removed = filled0 - v.filled_count();
        assert!(!boxes.is_empty(), "球域应提取出碎块盒");
        let vol_cells = (4.0 / 3.0 * std::f32::consts::PI * 1.0f32.powi(3)) / 0.5f32.powi(3);
        assert!(
            (removed as f32) > vol_cells * 0.6 && (removed as f32) < vol_cells * 1.4,
            "移除格数 {removed} 应接近球体积格数 {vol_cells:.1}"
        );
        // 球心处已空
        assert!(!v.get(4, 4, 4), "球心格应被挖掉");
        // 球外的角点仍在
        assert!(v.get(0, 0, 0));
        assert!(v.get(7, 7, 7));
    }

    #[test]
    fn voronoi_fracture_tiles_region_exactly() {
        // 守恒：Voronoi 分区是对域内占据格的**划分** ⇒ 提取总格数 = 原占据格数
        let mut v = VoxelVolume::new(Vec3::ZERO, 0.5, 8, 8, 8);
        v.fill_box(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0));
        let filled0 = v.filled_count();
        let seeds =
            VoxelVolume::seeds_jittered(Vec3::new(0.5, 0.5, 0.5), Vec3::new(3.5, 3.5, 3.5), 8, 0.6);
        assert_eq!(seeds.len(), 8);
        let cells = v.fracture_voronoi(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0), &seeds);
        assert!(!cells.is_empty(), "应至少产出一个碎块簇");
        // 守恒：提取到的格数（按盒体积折算）× 全部 = 原格数
        let mut removed = 0usize;
        for (_, boxes) in &cells {
            for (_, h) in boxes {
                // 盒体积 / 格体积 = 覆盖格数（贪心合并的盒都是整格并集）
                removed += ((2.0 * h.x / 0.5).round()
                    * (2.0 * h.y / 0.5).round()
                    * (2.0 * h.z / 0.5).round()) as usize;
            }
        }
        assert_eq!(removed, filled0, "Voronoi 分区必须恰好覆盖域内全部占据格");
        assert_eq!(v.filled_count(), 0, "域内应被全部提取");
        // 种子数 ≥ 2 时通常至少 2 个非空簇（8 个种子 + 抖动 ⇒ 必然多簇）
        assert!(cells.len() >= 2, "多种子应产出多个簇；实际 {}", cells.len());
    }

    #[test]
    fn bounds_track_occupied_cells() {
        let mut v = VoxelVolume::new(Vec3::ZERO, 1.0, 4, 4, 4);
        assert!(v.occupied_bounds().is_none());
        v.set(1, 0, 2, true);
        let b = v.occupied_bounds().unwrap();
        assert!((b.min.x - 1.0).abs() < 1e-6 && (b.max.x - 2.0).abs() < 1e-6);
        assert!((b.min.y - 0.0).abs() < 1e-6 && (b.max.y - 1.0).abs() < 1e-6);
        assert!((b.min.z - 2.0).abs() < 1e-6 && (b.max.z - 3.0).abs() < 1e-6);
        let _ = v.bounds(); // 覆盖 provider 路径
    }
}
