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

    /// 填充世界系盒域内的格（`min`/`max` 为世界坐标）。
    pub fn fill_box(&mut self, min: Vec3, max: Vec3) {
        let lo = self.grid_of(min);
        let hi = self.grid_of(max);
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

/// 盒形包络的体素专用接触（比默认角点采样更准：底面 4 角 + 各面中心共 6 点，
/// 覆盖「盒落在体素地面上」与「盒撞体素墙」两类主情形）。
pub fn contacts_box_voxel(
    v: &VoxelVolume,
    half: Vec3,
    pos: Vec3,
    rot: Quat,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let m = vxl_phys_core::Mat3::from_quat(rot);
    // 6 个面中心（局部系）：±X/±Y/±Z
    let dirs = [
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let mut any = false;
    for (k, d) in dirs.iter().enumerate() {
        // 面中心的 4 个角（沿另两轴取 ±half）
        let axes = match k {
            0 | 1 => [1usize, 2],
            2 | 3 => [0, 2],
            _ => [0, 1],
        };
        for (ci, (su, sv)) in [(-1.0f32, -1.0f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
            .into_iter()
            .enumerate()
        {
            // 面中心 ± 另两轴的 ±half（分量数组构造；Vec3 无索引）
            let h = [half.x, half.y, half.z];
            let dd = [d.x, d.y, d.z];
            let mut l = [dd[0] * h[0], dd[1] * h[1], dd[2] * h[2]];
            l[axes[0]] = su * h[axes[0]];
            l[axes[1]] = sv * h[axes[1]];
            let local = Vec3::new(l[0], l[1], l[2]);
            let p = pos + m.mul_vec3(local);
            if let Some(hit) = v.closest_point(p) {
                if hit.signed_dist < skin {
                    out.push(vxl_phys_core::interop::InteropContact {
                        point: hit.point,
                        normal: hit.normal,
                        depth: hit.depth(),
                        // 面号(1..6)×16 + 角号(0..3)：跨帧稳定，供 warm 匹配
                        feature: (k as u32 + 1) * 16 + ci as u32,
                    });
                    any = true;
                }
            }
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
        // 底面（面号 3 ⇒ feature 48..51）恰 4 点、穿透 ≈ 0.1
        let floor_hits: Vec<_> = out
            .iter()
            .filter(|c| (48..52).contains(&c.feature))
            .collect();
        assert_eq!(floor_hits.len(), 4, "底面 4 角应命中；out={}", out.len());
        for c in floor_hits {
            assert!((c.depth - 0.1).abs() < 1e-4, "depth={}", c.depth);
            assert!((c.normal.y - 1.0).abs() < 1e-4, "normal={:?}", c.normal);
        }
        // 侧面下部角点同样真穿透（0.1）⇒ 总接触数 ≥ 12（6 面 × 4 角中触地者）
        assert!(out.len() >= 12, "out={}", out.len());
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
