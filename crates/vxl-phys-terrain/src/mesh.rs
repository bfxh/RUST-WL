//! **网格域（任意三角网）**：静态关卡几何的提供者（ROUTE §3 域表最后一块）。
//!
//! 表示：顶点 + 三角索引（可含重复顶点/非流形——只做「最近面」查询，不做拓扑）。
//! 加速：均匀网格（桶边长 = max(均边, `MIN_BIN`)，**三角形按其 AABB 登记进所有覆盖桶**
//! ⇒ 单桶查询即可覆盖「覆盖查询点」的三角形；再查 3×3×3 邻域覆盖「近而不覆盖」的）。
//!
//! 接触模型（薄壳，与体素 SDF 不同的语义，文档内明示）：
//! - `depth = skin − dist(p, 最近三角形)`（正 = 已在面里侧）；法线 = 该三角形**面法线**
//!   （朝向由三角形顶点序决定 ⇒ 关卡导出时朝外，与 glTF/OBJ 惯例一致）。
//! - 无内外判定（薄壳语义）：远离面即无接触；穿到面背后会被面法线推出（单向屏障）。
//!
//! 精度前提：查询只保证「距离 ≤ `bin/2` 的最近面」正确 ⇒ 皮肤带应 < `bin/2`
//! （默认 `skin ≈ 0.02`、`bin ≥ 0.5` 时余量充足）。确定性：顶点/三角形序决定一切，
//! 无 HashMap 迭代序依赖。

use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Mat3, Quat, Vec3};

/// 桶边长下限（m）：保证「皮肤带 < bin/2」的精度前提在细网格上也成立。
pub const MIN_BIN: f32 = 0.5;

/// 三角形网格（静态关卡）。
#[derive(Clone, Debug)]
pub struct TriMesh {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    /// 面法线（单位；建时算好，查询零重算）。
    normals: Vec<Vec3>,
    grid: Option<MeshGrid>,
}

#[derive(Clone, Debug)]
struct MeshGrid {
    origin: Vec3,
    bin: f32,
    dims: (u32, u32, u32),
    /// 桶 → 三角形索引（按三角形序登记 ⇒ 桶内升序 ⇒ 确定性）。
    bins: Vec<Vec<u32>>,
}

/// 建桶的格数上限（超限退回全扫；防内存灾难）。
const GRID_MAX_BINS: u64 = 1 << 21;

impl TriMesh {
    /// 建网（顶点 + 三角索引；索引越界的三角形被跳过——不 panic，如实丢弃）。
    pub fn new(verts: Vec<Vec3>, tris: Vec<[u32; 3]>) -> Self {
        let n = verts.len() as u32;
        let mut kept: Vec<[u32; 3]> = Vec::with_capacity(tris.len());
        let mut normals: Vec<Vec3> = Vec::with_capacity(tris.len());
        for t in tris {
            if t[0] >= n || t[1] >= n || t[2] >= n {
                continue;
            }
            let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
            let nr = (b - a).cross(c - a);
            let ln = nr.length();
            let nrm = if ln > 1e-12 { nr * (1.0 / ln) } else { Vec3::Y };
            kept.push(t);
            normals.push(nrm);
        }
        Self {
            verts,
            tris: kept,
            normals,
            grid: None,
        }
    }

    /// 平面片（两个三角形；`n` 为朝外法线，`u`/`v` 为面内基）。
    pub fn quad(center: Vec3, u: Vec3, v: Vec3, n: Vec3) -> Self {
        let n = n.normalize();
        let (u, v) = (u, v);
        let p = |su: f32, sv: f32| center + u * su + v * sv;
        let verts = vec![p(-1.0, -1.0), p(1.0, -1.0), p(1.0, 1.0), p(-1.0, 1.0)];
        // 顶点序使面法线 = +n（右手）
        let tris = if (verts[1] - verts[0]).cross(verts[2] - verts[0]).dot(n) > 0.0 {
            vec![[0, 1, 2], [0, 2, 3]]
        } else {
            vec![[0, 2, 1], [0, 3, 2]]
        };
        Self::new(verts, tris)
    }

    #[inline]
    pub fn verts(&self) -> &[Vec3] {
        &self.verts
    }

    #[inline]
    pub fn tris(&self) -> &[[u32; 3]] {
        &self.tris
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }

    /// 世界包围盒（含 `margin` 膨胀）。
    /// 注意与 `ProviderColliders::bounds(id)` 同名不同签名 ⇒ 用独立名避免遮蔽。
    pub fn world_bounds(&self) -> Aabb {
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for v in &self.verts {
            lo = lo.min(*v);
            hi = hi.max(*v);
        }
        if self.verts.is_empty() {
            return Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            };
        }
        Aabb { min: lo, max: hi }
    }

    /// 建/重建加速网格（桶边长 = max(均边, [`MIN_BIN`])）。
    /// 幂等（同输入同结果）；三角形按其 AABB 登记进所有覆盖桶。
    pub fn build_grid(&mut self) {
        self.grid = None;
        if self.tris.is_empty() {
            return;
        }
        // 均边（确定性：顶点序）
        let mut sum = 0.0f32;
        let mut cnt = 0usize;
        for t in &self.tris {
            let (a, b, c) = (
                self.verts[t[0] as usize],
                self.verts[t[1] as usize],
                self.verts[t[2] as usize],
            );
            sum += (b - a).length() + (c - b).length() + (a - c).length();
            cnt += 3;
        }
        let mean_edge = if cnt > 0 { sum / cnt as f32 } else { MIN_BIN };
        let bin = mean_edge.max(MIN_BIN);
        let b = self.world_bounds();
        let dims = (
            ((b.max.x - b.min.x) / bin).ceil() as u32 + 1,
            ((b.max.y - b.min.y) / bin).ceil() as u32 + 1,
            ((b.max.z - b.min.z) / bin).ceil() as u32 + 1,
        );
        let n_bins = dims.0 as u64 * dims.1 as u64 * dims.2 as u64;
        if n_bins == 0 || n_bins > GRID_MAX_BINS {
            return;
        }
        let inv_bin = 1.0 / bin;
        let mut bins: Vec<Vec<u32>> = vec![Vec::new(); n_bins as usize];
        for (ti, t) in self.tris.iter().enumerate() {
            let (a, c, d) = (
                self.verts[t[0] as usize],
                self.verts[t[1] as usize],
                self.verts[t[2] as usize],
            );
            let lo = a.min(c).min(d);
            let hi = a.max(c).max(d);
            let b0 = (
                (((lo.x - b.min.x) * inv_bin).floor().max(0.0)) as u32,
                (((lo.y - b.min.y) * inv_bin).floor().max(0.0)) as u32,
                (((lo.z - b.min.z) * inv_bin).floor().max(0.0)) as u32,
            );
            let b1 = (
                (((hi.x - b.min.x) * inv_bin).ceil() as u32).min(dims.0 - 1),
                (((hi.y - b.min.y) * inv_bin).ceil() as u32).min(dims.1 - 1),
                (((hi.z - b.min.z) * inv_bin).ceil() as u32).min(dims.2 - 1),
            );
            for bx in b0.0..=b1.0 {
                for by in b0.1..=b1.1 {
                    for bz in b0.2..=b1.2 {
                        let i = ((bx * dims.1 + by) * dims.2 + bz) as usize;
                        bins[i].push(ti as u32);
                    }
                }
            }
        }
        self.grid = Some(MeshGrid {
            origin: b.min,
            bin,
            dims,
            bins,
        });
    }

    /// 最近面查询：返回 `(距离, 面上最近点, 面法线, 三角形序号)`；空网格返回 None。
    /// 有网格时只搜查询点所在桶的 3×3×3 邻域（覆盖「覆盖点」与「近而不覆盖」两类面）。
    ///
    /// **桶级距离剪枝**（2026-09-15）：`max_d` = 调用方关心的最大距离——桶 AABB 到
    /// 查询点超过它则整桶跳过。两个调用方（点/球）本就丢弃 `depth < −skin`
    /// （等价 `dist > max_d`），故剪枝不改变语义；实测大网格（bin ≈3.75 m、
    /// 3×3×3 ≈100 三角形/查询）下窄相耗时大幅下降。`f32::INFINITY` = 不剪枝
    /// （测试里的精确对照用）。
    fn closest(&self, p: Vec3, max_d: f32) -> Option<(f32, Vec3, Vec3, usize)> {
        let mut best: Option<(f32, Vec3, Vec3, usize)> = None;
        let consider = |ti: usize, best: &mut Option<(f32, Vec3, Vec3, usize)>| {
            let t = self.tris[ti];
            let (a, b, c) = (
                self.verts[t[0] as usize],
                self.verts[t[1] as usize],
                self.verts[t[2] as usize],
            );
            let q = closest_on_tri(p, a, b, c);
            let d = (p - q).length();
            if best.is_none_or(|(bd, _, _, _)| d < bd) {
                *best = Some((d, q, self.normals[ti], ti));
            }
        };
        match &self.grid {
            Some(g) => {
                let cx = ((p.x - g.origin.x) / g.bin).floor();
                let cy = ((p.y - g.origin.y) / g.bin).floor();
                let cz = ((p.z - g.origin.z) / g.bin).floor();
                if cx < -1.0 || cy < -1.0 || cz < -1.0 {
                    // 远在场外：全扫（保守；上层有机体 AABB 早已筛掉绝大多数）
                    for ti in 0..self.tris.len() {
                        consider(ti, &mut best);
                    }
                    return best;
                }
                let (cx, cy, cz) = (cx as i64, cy as i64, cz as i64);
                for dx in -1i64..=1 {
                    for dy in -1i64..=1 {
                        for dz in -1i64..=1 {
                            let (x, y, z) = (cx + dx, cy + dy, cz + dz);
                            if x < 0 || y < 0 || z < 0 {
                                continue;
                            }
                            let (x, y, z) = (x as u32, y as u32, z as u32);
                            if x >= g.dims.0 || y >= g.dims.1 || z >= g.dims.2 {
                                continue;
                            }
                            let i = ((x * g.dims.1 + y) * g.dims.2 + z) as usize;
                            if g.bins[i].is_empty() {
                                continue;
                            }
                            // 桶级剪枝：桶 AABB 到查询点的距离 > max_d ⇒ 整桶跳过
                            // （桶内任一三角形只会更远）。
                            if max_d.is_finite() {
                                let lo = Vec3::new(
                                    g.origin.x + x as f32 * g.bin,
                                    g.origin.y + y as f32 * g.bin,
                                    g.origin.z + z as f32 * g.bin,
                                );
                                let hi = lo + Vec3::splat(g.bin);
                                let d2 = Vec3::new(
                                    (lo.x - p.x).max(p.x - hi.x).max(0.0),
                                    (lo.y - p.y).max(p.y - hi.y).max(0.0),
                                    (lo.z - p.z).max(p.z - hi.z).max(0.0),
                                )
                                .length_squared();
                                if d2 > max_d * max_d {
                                    continue;
                                }
                            }
                            for &ti in &g.bins[i] {
                                consider(ti as usize, &mut best);
                            }
                        }
                    }
                }
                best
            }
            None => {
                for ti in 0..self.tris.len() {
                    consider(ti, &mut best);
                }
                best
            }
        }
    }
}

/// 三角形上离 `p` 最近的点（含内部/边/顶点三种情况；Ericson 的标准闭式解）。
fn closest_on_tri(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

impl ProviderColliders for TriMesh {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(self.world_bounds())
    }

    /// 点查询：`depth = skin − dist`；法线 = 面法线。
    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        let Some((d, q, n, ti)) = self.closest(p, skin) else {
            return true;
        };
        let depth = skin - d;
        if depth < -skin {
            return true; // 支持查询；不在带内
        }
        out.push(InteropContact {
            point: q,
            normal: n,
            depth,
            feature: (ti as u32) + 1,
        });
        true
    }

    /// 球查询：`depth = r − dist(球心)`（球心最近面 + 面法线）。
    fn contacts_sphere(
        &self,
        _id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let Some((d, q, n, ti)) = self.closest(center, radius + skin) else {
            return true;
        };
        let depth = radius - d;
        if depth < -skin {
            return true;
        }
        out.push(InteropContact {
            point: q,
            normal: n,
            depth,
            feature: (ti as u32) + 1,
        });
        true
    }

    /// 盒查询：8 顶点 + 6 面心（14 点采样；多点面接触稳定）。
    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let m = Mat3::from_quat(rot);
        let mut any = false;
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    let w = pos + m.mul_vec3(Vec3::new(sx * half.x, sy * half.y, sz * half.z));
                    any |= self.contacts_point(id, w, skin, out);
                }
            }
        }
        for axis in 0..3 {
            let h = match axis {
                0 => Vec3::new(half.x, 0.0, 0.0),
                1 => Vec3::new(0.0, half.y, 0.0),
                _ => Vec3::new(0.0, 0.0, half.z),
            };
            for s in [-1.0f32, 1.0] {
                any |= self.contacts_point(id, pos + m.mul_vec3(h * s), skin, out);
            }
        }
        any
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 水平地面：2×2 的四边形（y = 0，法线 +Y），中心在原点。
    fn ground() -> TriMesh {
        TriMesh::quad(Vec3::ZERO, Vec3::X, Vec3::Z, Vec3::Y)
    }

    #[test]
    fn point_query_depth_and_normal() {
        let mut m = ground();
        m.build_grid();
        let mut out = Vec::new();
        // 面内一点：距离 0 ⇒ depth = skin
        assert!(m.contacts_point(0, Vec3::new(0.3, 0.0, -0.2), 0.05, &mut out));
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.05).abs() < 1e-5, "d={}", out[0].depth);
        assert!(out[0].normal.y > 0.99, "n={:?}", out[0].normal);
        // 面上方 0.12：带外（skin 0.05 ⇒ 界限 2·skin = 0.1）⇒ 无接触
        out.clear();
        m.contacts_point(0, Vec3::new(0.0, 0.12, 0.0), 0.05, &mut out);
        assert!(out.is_empty());
        // 面上方 0.02：带内 ⇒ depth = 0.05 − 0.02 = 0.03
        out.clear();
        m.contacts_point(0, Vec3::new(0.0, 0.02, 0.0), 0.05, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.03).abs() < 1e-5, "d={}", out[0].depth);
    }

    #[test]
    fn grid_matches_brute_force_closest() {
        // 12×12 小格子斜坡网（均边 ≈ 0.18 < MIN_BIN ⇒ 桶按 MIN_BIN）
        let mut verts = Vec::new();
        let mut tris = Vec::new();
        let n = 12u32;
        for iy in 0..=n {
            for ix in 0..=n {
                let x = ix as f32 * 0.2 - 1.2;
                let z = iy as f32 * 0.2 - 1.2;
                verts.push(Vec3::new(x, 0.15 * x + 0.1 * z, z));
            }
        }
        for iy in 0..n {
            for ix in 0..n {
                let i = iy * (n + 1) + ix;
                tris.push([i, i + 1, i + n + 1]);
                tris.push([i + 1, i + n + 2, i + n + 1]);
            }
        }
        let mut with_grid = TriMesh::new(verts.clone(), tris.clone());
        with_grid.build_grid();
        let brute = TriMesh::new(verts, tris); // 不建网格 ⇒ 全扫
        // 采样点（含面内/面外/远处）
        let mut st: u32 = 0x2545_f491;
        for _ in 0..200 {
            st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let t = (st >> 8) as f32 / (1u32 << 24) as f32;
            st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let u = (st >> 8) as f32 / (1u32 << 24) as f32;
            let p = Vec3::new(t * 2.4 - 1.2, t * 0.6 - 0.2, u * 2.4 - 1.2);
            let (d1, q1, _, t1) = with_grid.closest(p, f32::INFINITY).unwrap();
            let (d2, q2, _, t2) = brute.closest(p, f32::INFINITY).unwrap();
            assert!((d1 - d2).abs() < 1e-4, "距离不一致 {d1} vs {d2} @ {p:?}");
            assert!((q1 - q2).length() < 1e-3, "最近点不一致 @ {p:?}");
            let _ = (t1, t2); // 等距多面时序号可能不同（并列），只比几何量
        }
    }

    #[test]
    fn sphere_query_on_slope() {
        let mut m = ground();
        m.build_grid();
        let mut out = Vec::new();
        // 球心在面上方 0.25、半径 0.3 ⇒ depth = 0.3 − 0.25 = 0.05
        assert!(m.contacts_sphere(0, Vec3::new(0.2, 0.25, 0.2), 0.3, 0.05, &mut out));
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.05).abs() < 1e-5, "d={}", out[0].depth);
    }

    #[test]
    fn box_query_box_query_four_corners() {
        let mut m = ground();
        m.build_grid();
        let mut out = Vec::new();
        // 盒底面与地面齐平（中心 y = 0.2、半长 0.2）⇒ 底面 4 顶点接触
        m.contacts_box(
            0,
            Vec3::splat(0.2),
            Vec3::new(0.0, 0.2, 0.0),
            Quat::IDENTITY,
            0.05,
            &mut out,
        );
        assert!(out.len() >= 4, "底四角应都接触：{}", out.len());
        assert!(out.iter().all(|c| c.normal.y > 0.99));
    }

    #[test]
    fn degenerate_triangle_is_skipped_not_panicking() {
        let m = TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::X],
            vec![[0, 1, 2], [0, 1, 9]], // 第二个索引越界 ⇒ 丢弃
        );
        assert_eq!(m.tris().len(), 1);
        assert!(m.closest(Vec3::Y, f32::INFINITY).is_some());
    }
}
