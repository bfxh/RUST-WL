//! fluid_density：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

mod wall_gather;

impl FluidSystem {
    /// 密度：ρ_i = m·(W(0) + Σ_j W(r_ij) + Σ_ghost W)（poly6，含自身项）。
    /// 边界镜像鬼影：壁邻粒子（0 < sdf < h）把真实邻居关于壁面（接触点 +
    /// 外法线）反射，补回固体一侧的**离散**核质量。不用连续半空间积分——
    /// 它按均匀连续介质补，而流体实际的近壁分布（沉降后成层、各向异性）
    /// 的离散亏量与之错带（实测底层差 ~12% ρ0，补不齐 ⇒ p≥0 死区复活）。
    /// 鬼影随流体局部分布同步：流体压缩/成层，鬼影同压缩/成层。
    ///
    /// **2b 边界粒子**（同数组、索引 ≥ `n_fluid`）：质量**逐粒**取（`pmass`）、
    /// 计入 `sum_b`。流体段的和式与遍历序与旧实现逐字相同 ⇒ **无边界粒子时
    /// `dens = mass·sum + 0.0` 与原 `dens = mass·sum` 逐位同值**（正数 +0.0 精确）。
    pub(crate) fn density_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.cfg.threads > 1 {
            self.density_pass_parallel(providers);
            return;
        }
        let nf = self.n_fluid;
        let nt = self.pos.len();
        for i in 0..nf {
            let pi = self.pos[i];
            let mut pl_pt = [Vec3::ZERO; 8];
            let mut pl_n = [Vec3::ZERO; 8];
            let np = self.wall_planes(pi, providers, &mut pl_pt, &mut pl_n);
            let mut sum = self.w0;
            let mut sum_b = 0.0f32;
            self.for_neighbors(i, |j, d, r2| {
                let t = self.h2 - r2;
                let w = self.k6 * t * t * t;
                if j < nf {
                    sum += w;
                } else {
                    sum_b += self.pmass[j] * w;
                }
                for k in 0..np {
                    let pj = pi - d;
                    let dn = (pj - pl_pt[k]).dot(pl_n[k]);
                    let g = pj - pl_n[k] * (2.0 * dn);
                    let rg2 = (pi - g).length_squared();
                    if rg2 <= self.h2 {
                        let tg = self.h2 - rg2;
                        sum += self.k6 * tg * tg * tg;
                    }
                }
            });
            self.dens[i] = self.mass * sum + sum_b;
        }
        // 边界粒子（2b）：**同一核、同一式**、含自身项，但密度只从**流体**取
        // （`j < nf`）——这是 Akinci 口径的"边界压力 = 外推的流体压力"：若让边界
        // 粒子互相供密度，**薄体**（厚度 < ~2h）两侧的内移层会在体内互相穿透，
        // ρ_b 被自身堆积顶到 q≫1 ⇒ p_b 爆抬 ⇒ 体积力/反作用整片失真（实测：
        // 0.12 m 盒给出 ρVg 的 5.9×、侧向 51 N 的假力，2026-09-22）。
        // 只吃流体 ⇒ 贴壁处 ρ_b ≈ 该处流体密度 ⇒ p_b ≈ p_f，与"冻结流体粒子"
        // 的原意一致，且对任意薄厚都稳定。自身项保留（与流体同式）。
        for i in nf..nt {
            let mut sum = 0.0f32;
            self.for_neighbors(i, |j, _d, r2| {
                if j >= nf {
                    return; // 不吃边界-边界对（见上）
                }
                let t = self.h2 - r2;
                sum += self.pmass[j] * (self.k6 * t * t * t);
            });
            self.dens[i] = self.pmass[i] * self.w0 + sum;
        }
    }

    /// 收集粒子 `pi` 的 h 带内壁面（点 + 外法线；角部可有多面），≤ 8 面。
    /// 供密度轮的镜像鬼影用（流体粒子专有；边界粒子不用，见 `density_pass`）。
    /// **自由函数形态**（并行密度相位在分块闭包里调用；`scratch` 由调用方给，
    /// 并行时每块一份 ⇒ 无共享可变状态）。
    pub fn wall_planes_in(
        boundaries: &[u32],
        h: f32,
        pi: Vec3,
        providers: &dyn ProviderColliders,
        scratch: &mut Vec<InteropContact>,
        pl_pt: &mut [Vec3; 8],
        pl_n: &mut [Vec3; 8],
    ) -> usize {
        if boundaries.is_empty() {
            return 0;
        }
        let mut np = 0usize;
        for &bid in boundaries {
            if let Some(bb) = providers.bounds(bid) {
                // 预滤余量取 h（镜像带 = sdf < h）。
                let m = h;
                if pi.x < bb.min.x - m
                    || pi.x > bb.max.x + m
                    || pi.y < bb.min.y - m
                    || pi.y > bb.max.y + m
                    || pi.z < bb.min.z - m
                    || pi.z > bb.max.z + m
                {
                    continue;
                }
            }
            scratch.clear();
            if providers.contacts_point(bid, pi, h, scratch) {
                for c in scratch.iter() {
                    // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，
                    // provider 无关）；穿透（≤ 0）交给投影，不补。
                    let sdf = (pi - c.point).dot(c.normal);
                    if sdf > 0.0 && sdf < h && np < 8 {
                        pl_pt[np] = c.point;
                        pl_n[np] = c.normal;
                        np += 1;
                    }
                }
            }
        }
        np
    }

    /// **密度相位·并行**（`threads > 1`）：逐粒独立（每粒只读他人的 pos/pmass、
    /// 只写自己的 `dens[i]`）⇒ 分块并行；壁面查询用**每块一份 scratch**
    /// （无共享可变状态）⇒ 结果与串行**逐位一致**。
    pub(crate) fn density_pass_parallel(&mut self, providers: &dyn ProviderColliders) {
        let nf = self.n_fluid;
        let threads = self.cfg.threads.max(1);
        {
            let Self {
                pos,
                dens,
                pmass,
                grid,
                h,
                h2,
                k6,
                w0,
                mass,
                boundaries,
                ..
            } = self;
            let (h, h2, k6, w0, mass) = (*h, *h2, *k6, *w0, *mass);
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, d_out) in dens[..nf].chunks_mut(per).enumerate() {
                    let base = ci * per;
                    let (pos, pmass, grid, boundaries) = (&*pos, &*pmass, &*grid, &*boundaries);
                    s.spawn(move || {
                        let mut scratch: Vec<InteropContact> = Vec::new();
                        for (k, d_out) in d_out.iter_mut().enumerate() {
                            let i = base + k;
                            let pi = pos[i];
                            let mut pl_pt = [Vec3::ZERO; 8];
                            let mut pl_n = [Vec3::ZERO; 8];
                            let np = Self::wall_planes_in(
                                boundaries,
                                h,
                                pi,
                                providers,
                                &mut scratch,
                                &mut pl_pt,
                                &mut pl_n,
                            );
                            let mut sum = w0;
                            let mut sum_b = 0.0f32;
                            grid.for_neighbors_in(pos, h2, i, |j, d, r2| {
                                let t = h2 - r2;
                                let w = k6 * t * t * t;
                                if j < nf {
                                    sum += w;
                                } else {
                                    sum_b += pmass[j] * w;
                                }
                                for kk in 0..np {
                                    let pj = pi - d;
                                    let dn = (pj - pl_pt[kk]).dot(pl_n[kk]);
                                    let g = pj - pl_n[kk] * (2.0 * dn);
                                    let rg2 = (pi - g).length_squared();
                                    if rg2 <= h2 {
                                        let tg = h2 - rg2;
                                        sum += k6 * tg * tg * tg;
                                    }
                                }
                            });
                            *d_out = mass * sum + sum_b;
                        }
                    });
                }
            });
        }
        // 边界粒子（2b）：同一式、只吃流体邻居（见串行路径注）。
        self.density_boundary_pass(nf, threads);
    }

    /// **边界粒子（2b）的密度**（并行档）：与流体段无关的另一批索引 ⇒ 单独一趟。
    /// 只吃**流体**邻居（Akinci 口径，理由见 `density_pass` 的注）。
    fn density_boundary_pass(&mut self, nf: usize, threads: usize) {
        let nt = self.pos.len();
        if nt <= nf {
            return;
        }
        let Self {
            pos,
            dens,
            pmass,
            grid,
            h2,
            k6,
            w0,
            ..
        } = self;
        let (h2, k6, w0) = (*h2, *k6, *w0);
        let nb = nt - nf;
        let per = nb.div_ceil(threads).max(1);
        std::thread::scope(|s| {
            for (ci, d_out) in dens[nf..].chunks_mut(per).enumerate() {
                let base = nf + ci * per;
                let (pos, pmass, grid) = (&*pos, &*pmass, &*grid);
                s.spawn(move || {
                    for (k, d_out) in d_out.iter_mut().enumerate() {
                        let i = base + k;
                        let mut sum = 0.0f32;
                        grid.for_neighbors_in(pos, h2, i, |j, _d, r2| {
                            if j >= nf {
                                return;
                            }
                            let t = h2 - r2;
                            sum += pmass[j] * (k6 * t * t * t);
                        });
                        *d_out = pmass[i] * w0 + sum;
                    }
                });
            }
        });
    }

    pub(crate) fn wall_planes(
        &mut self,
        pi: Vec3,
        providers: &dyn ProviderColliders,
        pl_pt: &mut [Vec3; 8],
        pl_n: &mut [Vec3; 8],
    ) -> usize {
        if self.boundaries.is_empty() {
            return 0;
        }
        let mut np = 0usize;
        for &bid in &self.boundaries {
            if let Some(bb) = providers.bounds(bid) {
                // 预滤余量取 h（镜像带 = sdf < h）。
                let m = self.h;
                if pi.x < bb.min.x - m
                    || pi.x > bb.max.x + m
                    || pi.y < bb.min.y - m
                    || pi.y > bb.max.y + m
                    || pi.z < bb.min.z - m
                    || pi.z > bb.max.z + m
                {
                    continue;
                }
            }
            self.contacts.clear();
            if providers.contacts_point(bid, pi, self.h, &mut self.contacts) {
                for c in &self.contacts {
                    // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，
                    // provider 无关）；穿透（≤ 0）交给投影，不补。
                    let sdf = (pi - c.point).dot(c.normal);
                    if sdf > 0.0 && sdf < self.h && np < 8 {
                        pl_pt[np] = c.point;
                        pl_n[np] = c.normal;
                        np += 1;
                    }
                }
            }
        }
        np
    }
}
