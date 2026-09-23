//! fluid_force：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl FluidSystem {
    /// 压力：Tait p = B·((ρ/ρ0)^γ − 1)；γ=7 走整数次幂乘法展开。
    /// `tensile_instability_suppression`（默认开）= Monaghan 自由面钳制
    /// p ≥ 0：表面密度截断（q ≈ 0.5） otherwise 会给出 p ≈ −B，经对称
    /// spiky 形式变成巨大粒子间吸力（单子步 Δv ~ 10² m/s）⇒ 全体炸散。
    /// 覆盖**全部**粒子（含 2b 边界粒子——它们要参与压力对，也吃同一套钳制：
    /// 悬空/真空里的边界粒子 ρ 很小 ⇒ p 钳到 0 ⇒ 不产生凭空吸力）。
    pub(crate) fn pressure_pass(&mut self) {
        let rho0 = self.cfg.rest_density;
        let g = self.cfg.gamma_tait;
        let clamp = self.cfg.tensile_instability_suppression;
        for i in 0..self.pos.len() {
            let q = self.dens[i] / rho0;
            let mut p = if (g - 7.0).abs() < 1e-6 {
                let q2 = q * q;
                let q4 = q2 * q2;
                self.b_tait * (q * q2 * q4 - 1.0)
            } else {
                self.b_tait * (q.powf(g) - 1.0)
            };
            if clamp && p < 0.0 {
                p = 0.0;
            }
            self.press[i] = p;
        }
    }

    /// 力：重力 + 压力梯度（spiky，对称形式 p_i/ρ_i² + p_j/ρ_j²）
    /// 与 XSPH 速度平滑（对称形式 Σ m·2/(ρ_i+ρ_j)·W·(v_j − v_i)；
    /// 系数逐对对称 ⇒ 与压力项一样动量守恒）。
    /// 压力项逐对反对称 ⇒ 零重力下总动量守恒（测试 #3 守门）。
    ///
    /// **2b**：质量逐粒取（`pmass`；流体段与旧常量同值 ⇒ 无边界时逐位不变）；
    /// 邻居 j 为边界粒子时，把**同一对**的作用力（`−F_i`，逐对反对称）累加到
    /// `bforce[j]` —— 这就是刚体受到的压力反作用（力 + 力矩，力矩在
    /// `aggregate_reactions` 里按绕体原点取）。XSPH 对边界邻居照常计入 ⇒
    /// 体面速度把流体拖向自身（无滑移近似，用的是既有那一式）。
    pub(crate) fn force_pass(&mut self) {
        // 并行档（`cfg.threads > 1`）：逐粒独立 ⇒ 分块并行（逐位一致，见 config 注）。
        if self.cfg.threads > 1 {
            self.force_pass_parallel();
            return;
        }
        self.force_pass_serial();
    }

    /// **力相位·并行**（`threads > 1`）：`acc`/`xsph` 逐粒独立 ⇒ 分块并行；
    /// 唯一跨粒子的 `bforce`（2b 反作用）在有边界粒子时走**串行补趟** ⇒ 保逐位一致。
    pub(crate) fn force_pass_parallel(&mut self) {
        let nf = self.n_fluid;
        let nt = self.pos.len();
        let threads = self.cfg.threads.max(1);
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        for f in &mut self.bforce[nf..] {
            *f = Vec3::ZERO;
        }
        {
            let Self {
                pos,
                vel,
                dens,
                press,
                acc,
                xsph,
                pmass,
                grid,
                h,
                h2,
                k6,
                ks,
                ..
            } = self;
            let (h, h2, k6, ks) = (*h, *h2, *k6, *ks);
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, (acc_c, xs_c)) in acc[..nf]
                    .chunks_mut(per)
                    .zip(xsph[..nf].chunks_mut(per))
                    .enumerate()
                {
                    let base = ci * per;
                    let (pos, vel, dens, press, pmass, grid) =
                        (&*pos, &*vel, &*dens, &*press, &*pmass, &*grid);
                    s.spawn(move || {
                        for (k, (a_out, x_out)) in acc_c.iter_mut().zip(xs_c.iter_mut()).enumerate()
                        {
                            let i = base + k;
                            let vi = vel[i];
                            let rho_i = dens[i];
                            let ci2 = press[i] / (rho_i * rho_i);
                            let mut a = g;
                            let mut xs = Vec3::ZERO;
                            grid.for_neighbors_in(pos, h2, i, |j, d, r2| {
                                let rho_j = dens[j];
                                let cj2 = press[j] / (rho_j * rho_j);
                                let r = r2.sqrt();
                                let t = h - r;
                                let mj = pmass[j];
                                let coef = mj * (ks * t * t) * (ci2 + cj2);
                                let denom = r.max(1e-9);
                                a += d * (coef / denom);
                                let vij = vi - vel[j];
                                let vdn = -vij.dot(d);
                                if vdn > 0.0 {
                                    let mu = vdn * h / (r2 + 0.01 * h2);
                                    let c = mj
                                        * (alpha_c * mu / (0.5 * (rho_i + rho_j)))
                                        * (ks * t * t);
                                    a += d * (c / denom);
                                }
                                let tt = h2 - r2;
                                let w = k6 * tt * tt * tt;
                                xs += (vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
                            });
                            *a_out = a;
                            *x_out = xs;
                        }
                    });
                }
            });
        }
        // 2b 反作用（逐粒 bforce）：**串行补趟**（只有边界粒子存在时才走）——
        // 与串行路径同一算式、同一遍历序 ⇒ 逐位一致。缓冲用 `take` 借出以便闭包内写。
        if nt > nf {
            let mut bforce = std::mem::take(&mut self.bforce);
            for i in 0..nf {
                let vi = self.vel[i];
                let rho_i = self.dens[i];
                let ci2 = self.press[i] / (rho_i * rho_i);
                let mi = self.pmass[i];
                self.for_neighbors(i, |j, d, r2| {
                    if j < nf {
                        return;
                    }
                    let rho_j = self.dens[j];
                    let cj2 = self.press[j] / (rho_j * rho_j);
                    let r = r2.sqrt();
                    let t = self.h - r;
                    let mj = self.pmass[j];
                    let coef = mj * (self.ks * t * t) * (ci2 + cj2);
                    let denom = r.max(1e-9);
                    let cv = {
                        let vij = vi - self.vel[j];
                        let vdn = -vij.dot(d);
                        if vdn > 0.0 {
                            let mu = vdn * self.h / (r2 + 0.01 * self.h2);
                            mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (self.ks * t * t)
                        } else {
                            0.0
                        }
                    };
                    bforce[j] -= d * (mi * (coef + cv) / denom);
                });
            }
            self.bforce = bforce;
        }
        self.aggregate_reactions();
    }

    /// **力相位·串行**（默认档：`threads == 1` ⇒ 与历史行为逐位一致）。
    pub(crate) fn force_pass_serial(&mut self) {
        let nf = self.n_fluid;
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        // `for_neighbors` 借 `&self`，闭包里要写边界粒子受力 ⇒ 把缓冲整体借出
        // （`Vec::take` 是 O(1)，不分配），循环后归还。
        let mut bforce = std::mem::take(&mut self.bforce);
        for f in &mut bforce[nf..] {
            *f = Vec3::ZERO; // 每子步重新累加（反作用 = 本子步的力）
        }
        for i in 0..nf {
            let vi = self.vel[i];
            let rho_i = self.dens[i];
            let ci2 = self.press[i] / (rho_i * rho_i);
            let mi = self.pmass[i];
            let mut acc = g;
            let mut xs = Vec3::ZERO;
            self.for_neighbors(i, |j, d, r2| {
                let rho_j = self.dens[j];
                let cj2 = self.press[j] / (rho_j * rho_j);
                let r = r2.sqrt();
                let t = self.h - r;
                let mj = self.pmass[j];
                // a_i += m_j·(p_i/ρ_i² + p_j/ρ_j²)·45/(πh⁶)·(h−r)²·d̂（d̂ 指离 j）。
                let coef = mj * (self.ks * t * t) * (ci2 + cj2);
                let denom = r.max(1e-9);
                acc += d * (coef / denom);
                // Monaghan 人工黏度：仅接近对（v_ij·d < 0），Π = α·c·μ/ρ̄、
                // μ = h·|v_ij·d|/(r² + 0.01h²)；逐对反对称 ⇒ 动量守恒，耗散法向动能。
                let vij = vi - self.vel[j];
                let vdn = -vij.dot(d);
                let cv = if vdn > 0.0 {
                    let mu = vdn * self.h / (r2 + 0.01 * self.h2);
                    let c = mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (self.ks * t * t);
                    acc += d * (c / denom);
                    c
                } else {
                    0.0
                };
                if j >= nf {
                    // 反作用（作用在边界粒子上）= −F_i = −m_i·d·(逐对系数/r)。
                    bforce[j] -= d * (mi * (coef + cv) / denom);
                }
                let tt = self.h2 - r2;
                let w = self.k6 * tt * tt * tt;
                xs += (self.vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
            });
            self.acc[i] = acc;
            self.xsph[i] = xs;
        }
        self.bforce = bforce;
        self.aggregate_reactions();
    }
}
