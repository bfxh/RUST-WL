//! fluid_medium：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// **介质场：`MediumField` 的第一个真实现**（ADR 0008/0009 的交互通道；`ROUTE.md` §4
/// 「刚体↔液体」格的第一刀 2a）。
///
/// 语义（只读采样；**不改流场、不改哈希**）：
/// - `sample(x)`：在 `x` 处按 **poly6 核**对 27 邻域求和，给出
///   `density = m·Σ_j W(r_ij)`（与流体内部的密度定义**同核同式**，只是不含自身项与鬼影项）、
///   `velocity = Σ w_j v_j / Σ w_j`（Shepard 平均 ⇒ 均匀流场下逐位精确）、
///   `occupied = clamp(ρ/ρ0, 0, 1)`（自由表面判据）。无近邻 ⇒ [`MediumSample::VACUUM`]。
/// - `deposit(…)`：**显式空实现**（不静默降级）——反作用要等 2b 的 Akinci 边界粒子把
///   「体↔流体」的动量交换喂回流场；在那之前，本域对刚体只是**单向的读数来源**。
/// - `viscosity`/`temperature` 报 0：XSPh 的 `ε` 是**无量纲**系数、不是 Pa·s，
///   本实现**不编造换算常数**（耦合侧用"密度 + 流速"算阻力即可，别依赖这个字段）。
impl MediumField for FluidSystem {
    fn sample(&self, x: Vec3) -> MediumSample {
        if self.pos.is_empty() || self.grid.items.is_empty() || self.grid.nz == 0 {
            return MediumSample::VACUUM;
        }
        let (bx, by, bz) = self.grid.bin_of(x);
        let (nx, ny, nz) = (self.grid.nx, self.grid.ny, self.grid.nz);
        let (mut wsum, mut rho) = (0.0f32, 0.0f32);
        let mut vsum = Vec3::ZERO;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let (ix, iy, iz) = (bx as i64 + dx, by as i64 + dy, bz as i64 + dz);
                    if ix < 0
                        || iy < 0
                        || iz < 0
                        || ix >= nx as i64
                        || iy >= ny as i64
                        || iz >= nz as i64
                    {
                        continue;
                    }
                    let idx =
                        ((ix as usize) * ny as usize + iy as usize) * nz as usize + iz as usize;
                    for &j in self.grid.bin_items(idx) {
                        let j = j as usize;
                        // **只认流体粒子**：本方法答的是"此处流体如何"，边界粒子（2b）
                        // 是固体侧的代表粒子，计进来会把"体内部"读成"满水位"
                        // （2a 的 `occupied` 会失真）。密度轮才计它们（那是流体自己的密度）。
                        if j >= self.n_fluid {
                            continue;
                        }
                        let p = self.pos[j];
                        let d = p - x;
                        let r2 = d.length_squared();
                        if r2 > self.h2 {
                            continue;
                        }
                        let t = self.h2 - r2;
                        let w = self.k6 * t * t * t;
                        wsum += w;
                        rho += self.mass * w;
                        vsum += self.vel[j] * w;
                    }
                }
            }
        }
        if wsum <= 0.0 {
            return MediumSample::VACUUM;
        }
        let inv = 1.0 / wsum;
        MediumSample {
            density: rho,
            velocity: vsum * inv,
            viscosity: 0.0,
            temperature: 0.0,
            occupied: (rho / self.cfg.rest_density).clamp(0.0, 1.0),
        }
    }

    fn deposit(&mut self, _x: Vec3, _momentum: Vec3, _mass: f32, _pressure_work: f32) {
        // 2b 的动量交换已在 `force_pass` 里**按对**完成（边界粒子与流体同核同式，
        // 逐对反对称 ⇒ 反作用自动等于流体受力的负值），不需要点式沉积。
        // 本方法仍是**显式**空实现：软体/布等"真点式"受体落地前，不做假装耦合。
    }
}
