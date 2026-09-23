//! # vxl-phys-fluid
//!
//! 流体（§4.8，按技术族分档）—— M3 CPU 档 / M4 GPU 档。
//!
//! - CPU SPH：WCSPH + XSPH 黏度（Monaghan 核）；
//! - GPU PBF：Müller 2013，poly6/spiky 核，密度约束迭代 3~4；
//! - GPU FLIP/APIC：PIC/FLIP 传输 + APIC 仿射速度，稀疏哈希网格（512³ 活跃格）；
//! - 刚体耦合：Akinci 边界粒子两层（体积/压力）。
//!
//! 首切片（docs/PLAN-0.3.md）：`CpuSph` 的 WCSPH 路径完整落地——
//! poly6 密度（含自身项）、spiky 压力梯度（中心不减益 ⇒ 张力不稳定抑制）、
//! XSPH 速度平滑、Tait 状态方程（γ=7）、晶格质量标定、半隐式欧拉 + CFL 限速；
//! 邻居 = 均匀网格（格边 = h，27 邻域，遍历序确定）；边界 = 统一提供者通道
//! `ProviderColliders::contacts_point`（体素/三角网/喷溅即插即用，非弹性投影）。
//! 确定性：全 f32、同输入两次运行逐位一致（测试守门）。
//!
//! **刚体→流体（2b，Akinci 两层边界粒子）**：边界粒子与流体粒子**同数组、同核、
//! 同式**（逐粒质量 `pmass`：流体 = `mass`、边界 = `ρ0·V_b`），只是**运动学冻结**
//! （不积分、由 facade 每 tick 按体重建，速度 = 体面速度 `v + ω×r`）。反作用 =
//! 边界粒子上压力梯度力之和 → 每体（力 + 绕体原点的力矩）。
//! 几何（表面采样/两层内移/体积）在 [`boundary`] 模块。**无边界粒子时逐位不变**
//! （`sum_b` 恒 0 ⇒ `mass·sum + 0.0` 与原式同值同序）。

#![forbid(unsafe_code)]

pub mod boundary;

pub use boundary::{lattice, BoundaryLattice, SurfaceLattice};

use vxl_phys_core::interop::{InteropContact, MediumField, MediumSample, ProviderColliders};
use vxl_phys_core::{Quat, Shape, Vec3};

// ── 按域拆出的子模块（子目录 src/）
mod config;
mod fluid_access;
mod fluid_boundary;
mod fluid_density;
mod fluid_force;
mod fluid_medium;
mod fluid_step;
mod grid;
mod system;
mod types;
pub use self::{config::*, grid::*, system::*, types::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests;

/// **微探针：邻居相位的"访存收集 vs 算术"账**（`#[ignore]`，只打印）——决定第三刀
/// （按格序重排粒子存储）值不值得动**公开数组序**。
///
/// 做法：同一份邻居表、同一遍历序，跑两个循环：
/// ① **收集版**：只碰 `pos/vel/dens/press/pmass` 五个数组并把值累加（几乎无算术）；
/// ② **算术版**：用同样收集到的值做真实力相位的那套运算（sqrt/乘加/分支）。
/// 两者之差 ≈ 纯算术成本；收集版本身 ≈ 访存成本。
/// ⇒ 若"收集 ≫ 算术" ⇒ 重排（缓存局部性）是杠杆；否则 SIMD 才是。
#[test]
#[ignore = "微探针（只打印）：访存 vs 算术账；见上注"]
pub(crate) fn micro_probe_gather_vs_compute() {
    use std::time::Instant;
    let n = 50usize; // 125k 粒（够大又不至于跑很久）
    let spacing = 0.05f32;
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders); // 建网格
    let h2 = f.h2;
    let (h, k6, ks) = (f.h, f.k6, f.ks);
    let alpha_c = f.cfg.artificial_viscosity * f.cfg.sound_speed;
    let nf = f.n_fluid;
    // ① 收集版
    let t0 = Instant::now();
    let mut sink = 0.0f32;
    for i in 0..nf {
        let vi = f.vel[i];
        let rho_i = f.dens[i];
        f.for_neighbors(i, |j, d, r2| {
            sink += f.pos[j].x + f.vel[j].y + f.dens[j] + f.press[j] + f.pmass[j];
            sink += d.x + r2;
            sink += vi.y + rho_i;
        });
    }
    let gather = t0.elapsed().as_secs_f64() * 1e3;
    // ② 算术版（真实力相位的算式，但结果丢弃）
    let t0 = Instant::now();
    let mut sink2 = 0.0f32;
    for i in 0..nf {
        let vi = f.vel[i];
        let rho_i = f.dens[i];
        let ci2 = f.press[i] / (rho_i * rho_i);
        let mut a = Vec3::ZERO;
        let mut xs = Vec3::ZERO;
        f.for_neighbors(i, |j, d, r2| {
            let rho_j = f.dens[j];
            let cj2 = f.press[j] / (rho_j * rho_j);
            let r = r2.sqrt();
            let t = h - r;
            let mj = f.pmass[j];
            let coef = mj * (ks * t * t) * (ci2 + cj2);
            let denom = r.max(1e-9);
            a += d * (coef / denom);
            let vij = vi - f.vel[j];
            let vdn = -vij.dot(d);
            if vdn > 0.0 {
                let mu = vdn * h / (r2 + 0.01 * h2);
                let c = mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ks * t * t);
                a += d * (c / denom);
            }
            let tt = h2 - r2;
            let w = k6 * tt * tt * tt;
            xs += (f.vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
        });
        sink2 += a.x + xs.y;
    }
    let compute = t0.elapsed().as_secs_f64() * 1e3;
    println!(
        "微探针（{nf} 粒）：收集版 {gather:.1} ms | 算术版 {compute:.1} ms | 算术占比 {:.0}% | sink {sink:.0}/{sink2:.0}",
        100.0 * compute / (gather + compute)
    );
}
