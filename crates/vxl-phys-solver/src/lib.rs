//! # vxl-phys-solver
//!
//! 刚体约束求解（§2.5/§2.6）：
//! - M0：顺序冲量（warm starting + Baumgarte 位置修正 + 摩擦锥）——即 M0 里程碑
//!   明确要求的「顺序冲量」；TGS-Soft（time-of-impact 分级 + 软约束）在 M1 升级，
//!   本文件求解循环骨架与岛/休眠/缓存层保持不变。
//! - 岛：并查集分岛；岛内约束按 (a, b, 点序) 固定排序（§4.14 确定性模式）。
//! - 休眠（§4.11）：线性 0.04 / 角速 0.05 rad/s、计时 0.5 s，岛级判定。
//! - CCD（§4.12/§2.7）：`ccd` 模块——选择性判据 + 保守推进扫描（M1）。
//! - 并行（§6）：清醒岛分组并行（组内串行、组间体集合不相交），
//!   gather→solve→scatter 走组内 scratch 速度缓冲，与串行 bit 级一致（§5）。

#![forbid(unsafe_code)]

pub mod ccd;
pub mod joints;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, Mat3, PhysConfig};
use vxl_phys_narrow::Manifold;

use vxl_phys_core::Vec3;

// ── 按域拆出的子模块（子目录 src/）
mod cfg;
mod constraint;
mod helpers;
mod island;
mod solver_impl;
mod stats;
mod types;
mod warm;
mod warm_data;
pub use self::{cfg::*, stats::*, types::*};
pub(crate) use self::{constraint::*, helpers::*, island::*, warm::*, warm_data::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{Quat, SerialJobSystem, Shape};
    use vxl_phys_narrow::{ContactPoint, Manifold};

    #[test]
    pub(crate) fn resting_box_velocities_damp_to_zero() {
        // 迷你闭环：每帧按当前位置重建流形（模拟窄相），验证顺序冲量把
        // 下落盒收敛到静置高度（y ≈ 1.0）且速度趋零、最终入睡。
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 2.5, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let g = b.push_static(
            Shape::Box {
                half: Vec3::new(10.0, 0.5, 10.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let cfg = PhysConfig::default();
        let mut solver = ImpulseSolver::new(cfg.contact_skin);
        let dt = cfg.dt;
        // 简化重力（真实管线经力场→积分器；此处直接并入速度积分）。
        let mut y = 2.5f32;
        for _ in 0..300 {
            // 引擎契约：积分器只对清醒体施加重力（沉睡体由岛冻结）。
            if b.awake[0] {
                b.linvel[0] += Vec3::new(0.0, -9.81 * dt, 0.0);
            }
            let depth = 1.0 - y;
            if depth > -cfg.contact_skin {
                let m = Manifold {
                    a: 0,
                    b: g,
                    normal: Vec3::new(0.0, -1.0, 0.0),
                    points: [ContactPoint {
                        point: Vec3::new(0.0, y - 0.5, 0.0),
                        depth,
                        feature: 0,
                    }]
                    .into(),
                };
                solver.solve(&mut b, &[m], &cfg, dt, &SerialJobSystem);
            } else {
                solver.solve(&mut b, &[], &cfg, dt, &SerialJobSystem);
            }
            y += b.linvel[0].y * dt;
            b.position[0].y = y;
        }
        assert!((y - 1.0).abs() < 0.05, "rest y = {y}");
        assert!(b.linvel[0].y.abs() < 0.05, "linvel {:?}", b.linvel[0]);
    }
}
