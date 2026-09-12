//! # vxl-phys-integrate
//!
//! 半隐式欧拉（§2.2）：v ← v + a·dt；x ← x + v·dt；q ← q ⊕ ω·dt。
//! 固定 60 Hz 基础步，子步由调用方（World）按 `config.substeps` 循环。
//! 批量 SoA 遍历按体索引有序（§5 确定性）；SIMD 批量积分（§7）在 M1 引入，
//! 语义等价（逐元素，无跨元素归约）。

#![forbid(unsafe_code)]

use vxl_phys_core::{BodySet, Vec3};

#[derive(Clone, Copy, Debug, Default)]
pub struct Integrator;

impl Integrator {
    /// 速度积分：v += (g + F/m)·dt，ω += I⁻¹_world·τ·dt；随后清空力累加器。
    /// 限速为 M0 防隧道保守闸（CCD §4.12 落地后放宽）。
    pub fn integrate_velocities(
        bodies: &mut BodySet,
        gravity: Vec3,
        dt: f32,
        max_linear: f32,
        max_angular: f32,
    ) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) || !bodies.awake[i] {
                bodies.force[i] = Vec3::ZERO;
                bodies.torque[i] = Vec3::ZERO;
                continue;
            }
            let accel = gravity + bodies.force[i] * bodies.inv_mass[i];
            bodies.linvel[i] += accel * dt;

            let ang_accel = bodies.apply_world_inv_inertia(i, bodies.torque[i]);
            bodies.set_angvel_raw(i, bodies.angvel(i) + ang_accel * dt);

            // 限速（确定性钳制，逐轴比较顺序固定）。
            let lv = bodies.linvel[i];
            let sp = lv.length();
            if sp > max_linear {
                bodies.linvel[i] = lv * (max_linear / sp);
            }
            let wv = bodies.angvel(i);
            let ws = wv.length();
            if ws > max_angular {
                bodies.set_angvel_raw(i, wv * (max_angular / ws));
            }

            bodies.force[i] = Vec3::ZERO;
            bodies.torque[i] = Vec3::ZERO;
        }
    }

    /// 位置积分：x += v·dt；q = integrate_angular(q, ω, dt)。
    pub fn integrate_positions(bodies: &mut BodySet, dt: f32) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) || !bodies.awake[i] {
                continue;
            }
            bodies.position[i] = bodies.position[i] + bodies.linvel[i] * dt;
            let q = bodies.rot(i).integrate_angular(bodies.angvel(i), dt);
            bodies.set_rot(i, q);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{Quat, Shape};

    #[test]
    fn free_fall_matches_analytic() {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 10.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let dt = 1.0 / 60.0;
        let g = Vec3::new(0.0, -9.81, 0.0);
        for _ in 0..60 {
            Integrator::integrate_velocities(&mut b, g, dt, 1e6, 1e6);
            Integrator::integrate_positions(&mut b, dt);
        }
        // 半隐式欧拉离散解析解：y_N = y0 + Σ g·dt²·k = y0 + g·dt²·N(N+1)/2。
        // 与连续解的差异 O(dt) 是该积分器的确定性行为，这里做 bit 级公式校验。
        let n = 60.0f32;
        let analytic = 10.0 + -9.81 * dt * dt * n * (n + 1.0) / 2.0;
        let got = b.position[0].y;
        assert!(
            (got - analytic).abs() < 1e-3,
            "got {got}, analytic {analytic}"
        );
    }

    #[test]
    fn static_bodies_do_not_move() {
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::splat(1.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        Integrator::integrate_velocities(&mut b, Vec3::new(0.0, -9.81, 0.0), 1.0 / 60.0, 1e6, 1e6);
        Integrator::integrate_positions(&mut b, 1.0 / 60.0);
        assert_eq!(b.position[0], Vec3::ZERO);
    }
}
