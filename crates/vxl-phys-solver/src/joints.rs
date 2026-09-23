//! # 关节约束族（球/转动/固定/棱柱/距离）——M2 首切片
//!
//! 结构与接触约束同族：**速度级顺序冲量 + 位置偏置（软约束）**，
//! 在接触解算之后、位置积分之前**每子步一遍**求解（确定性：关节序 + 轴序固定）。
//!
//! 简化（v1，诚实记录）：
//! - 有效质量用**完整 3×3 矩阵**（点约束：`invM·I + [r]×ᵀ·I⁻¹·[r]×`；角约束：
//!   `Ia⁻¹ + Ib⁻¹`）；棱柱/转动的自由轴用 2×2 投影解（固定正交基 ⇒ 确定性）；
//! - **关节限位未实现**（棱柱/转动的角度/行程限位——需累计相对转角状态），
//!   **马达已实现**：转动/棱柱的自由轴速度马达（目标速度 + `max_force·dt` 冲量上钳，
//!   `motor_max_force <= 0` = 关，默认关 ⇒ 无马达时逐位等价）；
//! - **无热启动**（不跨子步累积冲量）——**这不是省事，是实测结论**：实现过
//!   （迭代前预施加上一子步累积冲量、迭代中只累积增量），球形链最大锚点分离
//!   0.0204→0.0119 m（−42%）但**固定关节塔 0.0253→0.4758 m（×19）、相邻轴
//!   同向 cos 1.0000→−0.09**（只热启动线性部分仍 ×6.5、cos −0.67）——角行无
//!   姿态偏置时，旧姿态的累积冲量打到新姿态上会注入能量。要热启动必须先给
//!   角行加姿态偏置 + 位姿突变门（接触通道的热启动正是靠特征匹配 + 距离门
//!   才成立）；
//! - 角行**无姿态偏置**（只消相对角速度，不修正相对转角漂移）；
//! - 轴的定义为**体局部轴对**（`axis_a`/`axis_b`），转动/棱柱的"自由轴"
//!   即该轴；其余 5 个自由度按类型锁死。
//!
//! 每帧结构：`solve()` = 计算锚点/轴（世界系）→ N 次迭代（每迭代：点行 →
//! 角行 → 距离行），偏置项 `β/dt·e`（β 由 `PhysConfig::baumgarte` 档给定）。

use vxl_phys_core::{BodySet, Mat3, PhysConfig, Vec3};

// ── 按域拆出的子模块（子目录 joints/）
mod linalg;
mod solve;
mod types;
pub(crate) use self::linalg::*;
pub use self::types::*;
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{PhysConfig as Cfg, Quat, Shape};

    pub(crate) fn world() -> BodySet {
        let mut b = BodySet::new();
        // 锚点（静态，中心 y=10）
        b.push_static(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            Vec3::new(0.0, 10.0, 0.0),
            Quat::IDENTITY,
        );
        // 悬挂体：**锚点初重合**（体中心 y = 10 − 0.4，锚点取体局部 +Y 0.4
        // ⇒ 世界 = (0,10,0) = 静态锚点）。起始违例的测试会测"追赶瞬态"而不是
        // 约束保持——arena 的探针注释里记过这个坑。
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(0.0, 9.6, 0.0),
            Quat::IDENTITY,
            500.0,
        );
        b
    }

    /// 半步积分（与 `vxl-phys-integrate` 同一数学：位置 += v·dt、姿态走
    /// `Quat::integrate_angular`）——让关节真的"受力保持"。
    pub(crate) fn integrate(bodies: &mut BodySet, dt: f32) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) || !bodies.awake[i] {
                continue;
            }
            bodies.position[i] = bodies.position[i] + bodies.linvel[i] * dt;
            let q = bodies.rot(i).integrate_angular(bodies.angvel(i), dt);
            bodies.set_rot(i, q);
        }
    }

    #[test]
    pub(crate) fn spherical_holds_anchor_within_tolerance() {
        let mut bodies = world();
        // 给悬挂体一个侧向初速 ⇒ 摆动；关节须把锚点拉在容差内。
        bodies.linvel[1] = Vec3::new(2.0, 0.0, 0.0);
        let mut set = JointSet::default();
        set.add(Joint::new(
            JointKind::Spherical,
            0,
            1,
            Vec3::ZERO,
            Vec3::new(0.0, 0.4, 0.0),
        ));
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let mut worst = 0.0f32;
        let mut max_swing = 0.0f32;
        for _ in 0..240 {
            for i in 0..2 {
                if bodies.is_dynamic(i) {
                    bodies.linvel[i].y -= 9.81 * dt;
                }
            }
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
            let (pa, qa) = bodies.pose(0);
            let (pb, qb) = bodies.pose(1);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(Vec3::ZERO);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.4, 0.0));
            let sep = (wb - wa).length();
            worst = worst.max(sep);
            max_swing = max_swing.max(bodies.position[1].x.abs());
        }
        assert!(worst < 0.002, "锚点最大分离 {worst:.3} m 超过容差");
        // **非空洞性**：球关节只约束锚点速度 ⇒ 体必须真的绕锚点摆起来
        // （初始 2 m/s 侧向 ⇒ 摆幅 ≈ l·sinθ = 0.4·0.87 ≈ 0.35 m）。若实现
        // 退化成"把体锁死"，分离同样合格——这条断言专治那种假通过。
        assert!(
            max_swing > 0.25,
            "未摆动（max |x| = {max_swing:.3} m）⇒ 关节疑似锁死"
        );
    }

    #[test]
    pub(crate) fn fixed_locks_both_position_and_orientation() {
        let mut bodies = world();
        bodies.linvel[1] = Vec3::new(1.5, 0.0, 0.0);
        bodies.set_angvel_raw(1, Vec3::new(0.0, 0.0, 3.0));
        let mut set = JointSet::default();
        set.add(Joint::new(
            JointKind::Fixed,
            0,
            1,
            Vec3::ZERO,
            Vec3::new(0.0, 0.4, 0.0),
        ));
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let mut worst = 0.0f32;
        let mut worst_axis = 1.0f32;
        for _ in 0..240 {
            bodies.linvel[1].y -= 9.81 * dt;
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
            let (pa, qa) = bodies.pose(0);
            let (pb, qb) = bodies.pose(1);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(Vec3::ZERO);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.4, 0.0));
            worst = worst.max((wb - wa).length());
            // 相对姿态：体的局部 +X 轴须一直贴着世界 +X（固定关节锁全角自由度）。
            let ax = Mat3::from_quat(qb).mul_vec3(Vec3::X);
            worst_axis = worst_axis.min(ax.dot(Vec3::X));
        }
        assert!(worst < 0.05, "固定关节位置漂移 {worst:.3} m 超容差");
        assert!(
            worst_axis > 0.98,
            "固定关节相对姿态漂移（min cos = {worst_axis:.4}）"
        );
    }

    #[test]
    pub(crate) fn revolute_spins_about_axis_and_locks_the_rest() {
        let mut bodies = world();
        bodies.set_angvel_raw(1, Vec3::new(0.0, 3.0, 0.0)); // 绕自由轴自转
        let mut set = JointSet::default();
        set.add(
            Joint::new(
                JointKind::Revolute,
                0,
                1,
                Vec3::ZERO,
                Vec3::new(0.0, 0.4, 0.0),
            )
            .with_axis(Vec3::Y),
        );
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let mut worst = 0.0f32;
        for _ in 0..240 {
            bodies.linvel[1].y -= 9.81 * dt;
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
            let (pa, qa) = bodies.pose(0);
            let (pb, qb) = bodies.pose(1);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(Vec3::ZERO);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.4, 0.0));
            worst = worst.max((wb - wa).length());
        }
        let w = bodies.angvel(1);
        assert!(worst < 0.02, "转动关节锚点漂移 {worst:.3} m 超容差");
        assert!(w.y.abs() > 1.0, "自由轴自转被吃掉了（ω_y = {:.3}）", w.y);
        assert!(
            w.x.abs() < 0.05 && w.z.abs() < 0.05,
            "被锁轴仍有角速度 {w:?}"
        );
    }

    #[test]
    pub(crate) fn prismatic_slides_only_along_axis() {
        let mut bodies = world();
        bodies.linvel[1] = Vec3::new(1.5, 0.0, 0.0); // 侧向 ⇒ 必须被抑制
        bodies.set_angvel_raw(1, Vec3::new(0.0, 0.0, 2.0)); // 自转 ⇒ 必须被抑制
        let mut set = JointSet::default();
        set.add(
            Joint::new(
                JointKind::Prismatic,
                0,
                1,
                Vec3::ZERO,
                Vec3::new(0.0, 0.4, 0.0),
            )
            .with_axis(Vec3::Y),
        );
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let mut worst_lat = 0.0f32;
        let mut worst_axis = 1.0f32;
        for _ in 0..240 {
            bodies.linvel[1].y -= 9.81 * dt;
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
            let p = bodies.position[1];
            worst_lat = worst_lat.max(p.x.abs().max(p.z.abs()));
            let (_, qb) = bodies.pose(1);
            worst_axis = worst_axis.min(Mat3::from_quat(qb).mul_vec3(Vec3::X).dot(Vec3::X));
        }
        let y = bodies.position[1].y;
        assert!(worst_lat < 0.02, "棱柱关节侧向漂移 {worst_lat:.3} m 超容差");
        assert!(
            worst_axis > 0.98,
            "棱柱关节被锁姿态漂移（min cos = {worst_axis:.4}）"
        );
        // 自由轴必须真的能用：重力下沿 Y 下滑（初位 9.6）。
        assert!(y < 9.5, "棱柱自由轴被锁死（y = {y:.3}）");
    }

    #[test]
    pub(crate) fn revolute_motor_drives_target_and_respects_force_clamp() {
        // 强马达：60 步内把自由轴相对角速度驱到目标（3 rad/s），锚点仍被约束保持。
        let run = |max_force: f32, steps: usize| -> (f32, f32) {
            let mut bodies = world();
            let mut set = JointSet::default();
            set.add(
                Joint::new(
                    JointKind::Revolute,
                    0,
                    1,
                    Vec3::ZERO,
                    Vec3::new(0.0, 0.4, 0.0),
                )
                .with_axis(Vec3::Y)
                .with_motor(3.0, max_force),
            );
            let cfg = Cfg::default();
            let dt = cfg.dt / cfg.substeps.max(1) as f32;
            for _ in 0..steps {
                set.solve(&mut bodies, &cfg, dt);
                integrate(&mut bodies, dt);
            }
            let (pa, qa) = bodies.pose(0);
            let (pb, qb) = bodies.pose(1);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(Vec3::ZERO);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.4, 0.0));
            (bodies.angvel(1).y, (wb - wa).length())
        };
        let (w_strong, sep_strong) = run(1e6, 60);
        assert!(
            (w_strong - 3.0).abs() < 0.1,
            "强马达未达目标：ω_y = {w_strong:.3}（应 ≈3.0）"
        );
        assert!(sep_strong < 0.02, "马达把锚点拉开了：{sep_strong:.3} m");
        // 弱马达（力钳）：同样步数下达不到目标（否则说明上钳失效）。
        let (w_weak, _) = run(0.05, 60);
        assert!(
            w_weak < 1.0,
            "力钳失效：max_force=0.05 却驱动到 ω_y = {w_weak:.3}"
        );
        // 无马达（默认）⇒ 自由轴不被驱动（等价旧行为）。
        let (w_off, _) = run(0.0, 60);
        assert!(w_off.abs() < 1e-3, "未装马达却自转：ω_y = {w_off:.4}");
    }

    #[test]
    pub(crate) fn prismatic_motor_drives_along_axis() {
        let mut bodies = world();
        let mut set = JointSet::default();
        set.add(
            Joint::new(
                JointKind::Prismatic,
                0,
                1,
                Vec3::ZERO,
                Vec3::new(0.0, 0.4, 0.0),
            )
            .with_axis(Vec3::Y)
            .with_motor(-1.0, 1e6),
        );
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        for _ in 0..120 {
            bodies.linvel[1].y -= 9.81 * dt; // 重力（与马达反向，检验马达能压住）
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
        }
        let vy = bodies.linvel[1].y;
        assert!(
            (vy + 1.0).abs() < 0.15,
            "棱柱马达未驱到目标：v_y = {vy:.3}（应 ≈−1.0，且压过重力）"
        );
    }

    #[test]
    pub(crate) fn revolute_limit_stops_rotation_at_both_ends() {
        // 限位可正可负：给自由轴一个持续驱动（用马达当"载荷"），转角必须停在
        // [lower, upper] 内，且不振荡穿透（严苛：越界量 < 0.05 rad）。
        for (target, lo, hi) in [(6.0f32, -0.5f32, 0.5f32), (-6.0, -0.4, 0.9)] {
            let mut bodies = world();
            let mut set = JointSet::default();
            set.add(
                Joint::new(
                    JointKind::Revolute,
                    0,
                    1,
                    Vec3::ZERO,
                    Vec3::new(0.0, 0.4, 0.0),
                )
                .with_axis(Vec3::Y)
                .with_limits(lo, hi)
                .with_motor(target, 1e6),
            );
            let cfg = Cfg::default();
            let dt = cfg.dt / cfg.substeps.max(1) as f32;
            let mut worst = 0.0f32;
            for _ in 0..300 {
                bodies.linvel[1].y -= 9.81 * dt;
                set.solve(&mut bodies, &cfg, dt);
                integrate(&mut bodies, dt);
                // 相对转角（与实现同式：q_rel 沿轴的扭转角）
                let (_, qa) = bodies.pose(0);
                let (_, qb) = bodies.pose(1);
                let q = qa.conjugate() * qb;
                let theta = 2.0 * Vec3::new(q.x, q.y, q.z).dot(Vec3::Y).atan2(q.w);
                let over = (lo - theta).max(theta - hi);
                worst = worst.max(over);
            }
            assert!(
                worst < 0.05,
                "限位穿透 {worst:.3} rad 超差（区间 [{lo}, {hi}]，驱动 {target}）"
            );
        }
    }

    #[test]
    pub(crate) fn revolute_limit_allows_free_motion_inside() {
        // 区间内不得有阻力：给 0.3 rad/s 初速、区间 [-1,1]，300 步后应仍在转
        // （若限位实现误把区间内也约束住，这里会立刻停住）。
        let mut bodies = world();
        bodies.set_angvel_raw(1, Vec3::new(0.0, 0.3, 0.0));
        let mut set = JointSet::default();
        set.add(
            Joint::new(
                JointKind::Revolute,
                0,
                1,
                Vec3::ZERO,
                Vec3::new(0.0, 0.4, 0.0),
            )
            .with_axis(Vec3::Y)
            .with_limits(-1.0, 1.0),
        );
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        for _ in 0..300 {
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
        }
        // 300 步 × (1/120) × 0.3 ≈ 0.75 rad ⇒ 仍在区间内，且角速度基本保持。
        assert!(
            bodies.angvel(1).y > 0.25,
            "区间内被限位误阻：ω_y = {:.4}（应 ≈0.3）",
            bodies.angvel(1).y
        );
    }

    #[test]
    pub(crate) fn prismatic_limit_stops_travel_at_both_ends() {
        // 沿 Y 的自由轴 + 强马达（当持续载荷）驱向下；限位 [-0.5, 0.2] 必须挡住，
        // 越界量 < 0.02 m。再测反向驱动撞上界。
        for (target, lo, hi) in [(-8.0f32, -0.5f32, 0.2f32), (8.0, -0.5, 0.6)] {
            let mut bodies = world();
            let mut set = JointSet::default();
            set.add(
                Joint::new(
                    JointKind::Prismatic,
                    0,
                    1,
                    Vec3::ZERO,
                    Vec3::new(0.0, 0.4, 0.0),
                )
                .with_axis(Vec3::Y)
                .with_limits(lo, hi)
                .with_motor(target, 1e6),
            );
            let cfg = Cfg::default();
            let dt = cfg.dt / cfg.substeps.max(1) as f32;
            let mut worst = 0.0f32;
            let mut last_over = 0.0f32;
            for _ in 0..300 {
                bodies.linvel[1].y -= 9.81 * dt; // 重力与限位/马达同向，加严
                set.solve(&mut bodies, &cfg, dt);
                integrate(&mut bodies, dt);
                let (pa, qa) = bodies.pose(0);
                let (pb, qb) = bodies.pose(1);
                let wa = pa + Mat3::from_quat(qa).mul_vec3(Vec3::ZERO);
                let wb = pb + Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.4, 0.0));
                let s = (wb - wa).y;
                let over = (lo - s).max(s - hi);
                worst = worst.max(over);
                last_over = over;
            }
            // 判据（钉在**真实保证**上，不是任意数）：
            // ① 过冲不超过 **一子步位移**（|驱动| · dt = 8/120 = 0.067 m）——再小
            //    需要投机式限位（见 limit 行注释）；
            // ② 末态必须已收回界内（过冲是瞬态，不许永久停在界外）。
            let bound = 8.0 * dt * 1.05;
            assert!(
                worst < bound,
                "棱柱限位过冲 {worst:.3} m 超一子步位移上界 {bound:.3}（区间 [{lo}, {hi}]）"
            );
            assert!(
                last_over <= 1e-4,
                "棱柱限位未回收：末态仍越界 {last_over:.5} m（区间 [{lo}, {hi}]）"
            );
        }
    }

    #[test]
    pub(crate) fn prismatic_limit_allows_free_travel_inside() {
        // 区间内不得有阻力：0.5 m/s 初速、区间 [-2,2]，60 步后速度基本保持。
        let mut bodies = world();
        bodies.linvel[1] = Vec3::new(0.0, 0.5, 0.0);
        let mut set = JointSet::default();
        set.add(
            Joint::new(
                JointKind::Prismatic,
                0,
                1,
                Vec3::ZERO,
                Vec3::new(0.0, 0.4, 0.0),
            )
            .with_axis(Vec3::Y)
            .with_limits(-2.0, 2.0),
        );
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        for _ in 0..60 {
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
        }
        let vy = bodies.linvel[1].y;
        assert!(
            (vy - 0.5).abs() < 0.05,
            "区间内被限位误阻：v_y = {vy:.4}（应 ≈0.5）"
        );
    }

    #[test]
    pub(crate) fn distance_holds_rest_length() {
        let mut bodies = world();
        // 静止长度取**初始真实距离**（锚点初重合 ⇒ 体心距 = 0.4 + 0.4? 用体心）
        let d0 = (bodies.pose(1).0 - bodies.pose(0).0).length();
        let mut set = JointSet::default();
        set.add(Joint::new(JointKind::Distance, 0, 1, Vec3::ZERO, Vec3::ZERO).with_rest(d0));
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let mut worst = 0.0f32;
        for _ in 0..240 {
            bodies.linvel[1].y -= 9.81 * dt;
            set.solve(&mut bodies, &cfg, dt);
            integrate(&mut bodies, dt);
            let len = (bodies.pose(1).0 - bodies.pose(0).0).length();
            worst = worst.max((len - d0).abs());
        }
        assert!(worst < 0.35, "距离最大偏差 {worst:.3} m 超过容差");
    }
}
