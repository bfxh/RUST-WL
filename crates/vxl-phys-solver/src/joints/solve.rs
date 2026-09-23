//! solve：从 joints.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) fn params(cfg: &PhysConfig, dt: f32) -> JointParams {
    let inv_dt = 1.0 / dt.max(1e-6);
    // β 取配置的 baumgarte 档（默认 ≈0.2 档）——偏置率随步长归一，跨子步一致。
    JointParams {
        bias_inv_dt: cfg.baumgarte.max(0.0) * inv_dt,
        inv_dt,
        eps: 0.002,
        min_iters: 4,
    }
}

impl JointSet {
    pub fn add(&mut self, j: Joint) -> u32 {
        self.joints.push(j);
        (self.joints.len() - 1) as u32
    }

    pub fn clear(&mut self) {
        self.joints.clear();
    }

    pub fn len(&self) -> usize {
        self.joints.len()
    }

    pub fn is_empty(&self) -> bool {
        self.joints.is_empty()
    }

    /// 唤醒传播（**每子步、接触解算之前**调用）：关节任一端是**清醒的动态体**
    /// ⇒ 两端（动态端）一起醒。
    ///
    /// 静态端**不触发**唤醒：静态体的 `awake` 恒为 true，若按"任一端醒即醒"
    /// 判定，每个挂在静态锚点上的关节都会把关着的体每子步踢醒——悬挂摆永不
    /// 入睡（岛级休眠对关节场景失效）。
    pub fn wake(&self, bodies: &mut BodySet) {
        for j in &self.joints {
            let (a, b) = (j.a as usize, j.b as usize);
            let acting = (bodies.is_dynamic(a) && bodies.awake[a])
                || (bodies.is_dynamic(b) && bodies.awake[b]);
            if acting {
                bodies.wake(a);
                bodies.wake(b);
            }
        }
    }

    /// 每帧求解（**接触解算之后、位置积分之前**）：先唤醒传播，再按
    /// `config.joint_iterations` 迭代（早退与接触通道同口径）。
    pub fn solve(&mut self, bodies: &mut BodySet, config: &PhysConfig, dt: f32) {
        if self.joints.is_empty() {
            return;
        }
        self.wake(bodies);
        let sp = params(config, dt);
        // 关节独立迭代预算（接触档是标定过的接触预算；关节链的敏感度不同，
        // 见 `PhysConfig::joint_iterations` 的分离-迭代标定表）。
        let iters = config.joint_iterations.max(1);
        for it in 0..iters {
            let mut resid = 0.0f32;
            for j in self.joints.iter_mut() {
                let (a, b) = (j.a as usize, j.b as usize);
                if !joint_active(bodies, a, b) {
                    continue;
                }
                resid = resid.max(solve_joint(j, bodies, &sp));
            }
            if it + 1 >= sp.min_iters && resid < sp.eps {
                break;
            }
        }
    }
}

/// 该关节本子步是否活跃（至少一端是清醒动态体）。
///
/// 整组沉睡/双静态 ⇒ 不施加冲量：对沉睡体写速度/角速度会破坏岛级休眠语义
/// （体明明睡着却被关节推着走，`stability-sleep` 类判据就失去意义）。
#[inline]
pub(crate) fn joint_active(bodies: &BodySet, a: usize, b: usize) -> bool {
    (bodies.is_dynamic(a) && bodies.awake[a]) || (bodies.is_dynamic(b) && bodies.awake[b])
}

/// 求解一条关节一次（返回本次的最大速度级修正，供早退判据）。
///
/// **有效质量用完整 3×3 矩阵**（不是逐轴对角近似）：点约束的线性/角向耦合在
/// 「锚点偏移大 + 惯量小」时对角近似的迭代会发散（实测：悬挂摆 1 步内速度 ×4
/// 爆炸）——3×3 解是标准做法，也只在每关节每迭代算一次。
pub(crate) fn solve_joint(j: &mut Joint, bodies: &mut BodySet, sp: &JointParams) -> f32 {
    let (ai, bi) = (j.a as usize, j.b as usize);
    let (pa, qa) = bodies.pose(ai);
    let (pb, qb) = bodies.pose(bi);
    let ra = Mat3::from_quat(qa).mul_vec3(j.anchor_a);
    let rb = Mat3::from_quat(qb).mul_vec3(j.anchor_b);
    let wa = pa + ra;
    let wb = pb + rb;
    let err = wb - wa; // 位置误差（a→b）
    let inv_ma = bodies.inv_mass[ai];
    let inv_mb = bodies.inv_mass[bi];
    if inv_ma == 0.0 && inv_mb == 0.0 {
        return 0.0; // 双静态：无自由度
    }

    // ---- 线性行 ----
    let mut max_dv = match j.kind {
        JointKind::Distance => {
            // 单行：沿两锚点连线，约束 |d| = rest（一维，无耦合问题）。
            let d = wb - wa;
            let len = d.length();
            if len <= 1e-6 {
                0.0
            } else {
                let n = d * (1.0 / len);
                let bias = -(len - j.rest) * sp.bias_inv_dt;
                let v = rel_vel(bodies, ai, bi, ra, rb);
                let k = row_mass(bodies, ai, bi, ra, rb, n);
                let lambda = if k > 1e-9 {
                    -(v.dot(n) + bias) / k
                } else {
                    0.0
                };
                apply_pair(bodies, ai, bi, ra, rb, n * lambda);
                lambda.abs()
            }
        }
        JointKind::Prismatic => {
            // 锁两条与滑动轴垂直的线性自由度（2×2 投影求解）。
            let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
            let [t1, t2] = perp_basis(axis_w);
            let k3 = point_k(bodies, ai, bi, ra, rb);
            let (k11, k12, k22) = (proj(k3, t1, t1), proj(k3, t1, t2), proj(k3, t2, t2));
            let det = k11 * k22 - k12 * k12;
            if det.abs() < 1e-12 {
                0.0
            } else {
                let v = rel_vel(bodies, ai, bi, ra, rb);
                let r1 = -(v.dot(t1) + err.dot(t1) * sp.bias_inv_dt);
                let r2 = -(v.dot(t2) + err.dot(t2) * sp.bias_inv_dt);
                let l1 = (r1 * k22 - r2 * k12) / det;
                let l2 = (r2 * k11 - r1 * k12) / det;
                apply_pair(bodies, ai, bi, ra, rb, t1 * l1 + t2 * l2);
                l1.abs().max(l2.abs())
            }
        }
        _ => {
            // 球/转动/固定：三条线性行（锚点重合），3×3 求解。
            let k3 = point_k(bodies, ai, bi, ra, rb);
            let v = rel_vel(bodies, ai, bi, ra, rb);
            let rhs = -(v + err * sp.bias_inv_dt);
            match solve3(k3, rhs) {
                Some(lambda) => {
                    apply_pair(bodies, ai, bi, ra, rb, lambda);
                    lambda.length()
                }
                None => 0.0,
            }
        }
    };

    // ---- 角行（固定/棱柱锁 3 轴；转动锁垂直于 `axis_a` 的 2 轴）----
    let k_ang = ang_k(bodies, ai, bi);
    match j.kind {
        JointKind::Fixed | JointKind::Prismatic => {
            let wrel = bodies.angvel(bi) - bodies.angvel(ai);
            if let Some(lambda) = solve3(k_ang, -wrel) {
                apply_ang_pair(bodies, ai, bi, lambda);
                max_dv = max_dv.max(lambda.length());
            }
        }
        JointKind::Revolute => {
            let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
            let [t1, t2] = perp_basis(axis_w);
            let wrel = bodies.angvel(bi) - bodies.angvel(ai);
            let (k11, k12, k22) = (
                proj(k_ang, t1, t1),
                proj(k_ang, t1, t2),
                proj(k_ang, t2, t2),
            );
            let det = k11 * k22 - k12 * k12;
            if det.abs() >= 1e-12 {
                let r1 = -wrel.dot(t1);
                let r2 = -wrel.dot(t2);
                let l1 = (r1 * k22 - r2 * k12) / det;
                let l2 = (r2 * k11 - r1 * k12) / det;
                apply_ang_pair(bodies, ai, bi, t1 * l1 + t2 * l2);
                max_dv = max_dv.max(l1.abs().max(l2.abs()));
            }
        }
        _ => {}
    }

    // ---- 马达行（转动/棱柱；`motor_max_force <= 0` 直接跳过 ⇒ 无马达时逐位不变）----
    // 速度级马达：把自由轴上的相对速度驱到 `motor_target`，冲量按 `max_force·dt` 上钳。
    // **必须排在限位行之前**：限位是更硬的约束，要最后说话——否则马达会把限位刚
    // 修正好的速度重新驱回越界方向（实测：6 rad/s 的马达直接把限位冲穿 5.77 rad）。
    if j.motor_max_force > 0.0 && j.motor_max_force.is_finite() && sp.inv_dt > 0.0 {
        let lim = j.motor_max_force / sp.inv_dt; // 本子步可用冲量上限
        match j.kind {
            JointKind::Revolute => {
                let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
                let wrel = bodies.angvel(bi) - bodies.angvel(ai);
                let k = axis_w.dot(bodies.apply_world_inv_inertia(ai, axis_w))
                    + axis_w.dot(bodies.apply_world_inv_inertia(bi, axis_w));
                if k > 1e-9 {
                    let lambda = ((j.motor_target - wrel.dot(axis_w)) / k).clamp(-lim, lim);
                    apply_ang_pair(bodies, ai, bi, axis_w * lambda);
                    max_dv = max_dv.max(lambda.abs());
                }
            }
            JointKind::Prismatic => {
                let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
                let v = rel_vel(bodies, ai, bi, ra, rb);
                let k = row_mass(bodies, ai, bi, ra, rb, axis_w);
                if k > 1e-9 {
                    let lambda = ((j.motor_target - v.dot(axis_w)) / k).clamp(-lim, lim);
                    apply_pair(bodies, ai, bi, ra, rb, axis_w * lambda);
                    max_dv = max_dv.max(lambda.abs());
                }
            }
            _ => {}
        }
    }
    // ---- 转动限位（绕自由轴的相对转角；单边行 + 偏置推回）**最后解** ----
    // 角度从**当前相对姿态**直接算：`q_rel = q_a⁻¹·q_b` 沿轴的扭转角
    // `θ = 2·atan2(q_rel.vec·axis, q_rel.w)`（两者都取体局部轴 ⇒ 与体姿态无关）。
    // 用姿态而非累计角速度积分：无漂移、纯状态函数 ⇒ 确定性不受影响。
    // 排在马达之后 ⇒ 越界时马达无法把速度驱回越界方向（限位是更硬的约束）。
    if matches!(j.kind, JointKind::Revolute) && j.limit_lower < j.limit_upper {
        let q_rel = qa.conjugate() * qb;
        let v = Vec3::new(q_rel.x, q_rel.y, q_rel.z);
        let theta = 2.0 * v.dot(j.axis_a).atan2(q_rel.w);
        // err > 0 = 低于下界（需往正方向转回）；err < 0 = 高于上界（需往负方向）。
        let err = if theta < j.limit_lower {
            j.limit_lower - theta
        } else if theta > j.limit_upper {
            j.limit_upper - theta
        } else {
            0.0
        };
        if err != 0.0 {
            let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
            let wrel = bodies.angvel(bi) - bodies.angvel(ai);
            let k = axis_w.dot(bodies.apply_world_inv_inertia(ai, axis_w))
                + axis_w.dot(bodies.apply_world_inv_inertia(bi, axis_w));
            if k > 1e-9 {
                // 单边行：冲量只往"转回限位内"的方向给（带偏置 `β·err/dt` 推回）。
                // 偏置沿用接触档 β（`bias_inv_dt`）。**不要**改成 β=1：实测棱柱限位
                // 在 8 m/s 强驱动下过冲 0.033 m（β=1 反而 0.067，Baumgarte 振荡）。
                // 过冲的物理下界 ≈ **一子步位移 v·dt**（8 m/s × 1/120 = 0.067 m）——
                // 想把过冲压到 0 需要**投机式限位**（在仍处于界内、但本子步会越过时
                // 就建行，用预测速度 `s + v·dt` 判），与接触的 speculative margin 同思路。
                let bias = sp.bias_inv_dt * err;
                let vn = wrel.dot(axis_w);
                let lambda = if err > 0.0 {
                    ((bias - vn) / k).max(0.0)
                } else {
                    ((bias - vn) / k).min(0.0)
                };
                if lambda != 0.0 {
                    apply_ang_pair(bodies, ai, bi, axis_w * lambda);
                    max_dv = max_dv.max(lambda.abs());
                }
            }
        }
    }

    // ---- 棱柱行程限位（沿轴；同样**最后解** + 单边行 + 偏置推回）----
    // 行程 = 锚点沿轴的分离量（`err·axis_w`）——与转动限位同招：用**当前几何**
    // 直接算，不累计位移 ⇒ 无漂移、纯状态函数。锁定行保证其余方向分离 ≈0。
    if matches!(j.kind, JointKind::Prismatic) && j.limit_lower < j.limit_upper {
        let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
        let s = err.dot(axis_w);
        // err > 0 = 低于下界（需沿正方向推回）；err < 0 = 高于上界。
        let lerr = if s < j.limit_lower {
            j.limit_lower - s
        } else if s > j.limit_upper {
            j.limit_upper - s
        } else {
            0.0
        };
        if lerr != 0.0 {
            let v = rel_vel(bodies, ai, bi, ra, rb);
            let k = row_mass(bodies, ai, bi, ra, rb, axis_w);
            if k > 1e-9 {
                // 偏置沿用接触档 β（同转动限位；β=1 会诱发 Baumgarte 振荡，实测更差）。
                let bias = sp.bias_inv_dt * lerr;
                let vn = v.dot(axis_w);
                let lambda = if lerr > 0.0 {
                    ((bias - vn) / k).max(0.0)
                } else {
                    ((bias - vn) / k).min(0.0)
                };
                if lambda != 0.0 {
                    apply_pair(bodies, ai, bi, ra, rb, axis_w * lambda);
                    max_dv = max_dv.max(lambda.abs());
                }
            }
        }
    }

    max_dv
}
