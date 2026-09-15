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

/// 关节类型（与 PhysArena 的 JointKind 同构；`Spring` 归入 `Distance`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    /// 球形：两个锚点重合（3 线性约束），相对转动自由。
    Spherical,
    /// 转动：锚点重合 + 相对转动只允许绕 `axis`（锁 2 个角自由度）。
    Revolute,
    /// 固定：锚点重合 + 相对转动全锁（6 自由度）。
    Fixed,
    /// 棱柱：只允许沿 `axis` 平移（锁 2 线性 + 3 角自由度）。
    Prismatic,
    /// 距离：两锚点距离 = `rest`（1 约束）。
    Distance,
}

/// 一条关节（引擎侧 IR；锚点/轴均为**体局部**）。
#[derive(Clone, Copy, Debug)]
pub struct Joint {
    pub kind: JointKind,
    pub a: u32,
    pub b: u32,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    /// 局部轴（转动/棱柱用；球形/固定/距离忽略）。
    pub axis_a: Vec3,
    pub axis_b: Vec3,
    /// 距离关节的静止长度。
    pub rest: f32,
    /// **马达目标**（转动：自由轴的相对角速度 rad/s；棱柱：沿轴的相对线速度 m/s）。
    /// `motor_max_force <= 0` = 无马达（默认）⇒ 逐位等价于无马达行为。
    pub motor_target: f32,
    /// 马达最大力/矩（N 或 N·m）；每子步的冲量上钳 = `motor_max_force · dt`。
    pub motor_max_force: f32,
}

impl Joint {
    pub fn new(kind: JointKind, a: u32, b: u32, anchor_a: Vec3, anchor_b: Vec3) -> Self {
        Self {
            kind,
            a,
            b,
            anchor_a,
            anchor_b,
            axis_a: Vec3::X,
            axis_b: Vec3::X,
            rest: 0.0,
            motor_target: 0.0,
            motor_max_force: 0.0,
        }
    }

    /// 加**马达**（转动：目标角速度 rad/s；棱柱：目标线速度 m/s；`max_force <= 0` = 关）。
    /// 关节限位仍未实现（需累计相对转角状态）。
    pub fn with_motor(mut self, target_velocity: f32, max_force: f32) -> Self {
        self.motor_target = target_velocity;
        self.motor_max_force = max_force;
        self
    }

    pub fn with_axis(mut self, axis: Vec3) -> Self {
        self.axis_a = axis;
        self.axis_b = axis;
        self
    }

    pub fn with_rest(mut self, rest: f32) -> Self {
        self.rest = rest;
        self
    }
}

/// 关节集合（世界持有；每帧求解一遍）。
#[derive(Default)]
pub struct JointSet {
    pub joints: Vec<Joint>,
}

/// 关节求解参数（与接触通道同口径：软约束偏置 + CFM 正则化近似）。
#[derive(Clone, Copy)]
struct JointParams {
    /// 位置偏置率（1/s）：`β·inv_dt`。
    bias_inv_dt: f32,
    /// `1/dt`（马达冲量上钳换算用：`max_force·dt = max_force / inv_dt`）。
    inv_dt: f32,
    /// 速度级残差下限（早退判据，m/s）。
    eps: f32,
    /// 最少迭代数（早退前的保底）。
    min_iters: u32,
}

fn params(cfg: &PhysConfig, dt: f32) -> JointParams {
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
fn joint_active(bodies: &BodySet, a: usize, b: usize) -> bool {
    (bodies.is_dynamic(a) && bodies.awake[a]) || (bodies.is_dynamic(b) && bodies.awake[b])
}

/// 求解一条关节一次（返回本次的最大速度级修正，供早退判据）。
///
/// **有效质量用完整 3×3 矩阵**（不是逐轴对角近似）：点约束的线性/角向耦合在
/// 「锚点偏移大 + 惯量小」时对角近似的迭代会发散（实测：悬挂摆 1 步内速度 ×4
/// 爆炸）——3×3 解是标准做法，也只在每关节每迭代算一次。
fn solve_joint(j: &mut Joint, bodies: &mut BodySet, sp: &JointParams) -> f32 {
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
    // 与约束行同序（先线性/角行把自由度锁住，再驱动自由轴）⇒ 马达不会与锁定行抢。
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
    max_dv
}

/// 角约束的 3×3 有效质量：`Ia⁻¹ + Ib⁻¹`（世界系）。
#[inline]
fn ang_k(bodies: &BodySet, ai: usize, bi: usize) -> [[f32; 3]; 3] {
    let mut k = [[0.0f32; 3]; 3];
    for (j, axis) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
        let c =
            bodies.apply_world_inv_inertia(ai, *axis) + bodies.apply_world_inv_inertia(bi, *axis);
        for (i, row) in k.iter_mut().enumerate() {
            row[j] = [c.x, c.y, c.z][i];
        }
    }
    k
}

/// 点约束的 3×3 有效质量：`(invMa+invMb)·I + [ra]×ᵀ·Ia⁻¹·[ra]× + [rb]×ᵀ·Ib⁻¹·[rb]×`。
#[inline]
fn point_k(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3) -> [[f32; 3]; 3] {
    let mut k = [[0.0f32; 3]; 3];
    let diag = bodies.inv_mass[ai] + bodies.inv_mass[bi];
    for (i, row) in k.iter_mut().enumerate() {
        row[i] = diag;
    }
    // 角向贡献：K += Σ_j [ra]×ᵀ·Ia⁻¹·[ra]×，其 (i,j) 元 = e_i·((Ia⁻¹(ra×e_j)) × ra)
    // ——**先叉（r×e）→ 过惯量 → 再叉 r**。写成 `ra × (I⁻¹(ra × e_j))` 会整体
    // 反号（约束变成放大器：实测初速 2 m/s 在 3 步内炸到 1e5）；`row_mass`（距离
    // 关节的单行通道）用的就是这个正确顺序，两者必须一致。
    // 累加是 `+=`：对角线上已有 `invMa+invMb`，覆盖会把质量项冲掉（矩阵退化成
    // 纯角向项 ⇒ 与 r 平行的轴对角为 0 ⇒ 奇异，解不出来、约束静默失效）。
    for (j, axis) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
        let c_a = bodies
            .apply_world_inv_inertia(ai, ra.cross(*axis))
            .cross(ra);
        let c_b = bodies
            .apply_world_inv_inertia(bi, rb.cross(*axis))
            .cross(rb);
        let col = [c_a.x + c_b.x, c_a.y + c_b.y, c_a.z + c_b.z];
        for (i, row) in k.iter_mut().enumerate() {
            row[j] += col[i];
        }
    }
    k
}

#[inline]
fn proj(k: [[f32; 3]; 3], a: Vec3, b: Vec3) -> f32 {
    let ka = [
        k[0][0] * a.x + k[0][1] * a.y + k[0][2] * a.z,
        k[1][0] * a.x + k[1][1] * a.y + k[1][2] * a.z,
        k[2][0] * a.x + k[2][1] * a.y + k[2][2] * a.z,
    ];
    ka[0] * b.x + ka[1] * b.y + ka[2] * b.z
}

/// 3×3 解（克莱姆 + 行列式奇异守卫；确定性）。
#[inline]
fn solve3(k: [[f32; 3]; 3], rhs: Vec3) -> Option<Vec3> {
    let det = k[0][0] * (k[1][1] * k[2][2] - k[1][2] * k[2][1])
        - k[0][1] * (k[1][0] * k[2][2] - k[1][2] * k[2][0])
        + k[0][2] * (k[1][0] * k[2][1] - k[1][1] * k[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let d0 = rhs.x * (k[1][1] * k[2][2] - k[1][2] * k[2][1])
        - k[0][1] * (rhs.y * k[2][2] - k[1][2] * rhs.z)
        + k[0][2] * (rhs.y * k[2][1] - k[1][1] * rhs.z);
    let d1 = k[0][0] * (rhs.y * k[2][2] - k[1][2] * rhs.z)
        - rhs.x * (k[1][0] * k[2][2] - k[1][2] * k[2][0])
        + k[0][2] * (k[1][0] * rhs.z - rhs.y * k[2][0]);
    let d2 = k[0][0] * (k[1][1] * rhs.z - rhs.y * k[2][1])
        - k[0][1] * (k[1][0] * rhs.z - rhs.y * k[2][0])
        + rhs.x * (k[1][0] * k[2][1] - k[1][1] * k[2][0]);
    Some(Vec3::new(d0 * inv, d1 * inv, d2 * inv))
}

/// 世界系锚点相对速度（b 侧 − a 侧）。
#[inline]
fn rel_vel(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3) -> Vec3 {
    let la = bodies.linvel[ai] + bodies.angvel(ai).cross(ra);
    let lb = bodies.linvel[bi] + bodies.angvel(bi).cross(rb);
    lb - la
}

/// 沿单位轴 `n` 的有效质量（对角近似）。
#[inline]
fn row_mass(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3, n: Vec3) -> f32 {
    let wa = bodies.apply_world_inv_inertia(ai, ra.cross(n)).cross(ra);
    let wb = bodies.apply_world_inv_inertia(bi, rb.cross(n)).cross(rb);
    bodies.inv_mass[ai] + bodies.inv_mass[bi] + n.dot(wa) + n.dot(wb)
}

/// 施加成对**角**冲量（b 侧 +、a 侧 −）。
///
/// **必须按世界系逆惯量缩放**：直接 `ω_a −= λ` 会给静态体写入非零角速度，
/// 下一迭代 `wrel = ω_b − ω_a` 里越积越大 ⇒ 3 步内炸到 1e33（实测：固定/棱柱
/// 关节在静态端点上直接爆掉）。静态体 Ia⁻¹ = 0 ⇒ 天然不动。
#[inline]
fn apply_ang_pair(bodies: &mut BodySet, ai: usize, bi: usize, imp: Vec3) {
    let dwa = bodies.apply_world_inv_inertia(ai, imp);
    let dwb = bodies.apply_world_inv_inertia(bi, imp);
    bodies.set_angvel_raw(ai, bodies.angvel(ai) - dwa);
    bodies.set_angvel_raw(bi, bodies.angvel(bi) + dwb);
}

/// 施加成对线性冲量（b 侧 +、a 侧 −）。#[inline]
fn apply_pair(bodies: &mut BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3, imp: Vec3) {
    let (im_a, im_b) = (bodies.inv_mass[ai], bodies.inv_mass[bi]);
    bodies.linvel[ai] -= imp * im_a;
    bodies.linvel[bi] += imp * im_b;
    let dwa = bodies.apply_world_inv_inertia(ai, ra.cross(imp));
    let dwb = bodies.apply_world_inv_inertia(bi, rb.cross(imp));
    bodies.set_angvel_raw(ai, bodies.angvel(ai) - dwa);
    bodies.set_angvel_raw(bi, bodies.angvel(bi) + dwb);
}

/// 与 `axis` 垂直的单位正交基（固定顺序 ⇒ 确定性）。
#[inline]
fn perp_basis(axis: Vec3) -> [Vec3; 2] {
    let a = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let t1 = a.cross(axis).normalize();
    let t2 = axis.cross(t1).normalize();
    [t1, t2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{PhysConfig as Cfg, Quat, Shape};

    fn world() -> BodySet {
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
    fn integrate(bodies: &mut BodySet, dt: f32) {
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
    fn spherical_holds_anchor_within_tolerance() {
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
    fn fixed_locks_both_position_and_orientation() {
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
    fn revolute_spins_about_axis_and_locks_the_rest() {
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
    fn prismatic_slides_only_along_axis() {
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
    fn revolute_motor_drives_target_and_respects_force_clamp() {
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
    fn prismatic_motor_drives_along_axis() {
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
    fn distance_holds_rest_length() {
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
