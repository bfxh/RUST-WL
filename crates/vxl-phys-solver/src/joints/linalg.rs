//! linalg：从 joints.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 角约束的 3×3 有效质量：`Ia⁻¹ + Ib⁻¹`（世界系）。
#[inline]
pub(crate) fn ang_k(bodies: &BodySet, ai: usize, bi: usize) -> [[f32; 3]; 3] {
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
pub(crate) fn point_k(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3) -> [[f32; 3]; 3] {
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
pub(crate) fn proj(k: [[f32; 3]; 3], a: Vec3, b: Vec3) -> f32 {
    let ka = [
        k[0][0] * a.x + k[0][1] * a.y + k[0][2] * a.z,
        k[1][0] * a.x + k[1][1] * a.y + k[1][2] * a.z,
        k[2][0] * a.x + k[2][1] * a.y + k[2][2] * a.z,
    ];
    ka[0] * b.x + ka[1] * b.y + ka[2] * b.z
}

/// 3×3 解（克莱姆 + 行列式奇异守卫；确定性）。
#[inline]
pub(crate) fn solve3(k: [[f32; 3]; 3], rhs: Vec3) -> Option<Vec3> {
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
pub(crate) fn rel_vel(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3) -> Vec3 {
    let la = bodies.linvel[ai] + bodies.angvel(ai).cross(ra);
    let lb = bodies.linvel[bi] + bodies.angvel(bi).cross(rb);
    lb - la
}

/// 沿单位轴 `n` 的有效质量（对角近似）。
#[inline]
pub(crate) fn row_mass(bodies: &BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3, n: Vec3) -> f32 {
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
pub(crate) fn apply_ang_pair(bodies: &mut BodySet, ai: usize, bi: usize, imp: Vec3) {
    let dwa = bodies.apply_world_inv_inertia(ai, imp);
    let dwb = bodies.apply_world_inv_inertia(bi, imp);
    bodies.set_angvel_raw(ai, bodies.angvel(ai) - dwa);
    bodies.set_angvel_raw(bi, bodies.angvel(bi) + dwb);
}

/// 施加成对线性冲量（b 侧 +、a 侧 −）。#[inline]
pub(crate) fn apply_pair(
    bodies: &mut BodySet,
    ai: usize,
    bi: usize,
    ra: Vec3,
    rb: Vec3,
    imp: Vec3,
) {
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
pub(crate) fn perp_basis(axis: Vec3) -> [Vec3; 2] {
    let a = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let t1 = a.cross(axis).normalize();
    let t2 = axis.cross(t1).normalize();
    [t1, t2]
}
