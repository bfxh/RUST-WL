//! helpers：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

#[allow(clippy::too_many_arguments)] // 接触质量项的规范参数集（M1 TGS-Soft 重构时收敛为结构体）
pub(crate) fn contact_mass(
    im_a: f32,
    im_b: f32,
    ra: Vec3,
    rb: Vec3,
    dir: Vec3,
    bodies: &BodySet,
    a: usize,
    b: usize,
) -> f32 {
    let mut k = im_a + im_b;
    if im_a > 0.0 {
        let rn = ra.cross(dir);
        let w = bodies.apply_world_inv_inertia(a, rn);
        k += w.cross(ra).dot(dir);
    }
    if im_b > 0.0 {
        let rn = rb.cross(dir);
        let w = bodies.apply_world_inv_inertia(b, rn);
        k += w.cross(rb).dot(dir);
    }
    if k > 1e-12 {
        1.0 / k
    } else {
        0.0
    }
}

/// 切向 2×2 有效质量矩阵的**交叉项** k12（Rapier `ContactConstraintTangentPart.r[2]`
/// 同源）：k12 = J1ᵀ·M⁻¹·J2 = (im_a+im_b)·(d1·d2) + Σ (r×d1)ᵀ·I⁻¹·(r×d2)。
/// 各向异性 K 下逐轴（对角）解会留正交残差并旋转能量——摩擦模式在大堆里
/// 被泵成弹射（金样源码注释原文）；必须联立解。
#[allow(clippy::too_many_arguments)] // 与 contact_mass 同参数集（热路径内联）
pub(crate) fn contact_mass_cross(
    im_a: f32,
    im_b: f32,
    ra: Vec3,
    rb: Vec3,
    d1: Vec3,
    d2: Vec3,
    bodies: &BodySet,
    a: usize,
    b: usize,
) -> f32 {
    let mut k = (im_a + im_b) * d1.dot(d2);
    if im_a > 0.0 {
        let w = bodies.apply_world_inv_inertia(a, ra.cross(d2));
        k += ra.cross(d1).dot(w);
    }
    if im_b > 0.0 {
        let w = bodies.apply_world_inv_inertia(b, rb.cross(d2));
        k += rb.cross(d1).dot(w);
    }
    k
}

pub(crate) fn tangents(n: Vec3) -> (Vec3, Vec3) {
    let refr = if n.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let t1 = n.cross(refr).normalize();
    let t2 = n.cross(t1);
    (t1, t2)
}

/// 并查集 find（路径减半）。固定规则：小索引为根（确定性）。
pub(crate) fn find_small_root(parent: &mut [u32], x: u32) -> u32 {
    let mut x = x;
    while parent[x as usize] != x {
        let g = parent[x as usize];
        parent[x as usize] = parent[g as usize];
        x = parent[x as usize];
    }
    x
}

pub(crate) fn union_small_root(parent: &mut [u32], x: u32, y: u32) {
    let rx = find_small_root(parent, x);
    let ry = find_small_root(parent, y);
    if rx != ry && rx < ry {
        parent[ry as usize] = rx;
    } else if rx != ry {
        parent[rx as usize] = ry;
    }
}

/// 组内速度读取：局部索引 u32::MAX = 静态侧。静止体速度恒为精确 0
/// （inv_mass=0 的旧路径空写不改变 0 值），读 Vec3::ZERO 与其 bit 级等价。
#[inline]
pub(crate) fn group_vel(lv: &[Vec3], av: &[Vec3], local_of: &[u32], i: usize, r: Vec3) -> Vec3 {
    let k = local_of[i];
    if k == u32::MAX {
        Vec3::ZERO
    } else {
        let q = k as usize;
        lv[q] + av[q].cross(r)
    }
}

/// 组内冲量施加：动态侧写 scratch；静态/越组侧跳过（旧路径为 inv_mass=0
/// 的空写，数值结果相同）。
#[inline]
#[allow(clippy::too_many_arguments)] // 组内热路径内联目标：避免引入打包结构体的额外构造成本
pub(crate) fn group_apply(
    lv: &mut [Vec3],
    av: &mut [Vec3],
    local_of: &[u32],
    iw: &[Mat3],
    im: &[f32],
    i: usize,
    ra: Vec3,
    imp: Vec3,
    minus: bool,
) {
    let k = local_of[i];
    if k == u32::MAX {
        return;
    }
    let q = k as usize;
    let m = im[q];
    // 世界逆惯量矩阵每帧预积（`R·diag(inv_local)·Rᵀ`）⇒ 这里只剩**一次**矩阵乘；
    // 旧路径每点每轮要 `Rᵀ·x → diag → R·(...)` 两次矩阵乘。数学等价、舍入不同
    // （启用即换代；见 EXPERIMENTS 2026-09-15「每点求解成本」）。
    // A3：`iw`/`im` 都按**组内局部索引**读（原为全局散点：36B×全体数 + 4B×全体数）。
    let dw = iw[q].mul_vec3(ra.cross(imp));
    if minus {
        lv[q] -= imp * m;
        av[q] -= dw;
    } else {
        lv[q] += imp * m;
        av[q] += dw;
    }
}
