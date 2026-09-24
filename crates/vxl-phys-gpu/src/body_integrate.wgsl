// **刚体前缀的积分**（`integrate.wgsl` 的对应物）：让体也在卡上推进（全卡上跑一个世界的最后一块）。
//
// 与 CPU 同式（`vxl-phys-integrate/src/lib.rs` + `vxl-phys-core/src/body.rs`）：
//   v ← v + (g + F·inv_m) · dt                     （半隐式欧拉，速度先行）
//   ω ← ω + [R·((Rᵀτ) ⊙ l_local)] · dt             （`apply_world_inv_inertia`：世界系逆惯量）
//   限速：|v| > max_lin ⇒ 整体缩放；|ω| > max_ang 同理（与 CPU 同款"length 缩放"）
//   x ← x + v · dt
//   q ← normalize(q + ½·dt·quat(ω, 0)·q)           （`Quat::integrate_angular`）
//
// 与 `density.wgsl` 的共享口径：**逐轴展开、不用内建 `dot`/`cross`**（归约序未规定）。
//
// **量纲/口径**：本核只做"给定 F/τ + g ⇒ 一个 dt 的推进"，不碰接触/解算（那些仍在主机侧）
// ⇒ 它是"卡上世界"的**积分腿**，不是完整刚体管线。非动态/睡眠体的跳过由主机负责（本核收到的
// 就是"该积分的体"）。

/// 一个体 112 B（16 对齐）：位置 | 逆质量 | 姿态 | 线速度 | 角速度 | 局部逆惯量对角 | 力 | 力矩。
struct Body {
    pos: vec3<f32>,
    inv_mass: f32,
    rot: vec4<f32>,
    linvel: vec3<f32>,
    _p0: f32,
    angvel: vec3<f32>,
    _p1: f32,
    loc_inv_i: vec3<f32>,
    _p2: f32,
    force: vec3<f32>,
    _p3: f32,
    torque: vec3<f32>,
    _p4: f32,
};

struct Params {
    g: vec3<f32>,
    dt: f32,
    max_lin: f32,
    max_ang: f32,
    n: u32,
    _p: u32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read_write> bodies: array<Body>;

/// `R = Mat3::from_quat(q)`（标准旋转矩阵，与 glam 同形）。
fn quat_to_mat(q: vec4<f32>) -> mat3x3<f32> {
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let w = q.w;
    let x2 = x + x;
    let y2 = y + y;
    let z2 = z + z;
    let xx = x * x2;
    let xy = x * y2;
    let xz = x * z2;
    let yy = y * y2;
    let yz = y * z2;
    let zz = z * z2;
    let wx = w * x2;
    let wy = w * y2;
    let wz = w * z2;
    // 列主序（WGSL 的 mat3x3 按列构造）：col0 / col1 / col2。
    return mat3x3<f32>(
        vec3<f32>(1.0 - (yy + zz), xy + wz, xz - wy),
        vec3<f32>(xy - wz, 1.0 - (xx + zz), yz + wx),
        vec3<f32>(xz + wy, yz - wx, 1.0 - (xx + yy)),
    );
}

/// `apply_world_inv_inertia`：`R·((Rᵀ·τ) ⊙ l_local)`（逐轴展开）。
fn world_inv_inertia(q: vec4<f32>, l: vec3<f32>, t: vec3<f32>) -> vec3<f32> {
    let m = quat_to_mat(q);
    // Rᵀ·τ = 各列与 τ 的点积（逐轴展开）。
    let lt = vec3<f32>(
        m[0].x * t.x + m[0].y * t.y + m[0].z * t.z,
        m[1].x * t.x + m[1].y * t.y + m[1].z * t.z,
        m[2].x * t.x + m[2].y * t.y + m[2].z * t.z,
    );
    let scaled = lt * l;
    // R·(scaled)：列线性组合。
    return m[0] * scaled.x + m[1] * scaled.y + m[2] * scaled.z;
}

/// `Quat::integrate_angular`：`normalize(q + ½·dt·(quat(ω,0) · q))`（Hamilton 积，逐项展开）。
fn integrate_angular(q: vec4<f32>, w: vec3<f32>, dt: f32) -> vec4<f32> {
    // a = quat(w, 0)，b = q ⇒ a*b 的 Hamilton 积（a.w = 0）。
    let dw = -(w.x * q.x + w.y * q.y + w.z * q.z);
    let dx = w.x * q.w + w.y * q.z - w.z * q.y;
    let dy = -w.x * q.z + w.y * q.w + w.z * q.x;
    let dz = w.x * q.y - w.y * q.x + w.z * q.w;
    let h = 0.5 * dt;
    let r = vec4<f32>(q.x + dx * h, q.y + dy * h, q.z + dz * h, q.w + dw * h);
    let len = sqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
    return r * (1.0 / max(len, 1e-12));
}

@compute @workgroup_size(64)
fn integrate(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n) {
        return;
    }
    var b = bodies[i];
    // ① 速度（半隐式欧拉 + 世界系逆惯量 + 限速）。
    var v = b.linvel + (P.g + b.force * b.inv_mass) * P.dt;
    let sp = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (sp > P.max_lin) {
        v = v * (P.max_lin / sp);
    }
    var w = b.angvel + world_inv_inertia(b.rot, b.loc_inv_i, b.torque) * P.dt;
    let ws = sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    if (ws > P.max_ang) {
        w = w * (P.max_ang / ws);
    }
    // ② 位姿（用**新**速度，与 CPU 同序）。
    b.pos = b.pos + v * P.dt;
    b.rot = integrate_angular(b.rot, w, P.dt);
    b.linvel = v;
    b.angvel = w;
    bodies[i] = b;
}
