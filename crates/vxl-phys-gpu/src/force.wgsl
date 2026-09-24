// 力+黏度相位（与 CPU `force_pass` **同式、同遍历序**）：
//   a_i = g + Σ_j m_j·(p_i/ρ_i² + p_j/ρ_j²)·45/(πh⁶)·(h−r)²·d̂/r
//       + Σ_{v_dn>0} m_j·(α·c·μ/ρ̄)·45/(πh⁶)·(h−r)²·d̂/r     （Monaghan 人工黏度）
//   xs_i = Σ_j m_j·2/(ρ_i+ρ_j)·W(r)·(v_j − v_i)               （XSPH）
//
// 与 `density.wgsl` 同一纪律：**按 CPU 的求和序**（格坐标序 × 格内索引升序）、
// 点积**手写展开**（WGSL 的 `dot` 归约序未规定，CPU 是 x·x+y·y+z·z 左结合）。
//
// **绑定布局**（见 `density.wgsl` 头注；每核一张布局，每 stage ≤ 8 storage buffer 是 wgpu 默认上限
// ⇒ 不抬设备限制、靠布局适配）：本核用 0/1/2/3/4/5/6/7(dens 只读)/8(out 读写)。
// `out` = **acc.xyz + xsph.xyz 交错**（6 个 f32/粒；交错是为了省一个绑定槽）。
// ⚠️ 本核读的 `dens` 由**前一个 pass（density）在同一提交里写**，顺序由命令编码保证
//    ⇒ 与 CPU "先密度后力" 的次序一致。
// ⚠️ `press` 本片仍来自**输入数组**（CPU 侧 EOS 结果）——两侧同输入同式；EOS 搬上 GPU 属下一步。

struct Params {
    gmin: vec3<f32>,
    inv: f32,
    h2: f32,
    k6: f32,
    w0: f32,
    mass: f32,
    ks: f32,
    h: f32,
    alpha_c: f32,
    gvec: vec3<f32>,
    n_fluid: u32,
    nx: u32,
    ny: u32,
    nz: u32,
    /// 总粒子数（与 `density.wgsl` 同布局；本核只用 `n_fluid`——边界粒子的**力**属"反应读回"那一步）。
    n_total: u32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> pos: array<f32>;
@group(0) @binding(2) var<storage, read> vel: array<f32>;
@group(0) @binding(3) var<storage, read> pmass: array<f32>;
@group(0) @binding(4) var<storage, read> press: array<f32>;
@group(0) @binding(5) var<storage, read> cell_start: array<u32>;
@group(0) @binding(6) var<storage, read> cell_items: array<u32>;
@group(0) @binding(7) var<storage, read> dens: array<f32>;
@group(0) @binding(8) var<storage, read_write> out: array<f32>;

fn p3(i: u32) -> vec3<f32> {
    return vec3<f32>(pos[i * 3u], pos[i * 3u + 1u], pos[i * 3u + 2u]);
}
fn v3(i: u32) -> vec3<f32> {
    return vec3<f32>(vel[i * 3u], vel[i * 3u + 1u], vel[i * 3u + 2u]);
}
fn axis_idx(o: f32, v: f32, inv: f32, n: u32) -> i32 {
    let f = floor((v - o) * inv);
    return clamp(i32(max(f, 0.0)), 0, i32(n) - 1);
}

@compute @workgroup_size(64)
fn force(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= P.n_fluid {
        return;
    }
    let pi = p3(i);
    let vi = v3(i);
    let rho_i = dens[i];
    let ci2 = press[i] / (rho_i * rho_i);
    let ax = axis_idx(P.gmin.x, pi.x, P.inv, P.nx);
    let ay = axis_idx(P.gmin.y, pi.y, P.inv, P.ny);
    let az = axis_idx(P.gmin.z, pi.z, P.inv, P.nz);
    let ny = i32(P.ny);
    let nz = i32(P.nz);
    var a = P.gvec;
    var xs = vec3<f32>(0.0, 0.0, 0.0);
    for (var dz = -1; dz <= 1; dz = dz + 1) {
        let z = az + dz;
        if (z < 0 || z >= nz) { continue; }
        for (var dy = -1; dy <= 1; dy = dy + 1) {
            let y = ay + dy;
            if (y < 0 || y >= ny) { continue; }
            for (var dx = -1; dx <= 1; dx = dx + 1) {
                let x = ax + dx;
                if (x < 0 || x >= i32(P.nx)) { continue; }
                let ci = u32((x * ny + y) * nz + z);
                let a0 = cell_start[ci];
                let b0 = cell_start[ci + 1u];
                for (var k = a0; k < b0; k = k + 1u) {
                    let j = cell_items[k];
                    if (j == i) { continue; }
                    let pj = p3(j);
                    let d = pi - pj;
                    let r2 = d.x * d.x + d.y * d.y + d.z * d.z;
                    if (r2 > P.h2) { continue; }
                    let rho_j = dens[j];
                    let cj2 = press[j] / (rho_j * rho_j);
                    let r = sqrt(r2);
                    let t = P.h - r;
                    let mj = pmass[j];
                    let coef = mj * (P.ks * t * t) * (ci2 + cj2);
                    let denom = max(r, 1e-9);
                    a = a + d * (coef / denom);
                    // Monaghan 人工黏度（仅接近对）：v_dn = −(v_i−v_j)·d（展开同序）
                    let vij = vi - v3(j);
                    let vdn = -(vij.x * d.x + vij.y * d.y + vij.z * d.z);
                    if (vdn > 0.0) {
                        let mu = vdn * P.h / (r2 + 0.01 * P.h2);
                        let cc = mj * (P.alpha_c * mu / (0.5 * (rho_i + rho_j))) * (P.ks * t * t);
                        a = a + d * (cc / denom);
                    }
                    // XSPH：Σ m·2/(ρ_i+ρ_j)·W·(v_j − v_i)
                    let tt = P.h2 - r2;
                    let w = P.k6 * tt * tt * tt;
                    xs = xs + (v3(j) - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
                }
            }
        }
    }
    out[i * 6u] = a.x;
    out[i * 6u + 1u] = a.y;
    out[i * 6u + 2u] = a.z;
    out[i * 6u + 3u] = xs.x;
    out[i * 6u + 4u] = xs.y;
    out[i * 6u + 5u] = xs.z;
}
