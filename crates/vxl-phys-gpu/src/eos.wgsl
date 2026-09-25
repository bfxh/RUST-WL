// 压力 EOS：与 CPU `FluidSystem::pressure_pass` **同式**。
//
// `q = ρ/ρ0`；γ = 7 时走 `q*q2*q4`（**左结合**，与 CPU 那句字面同序）；`p = b_tait·(q⁷ − 1)`；
// 拉伸（p < 0）是否钳到 0 由 `clamp` 开关定（= `cfg.tensile_instability_suppression`）。
//
// ⚠️ γ ≠ 7 那条分支依赖两边 `pow` 实现一致 ⇒ 探针只在 γ = 7（引擎默认）下声明同式。

struct EosParams {
    b_tait: f32,
    rho0: f32,
    gamma: f32,
    clamp: u32,
    n: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0) var<uniform> E: EosParams;
@group(0) @binding(1) var<storage, read> dens: array<f32>;
@group(0) @binding(2) var<storage, read_write> press: array<f32>;

@compute @workgroup_size(64)
fn eos(@builtin(global_invocation_id) gid: vec3<u32>) {
    // 二维分派展平（见 `probe::split_2d`）：一维时 gid.y == 0 ⇒ 逐粒索引与旧式同（逐位不变）。
    let i = gid.x + gid.y * (65535u * 64u);
    if (i >= E.n) {
        return;
    }
    let q = dens[i] / E.rho0;
    var p = 0.0;
    if (abs(E.gamma - 7.0) < 1e-6) {
        let q2 = q * q;
        let q4 = q2 * q2;
        p = E.b_tait * (q * q2 * q4 - 1.0);
    } else {
        p = E.b_tait * (pow(q, E.gamma) - 1.0);
    }
    if (E.clamp != 0u && p < 0.0) {
        p = 0.0;
    }
    press[i] = p;
}
