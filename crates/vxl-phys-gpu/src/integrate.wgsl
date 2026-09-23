// 半隐式欧拉积分：与 CPU `FluidSystem::substep` 末段**同式**。
//
// `v ← v + a·dt + ε·xsph`；`|v| > vmax` 时整体缩回（CFL 防穿隧）；`x ← x + v·dt`。
// 只积分**流体粒子**（索引前缀）——本片是纯流体场景，直接对全部粒子做。
//
// ⚠️ 与 CPU 的已知差别（**口径 B**，见 `docs/PLAN-gpu.md` §9.6）：`a·dt + ε·xsph` 处的乘加可能被
// 驱动收缩成 FMA ⇒ 与 CPU 不逐位一致。这里只保证**同式**。
// 输入 `out` = 力相位的输出：`acc.xyz + xsph.xyz` **交错**（6 个 f32/粒）。

struct IntParams {
    dt: f32,
    vmax: f32,
    eps: f32,
    n: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    _pad3: u32,
};

@group(0) @binding(0) var<uniform> I: IntParams;
@group(0) @binding(1) var<storage, read_write> pos: array<f32>;
@group(0) @binding(2) var<storage, read_write> vel: array<f32>;
@group(0) @binding(3) var<storage, read> out: array<f32>;

@compute @workgroup_size(64)
fn integrate(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= I.n) {
        return;
    }
    let k = i * 3u;
    let a = vec3<f32>(out[i * 6u], out[i * 6u + 1u], out[i * 6u + 2u]);
    let xs = vec3<f32>(out[i * 6u + 3u], out[i * 6u + 4u], out[i * 6u + 5u]);
    var v = vec3<f32>(vel[k], vel[k + 1u], vel[k + 2u]) + a * I.dt + xs * I.eps;
    let s2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (s2 > I.vmax * I.vmax) {
        v = v * (I.vmax / sqrt(s2));
    }
    vel[k] = v.x;
    vel[k + 1u] = v.y;
    vel[k + 2u] = v.z;
    let x = vec3<f32>(pos[k], pos[k + 1u], pos[k + 2u]) + v * I.dt;
    pos[k] = x.x;
    pos[k + 1u] = x.y;
    pos[k + 2u] = x.z;
}
