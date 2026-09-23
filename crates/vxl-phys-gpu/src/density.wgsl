// 密度相位（与 CPU 版**同式、同遍历序**）：ρ_i = mass·(W(0) + Σ_j W) + Σ_b pmass_j·W
//
// 为什么要"同遍历序"：本仓确定性契约要求 GPU 与 CPU **可对拍**（口径 A：决定性设计 ⇒ 能位级就位级）。
// 邻域按 `cell_start/cell_items`（CPU 侧计数排序网格）**格坐标序 × 格内索引序**枚举，
// 与 CPU 的 `UniformGrid::for_neighbors_in` 完全一致 ⇒ 同一运算序。
//
// **绑定布局**（每核一张；wgpu 要求 storage 访问权限**精确匹配**，且每 stage ≤ 8 个 storage）：
//   全局槽位号（两核一致）：0 params(uniform) | 1 pos | 2 vel | 3 pmass | 4 press
//     | 5 cell_start | 6 cell_items | 7 dens | 8 out(acc.xyz + xsph.xyz 交错)
// 本核只用 0/1/3/5/6/7（`dens` 为读写：本核写出，力核只读）。
//
// **量纲/口径**：本片取纯流体场景（无边界粒子、无 provider）⇒ 与 CPU 的流体分支一致。

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
    _pad: u32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> pos: array<f32>;
@group(0) @binding(3) var<storage, read> pmass: array<f32>;
@group(0) @binding(5) var<storage, read> cell_start: array<u32>;
@group(0) @binding(6) var<storage, read> cell_items: array<u32>;
@group(0) @binding(7) var<storage, read_write> dens: array<f32>;

fn p3(i: u32) -> vec3<f32> {
    return vec3<f32>(pos[i * 3u], pos[i * 3u + 1u], pos[i * 3u + 2u]);
}

fn axis_idx(o: f32, v: f32, inv: f32, n: u32) -> i32 {
    let f = floor((v - o) * inv);
    return clamp(i32(max(f, 0.0)), 0, i32(n) - 1);
}

@compute @workgroup_size(64)
fn density(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= P.n_fluid {
        return;
    }
    let pi = p3(i);
    let ax = axis_idx(P.gmin.x, pi.x, P.inv, P.nx);
    let ay = axis_idx(P.gmin.y, pi.y, P.inv, P.ny);
    let az = axis_idx(P.gmin.z, pi.z, P.inv, P.nz);
    let ny = i32(P.ny);
    let nz = i32(P.nz);
    var sum = P.w0;
    var sum_b = 0.0;
    // 与 CPU 同序：dz 外层、dy 中层、dx 内层；格内按 items 序。
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
                let a = cell_start[ci];
                let b = cell_start[ci + 1u];
                for (var k = a; k < b; k = k + 1u) {
                    let j = cell_items[k];
                    if (j == i) { continue; }
                    let d = pi - p3(j);
                    // ⚠️ 不用 `dot(d,d)`：WGSL 的 dot 归约序未规定，而 CPU 侧 `length_squared`
                    // 是 `x*x + y*y + z*z` **左结合** ⇒ 写成同序才有机会逐位一致（口径 A）。
                    // 口径 A 已试尽（2026-09-23，见 PLAN §9.4 ④）：naga 27 **不支持 `precise`**
                    // （前端无此关键字，实测 "expected assignment"）；bitcast 屏障被优化器折掉
                    // （逐位率一字不变）；显式 fma 对照实验证明残差=**收缩噪声**（最大 6 ulp）。
                    let r2 = d.x * d.x + d.y * d.y + d.z * d.z;
                    if (r2 <= P.h2) {
                        let t = P.h2 - r2;
                        let w = P.k6 * t * t * t;
                        if (j < P.n_fluid) {
                            sum = sum + w;
                        } else {
                            sum_b = sum_b + pmass[j] * w;
                        }
                    }
                }
            }
        }
    }
    dens[i] = P.mass * sum + sum_b;
}
