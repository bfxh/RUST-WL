// 密度相位（与 CPU 版**同式、同遍历序**）：ρ_i = mass·(W(0) + Σ_j W) + Σ_b pmass_j·W
//
// 为什么要"同遍历序"：本仓确定性契约要求 GPU 与 CPU **可对拍**（口径 A：决定性设计 ⇒ 能位级就位级）。
// 邻域按 `cell_start/cell_items`（CPU 侧计数排序网格）**格坐标序 × 格内索引序**枚举，
// 与 CPU 的 `UniformGrid::for_neighbors_in` 完全一致 ⇒ 同一 f32 运算序 ⇒ 期望**逐位相同**。
//
// 布局（避开 WGSL 的 vec3 对齐坑）：数组一律**扁平 f32**，`pmass`/`dens` 每粒一个 f32。

struct Params {
    gmin: vec3<f32>,
    inv: f32,
    h2: f32,
    k6: f32,
    w0: f32,
    mass: f32,
    n_fluid: u32,
    nx: u32,
    ny: u32,
    nz: u32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> pos: array<f32>;
@group(0) @binding(2) var<storage, read> pmass: array<f32>;
@group(0) @binding(3) var<storage, read> cell_start: array<u32>;
@group(0) @binding(4) var<storage, read> cell_items: array<u32>;
@group(0) @binding(5) var<storage, read_write> dens: array<f32>;

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
