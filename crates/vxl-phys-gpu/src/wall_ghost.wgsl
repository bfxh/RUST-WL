// 提供者壁面的**镜像鬼影**（GPU 侧）：把固体一侧缺失的核质量按"真实邻居的镜像"补回来。
//
// 与 CPU 同式同序（`vxl-phys-fluid/src/fluid_density.rs` 的 `wall_planes` 段）：
//     for 邻居 j（格序）: for 壁面 k:  g = mirror_k(p_j); 若 |p_i − g| ≤ h ⇒ Σ += k6·(h² − |p_i − g|²)³
// 与 CPU 的唯一差别是**求和序**：CPU 把鬼影项与流体项**交错**累加，本核单独累加后一次性加到 `dens[i]`
// ⇒ 口径 B（与其它相位同源）。
//
// **稀疏三段**（只有近壁粒子有条目；主机侧用 `FluidSystem::wall_planes_in` 收集，≤8 面/粒）：
//   `ids[e]` = 粒子号；`start[e]..start[e+1]` = 该粒在 `planes` 里的区间；条目数 = `arrayLength(&ids)`。
//
// **绑定**（1 uniform + 7 storage；uniform 直接复用相位 uniform ⇒ 网格映射与密度核**逐字一致**）：
//   0 params(复用的相位 uniform) | 1 pos | 2 cell_start | 3 cell_items | 4 dens(rw) | 5 ids | 6 start | 7 planes

/// 与 `density.wgsl` 的 `Params` **逐字相同**（复用同一张 uniform ⇒ 不新增 uniform、也不会写歪）。
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
    n_total: u32,
};

/// 壁面平面：接触点 + 外法线（外法线指向流体侧 ⇒ `sdf = (p − point)·n > 0` = 在固体外）。
struct Wall {
    point: vec3<f32>,
    _p: f32,
    normal: vec3<f32>,
    _p2: f32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> pos: array<f32>;
@group(0) @binding(2) var<storage, read> cell_start: array<u32>;
@group(0) @binding(3) var<storage, read> cell_items: array<u32>;
@group(0) @binding(4) var<storage, read_write> dens: array<f32>;
@group(0) @binding(5) var<storage, read> ids: array<u32>;
@group(0) @binding(6) var<storage, read> start: array<u32>;
@group(0) @binding(7) var<storage, read> planes: array<Wall>;

fn p3(i: u32) -> vec3<f32> {
    return vec3<f32>(pos[i * 3u], pos[i * 3u + 1u], pos[i * 3u + 2u]);
}

fn axis_idx(o: f32, v: f32, inv: f32, n: u32) -> i32 {
    let f = floor((v - o) * inv);
    return clamp(i32(max(f, 0.0)), 0, i32(n) - 1);
}

@compute @workgroup_size(64)
fn wall_ghost(@builtin(global_invocation_id) gid: vec3<u32>) {
    let e = gid.x;
    if (e >= arrayLength(&ids)) {
        return;
    }
    let i = ids[e];
    let st = start[e];
    let en = start[e + 1u];
    if (st >= en) {
        return;
    }
    let pi = p3(i);
    let ax = axis_idx(P.gmin.x, pi.x, P.inv, P.nx);
    let ay = axis_idx(P.gmin.y, pi.y, P.inv, P.ny);
    let az = axis_idx(P.gmin.z, pi.z, P.inv, P.nz);
    let ny = i32(P.ny);
    let nz = i32(P.nz);
    var add = 0.0;
    // 邻居遍历与密度核**同序**（dz 外层 / dy 中层 / dx 内层；格内按 items 序）。
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
                    // 与 CPU 同式：d = p_i − p_j（**逐轴展开**，不用 dot——归约序未规定）。
                    let d = pi - p3(j);
                    let r2 = d.x * d.x + d.y * d.y + d.z * d.z;
                    if (r2 > P.h2) { continue; }
                    let pj = pi - d;
                    for (var q = st; q < en; q = q + 1u) {
                        let wall = planes[q];
                        let rel = pj - wall.point;
                        let dn = rel.x * wall.normal.x + rel.y * wall.normal.y + rel.z * wall.normal.z;
                        let g = pj - wall.normal * (2.0 * dn);
                        let dr = pi - g;
                        let rg2 = dr.x * dr.x + dr.y * dr.y + dr.z * dr.z;
                        if (rg2 <= P.h2) {
                            let t = P.h2 - rg2;
                            add = add + P.k6 * t * t * t;
                        }
                    }
                }
            }
        }
    }
    // 密度核已乘过 `mass`（流体项）⇒ 鬼影同样按 `mass` 计入（与 CPU 的 `mass·sum` 一致）。
    if (add != 0.0) {
        dens[i] = dens[i] + P.mass * add;
    }
}
