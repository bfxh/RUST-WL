// **格序副本的"重排"与"回写"**（`PLAN-gpu.md` §23.1 第 1、3 步；默认关，见 `PacketCfg::sort_copies`）。
//
// 为什么要它：两个邻域相位吃"按格序摆"的数据时快 **1.56×**（10M 档 44.49 → 28.49 ms/子步，§21.2），
// 但数据得先搬过去。这一版只搬**会变的三张**（pos/vel/press = 28 B/粒）：纯流体场景里 `pmass` 全是
// 常量 ⇒ 在建管线时填一次，不必每子步重排。（成本实测 2.54 + 2.15 ms/子步，§22。）
//
// 两个入口：
//   - `gather` ：索引序 → 格序。`x_c[m] = x[items[m]]`（**散读 + 连续写**；`items` 就是引擎那张
//                计数排序表，与 CPU 的枚举序同一张）。
//   - `scatter`：两相位输出（acc.xyz + xsph.xyz 交错，24 B/粒）从格序**散写回索引序**
//                ⇒ 下游（积分 / 反作用聚合 / 回读 / 耦合）**一字不动**。
//
// **为什么核可以不改**：格序档下，每个粒子的邻域枚举序列与平铺档**逐条相同**（判据见 §23），而
// `cell_start` 只取决于"每格有几个粒子"⇒ 两档**逐项相同**。于是只要把 `cell_items` 绑成**恒等表**
// （`id[k] = k`）并把 pos/vel/pmass/press 绑成副本，核里那行 `j = cell_items[k]` 拿到的就是格序
// 副本的下标 ⇒ 读变成连续、且**枚举序一字未变**。
//
// ⚠️ 两个入口都按 `n_total`（全部粒子）搬运；`Params` 只读末段的 `n_total`，前 76 字节按相位 uniform
// 的布局占位（不能只声明一个小 struct——字节偏移会错）。
//
// 绑定：0 params | gather: 1 items 2 pos 3 vel 4 press 5 pos_c 6 vel_c 7 press_c
//                   | scatter: 1 items 2 out_c 3 out

struct Params {
    a0: vec4<f32>, // 0..16   gmin.xyz | inv
    a1: vec4<f32>, // 16..32  h2 | k6 | w0 | mass
    a2: vec4<f32>, // 32..48  ks | h | alpha_c | —
    a3: vec4<f32>, // 48..64  gvec.xyz | n_fluid
    tail: vec4<u32>, // 64..80 nx | ny | nz | n_total
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> items: array<u32>;
@group(0) @binding(2) var<storage, read> src_a: array<f32>;
@group(0) @binding(3) var<storage, read> src_b: array<f32>;
@group(0) @binding(4) var<storage, read> src_c: array<f32>;
@group(0) @binding(5) var<storage, read_write> dst_a: array<f32>;
@group(0) @binding(6) var<storage, read_write> dst_b: array<f32>;
@group(0) @binding(7) var<storage, read_write> dst_c: array<f32>;

/// 二维分派展平（与各相位核同式，见 `probe::split_2d`）。
fn flat(gid: vec3<u32>) -> u32 {
    return gid.x + gid.y * (65535u * 64u);
}

@compute @workgroup_size(64)
fn gather(@builtin(global_invocation_id) gid: vec3<u32>) {
    let m = flat(gid);
    if m >= P.tail.w {
        return;
    }
    let k = items[m];
    for (var c = 0u; c < 3u; c = c + 1u) {
        dst_a[m * 3u + c] = src_a[k * 3u + c];
        dst_b[m * 3u + c] = src_b[k * 3u + c];
    }
    dst_c[m] = src_c[k];
}

@compute @workgroup_size(64)
fn scatter(@builtin(global_invocation_id) gid: vec3<u32>) {
    let m = flat(gid);
    if m >= P.tail.w {
        return;
    }
    let k = items[m];
    for (var c = 0u; c < 6u; c = c + 1u) {
        dst_a[k * 6u + c] = src_a[m * 6u + c];
    }
}
