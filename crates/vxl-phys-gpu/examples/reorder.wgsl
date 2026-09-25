// **重排 pass 的成本探针核**（`PLAN-gpu.md` §21.5 第 1 条：那条"按格重排"杠杆的**代价侧**）。
//
// 为什么要单独量：`sorted` 档（格序数据）在两个邻域相位上值 0.64×（10M 档 44.49 → 28.49 ms/子步），
// 但**数据得先搬到格序**——这一次搬运要花多少，决定了这条杠杆净剩多少。两个入口分开量：
//
//   - `gather` ：索引序 → 格序。`pos_c[m] = pos[items[m]]`（**散读 + 连续写**），三张表
//                （pos 12 B / vel 12 B / press 4 B 每粒）。
//   - `scatter`：把两相位的输出（`out` = acc.xyz + xsph.xyz 交错，24 B/粒）**散写回索引序**
//                （`out_idx[items[m]] = out_c[m]`）——这是"格序只当输入副本、下游仍吃索引序"那条路
//                的收尾代价。
//
// ⚠️ 本核**只为计价**，不参与生产路径：它读 `items` 的**方向**与真实用法一致（格序位置 m → 原索引），
// 但输入数据是探针造的（`out_c` 内容无关紧要——成本只看地址形态）。
//
// 绑定：0 uniform | gather: 1 items 2 pos 3 vel 4 press 5 pos_c 6 vel_c 7 press_c
//       scatter: 1 items 2 out_c | 3 out_idx

struct Params {
    n: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> items: array<u32>;
@group(0) @binding(2) var<storage, read> src_a: array<f32>;
@group(0) @binding(3) var<storage, read> src_b: array<f32>;
@group(0) @binding(4) var<storage, read> src_c: array<f32>;
@group(0) @binding(5) var<storage, read_write> dst_a: array<f32>;
@group(0) @binding(6) var<storage, read_write> dst_b: array<f32>;
@group(0) @binding(7) var<storage, read_write> dst_c: array<f32>;

/// 二维分派展平（与各相位核同式；见 `probe::split_2d`）。
fn flat(gid: vec3<u32>) -> u32 {
    return gid.x + gid.y * (65535u * 64u);
}

@compute @workgroup_size(64)
fn gather(@builtin(global_invocation_id) gid: vec3<u32>) {
    let m = flat(gid);
    if m >= P.n {
        return;
    }
    let k = items[m];
    // 逐分量拷贝：**不写 `dst[m*3..] = src[k*3..]` 那种向量化形式**，因为生产实现必须与它逐字对应
    // （口径：本探针量的是"搬运这 28 字节"的地址形态，不是向量的利用）。
    for (var c = 0u; c < 3u; c = c + 1u) {
        dst_a[m * 3u + c] = src_a[k * 3u + c];
        dst_b[m * 3u + c] = src_b[k * 3u + c];
    }
    dst_c[m] = src_c[k];
}

/// `out` 在两种排布下的互相搬运：读**格序**的 `out_c`，**散写**回原索引序。
/// （入参复用 `src_a` = out_c（格序，6 个 f32/粒），`dst_a` = out（索引序，6 个 f32/粒）。）
@compute @workgroup_size(64)
fn scatter(@builtin(global_invocation_id) gid: vec3<u32>) {
    let m = flat(gid);
    if m >= P.n {
        return;
    }
    let k = items[m];
    for (var c = 0u; c < 6u; c = c + 1u) {
        dst_a[k * 6u + c] = src_a[m * 6u + c];
    }
}
