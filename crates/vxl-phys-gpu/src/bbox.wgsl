//! **包围盒归约**（GPU 常驻管线"每子步重算箱子"的第一步：位置 → `(lo, hi)`）。
//!
//! 背景（`PLAN-gpu.md` §12.3 第一条）：本片箱子**固定**（在 `PacketCfg` 里一次给定）。真后端要
//! 每子步从位置重算 —— CPU 引擎就是每子步 `UniformGrid::rebuild(&pos, h)` 一次。箱子必须与
//! CPU 侧**同一条规则**，否则两条链分箱不同 ⇒ 漂移表失去意义。
//!
//! `reduce` 的确定性口径：**min/max 是精确运算、与求值顺序无关**（不像浮点求和会累加舍入）
//! ⇒ 用原子写不改变结果，不需要"按索引序归约"那套。做法是把 f32 位模式映射成**单调递增**的
//! u32（正数原样；负数按位取反），于是 `atomicMin/atomicMax<u32>` 就是对 f32 取 min/max。
//! 中性元：min 的 `+∞` = `f2o(+∞)` = `0xFF800000`；max 的 `−∞` = `f2o(−∞)` = `0x007FFFFF`
//! ⇒ **调用方每轮必须先把这 6 个槽写成中性元**（主机 `write_buffer` 24 字节，成本可忽略）。
//! （⚠️ 这两个常数**不是** f32 位模式本身：有序映射把正数的符号位翻成 1 ⇒ `+∞` 映射后高位是 1。
//! 首版按位模式写 `0x7F800000`，被 `gpu_box_matches_host_rule` 当场抓红。）
//!
//! **箱子三件套在主机侧算**（`vxl_phys_core::grid::grid_box`）：本核只负责给出 `(lo, hi)`，
//! 主机读回 24 字节后按同一条规则算 `bin/dims` —— 这样"规则"在仓里只有**一处**实现。
//! （首版另写了一个单线程 `box_setup` 核在卡上算，实测它写出的 `boxf` 是未初始化内存
//! ——归约累加器本身逐项正确，问题在那一遍 pass 的落写 ⇒ 已移除，见提交留痕。）
//!
//! 注意：`inv = 1.0 / bin` 由**主机**算，与 CPU 侧同一表达式 ⇒ 逐位相同。

/// 位置（xyz 扁平，长度 `3n`）。
@group(0) @binding(0) var<storage, read> pos: array<f32>;
/// 归约累加器：`[min.x, min.y, min.z, max.x, max.y, max.z]`（有序映射下的 u32）。
@group(0) @binding(1) var<storage, read_write> bb: array<atomic<u32>>;
/// `(n, ngroups, 0, 0)`。
@group(0) @binding(2) var<uniform> rp: vec4<u32>;
/// 工作组成员数（与主机的 `dispatch` 分组口径一致）。
const WG: u32 = 64u;

/// f32 → 单调 u32：正数原样（含 +0），负数把"按位取反 + 翻符号位"。
fn f2o(x: f32) -> u32 {
    let b = bitcast<u32>(x);
    let neg = (b & 0x80000000u) != 0u;
    return select(b ^ 0x80000000u, ~b, neg);
}

/// 全位置 min/max 归约（原子；跨组也用同一个数组 ⇒ 单次 dispatch 即可）。
@compute @workgroup_size(64)
fn reduce(@builtin(global_invocation_id) gid: vec3<u32>) {
    let n = rp.x;
    if (n == 0u) {
        return;
    }
    let stride = rp.y * WG;
    var mn = vec3<u32>(0xFF800000u, 0xFF800000u, 0xFF800000u);
    var mx = vec3<u32>(0x007FFFFFu, 0x007FFFFFu, 0x007FFFFFu);
    var i = gid.x;
    loop {
        if (i >= n) {
            break;
        }
        let o = vec3<u32>(f2o(pos[i * 3u]), f2o(pos[i * 3u + 1u]), f2o(pos[i * 3u + 2u]));
        mn = min(mn, o);
        mx = max(mx, o);
        i = i + stride;
    }
    atomicMin(&bb[0], mn.x);
    atomicMin(&bb[1], mn.y);
    atomicMin(&bb[2], mn.z);
    atomicMax(&bb[3], mx.x);
    atomicMax(&bb[4], mx.y);
    atomicMax(&bb[5], mx.z);
}
