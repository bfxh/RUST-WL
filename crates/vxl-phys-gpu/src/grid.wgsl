// 网格重建（bins / counts / start / items）——与 CPU `UniformGrid::rebuild` **逐位同表**。
//
// 四个入口与 CPU 的四步一一对应（`crates/vxl-phys-fluid/src/lib.rs::rebuild`）：
//   ① `bin_count`：`bin_i = min(clamp(floor((p − min)·inv), 0, n−1), total−1)`（与 CPU 同式、
//      同钳位顺序）⇒ 逐粒 bin **逐位一致**；`counts[bin+1] += 1`（原子加：值是精确整数、与顺序无关）
//   ② `scan`     ：`counts` 独占前缀和 ⇒ `start[]`（语义与 CPU 的 `start` 相同：`start[k]` = 格 k 起点）
//   ③ `place`    ：`cursor`（= `start` 的副本）上 `atomicAdd` 取槽 ⇒ `items[slot] = i`
//                  原子占位 ⇒ **格内序不定**（这正是第 ④ 步要规范化的对象）
//   ④ `canon`    ：逐格把 `items` 段**按粒子索引升序**插入排序
//                  ⇒ 与 CPU 的"格内索引升序"完全一致，且**结果与原子顺序无关**
//                  ⇒ 逐轮可复现 + 与 CPU 逐位同表
//
// 为什么不做基数排序：见 `docs/PLAN-gpu.md` §10。LSD 基数排序每趟要"直方图 + **有序** rank +
// 稳定散列"，是 O(n·趟) 的大工程；而"原子占位 + 逐格规范化"只要 O(Σ 格内元素²)，而 SPH 的格边 = h
// ⇒ 每格典型 8–32 粒 ⇒ 成本可忽略。**代价 = 最坏情况无上界**（粒子挤在一格时退化）⇒ 用 `cap` 卡住：
// 超限的格**不**规范化并把 `overflow` 加一（表仍"每格集合正确"，但不再是 CPU 同表）
// ⇒ **判据是 `overflow == 0`**；engine 侧可拿它当"回退 CPU 网格"的开关。
//
// 绑定：0 params(uniform) | 1 pos(只读) | 2 bins | 3 counts | 4 start | 5 items | 6 cursor | 7 overflow
// 除 `pos` 外**全部声明为 read_write** ⇒ 一份绑定布局覆盖四个入口（wgpu 要求 storage 访问权限
// 精确匹配 ⇒ 一处只读、别处读写就没法共用布局；`counts`/`cursor`/`overflow` 用 `atomic<u32>`
// 元素类型，读侧用 `atomicLoad`）。

struct GridParams {
    gmin: vec3<f32>,
    inv: f32,
    nx: u32,
    ny: u32,
    nz: u32,
    n: u32,
    total: u32,
    cap: u32,
    _pad0: u32,
    _pad1: u32,
};

@group(0) @binding(0) var<uniform> P: GridParams;
@group(0) @binding(1) var<storage, read> pos: array<f32>;
@group(0) @binding(2) var<storage, read_write> bins: array<u32>;
@group(0) @binding(3) var<storage, read_write> counts: array<atomic<u32>>;
@group(0) @binding(4) var<storage, read_write> start: array<u32>;
@group(0) @binding(5) var<storage, read_write> items: array<u32>;
@group(0) @binding(6) var<storage, read_write> cursor: array<atomic<u32>>;
@group(0) @binding(7) var<storage, read_write> overflow: array<atomic<u32>>;

/// 单轴分箱：与 CPU 的闭包逐字对应——
/// `((v − o) · inv).floor().max(0.0) as u32`，再 `.min(n − 1)`。
/// 注意只做"一次减、一次乘、一次 floor"⇒ 没有可被 FMA 收缩的 `a*b+c` ⇒ 与 CPU 逐位一致。
fn axis_bin(o: f32, v: f32, n: u32) -> u32 {
    let t = (v - o) * P.inv;
    let fl = max(floor(t), 0.0);
    // f32 → u32 的越界语义在各后端不一致 ⇒ 先按 Rust `as` 的饱和语义把浮点钳进来。
    let cl = clamp(fl, 0.0, 4294967295.0);
    return min(u32(cl), n - 1u);
}

@compute @workgroup_size(64)
fn bin_count(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n) {
        return;
    }
    let p = vec3<f32>(pos[i * 3u], pos[i * 3u + 1u], pos[i * 3u + 2u]);
    let bx = axis_bin(P.gmin.x, p.x, P.nx);
    let by = axis_bin(P.gmin.y, p.y, P.ny);
    let bz = axis_bin(P.gmin.z, p.z, P.nz);
    let c = min((bx * P.ny + by) * P.nz + bz, P.total - 1u);
    bins[i] = c;
    // counts[k] = 格 k−1 的粒数（CPU 用 counts[c+1]++ 这一写法让前缀和天然是"独占起点"）
    atomicAdd(&counts[c + 1u], 1u);
}

var<workgroup> s_scan: array<u32, 256>;

/// `counts` → `start`：**含尾前缀和**（单工作组、分块 Hillis-Steele + 逐块游标）。
/// ⚠️ 语义要与 CPU 对齐（这里**踩过一次**）：CPU 把"格 c 的计数"写在 `counts[c+1]`，
/// 于是"`counts` 的**含尾**前缀和"恰好 = "格 k 的起点"。（我第一版写成**独占**前缀 ⇒
/// 整体错一个格桶：`start[1]` 给成 `counts[0]`=0、而 CPU 给 8 ⇒ 表全错但形似。）
/// 全整数运算 ⇒ 与顺序无关，任何调度下都逐位相同。
@compute @workgroup_size(256)
fn scan(@builtin(local_invocation_index) lid: u32) {
    var running = 0u;
    let m = P.total + 1u;
    let chunks = (m + 255u) / 256u;
    for (var ch = 0u; ch < chunks; ch = ch + 1u) {
        let idx = ch * 256u + lid;
        var own = 0u;
        if (idx < m) {
            own = atomicLoad(&counts[idx]);
        }
        s_scan[lid] = own;
        workgroupBarrier();
        var inc = 1u;
        for (var k = 0u; k < 8u; k = k + 1u) {
            var add = 0u;
            if (lid >= inc) {
                add = s_scan[lid - inc];
            }
            workgroupBarrier();
            s_scan[lid] = s_scan[lid] + add;
            workgroupBarrier();
            inc = inc * 2u;
        }
        if (idx < m) {
            start[idx] = running + s_scan[lid];
        }
        running = running + s_scan[255];
        workgroupBarrier();
    }
}

@compute @workgroup_size(64)
fn place(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n) {
        return;
    }
    let c = bins[i];
    let slot = atomicAdd(&cursor[c], 1u);
    items[slot] = i;
}

@compute @workgroup_size(64)
fn canon(@builtin(global_invocation_id) gid: vec3<u32>) {
    let c = gid.x;
    if (c >= P.total) {
        return;
    }
    let a = start[c];
    let b = start[c + 1u];
    if (b - a < 2u) {
        return;
    }
    if (b - a > P.cap) {
        // 最坏情况护栏：不规范化（表仍"每格集合正确"）+ 记账，让判据能看见。
        atomicAdd(&overflow[0], 1u);
        return;
    }
    // 插入排序（段内升序 ⇒ 结果与原子占位顺序无关）
    for (var k = a + 1u; k < b; k = k + 1u) {
        let v = items[k];
        var j = k;
        loop {
            if (j <= a) {
                break;
            }
            if (items[j - 1u] <= v) {
                break;
            }
            items[j] = items[j - 1u];
            j = j - 1u;
        }
        items[j] = v;
    }
}
