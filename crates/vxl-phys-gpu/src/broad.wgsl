// **均匀格宽相**（卡上）：与 `crates/vxl-phys-broad/src/grid_phase.rs::GridBroadPhase` **同规则**
// ——目标是**配对集合逐位相同**（`base` 侧判据），而不是"性能差不多"。
//
// **为什么这个相位可以逐位同**：它只吃**逐体 AABB**（主机用 `shape_aabb` 算好，与 CPU 同一份数据）+
// 整数格坐标 + f32 比较 ⇒ 全是精确运算（§9.6："整数相位逐位可同"）。
//
// **与 CPU 的两处**有意差别**（都不影响判据）：
// 1. **格的表示**：CPU 用 `HashMap<CellKey, Vec<u32>>`（稀疏、坐标 = 世界格坐标）；本核用**稠密网格**
//    （主机给出全局格范围 + 原点），空格外无影响 —— 配对集合与"哪些格存在"无关。
// 2. **格内条目序**：CPU 按体序升序；本核用原子游标（不定序）。**最终输出要 `sort+dedup`**（CPU 也这么做）
//    ⇒ 输出与序无关 ⇒ 仍然逐位确定。
//
// 输出：`pairs`（每对 2 个 u32，`a < b` 归一）+ `pair_count`（原子计数）+ `overflow`（上限护栏）。
// 主机侧：读回 → `sort_unstable + dedup` → 与 CPU 的同一份比对。

struct Params {
    /// 稠密网格 (0,0,0) 格对应的**世界格坐标**（`cell_of` 的结果减去它得到稠密索引）。
    cell_min: vec3<i32>,
    /// 体数。
    n: u32,
    /// 稠密网格三维尺寸。
    dims: vec3<u32>,
    /// `dims.x * dims.y * dims.z`。
    total: u32,
    /// `1 / cell_size`（**主机算好** ⇒ 与 CPU 的 `1.0 / cell_size` 逐位同）。
    inv_cell: f32,
    /// 配对缓冲容量（对数）。
    cap_pairs: u32,
    _p0: u32,
    _p1: u32,
};

// 逐体 AABB：`min.xyz | pad | max.xyz | dyn`（32 B；`dyn` 用 f32 存 0/1 ⇒ 无位模式歧义）。
@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> boxes: array<f32>;
@group(0) @binding(2) var<storage, read_write> counts: array<atomic<u32>>;
@group(0) @binding(3) var<storage, read_write> start: array<u32>;
@group(0) @binding(4) var<storage, read_write> cursor: array<atomic<u32>>;
@group(0) @binding(5) var<storage, read_write> items: array<u32>;
@group(0) @binding(6) var<storage, read_write> pairs: array<u32>;
@group(0) @binding(7) var<storage, read_write> pair_count: atomic<u32>;
/// 护栏：条目总数超 `items` 容量、或配对数超 `cap_pairs` 时递增（>0 ⇒ 本帧**不可信**，调用方回退）。
@group(0) @binding(8) var<storage, read_write> overflow: atomic<u32>;

/// 世界坐标 → 世界格坐标。**与 CPU `GridBroadPhase::cell_of` 逐字同式**
/// （`(v * inv).floor().clamp(±1e6) as i32`；`inv` 由主机给 ⇒ 同值）。
fn cell_of(v: f32) -> i32 {
    let f = floor(v * P.inv_cell);
    let c = clamp(f, -1000000.0, 1000000.0);
    return i32(c);
}

/// 体 `i` 的 AABB 下界（`.xyz`）与动态标志（`.w`）。
fn bb(i: u32) -> vec4<f32> {
    let o = i * 8u;
    return vec4<f32>(boxes[o], boxes[o + 1u], boxes[o + 2u], boxes[o + 7u]);
}
/// 体 `i` 的 AABB 上界（`.xyz`；`.w` 空）。
fn bb_max(i: u32) -> vec4<f32> {
    let o = i * 8u;
    return vec4<f32>(boxes[o + 4u], boxes[o + 5u], boxes[o + 6u], 0.0);
}
fn is_dyn(i: u32) -> bool {
    return boxes[i * 8u + 7u] > 0.5;
}
/// AABB 相交（**与 `Aabb::overlaps` 逐项同式**：`a.min ≤ b.max && b.min ≤ a.max`，逐轴 6 个比较）。
fn overlaps(i: u32, j: u32) -> bool {
    let amin = bb(i);
    let amax = bb_max(i);
    let bmin = bb(j);
    let bmax = bb_max(j);
    let ok1 = amin.x <= bmax.x && amin.y <= bmax.y && amin.z <= bmax.z;
    let ok2 = bmin.x <= amax.x && bmin.y <= amax.y && bmin.z <= amax.z;
    return ok1 && ok2;
}

/// 稠密索引（把世界格坐标平移到网格原点）。**越界返回 `0xFFFFFFFF`**（调用方跳过并记账：
/// 主机按"全局 AABB 范围"定网格 ⇒ 正常不该越界，这是护栏）。
fn dense(c: vec3<i32>) -> u32 {
    let x = c.x - P.cell_min.x;
    let y = c.y - P.cell_min.y;
    let z = c.z - P.cell_min.z;
    if (x < 0 || y < 0 || z < 0) {
        return 0xFFFFFFFFu;
    }
    let ux = u32(x);
    let uy = u32(y);
    let uz = u32(z);
    if (ux >= P.dims.x || uy >= P.dims.y || uz >= P.dims.z) {
        return 0xFFFFFFFFu;
    }
    return (ux * P.dims.y + uy) * P.dims.z + uz;
}

/// ① 分箱计数：每体对其 AABB 覆盖的每个格 +1（格序 z 内层 ⇒ 与 CPU 的 `insert` 同序）。
@compute @workgroup_size(64)
fn bin_count(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n) {
        return;
    }
    let lo = bb(i);
    let hi = bb_max(i);
    let cmin = vec3<i32>(cell_of(lo.x), cell_of(lo.y), cell_of(lo.z));
    let cmax = vec3<i32>(cell_of(hi.x), cell_of(hi.y), cell_of(hi.z));
    for (var x = cmin.x; x <= cmax.x; x = x + 1) {
        for (var y = cmin.y; y <= cmax.y; y = y + 1) {
            for (var z = cmin.z; z <= cmax.z; z = z + 1) {
                let ci = dense(vec3<i32>(x, y, z));
                if (ci == 0xFFFFFFFFu) {
                    atomicAdd(&overflow, 1u);
                    continue;
                }
                atomicAdd(&counts[ci], 1u);
            }
        }
    }
}

/// ② 前缀和（**单线程顺序扫** ⇒ 逐位确定；`total` 是格数，本档场景量级 ≤ 十万 ⇒ 够用）。
/// 一次写两份：`start`（只读，长度 **total + 1**）与 `cursor`（`place` 的可变游标）——沿用流体档的形状。
@compute @workgroup_size(1)
fn scan() {
    var acc = 0u;
    for (var c = 0u; c < P.total; c = c + 1u) {
        start[c] = acc;
        atomicStore(&cursor[c], acc);
        acc = acc + atomicLoad(&counts[c]);
    }
    start[P.total] = acc; // 末格终点（`gen_pairs` 用 `start[ci + 1]`，省掉特例）
    if (acc > arrayLength(&items)) {
        atomicAdd(&overflow, 1u);
    }
}

/// ③ 落位：每体写进它覆盖的每个格（格内序不定 ⇒ 最终 `sort+dedup` 之后与序无关）。
@compute @workgroup_size(64)
fn place(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n) {
        return;
    }
    let lo = bb(i);
    let hi = bb_max(i);
    let cmin = vec3<i32>(cell_of(lo.x), cell_of(lo.y), cell_of(lo.z));
    let cmax = vec3<i32>(cell_of(hi.x), cell_of(hi.y), cell_of(hi.z));
    for (var x = cmin.x; x <= cmax.x; x = x + 1) {
        for (var y = cmin.y; y <= cmax.y; y = y + 1) {
            for (var z = cmin.z; z <= cmax.z; z = z + 1) {
                let ci = dense(vec3<i32>(x, y, z));
                if (ci == 0xFFFFFFFFu) {
                    continue; // 已在 `bin_count` 记过账
                }
                let k = atomicAdd(&cursor[ci], 1u);
                if (k < arrayLength(&items)) {
                    items[k] = i;
                }
            }
        }
    }
}

/// ④ 配对：**每个动态体一条线程**，扫它覆盖的格里的条目；规则与 CPU 逐条对齐：
/// - 静态条目 `j`：归一成 `(min,max)` 再测（CPU 也这么做）；**静态-静态永不入对**（只有动态体发起查询）；
/// - 动态条目 `j`：只取 `j > i`（去重靠索引而不是序 ⇒ 与格内序无关）。
@compute @workgroup_size(64)
fn gen_pairs(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n || !is_dyn(i)) {
        return;
    }
    let lo = bb(i);
    let hi = bb_max(i);
    let cmin = vec3<i32>(cell_of(lo.x), cell_of(lo.y), cell_of(lo.z));
    let cmax = vec3<i32>(cell_of(hi.x), cell_of(hi.y), cell_of(hi.z));
    for (var x = cmin.x; x <= cmax.x; x = x + 1) {
        for (var y = cmin.y; y <= cmax.y; y = y + 1) {
            for (var z = cmin.z; z <= cmax.z; z = z + 1) {
                let ci = dense(vec3<i32>(x, y, z));
                if (ci == 0xFFFFFFFFu) {
                    continue;
                }
                let a0 = start[ci];
                let a1 = start[ci + 1u];
                for (var k = a0; k < a1; k = k + 1u) {
                    let j = items[k];
                    if (j == i) {
                        continue;
                    }
                    if (is_dyn(j)) {
                        if (j > i && overlaps(i, j)) {
                            push_pair(i, j);
                        }
                    } else {
                        let a = min(i, j);
                        let b = max(i, j);
                        if (overlaps(a, b)) {
                            push_pair(a, b);
                        }
                    }
                }
            }
        }
    }
}

/// 追加一对（上限护栏：超容量只记账、不写 ⇒ 调用方据 `overflow` 回退而不是拿半份）。
fn push_pair(a: u32, b: u32) {
    let slot = atomicAdd(&pair_count, 1u);
    if (slot >= P.cap_pairs) {
        atomicAdd(&overflow, 1u);
        return;
    }
    pairs[slot * 2u] = a;
    pairs[slot * 2u + 1u] = b;
}
