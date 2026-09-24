// **窄相（卡上）：固定槽**。字数/偏移/种类值与主机 `crate::narrow` 的 `SLOT_WORDS` /
// `BODY_WORDS` / `KIND_*` **逐字对齐**（改了这边必须同步那边，否则槽会错位）。
//
// 槽 = 26 个 u32（104 B）/对，**对序即槽序**（不需要重排 ⇒ 流形表结构天然确定）：
//   w0 a | w1 b | w2..4 normal.xyz | w5 count | 4×(point.xyz | depth | feature)
//   `count` = 0（无流形）/ 1..4（点数）/ `NOT_HANDLED`(0xffffffff ⇒ 本档不接手该对，
//   调用方按**主机回填**处理并写进同一槽位）。
//
// 逐体输入 = 12 个 u32（48 B）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`
//   —— 浮点走 f32 位模式（`as_f32`），`kind` 是**裸整数**（1 = 球，`p0` = 半径；0 = 不接手）。
//   `rot` / `p1` / `p2` 本片不用，为后续族（盒的轴向与半长）预留 ⇒ 布局不动、只加分派。
//
// **本片只接球×球**（`PLAN-gpu.md` §17.5 的落地顺序：先用约定最简的一族把槽位/回读/判据
// 三件事跑通），几何**逐字照抄** CPU `pair_shaped.rs` 的球×球臂（含同心球那条特殊分支）。

const SLOT_WORDS: u32 = 26u;
const PT_BASE: u32 = 6u;
const PT_WORDS: u32 = 5u;
const W_NX: u32 = 2u;
const W_CNT: u32 = 5u;
const NOT_HANDLED: u32 = 0xffffffffu;

const BODY_WORDS: u32 = 12u;
const KIND_AT: u32 = 7u;
const P0_AT: u32 = 8u;
const KIND_SPHERE: u32 = 1u;

struct Params {
    n_bodies: u32,
    n_pairs: u32,
    pad0: u32,
    pad1: u32,
};

@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> bodies: array<u32>;
@group(0) @binding(2) var<storage, read> pairs: array<u32>;
@group(0) @binding(3) var<storage, read_write> slots: array<u32>;

fn as_f32(w: u32) -> f32 {
    return bitcast<f32>(w);
}

fn body_pos(base: u32) -> vec3<f32> {
    return vec3<f32>(
        as_f32(bodies[base]),
        as_f32(bodies[base + 1u]),
        as_f32(bodies[base + 2u]),
    );
}

fn put_normal(base: u32, n: vec3<f32>) {
    slots[base + W_NX] = bitcast<u32>(n.x);
    slots[base + W_NX + 1u] = bitcast<u32>(n.y);
    slots[base + W_NX + 2u] = bitcast<u32>(n.z);
}

fn put_point(base: u32, k: u32, p: vec3<f32>, depth: f32, feature: u32) {
    let o = base + PT_BASE + k * PT_WORDS;
    slots[o] = bitcast<u32>(p.x);
    slots[o + 1u] = bitcast<u32>(p.y);
    slots[o + 2u] = bitcast<u32>(p.z);
    slots[o + 3u] = bitcast<u32>(depth);
    slots[o + 4u] = feature;
}

// 球×球：与 CPU 同式（`d = pb − pa`、`dist = √(d·d)`、`rr = ra + rb`、
// `n = d·(1/dist)`、`point = pa + n·(ra − (rr − dist)·0.5)`、`depth = rr − dist`）。
// 分支序也照抄：`dist ≥ rr` 先返（槽留 0 = 无流形），再判同心（`dist < 1e-9`）。
fn sphere_pair(base: u32, ba: u32, bb: u32) {
    let pa = body_pos(ba);
    let pb = body_pos(bb);
    let ra = as_f32(bodies[ba + P0_AT]);
    let rb = as_f32(bodies[bb + P0_AT]);
    let d = pb - pa;
    let dist = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    let rr = ra + rb;
    if (dist >= rr) {
        return;
    }
    if (dist < 1e-9) {
        // 同心球（CPU 同分支）：法线恒定 +Y、点 = a 心、深度 = rr。
        put_normal(base, vec3<f32>(0.0, 1.0, 0.0));
        put_point(base, 0u, pa, rr, 0u);
        slots[base + W_CNT] = 1u;
        return;
    }
    let inv = 1.0 / dist;
    let n = vec3<f32>(d.x * inv, d.y * inv, d.z * inv);
    let point = pa + n * (ra - (rr - dist) * 0.5);
    put_normal(base, n);
    put_point(base, 0u, point, rr - dist, 0u);
    slots[base + W_CNT] = 1u;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= prm.n_pairs) {
        return;
    }
    let base = i * SLOT_WORDS;
    // 先整槽清零 ⇒ 未写的部分**确定性**（自证"连跑两次逐字相同"不受未初始化内存影响）。
    for (var k = 0u; k < SLOT_WORDS; k = k + 1u) {
        slots[base + k] = 0u;
    }
    let a = pairs[i * 2u];
    let b = pairs[i * 2u + 1u];
    slots[base] = a;
    slots[base + 1u] = b;
    if (a >= prm.n_bodies || b >= prm.n_bodies) {
        slots[base + W_CNT] = NOT_HANDLED;
        return;
    }
    let ba = a * BODY_WORDS;
    let bb = b * BODY_WORDS;
    if (bodies[ba + KIND_AT] != KIND_SPHERE || bodies[bb + KIND_AT] != KIND_SPHERE) {
        slots[base + W_CNT] = NOT_HANDLED;
        return;
    }
    sphere_pair(base, ba, bb);
}
