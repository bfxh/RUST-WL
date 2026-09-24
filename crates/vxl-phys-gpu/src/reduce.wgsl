// 反作用聚合核（`PLAN-gpu.md` §13.2 的"剩半步"）：逐粒反作用力 → 每体 (力, 绕体原点的力矩)
//
// **为什么这么写**（判据见 §13.2）：
// ① 一个体 = 一个 workgroup，**段内升序串行累加** ⇒ 求和序与 CPU
//    `fluid_boundary.rs::aggregate_reactions` 的 `for k in start..end` 逐条对应；
//    不用原子加（原子浮点加**不定序**，比口径 B 更差 —— 判据②）。
// ② 力矩按 CPU `Vec3::cross` 的**展开同序**写（y·z′−z·y′, z·x′−x·z′, x·y′−y·x′），
//    不用 `cross()`/`dot()` 内建（WGSL 内建归约序未规定，密度核同款注意）。
// ③ 只读 `out` 的**前 3 个分量**（力；后 3 个是 XSPH，反作用用不到）。
//
// **绑定**：0 pos(只读) | 1 out(只读) | 2 spans(只读) | 3 react(读写)
// 段表条目 = 8 × 32 位 / 32 B：`origin.xyz | start | end | 3 个填充槽`。
// 段表长度由**缓冲字节数**决定（`arrayLength`）⇒ 不需要额外的 uniform。

struct Span {
    origin: vec3<f32>,
    start: u32,
    end: u32,
    _p0: u32,
    _p1: u32,
    _p2: u32,
};

@group(0) @binding(0) var<storage, read> pos: array<f32>;
@group(0) @binding(1) var<storage, read> outp: array<f32>;
@group(0) @binding(2) var<storage, read> spans: array<Span>;
@group(0) @binding(3) var<storage, read_write> react: array<f32>;

@compute @workgroup_size(64)
fn reduce(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x;
    if (b >= arrayLength(&spans)) {
        return;
    }
    let sp = spans[b];
    var f = vec3<f32>(0.0);
    var tau = vec3<f32>(0.0);
    // 段内**升序串行**（与 CPU 同求和序）。段长千级，单线程循环足够 —— 并行化会改求和序。
    for (var k = sp.start; k < sp.end; k = k + 1u) {
        let fk = vec3<f32>(outp[k * 6u], outp[k * 6u + 1u], outp[k * 6u + 2u]);
        f = f + fk;
        let d = vec3<f32>(pos[k * 3u], pos[k * 3u + 1u], pos[k * 3u + 2u]) - sp.origin;
        tau = tau + vec3<f32>(
            d.y * fk.z - d.z * fk.y,
            d.z * fk.x - d.x * fk.z,
            d.x * fk.y - d.y * fk.x,
        );
    }
    react[b * 6u] = f.x;
    react[b * 6u + 1u] = f.y;
    react[b * 6u + 2u] = f.z;
    react[b * 6u + 3u] = tau.x;
    react[b * 6u + 4u] = tau.y;
    react[b * 6u + 5u] = tau.z;
}
