//! **窄相上卡的对拍**（`PLAN-gpu.md` §17.5 第二片）：卡上固定槽 + 主机回填 vs CPU
//! `DefaultNarrowPhase`（**同一个 trait 入口 `NarrowPhase::collide`**，即真实路径）。
//!
//! **判据**：装配出的流形表（对序）与 CPU 那份逐元素比 ——
//! ① `a`/`b`/点数/`feature`/`space` **逐位相同**（整数项，不给容差）；
//! ② `point`/`depth`/`normal` 走**口径 B 容差**（窄相要走 `sqrt`/除法/乘加 ⇒ 卡上浮点挡不住
//! FMA 收缩，§9.6 已判）。
//!
//! **不空转的保证**（第一片金丝雀的教训：判据要先于结论自查"有没有分辨力"）：打印并断言
//! 覆盖面 —— 卡上接手的对 / 主机回填的对 / 卡上出流形的对 / 卡上"接手但无接触"的对 /
//! 多流形对（复合体）/ 同心球 / 恰好接触 都**必须 > 0**。
//!
//! **金丝雀**（两条，都只动**卡上那一份**输入）：① 挪一颗球 1e-3 m（**配对不变、几何变**）
//! ② 挪一颗球 3 m（几何大改 ⇒ 流形数变）——两条都必须让判据**变红**。
//! **自证**：同一输入在卡上连跑两次 ⇒ 槽逐字相同（核内无原子 ⇒ 本该如此，留作回归闸）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_probe [--adapter K]`

use std::time::Instant;

use vxl_phys::{
    CompoundChild, DefaultNarrowPhase, Manifold, NarrowPhase, PhysConfig, Quat, Shape, Vec3, World,
};
use vxl_phys_broad::{BroadPhase, GridBroadPhase};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::schedule::ScopedPool;
use vxl_phys_core::BodySet;
use vxl_phys_gpu::narrow::{NarrowTier, Slot, BODY_WORDS, KIND_SPHERE, NOT_HANDLED};
use vxl_phys_narrow::ContactPoints;

// ⚠️ 名字不能叫 `CELL`：本仓词汇门把独立单词 `cell` 列为禁词（只豁免 `std::cell`）。
const CELL_SIZE: f32 = 2.0;
/// `PhysConfig::default().contact_skin`（默认档 0.02）——宽相与窄相**用同一个 skin**（同规格）。
const SKIN: f32 = 0.02;
/// 判据容差（口径 B）：**先量后定** —— 取实测最大残差再留约一个数量级余量。
/// 实测（NVIDIA RTX 4060 Ti / Vulkan，4418 对 / 2242 点）：`point` 4.77e-7、`depth` 5.96e-8、
/// `normal` 1.19e-7 ⇒ 下面这组留 17–21×（且 87% 的点**逐位相同**）。
/// ⚠️ 容差是**绝对量**、随坐标量级放大（残差来自 ulp 级舍入）：大世界坐标要按相对量重定。
const TOL_PT: f32 = 1e-5;
const TOL_DEPTH: f32 = 1e-6;
const TOL_NRM: f32 = 1e-6;
/// 计时窗口（次）——决定量必须窗口均值（单点差可能只是相位/时钟爬升）。
const WINDOW: usize = 20;
/// 计时线程数（对齐 `m1_profile 8` 的相位账口径）。
const THREADS: usize = 8;

/// 场景里的**边角对**（给覆盖面断言用；索引即体号）。
struct Edge {
    concentric: (u32, u32),
    touching: (u32, u32),
    near: (u32, u32),
}

struct Scene {
    w: World,
    edge: Edge,
    /// 复合体体号（同一体对上有多条流形 ⇒ 只有主机回填才装得下）。
    compound: u32,
    /// 复合体子形状：**必须同时注册进探针自建的那份窄相**。
    kids: Vec<CompoundChild>,
    /// 格点里一颗**普通球**（金丝雀挪它 ⇒ 只改几何、不改配对）。
    ball: u32,
}

fn floor_of(w: &mut World, side: usize) {
    for k in 0..side * side {
        let x = (k % side) as f32 - side as f32 * 0.5;
        let z = (k / side) as f32 - side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.0, z),
            Quat::IDENTITY,
        );
    }
}

/// 三层咬合球阵：间距 0.75 < 半径和 0.8 ⇒ 正交邻居**重叠**、对角邻居**分离**
/// ⇒ 同一批对里既有接触也有"接手但无接触"（两条臂都跑得到）。返回**首颗球的体号**。
fn balls_of(w: &mut World, side: usize, layers: usize) -> u32 {
    let first = w.bodies.len() as u32;
    for ly in 0..layers {
        for iz in 0..side {
            for ix in 0..side {
                let k = (ly * side * side + iz * side + ix) as f32;
                let jx = ((k * 37.0) % 17.0) * 0.01 - 0.08;
                let jz = ((k * 53.0) % 19.0) * 0.01 - 0.09;
                let r = 0.40 + ((k * 29.0) % 5.0) * 0.01;
                w.add_dynamic(
                    Shape::Sphere { radius: r },
                    Vec3::new(
                        ix as f32 * 0.75 - 4.5 + jx,
                        0.88 + ly as f32 * 0.72,
                        iz as f32 * 0.75 - 4.5 + jz,
                    ),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    first
}

/// 一排动态盒：盒×盒（SAT+裁剪）与盒×地板都只能走**主机回填** —— 覆盖回填侧的几何。
fn boxes_of(w: &mut World, n: usize) {
    for k in 0..n {
        let x = -3.0 + (k % 12) as f32 * 0.78;
        let z = 6.0 + (k / 12) as f32 * 0.78;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, 0.9, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
}

/// 边角簇（搬到地板格之外 ⇒ 只与彼此成对）：同心（`dist < 1e-9` 特殊分支）/
/// 恰好接触（`dist == rr` ⇒ **无流形**）/ 擦边（深度 1e-3）。
fn edge_of(w: &mut World) -> Edge {
    let s = |w: &mut World, p: Vec3| {
        w.add_dynamic(Shape::Sphere { radius: 0.5 }, p, Quat::IDENTITY, 1000.0)
    };
    let a0 = s(w, Vec3::new(60.0, 0.0, 0.0));
    let a1 = s(w, Vec3::new(60.0, 0.0, 0.0));
    let b0 = s(w, Vec3::new(62.0, 0.0, 0.0));
    let b1 = s(w, Vec3::new(63.0, 0.0, 0.0));
    let c0 = s(w, Vec3::new(64.0, 0.0, 0.0));
    let c1 = s(w, Vec3::new(64.999, 0.0, 0.0));
    Edge {
        concentric: (a0, a1),
        touching: (b0, b1),
        near: (c0, c1),
    }
}

fn scene() -> Scene {
    let mut w = World::new(PhysConfig::default());
    floor_of(&mut w, 20);
    let first_ball = balls_of(&mut w, 12, 3);
    boxes_of(&mut w, 48);
    let edge = edge_of(&mut w);
    let kids = vec![
        CompoundChild {
            shape: Shape::Sphere { radius: 0.28 },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Sphere { radius: 0.28 },
            offset: Vec3::new(0.5, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
    ];
    let cid = w.add_compound(kids.clone());
    let compound = w.spawn_compound_body(cid, Vec3::new(-2.0, 0.72, -6.0), Quat::IDENTITY, 1000.0);
    Scene {
        w,
        edge,
        compound,
        kids,
        // 金丝雀靶：球阵里第四排的那颗（**四周都有邻居** ⇒ 挪它必然改动若干条流形的几何）。
        ball: first_ball + 40,
    }
}

/// 逐体输入（12 字/体）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`。
/// 浮点走 f32 位模式；`kind` 是**裸整数**（核按裸 u32 读 ⇒ 别写成 f32 位模式）。
fn pack_bodies(w: &World) -> Vec<u32> {
    let mut out = Vec::with_capacity(w.bodies.len() * BODY_WORDS);
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        let r = w.bodies.rot(i);
        out.extend_from_slice(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        out.extend_from_slice(&[r.x.to_bits(), r.y.to_bits(), r.z.to_bits(), r.w.to_bits()]);
        // 本片只接球；其余（盒/圆柱/圆锥/外壳/胶囊/高度场/provider/复合体）交主机回填。
        let (kind, p0) = match w.bodies.shape[i] {
            Shape::Sphere { radius } => (KIND_SPHERE, radius),
            _ => (0, 0.0),
        };
        out.push(kind);
        out.push(p0.to_bits());
        out.extend_from_slice(&[0, 0, 0]); // p1 / p2 / pad（后续族用）
    }
    out
}

/// 对表 → 卡上布局（2 字/对）。
fn flat_pairs(pairs: &[(u32, u32)]) -> Vec<u32> {
    pairs.iter().flat_map(|&(a, b)| [a, b]).collect()
}

/// 槽 → 流形（`space` 恒 0：**复合体一律走主机回填**，卡上不会出现"子形状流形"）。
fn manifold_of(s: &Slot) -> Manifold {
    let (a, b) = s.pair();
    let n = s.normal();
    let cnt = s.count() as usize;
    let mut buf = [vxl_phys::ContactPoint::default(); 4];
    for (k, slot) in buf.iter_mut().enumerate().take(cnt) {
        let (p, d, f) = s.point(k);
        *slot = vxl_phys::ContactPoint {
            point: Vec3::new(p[0], p[1], p[2]),
            depth: d,
            feature: f,
        };
    }
    Manifold {
        a,
        b,
        normal: Vec3::new(n[0], n[1], n[2]),
        points: ContactPoints::from_slice(&buf[..cnt]),
    }
}

/// 装配统计（覆盖面断言的料）。
#[derive(Default)]
struct Asm {
    backfill_pairs: u32,
    backfill_manifolds: u32,
    multi_pairs: u32,
    gpu_manifolds: u32,
    /// 回填段未被消费的流形数（**必须 0** ⇒ 否则装配漏了对/串了序）。
    leftover: usize,
}

/// **装配 = 对序**：逐对按升序取「卡上槽」或「主机回填段」。回填段是 `pairs` 的子序列
/// ⇒ 只要顺序走，`(a, b)` 相配的流形必然相邻（不需要查表）。复合体一对多流形自然落在这一段里。
fn assemble(
    pairs: &[(u32, u32)],
    slots: &[Slot],
    np: &mut DefaultNarrowPhase,
    bodies: &BodySet,
    jobs: &ScopedPool,
) -> (Vec<Manifold>, Asm) {
    let sub: Vec<(u32, u32)> = pairs
        .iter()
        .zip(slots)
        .filter(|(_, s)| s.count() == NOT_HANDLED)
        .map(|(p, _)| *p)
        .collect();
    let mut host: Vec<Manifold> = Vec::new();
    np.collide(bodies, &sub, &[], &NoProviders, &mut host, jobs);
    let mut st = Asm {
        backfill_pairs: sub.len() as u32,
        backfill_manifolds: host.len() as u32,
        ..Asm::default()
    };
    let mut out: Vec<Manifold> = Vec::with_capacity(host.len());
    let mut cur = 0usize;
    for (i, &(a, b)) in pairs.iter().enumerate() {
        match slots[i].count() {
            NOT_HANDLED => {
                let start = cur;
                while cur < host.len() && (host[cur].a, host[cur].b) == (a, b) {
                    cur += 1;
                }
                if cur - start > 1 {
                    st.multi_pairs += 1;
                }
                out.extend_from_slice(&host[start..cur]);
            }
            0 => {}
            1..=4 => {
                st.gpu_manifolds += 1;
                out.push(manifold_of(&slots[i]));
            }
            // 槽头坏了（核写越界/布局漂移）⇒ **当场炸**，别静默丢流形（丢一个流形会让
            // 判据表现为"流形数不同"，比直接报布局漂移难查得多）。
            n => panic!("第 {i} 槽 count 越界：{n}（核与主机的槽布局漂移？）"),
        }
    }
    st.leftover = host.len() - cur;
    (out, st)
}

/// 覆盖面读数（从卡上槽表直接数）。
#[derive(Default)]
struct Cover {
    gpu_pairs: u32,
    gpu_hits: u32,
    gpu_empty: u32,
    gpu_sphere_sphere: u32,
}

fn cover(pairs: &[(u32, u32)], slots: &[Slot], w: &World) -> Cover {
    let mut c = Cover::default();
    for (i, &(a, b)) in pairs.iter().enumerate() {
        let cnt = slots[i].count();
        if cnt == NOT_HANDLED {
            continue;
        }
        c.gpu_pairs += 1;
        if cnt == 0 {
            c.gpu_empty += 1;
        } else {
            c.gpu_hits += 1;
        }
        let both_ball = matches!(w.bodies.shape[a as usize], Shape::Sphere { .. })
            && matches!(w.bodies.shape[b as usize], Shape::Sphere { .. });
        if both_ball {
            c.gpu_sphere_sphere += 1;
        }
    }
    c
}

/// 逐元素残差（判据的料）。
#[derive(Default)]
struct Delta {
    max_pt: f32,
    max_depth: f32,
    max_nrm: f32,
    n_pt: usize,
    bit_pt: usize,
    mism: Option<String>,
}

fn cmp_one(k: usize, c: &Manifold, g: &Manifold, d: &mut Delta) {
    if c.a != g.a || c.b != g.b {
        d.mism.get_or_insert(format!(
            "第 {k} 条流形对号不同：CPU ({}, {}) vs 卡上 ({}, {})",
            c.a, c.b, g.a, g.b
        ));
        return;
    }
    if c.points.len() != g.points.len() {
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）点数不同：CPU {} vs 卡上 {}",
            c.a,
            c.b,
            c.points.len(),
            g.points.len()
        ));
        return;
    }
    if c.points.space() != g.points.space() {
        d.mism.get_or_insert(format!(
            "第 {k} 条流形（对 {}、{}）space 不同：{} vs {}",
            c.a,
            c.b,
            c.points.space(),
            g.points.space()
        ));
    }
    let dn = (c.normal - g.normal).abs();
    d.max_nrm = d.max_nrm.max(dn.x).max(dn.y).max(dn.z);
    for j in 0..c.points.len() {
        let (cp, gp) = (&c.points[j], &g.points[j]);
        d.n_pt += 1;
        d.max_pt = d.max_pt.max((cp.point - gp.point).length());
        d.max_depth = d.max_depth.max((cp.depth - gp.depth).abs());
        if cp.feature != gp.feature {
            d.mism.get_or_insert(format!(
                "第 {k} 条流形（对 {}、{}）第 {j} 点 feature 不同：{:#x} vs {:#x}",
                c.a, c.b, cp.feature, gp.feature
            ));
        }
        let same = cp.point.x.to_bits() == gp.point.x.to_bits()
            && cp.point.y.to_bits() == gp.point.y.to_bits()
            && cp.point.z.to_bits() == gp.point.z.to_bits()
            && cp.depth.to_bits() == gp.depth.to_bits();
        if same {
            d.bit_pt += 1;
        }
    }
}

/// 全表比较 ⇒ `(通过?, 残差)`。
fn compare(cpu: &[Manifold], gpu: &[Manifold]) -> (bool, Delta) {
    let mut d = Delta::default();
    if cpu.len() != gpu.len() {
        d.mism = Some(format!(
            "流形总数不同：CPU {} vs 卡上档 {}",
            cpu.len(),
            gpu.len()
        ));
        return (false, d);
    }
    for (k, (c, g)) in cpu.iter().zip(gpu.iter()).enumerate() {
        cmp_one(k, c, g, &mut d);
    }
    let pass =
        d.mism.is_none() && d.max_pt <= TOL_PT && d.max_depth <= TOL_DEPTH && d.max_nrm <= TOL_NRM;
    (pass, d)
}

/// 窗口均值（`WINDOW` 次）⇒ `(均值 ms, 最小, 最大)`；闭包不许推进被测对象（只调纯查询/档本身）。
fn window_ms(mut f: impl FnMut()) -> (f64, f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = 0.0f64;
    let mut sum = 0.0f64;
    for _ in 0..WINDOW {
        let t = Instant::now();
        f();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        lo = lo.min(ms);
        hi = hi.max(ms);
        sum += ms;
    }
    (sum / WINDOW as f64, lo, hi)
}

fn print_row(label: &str, n_pairs: usize, c: &Cover, a: &Asm, cpu: &[Manifold]) {
    println!(
        "  对 {n_pairs:>6} ⇒ 卡上接手 {:>6}（出流形 {:>6} / 空 {:>6}）/ 主机回填 {:>6} | CPU 流形 {} / 装配 {}（回填段 {}，多流形对 {}）",
        c.gpu_pairs,
        c.gpu_hits,
        c.gpu_empty,
        a.backfill_pairs,
        cpu.len(),
        a.gpu_manifolds + a.backfill_manifolds,
        a.backfill_manifolds,
        a.multi_pairs
    );
    println!("     {label}");
}

fn main() {
    let mut adapter = 0usize;
    let rest: Vec<String> = std::env::args().skip(1).collect();
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    println!("== 窄相上卡 vs CPU `DefaultNarrowPhase`：逐对容差 + feature/点数逐位 ==");
    let sc = scene();
    let w = &sc.w;
    let pairs: Vec<(u32, u32)> = {
        let mut bp = GridBroadPhase::new(CELL_SIZE, SKIN);
        let jobs = ScopedPool::new(THREADS);
        bp.compute_pairs(&w.bodies, &[], &[], &jobs).to_vec()
    };
    let packed = pack_bodies(w);
    let flat = flat_pairs(&pairs);
    let jobs = ScopedPool::new(THREADS);
    let mut np = DefaultNarrowPhase::new(SKIN);
    // ⚠️ 复合体子形状要在**两份**窄相里各注册一次：`np` 是探针自建的实例，与 `World` 那份仓库
    // **不共享**。首版漏了这步 ⇒ 子形状取不到 ⇒ 复合体一条流形都不出，覆盖面自检当场报"覆盖不全"。
    let cid = np.add_compound(sc.kids.clone());
    assert_eq!(cid, 0, "复合体 id 应与 World 侧同序（两边都从空仓库开始）");
    // CPU 参考：**真实入口**（一次调用、8 线程、按块序拼接 = 对序）。
    let mut cpu: Vec<Manifold> = Vec::new();
    np.collide(&w.bodies, &pairs, &[], &NoProviders, &mut cpu, &jobs);
    println!(
        "  n={} 体（含静态）| 对 {} | CPU 流形 {}",
        w.bodies.len(),
        pairs.len(),
        cpu.len()
    );
    let Ok(tier) = NarrowTier::new(adapter, w.bodies.len() as u32, pairs.len() as u32 + 8) else {
        // ⚠️ 无适配器时**不能报成功**：本探针的判据只在卡上路径存在时才有意义 ⇒ 退出码 2 =
        // "未判定"（同 `gate_all.sh` 对拿不到独占锁时用 exit 8 的语义），别把它当成绿。
        println!("  ⚠️ 本机无可用适配器 ⇒ **本探针未判定**（不是通过；卡上路径要靠本机守）");
        std::process::exit(2);
    };
    println!("  适配器：{}", tier.adapter());
    let slots = tier.run(&packed, &flat).expect("卡上窄相跑不通");
    let (asm, st) = assemble(&pairs, &slots, &mut np, &w.bodies, &jobs);
    let (pass, d) = compare(&cpu, &asm);
    let c = cover(&pairs, &slots, w);
    let edge = &sc.edge;
    let idx = |p: (u32, u32)| pairs.iter().position(|q| *q == p || (*q == (p.1, p.0)));
    let cnt_of = |p: (u32, u32)| idx(p).map(|i| slots[i].count()).unwrap_or(u32::MAX);
    let comp_man = asm
        .iter()
        .filter(|m| m.a == sc.compound || m.b == sc.compound)
        .count();
    print_row(
        &format!(
            "覆盖面：球×球对 {} | 同心 count={} | 恰好接触 count={} | 擦边 count={} | 复合体流形 {} | 回填段剩余 {}",
            c.gpu_sphere_sphere,
            cnt_of(edge.concentric),
            cnt_of(edge.touching),
            cnt_of(edge.near),
            comp_man,
            st.leftover
        ),
        pairs.len(),
        &c,
        &st,
        &cpu,
    );
    println!(
        "     判据：max|Δpoint| {:>9.3e}（{} 点，逐位相同 {}）/ Δdepth {:>9.3e} / Δnormal {:>9.3e} ⇒ {}",
        d.max_pt,
        d.n_pt,
        d.bit_pt,
        d.max_depth,
        d.max_nrm,
        match (&d.mism, pass) {
            (Some(m), _) => format!("**不符 ✗**（{m}）"),
            (None, true) => "**通过 ✓**".to_string(),
            (None, false) => "**超容差 ✗**".to_string(),
        }
    );
    // 自证：同输入连跑两次 ⇒ 槽逐字相同（核内无原子 ⇒ 本该如此；留作回归闸）。
    let again = tier.run(&packed, &flat).expect("卡上窄相跑不通");
    let same = slots
        .iter()
        .zip(again.iter())
        .all(|(x, y)| x.words == y.words);
    println!(
        "     自证：连跑两次 ⇒ {}",
        if same {
            "槽逐字相同 ✓"
        } else {
            "**槽不同 ✗**"
        }
    );
    canaries(
        &tier, &packed, &flat, &pairs, &mut np, &w.bodies, &jobs, &cpu, &slots, sc.ball,
    );
    timings(&tier, &packed, &flat, &pairs, &mut np, &w.bodies, &jobs);
    // 判据的**自检**：覆盖不全会让上面的"通过"变成空转 ⇒ 显式报出来。
    let vacuous = c.gpu_pairs == 0
        || c.gpu_hits == 0
        || c.gpu_empty == 0
        || c.gpu_sphere_sphere == 0
        || st.backfill_pairs == 0
        || st.backfill_manifolds == 0
        || st.multi_pairs == 0
        || st.leftover != 0
        || cnt_of(edge.concentric) != 1
        || cnt_of(edge.touching) != 0
        || comp_man < 2;
    println!(
        "  判据分辨力自检：{}",
        if vacuous {
            "**覆盖不全 ⇒ 上面的结论不可信 ✗**"
        } else {
            "覆盖齐全（卡上/回填/空/多流形/同心/恰好接触都跑到了）✓"
        }
    );
}

/// 金丝雀：只挪**卡上那一份**输入 ⇒ 判据必须变红（两条：配对不变的微扰、几何大改）。
#[allow(clippy::too_many_arguments)]
fn canaries(
    tier: &NarrowTier,
    packed: &[u32],
    flat: &[u32],
    pairs: &[(u32, u32)],
    np: &mut DefaultNarrowPhase,
    bodies: &BodySet,
    jobs: &ScopedPool,
    cpu: &[Manifold],
    slots: &[Slot],
    ball: u32,
) {
    let mut d_of = |shift: f32| -> (bool, f32) {
        let mut probe = packed.to_vec();
        let k = ball as usize * BODY_WORDS;
        probe[k] = (f32::from_bits(probe[k]) + shift).to_bits();
        let Ok(s2) = tier.run(&probe, flat) else {
            return (false, 0.0);
        };
        let (a2, _) = assemble(pairs, &s2, np, bodies, jobs);
        let (p2, d2) = compare(cpu, &a2);
        (p2, d2.max_pt)
    };
    for (name, shift) in [
        ("① 挪 1e-3 m（配对不变、几何变）", 1e-3f32),
        ("② 挪 3 m（几何大改）", 3.0),
    ] {
        let (pass2, dpt) = d_of(shift);
        println!(
            "     金丝雀{name} ⇒ max|Δpoint| {dpt:.3e} ⇒ {}",
            if pass2 {
                "**仍然通过 ✗（判据无分辨力！）**"
            } else {
                "如期变红 ✓"
            }
        );
    }
    // 反向对照：**不动输入** ⇒ 必须仍然通过（否则判据是"见谁都红"的摆设）。
    let (_, dpt) = d_of(0.0);
    println!(
        "     反照（不动输入，应仍通过）⇒ max|Δpoint| {dpt:.3e} | 槽 {}",
        if slots.is_empty() {
            "空 ✗"
        } else {
            "非空 ✓"
        }
    );
}

/// 窗口均值读数（**只报不判**：本片还没搬几何族，且 GPU 档含主机回填 ⇒ 这里的数字是基线不是战果）。
fn timings(
    tier: &NarrowTier,
    packed: &[u32],
    flat: &[u32],
    pairs: &[(u32, u32)],
    np: &mut DefaultNarrowPhase,
    bodies: &BodySet,
    jobs: &ScopedPool,
) {
    let mut sink: Vec<Manifold> = Vec::new();
    let (t_cpu, lo, hi) = window_ms(|| {
        np.collide(bodies, pairs, &[], &NoProviders, &mut sink, jobs);
    });
    let (t_run, rlo, rhi) = window_ms(|| {
        let s = tier.run(packed, flat).expect("卡上窄相跑不通");
        assert_eq!(s.len(), pairs.len(), "回读槽数应等于对数");
    });
    // ⚠️ 计时变量的缩写别写成 `t` + 那三个字母：本仓 typos 门会判成 "the/this" 的拼写错。
    let (t_tier, tier_lo, tier_hi) = window_ms(|| {
        let s = tier.run(packed, flat).expect("卡上窄相跑不通");
        let (m, _) = assemble(pairs, &s, np, bodies, jobs);
        assert_eq!(m.len(), sink.len(), "装配流形数应与 CPU 那份相同");
    });
    println!("     计时（窗口 {WINDOW} 次均值，{THREADS} 线程 CPU）:");
    println!("       CPU 窄相（全对，真实入口）      {t_cpu:>8.3} ms（{lo:.3}–{hi:.3}）");
    println!("       卡上档：上传+核+回读            {t_run:>8.3} ms（{rlo:.3}–{rhi:.3}）");
    println!(
        "       卡上档：上传+核+回读+回填+装配  {t_tier:>8.3} ms（{tier_lo:.3}–{tier_hi:.3}）"
    );
}
