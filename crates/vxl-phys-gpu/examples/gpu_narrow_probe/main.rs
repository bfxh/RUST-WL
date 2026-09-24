//! **窄相上卡的对拍**（`PLAN-gpu.md` §17.5–§17.7 第二片）：卡上固定槽 + 主机回填 vs CPU
//! `DefaultNarrowPhase`（**同一个 trait 入口 `NarrowPhase::collide`**，即真实路径）。
//!
//! **判据（§17.7 起为集合口径）**：逐对结构（对号/流形数）+ **`feature` 多重集逐位** + **点集双射**
//! （容差内一对一）+ 法线容差；点的**次序**与同位次 ulp 噪声**只报不判**——逐元素「次序 + 逐位」
//! 在本栈不可达（`select_contacts` 的排序/去重/截断是离散选择，ulp 差会翻它）。判据与诊断在
//! `diag.rs`（本文件只留编排与打印）。
//!
//! **不空转的保证**（第一片金丝雀的教训：判据要先于结论自查"有没有分辨力"）：打印并断言覆盖面
//! —— 卡上接手的对 / 主机回填的对 / 球族 / 盒族（含出流形）/ "接手但无接触" / 多流形对（复合体）/
//! 同心球 / 恰好接触 / 盒阈三连 都**必须跑到**。
//!
//! **金丝雀**（只动**卡上那一份**输入）：① 挪一颗球 1e-3 m（**配对不变、几何变**）② 挪 3 m
//! （几何大改 ⇒ 流形数变）——两条都必须让判据**变红**；**反照**（不动输入）必须仍通过。
//! **自证**：同一输入在卡上连跑两次 ⇒ 槽逐字相同（核内无原子 ⇒ 本该如此，留作回归闸）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_probe [--adapter K]`

use std::time::Instant;

mod diag;
use diag::*;

use vxl_phys::{
    CompoundChild, DefaultNarrowPhase, Manifold, NarrowPhase, PhysConfig, Quat, Shape, Vec3, World,
};
use vxl_phys_broad::{BroadPhase, GridBroadPhase};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::schedule::ScopedPool;
use vxl_phys_core::BodySet;
use vxl_phys_gpu::narrow::{NarrowTier, Slot, BODY_WORDS, KIND_BOX, KIND_SPHERE, NOT_HANDLED};
use vxl_phys_narrow::ContactPoints;

// ⚠️ 名字不能叫 `CELL`：本仓词汇门把独立单词 `cell` 列为禁词（只豁免 `std::cell`）。
const CELL_SIZE: f32 = 2.0;
/// `PhysConfig::default().contact_skin`（默认档 0.02）——宽相与窄相**用同一个 skin**（同规格）。
const SKIN: f32 = 0.02;
/// 判据容差（口径 B）：**先量后定** —— 取实测最大残差再留约一个数量级余量。
/// 实测（NVIDIA RTX 4060 Ti / Vulkan）：球族 `point` 4.8e-7 / `depth` 6.0e-8 / `normal` 1.2e-7。
/// ⚠️ 下面是**绝对地板**，不是全部：残差是 ulp 级 ⇒ 随坐标量级线性放大（远场簇在 y≈40 时一个
/// ulp ≈ 5e-6）⇒ 实际判据取 `max(地板, 2ε·尺度)`，见 `diag::tol_of`。
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

/// 盒族的 skin 阈三连。
struct BoxEdge {
    touch: (u32, u32),
    near: (u32, u32),
    sep: (u32, u32),
}

struct Scene {
    w: World,
    edge: Edge,
    box_edge: BoxEdge,
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

/// 一排动态盒：盒×盒（卡上 SAT+裁剪）与盒×地板（**仍是主机回填**）—— 两侧都覆盖到。
/// ⚠️ x/z 间距**故意不相等**（0.781 / 0.783）：相等间距会让对角邻居在 x/z 两根轴上**精确平局**
/// （同渗透量）⇒ SAT 选哪根轴由末位比特决定，判据就变成"比谁的舍入更巧"（实测踩过，见 §17.7）。
fn boxes_of(w: &mut World, n: usize) {
    for k in 0..n {
        let x = -3.0 + (k % 12) as f32 * 0.781;
        let z = 6.0 + (k / 12) as f32 * 0.783;
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

/// 倾斜盒簇（搬到地板格之外、悬在 y≈30）：**彼此深叠 + 姿态各异** ⇒ 同时压到面轴与 9 条棱轴，
/// 并且能裁出 **4 点**流形。对齐盒只走面轴 ⇒ 单靠它们覆盖不到棱轴与多点裁剪这两条路。
fn tilted_boxes_of(w: &mut World, n: usize) -> u32 {
    let first = w.bodies.len() as u32;
    let axis = Vec3::new(0.3, 1.0, 0.2).normalize();
    for k in 0..n {
        let rot = Quat::from_axis_angle(axis, 0.25 + k as f32 * 0.07);
        let x = (k % 2) as f32 * 0.95;
        let z = ((k / 2) % 2) as f32 * 0.95;
        let y = 30.0 + (k / 4) as f32 * 0.9;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, y, z),
            rot,
            1000.0,
        );
    }
    first
}

/// 盒对的 skin 阈三连（体在半空、**三对分开放**（z 隔开）⇒ 只与各自那一对成对），
/// 覆盖 `sep > skin` 与**负深度（投机接触）**两条边：`gap = 0`（恰好接触）/`0.01`（带内 ⇒ 预期接触）/
/// `0.03`（AABB 仍相交 ⇒ 在配对表里，但 `sep > skin` ⇒ **无流形**）。
fn box_skin_cases(w: &mut World) -> BoxEdge {
    let mk = |w: &mut World, y: f32, z: f32| {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(40.0, y, z),
            Quat::IDENTITY,
            1000.0,
        )
    };
    BoxEdge {
        touch: (mk(w, 40.0, 0.0), mk(w, 41.0, 0.0)),
        near: (mk(w, 40.0, 4.0), mk(w, 41.01, 4.0)),
        sep: (mk(w, 40.0, 8.0), mk(w, 41.03, 8.0)),
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
    tilted_boxes_of(&mut w, 24);
    let box_edge = box_skin_cases(&mut w);
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
        box_edge,
        compound,
        kids,
        // 金丝雀靶：球阵里第四排的那颗（**四周都有邻居** ⇒ 挪它必然改动若干条流形的几何）。
        ball: first_ball + 40,
    }
}

/// 逐体输入（12 字/体）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`。
/// 浮点走 f32 位模式；`kind` 是**裸整数**（核按裸 u32 读 ⇒ 别写成 f32 位模式）。
/// 参数：球用 `p0` = 半径、盒用 `p0..p2` = `half.xyz`。
fn pack_bodies(w: &World) -> Vec<u32> {
    let mut out = Vec::with_capacity(w.bodies.len() * BODY_WORDS);
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        let r = w.bodies.rot(i);
        out.extend_from_slice(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        out.extend_from_slice(&[r.x.to_bits(), r.y.to_bits(), r.z.to_bits(), r.w.to_bits()]);
        // 卡上已接的族 = 球×球、盒×盒；其余（圆柱/圆锥/外壳/胶囊/高度场/provider/复合体）交主机回填。
        let (kind, params) = match w.bodies.shape[i] {
            Shape::Sphere { radius } => (KIND_SPHERE, [radius, 0.0, 0.0]),
            Shape::Box { half } => (KIND_BOX, [half.x, half.y, half.z]),
            _ => (0, [0.0; 3]),
        };
        out.push(kind);
        for v in params {
            out.push(v.to_bits());
        }
        out.push(0); // pad
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
    gpu_box_box: u32,
    /// 盒×盒里出了流形的对数（**盒族自己的**"有接触"计数 ⇒ 单看总数会被球族掩盖）。
    gpu_box_hits: u32,
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
        let ba = &w.bodies.shape[a as usize];
        let bb = &w.bodies.shape[b as usize];
        if matches!(ba, Shape::Sphere { .. }) && matches!(bb, Shape::Sphere { .. }) {
            c.gpu_sphere_sphere += 1;
        }
        if matches!(ba, Shape::Box { .. }) && matches!(bb, Shape::Box { .. }) {
            c.gpu_box_box += 1;
            if cnt > 0 {
                c.gpu_box_hits += 1;
            }
        }
    }
    c
}

/// 报告段的输入（场景侧那些"边角对"的体号 + 槽表 + 诊断字）。
struct Slate<'a> {
    pairs: &'a [(u32, u32)],
    slots: &'a [Slot],
    edge: &'a Edge,
    box_edge: &'a BoxEdge,
    compound: u32,
    diag0: u32,
}

/// **报告段**：覆盖面 / 判据 / 软差 / 分族 / 不符分解（`main` 只留编排）。
#[allow(clippy::too_many_arguments)]
fn report(
    cpu: &[Manifold],
    asm: &[Manifold],
    c: &Cover,
    st: &Asm,
    pass: bool,
    d: &Delta,
    sl: &Slate,
    w: &World,
) {
    let idx = |p: (u32, u32)| sl.pairs.iter().position(|q| *q == p || (*q == (p.1, p.0)));
    let cnt_of = |p: (u32, u32)| idx(p).map(|i| sl.slots[i].count()).unwrap_or(u32::MAX);
    let dep_of = |p: (u32, u32)| {
        idx(p)
            .filter(|&i| sl.slots[i].count() > 0)
            .map(|i| sl.slots[i].point(0).1)
            .unwrap_or(f32::NAN)
    };
    let (edge, bx) = (sl.edge, sl.box_edge);
    let comp_man = asm
        .iter()
        .filter(|m| m.a == sl.compound || m.b == sl.compound)
        .count();
    print_row(
        &format!(
            "覆盖面：球×球对 {} | 盒×盒对 {}（出流形 {}）| 同心 count={} | 球恰好接触 count={} | 球擦边 count={} | 盒阈 触/带内/超带 count={}/{}/{}（带内 depth {:+.4}）| 复合体流形 {} | 回填段剩余 {} | 裁剪越界 {}",
            c.gpu_sphere_sphere,
            c.gpu_box_box,
            c.gpu_box_hits,
            cnt_of(edge.concentric),
            cnt_of(edge.touching),
            cnt_of(edge.near),
            cnt_of(bx.touch),
            cnt_of(bx.near),
            cnt_of(bx.sep),
            dep_of(bx.near),
            comp_man,
            st.leftover,
            sl.diag0
        ),
        sl.pairs.len(),
        c,
        st,
        cpu,
    );
    println!(
        "     判据（配对口径）：max|Δpoint| {:>9.3e} / Δdepth {:>9.3e} / Δnormal {:>9.3e} ⇒ {}",
        d.max_pt,
        d.max_depth,
        d.max_nrm,
        match (&d.mism, pass) {
            (Some(m), _) => format!("**不符 ✗**（{m}）"),
            (None, true) => "**通过 ✓**".to_string(),
            (None, false) => "**超容差 ✗**".to_string(),
        }
    );
    println!(
        "     逐位对齐口径（含次序，只报不判）：max|Δpoint| {:.3e}，{} 点里逐位相同 {}",
        d.pt_idx, d.n_pt, d.bit_pt
    );
    println!(
        "     软差（**都不计入判定**）：纯置换 {} 条（点多重集逐位相同、只是次序不同，首 {:?}）| ulp 噪声 {} 条（同位次有点差 ≤ 容差，首 {:?}）",
        d.k_perm, d.ord_at, d.k_ulp, d.ulp_at
    );
    if let Some(k) = d.ord_at.or(d.ulp_at) {
        dump_pair(k, &cpu[k], &asm[k], w);
    }
    // 不符时把**首处**双方数据打出来（定位差异的起点），并给不符的形状分解。
    if let Some(k) = d.mism_at {
        dump_pair(k, &cpu[k], &asm[k], w);
        println!(
            "         不符分解（条数）：对号 {} / 点数 {} / space {} / 特征多重集 {} / 法线超差 {} / 点集双射失败 {} | 涉及 {} 对",
            d.k_ab,
            d.k_len,
            d.k_space,
            d.k_feat,
            d.k_nrm,
            d.k_bij,
            d.mism_pairs.len()
        );
    }
}

/// 分族逐位率 + 判据的**自检**（覆盖不全会让"通过"变成空转 ⇒ 显式报出来）。
fn self_check(c: &Cover, st: &Asm, sl: &Slate, cpu: &[Manifold], asm: &[Manifold], w: &World) {
    let fam = per_family(cpu, asm, w);
    println!(
        "     分族逐位相同率（流形）：球×球 {}/{} | 盒×盒·不转 {}/{} | 盒×盒·有转 {}/{}",
        fam[0].0, fam[0].1, fam[1].0, fam[1].1, fam[2].0, fam[2].1
    );
    let idx = |p: (u32, u32)| sl.pairs.iter().position(|q| *q == p || (*q == (p.1, p.0)));
    let cnt_of = |p: (u32, u32)| idx(p).map(|i| sl.slots[i].count()).unwrap_or(u32::MAX);
    let near_d = idx(sl.box_edge.near)
        .filter(|&i| sl.slots[i].count() > 0)
        .map(|i| sl.slots[i].point(0).1)
        .unwrap_or(f32::NAN);
    // 盒阈三连里 `cnt_of` 给的是**槽里的点数**（面-面接触 ⇒ 4 点），别拿 1 去比。
    let vacuous = c.gpu_pairs == 0
        || c.gpu_hits == 0
        || c.gpu_empty == 0
        || c.gpu_sphere_sphere == 0
        || c.gpu_box_box == 0
        || c.gpu_box_hits == 0
        || sl.diag0 != 0
        || st.backfill_pairs == 0
        || st.backfill_manifolds == 0
        || st.multi_pairs == 0
        || st.leftover != 0
        || cnt_of(sl.edge.concentric) != 1
        || cnt_of(sl.edge.touching) != 0
        || cnt_of(sl.box_edge.touch) == 0
        || cnt_of(sl.box_edge.near) == 0
        || cnt_of(sl.box_edge.sep) != 0
        // 带内必须是**负深度**（投机接触），否则那条边没跑到；NaN 也算没跑到。
        || near_d.is_nan()
        || near_d >= 0.0;
    println!(
        "  判据分辨力自检：{}",
        if vacuous {
            "**覆盖不全 ⇒ 上面的结论不可信 ✗**"
        } else {
            "覆盖齐全（球/盒/棱轴/多点裁剪/回填/空/多流形/同心/盒阈三连都跑到了）✓"
        }
    );
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
    println!("== 窄相上卡 vs CPU `DefaultNarrowPhase`：结构 + 特征多重集逐位 + 点集双射容差 ==");
    let sc = scene();
    let w = &sc.w;
    let tol = tol_of(w);
    let scale = coord_scale(w);
    println!(
        "  判据容差（口径 B，按场景尺度 {scale:+.1} 放宽）：point {:.2e} / depth {:.2e} / normal {:.2e}",
        tol.pt, tol.depth, tol.nrm
    );
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
    let Ok(tier) = NarrowTier::new(adapter, w.bodies.len() as u32, pairs.len() as u32 + 8, SKIN)
    else {
        // ⚠️ 无适配器时**不能报成功**：本探针的判据只在卡上路径存在时才有意义 ⇒ 退出码 2 =
        // "未判定"（同 `gate_all.sh` 对拿不到独占锁时用 exit 8 的语义），别把它当成绿。
        println!("  ⚠️ 本机无可用适配器 ⇒ **本探针未判定**（不是通过；卡上路径要靠本机守）");
        std::process::exit(2);
    };
    println!("  适配器：{}", tier.adapter());
    let r1 = tier.run(&packed, &flat).expect("卡上窄相跑不通");
    let slots = r1.slots;
    let (asm, st) = assemble(&pairs, &slots, &mut np, &w.bodies, &jobs);
    let (pass, d) = compare(&cpu, &asm, &tol);
    let c = cover(&pairs, &slots, w);
    let slate = Slate {
        pairs: &pairs,
        slots: &slots,
        edge: &sc.edge,
        box_edge: &sc.box_edge,
        compound: sc.compound,
        diag0: r1.diag[0],
    };
    report(&cpu, &asm, &c, &st, pass, &d, &slate, w);
    // **判据的刀刃有多宽**（与卡上无关的对照）：CPU 自己吃到 1 ulp 位置扰动时翻多少。
    let (d_eps, both) = cpu_self_sensitivity(&pairs, &cpu, &mut np, &jobs, &d.mism_pairs, &tol);
    println!(
        "     CPU 自敏感（位置 **1 ulp**、配对不动）：CPU 自己翻 置换 {} 条 / ulp 噪声 {} 条 / 硬项不符 {} 对（与卡上交集 {}）",
        d_eps.k_perm,
        d_eps.k_ulp,
        d_eps.mism_pairs.len(),
        both
    );
    println!(
        "       ⚠️ 读法：**0 不等于「软差没事」**——它只否证了「1 ulp 输入扰动就能复现」这条路；\n         软差的机制证据在分族行（不转盒 93.5% 逐位 vs 有转盒 0% 逐位，残差 ≤0.25 ulp）⇒ 落在**旋转路径的收缩/结合序**上，属口径 B。"
    );
    // 自证：同输入连跑两次 ⇒ 槽逐字相同（核内无原子 ⇒ 本该如此；留作回归闸）。
    let again = tier.run(&packed, &flat).expect("卡上窄相跑不通");
    let same = slots
        .iter()
        .zip(again.slots.iter())
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
        &tier, &packed, &flat, &pairs, &mut np, &w.bodies, &jobs, &cpu, sc.ball, &tol,
    );
    timings(&tier, &packed, &flat, &pairs, &mut np, &w.bodies, &jobs);
    self_check(&c, &st, &slate, &cpu, &asm, w);
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
    ball: u32,
    tol: &Tol,
) {
    let mut d_of = |shift: f32| -> (bool, String) {
        let mut probe = packed.to_vec();
        let k = ball as usize * BODY_WORDS;
        probe[k] = (f32::from_bits(probe[k]) + shift).to_bits();
        let Ok(s2) = tier.run(&probe, flat) else {
            return (false, "卡上跑不通".to_string());
        };
        let (a2, _) = assemble(pairs, &s2.slots, np, bodies, jobs);
        let (p2, d2) = compare(cpu, &a2, tol);
        (
            p2,
            format!(
                "配对后 max|Δpoint| {:.3e} | 硬项不符：对号 {} 点数 {} 特征 {} 双射 {}（{} 对）",
                d2.max_pt,
                d2.k_ab,
                d2.k_len,
                d2.k_feat,
                d2.k_bij,
                d2.mism_pairs.len()
            ),
        )
    };
    for (name, shift) in [
        ("① 挪 1e-3 m（配对不变、几何变）", 1e-3f32),
        ("② 挪 3 m（几何大改）", 3.0),
    ] {
        let (pass2, info) = d_of(shift);
        println!(
            "     金丝雀{name} ⇒ {info} ⇒ {}",
            if pass2 {
                "**仍然通过 ✗（判据无分辨力！）**"
            } else {
                "如期变红 ✓"
            }
        );
    }
    // 反向对照：**不动输入** ⇒ 必须仍然通过（否则判据是"见谁都红"的摆设）。
    let (pass0, info0) = d_of(0.0);
    println!(
        "     反照（不动输入，应仍通过）⇒ {info0} ⇒ {}",
        if pass0 {
            "仍通过 ✓"
        } else {
            "**变红了 ✗**"
        }
    );
}

/// 窗口均值读数（**只报不判**：卡上档仍含主机回填 ⇒ 这里的数字是基线不是战果）。
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
        assert_eq!(s.slots.len(), pairs.len(), "回读槽数应等于对数");
    });
    // ⚠️ 计时变量的缩写别写成 `t` + 那三个字母：本仓 typos 门会判成 "the/this" 的拼写错。
    let (t_tier, tier_lo, tier_hi) = window_ms(|| {
        let s = tier.run(packed, flat).expect("卡上窄相跑不通");
        let (m, _) = assemble(pairs, &s.slots, np, bodies, jobs);
        assert_eq!(m.len(), sink.len(), "装配流形数应与 CPU 那份相同");
    });
    println!("     计时（窗口 {WINDOW} 次均值，{THREADS} 线程 CPU）:");
    println!("       CPU 窄相（全对，真实入口）      {t_cpu:>8.3} ms（{lo:.3}–{hi:.3}）");
    println!("       卡上档：上传+核+回读            {t_run:>8.3} ms（{rlo:.3}–{rhi:.3}）");
    println!(
        "       卡上档：上传+核+回读+回填+装配  {t_tier:>8.3} ms（{tier_lo:.3}–{tier_hi:.3}）"
    );
}
