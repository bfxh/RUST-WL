//! 窄相卡上档验收的**判据与取证**（主文件 `main.rs` 只留编排与打印；本文件由 `mod diag;` 收进来
//! ⇒ 条目对父模块可见即可）。
//!
//! 内容三块：
//! - **判据**：`t1_report`（结构 / 特征多重集 / 几何 / 位姿差的读数）、`geom_ok`（点集双射 + 法线容差）。
//! - **量具自检**：`selftest`（复算表 vs 引擎单对 vs 卡上隔离，三方同轴才准拿表判案）。
//! - **轴取证**（只在判据变红时跑）：`axes_host`（主机侧 21 轴复算）、`axes_probe`（点名 / 前缀二分 /
//!   隔离 / 真索引 / 真对表复现）、`card_on_bodies` / `card_pair_normal` / `prefix_bisect`。
//!
//! **判据形态见 `PLAN-gpu.md` §17.7**：逐元素「次序 + 逐位」在本栈**不可达**（`select_contacts` 的
//! 排序/去重/截断是离散选择，ulp 差会翻它）⇒ 结构 + 特征多重集逐位 + 点集双射 + 法线容差；
//! 点次序与同位次 ulp 噪声**只报不判**。

use vxl_phys::{Manifold, NarrowPhase, Quat, Shape, Vec3, World};
use vxl_phys_gpu::narrow::NarrowTier;

use super::{cfg, scene, scene_m1};

/// 切换后第一 tick 的**诊断**：① 流形表结构（条数 / 对号 / 点数 / 特征多重集）是否一致；
/// ② 位置差的**分布**（超阈体数 + 最大 δv）；③ 几何不符的细分（法线变了几条、最近点距多大）。
///
/// 读法：结构不一致或"很多体一起漂" ⇒ 接线/参数**真差**；只有个别体且 δv 被放大
/// ⇒ 接触在 `sep ≈ skin` 的刀刃上翻面（口径 B 的离散事件，同 §15 的投影激活）。
pub(super) fn t1_report(a: &World, b: &World) -> (String, usize) {
    let (ma, mb) = (a.manifolds(), b.manifolds());
    let mut struct_ok = ma.len() == mb.len();
    let mut feat_ok = struct_ok;
    if struct_ok {
        for (x, y) in ma.iter().zip(mb.iter()) {
            if (x.a, x.b) != (y.a, y.b) || x.points.len() != y.points.len() {
                struct_ok = false;
                break;
            }
            let mut f1: Vec<u32> = x.points.iter().map(|p| p.feature).collect();
            let mut f2: Vec<u32> = y.points.iter().map(|p| p.feature).collect();
            f1.sort_unstable();
            f2.sort_unstable();
            if f1 != f2 {
                feat_ok = false;
            }
        }
    }
    let mut n_moved = 0usize;
    let mut max_dv = 0.0f32;
    for i in 0..a.bodies.len().min(b.bodies.len()) {
        if (a.bodies.position[i] - b.bodies.position[i]).length() > 1e-7 {
            n_moved += 1;
        }
        max_dv = max_dv.max((a.bodies.linvel[i] - b.bodies.linvel[i]).length());
    }
    // 几何不符的**细分**：多少个流形、法线变了几条（⇒ 换了 SAT 轴）、以及**最近点距**有多大
    // （⇒ 是"同一批点换了代表"（≈ 去重间距 min_sep 量级）还是"点集真不同"）。
    let mut n_geom_bad = 0usize;
    let mut n_norm_bad = 0usize;
    let mut max_dn = 0.0f32;
    let mut max_near = 0.0f32;
    for (x, y) in ma.iter().zip(mb.iter()) {
        let dn = (x.normal - y.normal).length();
        if dn > 1e-4 {
            n_norm_bad += 1;
        }
        max_dn = max_dn.max(dn);
        if !geom_ok(x, y) {
            n_geom_bad += 1;
            for p in x.points.iter() {
                let near = y
                    .points
                    .iter()
                    .map(|q| (p.point - q.point).length())
                    .fold(f32::MAX, f32::min);
                max_near = max_near.max(near);
            }
        }
    }
    (
        format!(
            "流形 {} / {}（对号+点数 {}, 特征多重集 {}, **几何不符 {} 条**（法线变 {}、max|Δn| {:.2e}、最近点距 max {:.2e} m））| 动了 {} 体（>1e-7 m）| max|Δv| {:.3e} m/s",
            ma.len(),
            mb.len(),
            yn(struct_ok),
            yn(feat_ok),
            n_geom_bad,
            n_norm_bad,
            max_dn,
            max_near,
            n_moved,
            max_dv
        ),
        n_geom_bad,
    )
}

pub(super) fn yn(ok: bool) -> &'static str {
    if ok {
        "一致 ✓"
    } else {
        "**不一致 ✗**"
    }
}

/// 逐流形的**几何**是否一致（点集双射 + 法线，容差 1e-4 m）—— 与"特征多重集"分开报，方能区分
/// "同一接触面换了命名"（刀刃）与"真的算出不同接触"。
pub(super) fn geom_ok(x: &Manifold, y: &Manifold) -> bool {
    const TOL: f32 = 1e-4;
    if x.points.len() != y.points.len() || (x.normal - y.normal).length() > TOL {
        return false;
    }
    let mut used = [false; 4];
    for i in 0..x.points.len() {
        let mut hit = false;
        for (j, u) in used.iter().enumerate().take(y.points.len()) {
            if *u {
                continue;
            }
            if (x.points[i].point - y.points[j].point).length() <= TOL
                && (x.points[i].depth - y.points[j].depth).abs() <= TOL
            {
                used[j] = true;
                hit = true;
                break;
            }
        }
        if !hit {
            return false;
        }
    }
    true
}

/// **量具自检**（先证明复算表可信，再用它判案）：手算得清的盒对，比对「复算表 #0」/「**引擎自己**的
/// 单对 `collide`」/「卡上隔离」三方选中的轴 ⇒ 全一致才拿复算表去判案，有分歧就**先修表**。
pub(super) fn selftest(tier: Option<&NarrowTier>) -> bool {
    type Case<'a> = (&'a str, [f32; 3], f32, [f32; 3], f32, [f32; 4]);
    let cases: [Case; 2] = [
        (
            "轴对齐·Y 浅叠",
            [0.0, 0.0, 0.0],
            0.5,
            [0.0, 0.95, 0.0],
            0.4,
            [0.0, 0.0, 0.0, 1.0],
        ),
        (
            "m1 复现对",
            [14.0, 0.5, -7.0],
            0.5,
            [13.672637, 1.395953, -6.085008],
            0.4,
            [-0.0013033069, -0.06619837, 0.0007164241, 0.9978054],
        ),
    ];
    let jobs = vxl_phys_core::schedule::SerialJobSystem;
    let mut all_ok = true;
    for (name, pa, ha, pb, hb, q) in cases {
        let (pa, pb) = (
            Vec3::new(pa[0], pa[1], pa[2]),
            Vec3::new(pb[0], pb[1], pb[2]),
        );
        let (ha, hb) = (Vec3::splat(ha), Vec3::splat(hb));
        let (ra, rb) = (Quat::IDENTITY, Quat::new(q[0], q[1], q[2], q[3]));
        let mut w = World::new(cfg());
        w.add_static(Shape::Box { half: ha }, pa, ra);
        w.add_dynamic(Shape::Box { half: hb }, pb, rb, 1000.0);
        let mut out = Vec::new();
        w.narrow.collide(
            &w.bodies,
            &[(0, 1)],
            &[],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &jobs,
        );
        let table = axes_host(pa, ra, ha, pb, rb, hb);
        let (sep0, n0, kind0) = &table[0];
        // ③ **卡上**也跑同一对（最小复现）：同一帧、同一个 2 体世界 ⇒ 卡上若与表 #0 不同轴，
        //    就是一条**十行可复现**的核差异（而不是 m1 那种"场景 + 240 tick"的长链路）。
        if let Some(t) = tier {
            match card_pair_normal(t, pa, ra, ha, pb, rb, hb) {
                Some((nv, cnt)) => {
                    let d_eng = out.first().map(|m| m.normal.dot(nv)).unwrap_or(f32::NAN);
                    let d_tab = nv.dot(*n0);
                    println!(
                        "        卡上：n={:?}（count {cnt}）| 与引擎 n·={d_eng:+.5} | 与表#0 [{kind0}] n·={d_tab:+.5} ⇒ {}",
                        [nv.x, nv.y, nv.z],
                        if cnt == 0 {
                            "（卡上=无接触；与引擎一致即「一致 ✓」，不一致要查）"
                        } else if d_tab > 0.999 && d_eng > 0.999 {
                            "三方同轴 ✓"
                        } else {
                            "**卡上与表/引擎不同轴 ✗（最小复现！）**"
                        }
                    );
                }
                None => println!("        卡上：跑不通"),
            }
        }
        match out.first() {
            Some(m) => {
                let d = m.normal.dot(*n0);
                let ok = d > 0.999;
                all_ok &= ok;
                println!(
                    "      {name}：引擎 n={:?} | 复算#0 [{kind0}] sep={sep0:+.6e} n={:?} n·={d:+.4} ⇒ {}",
                    [m.normal.x, m.normal.y, m.normal.z],
                    [n0.x, n0.y, n0.z],
                    if ok { "同轴 ✓" } else { "**不同轴 ✗**" }
                );
            }
            None => {
                let ok = *sep0 > cfg().contact_skin;
                all_ok &= ok;
                println!(
                    "      {name}：引擎=无接触 | 复算#0 sep={sep0:+.6e} ⇒ {}",
                    if ok {
                        "一致 ✓"
                    } else {
                        "**不一致 ✗**"
                    }
                );
            }
        }
    }
    println!(
        "     量具自检：{}",
        if all_ok {
            "复算表与引擎同轴 ✓（可用它判案）"
        } else {
            "**复算表与引擎不一致 ✗ ⇒ 先修复算表，别去动核**"
        }
    );
    all_ok
}

/// **卡上把这一对放在指定体号上**（`ia`/`ib`）：补占位体到该下标再放真身 ⇒ 实测与 `(0,1)` 隔离逐位
/// 相同（§17.9 补记七）。
#[allow(clippy::too_many_arguments)] // 一对盒 = 位置/姿态/半长 ×2 + 体号 ×2
pub(super) fn card_on_bodies(
    t: &NarrowTier,
    pa: Vec3,
    ra: Quat,
    ha: Vec3,
    pb: Vec3,
    rb: Quat,
    hb: Vec3,
    ia: u32,
    ib: u32,
) -> Option<(Vec3, u32)> {
    let mut w = World::new(cfg());
    let pad = |w: &mut World, n: u32| {
        for k in 0..n {
            w.add_static(
                Shape::Box {
                    half: Vec3::splat(0.01),
                },
                Vec3::new(0.0, -1000.0 - k as f32, 0.0),
                Quat::IDENTITY,
            );
        }
    };
    pad(&mut w, ia);
    w.add_static(Shape::Box { half: ha }, pa, ra);
    pad(&mut w, ib - ia - 1);
    w.add_dynamic(Shape::Box { half: hb }, pb, rb, 1000.0);
    assert_eq!(w.bodies.len() as u32, ib + 1, "占位体没补齐到指定体号");
    let packed = vxl_phys_core::narrow_tier::pack_bodies(&w.bodies);
    let flat = vxl_phys_core::narrow_tier::flat_pairs(&[(ia, ib)]);
    let r = t.run(&packed, &flat).ok()?;
    let s = &r.slots[0];
    let n = s.normal();
    Some((Vec3::new(n[0], n[1], n[2]), s.count()))
}

/// **卡上**对**这一对**（给定帧）的答案 = `(法线, 点数)`：建一个 2 体世界 → 打包 → 卡上跑一趟。
pub(super) fn card_pair_normal(
    t: &NarrowTier,
    pa: Vec3,
    ra: Quat,
    ha: Vec3,
    pb: Vec3,
    rb: Quat,
    hb: Vec3,
) -> Option<(Vec3, u32)> {
    card_on_bodies(t, pa, ra, ha, pb, rb, hb, 0, 1)
}

/// **前缀二分（最小形态）**：同一对 `(0,1)` 只变对表长度 m，每次读最后一个槽 ⇒ 答案应与列表长度无关。
pub(super) fn prefix_bisect(
    t: &NarrowTier,
    pa: Vec3,
    ra: Quat,
    ha: Vec3,
    pb: Vec3,
    rb: Quat,
    hb: Vec3,
) {
    let want = axes_host(pa, ra, ha, pb, rb, hb)[0].1; // 正确方向 = 复算表 #0
    let mut w = World::new(cfg());
    w.add_static(Shape::Box { half: ha }, pa, ra);
    w.add_dynamic(Shape::Box { half: hb }, pb, rb, 1000.0);
    let packed = vxl_phys_core::narrow_tier::pack_bodies(&w.bodies);
    let mut s = String::new();
    for m in [1usize, 2, 4, 16, 64, 256, 1024, 4096] {
        let flat: Vec<u32> = (0..m).flat_map(|_| [0u32, 1u32]).collect();
        match t.run(&packed, &flat) {
            Ok(r) => {
                let n = r.slots[m - 1].normal();
                let nv = Vec3::new(n[0], n[1], n[2]);
                s.push_str(&format!("m={m}:{:+.4} ", nv.dot(want)));
            }
            Err(_) => s.push_str(&format!("m={m}:跑不通 ")),
        }
    }
    println!("          ⑃ 前缀二分（槽位随 m 变，n·#0 越接近 +1 越对）：{s}");
}

/// 对**几何不符**的前几对做**轴取证**：点名（双方选中的轴排第几 + sep）、前缀二分、隔离对照。
pub(super) fn axes_probe(
    a: &World,
    b: &World,
    snap: &(Vec<Vec3>, Vec<Quat>),
    max_n: usize,
    tier: Option<&NarrowTier>,
    mult: usize,
) {
    let (ma, mb) = (a.manifolds(), b.manifolds());
    let mut shown = 0usize;
    for (x, y) in ma.iter().zip(mb.iter()) {
        if geom_ok(x, y) {
            continue;
        }
        let (ha, hb) = match (a.bodies.shape[x.a as usize], a.bodies.shape[x.b as usize]) {
            (Shape::Box { half: h1 }, Shape::Box { half: h2 }) => (h1, h2),
            _ => (Vec3::ZERO, Vec3::ZERO),
        };
        let (pa, pb) = (snap.0[x.a as usize], snap.0[x.b as usize]);
        let (ra, rb) = (snap.1[x.a as usize], snap.1[x.b as usize]);
        println!(
            "       ⌖ 对 ({}, {})：a={:?} | b={:?}",
            x.a,
            x.b,
            (pa, ra, ha),
            (pb, rb, hb)
        );
        let table = axes_host(pa, ra, ha, pb, rb, hb);
        // **点名**：双方各自选中的那条在表里排第几、它的 sep 多少（近平行轴时"top-5 里找"会一起命中）。
        let rank_of = |dir: Vec3| -> String {
            let mut best = (0usize, f32::NAN, -2.0f32, String::new());
            for (k, (sep, n, kind)) in table.iter().enumerate() {
                let d = n.dot(dir);
                if d > best.2 {
                    best = (k, *sep, d, kind.clone());
                }
            }
            format!(
                "轴#{}[{}] sep={:+.6e} n·dir={:+.5}",
                best.0, best.3, best.1, best.2
            )
        };
        let gap = table.get(1).map(|v| v.0 - table[0].0).unwrap_or(f32::NAN);
        println!(
            "          ▸ 点名：CPU → {} | 卡上 → {} | 前两名 sep 差 {:.3e}",
            rank_of(x.normal),
            rank_of(y.normal),
            gap
        );
        if let Some(t) = tier {
            prefix_bisect(t, pa, ra, ha, pb, rb, hb);
        }
        // **隔离 / 真索引对照**：同一份记录喂两次卡上 —— ① 放在 (0,1)（两体世界）；
        // ② 放在**真实体号**（补齐到该下标的占位体表）。两次答案不同 ⇒ 问题在"核按体号取记录"
        // （索引/表规模），与几何无关；相同 ⇒ 索引无关，剩下的只有"真实对表的其它内容/其它体"。
        if let Some(t) = tier {
            let one = |ia: u32, ib: u32| -> String {
                match card_on_bodies(t, pa, ra, ha, pb, rb, hb, ia, ib) {
                    Some((n, c)) => format!(
                        "n={:?}(c{c}) n·#0={:+.5}",
                        [n.x, n.y, n.z],
                        n.dot(table[0].1)
                    ),
                    None => "跑不通".into(),
                }
            };
            println!("          ⌗ 隔离(0,1)：{}", one(0, 1));
            println!("          ⌗ 真索引({}, {})：{}", x.a, x.b, one(x.a, x.b));
            // ③ **真实对表复现**：把世界重建到**快照体态** + 用**本 tick 的真对表**喂卡上 = 复刻那一趟的调用。
            // 它若复现全量跑的答案 ⇒ 差异就在"对的列表/其它体"；若给 #0 ⇒ 输入还有别的不同。
            let mut w3 = if mult > 0 { scene_m1(mult) } else { scene() };
            for (i, (p, r)) in snap.0.iter().zip(snap.1.iter()).enumerate() {
                w3.bodies.position[i] = *p;
                w3.bodies.set_rot(i, *r);
            }
            let pr = a.pairs();
            let ti = pr.iter().position(|p| *p == (x.a, x.b)).unwrap_or(0);
            let packed3 = vxl_phys_core::narrow_tier::pack_bodies(&w3.bodies);
            let flat3 = vxl_phys_core::narrow_tier::flat_pairs(pr);
            match t.run(&packed3, &flat3) {
                Ok(r) => {
                    let s = &r.slots[ti];
                    let n = s.normal();
                    let nv = Vec3::new(n[0], n[1], n[2]);
                    println!(
                        "          ⌗ 真对表复现（{} 对，目标第 {ti} 位）：n={:?}(c{}) n·#0={:+.5}{}",
                        pr.len(),
                        [nv.x, nv.y, nv.z],
                        s.count(),
                        nv.dot(table[0].1),
                        if nv.dot(y.normal) > 0.9999 {
                            " 复现✓"
                        } else {
                            ""
                        }
                    );
                }
                Err(e) => println!("          ⌗ 真对表复现：跑不通（{e}）"),
            }
        }
        shown += 1;
        if shown >= max_n {
            break;
        }
    }
}

/// **主机侧复算**该对的 21 条分离轴（镜像 `sat.rs::build_axes` + 标量 sep），返回按 `sep` 降序的
/// `(sep, n, 轴类)`。
pub(super) fn axes_host(
    pa: Vec3,
    ra: Quat,
    ha: Vec3,
    pb: Vec3,
    rb: Quat,
    hb: Vec3,
) -> Vec<(f32, Vec3, String)> {
    let aa = [
        ra.rotate_vec3(Vec3::X),
        ra.rotate_vec3(Vec3::Y),
        ra.rotate_vec3(Vec3::Z),
    ];
    let ab = [
        rb.rotate_vec3(Vec3::X),
        rb.rotate_vec3(Vec3::Y),
        rb.rotate_vec3(Vec3::Z),
    ];
    // 面轴序照抄 BOX_FACES：+X,−X,+Y,−Y,+Z,−Z（体轴带符号）。
    let faces = [
        (0usize, 1.0f32),
        (0, -1.0),
        (1, 1.0),
        (1, -1.0),
        (2, 1.0),
        (2, -1.0),
    ];
    let mut axes: Vec<(Vec3, String)> = Vec::new();
    for (f, (ax, sg)) in faces.iter().enumerate() {
        axes.push((aa[*ax] * *sg, format!("A面{f}")));
    }
    for (f, (ax, sg)) in faces.iter().enumerate() {
        axes.push((ab[*ax] * *sg, format!("B面{f}")));
    }
    let (ea, eb) = ([aa[1], aa[2], aa[0]], [ab[1], ab[2], ab[0]]);
    for (i, x) in ea.iter().enumerate() {
        for (j, y) in eb.iter().enumerate() {
            let c = x.cross(*y);
            let l2 = c.length_squared();
            if l2 > 1e-8 {
                axes.push((c * (1.0 / l2.sqrt()), format!("棱{i}{j}")));
            }
        }
    }
    let mut out: Vec<(f32, Vec3, String)> = axes
        .into_iter()
        .filter(|(n, _)| n.length_squared() >= 0.5)
        .map(|(n0, kind)| {
            let ra_ = ha.x * aa[0].dot(n0).abs()
                + ha.y * aa[1].dot(n0).abs()
                + ha.z * aa[2].dot(n0).abs();
            let rb_ = hb.x * ab[0].dot(n0).abs()
                + hb.y * ab[1].dot(n0).abs()
                + hb.z * ab[2].dot(n0).abs();
            let ca = pa.dot(n0);
            let cb = pb.dot(n0);
            let sep1 = (cb - rb_) - (ca + ra_);
            let sep2 = (ca - ra_) - (cb + rb_);
            let (sep, n) = if sep1 >= sep2 {
                (sep1, n0)
            } else {
                (sep2, -n0)
            };
            (sep, n, kind)
        })
        .collect();
    out.sort_by(|x, y| y.0.total_cmp(&x.0));
    out
}
