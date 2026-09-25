//! **窄相卡上档的接线验收**（`PLAN-gpu.md` §17.9）：同一个世界跑两条链，**只差"谁跑窄相"**
//! —— A = 默认（`DefaultNarrowPhase` 在主机），B = 注册了卡上档（`World::set_narrow_tier`，
//! 卡上跑已接的族 + 主机回填 + 按对序装配）。
//!
//! **判据**（与流体档的 facade 探针同族）：
//! ① **t=1 探测器**：第一个 tick 的 `max|Δx|` 必须只有 ulp/几何量级（接线错会在 t=1 就大）；
//! ② 逐 tick 的 `max|Δx|` 走势：**线性增长才是接线错**，随机游走是口径 B 混沌（§13.5 的判读法）；
//! ③ **金丝雀**：把后端换成"槽表原样返回但**清空所有 count**"（= 卡上的流形没送到求解器）
//!    ⇒ t=1 必须明显变大（证明①有分辨力）。
//!
//! ⚠️ **边界**：本档只接"检测趟"的窄相；**CCD 回扫（`world_ccd`）仍在主机**跑 CPU 窄相
//! （它有自己的对表与输出槽）——那是已知边界，不是遗漏。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_tier_probe [--adapter K]`

use vxl_phys::{NarrowPhase, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::narrow_tier::{NarrowSlots, NarrowTierBackend, SLOT_WORDS};
use vxl_phys_gpu::narrow::NarrowTier;

/// 切换后第一 tick 的**诊断**：① 流形表结构（条数 / 对号 / 点数 / 特征多重集）是否一致；
/// ② 位置差的**分布**（超阈体数 + 最大 δv）。
///
/// 读法：结构不一致或"很多体一起漂" ⇒ 接线/参数**真差**；只有个别体且 δv 被放大
/// ⇒ 接触在 `sep ≈ skin` 的刀刃上翻面（口径 B 的离散事件，同 §15 的投影激活）。
fn t1_report(a: &World, b: &World) -> (String, usize) {
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
    // 几何不符的**细分**：多少个流形、是法线变了（⇒ 选了另一根 SAT 轴，接触面都换了一张）
    // 还是只有点集/深度差（⇒ 同一张面上的裁剪差）。
    let mut n_geom_bad = 0usize;
    let mut n_norm_bad = 0usize;
    let mut max_dn = 0.0f32;
    for (x, y) in ma.iter().zip(mb.iter()) {
        let dn = (x.normal - y.normal).length();
        if dn > 1e-4 {
            n_norm_bad += 1;
        }
        max_dn = max_dn.max(dn);
        if !geom_ok(x, y) {
            n_geom_bad += 1;
        }
    }
    let n_geom = n_geom_bad;
    (
        format!(
            "流形 {} / {}（对号+点数 {}, 特征多重集 {}, **几何不符 {} 条**（其中法线变 {} 条、max|Δn| {:.2e}））| 动了 {} 体（>1e-7 m）| max|Δv| {:.3e} m/s",
            ma.len(),
            mb.len(),
            yn(struct_ok),
            yn(feat_ok),
            n_geom_bad,
            n_norm_bad,
            max_dn,
            n_moved,
            max_dv
        ),
        n_geom,
    )
}

fn yn(ok: bool) -> &'static str {
    if ok {
        "一致 ✓"
    } else {
        "**不一致 ✗**"
    }
}

/// 逐流形的**几何**是否一致（点集双射 + 法线，容差 1e-4 m —— m1 档坐标 ~50 m 的 ulp ≈ 6e-6，
/// 留 ~16× 余量）：与"特征多重集"分开报，用来区分 **"同一接触面换了命名"**（口径 B 刀刃，几何一致）
/// 与 **"真的算出不同接触"**（几何错，要查）。
fn geom_ok(x: &vxl_phys::Manifold, y: &vxl_phys::Manifold) -> bool {
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

/// 一个 tick 的判据阈值：t=1 的 `max|Δx|` 超过它就算接线错（坐标 ~5 m 的 ulp ≈ 5e-7；
/// 本档实测残差在 ulp 量级 ⇒ 这里留 ~20× 余量）。
const T1_MAX: f32 = 1e-5;
const TICKS: usize = 240;
/// **预热 tick 数**（切换档之前两条链一起空跑）：把场景推到接触密集的状态，t=1 判据才不空转。
/// m1 档的动态体从 y≈12–40 落下 ⇒ 240 tick（4 s）足够让绝大多数落地并进入接触。
const WARM: usize = 240;
/// 动体下落场景的规模（静态地板 12×12 + 60 球 + 40 盒）。
const CAP_BODIES: u32 = 512;
const CAP_PAIRS: u32 = 1 << 15;
/// m1 档场景的规模（10k 静态盒 + 1k 动态盒）。
const CAP_BODIES_M1: u32 = 12_000;
const CAP_PAIRS_M1: u32 = 1 << 20;

fn cfg() -> PhysConfig {
    PhysConfig {
        threads: 8,
        // ⚠️ **子步必须 = 1**：`step()` 按 `substeps` 逐个跑 `substep(dt, k == 0, reuse)`，而
        // `detect = first || !reuse_manifolds` ⇒ 非准静态时**每个子步都重跑窄相**，于是 tick 末留在
        // `manifolds` 里的是**最后一个子步**的表 —— 那一帧的输入已被上一子步的解算+积分推进过；
        // 直接比它 = 比**两个不同的帧**（口径 B 的差被这一步放大成"36 条几何不符"）。
        // 子步 = 1 ⇒ 窄相只在 tick 起点的位姿上跑一次 ⇒ 与"切档前快照"**同一帧**，判据才有定义。
        substeps: 1,
        ..PhysConfig::default()
    }
}

/// m1 档场景（**与 `m1_profile 8` 同族**：10k 静态盒 + 1k 动态盒 + 平地高度场）
/// —— 全盒族 ⇒ **回填为 0**，是"卡上档该赢"的那一档。
fn scene_m1() -> World {
    use vxl_phys::HeightField;
    let mut w = World::new(cfg());
    w.add_heightfield(HeightField::flat(-60.0, -60.0, 121, 121, 1.0, 0.0));
    for k in 0..10_000usize {
        let x = (k % 100) as f32 - 50.0;
        let z = (k / 100) as f32 - 50.0;
        w.add_static(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..1_000usize {
        let x = ((k * 37) % 97) as f32 / 97.0 * 40.0 - 20.0;
        let z = ((k * 53) % 89) as f32 / 89.0 * 40.0 - 20.0;
        let y = 12.0 + ((k * 29) % 71) as f32 / 71.0 * 28.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

/// 确定性场景：静态盒地板 + 一层掉落的球 + 一层掉落且**带旋转**的盒（三条卡上族都跑到）。
fn scene() -> World {
    let mut w = World::new(cfg());
    for k in 0..144 {
        let x = (k % 12) as f32 - 6.0;
        let z = (k / 12) as f32 - 6.0;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.0, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..60 {
        let x = ((k * 37) % 23) as f32 * 0.35 - 4.0;
        let z = ((k * 53) % 19) as f32 * 0.35 - 3.2;
        let y = 2.0 + ((k * 29) % 11) as f32 * 0.25;
        w.add_dynamic(
            Shape::Sphere { radius: 0.4 },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    let axis = Vec3::new(0.3, 1.0, 0.2).normalize();
    for k in 0..40 {
        let x = ((k * 41) % 17) as f32 * 0.4 - 3.2;
        let z = ((k * 59) % 13) as f32 * 0.4 - 2.4;
        let y = 3.0 + ((k * 31) % 9) as f32 * 0.3;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::from_axis_angle(axis, 0.1 * k as f32),
            1000.0,
        );
    }
    w
}

/// 金丝雀后端：真后端 + **清空所有 `count`**（= "卡上算出来的流形没送到求解器"）。
struct DropAll(NarrowTier);

impl NarrowTierBackend for DropAll {
    fn narrow_run(&self, bodies: &[u32], pairs: &[u32]) -> Result<NarrowSlots, String> {
        let mut s = self.0.narrow_run(bodies, pairs)?;
        let n = s.words.len() / SLOT_WORDS;
        for i in 0..n {
            s.words[i * SLOT_WORDS + 5] = 0;
        }
        Ok(s)
    }
}

/// 逐体 `max|Δx|`（两条链的体数必须相同；位姿表按体号对齐）。
fn max_dx(a: &World, b: &World) -> f32 {
    let mut m = 0.0f32;
    for i in 0..a.bodies.len().min(b.bodies.len()) {
        m = m.max((a.bodies.position[i] - b.bodies.position[i]).length());
    }
    m
}

fn max_dv(a: &World, b: &World) -> f32 {
    let mut m = 0.0f32;
    for i in 0..a.bodies.len().min(b.bodies.len()) {
        m = m.max((a.bodies.linvel[i] - b.bodies.linvel[i]).length());
    }
    m
}

/// `run_pair` 的读数：`(切换后 t=1 的 Δx, 最坏 Δx, 最坏 tick, 末态 Δx, 末态 Δv, 走势检查点,
/// 两边窄相累计 µs)`。抽别名是 clippy 的 `type_complexity` 逼的（与流体档 `FluidSlot` 同款）。
type Reading = (f32, f32, usize, f32, f32, Vec<(usize, f32)>, [u64; 2]);

/// 跑两条链：先**同时空跑 `warm` 个 tick**（都不注册 ⇒ 必须逐位相同，顺带把场景推到"接触密集"
/// 的状态），再给 B 注册 `tier` 并逐 tick 比。
///
/// 为什么要预热：m1 档的动态体从 y≈12–40 落下 ⇒ 前 ~100 tick **根本没有接触**，那时窄相不参与
/// ⇒ t=1 恒为 0 的判据是**空转**（首版就踩到：金丝雀都红不了）。预热到接触密集再切档，
/// t=1 才真的是"接线错探测器"（同 §15 ②h 的"先静置再比"手法）。
///
/// 返回 `(切换后第一 tick 的 Δx, 最坏值, 最坏 tick, 末态 Δx, 末态 Δv, 检查点, 两边窄相累计 µs)`。
fn run_pair(
    tier: Option<Box<dyn NarrowTierBackend>>,
    label: &str,
    big: bool,
    warm: usize,
    want_dump: bool,
    adapter: usize,
) -> Reading {
    // 诊断用的**第二份**卡上档（`run` 只要 `&self`；正题那份要被移进 World ⇒ 不能共用）。
    let diag = if want_dump {
        let (cb, cp) = if big {
            (CAP_BODIES_M1, CAP_PAIRS_M1)
        } else {
            (CAP_BODIES, CAP_PAIRS)
        };
        NarrowTier::new(adapter, cb, cp, cfg().contact_skin).ok()
    } else {
        None
    };
    let mut a = if big { scene_m1() } else { scene() };
    let mut b = if big { scene_m1() } else { scene() };
    assert_eq!(a.bodies.len(), b.bodies.len());
    for _ in 0..warm {
        a.step();
        b.step();
    }
    let warm_d = max_dx(&a, &b);
    if warm_d != 0.0 {
        println!("     ⚠️ 预热段（都不注册）出现差异 {warm_d:.3e} ⇒ 场景/比较器有不确定性");
    }
    if let Some(t) = tier {
        b.set_narrow_tier(t);
    }
    // 切档**之前**的状态快照（诊断用：流形是在这一刻的位姿上算的，事后读体态已推进过一步）。
    let snap: (Vec<Vec3>, Vec<Quat>) = (
        (0..a.bodies.len()).map(|i| a.bodies.position[i]).collect(),
        (0..a.bodies.len()).map(|i| a.bodies.rot(i)).collect(),
    );
    let mut t1 = 0.0f32;
    let mut t1_diag = String::new();
    let mut geom_bad = 0usize;
    let mut worst = 0.0f32;
    let mut worst_at = 0usize;
    let mut cps: Vec<(usize, f32)> = Vec::new();
    for t in 1..=TICKS {
        a.step();
        b.step();
        let d = max_dx(&a, &b);
        if t == 1 {
            t1 = d;
            let (diag, gb) = t1_report(&a, &b);
            t1_diag = diag;
            geom_bad = gb;
        }
        if d > worst {
            worst = d;
            worst_at = t;
        }
        if TICKS.is_multiple_of(t) {
            cps.push((t, d)); // 1/2/4/5/6/8/10/…/240：够看走势（线性 vs 随机游走）
        }
    }
    let end = max_dx(&a, &b);
    let endv = max_dv(&a, &b);
    println!(
        "  {label}：t=1 max|Δx| {t1:.3e} m | 最坏 {worst:.3e}（第 {worst_at} tick）| 末态 max|Δx| {end:.3e} / max|Δv| {endv:.3e}",
    );
    if !t1_diag.is_empty() {
        println!("     t=1 诊断：{t1_diag}");
        if want_dump && !t1_diag.contains("几何不符 0 条") {
            axes_probe(&a, &b, &snap, 1, diag.as_ref());
        }
    }
    println!(
        "     ⇒ {}",
        if t1 <= T1_MAX {
            format!("t=1 ≤ {T1_MAX:.0e} ⇒ **接线到位**（之后的漂移是口径 B 混沌，判读法同 §13.5）")
        } else {
            format!("**t=1 > {T1_MAX:.0e}**，其中几何不符 {geom_bad} 条：接线把流形送到了求解器（结构一致 ✓），但**卡上与 CPU 选了不同的接触**——小档应为 0 ⇒ 与规模/退化构型有关")
        }
    );
    let us = [a.timings().narrowphase_us, b.timings().narrowphase_us];
    (t1, worst, worst_at, end, endv, cps, us)
}

fn main() {
    let mut adapter = 0usize;
    let rest: Vec<String> = std::env::args().skip(1).collect();
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    let big = rest.iter().any(|x| x == "--m1");
    println!("== 窄相卡上档接线 vs 默认 CPU 窄相：两条链只差「谁跑窄相」==");
    println!(
        "  {TICKS} tick | threads=8 | 场景 = {} | 判据 t=1 ≤ {T1_MAX:.0e} m",
        if big {
            "m1 档（10k 静 + 1k 动盒 + 平地 ⇒ 全盒族、回填 0）"
        } else {
            "小档（144 静 + 60 球 + 40 带旋转盒）"
        }
    );
    let (cap_b, cap_p) = if big {
        (CAP_BODIES_M1, CAP_PAIRS_M1)
    } else {
        (CAP_BODIES, CAP_PAIRS)
    };
    let skin = cfg().contact_skin;
    let Ok(tier) = NarrowTier::new(adapter, cap_b, cap_p, skin) else {
        // 无适配器 ⇒ **未判定**（别当绿；同 `gpu_narrow_probe` 的约定）。
        println!("  ⚠️ 本机无可用适配器 ⇒ **本探针未判定**（不是通过）");
        std::process::exit(2);
    };
    println!("  适配器：{}", tier.adapter());
    // 0) **量具自检**：把复算表拿到「引擎」与「卡上」面前各对一次（不一致就先修表，别去动核）。
    if !selftest(Some(&tier)) {
        println!("  ⚠️ 三方不同轴 ⇒ 复算表或核有问题（上面已点名），下面的「轴」诊断只能当线索");
    }
    // ① 自证：两条链都**不注册** ⇒ 应当逐位相同（证明比较器本身不引入差）；同时把场景预热。
    let (t1_self, end_self, ..) = run_pair(None, "自证（都不注册）", big, WARM, false, adapter);
    if t1_self != 0.0 || end_self != 0.0 {
        println!("     ⚠️ 自证不为零 ⇒ 比较器/场景有不确定性，下面的读数不可信");
    }
    // ② 正题：B 链注册卡上档。
    let (t1, _, _, _, _, cps, us) = run_pair(
        Some(Box::new(
            NarrowTier::new(adapter, cap_b, cap_p, skin).expect("建卡上档"),
        )),
        "卡上档（B 注册 / A 默认）",
        big,
        WARM,
        true,
        adapter,
    );
    // 走势检查点：**线性增长才是接线错**（指数/饱和 = 口径 B 混沌，§13.5 的判读法）。
    let trend: Vec<String> = cps
        .iter()
        .filter(|(t, _)| matches!(t, 1 | 10 | 60 | 120 | 240))
        .map(|(t, d)| format!("t{t}={d:.2e}"))
        .collect();
    println!("     走势检查点：{}", trend.join(" | "));
    println!(
        "     窄相累计（{TICKS} tick）：默认 {:.3} ms（{:.3} ms/tick）| 卡上档（含回填+装配）{:.3} ms（{:.3} ms/tick）⇒ {}",
        us[0] as f64 / 1e3,
        us[0] as f64 / 1e3 / TICKS as f64,
        us[1] as f64 / 1e3,
        us[1] as f64 / 1e3 / TICKS as f64,
        if us[1] < us[0] {
            "**卡上档更快**"
        } else {
            "**这一档规模下卡上更慢**（往返固定开销主导）"
        }
    );
    // ③ 金丝雀：清空 count ⇒ 判据必须变红。
    let (t1c, ..) = run_pair(
        Some(Box::new(DropAll(
            NarrowTier::new(adapter, cap_b, cap_p, skin).expect("建卡上档"),
        ))),
        "金丝雀（清空 count）",
        big,
        WARM,
        false,
        adapter,
    );
    println!(
        "  裁决：{}\n    （t=1：正题 {t1:.3e} / 金丝雀 {t1c:.3e} ⇒ {}）",
        if t1 <= T1_MAX && t1c > T1_MAX {
            "**接线成立 ✓**（正题过、金丝雀如期红）"
        } else {
            "**未通过 ✗**（正题超阈 或 金丝雀没红 = 判据无分辨力）"
        },
        if t1c > T1_MAX {
            "金丝雀如期变红 ✓"
        } else {
            "**金丝雀没红 ✗**"
        }
    );
}

/// **量具自检**（先证明复算表可信，再用它判案）：几条**手算得清**的盒对，各跑一次
/// 「我的 21 轴复算表 #0」与「**引擎自己**的 `DefaultNarrowPhase::collide`（单对，同一条代码路径）」，
/// 比双方选中的轴。判据：全一致 ⇒ 复算表可用（那 m1 档的分歧就是真差）；有分歧 ⇒ **先修复算表**。
fn selftest(tier: Option<&NarrowTier>) -> bool {
    type Case<'a> = (&'a str, [f32; 3], f32, [f32; 3], f32, [f32; 4]);
    let cases: [Case; 4] = [
        (
            "轴对齐·Y 浅叠",
            [0.0, 0.0, 0.0],
            0.5,
            [0.0, 0.95, 0.0],
            0.4,
            [0.0, 0.0, 0.0, 1.0],
        ),
        (
            "深叠·多轴近等",
            [0.0, 0.0, 0.0],
            0.5,
            [0.3, 0.3, 0.3],
            0.4,
            [0.0, 0.0, 0.0, 1.0],
        ),
        (
            "绕 Y 小角",
            [0.0, 0.0, 0.0],
            0.5,
            [0.6, 0.97, 0.0],
            0.4,
            [0.0, -0.06619837, 0.0, 0.9978054],
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

/// **卡上**对**这一对**（给定帧）的答案 = `(法线, 点数)`：建一个 2 体世界 → 打包 → 卡上跑一趟。
/// 与 `engine_pair_normal` 配对，就是"同一对、同一帧：引擎 vs 卡上"的最小对照。
fn card_pair_normal(
    t: &NarrowTier,
    pa: Vec3,
    ra: Quat,
    ha: Vec3,
    pb: Vec3,
    rb: Quat,
    hb: Vec3,
) -> Option<(Vec3, u32)> {
    let mut w = World::new(cfg());
    w.add_static(Shape::Box { half: ha }, pa, ra);
    w.add_dynamic(Shape::Box { half: hb }, pb, rb, 1000.0);
    let packed = vxl_phys_core::narrow_tier::pack_bodies(&w.bodies);
    let flat = vxl_phys_core::narrow_tier::flat_pairs(&[(0, 1)]);
    let r = t.run(&packed, &flat).ok()?;
    let s = &r.slots[0];
    let n = s.normal();
    Some((Vec3::new(n[0], n[1], n[2]), s.count()))
}

/// **引擎自己**对**单对**盒的答案（法线）：建一个两体世界跑一次 `collide` ⇒ 与全量跑同一条代码路径。
/// 用来做"同一帧证明"：哪一帧的体态能复现全量跑的流形，就说明全量跑窄相看的是那一帧。
fn engine_pair_normal(pa: Vec3, ra: Quat, ha: Vec3, pb: Vec3, rb: Quat, hb: Vec3) -> Option<Vec3> {
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
        &vxl_phys_core::schedule::SerialJobSystem,
    );
    out.first().map(|m| m.normal)
}

/// 对**几何不符**的前几对，打**主机侧复算的 21 条分离轴**（按 sep 降序）——判据是：
/// 卡上选中的那条，是否就是这里的第一名。不是 ⇒ 卡上的轴集/择优**真差**（查核里棱轴的构建）；
/// 前两名只差 ~1 ulp ⇒ 刀刃（口径 B），判据该放宽。
fn axes_probe(
    a: &World,
    b: &World,
    snap: &(Vec<Vec3>, Vec<Quat>),
    max_n: usize,
    tier: Option<&NarrowTier>,
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
            "       ⌖ 轴表 对 ({}, {})：切档前 a.pos={:?} half={:?} | b.pos={:?} half={:?}",
            x.a,
            x.b,
            [pa.x, pa.y, pa.z],
            [ha.x, ha.y, ha.z],
            [pb.x, pb.y, pb.z],
            [hb.x, hb.y, hb.z]
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
        // **同一帧证明**：把这一对分别按「切档前快照」与「事后（tick 末）体态」各喂一次**引擎自己**的
        // 单对 `collide`，看哪一帧能复现全量跑里 CPU 那条流形 —— 这决定"两边比的是不是同一帧"。
        let (pa2, pb2) = (
            a.bodies.position[x.a as usize],
            a.bodies.position[x.b as usize],
        );
        let (ra2, rb2) = (a.bodies.rot(x.a as usize), a.bodies.rot(x.b as usize));
        println!(
            "          ⌗ 同一帧证明：全量 CPU n={:?} | 全量卡上 n={:?}",
            [x.normal.x, x.normal.y, x.normal.z],
            [y.normal.x, y.normal.y, y.normal.z]
        );
        for (tag, qa, qb, ppos) in [
            ("切档前快照", (ra, rb), (pa, pb), 0u8),
            ("事后(tick末)", (ra2, rb2), (pa2, pb2), 1),
        ] {
            let _ = ppos;
            match engine_pair_normal(qb.0, qa.0, ha, qb.1, qa.1, hb) {
                Some(n) => println!(
                    "             {tag}：引擎单对 n={:?}{}{}",
                    [n.x, n.y, n.z],
                    if n.dot(x.normal) > 0.999 {
                        " =全量CPU ✓"
                    } else {
                        ""
                    },
                    if n.dot(y.normal) > 0.999 {
                        " =全量卡上 ✓"
                    } else {
                        ""
                    }
                ),
                None => println!("             {tag}：引擎单对=无接触"),
            }
            // **卡上也是同一对**：隔离跑一次卡上档 ⇒ 若它在隔离下选 #0（与引擎一致）而全量跑选 #3，
            // 说明卡上的答案依赖"这一对在批次里的位置/邻居"，不是这一对本身的几何问题。
            if let Some(t) = tier {
                let mut w2 = World::new(cfg());
                w2.add_static(Shape::Box { half: ha }, qb.0, qa.0);
                w2.add_dynamic(Shape::Box { half: hb }, qb.1, qa.1, 1000.0);
                let packed = vxl_phys_core::narrow_tier::pack_bodies(&w2.bodies);
                let flat = vxl_phys_core::narrow_tier::flat_pairs(&[(0, 1)]);
                if let Ok(r) = t.run(&packed, &flat) {
                    let s = &r.slots[0];
                    let n = s.normal();
                    let nv = Vec3::new(n[0], n[1], n[2]);
                    let n_tab = table[0].1;
                    println!(
                        "             {tag}：**卡上隔离** n={:?}（count {}）| 与表#0 n·={:+.5}{}",
                        [nv.x, nv.y, nv.z],
                        s.count(),
                        nv.dot(n_tab),
                        if nv.dot(n_tab) > 0.999 {
                            " =表#0 ✓"
                        } else {
                            " **≠表#0 ✗**"
                        }
                    );
                }
            }
        }
        shown += 1;
        if shown >= max_n {
            break;
        }
    }
}

/// **主机侧复算**该对的 21 条分离轴（镜像 `sat.rs::build_axes` + 标量 sep 公式，全用 core 数学）。
/// 返回按 `sep` 降序的 `(sep, n_a→b, 轴类)`。**这只是诊断**：真值以 CPU 窄相自己算的流形为准。
fn axes_host(
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
