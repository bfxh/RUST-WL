//! **窄相卡上档的接线验收**（`PLAN-gpu.md` §17.9–§17.10）：同一个世界跑两条链，**只差"谁跑窄相"**
//! —— A = 默认（`DefaultNarrowPhase` 在主机），B = 注册了卡上档（`World::set_narrow_tier`，
//! 卡上跑已接的族 + 主机回填 + 按对序装配）。
//!
//! **判据**（与流体档的 facade 探针同族）：
//! ① **t=1 探测器**：第一个 tick 的 `max|Δx|` 必须只有 ulp/几何量级（接线错会在 t=1 就大）；
//! ② 逐 tick 的 `max|Δx|` 走势：**线性增长才是接线错**，随机游走是口径 B 混沌（§13.5 的判读法）；
//! ③ **金丝雀**：把后端换成"槽表原样返回但**清空所有 count**"（= 卡上的流形没送到求解器）
//!    ⇒ t=1 必须明显变大（证明①有分辨力）。
//! 判据与取证在 `diag.rs`（本文件只留编排与打印）。
//!
//! **性能读数**（§17.10 补记）：每趟的**分项**（门面 `NarrowTierStats` + 卡上内部 `prof`）与
//! **每 tick 分布**（p50/p90/max，累计均值会被少数同步卡顿的 tick 拖走）；`--mult K` 换规模。
//!
//! ⚠️ **边界**：本档只接"检测趟"的窄相；**CCD 回扫（`world_ccd`）仍在主机**跑 CPU 窄相
//! （它有自己的对表与输出槽）——那是已知边界，不是遗漏。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_tier_probe [--adapter K] [--mult K]`

mod diag;

use diag::{axes_probe, selftest, t1_report};

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::narrow_tier::{NarrowSlots, NarrowTierBackend, SLOT_WORDS};
use vxl_phys_gpu::narrow::{prof_snapshot, NarrowTier};

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
/// m 档场景的规模（10k 静态盒 + 1k 动态盒，×`mult`）。
const CAP_BODIES_M1: u32 = 12_000;
const CAP_PAIRS_M1: u32 = 1 << 20;

pub(crate) fn cfg() -> PhysConfig {
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

/// m 档场景（**与 `m1_profile 8` 同族**：10k 静态盒 + 1k 动态盒 + 平地高度场）
/// —— 全盒族 ⇒ **回填为 0**，是"卡上档该赢"的那一档。
///
/// `mult` = **规模倍数**（`--mult K`）：静态/动态体数与 x 向铺展各 ×K，密度与 m1 同族。用途是找
/// **跨越点**：卡上档每趟的**固定**往返开销不随规模涨，CPU 的**每对**开销随规模线性涨 ⇒ 两者必有
/// 一处相交；m1 档正坐在交点附近，×2/×4 才看得出该谁赢。
pub(crate) fn scene_m1(mult: usize) -> World {
    use vxl_phys::HeightField;
    let m = mult.max(1);
    let cols = 100 * m;
    let fx = m as f32;
    let mut w = World::new(cfg());
    w.add_heightfield(HeightField::flat(
        -60.0 * fx,
        -60.0,
        (121 * m) as u32,
        121,
        1.0,
        0.0,
    ));
    for k in 0..10_000 * m {
        let x = (k % cols) as f32 - 50.0 * fx;
        let z = (k / cols) as f32 - 50.0;
        w.add_static(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..1_000 * m {
        let x = ((k * 37) % (97 * m)) as f32 / (97 * m) as f32 * (40.0 * fx) - 20.0 * fx;
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
pub(crate) fn scene() -> World {
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

/// 与核上的接手规则同款：两端都是球/盒 ⇒ 卡上接手；其余（地形/外壳/胶囊/圆柱/圆锥/复合体，
/// 以及不存在的体号）一律 `KIND_NONE` ⇒ 由主机回填。
fn needs_backfill(w: &World, p: (u32, u32)) -> bool {
    let ok = |i: u32| {
        matches!(
            w.bodies.shape.get(i as usize),
            Some(Shape::Sphere { .. }) | Some(Shape::Box { .. })
        )
    };
    !(ok(p.0) && ok(p.1))
}

/// `run_pair` 的读数：`(t=1 的 Δx, 最坏 Δx, 最坏 tick, 末态 Δx, 末态 Δv, 走势检查点, 窄相累计 µs)`。
type Reading = (f32, f32, usize, f32, f32, Vec<(usize, f32)>, [u64; 2]);

fn max_dx(a: &World, b: &World) -> f32 {
    (0..a.bodies.len())
        .map(|i| (a.bodies.position[i] - b.bodies.position[i]).length())
        .fold(0.0f32, f32::max)
}

fn max_dv(a: &World, b: &World) -> f32 {
    (0..a.bodies.len())
        .map(|i| (a.bodies.linvel[i] - b.bodies.linvel[i]).length())
        .fold(0.0f32, f32::max)
}

/// 分位数（p50/p90/max）。**必须看分布**：240 tick 的累计均值会被少数 GPU 同步卡顿的 tick 拖走
/// （单趟地板的 min/max 就相差近十倍）⇒ 只看均值分不清"真慢"与"抖"。
fn stat(v: &mut [f32]) -> (f32, f32, f32) {
    v.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let p = |q: f64| v[((v.len() as f64 - 1.0) * q) as usize];
    (p(0.5), p(0.9), *v.last().unwrap_or(&0.0))
}

/// 每 tick 的**横轴与分布**计账（把 `run_pair` 的循环体瘦到尺寸门以内）。
#[derive(Default)]
struct Meters {
    pair_sum: u64,
    bf_sum: u64,
    pair_max: usize,
    per_a: Vec<f32>,
    per_b: Vec<f32>,
    prev: (u64, u64),
}

impl Meters {
    /// 记**零点**（要在循环外、预热之后调：`narrowphase_us` 从 tick 0 就在累加）。
    fn start(&mut self, a: &World, b: &World) {
        self.prev = (a.timings().narrowphase_us, b.timings().narrowphase_us);
    }

    fn tic(&mut self, a: &World, b: &World) {
        let cur = (a.timings().narrowphase_us, b.timings().narrowphase_us);
        self.per_a.push((cur.0 - self.prev.0) as f32 / 1e3);
        self.per_b.push((cur.1 - self.prev.1) as f32 / 1e3);
        self.prev = cur;
        self.pair_sum += a.pairs().len() as u64;
        self.pair_max = self.pair_max.max(a.pairs().len());
        self.bf_sum += a.pairs().iter().filter(|p| needs_backfill(a, **p)).count() as u64;
    }

    fn report(&mut self, a: &World, us: [u64; 2]) {
        let mean_pairs = self.pair_sum as f64 / TICKS as f64;
        println!(
            "     对表（{TICKS} tick 均值）：**{mean_pairs:.0} 对/tick**（峰 **{}**），其中需回填 {:.1}；末 tick {} 对",
            self.pair_max,
            self.bf_sum as f64 / TICKS as f64,
            a.pairs().len()
        );
        println!(
            "     折算：默认 **{:.1} ns/对**（8 线程）| 卡上 **{:.1} ns/对**（含回填+装配+主机打包）",
            us[0] as f64 * 1e3 / TICKS as f64 / mean_pairs,
            us[1] as f64 * 1e3 / TICKS as f64 / mean_pairs
        );
        let (a50, a90, amax) = stat(&mut self.per_a);
        let (b50, b90, bmax) = stat(&mut self.per_b);
        println!(
            "     每 tick 窄相（ms）：默认 p50 {a50:.3} / p90 {a90:.3} / max {amax:.3} | 卡上 p50 **{b50:.3}** / p90 {b90:.3} / max **{bmax:.3}**"
        );
    }
}

/// 卡上那一趟的**内账**：常驻是否生效 / 门面分项 / 卡上看到的量 / 卡上内部 / 同数据直调。
fn report_card(b: &World, diag: Option<&NarrowTier>) {
    if !b.has_narrow_tier() {
        return;
    }
    println!(
        "     常驻体表：{}",
        if b.narrow_resident_active() {
            "**已生效 ✓**（体表留卡上，每趟只推变动记录）"
        } else {
            "**未生效 ✗**（每趟整表上传 ⇒ 这一档白搭）"
        }
    );
    let st = b.narrow_tier_stats();
    if st.calls == 0 {
        return;
    }
    let n = st.calls as f64;
    let ms = |u: u64| u as f64 / n / 1e3;
    println!(
        "     分项（每趟均值 ms，{} 趟）：打包 {:.3} | 比对 {:.3} | 上传 {:.3} | 后端(写+派发+回读+解码) {:.3} | 回填+装配 {:.3} ⇒ 合计 {:.3}",
        st.calls,
        ms(st.pack_us),
        ms(st.diff_us),
        ms(st.upload_us),
        ms(st.run_us),
        ms(st.assemble_us),
        ms(st.pack_us + st.diff_us + st.upload_us + st.run_us + st.assemble_us)
    );
    println!(
        "     卡上看到的量：对 累计 {} / 峰 {}（均值 {:.0}）| 变动记录 累计 {} / 峰 {} | 核诊断字峰 [{}, {}]（`[0]` 必须 0）",
        st.pairs_sum,
        st.pairs_max,
        st.pairs_sum as f64 / n,
        st.changed_sum,
        st.changed_max,
        st.diag0_max,
        st.diag1_max
    );
    // **卡上档内部**的分项（进程级累加 ⇒ 门面移进 `World` 的那个对象也数得到）：
    // 判"World 路径比同数据直调慢"到底慢在写 / 等 / 解码 / 摊平。
    let p = prof_snapshot();
    if p[7] > 0 {
        let m = p[7] as f64;
        println!(
            "     卡上内部（每趟均值 ms，{} 趟）：变动打包 {:.3} | 上传写 {:.3} | 参数+对表写 {:.3} | 编码 {:.3} | **提交+映射+等 {:.3}** | 解码 {:.3} | 摊平 {:.3}",
            p[7],
            p[0] as f64 / m / 1e3,
            p[1] as f64 / m / 1e3,
            p[2] as f64 / m / 1e3,
            p[3] as f64 / m / 1e3,
            p[4] as f64 / m / 1e3,
            p[5] as f64 / m / 1e3,
            p[6] as f64 / m / 1e3
        );
    }
    // **同数据直调**：把引擎末 tick 的那份对表直接喂给**另一个**卡上档（先整表同步）⇒ 直调快而
    // World 路径慢，差就在"数据/时机"，不在后端代码；反之差在 World 的调用方式。
    if let Some(t) = diag {
        let flat_b = vxl_phys_core::narrow_tier::flat_pairs(b.pairs());
        let pack_b = vxl_phys_core::narrow_tier::pack_bodies(&b.bodies);
        let all_b: Vec<u32> = (0..(pack_b.len() / 12) as u32).collect();
        if t.upload_records(&pack_b, &all_b).is_ok() {
            let mut ms_v = 0.0f64;
            for _ in 0..20 {
                let t0 = std::time::Instant::now();
                let _ = t.run_resident(&flat_b).expect("跑不通");
                ms_v += t0.elapsed().as_secs_f64() * 1e3 / 20.0;
            }
            println!(
                "     同数据直调（{} 对，20 次均值）：**{ms_v:.3} ms**",
                flat_b.len() / 2
            );
        }
    }
}

/// 跑两条链：先**同时空跑 `warm` 个 tick**（都不注册 ⇒ 必须逐位相同，顺带把场景推到"接触密集"的状态；
/// m 档动态体从高处落下，前 ~100 tick **没有接触** ⇒ 不预热的话 t=1 判据**空转**、金丝雀都红不了），
/// 再给 B 注册 `tier` 并逐 tick 比。
fn run_pair(
    tier: Option<Box<dyn NarrowTierBackend>>,
    label: &str,
    mult: usize,
    warm: usize,
    want_dump: bool,
    adapter: usize,
) -> Reading {
    // 诊断用的**第二份**卡上档（`run` 只要 `&self`；正题那份要被移进 World ⇒ 不能共用）。
    let diag = if want_dump {
        let (cb, cp) = if mult > 0 {
            (CAP_BODIES_M1 * mult as u32, CAP_PAIRS_M1 * mult as u32 / 4)
        } else {
            (CAP_BODIES, CAP_PAIRS)
        };
        NarrowTier::new(adapter, cb, cp, cfg().contact_skin).ok()
    } else {
        None
    };
    let mut a = if mult > 0 { scene_m1(mult) } else { scene() };
    let mut b = if mult > 0 { scene_m1(mult) } else { scene() };
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
        // 分项计时的**零点**：只统计这一条链（自证链也会在别处累加 ⇒ 先清）。
        vxl_phys_gpu::narrow::prof::reset();
    }
    // 切档**之前**的状态快照（诊断用：流形是在这一刻的位姿上算的，事后读体态已推进过一步）。
    let snap: (Vec<Vec3>, Vec<Quat>) = (
        (0..a.bodies.len()).map(|i| a.bodies.position[i]).collect(),
        (0..a.bodies.len()).map(|i| a.bodies.rot(i)).collect(),
    );
    let mut meters = Meters::default();
    meters.start(&a, &b);
    let (mut t1, mut t1_diag, mut geom_bad) = (0.0f32, String::new(), 0usize);
    let (mut worst, mut worst_at) = (0.0f32, 0usize);
    let mut cps: Vec<(usize, f32)> = Vec::new();
    for t in 1..=TICKS {
        a.step();
        b.step();
        meters.tic(&a, &b);
        let d = max_dx(&a, &b);
        if t == 1 {
            t1 = d;
            let (rdiag, gb) = t1_report(&a, &b);
            t1_diag = rdiag;
            geom_bad = gb;
            // ⚠️ **必须在 t=1 当场取证**：`a.manifolds()`/`a.pairs()` 是"本 tick"的，循环结束后再去读
            // 就是**最后一个 tick** 的（首版把取证放在循环外 ⇒ 拿"末 tick 的流形"对"首 tick 的体态"
            // ⇒ 一组**跨 tick 的错配**，把整条排查带偏）。
            if want_dump && !t1_diag.contains("几何不符 0 条") {
                axes_probe(&a, &b, &snap, 1, diag.as_ref(), mult);
            }
        }
        if d > worst {
            worst = d;
            worst_at = t;
        }
        if TICKS.is_multiple_of(t) {
            cps.push((t, d)); // 1/2/4/5/6/8/10/…/240：够看走势（线性 vs 随机游走）
        }
    }
    let (end, endv) = (max_dx(&a, &b), max_dv(&a, &b));
    println!(
        "  {label}：t=1 max|Δx| {t1:.3e} m | 最坏 {worst:.3e}（第 {worst_at} tick）| 末态 max|Δx| {end:.3e} / max|Δv| {endv:.3e}",
    );
    if !t1_diag.is_empty() {
        println!("     t=1 诊断：{t1_diag}");
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
    meters.report(&a, us);
    report_card(&b, diag.as_ref());
    (t1, worst, worst_at, end, endv, cps, us)
}

fn main() {
    let mut adapter = 0usize;
    let rest: Vec<String> = std::env::args().skip(1).collect();
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    let big = rest.iter().any(|x| x == "--m1");
    // `mult` = 规模倍数：`--m1` ⇒ 1，`--mult K` ⇒ K（找**跨越点**：卡的固定往返 vs CPU 的每对开销）。
    let mult = if let Some(k) = rest.iter().position(|x| x == "--mult") {
        rest.get(k + 1)
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(1)
    } else if big {
        1
    } else {
        0
    };
    println!("== 窄相卡上档接线 vs 默认 CPU 窄相：两条链只差「谁跑窄相」==");
    println!(
        "  {TICKS} tick | threads=8 | 场景 = {} | 判据 t=1 ≤ {T1_MAX:.0e} m",
        if mult > 0 {
            format!(
                "m{mult} 档（{} 静 + {} 动盒 + 平地 ⇒ 全盒族、回填 0）",
                10_000 * mult,
                1_000 * mult
            )
        } else {
            "小档（144 静 + 60 球 + 40 带旋转盒）".to_string()
        }
    );
    let (cap_b, cap_p) = if mult > 0 {
        (CAP_BODIES_M1 * mult as u32, CAP_PAIRS_M1 * mult as u32 / 4)
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
    let (t1_self, end_self, ..) = run_pair(None, "自证（都不注册）", mult, WARM, false, adapter);
    if t1_self != 0.0 || end_self != 0.0 {
        println!("     ⚠️ 自证不为零 ⇒ 比较器/场景有不确定性，下面的读数不可信");
    }
    // ② 正题：B 链注册卡上档。
    let (t1, _, _, _, _, cps, us) = run_pair(
        Some(Box::new(
            NarrowTier::new(adapter, cap_b, cap_p, skin).expect("建卡上档"),
        )),
        "卡上档（B 注册 / A 默认）",
        mult,
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
        mult,
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
