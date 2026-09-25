//! **常驻体表 + 卡上档内外分项**（`PLAN-gpu.md` §17.10）：同一份 m 档场景、同一份对表，逐项定量 ——
//! ① **整表上传**（`NarrowTier::run`）② **常驻 + 变动记录**（`upload_records(delta)` + `run_resident`）
//! ③ 只跑（体表已常驻）④ 主机侧打包/比对 ⑧ 往返地板 ⑨⑩ 回读解码 ⑫ 空转对往返的影响。
//!
//! **口径**：窗口均值、独占（perf-lock）、**同输入**（同一帧的体表与对表）、**同进程交替**；
//! 两条路的**槽表逐位对照**（自证：换上传方式不该改结果）。`--mult K` 换规模、`--capP K` 换对数容量、
//! `--gap MS` 换 ⑫ 的空转。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_up_probe [--adapter K] [--mult K]`

use std::time::Instant;

use vxl_phys::{BroadPhase, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::narrow_tier::{flat_pairs, pack_bodies, BODY_WORDS, SLOT_BYTES, SLOT_WORDS};
use vxl_phys_gpu::narrow::{prof, prof_snapshot, NarrowTier, Slot};

const WARM: usize = 240;
const WINDOW: usize = 30;
const CAP_BODIES: u32 = 12_000;
const CAP_PAIRS: u32 = 1 << 20;

fn cfg() -> PhysConfig {
    PhysConfig {
        threads: 8,
        substeps: 1,
        ..PhysConfig::default()
    }
}

/// 与 `gpu_narrow_tier_probe` 的 m1 档同源（10k 静态盒 + 1k 动态盒 + 平地）；`mult` = 规模倍数
/// （静态/动态体数与 x 向铺展各 ×K）⇒ 用来量"卡段里的哪一项随**规模**涨"。
fn scene_m1(mult: usize) -> World {
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
        let z = ((k * 53) % 89) as f32 / 89.0 * 40.0 - 40.0;
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

/// 窗口均值（`WINDOW` 次）⇒ `(均值, 最小, 最大)`（ms）。闭包不许推进被测对象。
fn window_ms(mut f: impl FnMut()) -> (f64, f64, f64) {
    let (mut lo, mut hi, mut sum) = (f64::INFINITY, 0.0f64, 0.0f64);
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

/// 同 `window_ms`，但**只计闭包本身**、每次调用之间空转 `gap_ms`（空转不计入读数）：
/// 用来量"GPU 在两次提交之间歇了多久"对**同步往返**的影响。
fn window_ms_gap(mut f: impl FnMut(), gap_ms: u64) -> (f64, f64, f64) {
    let (mut lo, mut hi, mut sum) = (f64::INFINITY, 0.0f64, 0.0f64);
    for _ in 0..WINDOW {
        let t = Instant::now();
        f();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        lo = lo.min(ms);
        hi = hi.max(ms);
        sum += ms;
        std::thread::sleep(std::time::Duration::from_millis(gap_ms));
    }
    (sum / WINDOW as f64, lo, hi)
}

/// 命令行参数。
struct Args {
    adapter: usize,
    mult: usize,
    gap_ms: u64,
    cap_p: Option<u32>,
}

fn parse_args() -> Args {
    let rest: Vec<String> = std::env::args().skip(1).collect();
    let get = |k: &str| -> Option<String> {
        rest.iter()
            .position(|x| x == k)
            .and_then(|i| rest.get(i + 1))
            .cloned()
    };
    Args {
        adapter: get("--adapter").and_then(|v| v.parse().ok()).unwrap_or(0),
        // `--mult K` = 规模倍数（m1 档 ×K）。
        mult: get("--mult").and_then(|v| v.parse().ok()).unwrap_or(1),
        // `--gap MS` = ⑫ 里两次调用之间的空转（默认 5 ms ≈ 大档每 tick 的 CPU 时长）。
        gap_ms: get("--gap").and_then(|v| v.parse().ok()).unwrap_or(5),
        // `--capP K` = 强制对数容量（只影响缓冲大小的对照：判"每趟代价是否随**容量**涨"）。
        cap_p: get("--capP").and_then(|v| v.parse().ok()),
    }
}

/// 一次探针的上下文：把 `main` 拆成几段而**不堆参数**。
struct Ctx<'a> {
    tier: &'a NarrowTier,
    bodies: &'a vxl_phys_core::BodySet,
    packed: &'a [u32],
    flat: &'a [u32],
    pairs: &'a [(u32, u32)],
    moving: &'a [u32],
    all: &'a [u32],
    n: u32,
}

/// 卡上档内部的分项（`prof` 的 8 项，进程级累加 ⇒ `World` 路径里的那个档也数得到）。
fn show_prof(label: &str, p: &[u64; 8]) {
    let n = p[7].max(1) as f64;
    let ms = |i: usize| p[i] as f64 / n / 1e3;
    println!(
        "       {label}内部（每趟均值 ms，{} 趟）：变动打包 {:.3} | 上传写 {:.3} | 参数+对表写 {:.3} | 编码 {:.3} | **提交+映射+等 {:.3}** | 解码 {:.3} | 摊平 {:.3}",
        p[7],
        ms(0),
        ms(1),
        ms(2),
        ms(3),
        ms(4),
        ms(5),
        ms(6)
    );
}

/// **自证**：整表上传 vs 常驻 ⇒ 槽表必须**逐位相同**（换上传方式不该改结果）。
///
/// ⚠️ 必须先**整表同步**一次常驻表：`run_resident` 吃的是卡上那一份，未同步时它是**未初始化**的
/// （首版把自证排在 ② 之后 ⇒ "顺带"成立；重排到最前后必须显式同步，否则自证测的是垃圾 —— 实测
/// 表现为"不同 ✗"，差点被当成解码改动的回归）。
fn report_selfcheck(c: &Ctx) -> bool {
    c.tier.upload_records(c.packed, c.all).expect("整表同步");
    let a = c.tier.run(c.packed, c.flat).expect("跑不通");
    let b = c.tier.run_resident(c.flat).expect("跑不通");
    let same = a.slots.len() == b.slots.len()
        && a.slots
            .iter()
            .zip(b.slots.iter())
            .all(|(x, y)| x.words == y.words);
    println!(
        "     自证：整表 vs 常驻 槽表 ⇒ {}",
        if same {
            "**逐位相同 ✓**"
        } else {
            "**不同 ✗**"
        }
    );
    same
}

/// ① 整表上传 + 跑 / ② 变动记录上传 + 跑 / ③ 只跑 / ⑧ 往返地板 / ⑫ 空转下的往返 + 后端内部分项。
fn report_calls(c: &Ctx, gap_ms: u64) {
    let (t_full, flo, fhi) = window_ms(|| {
        let _ = c.tier.run(c.packed, c.flat).expect("跑不通");
    });
    // ② 首帧整表（一次）⇒ 之后只传变动记录 + 跑。
    c.tier.upload_records(c.packed, c.all).expect("首帧上传");
    let (t_res, rlo, rhi) = window_ms(|| {
        c.tier.upload_records(c.packed, c.moving).expect("变动上传");
        let _ = c.tier.run_resident(c.flat).expect("跑不通");
    });
    // ③ 只跑（体表已常驻、连变动都不传）。
    prof::reset();
    let (t_kern, klo, khi) = window_ms(|| {
        let _ = c.tier.run_resident(c.flat).expect("跑不通");
    });
    show_prof("③", &prof_snapshot());
    // ⑧ **往返地板**（对表 16 对）+ ⑫ 同 ⑧ 但每次之间空转 `gap` ms。
    let tiny: Vec<u32> = c.flat[..32.min(c.flat.len())].to_vec();
    prof::reset();
    let (t_floor, clo, chi) = window_ms(|| {
        let _ = c.tier.run_resident(&tiny).expect("跑不通");
    });
    show_prof("⑧", &prof_snapshot());
    let (t_gap, gl2, gh2) = window_ms_gap(
        || {
            let _ = c.tier.run_resident(&tiny).expect("跑不通");
        },
        gap_ms,
    );
    println!("     读数（窗口均值 ms）:");
    println!("       ① 整表上传 + 跑            {t_full:>7.3}（{flo:.3}–{fhi:.3}）");
    println!("       ② 变动记录上传 + 跑        {t_res:>7.3}（{rlo:.3}–{rhi:.3}）");
    println!("       ③ 只跑（体表已常驻）       {t_kern:>7.3}（{klo:.3}–{khi:.3}）");
    println!("       ⑧ 往返地板（对表 16 对）   {t_floor:>7.3}（{clo:.3}–{chi:.3}）");
    println!("       ⑫ 同 ⑧ 但之间空转 {gap_ms} ms   {t_gap:>7.3}（{gl2:.3}–{gh2:.3}）");
    println!(
        "     ⇒ ⑫/⑧ = {:.1}× ⇒ {}",
        t_gap / t_floor.max(1e-9),
        if t_gap > t_floor * 2.0 {
            "**同步往返在 GPU 低谷后要贵一个量级**（主项是唤醒延迟，不是算/带宽/解码）"
        } else {
            "空转对往返影响不大（主项不在唤醒延迟）"
        }
    );
    let (up_full, up_delta) = (t_full - t_kern, t_res - t_kern);
    println!(
        "     ⇒ 上传这一项：整表 {up_full:.3} ms → 变动 {up_delta:.3} ms（{}）",
        if up_full > 0.0 && up_delta > 0.0 {
            format!("快 {:.1}×", up_full / up_delta)
        } else {
            "（窗口太吵，看不出比值）".to_string()
        }
    );
}

/// ④ 整表打包 / ⑤ 逐记录比对 / ⑦ 对表打平 / ⑥ 只打包动体（杠杆①的理想上界）。
fn report_host(c: &Ctx) {
    let (t_pack, plo, phi) = window_ms(|| {
        std::hint::black_box(pack_bodies(c.bodies)); // 每次新分配 + 写整表（现行为）
    });
    // 逐记录比对 + 命中写回：照抄门面里的循环（升序、逐记录 12 词切片比）。
    let mut cache = c.packed.to_vec();
    let mut changed: Vec<u32> = Vec::new();
    let (t_diff, dlo, dhi) = window_ms(|| {
        changed.clear();
        for i in 0..c.n as usize {
            let o = i * BODY_WORDS;
            if cache[o..o + BODY_WORDS] != c.packed[o..o + BODY_WORDS] {
                changed.push(i as u32);
                cache[o..o + BODY_WORDS].copy_from_slice(&c.packed[o..o + BODY_WORDS]);
            }
        }
        std::hint::black_box(&changed);
    });
    // ⑥ 只打包动体：用整表当数据源只是**同位替代**（真身是直取 `BodySet`），量级可比。
    let mut drec: Vec<u32> = Vec::new();
    let (t_dyn, ylo, yhi) = window_ms(|| {
        drec.clear();
        for &i in c.moving {
            let o = i as usize * BODY_WORDS;
            drec.extend_from_slice(&c.packed[o..o + BODY_WORDS]);
        }
        std::hint::black_box(&drec);
    });
    let (t_flat, glo, ghi) = window_ms(|| {
        std::hint::black_box(flat_pairs(c.pairs));
    });
    println!("     主机侧分项（卡段之外，窗口均值 ms）:");
    println!("       ④ 整表打包 pack_bodies     {t_pack:>7.3}（{plo:.3}–{phi:.3}）");
    println!("       ⑤ 逐记录比对 + 写回缓存    {t_diff:>7.3}（{dlo:.3}–{dhi:.3}）");
    println!("       ⑦ 对表打平 flat_pairs      {t_flat:>7.3}（{glo:.3}–{ghi:.3}）");
    println!(
        "       ⑥ 只打包动体（{} 体，{} KB）  {t_dyn:>7.3}（{ylo:.3}–{yhi:.3}）",
        c.moving.len(),
        c.moving.len() * BODY_WORDS * 4 / 1024
    );
    println!(
        "     ⇒ 现主机侧 ④+⑤+⑦ = {:.3} ms；杠杆①的理想上界（⑥ 替 ④+⑤） = {t_dyn:.3} ms ⇒ {}",
        t_pack + t_diff + t_flat,
        if t_pack + t_diff > t_dyn {
            format!("**省 {:.3} ms**", t_pack + t_diff - t_dyn)
        } else {
            "**不省**（别做）".to_string()
        }
    );
}

/// ⑨ 现状解码（逐槽逐字 + 逐槽 extend）/ ⑩ 一次过（`chunks_exact` 直出字表）。
fn report_decode(c: &Ctx) {
    let nb = c.pairs.len() * SLOT_BYTES;
    let bytes: Vec<u8> = vec![0u8; nb];
    let (t_dec, elo, ehi) = window_ms(|| {
        let out: Vec<Slot> = (0..c.pairs.len())
            .map(|i| {
                let b = i * SLOT_BYTES;
                let mut words = [0u32; SLOT_WORDS];
                for (k, w) in words.iter_mut().enumerate() {
                    let o = b + k * 4;
                    *w = u32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
                }
                Slot { words }
            })
            .collect();
        let mut flat: Vec<u32> = Vec::with_capacity(out.len() * SLOT_WORDS);
        for s in &out {
            flat.extend_from_slice(&s.words);
        }
        std::hint::black_box(&flat);
    });
    let (t_one, ilo, ihi) = window_ms(|| {
        let mut w: Vec<u32> = Vec::with_capacity(nb / 4);
        for ch in bytes.chunks_exact(4) {
            w.push(u32::from_le_bytes([ch[0], ch[1], ch[2], ch[3]]));
        }
        std::hint::black_box(&w);
    });
    println!("     回读解码（含在 ③ 里，随对数线性）:");
    println!("       ⑨ 现状：逐字 from_le_bytes + 逐槽 extend  {t_dec:>7.3}（{elo:.3}–{ehi:.3}）");
    println!("       ⑩ 一次过：chunks_exact 直出字表        {t_one:>7.3}（{ilo:.3}–{ihi:.3}）");
    println!(
        "     ⇒ ⑨→⑩ 省 {:.3} ms（{}）",
        t_dec - t_one,
        if t_dec > t_one {
            format!("{:.1}×", t_dec / t_one.max(1e-9))
        } else {
            "**不省**（别做）".to_string()
        }
    );
}

fn main() {
    let a = parse_args();
    println!(
        "== 常驻体表：整表上传 vs 变动记录（m{} 档，同输入、同进程交替）==",
        a.mult
    );
    // 场景推进到**接触密集**（与 §17.9 的探针同款预热：不预热则对表近乎空）。
    let mut w = scene_m1(a.mult);
    let jobs = vxl_phys_core::schedule::ScopedPool::new(8);
    let mut bp = vxl_phys_broad::GridBroadPhase::new(2.0, cfg().contact_skin);
    for _ in 0..WARM {
        w.step();
    }
    let n = w.bodies.len() as u32;
    // 参考对表：用**确定性**的 GridBroadPhase（与卡上档的输入无关，只为给一份同规模的真对表）。
    let pairs = bp.compute_pairs(&w.bodies, &[], &[], &jobs).to_vec();
    let packed = pack_bodies(&w.bodies);
    let flat = flat_pairs(&pairs);
    let cap_p = a
        .cap_p
        .unwrap_or((pairs.len() as u32 + 8).min(CAP_PAIRS))
        .max(pairs.len() as u32 + 8);
    let Ok(tier) = NarrowTier::new(
        a.adapter,
        CAP_BODIES * a.mult as u32,
        cap_p,
        cfg().contact_skin,
    ) else {
        println!("  ⚠️ 本机无可用适配器 ⇒ **未判定**");
        std::process::exit(2);
    };
    println!("  适配器：{}", tier.adapter());
    println!(
        "  体 {n}（表 {} KB）| 对 {}（表 {} KB）| 窗口 {WINDOW}",
        packed.len() * 4 / 1024,
        pairs.len(),
        flat.len() * 4 / 1024
    );
    // **变动集** = 清醒的动体（真场景里每 tick 动的就是它们）。
    let moving: Vec<u32> = (0..n)
        .filter(|&i| w.bodies.awake[i as usize] && w.bodies.is_dynamic(i as usize))
        .collect();
    let mut all: Vec<u32> = (0..n).collect();
    all.shrink_to_fit();
    println!(
        "  变动集：动体 {} 个（{:.1}% 的记录；{} KB vs 整表 {} KB）",
        moving.len(),
        moving.len() as f64 / n as f64 * 100.0,
        moving.len() * BODY_WORDS * 4 / 1024,
        packed.len() * 4 / 1024,
    );
    let c = Ctx {
        tier: &tier,
        bodies: &w.bodies,
        packed: &packed,
        flat: &flat,
        pairs: &pairs,
        moving: &moving,
        all: &all,
        n,
    };
    report_selfcheck(&c);
    report_calls(&c, a.gap_ms);
    report_host(&c);
    report_decode(&c);
}
