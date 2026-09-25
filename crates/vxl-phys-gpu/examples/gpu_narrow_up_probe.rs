//! **常驻体表的对照读数**（`PLAN-gpu.md` §17.10）：同一份 m1 档场景、同一份对表，比两条路 ——
//! ① **整表上传**（`NarrowTier::run`：每趟 11k 体 × 48 B = 528 KB + 对表）；
//! ② **常驻 + 变动记录**（`upload_records(delta)` 只在首帧全传，之后只传**动**的那些体 + `run_resident`）。
//!
//! **口径**：窗口均值、独占（perf-lock）、**同输入**（同一帧的体表与对表）、**同进程交替**；
//! 两条路的**槽表逐位对照**（自证：换上传方式不该改结果）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_narrow_up_probe [--adapter K]`

use std::time::Instant;

use vxl_phys::{BroadPhase, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::narrow_tier::{flat_pairs, pack_bodies, BODY_WORDS};
use vxl_phys_gpu::narrow::NarrowTier;

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

/// 与 `gpu_narrow_tier_probe` 的 m1 档同源（10k 静态盒 + 1k 动态盒 + 平地）。
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

fn main() {
    let mut adapter = 0usize;
    let rest: Vec<String> = std::env::args().skip(1).collect();
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    println!("== 常驻体表：整表上传 vs 变动记录（m1 档，同输入、同进程交替）==");
    // 场景推进到**接触密集**（与 §17.9 的探针同款预热：不预热则对表近乎空）。
    let mut w = scene_m1();
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
    let cap_p = (pairs.len() as u32 + 8).min(CAP_PAIRS);
    let Ok(tier) = NarrowTier::new(adapter, CAP_BODIES, cap_p, cfg().contact_skin) else {
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
    // ① 整表上传 + 跑。
    let (t_full, flo, fhi) = window_ms(|| {
        let _ = tier.run(&packed, &flat).expect("跑不通");
    });
    // ② 首帧整表（一次）⇒ 之后只传变动记录 + 跑。
    tier.upload_records(&packed, &all).expect("首帧上传");
    let (t_res, rlo, rhi) = window_ms(|| {
        tier.upload_records(&packed, &moving).expect("变动上传");
        let _ = tier.run_resident(&flat).expect("跑不通");
    });
    // ③ 只跑（体表已常驻、连变动都不传）：给"上传"这一项单独定量。
    let (t_kern, klo, khi) = window_ms(|| {
        let _ = tier.run_resident(&flat).expect("跑不通");
    });
    // **自证**：两条路的槽表必须**逐位相同**（换上传方式不该改结果）。
    let a = tier.run(&packed, &flat).expect("跑不通");
    let b = tier.run_resident(&flat).expect("跑不通");
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
    println!("     读数（窗口均值 ms）:");
    println!("       ① 整表上传 + 跑            {t_full:>7.3}（{flo:.3}–{fhi:.3}）");
    println!("       ② 变动记录上传 + 跑        {t_res:>7.3}（{rlo:.3}–{rhi:.3}）");
    println!("       ③ 只跑（体表已常驻）       {t_kern:>7.3}（{klo:.3}–{khi:.3}）");
    let up_full = t_full - t_kern;
    let up_delta = t_res - t_kern;
    println!(
        "     ⇒ 上传这一项：整表 {up_full:.3} ms → 变动 {up_delta:.3} ms（{}）",
        if up_full > 0.0 && up_delta > 0.0 {
            format!("快 {:.1}×", up_full / up_delta)
        } else {
            "（窗口太吵，看不出比值）".to_string()
        }
    );
}
