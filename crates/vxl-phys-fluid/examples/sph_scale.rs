//! **SPH 规模档**（首个体量档读数）：SPEC §3 给 CPU SPH 的目标是 **30 万粒 @30 FPS**
//! （= 33.3 ms/tick 的整帧预算，流体占其中一部分）。本 example 量"纯流体"的每 tick 成本：
//! 造 `n³` 晶格水块（间距 `h/2`），跑 `ticks` 个 tick，打印 ms/tick 与等效 FPS。
//!
//! 运行：`cargo run --release -p vxl-phys-fluid --example sph_scale -- [n] [substeps] [ticks]`
//! 默认 `n=40`（64 000 粒）、`substeps=4`、`ticks=30`。
//! ⚠️ 这是**成本探针**，不是稳定性验收（大块水在显式压缩求解器下的沉降瞬态见 `PLAN-0.3.md` §4.2）。
//! ⚠️ 不接 provider（只量求解成本，避免把体素查询成本混进来）。

use std::time::Instant;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;

fn main() {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let substeps: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(4);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    let spacing = 0.05f32;
    let cfg = vxl_phys_fluid::FluidConfig {
        substeps,
        threads,
        ..vxl_phys_fluid::FluidConfig::default()
    };
    let mut sys = vxl_phys_fluid::FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    let np = sys.len();
    let h = sys.config().smoothing_radius;
    // 预热（首 tick 含网格/缓存首建）
    sys.step(1.0 / 60.0, &NoProviders);
    sys.reset_phase_us();
    let t0 = Instant::now();
    for _ in 0..ticks {
        sys.step(1.0 / 60.0, &NoProviders);
    }
    let ms = t0.elapsed().as_secs_f64() * 1e3 / ticks as f64;
    let ph = sys.phase_us();
    let per = |x: u64| x as f64 / 1e3 / ticks as f64;
    let mut vmax = 0.0f32;
    let mut nan = 0usize;
    // **状态哈希**（逐位口径）：pos+vel 的 f32 位按索引序折入 u64 ⇒ 串行/并行可比对。
    let mut hsh: u64 = 0xcbf2_9ce4_8422_2325;
    for (p, v) in sys.positions().iter().zip(sys.velocities().iter()) {
        if !p.is_finite() || !v.is_finite() {
            nan += 1;
        }
        vmax = vmax.max(v.length());
        for b in [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()] {
            hsh = (hsh ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3);
        }
        for b in [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()] {
            hsh = (hsh ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    println!(
        "SPH 规模档：{np} 粒（{n}³ @ {spacing} m、h {h:.2}）| substeps {substeps} | threads {threads} | {ticks} tick \
         ⇒ **{ms:.2} ms/tick**（{:>7.1} FPS 等效）| NaN {nan} | |v|max {vmax:.2} | HASH {hsh:#018x}",
        1000.0 / ms
    );
    println!(
        "  相位（ms/tick）：网格 {:6.2} | 密度 {:6.2} | 压力 {:6.2} | 力+黏度 {:6.2} | 积分+边界 {:6.2} | 相位合计 {:6.2}",
        per(ph[0]),
        per(ph[1]),
        per(ph[2]),
        per(ph[3]),
        per(ph[4]),
        per(ph[0] + ph[1] + ph[2] + ph[3] + ph[4])
    );
}
