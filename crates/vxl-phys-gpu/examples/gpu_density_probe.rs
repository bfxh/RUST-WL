//! **GPU vs CPU 密度相位对拍**（首里程碑第一片；`docs/PLAN-gpu.md` §6）。
//!
//! 产出三件（缺一不可）：
//! ① **适配器清单**（含 Intel iGPU ⇒ 证明"不绑厂商"可验）；
//! ② **一致性**：同一份状态、同一张邻域表 ⇒ GPU 密度 vs CPU 密度的**最大绝对差**与
//!    **逐位相同**判定（口径 A 的判据）；
//! ③ **性能**：GPU 一轮（上传+分派+回读）的墙钟 ms，与 CPU 密度相位的 ms/tick 并列（先给量级）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_density_probe -- [n] [--adapter K]`
//! 默认 `n=40`（64 000 粒）；`n=50` ⇒ 125 000 粒。

use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::probe::{self, DensityParams};

fn main() {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let mut adapter_index = 0usize;
    let rest: Vec<String> = args.collect();
    if let Some(k) = rest.iter().position(|a| a == "--adapter") {
        adapter_index = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }

    println!("== 适配器清单（不绑厂商核对：Intel iGPU 也应在列）==");
    let list = probe::adapters();
    if list.is_empty() {
        println!("  （本机无可用适配器 ⇒ CI 环境下的正常路径；本探针不构成门禁）");
        return;
    }
    for (i, a) in list.iter().enumerate() {
        println!("  [{i}] {a}");
    }

    // —— CPU 侧：跑一个 tick（CPU 算好密度 + 建好邻域网格）——
    let spacing = 0.05f32;
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        vxl_phys_core::Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    let t0 = std::time::Instant::now();
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    let cpu_ms = t0.elapsed().as_secs_f64() * 1e3;
    let cpu_dens = f.densities().to_vec();
    let np = f.len();

    // CPU 侧参数（与 `FluidSystem::new` 内同式：k6/w0 由 h 推）
    let h2 = h * h;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let w0 = k6 * h2 * h2 * h2;
    let mass = f.particle_mass();

    // 扁平化位置（GPU 布局 = 每粒 3 个 f32）
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for p in f.positions() {
        pos_flat.extend_from_slice(&[p.x, p.y, p.z]);
    }
    let pmass = vec![mass; np]; // 纯流体：逐粒质量 = mass（边界分支不会触发）
    let g = f.neighbor_grid();
    let params = DensityParams {
        gmin: [g.min.x, g.min.y, g.min.z],
        inv: g.inv,
        h2,
        k6,
        w0,
        mass,
        n_fluid: np as u32,
        nx: g.dims.0,
        ny: g.dims.1,
        nz: g.dims.2,
    };

    // —— GPU 侧 ——
    let out = probe::density_on_adapter(
        adapter_index,
        &pos_flat,
        &pmass,
        g.start,
        g.items,
        params,
        np,
        20,
    );
    if let Some(e) = out.error {
        println!("GPU 路径不可用：{e}");
        return;
    }
    // 一致性：最大绝对差 + 逐位相同计数
    let mut max_diff = 0.0f32;
    let mut bit_same = 0usize;
    for (a, b) in cpu_dens.iter().zip(out.dens.iter()) {
        let d = (a - b).abs();
        if d > max_diff {
            max_diff = d;
        }
        if a.to_bits() == b.to_bits() {
            bit_same += 1;
        }
    }
    println!("== 对拍（密度相位 {np} 粒）==");
    println!("  适配器：{}", out.adapter);
    println!(
        "  一致性：最大绝对差 {max_diff:.3e} kg/m³ | 逐位相同 {bit_same}/{np}（{:.2}%）",
        100.0 * bit_same as f64 / np as f64
    );
    println!(
        "  耗时：GPU **稳态每轮**（仅分派+提交，20 次均值）{:.3} ms | 一次性 setup（设备/缓冲/管线+上传+回读）{:.1} ms",
        out.per_dispatch_ms, out.setup_ms
    );
    println!(
        "  参照：CPU 完整 tick（网格+密度+压力+力+积分）{cpu_ms:.1} ms；其中**密度相位**请用 `sph_scale` 相位报表读同档读数"
    );
    println!("  口径：本片只搬了**密度相位**；GPU 侧尚未含「网格重建」与其它相位 ⇒ 只作首片量级与一致性对照。");
}
