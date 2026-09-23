//! **GPU 网格重建 vs CPU 计数排序：逐位同表**（`docs/PLAN-gpu.md` §11）。
//!
//! 产出三件：
//! ① `start` / `items` 与 CPU **逐位比对**（含逐粒 `bin` 反查一致性——CPU 侧从 `start/items`
//!    **反推**每粒的格号，不重抄分箱公式，免得"两边抄同一份错"）；
//! ② `overflow`（逐格规范化的护栏计数，**判据是 0**）；
//! ③ 性能：GPU 每轮 ms（四入口 + 清零 + 拷贝 + 同步回读）vs CPU 网格相位 ms（同档同输入）。
//!
//! ⚠️ **口径（踩过的坑）**：引擎在**子步开头**用当时的位置建表，而 `positions()` 在步进后已经
//! 前进了一个子步 ⇒ 必须先**快照**位置，再跑一步，拿那一步的 `neighbor_grid()` 与快照对齐。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_grid_probe -- [n] [--adapter K] [--cap C]`

use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::grid::{self, GridInputs, GridParams};
use vxl_phys_gpu::probe;

fn main() {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let rest: Vec<String> = args.collect();
    let mut adapter_index = 0usize;
    let mut cap = 512u32;
    if let Some(k) = rest.iter().position(|a| a == "--adapter") {
        adapter_index = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    if let Some(k) = rest.iter().position(|a| a == "--cap") {
        cap = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(512);
    }

    println!("== 适配器清单（不绑厂商核对）==");
    for (i, a) in probe::adapters().iter().enumerate() {
        println!("  [{i}] {a}");
    }

    let spacing = 0.05f32;
    let cfg = FluidConfig::default();
    let substeps = cfg.substeps.max(1) as f64;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    // 快照（这一组位置才是"下一步要建表用的"）⇒ 跑一步，拿它建出来的表
    let pos: Vec<Vec3> = f.positions().to_vec();
    f.reset_phase_us();
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    let cpu_grid_ms_step = f.phase_us()[0] as f64 / 1e3;
    let (cmin, cinv, cdims, cstart, citems) = {
        let gd = f.neighbor_grid();
        (
            gd.min,
            gd.inv,
            gd.dims,
            gd.start.to_vec(),
            gd.items.to_vec(),
        )
    };
    let np = pos.len();
    let total = (cdims.0 as usize) * (cdims.1 as usize) * (cdims.2 as usize);

    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for p in &pos {
        pos_flat.extend_from_slice(&[p.x, p.y, p.z]);
    }
    let params = GridParams {
        gmin: [cmin.x, cmin.y, cmin.z],
        inv: cinv,
        nx: cdims.0,
        ny: cdims.1,
        nz: cdims.2,
        n: np as u32,
        total: total as u32,
        cap,
        _pad0: 0,
        _pad1: 0,
    };
    let out = grid::grid_on_adapter(
        adapter_index,
        &GridInputs {
            pos_flat: &pos_flat,
        },
        params,
        20,
    );
    if let Some(e) = out.error {
        println!("GPU 路径不可用：{e}");
        return;
    }

    // ① start / items 逐位
    let cmp_u32 = |a: &[u32], b: &[u32]| -> (usize, Option<(usize, u32, u32)>) {
        let mut bad = 0usize;
        let mut first = None;
        for k in 0..a.len().min(b.len()) {
            if a[k] != b[k] {
                bad += 1;
                if first.is_none() {
                    first = Some((k, a[k], b[k]));
                }
            }
        }
        (bad, first)
    };
    let (s_bad, s_first) = cmp_u32(&cstart, &out.start);
    let (i_bad, i_first) = cmp_u32(&citems, &out.items);
    // ② 逐粒 bin：CPU 侧从表反推
    let mut cpu_bin = vec![u32::MAX; np];
    for c in 0..total {
        for k in cstart[c] as usize..cstart[c + 1] as usize {
            cpu_bin[citems[k] as usize] = c as u32;
        }
    }
    let (b_bad, b_first) = cmp_u32(&cpu_bin, &out.bins);
    let cpu_grid_ms_sub = cpu_grid_ms_step / substeps;
    // 表哈希（FNV-1a over start/items/bins）：① 可当将来的回归门读数；
    // ② 证明"原子占位的不定序"已被 `canon` 规范化掉——同一输入任意轮次都该给同一哈希。
    let fnv = |h: &mut u64, v: u32| {
        *h ^= u64::from(v);
        *h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    let hash_of = |a: &[u32], b: &[u32], c: &[u32]| -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for v in a {
            fnv(&mut h, *v);
        }
        fnv(&mut h, 0xffff_ffff);
        for v in b {
            fnv(&mut h, *v);
        }
        fnv(&mut h, 0xffff_ffff);
        for v in c {
            fnv(&mut h, *v);
        }
        h
    };
    let h_cpu = hash_of(&cstart, &citems, &cpu_bin);
    let h_gpu = hash_of(&out.start, &out.items, &out.bins);

    println!(
        "== GPU 网格重建对拍（{np} 粒；{}×{}×{} = {total} 格；cap {cap}）==",
        cdims.0, cdims.1, cdims.2
    );
    println!("  适配器：{}", out.adapter);
    println!("  表哈希：CPU 0x{h_cpu:016x} | GPU 0x{h_gpu:016x}");
    println!(
        "  start：{} 项 | 与 CPU 逐位不同的项 {s_bad}{}",
        cstart.len(),
        match s_first {
            None => " ⇒ ✅ **全等**".to_string(),
            Some((k, a, b)) => format!("（首处 k={k}：CPU {a} vs GPU {b}）"),
        }
    );
    println!(
        "  items：{} 项 | 与 CPU 逐位不同的项 {i_bad}{}",
        citems.len(),
        match i_first {
            None => " ⇒ ✅ **全等**（格内序 = 索引升序，与 CPU 一致）".to_string(),
            Some((k, a, b)) => format!("（首处 k={k}：CPU {a} vs GPU {b}）"),
        }
    );
    println!(
        "  逐粒 bin（CPU 由表反推）：不同 {b_bad} 粒{}",
        match b_first {
            None => " ⇒ ✅ 一致".to_string(),
            Some((k, a, b)) => format!("（首处 i={k}：CPU {a} vs GPU {b}）"),
        }
    );
    println!(
        "  overflow（未规范化的格数，判据 0）：{} {}",
        out.overflow,
        if out.overflow == 0 {
            "✅"
        } else {
            "❌ ⇒ 有格段超过 cap，表不再与 CPU 同表"
        }
    );
    println!(
        "  耗时：GPU **{:.3} ms/轮**（四入口 + 清零 + 拷贝 + 同步回读，20 轮均值）| 一次性 setup {:.0} ms",
        out.per_run_ms, out.setup_ms
    );
    println!(
        "        CPU **{:.3} ms/子步**（`phase_us()[0]` ÷ {substeps} 子步，threads={}）⇒ **{:.1}×**",
        cpu_grid_ms_sub,
        f.config().threads,
        cpu_grid_ms_sub / out.per_run_ms.max(1e-6) as f64
    );
    println!(
        "  量纲提醒：GPU 侧这 {:.3} ms 含**每轮一次同步回读**（校验用）；进真实管线后回读只在整 tick 做一次。",
        out.per_run_ms
    );
}
