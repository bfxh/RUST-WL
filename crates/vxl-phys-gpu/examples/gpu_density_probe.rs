//! **GPU vs CPU 两相位对拍**（首里程碑：密度 + 力/黏度；`docs/PLAN-gpu.md` §6/§9）。
//!
//! 产出四件：
//! ① **适配器清单**（含 Intel iGPU ⇒ "不绑厂商"可验）；
//! ② **一致性**：与 GPU **吃同一份输入**的 CPU 参考实现逐位/量化比对（口径 A 的判据）；
//! ③ **性能**：GPU 两相位**稳态每轮**（含同步回读）与一次性 setup 分开报；
//! ④ 与**引擎相位报表**（`sph_scale`）并列，给"离真实 tick 还有多远"的量级。
//!
//! ⚠️ **口径（首片踩过）**：不能拿引擎 `step()` 之后的 `densities()` 与"按当前位置算的 GPU 密度"
//! 相比——引擎的密度是**积分前**位置算的，位置已经前进了一个子步（≈0.4–12 mm）⇒ 那是拿两个
//! 不同输入在比。本示例因此自带 CPU 参考实现（与 GPU **同输入、同式、同遍历序**）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_density_probe -- [n] [--adapter K]`

use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::probe::{self, PhaseInputs, PhaseParams, PhasesOut};

/// **CPU 参考实现**（与 GPU 同输入、同式、同遍历序）：返回 `(密度, 加速度, xsph)`。
fn cpu_reference(f: &FluidSystem) -> (Vec<f32>, Vec<Vec3>, Vec<Vec3>) {
    let h = f.config().smoothing_radius;
    let h2 = h * h;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let w0 = k6 * h2 * h2 * h2;
    let mass = f.particle_mass();
    let g = f.config().gravity;
    let alpha_c = f.config().artificial_viscosity * f.config().sound_speed;

    let np = f.len();
    let pos: Vec<Vec3> = f.positions().to_vec();
    let vel: Vec<Vec3> = f.velocities().to_vec();
    let dens_cpu: Vec<f32> = f.densities().to_vec();
    let press_cpu: Vec<f32> = f.pressures().to_vec();
    let gd = f.neighbor_grid();

    let cell_range = |p: Vec3| -> (i32, i32, i32) {
        let f = |o: f32, v: f32, m: u32| -> i32 {
            ((((v - o) * gd.inv).floor().max(0.0) as u32).min(m - 1)) as i32
        };
        (
            f(gd.min.x, p.x, gd.dims.0),
            f(gd.min.y, p.y, gd.dims.1),
            f(gd.min.z, p.z, gd.dims.2),
        )
    };
    let (nx, ny, nz) = (gd.dims.0 as i32, gd.dims.1 as i32, gd.dims.2 as i32);
    let mut ref_dens = vec![0.0f32; np];
    let mut ref_acc = vec![Vec3::ZERO; np];
    let mut ref_xsph = vec![Vec3::ZERO; np];
    for i in 0..np {
        let pi = pos[i];
        let (cx, cy, cz) = cell_range(pi);
        let mut sum = w0;
        let vi = vel[i];
        let rho_i = dens_cpu[i];
        let ci2 = press_cpu[i] / (rho_i * rho_i);
        let mut a = g;
        let mut xs = Vec3::ZERO;
        for dz in -1..=1i32 {
            let z = cz + dz;
            if z < 0 || z >= nz {
                continue;
            }
            for dy in -1..=1i32 {
                let y = cy + dy;
                if y < 0 || y >= ny {
                    continue;
                }
                for dx in -1..=1i32 {
                    let x = cx + dx;
                    if x < 0 || x >= nx {
                        continue;
                    }
                    let ci = ((x as u32 * ny as u32 + y as u32) * nz as u32 + z as u32) as usize;
                    for k in gd.start[ci]..gd.start[ci + 1] {
                        let j = gd.items[k as usize] as usize;
                        if j == i {
                            continue;
                        }
                        let d = pi - pos[j];
                        // 口径 A 的**判定性诊断**（2026-09-23 做过，勿重跑）：把这一行换成
                        // `d.y.mul_add(d.y, d.x*d.x) + d.z*d.z`（模拟 GPU 的 FMA 收缩）⇒ >2ulp 的粒
                        // 2465→1437（-42%）但**最大 ulp 仍是 6** ⇒ 残差是**收缩噪声**、非结构性：
                        // 一个邻居项约 3e5 ulp，若邻域集/遍历序有差，ulp 差会是 6 位数。结论：
                        // naga 27 无 `precise`（前端没有该关键字）+ 驱动自由收缩 ⇒ **口径 A 到此为止**。
                        let r2 = d.x * d.x + d.y * d.y + d.z * d.z;
                        if r2 > h2 {
                            continue;
                        }
                        let t = h2 - r2;
                        let w = k6 * t * t * t;
                        sum += w;
                        // 力相位（同 GPU 核）
                        let rho_j = dens_cpu[j];
                        let cj2 = press_cpu[j] / (rho_j * rho_j);
                        let r = r2.sqrt();
                        let tr = h - r;
                        let coef = mass * (ks * tr * tr) * (ci2 + cj2);
                        let denom = r.max(1e-9);
                        a += d * (coef / denom);
                        let vij = vi - vel[j];
                        let vdn = -(vij.x * d.x + vij.y * d.y + vij.z * d.z);
                        if vdn > 0.0 {
                            let mu = vdn * h / (r2 + 0.01 * h2);
                            let c =
                                mass * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ks * tr * tr);
                            a += d * (c / denom);
                        }
                        xs += (vel[j] - vi) * (mass * 2.0 / (rho_i + rho_j) * w);
                    }
                }
            }
        }
        ref_dens[i] = mass * sum;
        ref_acc[i] = a;
        ref_xsph[i] = xs;
    }
    (ref_dens, ref_acc, ref_xsph)
}

/// f32 的单调位序键（ulp 比较用；负数区要翻转）。
fn ukey(x: f32) -> i32 {
    let b = x.to_bits() as i32;
    if b < 0 {
        i32::MIN.wrapping_sub(b)
    } else {
        b
    }
}

/// 两个 f32 的 ulp 距离。
fn ulp(a: f32, b: f32) -> u32 {
    ukey(a).abs_diff(ukey(b))
}

/// 向量场三通道比对：`(最大绝对差, 最大相对差, 逐位相同数, 最大 ulp, >2ulp 计数)`。
///
/// **ulp 形态诊断**：残差若是 1–2 ulp 量级 ⇒ 只是"逐步舍入/FMA 收缩"噪声；若成片多 ulp
/// ⇒ 结构性差异（得回头查式子/遍历序）。判据用 ulp 而不是相对差：相对差在近零处会爆掉。
fn cmp3(a: &[Vec3], b: &[f32]) -> (f32, f32, usize, u32, usize) {
    let mut maxd = 0.0f32;
    let mut maxr = 0.0f32;
    let mut bit = 0usize;
    let mut maxu = 0u32;
    let mut gt2 = 0usize;
    for (i, v) in a.iter().enumerate() {
        for (k, x) in [v.x, v.y, v.z].into_iter().enumerate() {
            let y = b[i * 3 + k];
            let d = (x - y).abs();
            if d > maxd {
                maxd = d;
            }
            let den = x.abs().max(1e-6);
            if d / den > maxr {
                maxr = d / den;
            }
            if x.to_bits() == y.to_bits() {
                bit += 1;
            }
            let u = ulp(x, y);
            if u > maxu {
                maxu = u;
            }
            if u > 2 {
                gt2 += 1;
            }
        }
    }
    (maxd, maxr, bit, maxu, gt2)
}

/// 一次对拍的全部读数（密度 + acc + xsph）。
struct Cmp {
    dmx: f32,
    dbt: usize,
    dmaxu: u32,
    dgt2: usize,
    amax: f32,
    arel: f32,
    abit: usize,
    amaxu: u32,
    agt2: usize,
    xmax: f32,
    xrel: f32,
    xbit: usize,
    xmaxu: u32,
    xgt2: usize,
}

/// 比对：密度逐粒（含 ulp 形态）+ acc / xsph 三通道。
fn compare(ref_dens: &[f32], ref_acc: &[Vec3], ref_xsph: &[Vec3], out: &PhasesOut) -> Cmp {
    let mut dmx = 0.0f32;
    let mut dbt = 0usize;
    let mut dmaxu = 0u32;
    let mut dgt2 = 0usize;
    for (a, b) in ref_dens.iter().zip(out.dens.iter()) {
        let dd = (a - b).abs();
        if dd > dmx {
            dmx = dd;
        }
        if a.to_bits() == b.to_bits() {
            dbt += 1;
        }
        let u = ulp(*a, *b);
        if u > dmaxu {
            dmaxu = u;
        }
        if u > 2 {
            dgt2 += 1;
        }
    }
    let (amax, arel, abit, amaxu, agt2) = cmp3(ref_acc, &out.acc);
    let (xmax, xrel, xbit, xmaxu, xgt2) = cmp3(ref_xsph, &out.xsph);
    Cmp {
        dmx,
        dbt,
        dmaxu,
        dgt2,
        amax,
        arel,
        abit,
        amaxu,
        agt2,
        xmax,
        xrel,
        xbit,
        xmaxu,
        xgt2,
    }
}

/// 对拍报表（读数 + 性能 + 口径声明）。
fn report(out: &PhasesOut, c: &Cmp, np: usize, cpu_ms: f64) {
    println!("== 对拍（两相位 {np} 粒；CPU 参考 = 示例内实现，与 GPU **同输入/同式/同遍历序**）==");
    println!("  适配器：{}", out.adapter);
    println!(
        "  密度：最大绝对差 {:.3e} kg/m³ | 逐位相同 {}/{}（{:.2}%）| 最大 ulp 差 {} | >2ulp 的粒 {}",
        c.dmx,
        c.dbt,
        np,
        100.0 * c.dbt as f64 / np as f64,
        c.dmaxu,
        c.dgt2
    );
    println!(
        "  acc ：最大绝对差 {:.3e} m/s² | 最大相对差 {:.2e} | 逐位相同 {}/{}（{:.2}%）| 最大 ulp 差 {} | >2ulp {}",
        c.amax,
        c.arel,
        c.abit,
        np * 3,
        100.0 * c.abit as f64 / (np * 3) as f64,
        c.amaxu,
        c.agt2
    );
    println!(
        "  xsph：最大绝对差 {:.3e} m/s | 最大相对差 {:.2e} | 逐位相同 {}/{}（{:.2}%）| 最大 ulp 差 {} | >2ulp {}",
        c.xmax,
        c.xrel,
        c.xbit,
        np * 3,
        100.0 * c.xbit as f64 / (np * 3) as f64,
        c.xmaxu,
        c.xgt2
    );
    println!(
        "  耗时：GPU **稳态每轮**（两核 + 同步回读，20 次均值）{:.3} ms | 一次性 setup {:.1} ms | CPU 参考实现（单线程朴素）{cpu_ms:.1} ms",
        out.per_dispatch_ms, out.setup_ms
    );
    println!(
        "  参照（引擎同档相位报表）：密度+力 ≈ {:.0} ms/tick（`sph_scale` 读；4 子步）",
        44.2 + 80.8
    );
    println!(
        "  口径：GPU 侧尚未含「网格重建」与「积分」；口径 A（位级）未达 ⇒ 走量化+容差（见 §9.2）。"
    );
}

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

    // —— 场景（纯流体：无 provider、无边界粒子 ⇒ 与两核的"流体分支"口径一致）——
    let spacing = 0.05f32;
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
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
    // 跑几个 tick 让状态远离初始晶格（密度/压力有内容），再取**同一份**输入给两边
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let np = f.len();
    let pos: Vec<Vec3> = f.positions().to_vec();
    let vel: Vec<Vec3> = f.velocities().to_vec();
    let press_cpu: Vec<f32> = f.pressures().to_vec();
    let mass = f.particle_mass();
    let g = f.config().gravity;
    let alpha_c = f.config().artificial_viscosity * f.config().sound_speed;
    let h2 = h * h;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let w0 = k6 * h2 * h2 * h2;
    let gd = f.neighbor_grid();

    // —— CPU 参考实现（与 GPU 同输入、同式、同遍历序）——
    let t_cpu = std::time::Instant::now();
    let (ref_dens, ref_acc, ref_xsph) = cpu_reference(&f);
    let cpu_ms = t_cpu.elapsed().as_secs_f64() * 1e3;

    // —— 扁平化输入（与 CPU 参考同一份）——
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        pos_flat.extend_from_slice(&[pos[k].x, pos[k].y, pos[k].z]);
        vel_flat.extend_from_slice(&[vel[k].x, vel[k].y, vel[k].z]);
    }
    let pmass = vec![mass; np];
    let params = PhaseParams {
        gmin: [gd.min.x, gd.min.y, gd.min.z],
        inv: gd.inv,
        h2,
        k6,
        w0,
        mass,
        ks,
        h,
        alpha_c,
        _pad0: 0.0,
        gvec: [g.x, g.y, g.z],
        n_fluid: np as u32,
        nx: gd.dims.0,
        ny: gd.dims.1,
        nz: gd.dims.2,
        _pad: 0,
    };
    let inputs = PhaseInputs {
        pos_flat: &pos_flat,
        vel_flat: &vel_flat,
        pmass: &pmass,
        press: &press_cpu,
        cell_start: gd.start,
        cell_items: gd.items,
    };

    let out = probe::phases_on_adapter(adapter_index, &inputs, params, np, 20);
    if let Some(e) = out.error {
        println!("GPU 路径不可用：{e}");
        return;
    }

    let c = compare(&ref_dens, &ref_acc, &ref_xsph, &out);
    report(&out, &c, np, cpu_ms);
}
