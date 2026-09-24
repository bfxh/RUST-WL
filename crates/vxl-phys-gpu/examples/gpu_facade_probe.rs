//! **facade 接线验收**（`PLAN-gpu.md` §13.7 选项 A）：同一个场景跑两条链——
//! A：整段 CPU（`World::step` 原路径）；B：该流体的**步进在卡上**（`World::set_fluid_stepper`）。
//! 要证明的是：**主机侧只留"边界重建 + 卡上 AABB"之后，场景仍与整段 CPU 一致**。
//!
//! 场景（与 `tests/fluid_boundary.rs` 同款铸装思路）：静态盒体容器（地板 + 四壁 ⇒ 2b 边界粒子）
//! + 8³@0.05 水块 + 一块**比水轻的动态盒**（浮起/起伏 ⇒ 全程有反作用流回体上）。
//!
//! 两条链的差别只有一处：B 链的 `fluid_pass` 走 `FluidStepper::step`（只上传**边界段**、卡上
//! 推进 + 聚合每体 `(F, τ)`），主机不再推进自己的流体状态（那份只当边界重建的脚手架）。
//!
//! 判据（**口径 B**：浮点相位不逐位，见 §9.6）：① 动态体的轨迹（`y`/`vy`，CPU vs GPU）；
//! ② 逐 tick 最坏相对差（只在**有载** tick 上取，见 §13.6 的教训）；③ 无 NaN。
//! ⚠️ B 链的近域盒子来自**卡上**（卡上网格盒 ≈ 真实 AABB + ≤1 bin）⇒ 容器体贴着水时不触发差异；
//! 临界体（恰在一 bin 带内）可能与 A 链的近域集不同，那是这套接法的**已知边界**，写在这里备查。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_facade_probe -- [ticks] [--adapter K]`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{GpuFluidStepper, Packet, PacketCfg};

/// 水块：8³ @ 0.05（足印 0.4 m、深 0.4 m），铸装就位（不带落差入盆——那会触发顶心喷泉）。
const SPACING: f32 = 0.05;
/// 浮盒密度（kg/m³）：比水轻 ⇒ 浮起。
const FLOAT_RHO: f32 = 400.0;

/// 造场景：静态盒体容器 + 水块（2b）+ 浮盒（动态）。返回 `World` 与流体索引。
fn scene() -> (World, usize) {
    let mut w = World::new(PhysConfig::default());
    // 容器：地板（顶面 y = 1.05 = 水底）+ 四壁（内面 x/z = ±0.2 = 水足印边）。
    let t = 0.05f32;
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.3, t, 0.3),
        },
        Vec3::new(0.0, 1.05 - t, 0.0),
        Quat::IDENTITY,
    );
    for (x, z) in [(0.25, 0.0), (-0.25, 0.0), (0.0, 0.25), (0.0, -0.25)] {
        let (hx, hz) = if x != 0.0 { (t, 0.3) } else { (0.3, t) };
        w.add_static(
            Shape::Box {
                half: Vec3::new(hx, 0.25, hz),
            },
            Vec3::new(x, 1.3, z),
            Quat::IDENTITY,
        );
    }
    let sys = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        SPACING,
    );
    let fi = w.add_fluid_with_boundary_coupling(sys, &[]);
    // 浮盒：0.2 m 立方（4h 厚 ⇒ 反作用准），放在水面上方 ⇒ 落水后起伏。
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 1.55, 0.0),
        Quat::IDENTITY,
        FLOAT_RHO,
    );
    (w, fi)
}

/// 造 `PacketCfg`（与各探针同一套映射）。
fn make_cfg(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let gd = f.neighbor_grid();
    let (apos, ..) = f.raw_particles();
    PacketCfg {
        n: apos.len() as u32,
        n_fluid: f.len() as u32,
        total: gd.dims.0 * gd.dims.1 * gd.dims.2,
        gmin: [gd.min.x, gd.min.y, gd.min.z],
        inv: gd.inv,
        dims: [gd.dims.0, gd.dims.1, gd.dims.2],
        cap: 512,
        h,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        ks,
        mass: f.particle_mass(),
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        gravity: [
            f.config().gravity.x,
            f.config().gravity.y,
            f.config().gravity.z,
        ],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box: true,
        grid_bins_cap: 1 << 20,
    }
}

/// 按世界当前流体状态（**含已生成的边界段**）建卡上后端。`None` = 没起 GPU 管线。
fn build_stepper(w: &World, fi: usize, adapter: usize) -> Option<GpuFluidStepper> {
    let f = &w.fluids()[fi].0;
    let (apos, avel, apmass, nf) = f.raw_particles();
    let nb = apos.len() - nf;
    let mut p = Vec::with_capacity(apos.len() * 3);
    let mut v = Vec::with_capacity(apos.len() * 3);
    for k in 0..apos.len() {
        p.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        v.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    let pc = make_cfg(f);
    let substeps = f.config().substeps.max(1) as usize;
    let pk = Packet::new(adapter, pc, &p, &v, apmass).ok()?;
    // 本场景的容器是 2b 边界粒子（不是 provider 壁面）⇒ 壁面档给 `None`、边界表给空。
    Some(GpuFluidStepper::new(
        pk,
        None,
        Vec::new(),
        pc,
        substeps,
        nf as u32,
        nb,
    ))
}

/// 浮盒（唯一动态体）的 `(y, vy)`；两个世界同序 ⇒ 索引相同。
fn body_yv(w: &World) -> (f32, f32) {
    let mut out = (0.0f32, 0.0f32);
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            out = (w.bodies.position[i].y, w.bodies.linvel[i].y);
        }
    }
    out
}

fn main() {
    let mut it = std::env::args().skip(1);
    let ticks: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(150);
    let rest: Vec<String> = it.collect();
    let mut adapter = 0usize;
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    let (mut wa, fi) = scene();
    let (mut wb, fib) = scene();
    // 热身 **1 tick**：让边界粒子生成（B 链要从这一刻的同一状态建 Packet）。
    wa.step();
    wb.step();
    assert_eq!(fi, fib);
    let Some(st) = build_stepper(&wb, fib, adapter) else {
        println!("⚠️ 没起 GPU 管线——本探针需要适配器");
        return;
    };
    let nb = wb.fluids()[fib].0.boundary_count();
    assert!(
        wb.set_fluid_stepper(fib, Box::new(st)),
        "登记卡上后端失败（流体索引越界）"
    );
    let (mut worst_rel, mut worst_v, mut vscale) = (0.0f32, 0.0f32, 0.0f32);
    let mut trace = Vec::new();
    for t in 0..ticks {
        wa.step();
        wb.step();
        let (ya, va) = body_yv(&wa);
        let (yb, vb) = body_yv(&wb);
        // 只在**有载**时取相对量（体还在空中时水未接触 ⇒ 力是噪声，比值无意义，见 §13.6）。
        if ya < 1.52 {
            worst_rel = worst_rel.max((ya - yb).abs() / ya.abs().max(1e-6));
            worst_v = worst_v.max((va - vb).abs());
            vscale = vscale.max(va.abs());
        }
        let tk = t + 1;
        if matches!(tk, 1 | 5 | 20) || tk % 50 == 0 || tk == ticks {
            trace.push((tk, ya, yb, va, vb));
        }
    }
    let (_, va) = body_yv(&wa);
    let (_, vb) = body_yv(&wb);
    println!("== facade 接线（选项 A：步进在卡上）vs 整段 CPU ==");
    println!(
        "  8³@0.05 水块（{nb} 边界粒子）+ 浮盒（ρ={FLOAT_RHO}）| {ticks} tick（前 1 tick 热身同为 CPU 路径）"
    );
    println!("  ① 浮盒轨迹（y / vy，CPU vs 卡上）：");
    for (t, ya, yb, va, vb) in &trace {
        println!(
            "     t={t:>4}：y {ya:.5} vs {yb:.5}（差 {:.2e}）| vy {va:.5} vs {vb:.5}（差 {:.2e}）",
            (ya - yb).abs(),
            (va - vb).abs()
        );
    }
    println!(
        "  ② 逐 tick 最坏差（只在有载 tick 上取）：|Δy| 相对 {worst_rel:.2e}；|Δvy| 绝对 {worst_v:.2e} m/s（按 |vy| 幅值 {vscale:.4} 归一 ⇒ {:.2e}）",
        worst_v / vscale.max(1e-6)
    );
    let (nya, nyb) = (body_yv(&wa).0, body_yv(&wb).0);
    println!(
        "  ③ 末速 vy：CPU {va:.5} / 卡上 {vb:.5}（相对差 {:.2e}）| 非有限 y：{} / {}",
        (va - vb).abs() / va.abs().max(1e-6),
        !nya.is_finite(),
        !nyb.is_finite()
    );
    println!(
        "      ⚠️ vy 的相对差要按**幅值**读：浮盒在起伏，vy 过零时逐点相对量会放大（§13.6 同款教训）。"
    );
}
