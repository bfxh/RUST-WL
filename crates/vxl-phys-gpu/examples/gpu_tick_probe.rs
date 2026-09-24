//! **GPU 常驻管线：整 tick 读数 + 与 CPU 引擎的状态对照**（`docs/PLAN-gpu.md` §9.4 ③ / §12）。
//!
//! 做的事：同一初始状态、同一 tick 数，两条路各跑一遍——
//! ① **CPU 引擎**（`FluidSystem::step`，相位计时给出 ms/tick）；
//! ② **GPU 常驻管线**（`pipeline::Packet`：网格 → 密度 → EOS → 力 → 积分，缓冲只建一次）。
//! 产出：GPU 整 tick ms（**无逐 tick 回读**）、"含耦合回读"的预估、以及两侧末态的对照
//! （NaN 计数、|Δpos| / |Δv| 的极值与均值、动能比）——**口径 B 的轨迹口径**：浮点相位不逐位，
//! 所以多 tick 后位置会漂（混沌放大），这里只声明"同一物理、无发散"，不声明位级一致。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_tick_probe -- [n] [ticks] [--adapter K]`

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

/// `--tank` 的静态盒体（地板 + 四壁）：按"表面采样 + 两层内移"变成 2b 边界粒子
/// （壁厚取 2×晶格间距 ⇒ 采成实心；四壁贴着流体块的 x/z 边界，地板顶面 = 流体底面）。
fn tank_bodies(n: usize, spacing: f32, h: f32) -> Vec<(u32, Shape, vxl_phys_fluid::BodyPose)> {
    let half = n as f32 * spacing * 0.5;
    let t = 2.0 * spacing;
    let hgt = (n as f32 * spacing) + 2.0 * h;
    let tip = 0.5 + hgt * 0.5;
    let pose = |pos: Vec3| vxl_phys_fluid::BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    };
    let span = half + 2.0 * t;
    let wall_x = |x: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(t * 0.5, hgt * 0.5, span),
            },
            pose(Vec3::new(x, tip, 0.0)),
        )
    };
    let wall_z = |z: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(span, hgt * 0.5, t * 0.5),
            },
            pose(Vec3::new(0.0, tip, z)),
        )
    };
    vec![
        (
            0u32,
            Shape::Box {
                half: Vec3::new(span, t * 0.5, span),
            },
            pose(Vec3::new(0.0, 0.5 - t * 0.5, 0.0)),
        ),
        wall_x(half + t * 0.5, 1),
        wall_x(-(half + t * 0.5), 2),
        wall_z(half + t * 0.5, 3),
        wall_z(-(half + t * 0.5), 4),
    ]
}

fn main() {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
    let rest: Vec<String> = args.collect();
    let mut adapter_index = 0usize;
    if let Some(k) = rest.iter().position(|a| a == "--adapter") {
        adapter_index = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    // `--box=follow`：每子步从位置重算箱子（与 CPU 同频）；默认 `fixed` = §12.1 的读数口径。
    let follow_box = rest.iter().any(|a| a == "--box=follow");
    // `--gravity`：开重力（自由落体；配合 `--box=follow` 才能不跑出箱子）。
    let gravity_on = rest.iter().any(|a| a == "--gravity");
    // `--tank`：给 2b **边界粒子**（地板 + 四壁的静态盒体）——用来验"边界粒子在 GPU 侧也跑得对"：
    // 密度/力核里的 `sum_b`/`m_j = pmass[j]` 是 2b 的数学，积分则必须**只跑流体前缀**。
    let tank = rest.iter().any(|a| a == "--tank");

    let spacing = 0.05f32;
    // **零重力场景**：`pipeline::Packet` 的箱子是**固定**的（见其头注），自由落体会在几十个 tick 后
    // 把粒子全部钳进边缘格（段长撞 `cap`、计时失真）。关掉重力 ⇒ 流体留在箱内，且压 wave 让场景
    // 比自由落体更混沌 ⇒ 是**更好的**容差口径测试。重力项是每粒一次加法，不影响相位成本量级。
    let cfg0 = FluidConfig {
        gravity: if gravity_on {
            Vec3::new(0.0, -9.81 * 0.1, 0.0)
        } else {
            Vec3::ZERO
        },
        ..FluidConfig::default()
    };
    let h = cfg0.smoothing_radius;
    let substeps = cfg0.substeps.max(1) as usize;
    let dt_tick = 1.0 / 60.0;
    let mut f = FluidSystem::new(
        cfg0,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    for _ in 0..5 {
        f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
    }
    if tank {
        let bodies = tank_bodies(n, spacing, h);
        let nb = f.set_boundary_particles(&bodies);
        println!("  2b 边界粒子：{nb} 个（地板 + 四壁；流体 {} 个）", f.len());
    }
    // **给一个剪切初速**：零重力下完美晶格的 ρ ≡ ρ0 ⇒ p ≡ 0 ⇒ 全程静止（那样容差口径根本没被压到）。
    // 加一层 shear ⇒ 有真动力学，又是**受限**的（流体留在箱内，不撞固定箱子的边缘格）。
    {
        let mut vs = f.velocities().to_vec();
        for (i, v) in vs.iter_mut().enumerate() {
            let p = f.positions()[i];
            v.x += 0.6 * (p.y * 12.0).sin();
            v.z += 0.4 * (p.y * 8.0).cos();
        }
        f.set_velocities(&vs);
    }
    // 起点 = 这一刻；箱子取这一刻引擎算出来的那套（**本片固定不重算**，见 pipeline.rs 头注）
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    // **全部粒子**（含 2b 边界粒子）：`raw_particles` 给后端用的全量视图（`positions()` 只给流体前缀）。
    let (apos, avel, apmass, n_fluid) = f.raw_particles();
    let pos0: Vec<Vec3> = apos.to_vec();
    let vel0: Vec<Vec3> = avel.to_vec();
    let np = pos0.len();
    let total = gdims.0 * gdims.1 * gdims.2;

    // ① GPU 常驻管线（与 CPU 引擎**同一 pos0/vel0**，逐 tick 并行推进）
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        pos_flat.extend_from_slice(&[pos0[k].x, pos0[k].y, pos0[k].z]);
        vel_flat.extend_from_slice(&[vel0[k].x, vel0[k].y, vel0[k].z]);
    }
    // **逐粒质量**（流体 = 粒子质量；边界 = 2b 的面密度质量 ⇒ 那个 `sum_b`/`m_j` 用的就是它）。
    let pmass: Vec<f32> = apmass.to_vec();
    let mass = f.particle_mass();
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let pc = PacketCfg {
        n: np as u32,
        // 流体前缀（积分只跑它）；纯流体场景时 = n。
        n_fluid: n_fluid as u32,
        total,
        gmin: [gmin.x, gmin.y, gmin.z],
        inv: ginv,
        dims: [gdims.0, gdims.1, gdims.2],
        cap: 512,
        h,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        ks,
        mass,
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
        recompute_box: follow_box,
        // 跟随时给足额度（= CPU 侧同一个预算上限）：粒子散开后箱子变大，格表要装得下。
        grid_bins_cap: if follow_box { 1 << 20 } else { total },
    };
    let mut pk = match Packet::new(adapter_index, pc, &pos_flat, &vel_flat, &pmass) {
        Ok(p) => p,
        Err(e) => {
            println!("GPU 路径不可用：{e}");
            return;
        }
    };
    // ③ 逐 tick 漂移形态表：CPU 与 GPU **每 tick 各推进一步**再比位置
    //    ⇒ 一次分清"接线错"（线性增长）与"混沌放大"（随机游走/指数）。
    let mut drift: Vec<(usize, f32, f64, f32, f64)> = Vec::new(); // (t, dpos_max, dpos_mean, dv_max, ke比)
    let probe_at = [1usize, 2, 3, 5, 8, 13, 21, 30];
    let mut nan_cnt = 0usize;
    {
        let cmp = |pos_cpu: &[Vec3], vel_cpu: &[Vec3], pos_gpu: &[f32], vel_gpu: &[f32]| {
            let mut mx = 0.0f32;
            let mut sum = 0.0f64;
            let mut dv = 0.0f32;
            let mut ke_c = 0.0f64;
            let mut ke_g = 0.0f64;
            let mut bad = 0usize;
            // **只比流体前缀**：CPU 侧的 `positions()/velocities()` 就是流体，边界粒子是运动学冻结的
            // （比它没意义，且下标会越界）。
            let nf = n_fluid;
            for i in 0..nf {
                let (px, py, pz) = (pos_gpu[i * 3], pos_gpu[i * 3 + 1], pos_gpu[i * 3 + 2]);
                if !px.is_finite() || !py.is_finite() || !pz.is_finite() {
                    bad += 1;
                    continue;
                }
                let d = Vec3::new(px - pos_cpu[i].x, py - pos_cpu[i].y, pz - pos_cpu[i].z);
                let dl = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
                if dl > mx {
                    mx = dl;
                }
                sum += dl as f64;
                let dvv = Vec3::new(
                    vel_gpu[i * 3] - vel_cpu[i].x,
                    vel_gpu[i * 3 + 1] - vel_cpu[i].y,
                    vel_gpu[i * 3 + 2] - vel_cpu[i].z,
                );
                let dvl = (dvv.x * dvv.x + dvv.y * dvv.y + dvv.z * dvv.z).sqrt();
                if dvl > dv {
                    dv = dvl;
                }
                let vc = vel_cpu[i];
                let vg = Vec3::new(vel_gpu[i * 3], vel_gpu[i * 3 + 1], vel_gpu[i * 3 + 2]);
                ke_c += 0.5 * f64::from(mass) * f64::from(vc.length_squared());
                ke_g += 0.5 * f64::from(mass) * f64::from(vg.length_squared());
            }
            (bad, mx, sum / nf.max(1) as f64, dv, ke_g / ke_c.max(1e-30))
        };
        for t in 1..=ticks {
            f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
            pk.run(&pc, 1, substeps, false);
            let (pg, vg) = pk.read_state();
            let (bad, mx, mean, dv, ker) = cmp(f.positions(), f.velocities(), &pg, &vg);
            nan_cnt = nan_cnt.max(bad);
            if probe_at.contains(&t) || t == ticks {
                drift.push((t, mx, mean, dv, ker));
            }
        }
    }
    // ④ CPU 引擎的相位计时（**放在漂移表之后**：它会把状态推进，放在前面会让对照组错位一个 tick
    //    ——踩过：那样 GPU 全程滞后一 tick，漂移表里表现为"刚性平移 + 精确 g·Δt"的假信号；
    //    同一类坑还有一次：`Packet::run` 内部曾自带"预热一个 tick" ⇒ 每次调用都多推一 tick。
    f.reset_phase_us();
    let t_cpu = std::time::Instant::now();
    for _ in 0..ticks {
        f.step(dt_tick, &vxl_phys_core::interop::NoProviders);
    }
    let cpu_wall_ms = t_cpu.elapsed().as_secs_f64() * 1e3;
    let cpu_phase_ms: f64 = f.phase_us().iter().sum::<u64>() as f64 / 1e3;

    // ⑤ 稳态计时（无逐 tick 回读）。先跑一次**丢弃计时**的热身（摊掉首轮编译/首触访存）——
    //    漂移表已经跑完，这里再推进状态不影响任何对照。
    //    两条实测教训：
    //    ① 短窗受 **GPU 时钟爬升**支配（同配置能差 2×）⇒ 3 次 × 100 tick 取最小，原值一并打印；
    //    ② **每次计时前必须 `restore` 到同一状态**——计时本身会推进流体（剪切压实 ⇒ 邻域候选
    //       变多），不还原就会把"状态演化"混进"相位成本"（实测：同一全链先后差 3×）。
    let _ = pk.run(&pc, 2, substeps, false);
    let gpu_ticks = ticks.max(100);
    let (snap_pos, snap_vel) = pk.snapshot();
    let mut reps = [0.0f32; 3];
    let mut box_ms = 0.0f32;
    for r in reps.iter_mut() {
        pk.restore(&snap_pos, &snap_vel);
        let t = pk.run(&pc, gpu_ticks, substeps, false);
        *r = t.per_tick;
        // 记最后一次的"每子步箱子"累计（读数含在 `per_tick` 里，这里只把它单列出来）。
        box_ms = t.box_ms / gpu_ticks.max(1) as f32;
    }
    let gpu_ms = reps.iter().cloned().fold(f32::INFINITY, f32::min);
    // **相位消融**（位：1 分箱 / 2 扫描+占位 / 4 规范化 / 8 密度 / 16 EOS / 32 力 / 64 积分）
    // 单项单次测量有 ±2 ms 噪声 ⇒ 每项 2 次取最小（总量那一行才是可信的主读数）。
    let mut stage_ms = Vec::new();
    for (name, mask) in [
        ("分箱", 0b000_0001u32),
        ("+扫描/占位", 0b000_0011),
        ("+规范化", 0b000_0111),
        ("+密度", 0b000_1111),
        ("+EOS", 0b001_1111),
        ("+力", 0b011_1111),
        ("+积分=全链", 0b111_1111),
    ] {
        let mut best = f32::INFINITY;
        for _ in 0..2 {
            pk.restore(&snap_pos, &snap_vel);
            let ms = pk
                .run_stages(&pc, mask, gpu_ticks, substeps, false)
                .per_tick;
            best = best.min(ms);
        }
        stage_ms.push((name, best));
    }
    pk.restore(&snap_pos, &snap_vel);
    let rb_ms = pk.measure_readback_ms();
    let overflow = pk.read_overflow();

    println!(
        "== GPU 常驻管线 vs CPU 引擎（{np} 粒；{ticks} tick × {substeps} 子步；箱子 {gdims:?}）=="
    );
    println!(
        "  CPU 引擎：相位合计 **{:.1} ms**（{:.2} ms/tick）| 壁钟 {:.0} ms",
        cpu_phase_ms,
        cpu_phase_ms / ticks as f64,
        cpu_wall_ms
    );
    println!(
        "  GPU 常驻：**{gpu_ms:.2} ms/tick**（{gpu_ticks} tick；3 次原值 {:?}，取最小）",
        reps.map(|v| (v * 100.0).round() / 100.0)
    );
    println!(
        "  ⇒ 整 tick **{:.1}×**（对 CPU 相位口径）；单独量的一次状态回读+同步 = {:.2} ms ⇒ 含耦合回读 {:.2} ms/tick（{:.1}×）",
        cpu_phase_ms / ticks as f64 / f64::from(gpu_ms.max(1e-9)),
        rb_ms,
        f64::from(gpu_ms + rb_ms),
        cpu_phase_ms / ticks as f64 / f64::from((gpu_ms + rb_ms).max(1e-9))
    );
    if pc.recompute_box {
        // 诚实记账：`--box=follow` 的"每子步一次归约 + 24 B 回读往返"占整 tick 多少（已含在上面）。
        println!(
            "  ├ 其中**每子步箱子**：{:.2} ms/tick（{:.0}%，{substeps} 子步 × 归约+回读+写 uniform）",
            box_ms,
            100.0 * box_ms / gpu_ms.max(1e-9)
        );
    }
    {
        let mut line = String::from("  相位消融（ms/tick）");
        let mut prev = 0.0f32;
        for (name, ms) in &stage_ms {
            line.push_str(&format!(" | {name} {ms:.2}（+{:.2}）", ms - prev));
            prev = *ms;
        }
        println!("{line}");
    }
    println!(
        "  末态对照（**口径 B**：浮点相位不逐位 ⇒ 多 tick 后位置会漂，这里只声明同一物理、无发散）；"
    );
    println!("    NaN 粒 {nan_cnt} | 漂移形态（tick: |Δpos|max / mean / |Δv|max / 动能比）：");
    for (t, mx, mean, dv, ker) in &drift {
        println!("      t={t:>3}：{mx:.5} m / {mean:.5} m / {dv:.5} m/s / {ker:.4}");
    }
    // 速度量级（读动能比要用它标定：本探针零重力 ⇒ |v| 极小，动能比对绝对差极敏感）
    let mut vmax_cpu = 0.0f32;
    let mut vsum = 0.0f64;
    for v in f.velocities() {
        let s = v.length_squared().sqrt();
        if s > vmax_cpu {
            vmax_cpu = s;
        }
        vsum += s as f64;
    }
    println!(
        "    标定：CPU 侧 |v| max {vmax_cpu:.4} m/s / mean {:.4} m/s（零重力 ⇒ 极小 ⇒ 动能比只当「发散与否」看）",
        vsum / np as f64
    );
    println!(
        "    ⚠️ 判读：**线性增长 ⇒ 接线/口径错**；随机游走或指数 ⇒ 混沌放大（口径 B 的必然）。"
    );
    println!(
        "    网格护栏 overflow（段长超 cap 的格数，应 0）：{overflow}{}",
        if overflow == 0 {
            ""
        } else {
            " ⚠️ 非 0 ⇒ 表未规范化（箱内粒子过挤，换箱或调 cap）"
        }
    );
}
