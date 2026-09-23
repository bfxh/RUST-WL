//! **"漂浮体安不安分"探针**（仪表，只打印不断言）：把探针测到的**反作用波动**
//! （`±0.8–1.6×ρVg`）落到**用户看得见**的量上——漂浮的体到底抖不抖。
//!
//! 动机：`boundary_accuracy_probe` 测的是静止体的反作用均值/波动；若波动在**动力学**里被
//! 体自身惯性积掉，那它就只是仪表噪声；若透传到体上，才值得做"两层 → 3–4 层"那一刀。
//!
//! 口径：铸装水块 + 0.5 m 盆腔（与 `fluid_boundary` 门同场景）；轻盒（300 kg/m³、半长 0.06）
//! 从水面上方落入 → 漂稳 300 tick → **窗口 180 tick** 记录体心 y、|v|、|ω| 的**峰峰值与均值**。
//! 对照 = 同场景 `add_fluid`（2a 单向，介质场浮力/阻力）。
//!
//! 跑法：`cargo test --release -p vxl-phys --test float_quiet_probe -- --ignored --nocapture`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

fn water(layers: u32) -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            boundary_layers: layers,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

/// 返回 `(y 峰峰, |v| 峰峰, |ω| 峰峰, y 均值, 反作用 f.y 峰峰/ρVg)`。
fn quiet(couple: bool, layers: u32) -> (f32, f32, f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water(layers);
    if couple {
        w.add_fluid_with_boundary_coupling(sys, &[v]);
    } else {
        w.add_fluid(sys, &[v]);
    }
    // 轻盒从水面上方落入（干净开局：不落在已有水格上）。
    let half = 0.06f32;
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(half),
        },
        Vec3::new(0.0, 1.50, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    for _ in 0..300 {
        w.step(); // 落水 + 漂稳
    }
    let win = 180usize;
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut vlo, mut vhi) = (f32::MAX, f32::MIN);
    let (mut wlo, mut whi) = (f32::MAX, f32::MIN);
    let (mut ysum, mut fsum) = (0.0f64, 0.0f64);
    let (mut flo, mut fhi) = (f32::MAX, f32::MIN);
    let expect = 1000.0 * (2.0 * half).powi(3) * 9.81;
    for _ in 0..win {
        w.step();
        let p = w.bodies.position[b as usize];
        let lv = w.bodies.linvel[b as usize].length();
        let av = w.bodies.angvel(b as usize).length();
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        vlo = vlo.min(lv);
        vhi = vhi.max(lv);
        wlo = wlo.min(av);
        whi = whi.max(av);
        ysum += p.y as f64;
        // 反作用：2b 档读流体侧；2a 档没有反作用（读体受力替代不了）⇒ 只 2b 档有意义。
        if let Some(r) = w.fluids()[0]
            .0
            .boundary_reactions()
            .iter()
            .find(|r| r.0 == b)
        {
            fsum += r.1.y as f64;
            flo = flo.min(r.1.y);
            fhi = fhi.max(r.1.y);
        }
    }
    let n = win as f64;
    let fpp = if couple { (fhi - flo) / expect } else { 0.0 };
    let _ = fsum / n;
    (yhi - ylo, vhi - vlo, whi - wlo, (ysum / n) as f32, fpp)
}

#[test]
#[ignore = "仪表（只打印）：漂浮体安分度；见文件头跑法"]
fn floating_body_quietness() {
    println!("漂浮体安分度（窗口 180 tick；轻盒 300 kg/m³、半长 0.06；水槽 = 0.5 m 盆腔）");
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>12}",
        "档", "y 峰峰/mm", "|v| 峰峰", "|ω| 峰峰", "y 均值", "f.y 峰峰/ρVg"
    );
    let (ypp, vpp, wpp, ymean, fpp) = quiet(false, 2);
    println!(
        "{:>10} {:>10.1} {:>10.3} {:>10.2} {:>10.3} {:>12.2}",
        "2a 粗档",
        ypp * 1e3,
        vpp,
        wpp,
        ymean,
        fpp
    );
    for layers in [2u32, 3, 4, 6] {
        let (ypp, vpp, wpp, ymean, fpp) = quiet(true, layers);
        println!(
            "{:>10} {:>10.1} {:>10.3} {:>10.2} {:>10.3} {:>12.2}",
            format!("2b {layers} 层"),
            ypp * 1e3,
            vpp,
            wpp,
            ymean,
            fpp
        );
    }
}

/// **自旋诊断**（仪表，只打印）：漂浮体的 `|ω|` 峰 1.22 rad/s（≈70°/s）——**对称盒不该自转**。
/// 判"是数值相位伪影还是流场真实驱动"：同场景换**体形/姿态**看自旋是否跟着变——
/// - 若换姿态（盒绕 Y 转 45°）自旋即大改 ⇒ **面栅格相位伪影**（数值）；
/// - 若换体形（球，表面无面栅格）自旋同量级 ⇒ **流场/力矩建模**（更接近物理）；
/// - 另打印**力矩均值**（系统性偏置 = 真扭矩；零均值大方差 = 噪声）与**累计偏航**（真转了多少）。
#[test]
#[ignore = "仪表（只打印）：漂浮体自旋的来源；见上注"]
fn float_spin_source_diagnosis() {
    println!("漂浮体自旋诊断（窗口 180 tick；水槽 0.5 m 盆腔、轻体 300 kg/m³）");
    println!(
        "{:>10} {:>10} {:>10} {:>12} {:>12} {:>12}",
        "体形/姿态", "y 峰峰/mm", "|v| 峰峰", "|ω| 峰峰", "|τ| 均值", "累计偏航/rad"
    );
    for (name, shape) in [
        (
            "盒 0°",
            Shape::Box {
                half: Vec3::splat(0.06),
            },
        ),
        (
            "盒 45°",
            Shape::Box {
                half: Vec3::splat(0.06),
            },
        ),
    ] {
        let rot = if name.ends_with("45°") {
            Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_4)
        } else {
            Quat::IDENTITY
        };
        let (ypp, vpp, wpp, tsum, yaw) = quiet_shaped(true, shape, rot);
        println!(
            "{name:>10} {:>10.1} {:>10.3} {:>12.3} {:>12.4} {:>12.3}",
            ypp * 1e3,
            vpp,
            wpp,
            tsum,
            yaw
        );
    }
    let (ypp, vpp, wpp, tsum, yaw) =
        quiet_shaped(true, Shape::Sphere { radius: 0.06 }, Quat::IDENTITY);
    println!(
        "{:>10} {:>10.1} {:>10.3} {:>12.3} {:>12.4} {:>12.3}",
        "球",
        ypp * 1e3,
        vpp,
        wpp,
        tsum,
        yaw
    );
}

/// 同 `quiet`，但可传体形/姿态，并额外返回（|τ| 窗口均值、累计偏航）。
fn quiet_shaped(couple: bool, shape: Shape, rot: Quat) -> (f32, f32, f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water(2);
    if couple {
        w.add_fluid_with_boundary_coupling(sys, &[v]);
    } else {
        w.add_fluid(sys, &[v]);
    }
    let b = w.add_dynamic(shape, Vec3::new(0.0, 1.50, 0.0), rot, 300.0);
    for _ in 0..300 {
        w.step(); // 落水 + 漂稳
    }
    let mut prev_yaw = 0.0f32;
    let mut unwrapped = 0.0f32;
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut vlo, mut vhi) = (f32::MAX, f32::MIN);
    let (mut wlo, mut whi) = (f32::MAX, f32::MIN);
    let mut tsum = 0.0f64;
    for _ in 0..180 {
        w.step();
        let p = w.bodies.position[b as usize];
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        let lv = w.bodies.linvel[b as usize].length();
        vlo = vlo.min(lv);
        vhi = vhi.max(lv);
        let av = w.bodies.angvel(b as usize).length();
        wlo = wlo.min(av);
        whi = whi.max(av);
        if let Some(r) = w.fluids()[0]
            .0
            .boundary_reactions()
            .iter()
            .find(|r| r.0 == b)
        {
            tsum += r.2.length() as f64;
        }
        // 累计偏航（unwrap：把每步 Δyaw 折到 (-π, π] 再累加）
        let yaw = yaw_of(w.bodies.rot(b as usize));
        let mut d = yaw - prev_yaw;
        while d > std::f32::consts::PI {
            d -= 2.0 * std::f32::consts::PI;
        }
        while d < -std::f32::consts::PI {
            d += 2.0 * std::f32::consts::PI;
        }
        unwrapped += d;
        prev_yaw = yaw;
    }
    (
        yhi - ylo,
        vhi - vlo,
        whi - wlo,
        (tsum / 180.0) as f32,
        unwrapped,
    )
}

/// 姿态的偏航角（绕 Y；用旋转矩阵的 x 轴投影求）。
fn yaw_of(q: Quat) -> f32 {
    let x = q.rotate_vec3(Vec3::X);
    x.z.atan2(x.x)
}

/// **自旋偏置来源隔离**（仪表，只打印）——上一轮已证"噪声与偏置是两个成分"，本测试只追偏置。
///
/// 三格对拍（同槽同体，窗口 180 tick）：
/// - **干槽**（无流体）：体落在槽底。若它也转 ⇒ 偏置在**刚体接触侧**（排序/法线选择）；
///   （干槽是刚体↔体素提供者接触，与 2b 无关，能一刀切开"流体 vs 刚体"。）
/// - **静态体**（marker，inv_mass=0）+ 流体：体不动 ⇒ 读**反作用力矩均值**——
///   非零即证明"流体对**静止**体也施加系统性转矩"（与体的转动反馈无关）。
/// - **自由体 + 流体**：基准（复现上一轮的 −0.142 rad/3 s）。
#[test]
#[ignore = "仪表（只打印）：自旋偏置来源；见上注"]
fn float_spin_bias_isolation() {
    println!("自旋偏置来源隔离（窗口 180 tick；盒半长 0.06、300 kg/m³）");
    // ① 干槽：无流体
    {
        let mut w = World::new(PhysConfig::default());
        let _v = tank(&mut w);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, 1.50, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        let (yaw, wpp) = run_yaw(&mut w, b as usize, 480, 180);
        println!("  ① 干槽（无流体）：累计偏航 {yaw:+.3} rad | |ω| 峰 {wpp:.3} rad/s");
    }
    // ② 静态体 + 流体：读反作用力矩均值（含 τ_y 分量）
    {
        let mut w = World::new(PhysConfig::default());
        let v = tank(&mut w);
        let sys = water(2);
        let fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
        let half = Vec3::splat(0.06);
        let s = w.add_static(
            Shape::Box { half },
            Vec3::new(0.0, 1.20, 0.0),
            Quat::IDENTITY,
        );
        let mut tsum = Vec3::ZERO;
        let mut n = 0.0f32;
        for _ in 0..480 {
            w.step();
            if let Some(r) = w.fluids()[fid]
                .0
                .boundary_reactions()
                .iter()
                .find(|r| r.0 == s)
            {
                tsum += r.2;
                n += 1.0;
            }
        }
        let tm = tsum * (1.0 / n.max(1.0));
        println!(
            "  ② 静态体 + 流体：反作用力矩均值 τ = ({:+.4}, {:+.4}, {:+.4}) N·m | |τ| {:.4}",
            tm.x,
            tm.y,
            tm.z,
            tm.length()
        );
    }
    // ③ 自由体 + 流体（基准）
    {
        let mut w = World::new(PhysConfig::default());
        let v = tank(&mut w);
        let sys = water(2);
        let _fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, 1.50, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        let (yaw, wpp) = run_yaw(&mut w, b as usize, 300, 180);
        println!("  ③ 自由体 + 流体（基准）：累计偏航 {yaw:+.3} rad | |ω| 峰 {wpp:.3} rad/s");
    }
}

/// 跑 `settle` 步再取 `win` 步窗口，返回（累计偏航、|ω| 峰）。
fn run_yaw(w: &mut World, body: usize, settle: usize, win: usize) -> (f32, f32) {
    for _ in 0..settle {
        w.step();
    }
    let mut prev = yaw_of(w.bodies.rot(body));
    let mut unwrapped = 0.0f32;
    let mut wpeak = 0.0f32;
    for _ in 0..win {
        w.step();
        let y = yaw_of(w.bodies.rot(body));
        let mut d = y - prev;
        while d > std::f32::consts::PI {
            d -= 2.0 * std::f32::consts::PI;
        }
        while d < -std::f32::consts::PI {
            d += 2.0 * std::f32::consts::PI;
        }
        unwrapped += d;
        prev = y;
        wpeak = wpeak.max(w.bodies.angvel(body).length());
    }
    (unwrapped, wpeak)
}

/// **冲击 vs 稳态**（仪表，只打印）：三格隔离实验里体都是**从 1.5 m 落下**（冲击 ≈4.4 m/s）⇒
/// 那个符号确定的偏航**可能只是"一次性冲击的不对称残差"**（角动量守恒 ⇒ 转了就留着），
/// 而不是稳态偏置。本测试把同一格改成**轻放**（起点就在平衡位附近，几乎无冲击）：
/// - 若轻放偏航≈0 ⇒ 偏置是**冲击瞬态**（改法在接触求解/冲击那一侧）；
/// - 若轻放仍有同量级偏航 ⇒ 是**稳态偏置**（改法在迭代/采样一侧）。
#[test]
#[ignore = "仪表（只打印）：偏置是冲击瞬态还是稳态；见上注"]
fn spin_bias_impact_vs_steady() {
    println!("冲击 vs 稳态（窗口 480 tick、盒半长 0.06、300 kg/m³）");
    // 干槽：重落 vs 轻放（槽底顶面 y = 1.0，体半长 0.06 ⇒ 平衡位 1.06）
    for (name, y0) in [("干槽·重落(1.50)", 1.50f32), ("干槽·轻放(1.07)", 1.07)] {
        let mut w = World::new(PhysConfig::default());
        let _v = tank(&mut w);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, y0, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        let (yaw, wpp) = run_yaw(&mut w, b as usize, 60, 480);
        println!("  {name}：累计偏航 {yaw:+.4} rad | |ω| 峰 {wpp:.3} rad/s");
    }
    // 流体：重落 vs 轻放（水面 ≈1.42 ⇒ 体浮着时体心 ≈1.46）
    for (name, y0) in [("流体·重落(1.50)", 1.50f32), ("流体·轻放(1.47)", 1.47)] {
        let mut w = World::new(PhysConfig::default());
        let v = tank(&mut w);
        let sys = water(2);
        let _fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, y0, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        let (yaw, wpp) = run_yaw(&mut w, b as usize, 300, 480);
        println!("  {name}：累计偏航 {yaw:+.4} rad | |ω| 峰 {wpp:.3} rad/s");
    }
}

/// **合格协议下的稳态自旋测量**（仪表，只打印）。
///
/// 为什么重做（2026-09-22，我自己的复核教训）：此前所有自旋读数都用"从 1.5 m 落下"的投放，
/// 而重落/轻放两格**连符号都翻** ⇒ 那些 yaw 是**投放瞬态**，不是稳态偏置。本协议三件套：
/// 1. **零冲击就位**：先从**无体**仿真测出自由水面高度，再按浮力平衡位（吃水 = ρ_body/ρ_water × 2·half）
///    把体**精确放在平衡高度**（干槽则放在"槽底顶面 + half + 0.5 mm"）⇒ 无落差、无冲击；
/// 2. **长静置**（`SETTLE` tick）；
/// 3. **窗口前自检**：体在窗口起点必须**确已静止**（`|v| < 0.01` 且 `|ω| < 0.01`），
///    否则打印 **"窗口前未静止 ⇒ 本次读数无效"**，不给数字（宁可空着，不给伪影）。
#[test]
#[ignore = "仪表（只打印）：合格协议下的稳态自旋；见上注"]
fn steady_spin_with_valid_protocol() {
    let settle_ticks: usize = 900;
    let win_ticks: usize = 480;
    for (tag, cfg) in [
        ("默认档", PhysConfig::default()),
        (
            "机制开(wakeK8+hold4)",
            PhysConfig {
                wake_gate_k: 8,
                settled_hold_iterations: 4,
                ..PhysConfig::default()
            },
        ),
    ] {
        println!(
            "== {tag} ==（静置 {settle_ticks}、窗口 {win_ticks} tick；盒半长 0.06、300 kg/m³）"
        );
        run_protocol_block(cfg, settle_ticks, win_ticks);
    }
}

/// 一档配置下的完整协议表。
fn run_protocol_block(cfg: PhysConfig, settle_ticks: usize, win_ticks: usize) {
    let _ = &cfg;
    println!(
        "{:>16} {:>9} {:>10} {:>10} {:>6} {:>9} {:>12} {:>12}",
        "格", "就位y", "起点|v|", "起点|ω|", "awake", "箱体y", "累计偏航", "τ_y 均值"
    );
    // —— 干槽（无流体）：槽底顶面 y=1.0 ⇒ 体心平衡位 1.06 ——
    for (name, rot) in [("干槽 0°", Quat::IDENTITY), ("干槽 45°", quat_y_45())] {
        let mut w = World::new(cfg.clone());
        let _v = tank(&mut w);
        let y0 = 1.06 + 0.0005;
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, y0, 0.0),
            rot,
            300.0,
        );
        report(name, &mut w, b as usize, y0, settle_ticks, win_ticks, false);
    }
    // —— 流体：先从无体仿真测自由水面 ⇒ 算平衡位 ⇒ 精确就位 ——
    for (name, shape, rot) in [
        (
            "流体 盒 0°",
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Quat::IDENTITY,
        ),
        (
            "流体 盒 45°",
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            quat_y_45(),
        ),
        ("流体 球", Shape::Sphere { radius: 0.06 }, Quat::IDENTITY),
    ] {
        // ① 无体仿真：测自由水面（中心柱 x,z∈[−0.05,0.05] 内的最高粒子）
        let (surface, fid) = {
            let mut w = World::new(cfg.clone());
            let v = tank(&mut w);
            let sys = water(2);
            let fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
            for _ in 0..300 {
                w.step();
            }
            let mut hi = 0.0f32;
            for (p, _) in w.fluids()[fid]
                .0
                .positions()
                .iter()
                .zip(w.fluids()[fid].0.velocities().iter())
            {
                if p.x.abs() < 0.05 && p.z.abs() < 0.05 {
                    hi = hi.max(p.y);
                }
            }
            (hi, fid)
        };
        // ② 按浮力平衡位精确就位（盒：吃水 0.3×0.12；球：体积等效 —— 球半径 0.06 同径，
        //    但密度 300 ⇒ 吃水按体积比 0.3 ⇒ 浸没深度 0.3×直径）
        let half_equiv = 0.06f32;
        let draft = 0.3 * 2.0 * half_equiv;
        let y0 = surface - draft + half_equiv;
        let mut w = World::new(cfg.clone());
        let v = tank(&mut w);
        let sys = water(2);
        let _f2 = w.add_fluid_with_boundary_coupling(sys, &[v]);
        let _ = fid;
        let b = w.add_dynamic(shape, Vec3::new(0.0, y0, 0.0), rot, 300.0);
        report(name, &mut w, b as usize, y0, settle_ticks, win_ticks, true);
    }
}

fn quat_y_45() -> Quat {
    Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_4)
}

/// 静置后**自检**再取窗口；未静止则报"无效"。
fn report(
    name: &str,
    w: &mut World,
    body: usize,
    y0: f32,
    settle: usize,
    win: usize,
    coupled: bool,
) {
    for _ in 0..settle {
        w.step();
    }
    let v0 = w.bodies.linvel[body].length();
    let w0 = w.bodies.angvel(body).length();
    let valid = v0 < 0.01 && w0 < 0.01;
    let mut prev = yaw_of(w.bodies.rot(body));
    let mut unwrapped = 0.0f32;
    let mut ty = 0.0f64;
    let mut n = 0.0f64;
    for _ in 0..win {
        w.step();
        let y = yaw_of(w.bodies.rot(body));
        let mut d = y - prev;
        while d > std::f32::consts::PI {
            d -= 2.0 * std::f32::consts::PI;
        }
        while d < -std::f32::consts::PI {
            d += 2.0 * std::f32::consts::PI;
        }
        unwrapped += d;
        prev = y;
        if coupled {
            if let Some(r) = w.fluids()[0]
                .0
                .boundary_reactions()
                .iter()
                .find(|r| r.0 as usize == body)
            {
                ty += r.2.y as f64;
                n += 1.0;
            }
        }
    }
    let tym = if n > 0.0 { (ty / n) as f32 } else { 0.0 };
    println!(
        "        ↳ awake={} 箱体y={:.4} sleep_timer={:.3}（判据：静置后应 awake=false 且 |v|/|ω| ≈ 0）",
        w.bodies.awake[body], w.bodies.position[body].y, w.bodies.sleep_timer[body]
    );
    if valid {
        println!("{name:>16} {y0:>9.4} {v0:>10.4} {w0:>10.4} {unwrapped:>12.4} {tym:>12.5}");
    } else {
        println!("{name:>16} {y0:>9.4} {v0:>10.4} {w0:>10.4}   ← 窗口前未静止 ⇒ 本次读数无效");
    }
}

/// **P10 逐 tick 轨迹**（仪表，只打印）：单盒静置在干槽地板上的前 40 tick + 每 100 tick 采样，
/// 回答三个判别问题：① **绕哪个轴转**（ω 分量）；② **逐 tick 注入还是自增长**（|ω| 走势）；
/// ③ **有没有"睡着又被叫醒"**（awake / sleep_timer）；并顺带打印**流形**（点数与相对体心的面内偏移
/// —— 判"接触补丁是否对称"：对称补丁的点应成 ±x / ±z 对出现）。
#[test]
#[ignore = "仪表（只打印）：P10 单盒自旋的逐 tick 轨迹；见上注"]
fn p10_single_box_spin_trace() {
    let mut w = World::new(PhysConfig::default());
    let _v = tank(&mut w);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.0605, 0.0),
        Quat::IDENTITY,
        300.0,
    ) as usize;
    println!("P10 轨迹：单盒（半长 0.06、300 kg/m³）零冲击就位在干槽地板（平衡位 1.06）");
    println!(
        "{:>5} {:>8} {:>8} {:>9} {:>9} {:>9} {:>6} {:>7} {:>7}",
        "tick", "y", "|v|", "ωx", "ωy", "ωz", "awake", "timer", "流形点"
    );
    for t in 1..=600 {
        w.step();
        if t <= 20 || t % 100 == 0 {
            let p = w.bodies.position[b];
            let v = w.bodies.linvel[b].length();
            let wv = w.bodies.angvel(b);
            let man = w.manifolds();
            let mut pts = 0usize;
            let mut offs = String::new();
            for m in man {
                if m.a as usize == b || m.b as usize == b {
                    for q in m.points.iter() {
                        pts += 1;
                        if offs.len() < 60 {
                            offs += &format!("({:+.3},{:+.3})", q.point.x - p.x, q.point.z - p.z);
                        }
                    }
                }
            }
            println!(
                "{t:>5} {y:>8.4} {v:>8.4} {wx:>9.4} {wy:>9.4} {wz:>9.4} {aw:>6} {tm:>7.3} {pts:>7}  {offs}",
                y = p.y, v = v, wx = wv.x, wy = wv.y, wz = wv.z,
                aw = w.bodies.awake[b], tm = w.bodies.sleep_timer[b], pts = pts, offs = offs
            );
        }
    }
}

/// **漂浮体"不静止"是缺陷还是随波？**（仪表，只打印）
///
/// 干槽单盒的不静止是**确定缺陷**（平地板上自旋，已由 P10 修好）。但浮在水面上的体
/// **可能只是在真实地随波起伏** ⇒ 判据不能是"|v| < 阈值"，而要问：
/// **体的运动 == 局部水的运动吗？** 本测试对同场景打印：
/// ① 体心附近的**流体平均/最大 |v|**（局部水动强度）；② 体 |v|/|ω|；
/// ③ 窗口内**体心 y 起伏**与**局部水面高度起伏**（比振幅）；
/// ④ **2a 单向**（`add_fluid`，无边界粒子反作用）对照 —— 若两档同量级 ⇒ 与 2b 无关。
#[test]
#[ignore = "P10 续：漂浮体不静止的归因（随波 vs 数值）；见上注"]
fn float_motion_vs_local_water_motion() {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water(2);
    let fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
    // 零冲击就位：先无体测水面
    let surface = {
        let mut w0 = World::new(PhysConfig::default());
        let v0 = tank(&mut w0);
        let s0 = water(2);
        let f0 = w0.add_fluid_with_boundary_coupling(s0, &[v0]);
        for _ in 0..300 {
            w0.step();
        }
        w0.fluids()[f0]
            .0
            .positions()
            .iter()
            .filter(|p| p.x.abs() < 0.05 && p.z.abs() < 0.05)
            .map(|p| p.y)
            .fold(0.0f32, f32::max)
    };
    let y0 = surface - 0.036 + 0.06;
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, y0, 0.0),
        Quat::IDENTITY,
        300.0,
    ) as usize;
    for _ in 0..900 {
        w.step();
    }
    // 窗口统计：体动 vs 局部水动
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut slo, mut shi) = (f32::MAX, f32::MIN);
    let (mut vsum, mut vmax, mut n) = (0.0f64, 0.0f32, 0.0f64);
    let (mut bvsum, mut bwsum) = (0.0f64, 0.0f64);
    let win = 480usize;
    for _ in 0..win {
        w.step();
        let p = w.bodies.position[b];
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        bvsum += w.bodies.linvel[b].length() as f64;
        bwsum += w.bodies.angvel(b).length() as f64;
        let f = &w.fluids()[fid].0;
        let mut sh = 0.0f32;
        for (q, vv) in f.positions().iter().zip(f.velocities().iter()) {
            if (q.x - p.x).abs() < 0.08 && (q.z - p.z).abs() < 0.08 {
                let sp = vv.length();
                vsum += sp as f64;
                vmax = vmax.max(sp);
                n += 1.0;
                sh = sh.max(q.y);
            }
        }
        slo = slo.min(sh);
        shi = shi.max(sh);
    }
    println!("漂浮体运动 vs 局部水动（窗口 {win} tick；盒 0°、300 kg/m³、2b 开）");
    println!(
        "  体：y 起伏 {:.1} mm | 平均 |v| {:.4} | 平均 |ω| {:.4}",
        (yhi - ylo) * 1e3,
        bvsum / win as f64,
        bwsum / win as f64
    );
    println!(
        "  邻近水（水平 ±0.08 m）：平均 |v| {:.4} | 峰 |v| {:.4} | 水面高度起伏 {:.1} mm",
        vsum / n.max(1.0),
        vmax,
        (shi - slo) * 1e3
    );
    println!("  ⇒ 若『体平均 |v| ≈ 水平均 |v|』且『体 y 起伏 ≈ 水面起伏』⇒ **随波（非缺陷）**；");
    println!("     若体 |v| 远大于水 |v| 或体起伏 ≫ 水面起伏 ⇒ **数值抖动（缺陷）**。");
    // —— 对照：**2a 单向**（`add_fluid`，无边界粒子反作用）——判定"水为什么一直不静" ——
    let mut w2 = World::new(PhysConfig::default());
    let v2 = tank(&mut w2);
    let s2 = water(2);
    let fid2 = w2.add_fluid(s2, &[v2]); // 2a：无边界粒子
    let surface2 = {
        let mut w3 = World::new(PhysConfig::default());
        let v3 = tank(&mut w3);
        let s3 = water(2);
        let f3 = w3.add_fluid(s3, &[v3]);
        for _ in 0..300 {
            w3.step();
        }
        w3.fluids()[f3]
            .0
            .positions()
            .iter()
            .filter(|p| p.x.abs() < 0.05 && p.z.abs() < 0.05)
            .map(|p| p.y)
            .fold(0.0f32, f32::max)
    };
    let b2 = w2.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, surface2 - 0.036 + 0.06, 0.0),
        Quat::IDENTITY,
        300.0,
    ) as usize;
    for _ in 0..900 {
        w2.step();
    }
    let (mut vsum2, mut vmax2, mut n2) = (0.0f64, 0.0f32, 0.0f64);
    let (mut bv2, mut bw2) = (0.0f64, 0.0f64);
    let (mut ylo2, mut yhi2) = (f32::MAX, f32::MIN);
    for _ in 0..win {
        w2.step();
        let p = w2.bodies.position[b2];
        ylo2 = ylo2.min(p.y);
        yhi2 = yhi2.max(p.y);
        bv2 += w2.bodies.linvel[b2].length() as f64;
        bw2 += w2.bodies.angvel(b2).length() as f64;
        for (q, vv) in w2.fluids()[fid2]
            .0
            .positions()
            .iter()
            .zip(w2.fluids()[fid2].0.velocities().iter())
        {
            if (q.x - p.x).abs() < 0.08 && (q.z - p.z).abs() < 0.08 {
                let sp = vv.length();
                vsum2 += sp as f64;
                vmax2 = vmax2.max(sp);
                n2 += 1.0;
            }
        }
    }
    println!(
        "  [对照 2a 单向（无边界粒子）] 体：y 起伏 {:.1} mm | 平均 |v| {:.4} | 平均 |ω| {:.4}",
        (yhi2 - ylo2) * 1e3,
        bv2 / win as f64,
        bw2 / win as f64
    );
    println!(
        "                             邻近水：平均 |v| {:.4} | 峰 |v| {:.4}",
        vsum2 / n2.max(1.0),
        vmax2
    );
}
