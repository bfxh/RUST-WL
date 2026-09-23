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
