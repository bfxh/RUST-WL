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
