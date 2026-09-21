//! **浮力/阻力演示（2a 施力侧）**：`ROUTE.md` §4「刚体↔液体」格的**示例**半边
//! （测试半边是 `crates/vxl-phys/tests/fluid_coupling.rs`；本文件与它共用场景口径）。
//!
//! 运行：`cargo run --release -p vxl-phys --example fluid_buoyancy [ticks]`
//! 默认 240 tick（4 s）。**打印版**：每 30 tick 打一次四个盒子的高度与速度，
//! 收尾给"浮/沉"判定表。要看可视转储（VXLD/GIF）请照 `showcase.rs` 加**刚体记录**段
//! （`dam_break` 那份是"零刚体"版，不能直接复用）。
//!
//! 物理：密度 300 / 700 的盒子应**浮**（分别约 30% / 70% 吃水），1200 / 2000 的应**沉**到盆底。
//! 浮力 = `−g·ρ_med·V·frac_sub`（`frac_sub` 取体心 + 4 个水平表面点的占用率均值），
//! 另有二次阻力 `−½·ρ·Cd·A·|v_rel|·v_rel`（与喷溅介质同一式）。
//!
//! ⚠️ **场景必须按"铸装"口径搭**（`PLAN-0.3.md` §4.2 的负面结论）：水块按**沉降后几何**
//! 直接就位——`[8,8,8]@0.05` 对 0.5 m 盆腔（与门禁/`showcase` 同款量级）。任何"带落差入盆"
//! 都会触发 WCSPH 驻留瞬态"顶心喷泉"，把水抛出去，演示与断言都会失效。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(240);

    let mut w = World::new(PhysConfig::default());

    // ---- 水槽：5×5 格（外沿 2.5 m）地板 + 中心一格围堰 ⇒ 内腔 0.5×0.5 m ----
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue; // 腔体
            }
            vol.set(ix, 2, iz, true);
        }
    }
    let voxel_id = w.add_voxel(vol);

    // ---- 水：铸装近平衡（足印 0.4、深 0.4；摊到 0.5 腔后水深 ~0.26 m）----
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    let _fluid = w.add_fluid(sys, &[voxel_id]);

    // ---- 四个盒子：密度 300 / 700（应浮）与 1200 / 2000（应沉）----
    let half = Vec3::splat(0.06);
    let densities = [300.0f32, 700.0, 1200.0, 2000.0];
    let xs = [-0.12f32, 0.12, -0.12, 0.12];
    let zs = [-0.12f32, -0.12, 0.12, 0.12];
    let mut ids = Vec::new();
    for (k, &rho) in densities.iter().enumerate() {
        let id = w.add_dynamic(
            Shape::Box { half },
            Vec3::new(xs[k], 1.12, zs[k]),
            Quat::IDENTITY,
            rho,
        );
        ids.push(id);
    }

    println!(
        "流体↔刚体（2a）：水槽内腔 0.5 m、水深 ~0.26 m；四盒密度 {:?}（半长 0.06）| {ticks} tick",
        densities
    );
    let y0 = 1.12f32;
    for t in 1..=ticks {
        w.step();
        if t % 30 == 0 || t == ticks {
            let mut line = format!("t={t:3}:");
            for (k, &id) in ids.iter().enumerate() {
                let p = w.bodies.position[id as usize];
                let v = w.bodies.linvel[id as usize];
                line += &format!(
                    "  ρ{:<4.0} y {:.3} |v| {:.2}",
                    densities[k],
                    p.y,
                    v.length()
                );
            }
            println!("{line}");
        }
    }

    // ---- 判定表（浮/沉），与物理预期对照 ----
    let h = w.health();
    println!("----");
    println!("末态（起点 y = {y0:.3}；地板顶 y = 1.000）：");
    let mut ok = true;
    for (k, &id) in ids.iter().enumerate() {
        let y = w.bodies.position[id as usize].y;
        let expect_float = densities[k] < 1000.0;
        let (what, good) = if expect_float {
            ("浮（应在水面上）", y > y0 - 0.02)
        } else {
            ("沉（应在盆底附近）", y < y0 - 0.05)
        };
        ok &= good;
        println!(
            "  ρ{:<4.0} → y {y:.3}  {what} {}",
            densities[k],
            if good { "✅" } else { "❌" }
        );
    }
    println!(
        "健康：NaN {} | 深穿透 {}",
        h.nan_bodies, h.deep_penetrations
    );
    // **物理一致性**：同为浮体时，密度大的应浮得更低（吃水更深）。
    let (y300, y700) = (
        w.bodies.position[ids[0] as usize].y,
        w.bodies.position[ids[1] as usize].y,
    );
    let ordered = y700 < y300;
    println!(
        "  密度序：ρ700 {y700:.3} < ρ300 {y300:.3} ⇒ 吃水随密度增大 {}",
        if ordered { "✅" } else { "❌" }
    );
    println!(
        "{}",
        if ok && ordered && h.nan_bodies == 0 && h.deep_penetrations == 0 {
            "✅ 演示通过：轻者浮、重者沉、吃水随密度单调（浮力+阻力按物理量纲生效）"
        } else {
            "❌ 演示不通过（见上表）"
        }
    );
}
