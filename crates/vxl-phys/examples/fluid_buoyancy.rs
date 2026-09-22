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
    // 追加 `2b` 参数 ⇒ 再跑一段 **Akinci 两层边界粒子（2b 双向耦合）** 对照：
    // 同槽、同四密度，但体从**水面上方**落入，边界粒子每 tick 按体重建。
    // （2b 下体不能**直接**落在已有水格上：边界粒子与流体粒子近同位 ⇒ ρ 爆 ⇒
    //  CFL 把水抛出去。那是初值重叠，不是耦合失稳，见 `tests/fluid_boundary.rs` ②。）
    let two_b = std::env::args().any(|a| a == "2b");

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
    let t_2a = std::time::Instant::now();
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
    let ms_2a = t_2a.elapsed().as_secs_f64() * 1e3 / ticks as f64;

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
    println!("2a 档耗時 {ms_2a:.3} ms/tick");

    if !two_b {
        println!("（加参数 `2b` 可再跑 Akinci 两层边界粒子的双向耦合对照）");
        return;
    }

    // ================= 2b：Akinci 两层边界粒子（刚体 → 液体，双向） =================
    // 同一水槽/四密度；体从水面上方落入（干净开局）。浮力**只**来自边界粒子的压力
    // 反作用（覆盖的体 2a 已让位）⇒ 这一档同时是"双向耦合能产生浮力"的演示。
    let mut w2 = World::new(PhysConfig::default());
    let mut vol2 =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol2.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol2.set(ix, 2, iz, true);
        }
    }
    let voxel2 = w2.add_voxel(vol2);
    let sys2 = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    let fluid2 = w2.add_fluid_with_boundary_coupling(sys2, &[voxel2]);

    let mut ids2 = Vec::new();
    let y_start = 1.52f32; // 水面 ≈ 1.42、堰顶 1.50 ⇒ 体从水面之上落入
    for (k, &rho) in densities.iter().enumerate() {
        let id = w2.add_dynamic(
            Shape::Box { half },
            Vec3::new(xs[k], y_start, zs[k]),
            Quat::IDENTITY,
            rho,
        );
        ids2.push(id);
    }
    println!("----");
    println!("2b（Akinci 两层边界粒子，双向耦合）：同槽同上四密度，起点 y = {y_start:.2}");
    let t_2b = std::time::Instant::now();
    for t in 1..=ticks {
        w2.step();
        if t % 60 == 0 || t == ticks {
            let mut line = format!("t={t:3}:");
            for (k, &id) in ids2.iter().enumerate() {
                let p = w2.bodies.position[id as usize];
                let v = w2.bodies.linvel[id as usize];
                let wv = w2.bodies.angvel(id as usize);
                line += &format!(
                    "  ρ{:<4.0} y {:.3} |v| {:.2} |ω| {:.1}",
                    densities[k],
                    p.y,
                    v.length(),
                    wv.length()
                );
            }
            println!("{line}");
        }
    }
    let ms_2b = t_2b.elapsed().as_secs_f64() * 1e3 / ticks as f64;
    let nb = w2.fluids()[fluid2].0.boundary_count();
    let np = w2.fluids()[fluid2].0.len();
    let h2 = w2.health();
    println!("----");
    println!("对照（末态吃水位 y；起点 2a {y0:.2} / 2b {y_start:.2}）：");
    let mut ok_light = true;
    let mut heavy_spike = false;
    for (k, &id) in ids.iter().enumerate() {
        let a = w.bodies.position[id as usize].y;
        let b = w2.bodies.position[ids2[k] as usize].y;
        let spin = w2.bodies.angvel(ids2[k] as usize).length();
        if densities[k] < 1000.0 {
            // 轻体：必须浮在水面（本档实测 ~1.24–1.28）。
            ok_light &= b > 1.20;
            println!(
                "  ρ{:<4.0}  2a y {a:.3} | 2b y {b:.3}  浮 {}",
                densities[k],
                if b > 1.20 { "✅" } else { "❌" }
            );
        } else {
            // 重体：**已知适用边界**（体 ≲ 2h 时离散挤压出尖峰 ⇒ 被推飞，M1-EXIT §4 记账），
            // 不计入通过判据，但必须显式报出来（别把已知问题藏进"沉/浮"两个字）。
            // "沉"必须是**沉到盆底附近**（1.03–1.20）：掉出世界（y ≪ 1）不算沉。
            let sunk = (1.03..1.20).contains(&b);
            heavy_spike |= !sunk;
            println!(
                "  ρ{:<4.0}  2a y {a:.3} | 2b y {b:.3}  {}（|ω| {:.1}）{}",
                densities[k],
                if sunk { "沉 ✅" } else { "未沉 ⚠️" },
                spin,
                if sunk {
                    ""
                } else {
                    "  ← 已知边界：≲2h 重体被离散挤压尖峰推飞"
                }
            );
        }
    }
    println!(
        "  ρ700 吃水比 ρ300 深（2b）：{} | 健康：NaN {} | 深穿透 {}",
        if w2.bodies.position[ids2[1] as usize].y < w2.bodies.position[ids2[0] as usize].y {
            "✅"
        } else {
            "❌"
        },
        h2.nan_bodies,
        h2.deep_penetrations
    );
    println!(
        "造价：边界粒子 {nb} / 流体粒子 {np}（{:.2}×）| 耗时 2a {ms_2a:.3} ｜ 2b {ms_2b:.3} ms/tick",
        nb as f32 / np as f32
    );
    println!(
        "{}",
        if ok_light && h2.nan_bodies == 0 && h2.deep_penetrations == 0 && !heavy_spike {
            "✅ 2b 演示通过：浮力由边界粒子的压力**反作用**给出（覆盖体 2a 已让位，无叠加）"
        } else if ok_light && h2.nan_bodies == 0 && h2.deep_penetrations == 0 {
            "✅ 2b 轻体通过（浮力/吃水序正确）；⚠️ 重体见上表已知边界（≲2h 离散挤压尖峰，记账在 M1-EXIT §4）"
        } else {
            "❌ 2b 演示不通过（见上表）"
        }
    );
}
