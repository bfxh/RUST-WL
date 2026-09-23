//! probes_a：从 main.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// PhysArena `triangularLevels(n)` 的复刻。
pub(crate) fn triangular_levels(n: usize) -> usize {
    let mut l = 1usize;
    while (l * (l + 1)) / 2 < n {
        l += 1;
    }
    l.clamp(2, 40)
}

/// PhysArena `levelsOf(n, levels)` 的复刻。
pub(crate) fn levels_of(n: usize, levels: usize) -> Vec<usize> {
    let total = (levels * (levels + 1)) / 2;
    (0..levels)
        .map(|i| ((n * (levels - i)) as f32 / total as f32).round().max(1.0) as usize)
        .collect()
}

pub(crate) fn mat(w: &mut World, friction: f32, restitution: f32) -> vxl_phys_core::MaterialId {
    w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: friction },
        restitution,
    })
}

pub(crate) fn add_box(
    w: &mut World,
    pos: Vec3,
    half: Vec3,
    m: vxl_phys_core::MaterialId,
    density: f32,
) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Box { half },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

pub(crate) fn add_sphere(
    w: &mut World,
    pos: Vec3,
    r: f32,
    m: vxl_phys_core::MaterialId,
    density: f32,
) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Sphere { radius: r },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

pub(crate) fn ground(w: &mut World, size: f32) {
    let m = mat(w, 0.7, 0.05);
    let i = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::new(size / 2.0, 1.0, size / 2.0),
        },
        Vec3::new(0.0, -1.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(i, m);
}

/// PhysArena 金字塔（210 体；层距 PITCH=1.01、箱半长 0.5）。
pub(crate) fn scene_pyramid(cfg: PhysConfig) -> World {
    scene_pyramid_mu(cfg, 0.6)
}

/// 金字塔 + **可变摩擦**（`--mu` 旋钮）：判定"堆不入睡"是否由摩擦（锚点漂移 ⇒
/// 摩擦注能的泵模式）驱动——μ=0 时若堆能停/睡，说明摩擦侧是能量源。
pub(crate) fn scene_pyramid_mu(cfg: PhysConfig, mu: f32) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, mu, 0.02);
    let n = 210usize;
    let box_half = 0.5f32;
    let pitch = box_half * 2.0 * 1.01;
    let levels = triangular_levels(n);
    let counts = levels_of(n, levels);
    for (k, &count) in counts.iter().enumerate() {
        let y = box_half + k as f32 * pitch;
        for i in 0..count {
            let x = (i as f32 - (count as f32 - 1.0) / 2.0) * pitch;
            add_box(
                &mut w,
                Vec3::new(x, y, 0.0),
                Vec3::splat(box_half),
                m,
                1000.0,
            );
        }
    }
    w
}

/// PhysArena 砖墙（200 体，错缝；行距 0.57、列距 1.22）。
pub(crate) fn scene_wall(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, 0.7, 0.01);
    let (hw, hh, hd) = (0.6f32, 0.28f32, 0.3f32);
    let per_row = 16usize;
    let rows = (200usize).div_ceil(per_row);
    let mut made = 0usize;
    'outer: for row in 0..rows {
        let offset = if row % 2 == 0 { 0.0 } else { hw };
        for i in 0..per_row {
            if made >= 200 {
                break 'outer;
            }
            let x = (i as f32 - (per_row as f32 - 1.0) / 2.0) * (hw * 2.0 + 0.02) + offset;
            let y = hh + row as f32 * (hh * 2.0 + 0.01);
            add_box(
                &mut w,
                Vec3::new(x, y, 0.0),
                Vec3::new(hw, hh, hd),
                m,
                1000.0,
            );
            made += 1;
        }
    }
    w
}

/// PhysArena 球坑（400 球点阵 + 4 面环墙）。
pub(crate) fn scene_ballpit(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let n = 400usize;
    let r = (4.0f32).max((n as f32).cbrt() * 1.4);
    ground(&mut w, 160.0);
    let mw = mat(&mut w, 0.5, 0.05);
    for i in 0..4 {
        let a = (i as f32 / 4.0) * std::f32::consts::PI * 2.0;
        let qi = w.bodies.len();
        w.add_static(
            Shape::Box {
                half: Vec3::new(r * 0.75, 1.5, 0.4),
            },
            Vec3::new(a.cos() * r, 1.5, a.sin() * r),
            vxl_phys_core::Quat {
                x: 0.0,
                y: ((a + std::f32::consts::FRAC_PI_2) / 2.0).sin(),
                z: 0.0,
                w: ((a + std::f32::consts::FRAC_PI_2) / 2.0).cos(),
            },
        );
        w.bodies.set_material(qi, mw);
    }
    let m = mat(&mut w, 0.45, 0.1);
    let side = (n as f32).cbrt().ceil() as usize;
    for i in 0..n {
        let ix = i % side;
        let iy = (i / side) % side;
        let iz = i / (side * side);
        add_sphere(
            &mut w,
            Vec3::new(
                (ix as f32 - (side as f32 - 1.0) / 2.0) * 0.7,
                0.4 + iy as f32 * 0.72,
                (iz as f32 - (side as f32 - 1.0) / 2.0) * 0.7,
            ),
            0.32,
            m,
            800.0,
        );
    }
    w
}

pub(crate) fn bench(name: &str, mut w: World, extra_steps: usize) {
    for _ in 0..WARMUP {
        w.step();
    }
    w.reset_timings();
    let mut samples = Vec::with_capacity(MEASURE);
    for _ in 0..MEASURE {
        let t0 = std::time::Instant::now();
        w.step();
        samples.push(t0.elapsed().as_secs_f64() * 1000.0);
    }
    // 长跑稳定性：窗口后再推 extra_steps，观察是否继续收敛（抖动/穿透/堆高）。
    // **离场轨迹追踪**（诊断 P5：那 ~10 个体到底是"被接触打飞"还是"自己滑出地形边缘"）：
    // 记录每个体**第一次落到 y < −5**（地形最低点约 −4）时的**上一 tick 速度**——
    // 那是它"离开台面"的瞬时状态，与它此刻在哪无关。|v| 温和（≤10 m/s）⇒ 滑出边缘后自由落体；
    // |v| 数十上百 ⇒ 接触把能量打进去了。
    let nb = w.bodies.len();
    let mut exits: Vec<(usize, f32, Vec3, Vec3)> = Vec::new(); // (步, |v|上一tick, v上一tick, 位置)
    let mut was_in: Vec<bool> = (0..nb).map(|_| true).collect();
    for step in 0..extra_steps {
        let prev_v: Vec<Vec3> = (0..nb).map(|i| w.bodies.linvel[i]).collect();
        let prev_p: Vec<Vec3> = (0..nb).map(|i| w.bodies.pose(i).0).collect();
        w.step();
        for i in 0..nb {
            if !was_in[i] || !w.bodies.is_dynamic(i) {
                continue;
            }
            if w.bodies.pose(i).0.y < -5.0 {
                was_in[i] = false;
                exits.push((step, prev_v[i].length(), prev_v[i], prev_p[i]));
            }
        }
    }
    if !exits.is_empty() {
        println!(
            "   离场事件 {} 起（y < −5 时的**上一 tick**状态）：",
            exits.len()
        );
        for (step, sp, v, p) in exits.iter().take(10) {
            println!(
                "      第 {step:5} 步  |v| {sp:8.3}  v=({:+8.3},{:+8.3},{:+8.3})  位置=({:+8.2},{:+8.2},{:+8.2})",
                v.x, v.y, v.z, p.x, p.y, p.z
            );
        }
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p50 = samples[samples.len() / 2];
    let p95 = samples[(samples.len() as f64 * 0.95) as usize];
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    let t = w.timings();
    let total_us = (t.broadphase_us
        + t.narrowphase_us
        + t.solve_us
        + t.integrate_vel_us
        + t.integrate_pos_us
        + t.fields_us
        + t.ccd_us)
        .max(1);
    let bodies = w.bodies.len();
    let dynb = (0..bodies).filter(|&i| w.bodies.is_dynamic(i)).count();
    // 接触规模 + 质量读数（跨迭代数对比用：点数、最大穿透、堆高、能耗）。
    let points: usize = w.manifolds().iter().map(|m| m.points.len()).sum();
    let mut min_sep = f32::INFINITY;
    for m in w.manifolds() {
        for p in &m.points {
            min_sep = min_sep.min(p.depth);
        }
    }
    let mut max_y = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut kin = 0.0f64;
    // **越界体剔除后的动能**（P5 ⑲ 的落地）：无边界地形（如 `trimesh`）上滚体滚出边缘后
    // 无限自由落体，`Σv²` 会被"掉出去几个体"整份主导（10 体 × 100² = 1e5）⇒ 该数就不是
    // 引擎质量度量了。这里同时给**总量**（与历史读数可比）与**有效量**（y > −5 的体），
    // 并报出越界体数——判地形场景的长跑收敛请读有效量。
    let mut kin_in_bounds = 0.0f64;
    let mut out_of_bounds = 0usize;
    let mut awake_n = 0usize;
    // 逐体明细（诊断 P5：判定"掉穿地形"还是"斜坡滚动"——见 `OPEN-PROBLEMS.md` P5）。
    let mut ys: Vec<(f32, f32, f32, f32)> = Vec::new(); // (y, x, z, |v|)
    for i in 0..bodies {
        let (pos, _) = w.bodies.pose(i);
        if w.bodies.is_dynamic(i) {
            max_y = max_y.max(pos.y);
            min_y = min_y.min(pos.y);
            let v = w.bodies.linvel[i];
            kin += (v.x * v.x + v.y * v.y + v.z * v.z) as f64;
            if pos.y > -5.0 {
                kin_in_bounds += (v.x * v.x + v.y * v.y + v.z * v.z) as f64;
            } else {
                out_of_bounds += 1;
            }
            if w.bodies.awake[i] {
                awake_n += 1;
            }
            ys.push((pos.y, pos.x, pos.z, v.length()));
        }
    }
    {
        let q = |p: f32| -> f32 {
            let k = ((ys.len() as f32 - 1.0) * p).clamp(0.0, ys.len() as f32 - 1.0) as usize;
            ys[k].0
        };
        println!(
            "   y 分布：min {:.3} | p05 {:.3} | p50 {:.3} | p95 {:.3} | max {:.3}",
            min_y,
            q(0.05),
            q(0.50),
            q(0.95),
            max_y
        );
        let mut by_y = ys.clone();
        by_y.sort_by(|a, b| a.0.total_cmp(&b.0));
        println!("   最低 5 体（y, x, z, |v|）：");
        for r in by_y.iter().take(5) {
            println!(
                "      y {:.3}  x {:.2}  z {:.2}  |v| {:.2}",
                r.0, r.1, r.2, r.3
            );
        }
        let mut by_v = ys.clone();
        by_v.sort_by(|a, b| b.3.total_cmp(&a.3));
        println!("   最快 5 体（|v|, y, x, z）：");
        for r in by_v.iter().take(5) {
            println!(
                "      |v| {:.2}  y {:.3}  x {:.2}  z {:.2}",
                r.3, r.0, r.1, r.2
            );
        }
    }
    println!(
        "{name}: {bodies} 体（动 {dynb}） p50 {p50:.3} ms  p95 {p95:.3}  mean {mean:.3}  等效 {:>6.0} FPS",
        1000.0 / mean
    );
    println!(
        "  相位均值/步：宽相 {:>7.1} µs（{:>4.0}%） 窄相 {:>7.1} µs（{:>4.0}%） 求解 {:>8.1} µs（{:>4.0}%） 积分 {:>6.1} µs（{:>4.0}%） CCD {:>5.1} µs  力场 {:>5.1} µs",
        t.broadphase_us as f64 / MEASURE as f64,
        100.0 * t.broadphase_us as f64 / total_us as f64,
        t.narrowphase_us as f64 / MEASURE as f64,
        100.0 * t.narrowphase_us as f64 / total_us as f64,
        t.solve_us as f64 / MEASURE as f64,
        100.0 * t.solve_us as f64 / total_us as f64,
        (t.integrate_vel_us + t.integrate_pos_us) as f64 / MEASURE as f64,
        100.0 * (t.integrate_vel_us + t.integrate_pos_us) as f64 / total_us as f64,
        t.ccd_us as f64 / MEASURE as f64,
        t.fields_us as f64 / MEASURE as f64,
    );
    println!(
        "  规模/质量：流形 {} 个、接触点 {points} 个（{:.1} 点/流形）  最大穿透 {:+.4} m  堆顶 y={max_y:.3}  末态动能 Σv²={kin:.4}（**有效 {kin_in_bounds:.4}**，越界 {out_of_bounds} 体）  末态清醒 {awake_n}/{dynb}",
        w.manifolds().len(),
        points as f64 / w.manifolds().len().max(1) as f64,
        min_sep,
    );
    let p = w.narrow.probe_stats();
    let per = |v: u64| v as f64 / MEASURE as f64;
    println!(
        "  窄相计数/步：裁剪 {:.0} 次（内层迭代 {:.1}/次、插值 {:.2}/次、候选点 {:.2}/次）  裁剪多边形峰值 {} 顶点",
        per(p.0),
        per(p.1) / per(p.0).max(1.0),
        per(p.2) / per(p.0).max(1.0),
        per(p.3) / per(p.0).max(1.0),
        p.4,
    );
    let lp = w.solver.last_points;
    println!(
        "  点承载力（**上一次解算调用**＝一个子步）：被解算接触点 {} 个，其中法向冲量≈0 的 {} 个（{:.1}%）",
        lp.0,
        lp.1,
        100.0 * lp.1 as f64 / lp.0.max(1) as f64,
    );
    let (we, wf, wm) = vxl_phys_solver::warm_match_stats_take();
    let wt = (we + wf + wm).max(1) as f64;
    println!(
        "  warm 匹配分支（全程累计）：精确特征 {}（{:.1}%） 近邻回退 {}（{:.1}%） 未匹配 {}（{:.1}%）\
         ——精确率高 ⇒ 流形跨帧稳定（§9/§10 的\"裁剪产物\"归因不成立）",
        we,
        100.0 * we as f64 / wt,
        wf,
        100.0 * wf as f64 / wt,
        wm,
        100.0 * wm as f64 / wt,
    );
    let (bs, bc, bh, bsame) = vxl_phys_solver::warm_fallback_kind_take();
    let bt = (bs + bc + bh + bsame).max(1) as f64;
    println!(
        "  回退命中的成因（占回退）：侧别翻转 {}（{:.1}%） 裁剪路变 {}（{:.1}%） 哈希变 {}（{:.1}%） 特征同 {}（{:.1}%）",
        bs,
        100.0 * bs as f64 / bt,
        bc,
        100.0 * bc as f64 / bt,
        bh,
        100.0 * bh as f64 / bt,
        bsame,
        100.0 * bsame as f64 / bt,
    );
    let (d_island, d_solve, d_sleep, _) = w.solver.last_phase_us;
    let dd = w.solver.last_detail_us;
    let dd_sum = (dd[0] + dd[1] + dd[2] + dd[3]).max(1) as f64;
    if dd[1] + dd[2] + dd[3] == 0 {
        println!(
            "  ⚠️ 逐岛三段细分已关（`vxl_phys_solver::ISLAND_SEG_PROBE` = {}）⇒ 下面「约束构建/热启动/迭代」三列恒 0：这是**默认**（该探针每岛读 6 次钟，10 万岛场景实测 ≈26–30 ms/tick）；需要细分就把它置 true 重编。",
            vxl_phys_solver::ISLAND_SEG_PROBE
        );
    }
    println!(
        "  求解细分/步：建岛 {:>6.1} µs  约束构建 {:>6.1} µs（{:>4.0}%）  热启动预施加 {:>6.1} µs（{:>4.0}%）  迭代扫掠 {:>6.1} µs（{:>4.0}%）",
        dd[0] as f64 / MEASURE as f64,
        dd[1] as f64 / MEASURE as f64,
        100.0 * dd[1] as f64 / dd_sum,
        dd[2] as f64 / MEASURE as f64,
        100.0 * dd[2] as f64 / dd_sum,
        dd[3] as f64 / MEASURE as f64,
        100.0 * dd[3] as f64 / dd_sum,
    );
    println!(
        "  求解器内部：岛构建 {:>7.1} µs（{:>4.0}%） 迭代 {:>8.1} µs（{:>4.0}%） 休眠 {:>6.1} µs（{:>4.0}%）",
        d_island as f64 / MEASURE as f64,
        100.0 * d_island as f64 / d_island.max(1) as f64,
        d_solve as f64 / MEASURE as f64,
        100.0 * d_solve as f64 / (d_island + d_solve + d_sleep).max(1) as f64,
        d_sleep as f64 / MEASURE as f64,
        100.0 * d_sleep as f64 / d_island.max(1) as f64,
    );
}
