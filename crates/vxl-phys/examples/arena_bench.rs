//! **离线对比台场景复刻 + 相位剖析**（不依赖浏览器/前端）。
//!
//! 用 PhysArena 的同一批场景（同体数/同尺寸/同出生位姿/同材质）在原生侧跑
//! 固定步长基准，输出每步 p50/p95 与**相位分解**（宽相/窄相/求解/积分/CCD），
//! 用于引擎侧优化迭代。
//!
//! 运行：`cargo run --release -p vxl-phys --example arena_bench [场景] [--iters N]`
//! 场景：pyramid（默认）/ wall / ballpit / all

use vxl_phys::*;
use vxl_phys_core::{FrictionModel, Material, PhysConfig, Shape, Vec3};

const WARMUP: usize = 30;
const MEASURE: usize = 180;

/// PhysArena `triangularLevels(n)` 的复刻。
fn triangular_levels(n: usize) -> usize {
    let mut l = 1usize;
    while (l * (l + 1)) / 2 < n {
        l += 1;
    }
    l.clamp(2, 40)
}

/// PhysArena `levelsOf(n, levels)` 的复刻。
fn levels_of(n: usize, levels: usize) -> Vec<usize> {
    let total = (levels * (levels + 1)) / 2;
    (0..levels)
        .map(|i| ((n * (levels - i)) as f32 / total as f32).round().max(1.0) as usize)
        .collect()
}

fn mat(w: &mut World, friction: f32, restitution: f32) -> vxl_phys_core::MaterialId {
    w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: friction },
        restitution,
    })
}

fn add_box(w: &mut World, pos: Vec3, half: Vec3, m: vxl_phys_core::MaterialId, density: f32) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Box { half },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

fn add_sphere(w: &mut World, pos: Vec3, r: f32, m: vxl_phys_core::MaterialId, density: f32) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Sphere { radius: r },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

fn ground(w: &mut World, size: f32) {
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
fn scene_pyramid(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, 0.6, 0.02);
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
fn scene_wall(cfg: PhysConfig) -> World {
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
fn scene_ballpit(cfg: PhysConfig) -> World {
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

fn bench(name: &str, mut w: World, extra_steps: usize) {
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
    for _ in 0..extra_steps {
        w.step();
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
    let mut kin = 0.0f64;
    let mut awake_n = 0usize;
    for i in 0..bodies {
        let (pos, _) = w.bodies.pose(i);
        if w.bodies.is_dynamic(i) {
            max_y = max_y.max(pos.y);
            let v = w.bodies.linvel[i];
            kin += (v.x * v.x + v.y * v.y + v.z * v.z) as f64;
            if w.bodies.awake[i] {
                awake_n += 1;
            }
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
        "  规模/质量：流形 {} 个、接触点 {points} 个（{:.1} 点/流形）  最大穿透 {:+.4} m  堆顶 y={max_y:.3}  末态动能 Σv²={kin:.4}  末态清醒 {awake_n}/{dynb}",
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
    let (d_island, d_solve, d_sleep, _) = w.solver.last_phase_us;
    let dd = w.solver.last_detail_us;
    let dd_sum = (dd[0] + dd[1] + dd[2] + dd[3]).max(1) as f64;
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

/// **摩擦保真度**：静置地面上的盒以 12 m/s 滑行，打印速度轨迹与减速比
/// （实测减速度 / 理论 μ·g）。物理上 μ=0.7 时减速度 ≈6.9 m/s²（0.115 m/s/tick）。
/// 用途：定位/验证「摩擦界含偏置冲量」（EXPERIMENTS 2026-09-15 第二轮）。
fn scene_slide(cfg: PhysConfig, ticks: usize) {
    let dt = cfg.dt;
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, 0.7, 0.0);
    // 半 0.4 的盒，底面贴地（y=0.4），初速 +12 m/s。
    add_box(
        &mut w,
        Vec3::new(0.0, 0.4, 0.0),
        Vec3::splat(0.4),
        m,
        1000.0,
    );
    w.bodies.set_linvel(1, Vec3::new(12.0, 0.0, 0.0));
    let mu = 0.7f32;
    let ideal = mu * 9.81 * dt; // 每 tick 理论减速（m/s）
    println!("slide: μ={mu} 理论减速 {ideal:.4} m/s/tick；实测轨迹（tick: vx / 累计比）");
    let mut last_v = 12.0f32;
    let mut last_t = 0usize;
    for t in 0..ticks {
        w.step();
        if t == 0 || (t + 1) % 10 == 0 {
            let v = w.bodies.linvel[1].x;
            let dt_ticks = (t + 1 - last_t) as f32;
            let measured = (last_v - v) / dt_ticks;
            let ratio = measured / ideal;
            println!(
                "  t={:>4}  vx={v:>7.3}  区间减速 {measured:>7.4} m/s/tick  比理论 ×{ratio:>6.2}",
                t + 1
            );
            last_v = v;
            last_t = t + 1;
        }
    }
}

/// **接触响应刚度**：12 m/s 的盒冲向静态薄墙，打印"最小表面间距"轨迹——
/// 物理上盒面应先贴近到 skin 量级再被拦住；若停在明显间距外，说明去穿透
/// （ERP）目标速度把接触做成了弹簧/保险杠（子步把 dt 减半时 ERP 会变硬）。
/// 指标：末态表面间距（期望 ≈ skin 0.02）与"是否发生真实接近"。
fn scene_approach(cfg: PhysConfig) {
    let dt = cfg.dt;
    let sub = cfg.substeps.max(1) as f32;
    let mut w = World::new(cfg);
    // 静态薄墙：x ∈ [0, 0.1]，高度足够。
    let mw = mat(&mut w, 0.5, 0.0);
    let wall = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.05, 3.0, 3.0),
        },
        Vec3::new(0.05, 0.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(wall, mw);
    // 仅静态墙 + 弹体（无地板）：排除摩擦/支撑干扰，直测接触响应。
    let mb = mat(&mut w, 0.3, 0.0);
    add_box(
        &mut w,
        Vec3::new(-3.0, 0.0, 0.0),
        Vec3::splat(0.4),
        mb,
        1000.0,
    );
    w.bodies.set_linvel(1, Vec3::new(12.0, 0.0, 0.0));
    println!(
        "approach: 12 m/s 撞 0.1 m 薄墙（substeps={sub}，子步 dt={:.5}）；表面间距轨迹：",
        dt / sub
    );
    let mut min_gap = f32::INFINITY;
    for t in 0..90 {
        w.step();
        let x = w.bodies.position[1].x;
        let gap = 0.0 - (x + 0.4); // 墙左面(x=0) − 盒右面
        min_gap = min_gap.min(gap);
        if t < 20 || (t + 1) % 15 == 0 {
            println!(
                "  t={:>3}  x={x:>7.3}  vx={:>7.3}  表面间距 {gap:>7.4}",
                t + 1,
                w.bodies.linvel[1].x
            );
        }
    }
    println!("  最小表面间距 {min_gap:.4} m（≈ 0 表示贴合；>0.05 说明被挡在弹性层外）");
}

/// **体素地面落体保真度**：盒从 2.5 m 落到体素地板（顶面 y=1.0），打印
/// y/vy 轨迹与末态读数——期望：贴住顶面（y ≈ 1.5 + skin 余量）、不穿地、最终入睡。
/// 用途：provider 接触语义（检测带 vs 真实深度）的回归基准。
fn scene_voxel_land(cfg: PhysConfig) {
    let mut w = World::new(cfg);
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
    w.add_voxel(vol);
    let m = mat(&mut w, 0.6, 0.0);
    add_box(
        &mut w,
        Vec3::new(0.0, 2.5, 0.0),
        Vec3::splat(0.5),
        m,
        1000.0,
    );
    println!("voxel_land: 盒（半 0.5）落到体素顶面 y=1.0（期望静置 y≈1.5）；轨迹：");
    let mut min_y = f32::INFINITY;
    for t in 0..180 {
        w.step();
        let y = w.bodies.position[1].y;
        min_y = min_y.min(y);
        if t < 8 || (t + 1) % 15 == 0 {
            println!(
                "  t={:>3}  y={y:>8.3}  vy={:>8.3}  流形={}",
                t + 1,
                w.bodies.linvel[1].y,
                w.manifolds().len()
            );
        }
        // 落体窗口内打印流形细节（诊断 depth 符号与求解器可见性）
        if (24..40).contains(&t) {
            for mf in w.manifolds() {
                let ds: Vec<String> = mf
                    .points
                    .iter()
                    .map(|p| format!("{:.4}", p.depth))
                    .collect();
                println!(
                    "    [流形] a={} b={} 法线=({:.2},{:.2},{:.2}) 点数={} 深度=[{}]",
                    mf.a,
                    mf.b,
                    mf.normal.x,
                    mf.normal.y,
                    mf.normal.z,
                    mf.points.len(),
                    ds.join(", ")
                );
            }
        }
    }
    let y = w.bodies.position[1].y;
    let h = w.health();
    println!(
        "  末态 y={y:.3}（期望 1.42–1.60）  最低 y={min_y:.3}（<-1 即穿地）  清醒={}  干净={}",
        w.bodies.awake[1],
        h.is_clean()
    );
}

/// **provider 撞击保真度**：12 m/s 的盒撞 1 格厚体素墙（+ 体素地板），
/// 打印撞击窗口的流形（法线/深度）与体速轨迹。期望：贴近到 skin 量级被拦、
/// 且"解算前接近速度"≥ 阈值（破坏管线可判冲击）。
fn scene_wall_provider(cfg: PhysConfig) {
    let mut w = World::new(cfg);
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 0.5, 4.0)); // 地板 1 层
    vol.fill_box(Vec3::new(0.0, 0.5, -2.0), Vec3::new(0.5, 2.5, 2.0)); // 墙 1 格厚
    w.add_voxel(vol);
    let m = mat(&mut w, 0.3, 0.0);
    add_box(
        &mut w,
        Vec3::new(-3.0, 1.0, 0.0),
        Vec3::splat(0.4),
        m,
        1000.0,
    );
    w.bodies.set_linvel(1, Vec3::new(12.0, 0.0, 0.0));
    println!("wall_provider: 12 m/s 撞 1 格厚体素墙（x∈[0,0.5]）；轨迹：");
    for t in 0..40 {
        w.step();
        let x = w.bodies.position[1].x;
        let gap = 0.0 - (x + 0.4);
        if (9..30).contains(&t) || t < 3 {
            println!(
                "  t={:>3}  x={x:>7.3}  vx={:>7.3}  右面间距 {gap:>7.4}  流形={}",
                t + 1,
                w.bodies.linvel[1].x,
                w.manifolds().len()
            );
            if (12..16).contains(&t) {
                for mf in w.manifolds() {
                    let ds: Vec<String> = mf
                        .points
                        .iter()
                        .map(|p| format!("{:.4}", p.depth))
                        .collect();
                    println!(
                        "      [流形] 法线=({:.2},{:.2},{:.2}) 深度=[{}]",
                        mf.normal.x,
                        mf.normal.y,
                        mf.normal.z,
                        ds.join(", ")
                    );
                }
            }
        }
    }
}

/// PhysArena 三角网地形（90 m × 24 段高度场 + 200 体混合几何）复刻：
/// 定位「三角网原生路径」的相位分解（arena 实测 22 ms，其它引擎 0.36–11 ms）。
fn scene_trimesh_terrain(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let size = 90.0f32;
    let seg = 24usize;
    let step = size / seg as f32;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let h = |x: f32, z: f32| {
        (x * 0.13).sin() * 1.6 + (z * 0.11).cos() * 1.4 + ((x + z) * 0.05).sin() * 1.1
    };
    for iz in 0..=seg {
        for ix in 0..=seg {
            let x = -size / 2.0 + ix as f32 * step;
            let z = -size / 2.0 + iz as f32 * step;
            verts.push(Vec3::new(x, h(x, z), z));
        }
    }
    let row = (seg + 1) as u32;
    for iz in 0..seg as u32 {
        for ix in 0..seg as u32 {
            let a = iz * row + ix;
            let b = a + 1;
            let c = a + row;
            let d = c + 1;
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(verts, tris));
    let mut r = rng_lcg(131);
    let n = 200usize;
    let m = mat(&mut w, 0.6, 0.05);
    for i in 0..n {
        let x = (r() - 0.5) * 30.0;
        let z = (r() - 0.5) * 30.0;
        let y = 14.0 + (i % 12) as f32 * 1.2 + r() * 0.5;
        match i % 3 {
            0 => add_sphere(&mut w, Vec3::new(x, y, z), 0.36, m, 1000.0),
            1 => add_box(&mut w, Vec3::new(x, y, z), Vec3::splat(0.32), m, 1000.0),
            _ => {
                // 复刻 arena 的 cylinder→凸包 降级（16 段双环 + 端心 ≈34 顶点）：
                // 验证「provider 逐顶点查询」的成本随顶点数线性增长。
                let (r, hh) = (0.3f32, 0.34f32);
                let mut pts: Vec<Vec3> = Vec::new();
                for ring in [-1.0f32, 1.0] {
                    for k in 0..16 {
                        let a = k as f32 / 16.0 * std::f32::consts::TAU;
                        pts.push(Vec3::new(a.cos() * r, ring * hh, a.sin() * r));
                    }
                }
                pts.push(Vec3::new(0.0, -hh, 0.0));
                pts.push(Vec3::new(0.0, hh, 0.0));
                let hull = w.add_hull(pts);
                let bi = w.spawn_hull_body(
                    hull,
                    Vec3::new(x, y, z),
                    vxl_phys_core::Quat::IDENTITY,
                    1000.0,
                );
                w.bodies.set_material(bi as usize, m);
            }
        }
    }
    w
}

/// 与 PhysArena `sunnyrng` 等价的线性同余（复刻场景用；非确定性要求，仅占位）。
fn rng_lcg(seed: u32) -> impl FnMut() -> f32 {
    let mut a = seed;
    move || {
        a = a.wrapping_mul(1664525).wrapping_add(1013904223);
        (a >> 8) as f32 / 16777216.0
    }
}

/// **网格静置保真度**：盒轻放到水平三角网（y=0）上，打印静置高度轨迹。
/// 期望：停在 **面 + skin 量级**（≈0.52）且 600 步不持续下沉——薄壳语义的
/// 「单向屏障」要求体一旦压进面内侧就被顶出（符号距离修正的直接检验）。
fn scene_mesh_land(cfg: PhysConfig) {
    let mut w = World::new(cfg);
    let n = 10usize;
    let step = 1.0f32;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            verts.push(Vec3::new(
                -(n as f32) / 2.0 + ix as f32 * step,
                0.0,
                -(n as f32) / 2.0 + iz as f32 * step,
            ));
        }
    }
    let row = (n + 1) as u32;
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * row + ix;
            let b = a + 1;
            let c = a + row;
            let d = c + 1;
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(verts, tris));
    let m = mat(&mut w, 0.6, 0.0);
    add_box(
        &mut w,
        Vec3::new(0.0, 0.52, 0.0),
        Vec3::splat(0.5),
        m,
        1000.0,
    );
    println!("mesh_land: 盒（半 0.5）轻放水平网格（面 y=0；期望静置 y≈0.52）：");
    let mut min_y = f32::INFINITY;
    for t in 0..600 {
        w.step();
        let y = w.bodies.position[1].y;
        min_y = min_y.min(y);
        if t < 5 || (t + 1) % 60 == 0 {
            println!(
                "  t={:>4}  y={y:>8.4}  vy={:>8.4}",
                t + 1,
                w.bodies.linvel[1].y
            );
        }
    }
    println!(
        "  末态 y={:.4}（≈0.52 为正常；持续下降 = 单向屏障失效）  最低 y={min_y:.4}",
        w.bodies.position[1].y
    );
}

/// PhysArena `jointProbe` 的逐字复刻（5 种关节；几何/容差/判据同源）。
///
/// 探针结构：静态锚块（半 0.2，y=10）+ 悬挂盒（半 0.4，密度 500，间隔 0.25
/// 避免两端接触），锚点在 t=0 **恰好重合**（起始违例测的是追赶瞬态，不是约束
/// 保持力）。判据与 arena 同口径：worldPoint 展开后的锚点分离 ≤ 容差
/// （固定 0.3、其余 0.2；距离走 |d − rest| ≤ 0.35）。240 步。
fn scene_joint_probes(cfg: PhysConfig) {
    const ANCHOR_HALF: f32 = 0.2;
    const BODY_HALF: f32 = 0.4;
    let anchor_y = 10.0f32;
    let attach_y = anchor_y - ANCHOR_HALF;
    let a_local = Vec3::new(0.0, -ANCHOR_HALF, 0.0);
    let gap = 0.25f32;
    let rest = 1.2f32;

    // 与 arena `worldPoint` 同一数学：体局部点 → 世界系。
    let world_point = |p: Vec3, q: vxl_phys_core::Quat, local: Vec3| -> Vec3 {
        let (x, y, z, w) = (q.x, q.y, q.z, q.w);
        let tx = 2.0 * (y * local.z - z * local.y);
        let ty = 2.0 * (z * local.x - x * local.z);
        let tz = 2.0 * (x * local.y - y * local.x);
        Vec3::new(
            p.x + local.x + w * tx + (y * tz - z * ty),
            p.y + local.y + w * ty + (z * tx - x * tz),
            p.z + local.z + w * tz + (x * ty - y * tx),
        )
    };

    let kinds: [(&str, JointKind, [f32; 3], f32); 5] = [
        (
            "joint-spherical",
            JointKind::Spherical,
            [0.0, 0.0, 1.0],
            0.2,
        ),
        ("joint-revolute", JointKind::Revolute, [0.0, 0.0, 1.0], 0.2),
        ("joint-fixed", JointKind::Fixed, [0.0, 0.0, 1.0], 0.3),
        (
            "joint-prismatic",
            JointKind::Prismatic,
            [1.0, 0.0, 0.0],
            0.2,
        ),
        ("joint-distance", JointKind::Distance, [0.0, 0.0, 1.0], 0.35),
    ];
    println!("关节探针（PhysArena jointProbe 复刻；240 步）：");
    for (name, kind, axis, tol) in kinds {
        let rope = kind == JointKind::Distance;
        let b_local = if rope {
            Vec3::ZERO
        } else {
            Vec3::new(0.0, BODY_HALF + gap, 0.0)
        };
        let body_y = if rope {
            attach_y - rest
        } else {
            attach_y - BODY_HALF - gap
        };
        let mut w = World::new(cfg.clone());
        ground(&mut w, 80.0);
        // 锚块必须走 `add_static`：`add_box` 恒为动态体，而 `mass_props` 会把
        // `density <= 0` 回退成 1.0 ⇒ 用密度 0 假装静态只会得到一个普通动态体，
        // 锚与悬挂体一起自由落体、相对分离恒 0 ——探针假通过（本轮踩过）。
        let m = mat(&mut w, 0.5, 0.0);
        let ia = w.bodies.len();
        w.add_static(
            Shape::Box {
                half: Vec3::splat(ANCHOR_HALF),
            },
            Vec3::new(0.0, anchor_y, 0.0),
            vxl_phys_core::Quat::IDENTITY,
        );
        w.bodies.set_material(ia, m);
        let mb = mat(&mut w, 0.5, 0.0);
        add_box(
            &mut w,
            Vec3::new(0.0, body_y, 0.0),
            Vec3::splat(BODY_HALF),
            mb,
            500.0,
        );
        let mut j = Joint::new(kind, 1, 2, a_local, b_local);
        if !rope {
            j = j.with_axis(Vec3::new(axis[0], axis[1], axis[2]));
        }
        if rope {
            j = j.with_rest(rest);
        }
        w.add_joint(j);
        let mut worst = 0.0f32;
        let mut last = 0.0f32;
        for _ in 0..240 {
            w.step();
            let (pa, qa) = w.bodies.pose(1);
            let (pb, qb) = w.bodies.pose(2);
            let wa = world_point(pa, qa, a_local);
            let wb = world_point(pb, qb, b_local);
            let d = (wb - wa).length();
            let err = if rope { (d - rest).abs() } else { d };
            worst = worst.max(err);
            last = err;
        }
        let y = w.bodies.position[2].y;
        let verdict = if worst <= tol { "PASS" } else { "FAIL" };
        println!(
            "  {name:<16} {verdict}  峰值 {worst:.4} m（上限 {tol:.2}） 末态 {last:.4} m  悬挂体 y={y:.3}（初 {body_y:.3}）"
        );
    }
}

/// PhysArena `chain-hinge` / `hanging-tower` 的复刻（动态关节链：拉伸是判据）。
///
/// 链场景是关节求解器最难看的形态——一个子步内的冲量要在整条链上传播，
/// 迭代数不足就表现为"橡皮筋"。指标：全链**最大锚点分离**与**链长拉伸**
/// （首末体距离 / 名义长度），外加是否发散（NaN）与末态速度。
fn scene_joint_chains(cfg: PhysConfig) {
    let n = 40usize;
    let link = 0.6f32;

    // ---- 铰链链：静态锚块 + n 个球，球形关节（锚点 ±0.3 局部 X）----
    let mut w = World::new(cfg.clone());
    ground(&mut w, 120.0);
    let m = mat(&mut w, 0.4, 0.0);
    let ia = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::splat(0.15),
        },
        Vec3::new(0.0, 8.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(ia, m);
    let mut prev = ia as u32;
    for i in 0..n {
        let ms = mat(&mut w, 0.4, 0.0);
        add_sphere(
            &mut w,
            Vec3::new(link * (i + 1) as f32, 8.0, 0.0),
            0.17,
            ms,
            2000.0,
        );
        let cur = (w.bodies.len() - 1) as u32;
        w.add_joint(Joint::new(
            JointKind::Spherical,
            prev,
            cur,
            Vec3::new(0.3, 0.0, 0.0),
            Vec3::new(-0.3, 0.0, 0.0),
        ));
        prev = cur;
    }
    let nominal = link * n as f32;
    let mut worst = 0.0f32;
    let mut nan = false;
    let t0 = std::time::Instant::now();
    for _ in 0..600 {
        w.step();
        for j in 0..n {
            let (pa, qa) = w.bodies.pose(1 + j);
            let (pb, qb) = w.bodies.pose(2 + j);
            let ra = vxl_phys_core::Mat3::from_quat(qa).mul_vec3(Vec3::new(0.3, 0.0, 0.0));
            let rb = vxl_phys_core::Mat3::from_quat(qb).mul_vec3(Vec3::new(-0.3, 0.0, 0.0));
            let d = ((pb + rb) - (pa + ra)).length();
            if !d.is_finite() {
                nan = true;
            }
            worst = worst.max(d);
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0 / 600.0;
    let (p0, _) = w.bodies.pose(1);
    let (pn, _) = w.bodies.pose(n);
    let span = (pn - p0).length();
    println!("chain-hinge（{n} 环，600 步）：");
    println!(
        "  最大锚点分离 {worst:.4} m  首末间距 {span:.3}（名义 {nominal:.2}）  发散={nan}  步耗时 {ms:.3} ms"
    );
    println!(
        "  末态：链尾 y={:.3}（初 {:.3}）",
        w.bodies.position[n].y, 8.0
    );

    // ---- 悬挂塔：固定关节，方块半 0.3、节距 0.62（锚点 ±0.31 局部 Y）----
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let top = 3.0 + n as f32 * 0.62;
    let m = mat(&mut w, 0.5, 0.0);
    let ia = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(0.0, top, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(ia, m);
    let mut prev = ia as u32;
    for i in 0..n {
        let mb = mat(&mut w, 0.5, 0.0);
        add_box(
            &mut w,
            Vec3::new(0.0, top - (i + 1) as f32 * 0.62, 0.0),
            Vec3::splat(0.3),
            mb,
            1500.0,
        );
        let cur = (w.bodies.len() - 1) as u32;
        w.add_joint(Joint::new(
            JointKind::Fixed,
            prev,
            cur,
            Vec3::new(0.0, -0.31, 0.0),
            Vec3::new(0.0, 0.31, 0.0),
        ));
        prev = cur;
    }
    let mut worst = 0.0f32;
    let mut nan = false;
    let mut worst_tilt = 1.0f32;
    let t0 = std::time::Instant::now();
    for _ in 0..600 {
        w.step();
        for j in 0..n {
            let (pa, qa) = w.bodies.pose(1 + j);
            let (pb, qb) = w.bodies.pose(2 + j);
            let ra = vxl_phys_core::Mat3::from_quat(qa).mul_vec3(Vec3::new(0.0, -0.31, 0.0));
            let rb = vxl_phys_core::Mat3::from_quat(qb).mul_vec3(Vec3::new(0.0, 0.31, 0.0));
            let d = ((pb + rb) - (pa + ra)).length();
            if !d.is_finite() {
                nan = true;
            }
            worst = worst.max(d);
            // 固定关节的"硬度"还要看相对姿态：相邻块的局部 X 轴应始终同向。
            let axa = vxl_phys_core::Mat3::from_quat(qa).mul_vec3(Vec3::X);
            let axb = vxl_phys_core::Mat3::from_quat(qb).mul_vec3(Vec3::X);
            worst_tilt = worst_tilt.min(axa.dot(axb));
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0 / 600.0;
    println!("hanging-tower（{n} 段，600 步）：");
    println!(
        "  最大锚点分离 {worst:.4} m  最小相邻轴同向 cos {worst_tilt:.4}  发散={nan}  步耗时 {ms:.3} ms"
    );
    println!("  末态：塔尾 y={:.3}", w.bodies.position[n].y);
}

fn main() {
    let mut args = std::env::args().skip(1);
    let which = args.next().unwrap_or_else(|| "pyramid".to_string());
    let mut cfg = PhysConfig::default();
    // 允许 --iters N 做迭代数敏感性实验（默认 16）。
    let rest: Vec<String> = args.collect();
    if let Some(pos) = rest.iter().position(|a| a == "--iters") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.velocity_iterations = v;
        }
    }
    if let Some(pos) = rest.iter().position(|a| a == "--substeps") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.substeps = v;
        }
    }
    if let Some(pos) = rest.iter().position(|a| a == "--inner") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.normal_inner = v;
        }
    }
    if let Some(pos) = rest.iter().position(|a| a == "--serial") {
        // 串行作业系统（对照"每步开线程"的开销）。
        if rest.get(pos + 1).map(|s| s.as_str()) != Some("0") {
            let mut w_cfg = cfg.clone();
            w_cfg.velocity_iterations = cfg.velocity_iterations;
            cfg = w_cfg;
        }
    }
    println!(
        "配置：iterations={} inner={} substeps={} contact_skin={}",
        cfg.velocity_iterations, cfg.normal_inner, cfg.substeps, cfg.contact_skin
    );
    let scenes: Vec<&str> = if which == "all" {
        vec!["pyramid", "wall", "ballpit"]
    } else {
        vec![which.as_str()]
    };
    for s in scenes {
        // slide / approach 是"打印轨迹"型基准，不走 bench() 的统计口径。
        if s == "slide" {
            scene_slide(cfg.clone(), 120);
            continue;
        }
        if s == "approach" {
            scene_approach(cfg.clone());
            continue;
        }
        if s == "voxel_land" {
            scene_voxel_land(cfg.clone());
            continue;
        }
        if s == "wall_provider" {
            scene_wall_provider(cfg.clone());
            continue;
        }
        if s == "mesh_land" {
            scene_mesh_land(cfg.clone());
            continue;
        }
        if s == "joints" {
            scene_joint_probes(cfg.clone());
            continue;
        }
        if s == "joint_chains" {
            println!(
                "配置：joint_iterations={} substeps={}",
                cfg.joint_iterations, cfg.substeps
            );
            scene_joint_chains(cfg.clone());
            continue;
        }
        let serial = rest.iter().any(|a| a == "--serial");
        let mut w = match s {
            "pyramid" => scene_pyramid(cfg.clone()),
            "wall" => scene_wall(cfg.clone()),
            "ballpit" => scene_ballpit(cfg.clone()),
            "trimesh" => scene_trimesh_terrain(cfg.clone()),
            other => {
                eprintln!("未知场景 {other}（pyramid / wall / ballpit / all）");
                return;
            }
        };
        if serial {
            w.jobs = Box::new(vxl_phys_core::SerialJobSystem);
        }
        let extra = rest
            .iter()
            .position(|a| a == "--steps")
            .and_then(|pos| rest.get(pos + 1))
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(0);
        bench(s, w, extra);
    }
}
