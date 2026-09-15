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
    w.add_dynamic(Shape::Box { half }, pos, vxl_phys_core::Quat::IDENTITY, density);
    w.bodies.set_material(i, m);
}

fn add_sphere(w: &mut World, pos: Vec3, r: f32, m: vxl_phys_core::MaterialId, density: f32) {
    let i = w.bodies.len();
    w.add_dynamic(Shape::Sphere { radius: r }, pos, vxl_phys_core::Quat::IDENTITY, density);
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
            add_box(&mut w, Vec3::new(x, y, 0.0), Vec3::splat(box_half), m, 1000.0);
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
            add_box(&mut w, Vec3::new(x, y, 0.0), Vec3::new(hw, hh, hd), m, 1000.0);
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
    let total_us = (t.broadphase_us + t.narrowphase_us + t.solve_us + t.integrate_vel_us
        + t.integrate_pos_us + t.fields_us + t.ccd_us)
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
    let (d_island, d_solve, d_sleep, _) = w.solver.last_phase_us;
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
    add_box(&mut w, Vec3::new(0.0, 0.4, 0.0), Vec3::splat(0.4), m, 1000.0);
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
    add_box(&mut w, Vec3::new(-3.0, 0.0, 0.0), Vec3::splat(0.4), mb, 1000.0);
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
        let serial = rest.iter().any(|a| a == "--serial");
        let mut w = match s {
            "pyramid" => scene_pyramid(cfg.clone()),
            "wall" => scene_wall(cfg.clone()),
            "ballpit" => scene_ballpit(cfg.clone()),
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
