//! **演示场景转储**（headless 引擎 → 渲染器）：把逐帧体状态写成二进制，
//! 由 `scripts/render_demo.py`（Pillow）渲染成 GIF/帧序列。
//!
//! 运行：`cargo run --release -p vxl-phys --example showcase [ticks] [out.bin]`
//! 默认 600 tick（每帧 2 tick ⇒ 30 fps × 10 s）。
//!
//! 场景（四个域同屏，ROUTE §3）：
//! - 体素：地板 + 墙体；炮弹冲击 ⇒ 挖洞 + 碎块（破坏管线）；
//! - 多边形：立方体外壳 Voronoi 预断裂 ⇒ 8 个凸碎块落地；
//! - 高斯喷溅：球状云 ⇒ 盒堆落在隐式场等值面上；
//! - 刚体盒：自由堆积（宽相/求解器负载）。
//!
//! 转储格式（小端）：见 `write_header`/`write_frame` 注释（渲染器逐字节对应）。

use std::io::Write;
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

const KIND_BOX: u8 = 0;
const KIND_SPHERE: u8 = 1;
const KIND_HULL: u8 = 2;
const VERSION: u32 = 1;

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let out_path = args
        .next()
        .unwrap_or_else(|| "out/showcase.bin".to_string());

    let mut w = World::new(PhysConfig::default());

    // ---- 体素地板 + 墙（x = +4 处）----
    let mut vol = vxl_phys_terrain::voxel::VoxelVolume::new(
        Vec3::new(-8.0, 0.0, -8.0),
        0.5,
        32,
        3,
        32,
    );
    vol.fill_box(Vec3::new(-8.0, 0.0, -8.0), Vec3::new(8.0, 1.5, 8.0)); // 地板 3 层
    vol.fill_box(Vec3::new(3.0, 1.5, -3.0), Vec3::new(3.5, 4.5, 3.0)); // 墙 1×6×12 格
    let voxel_id = w.add_voxel(vol);

    // ---- 高斯喷溅云（x = -4 处的半球）----
    let mut field = vxl_phys_splat::GaussianSplatField::new(0.5);
    for i in -4..=4 {
        for j in 0..=5 {
            for k in -4..=4 {
                let c = Vec3::new(i as f32 * 0.45, j as f32 * 0.45 + 1.0, k as f32 * 0.45);
                let d = Vec3::new(c.x, c.y - 1.0, c.z).length();
                if d <= 1.4 {
                    let t = (d / 1.4).clamp(0.0, 1.0);
                    field.push(vxl_phys_splat::Splat {
                        center: c + Vec3::new(-4.0, 0.0, 0.0),
                        scale: Vec3::splat(0.42),
                        rot: vxl_phys_core::Mat3::IDENTITY,
                        opacity: 1.0,
                        // 颜色随高度渐变（渲染桥字段；物理不消费）
                        color: [0.35 + 0.5 * t, 0.55 + 0.35 * (1.0 - t), 0.95 - 0.35 * t],
                    });
                }
            }
        }
    }
    let splat_id = w.add_splat_field(field);

    // ---- 多边形：立方体外壳 Voronoi 预断裂（x = 0）----
    let cube: Vec<Vec3> = {
        let mut p = Vec::new();
        for &x in &[-0.6f32, 0.6] {
            for &y in &[-0.6f32, 0.6] {
                for &z in &[-0.6f32, 0.6] {
                    p.push(Vec3::new(x, y, z));
                }
            }
        }
        p
    };
    let hull = w.add_hull(cube);
    let seeds: Vec<Vec3> = [
        Vec3::new(-0.35, -0.3, -0.4),
        Vec3::new(-0.35, -0.3, 0.3),
        Vec3::new(-0.35, 0.4, -0.4),
        Vec3::new(-0.35, 0.4, 0.3),
        Vec3::new(0.35, -0.3, -0.4),
        Vec3::new(0.35, -0.3, 0.3),
        Vec3::new(0.35, 0.4, -0.4),
        Vec3::new(0.35, 0.4, 0.3),
    ]
    .to_vec();
    let pieces = w.spawn_hull_pieces(hull, &seeds, Vec3::new(0.0, 3.4, 0.0), Quat::IDENTITY, 800.0);

    // ---- 刚体盒堆（分两处）----
    let mut boxes = Vec::new();
    for i in 0..24 {
        let x = -6.2 + (i % 4) as f32 * 0.75;
        let z = -1.0 + (i / 4) as f32 * 0.75;
        boxes.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.32),
            },
            Vec3::new(x, 3.0 + (i / 4) as f32 * 0.1, z),
            Quat::IDENTITY,
            900.0,
        ));
    }
    // 落在高斯云上的盒堆
    let mut cloud_boxes = Vec::new();
    for i in 0..6 {
        cloud_boxes.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.28),
            },
            Vec3::new(-4.4 + (i % 3) as f32 * 0.55, 4.2 + (i / 3) as f32 * 0.7, -0.3 + (i % 2) as f32 * 0.6),
            Quat::IDENTITY,
            700.0,
        ));
    }

    // ---- 炮弹（冲击体素墙）----
    let bullet = w.add_dynamic(
        Shape::Sphere { radius: 0.35 },
        Vec3::new(-3.0, 3.6, 0.0),
        Quat::IDENTITY,
        3000.0,
    );
    w.bodies.linvel[bullet as usize] = Vec3::new(11.0, -0.5, 0.0);

    // ---- 转储 ----
    std::fs::create_dir_all("out").ok();
    let mut f = std::io::BufWriter::new(std::fs::File::create(&out_path).expect("open dump"));
    // 头：magic + version + ticks + ticks_per_frame + dt
    f.write_all(b"VXLD").unwrap();
    f.write_all(&VERSION.to_le_bytes()).unwrap();
    f.write_all(&(ticks as u32).to_le_bytes()).unwrap();
    f.write_all(&2u32.to_le_bytes()).unwrap(); // 每帧 2 tick
    f.write_all(&(1.0f32 / 60.0).to_le_bytes()).unwrap();

    // 体素体尺寸（一次；占用位每帧重发，见帧内）
    let (vox_dims, vox_origin, vox_step) = w
        .providers()
        .voxel(voxel_id)
        .map(|v| {
            (v.dims(), v.origin(), v.step())
        })
        .unwrap();
    for x in [vox_dims.0, vox_dims.1, vox_dims.2] {
        f.write_all(&x.to_le_bytes()).unwrap();
    }
    f.write_all(&vox_origin.x.to_le_bytes()).unwrap();
    f.write_all(&vox_origin.y.to_le_bytes()).unwrap();
    f.write_all(&vox_origin.z.to_le_bytes()).unwrap();
    f.write_all(&vox_step.to_le_bytes()).unwrap();
    // 喷溅场（静态一次）：数量 + 每颗（中心3 + 尺度3 + 透明1 + 颜色3）
    let field = w.providers().splat(splat_id).unwrap();
    f.write_all(&(field.len() as u32).to_le_bytes()).unwrap();
    for s in field.splats() {
        for v in [s.center.x, s.center.y, s.center.z, s.scale.x, s.scale.y, s.scale.z, s.opacity] {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
        for c in s.color {
            f.write_all(&c.to_le_bytes()).unwrap();
        }
    }

    let mut ms_sum = 0f64;
    let mut ms_max = 0f64;
    for t in 1..=ticks {
        let t0 = std::time::Instant::now();
        w.step();
        // 冲击破坏：体素墙被炮弹打中即挖洞 + 碎块（每 tick 扫描接触，判据=法向接近速度）
        w.apply_impact_destruction(voxel_id, 4.0, 900.0);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        ms_sum += ms;
        ms_max = ms_max.max(ms);
        if t % 2 != 0 {
            continue; // 每帧 = 2 tick
        }
        // 帧：tick + 每 tick 毫秒 + 动态体记录
        f.write_all(&(t as u32).to_le_bytes()).unwrap();
        f.write_all(&(ms as f32).to_le_bytes()).unwrap();
        let mut n: u32 = 0;
        let mut body_recs: Vec<Vec<u8>> = Vec::new();
        for i in 0..w.bodies.len() {
            let shape = w.bodies.shape[i];
            if matches!(shape, Shape::Provider(_) | Shape::HeightField(_)) {
                continue; // 提供者 marker 不画（体数据另发）
            }
            let mut rec: Vec<u8> = Vec::with_capacity(48);
            let kind = match shape {
                Shape::Box { .. } => KIND_BOX,
                Shape::Sphere { .. } => KIND_SPHERE,
                Shape::ConvexHull { .. } => KIND_HULL,
                Shape::Cylinder { .. } => KIND_BOX,
                _ => continue,
            };
            rec.push(kind);
            rec.push(u8::from(w.bodies.awake[i]));
            let p = w.bodies.position[i];
            let r = w.bodies.rot(i);
            rec.extend_from_slice(&p.x.to_le_bytes());
            rec.extend_from_slice(&p.y.to_le_bytes());
            rec.extend_from_slice(&p.z.to_le_bytes());
            for q in [r.x, r.y, r.z, r.w] {
                rec.extend_from_slice(&q.to_le_bytes());
            }
            // 尺寸：盒/圆柱 = half(3)；球 = 半径(1)；外壳 = 点云
            match shape {
                Shape::Box { half } => {
                    for v in [half.x, half.y, half.z] {
                        rec.extend_from_slice(&v.to_le_bytes());
                    }
                }
                Shape::Sphere { radius } => {
                    rec.extend_from_slice(&radius.to_le_bytes());
                }
                Shape::ConvexHull { hull, .. } => {
                    let pts = w.hull_points(hull);
                    rec.extend_from_slice(&(pts.len() as u32).to_le_bytes());
                    for p in pts {
                        for v in [p.x, p.y, p.z] {
                            rec.extend_from_slice(&v.to_le_bytes());
                        }
                    }
                }
                _ => {}
            }
            n += 1;
            body_recs.push(rec);
        }
        f.write_all(&n.to_le_bytes()).unwrap();
        for rec in &body_recs {
            f.write_all(rec).unwrap();
        }
        // 体素占用位（每帧发；32×3×32 = 3072 bit = 384 B）
        let v = w.providers().voxel(voxel_id).unwrap();
        let (nx, ny, nz) = v.dims();
        let mut bits: Vec<u8> = vec![0u8; ((nx * ny * nz) as usize).div_ceil(8)];
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    if v.get(ix, iy, iz) {
                        let idx = (ix * ny * nz + iy * nz + iz) as usize;
                        bits[idx / 8] |= 1 << (idx % 8);
                    }
                }
            }
        }
        f.write_all(&bits).unwrap();
    }
    f.flush().unwrap();
    let frames = ticks / 2;
    println!(
        "转储完成：{out_path}（{frames} 帧 | {ticks} tick）\n\
         体素 {} 格 | 外壳碎块 {} | 盒 {} | 云上盒 {} | 喷溅 {} 颗\n\
         单 tick 均值 {:.2} ms（{:.0} FPS）| 峰值 {:.2} ms",
        w.providers().voxel(voxel_id).unwrap().filled_count(),
        pieces.len(),
        boxes.len(),
        cloud_boxes.len(),
        w.providers().splat(splat_id).unwrap().len(),
        ms_sum / ticks as f64,
        1000.0 / (ms_sum / ticks as f64),
        ms_max,
    );
}
