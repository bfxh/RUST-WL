//! **演示场景转储**（headless 引擎 → 渲染器）：把逐帧体状态写成二进制，
//! 由 `scripts/render_demo.py`（Pillow）渲染成 GIF/帧序列。
//!
//! 运行：`cargo run --release -p vxl-phys --example showcase [ticks] [out.bin]`
//! 默认 600 tick（每帧 2 tick ⇒ 30 fps × 10 s）。
//!
//! 场景（六个域同屏，ROUTE §3）：
//! - 体素：地板 + 墙体；炮弹冲击 ⇒ 挖洞 + 碎块（破坏管线）；
//! - 多边形：立方体外壳 Voronoi 预断裂 ⇒ 8 个凸碎块落地；
//! - 高斯喷溅：球状云 ⇒ 盒堆落在隐式场等值面上；
//! - 三角网：波浪台面（TriMesh 薄壳 + 均匀网格加速）⇒ 盒/球落在任意三角面上；
//! - 刚体盒：自由堆积（宽相/求解器负载）；
//! - 流体（WCSPH）：地板凿出石盆 + 铸装近平衡水块 ⇒ 平稳驻留水面。
//!
//! 转储格式（小端）：见 `write_header`/`write_frame` 注释（渲染器逐字节对应）。
//! 版本 2 = 头部在喷溅节后追加三角网节（张数 + 每张顶点/三角形表）。
//! 版本 3 = 头部追加流体系统数；帧尾追加流体粒子节（每系统：粒子数 + 位置 3f32）。

use std::io::Write;
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

const KIND_BOX: u8 = 0;
const KIND_SPHERE: u8 = 1;
const KIND_HULL: u8 = 2;
const VERSION: u32 = 3;

/// 六域建景产物：转储与末行报表还要用的句柄/计数（main 不再关心各域怎么搭）。
struct Scene {
    voxel_id: u32,
    splat_id: u32,
    mesh_pid: u32,
    hull_pieces: usize,
    pile_boxes: usize,
    cloud_boxes: usize,
    mesh_bodies: usize, // 台面上盒 + 球（口径同旧报表：mesh_boxes.len() + 1）
}

/// 体素域：地板 3 层 + x = +4 墙 + 石盆单格凹坑（流体的盆腔）。
fn terrain_volume() -> vxl_phys_terrain::voxel::VoxelVolume {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-8.0, 0.0, -8.0), 0.5, 32, 3, 32);
    vol.fill_box(Vec3::new(-8.0, 0.0, -8.0), Vec3::new(8.0, 1.5, 8.0)); // 地板 3 层
    vol.fill_box(Vec3::new(3.0, 1.5, -3.0), Vec3::new(3.5, 4.5, 3.0)); // 墙 1×6×12 格
    vol.set(18, 2, 22, false); // 凿出流体石盆：单格凹坑 x∈[1,1.5] y∈[1,1.5] z∈[3,3.5]
    vol
}

/// 流体域：石盆里的铸装近平衡水块。
/// [8,8,5] 铸装口径与门面门禁测试同款：足印 0.35，沉降高 ≈0.24，盆腔
/// 0.5×0.5 恰好半满。任何带落差的入水都会触发 WCSPH 驻留瞬态（压实波
/// 在块顶心聚焦 ⇒ 近钳制速度喷泉，PLAN-0.3 §4 负面结论），铸装则无。
fn fluid_system() -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(1.075, 1.05, 3.075),
        [8, 8, 5],
        0.05,
    )
}

/// 高斯喷溅域：x = -4 处半径 1.4 的半球云（颜色随高度渐变是渲染桥字段，物理不消费）。
fn splat_cloud() -> vxl_phys_splat::GaussianSplatField {
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
                        color: [0.35 + 0.5 * t, 0.55 + 0.35 * (1.0 - t), 0.95 - 0.35 * t],
                    });
                }
            }
        }
    }
    field
}

/// 多边形域：立方体外壳 Voronoi 预断裂 ⇒ 8 个凸碎块从 3.4 m 处落下。
fn fractured_hull_pieces(w: &mut World) -> Vec<u32> {
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
    w.spawn_hull_pieces(
        hull,
        &seeds,
        Vec3::new(0.0, 3.4, 0.0),
        Quat::IDENTITY,
        800.0,
    )
}

/// 刚体盒域：地面盒堆 24 个 + 落在高斯云上的盒堆 6 个，返回两堆数量。
fn box_piles(w: &mut World) -> (usize, usize) {
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
            Vec3::new(
                -4.4 + (i % 3) as f32 * 0.55,
                4.2 + (i / 3) as f32 * 0.7,
                -0.3 + (i % 2) as f32 * 0.6,
            ),
            Quat::IDENTITY,
            700.0,
        ));
    }
    (boxes.len(), cloud_boxes.len())
}

/// 三角网域：波浪台面（x,z ∈ [4,8]，9×9 格，绕序使面法线朝 +y，架在体素地板上方）。
fn wave_mesh() -> vxl_phys_terrain::mesh::TriMesh {
    let (mn, mstep) = (9usize, 0.5f32);
    let mut mesh_verts: Vec<Vec3> = Vec::new();
    for i in 0..mn {
        for j in 0..mn {
            let u = i as f32 * mstep;
            let v = j as f32 * mstep;
            let y = 1.55 + 0.225 * (1.0 + (0.9 * u).sin() * (0.9 * v).sin());
            mesh_verts.push(Vec3::new(4.0 + u, y, -7.5 + v));
        }
    }
    let mut mesh_tris: Vec<[u32; 3]> = Vec::new();
    for i in 0..mn - 1 {
        for j in 0..mn - 1 {
            let a = (i * mn + j) as u32;
            let b = ((i + 1) * mn + j) as u32;
            let c = ((i + 1) * mn + j + 1) as u32;
            let d = (i * mn + j + 1) as u32;
            mesh_tris.push([a, d, b]);
            mesh_tris.push([b, d, c]);
        }
    }
    vxl_phys_terrain::mesh::TriMesh::new(mesh_verts, mesh_tris)
}

/// 注册波浪台面并返回其 provider id——**必须**在 add_mesh 之前取：
/// push_mesh 追加 ⇒ provider id = 注册前长度。
fn add_wave_table(w: &mut World) -> u32 {
    let mesh_pid = w.providers().len() as u32;
    w.add_mesh(wave_mesh());
    mesh_pid
}

/// 炮弹：冲击体素墙（破坏由主循环逐 tick 调 `apply_impact_destruction` 完成）。
fn fire_bullet(w: &mut World) {
    let bullet = w.add_dynamic(
        Shape::Sphere { radius: 0.35 },
        Vec3::new(-3.0, 3.6, 0.0),
        Quat::IDENTITY,
        3000.0,
    );
    w.bodies.linvel[bullet as usize] = Vec3::new(11.0, -0.5, 0.0);
}

/// 落在波浪台面上的 5 盒 + 1 球。注册顺序在炮弹之后，勿提前：体序影响求解顺序。
fn table_bodies(w: &mut World) -> usize {
    let mut mesh_boxes = Vec::new();
    for i in 0..5 {
        mesh_boxes.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.24),
            },
            Vec3::new(
                4.6 + (i % 3) as f32 * 0.6,
                4.0 + (i / 3) as f32 * 0.55,
                -5.3 + (i / 3) as f32 * 0.7,
            ),
            Quat::IDENTITY,
            750.0,
        ));
    }
    w.add_dynamic(
        Shape::Sphere { radius: 0.3 },
        Vec3::new(6.2, 4.6, -4.1),
        Quat::IDENTITY,
        700.0,
    );
    mesh_boxes.len() + 1
}

/// 六域建景。**装配顺序勿改**：mesh 的 provider id 与全体 body 索引都取决于注册序。
fn build_scene(w: &mut World) -> Scene {
    let voxel_id = w.add_voxel(terrain_volume());
    w.add_fluid(fluid_system(), &[voxel_id]);
    let splat_id = w.add_splat_field(splat_cloud());
    let hull_pieces = fractured_hull_pieces(w).len();
    let (pile_boxes, cloud_boxes) = box_piles(w);
    let mesh_pid = add_wave_table(w);
    fire_bullet(w);
    let mesh_bodies = table_bodies(w);
    Scene {
        voxel_id,
        splat_id,
        mesh_pid,
        hull_pieces,
        pile_boxes,
        cloud_boxes,
        mesh_bodies,
    }
}

/// 转储头（静态一次）：magic + version + ticks + ticks_per_frame + dt，
/// 之后依次为体素体尺寸 / 喷溅场 / 三角网 / 流体系统数。
fn write_header(f: &mut impl Write, w: &World, s: &Scene, ticks: usize) {
    f.write_all(b"VXLD").unwrap();
    f.write_all(&VERSION.to_le_bytes()).unwrap();
    f.write_all(&(ticks as u32).to_le_bytes()).unwrap();
    f.write_all(&2u32.to_le_bytes()).unwrap(); // 每帧 2 tick
    f.write_all(&(1.0f32 / 60.0).to_le_bytes()).unwrap();

    // 体素体尺寸（一次；占用位每帧重发，见帧内）
    let (vox_dims, vox_origin, vox_step) = w
        .providers()
        .voxel(s.voxel_id)
        .map(|v| (v.dims(), v.origin(), v.step()))
        .unwrap();
    for x in [vox_dims.0, vox_dims.1, vox_dims.2] {
        f.write_all(&x.to_le_bytes()).unwrap();
    }
    f.write_all(&vox_origin.x.to_le_bytes()).unwrap();
    f.write_all(&vox_origin.y.to_le_bytes()).unwrap();
    f.write_all(&vox_origin.z.to_le_bytes()).unwrap();
    f.write_all(&vox_step.to_le_bytes()).unwrap();
    // 喷溅场（静态一次）：数量 + 每颗（中心3 + 尺度3 + 透明1 + 颜色3）
    let field = w.providers().splat(s.splat_id).unwrap();
    f.write_all(&(field.len() as u32).to_le_bytes()).unwrap();
    for sp in field.splats() {
        for v in [
            sp.center.x,
            sp.center.y,
            sp.center.z,
            sp.scale.x,
            sp.scale.y,
            sp.scale.z,
            sp.opacity,
        ] {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
        for c in sp.color {
            f.write_all(&c.to_le_bytes()).unwrap();
        }
    }

    // 三角网（静态一次）：张数 + 每张（顶点数 + 三角形数 + 顶点 3f + 三角形 3×u32）
    let mesh = w.providers().mesh(s.mesh_pid).unwrap();
    f.write_all(&1u32.to_le_bytes()).unwrap(); // 场景内网格张数
    f.write_all(&(mesh.verts().len() as u32).to_le_bytes())
        .unwrap();
    f.write_all(&(mesh.tris().len() as u32).to_le_bytes())
        .unwrap();
    for p in mesh.verts() {
        for v in [p.x, p.y, p.z] {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
    }
    for t in mesh.tris() {
        for ix in t {
            f.write_all(&ix.to_le_bytes()).unwrap();
        }
    }

    // 流体系统数（静态；粒子位置每帧在帧尾发）
    f.write_all(&(w.fluids().len() as u32).to_le_bytes())
        .unwrap();
}

/// 帧内动态体记录：kind + awake + 位姿 + 尺寸（提供者 marker 不画，体数据另发）。
/// 尺寸：盒/圆柱 = half(3)；球 = 半径(1)；外壳 = 点云。
fn write_body_records(f: &mut impl Write, w: &World) {
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
}

/// 帧内体素占用位（每帧发；32×3×32 = 3072 bit = 384 B）。
fn write_voxel_bits(f: &mut impl Write, w: &World, voxel_id: u32) {
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

/// 帧尾流体粒子位置（每系统：粒子数 + 位置 3f32 × n）。
fn write_fluid_particles(f: &mut impl Write, w: &World) {
    for (sys, ..) in w.fluids() {
        let ps = sys.positions();
        f.write_all(&(ps.len() as u32).to_le_bytes()).unwrap();
        for p in ps {
            for v in [p.x, p.y, p.z] {
                f.write_all(&v.to_le_bytes()).unwrap();
            }
        }
    }
}

/// 一帧 = tick + 该 tick 毫秒 + 动态体记录 + 体素占用位 + 流体粒子。
fn write_frame(f: &mut impl Write, w: &World, s: &Scene, t: usize, ms: f32) {
    f.write_all(&(t as u32).to_le_bytes()).unwrap();
    f.write_all(&ms.to_le_bytes()).unwrap();
    write_body_records(f, w);
    write_voxel_bits(f, w, s.voxel_id);
    write_fluid_particles(f, w);
}

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let out_path = args
        .next()
        .unwrap_or_else(|| "out/showcase.bin".to_string());

    let mut w = World::new(PhysConfig::default());
    let s = build_scene(&mut w);

    // ---- 转储 ----
    std::fs::create_dir_all("out").ok();
    let mut f = std::io::BufWriter::new(std::fs::File::create(&out_path).expect("open dump"));
    write_header(&mut f, &w, &s, ticks);

    let mut ms_sum = 0f64;
    let mut ms_max = 0f64;
    for t in 1..=ticks {
        let t0 = std::time::Instant::now();
        w.step();
        // 冲击破坏：体素墙被炮弹打中即挖洞 + 碎块（每 tick 扫描接触，判据=法向接近速度）
        w.apply_impact_destruction(s.voxel_id, 4.0, 900.0);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        ms_sum += ms;
        ms_max = ms_max.max(ms);
        if t % 2 != 0 {
            continue; // 每帧 = 2 tick
        }
        write_frame(&mut f, &w, &s, t, ms as f32);
    }
    f.flush().unwrap();
    let frames = ticks / 2;
    println!(
        "转储完成：{out_path}（{frames} 帧 | {ticks} tick）\n\
         体素 {} 格 | 外壳碎块 {} | 盒 {} | 云上盒 {} | 台面上盒 {} + 球 1 | 喷溅 {} 颗 | 流体 {} 粒\n\
         单 tick 均值 {:.2} ms（{:.0} FPS）| 峰值 {:.2} ms",
        w.providers().voxel(s.voxel_id).unwrap().filled_count(),
        s.hull_pieces,
        s.pile_boxes,
        s.cloud_boxes,
        s.mesh_bodies,
        w.providers().splat(s.splat_id).unwrap().len(),
        w.fluids()
            .first()
            .map(|(sys, ..)| sys.positions().len())
            .unwrap_or(0),
        ms_sum / ticks as f64,
        1000.0 / (ms_sum / ticks as f64),
        ms_max,
    );
}
