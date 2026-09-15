//! **塌坝演示**（液体域专项）：高水柱失支撑坍塌 ⇒ 波前沿盆底推进 ⇒ 拍打围堰。
//!
//! 运行：`cargo run --release -p vxl-phys --example dam_break [ticks] [out.bin]`
//! 默认 400 tick（每帧 2 tick ⇒ 30 fps × ~6.7 s）。
//! 渲染：`python scripts/render_demo.py --src dam_break.bin --dst dam_break.gif --dist 8`
//!
//! 与 showcase 同格式的 VXLD v3 转储（单体素盆 + 零刚体 + 流体粒子节）。
//! 水柱为**自由柱**（不铸装近平衡）：驻留瞬态顶心喷泉在此就是演示动作——
//! 塌坝本就该飞溅，无驻留断言；与门面门禁的铸装口径相区分（PLAN-0.3 §4）。

use std::io::Write;
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Vec3};

const VERSION: u32 = 3;

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(400);
    let out_path = args
        .next()
        .unwrap_or_else(|| "out/dam_break.bin".to_string());

    let mut w = World::new(PhysConfig::default());

    // ---- 盆：地板 2 层（y∈[0,1]）+ 围堰 1 层（y∈[1,1.5]），内域 1.5×1.5 m ----
    // 盆尺寸按水量配：0.063 m³ 摊在 7 m 盆上只有 ~1 mm 厚（渲染成散点），
    // 缩到 1.5 m 内域后水层 ~30 mm，粒子间距 ≈ 软团半径 ⇒ 连续水面。
    let mut vol = vxl_phys_terrain::voxel::VoxelVolume::new(
        Vec3::new(-1.25, 0.0, -1.25),
        0.5,
        5,
        3,
        5,
    );
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 0 || ix == 4 || iz == 0 || iz == 4 {
                vol.set(ix, 2, iz, true);
            }
        }
    }
    let voxel_id = w.add_voxel(vol);

    // ---- 水柱：0.25×0.25×0.65（6×6×14 = 504 粒），贴 −x 围堰立柱 ----
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.73, 1.05, -0.125),
        [6, 6, 14],
        0.05,
    );
    let _fluid = w.add_fluid(sys, &[voxel_id]);

    // ---- 转储（VXLD v3：单体积素 + 零喷溅 + 零网格 + 帧尾流体）----
    std::fs::create_dir_all("out").ok();
    let mut f = std::io::BufWriter::new(std::fs::File::create(&out_path).expect("open dump"));
    f.write_all(b"VXLD").unwrap();
    f.write_all(&VERSION.to_le_bytes()).unwrap();
    f.write_all(&(ticks as u32).to_le_bytes()).unwrap();
    f.write_all(&2u32.to_le_bytes()).unwrap(); // 每帧 2 tick
    f.write_all(&(1.0f32 / 60.0).to_le_bytes()).unwrap();

    // 体素体尺寸
    let (vox_dims, vox_origin, vox_step) = w
        .providers()
        .voxel(voxel_id)
        .map(|v| (v.dims(), v.origin(), v.step()))
        .unwrap();
    for x in [vox_dims.0, vox_dims.1, vox_dims.2] {
        f.write_all(&x.to_le_bytes()).unwrap();
    }
    for v in [vox_origin.x, vox_origin.y, vox_origin.z] {
        f.write_all(&v.to_le_bytes()).unwrap();
    }
    f.write_all(&vox_step.to_le_bytes()).unwrap();

    // 喷溅 0 颗 + 网格 0 张 + 流体 1 系统
    f.write_all(&0u32.to_le_bytes()).unwrap();
    f.write_all(&0u32.to_le_bytes()).unwrap();
    f.write_all(&(w.fluids().len() as u32).to_le_bytes()).unwrap();

    let mut ms_sum = 0f64;
    let mut ms_max = 0f64;
    for t in 1..=ticks {
        let t0 = std::time::Instant::now();
        w.step();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        ms_sum += ms;
        ms_max = ms_max.max(ms);
        if t % 2 != 0 {
            continue; // 每帧 = 2 tick
        }
        f.write_all(&(t as u32).to_le_bytes()).unwrap();
        f.write_all(&(ms as f32).to_le_bytes()).unwrap();
        f.write_all(&0u32.to_le_bytes()).unwrap(); // 刚体 0 个
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
        for (sys, _) in w.fluids() {
            let ps = sys.positions();
            f.write_all(&(ps.len() as u32).to_le_bytes()).unwrap();
            for p in ps {
                for v in [p.x, p.y, p.z] {
                    f.write_all(&v.to_le_bytes()).unwrap();
                }
            }
        }
    }
    f.flush().unwrap();
    let frames = ticks / 2;
    let fl = w.fluids()[0].0.positions();
    let (mut minx, mut maxx) = (f32::INFINITY, f32::NEG_INFINITY);
    let mut miny = f32::INFINITY;
    for p in fl {
        minx = minx.min(p.x);
        maxx = maxx.max(p.x);
        miny = miny.min(p.y);
    }
    println!(
        "转储完成：{out_path}（{frames} 帧 | {ticks} tick）\n\
         盆 {}×{}×{} 格 | 水柱 504 粒 → 终态 x∈[{minx:.2},{maxx:.2}] 最低 y={miny:.3}\n\
         单 tick 均值 {:.2} ms（{:.0} FPS）| 峰值 {:.2} ms",
        vox_dims.0, vox_dims.1, vox_dims.2,
        ms_sum / ticks as f64,
        1000.0 / (ms_sum / ticks as f64),
        ms_max,
    );
}
