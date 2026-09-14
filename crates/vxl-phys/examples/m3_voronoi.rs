//! M3 演示（headless）：**Voronoi 预断裂**——体素墙被切成 N 块刚体碎块后下落/沉降。
//!
//! 运行：`cargo run --release -p vxl-phys --example m3_voronoi [pieces] [ticks]`
//! 输出：断裂后的碎块数、逐 40 tick 的「动态体 / KE / 清醒 / 最深 / 逃逸」+ 末态汇总。

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Vec3};

fn main() {
    let mut args = std::env::args().skip(1);
    let pieces: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(240);

    let mut w = World::new(PhysConfig::default());
    // 地板（3 层）+ 待断裂的墙块（1 格厚 × 8 格高 × 8 格深）
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.5, 4.0));
    vol.fill_box(Vec3::new(0.0, 1.5, -2.0), Vec3::new(0.5, 5.5, 2.0));
    let block = vol.filled_count();
    w.add_voxel(vol);

    // 预断裂：确定性抖动种子（同参数 ⇒ 同结果）
    let seeds = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
        Vec3::new(0.0, 1.5, -2.0),
        Vec3::new(0.5, 5.5, 2.0),
        pieces,
        0.7,
    );
    let n = w.fracture_voronoi(
        0,
        Vec3::new(0.0, 1.5, -2.0),
        Vec3::new(0.5, 5.5, 2.0),
        &seeds,
        1000.0,
    );
    println!("场景：地板+墙 {block} 格 | 种子 {pieces} | 断裂出碎块 {n} 个 | {ticks} tick");
    println!("tick | 动态体 | KE(J) | 清醒 | 最深 | 逃逸");
    for t in 1..=ticks {
        w.step();
        if t % 40 == 0 || t == ticks {
            let (mut ke, mut dyn_n, mut awake, mut fallen) = (0.0f32, 0usize, 0usize, 0usize);
            for i in 0..w.bodies.len() {
                if w.bodies.is_dynamic(i) {
                    dyn_n += 1;
                    if w.bodies.awake[i] {
                        awake += 1;
                    }
                    if w.bodies.position[i].y < -1.0 {
                        fallen += 1;
                    }
                    let m = 1.0 / w.bodies.inv_mass[i].max(1e-12);
                    ke += 0.5 * m * w.bodies.linvel[i].length_squared();
                }
            }
            let h = w.health();
            println!(
                "{t:4} | {dyn_n:6} | {ke:8.1} | {awake:4} | {:.3} | {fallen}",
                h.max_depth
            );
        }
    }
    let h = w.health();
    println!(
        "== 汇总：碎块 {n} 个 | 末态干净 {} | 剩余体素 {} 格 | hash {:#x}",
        h.is_clean(),
        w.providers().voxel(0).unwrap().filled_count(),
        w.state_hash()
    );
}
