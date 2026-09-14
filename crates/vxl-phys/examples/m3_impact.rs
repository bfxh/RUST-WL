//! M3 冲击破坏演示（headless）：体素墙 + 高速炮弹 ⇒ 接触点挖洞 + 碎块。
//!
//! 运行：`cargo run --release -p vxl-phys --example m3_impact [ticks] [speed]`
//! 输出：逐 20 tick 的「墙体素 / 碎块数 / 动态体数 / KE / 清醒数」+ 末态汇总。
//! 说明：本仓无可视化（headless 引擎）；本例即「实际演示」的文本形态——
//! 破坏管线的每一步（挖洞、碎块生成、碎块再碰撞、入睡）都在数字里可见。
//!
//! **已知限制（如实）**：挖出的碎块若与残余结构重叠，会被位置修正挤出并可能
//! 隧穿逃逸（汇总里的「逃逸」数；KE 前几名通常是逃逸块的自由落体）。修法候选：
//! 只挖「完全落在挖域内的格」、碎块生成给出微小外偏、或对小碎块走「无碎块销毁」。

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    let speed: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(8.0);
    // 触发阈值（m/s）：低阈值 ⇒ 碎块自己落地也触发 ⇒ 级联挖穿地板（实测教训）
    let threshold: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(8.0);

    // **CCD 默认关**（保持引擎默认）：已实测「CCD × 静止接触」会互相打架——
    // 贴地滑行的体每个扫描采样都判定为命中 ⇒ 被钳回起点、原地锁死（真 bug，
    // 判据需细化为「只对**新出现的/更深的**接触钳位」）。因此本例默认速度取
    // 8 m/s（0.13 m/tick < 皮肤带 × 格边距 ⇒ 无隧穿），无需 CCD 也能跑干净。
    let mut w = World::new(PhysConfig::default());
    // 地板（整幅 1 层）+ 墙（1 格厚、4 格高、8 格深）
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    // 地板 3 层厚（1.5 m）：薄地板会被挖穿 ⇒ 碎块掉出世界、KE 无界（实测教训）
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.5, 4.0));
    vol.fill_box(Vec3::new(0.0, 1.5, -2.0), Vec3::new(0.5, 3.5, 2.0));
    let filled0 = vol.filled_count();
    w.add_voxel(vol);

    // 炮弹
    let bullet = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(-3.0, 2.0, 0.0),
        Quat::IDENTITY,
        2000.0,
    );
    w.bodies.linvel[bullet as usize] = Vec3::new(speed, 0.0, 0.0);

    println!(
        "场景：体素墙 {} 格（初始）| 炮弹 {speed} m/s | {ticks} tick | 阈值 {threshold} m/s",
        filled0
    );
    println!("tick | 墙体素 | 碎块累计 | 动态体 | KE(J) | 清醒 | 最深");
    let mut debris_total = 0usize;
    let mut first_hit = None;
    for t in 1..=ticks {
        w.step();
        let d = w.apply_impact_destruction(0, 3.0, 1000.0);
        if d > 0 && first_hit.is_none() {
            first_hit = Some(t);
        }
        debris_total += d;
        if t % 20 == 0 || t == ticks {
            let filled = w.providers().voxel(0).unwrap().filled_count();
            let mut ke = 0.0f32;
            let mut dyn_n = 0usize;
            let mut awake = 0usize;
            for i in 0..w.bodies.len() {
                if w.bodies.is_dynamic(i) {
                    dyn_n += 1;
                    if w.bodies.awake[i] {
                        awake += 1;
                    }
                    let m = 1.0 / w.bodies.inv_mass[i].max(1e-12);
                    ke += 0.5 * m * w.bodies.linvel[i].length_squared();
                }
            }
            let h = w.health();
            // 世界外（掉出）计数：KE 无界增长的常见成因是「掉出世界」而非注入
            let mut fallen = 0usize;
            for i in 0..w.bodies.len() {
                if w.bodies.is_dynamic(i) && w.bodies.position[i].y < -1.0 {
                    fallen += 1;
                }
            }
            let _ = fallen;
            println!(
                "{t:4} | {:6} | {:8} | {:6} | {:7.1} | {:4} | {:.3}",
                filled, debris_total, dyn_n, ke, awake, h.max_depth
            );
        }
    }
    // KE 前 5 体诊断（质量/速度）——用于判别「注入」vs「重碎块自由落体」
    {
        let mut top: Vec<(f32, f32, f32, Vec3)> = Vec::new();
        for i in 0..w.bodies.len() {
            if w.bodies.is_dynamic(i) {
                let m = 1.0 / w.bodies.inv_mass[i].max(1e-12);
                let v = w.bodies.linvel[i];
                top.push((
                    0.5 * m * v.length_squared(),
                    m,
                    v.length(),
                    w.bodies.position[i],
                ));
            }
        }
        top.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (ke, m, sp, pos) in top.iter().take(5) {
            println!(
                "KE-top: {ke:>12.1} J | m={m:>10.1} kg | |v|={sp:>7.2} | pos=({:.2},{:.2},{:.2})",
                pos.x, pos.y, pos.z
            );
        }
    }
    let filled1 = w.providers().voxel(0).unwrap().filled_count();
    let h = w.health();
    let mut fallen = 0usize;
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) && w.bodies.position[i].y < -1.0 {
            fallen += 1;
        }
    }
    println!(
        "== 汇总：挖掉 {} 格（{} → {}）| 碎块 {} 个 | 逃逸(掉出世界) {} 个 | 首次命中 tick {:?} | 末态干净 {} | hash {:#x}",
        filled0 - filled1,
        filled0,
        filled1,
        debris_total,
        fallen,
        first_hit,
        h.is_clean(),
        w.state_hash()
    );
}
