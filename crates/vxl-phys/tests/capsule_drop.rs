//! 胶囊体落地静止（浏览器自检 `shape-capsule` 的等价完整场景）。
//!
//! 覆盖窄相单测之外的两半：**积分 + 求解 + 休眠全链**。原生实测：落点 y=0.695
//! （= half_height + radius = 0.4 + 0.3，直立静止）、速度归零、随后按岛级规则入睡。
//!
//! ⚠️ 已知缺口（2026-09-20）：**浏览器路径会穿地**（接线后自检 15/4/0 → 15/3/1，
//! 已回退接线）。本测试在原生下通过 ⇒ 差异在 **wasm 桥/适配层**那一侧，不是引擎物理。
//! 下一步的第一件仪器：给 `physarena/wasm-bridge` 加一个原生 example，把桥的
//! `add_body`/`vxl_step` 路径在本机跑一遍（同一份桥代码），逐层定位。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

#[test]
fn capsule_drop_trace() {
    let mut w = World::new(PhysConfig::default());
    // 地板：与 arena `ground(60, 1, 0)` 同几何（半 30×1×30，中心 y=-1 ⇒ 顶面 y=0）。
    w.add_static(
        Shape::Box {
            half: Vec3::new(30.0, 1.0, 30.0),
        },
        Vec3::new(0.0, -1.0, 0.0),
        Quat::IDENTITY,
    );
    let i = w.add_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    for t in 1..=180 {
        w.step();
        if t % 15 == 0 || (w.bodies.position[i].y < 1.0 && t % 3 == 0) {
            println!(
                "t={t:3} y={:8.3} v={:7.3} 流形={}",
                w.bodies.position[i].y,
                w.bodies.linvel[i].length(),
                w.manifolds().len()
            );
        }
    }
    let y = w.bodies.position[i].y;
    println!("末态 y={y:.3}");
    assert!(
        (0.26..=0.78).contains(&y),
        "胶囊应静止在 y∈[0.26,0.78]，实得 {y:.3}（穿地 ⇒ 接触没建立/没生效）"
    );
}
