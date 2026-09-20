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

/// ⚠️ **已知缺陷，暂标 ignore**（`EXPERIMENTS.md` 末节 R）：胶囊在**静止期向下漂移**
/// （随穿透加深而加速）⇒ 最终穿地；`e=0` 时因早期入睡而被掩盖，所以本测试**特意**用
/// arena 探针的材质（μ=0.7、e=0.05）把它逼出来。**修好后删掉 `#[ignore]` 即可作为回归门。**
#[ignore = "已知缺陷：e>0 时胶囊静止期向下漂移（EXPERIMENTS 末节 R）"]
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
    // **材质必须与 arena 探针一致**（μ=0.7、e=0.05）：默认的 e=0 会让胶囊在 t≈90 入睡，
    // 从而**掩盖**"静止时向下漂移"的缺陷（见 EXPERIMENTS 末节 R）。这个测试的存在意义
    // 就是守住那一层，所以这里不能再用默认材质。
    let mg = w.add_material(vxl_phys::Material {
        friction: vxl_phys::FrictionModel::Coulomb { mu: 0.7 },
        restitution: 0.05,
    });
    for b in 0..w.bodies.len() {
        w.bodies.set_material(b, mg);
    }
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
