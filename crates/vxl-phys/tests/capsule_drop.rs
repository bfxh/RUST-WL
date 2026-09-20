//! 胶囊体落地静止（浏览器自检 `shape-capsule` 的等价完整场景）。
//!
//! 覆盖窄相单测之外的两半：**积分 + 求解 + 休眠全链**。原生静置高度 = half_height +
//! radius = 0.4 + 0.3 = 0.700（直立、零速、按岛级规则入睡）。
//!
//! 历史（2026-09-20 修复）：浏览器路径曾穿地（自检 15/4/0 → 15/3/1，接线回退）。定位链为
//! 桥的原生 example → 引擎级重现（本文件）→ 根因＝窄相相交支的 EPA + 支撑平面（`EXPERIMENTS.md`
//! R/R.1/R.2）。修复后本测试转为常开回归门。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 回归门（2026-09-20 解除 ignore）：曾因"静止期向下漂移 ⇒ 穿地"被标 ignore。根因在
/// **相交支**——EPA 对光滑支撑不适定 + 对方取**支撑平面**（大盒配微倾法线给出数十米偏移），
/// 见 `EXPERIMENTS.md` R.1/R.2。改成**解析最近点**（`capsule_axis_reach`）后末态
/// y=0.7000 恒定、法线 (0,1,0)、零速静止。
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
        // 判定窗口：落地前后（t≈55–75）逐 tick 打印接触细节与速度——
        // 「静止后向下漂移」的偏置方向问题在这一段显现（EXPERIMENTS 末节 R）。
        if (55..=70).contains(&t) {
            let m = w.manifolds();
            let mut info = String::from("无流形");
            for mm in m {
                let dmax = mm.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
                info = format!(
                    "n=[{:6.3},{:6.3},{:6.3}] 深度max={:8.4}",
                    mm.normal.x, mm.normal.y, mm.normal.z, dmax
                );
            }
            println!(
                "t={t:3} y={:8.4} vy={:8.4} 流形={} {}",
                w.bodies.position[i].y,
                w.bodies.linvel[i].y,
                m.len(),
                info
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
