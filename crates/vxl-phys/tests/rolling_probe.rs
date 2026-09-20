//! 滚动探针：**多面化圆柱的"打摆"到底有多大？**（先算账，再决定要不要上解析接触）。
//!
//! 机理：圆柱碰撞体是 16 边内接棱柱（`CYLINDER_SEGMENTS = 16`），相邻**侧面法线相差
//! 360°/16 = 22.5°、相对竖直的最大偏角 11.25°** ⇒ 滚动时接触法线在面与面之间跳变，
//! 中心高度跟着起伏 `r·(1 − cos 11.25°)`。同半径**球**（解析支撑）作为对照基线。
//!
//! 判据（本探针只打印、不断言；读数决定"解析接触"这条大件值不值得做）：
//! - `法线偏角max`：圆柱应 ≈11°、球应 ≈0°；
//! - `y 起伏`：圆柱应 ≈7.7 mm（r=0.4）、球应 ≈0。

use vxl_phys::{FrictionModel, Material, PhysConfig, Quat, Shape, Vec3, World};

/// 让 `shape` 在盒地板上以"纯滚动"初条件前进，返回 (法线相对竖直的最大偏角, y 起伏)。
fn roll(shape: Shape, radius: f32) -> (f32, f32) {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(50.0, 0.5, 50.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 圆轴水平：局部 +Y（圆柱轴 / 球无所谓）转到 +X ⇒ 绕 X 滚动、沿 Z 前进。
    let rot = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
    let i = w.add_dynamic(shape, Vec3::new(0.0, radius, 0.0), rot, 1000.0) as usize;
    let mg = w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.0,
    });
    for b in 0..w.bodies.len() {
        w.bodies.set_material(b, mg);
    }
    // 纯滚动：v = ω × r ⇒ 绕 X 转 ω、沿 Z 走 ω·r。
    let omega = 4.0f32;
    w.bodies.set_angvel(i, Vec3::new(-omega, 0.0, 0.0));
    w.bodies.set_linvel(i, Vec3::new(0.0, 0.0, omega * radius));

    let mut max_tilt = 0.0f32;
    let mut y_min = f32::INFINITY;
    let mut y_max = f32::NEG_INFINITY;
    for t in 0..120 {
        w.step();
        if t < 10 {
            continue; // 落地/稳定段不计
        }
        for m in w.manifolds() {
            // 只看法向相对竖直的偏角（接触法线的"跳面"信号）。
            let tilt = m.normal.y.abs().clamp(-1.0, 1.0).acos().to_degrees();
            if tilt > max_tilt {
                max_tilt = tilt;
            }
        }
        let y = w.bodies.position[i].y;
        y_min = y_min.min(y);
        y_max = y_max.max(y);
    }
    (max_tilt, y_max - y_min)
}

#[test]
fn cylinder_roll_wobble_probe() {
    let (tilt_cyl, dy_cyl) = roll(
        Shape::Cylinder {
            half_height: 0.4,
            radius: 0.4,
        },
        0.4,
    );
    let (tilt_sph, dy_sph) = roll(Shape::Sphere { radius: 0.4 }, 0.4);
    println!(
        "圆柱(16 面): 法线偏角max {tilt_cyl:.2}° | y 起伏 {:.4} m",
        dy_cyl
    );
    println!(
        "球(解析支撑): 法线偏角max {tilt_sph:.2}° | y 起伏 {:.4} m",
        dy_sph
    );
    // 只作对照读数；打摆的"是否可接受"由人读数字后决定（不设断言，避免把待测量的量冻住）。
}
