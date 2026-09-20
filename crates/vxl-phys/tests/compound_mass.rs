//! 复合体质量属性：**精确并集**（按子形状求和 + 平行轴），不是并集 AABB 盒近似。
//!
//! 判据用"两球 + 横杆"哑铃（arena `shape-compound` 探针同形）：真体积 = 两球 + 长条，
//! 并集 AABB 盒体积约为它的 2.4 倍 ⇒ 两条路径的质量可区分（守住"别再退回近似"）。

use vxl_phys::{CompoundChild, PhysConfig, Quat, Shape, Vec3, World};

#[test]
fn compound_mass_is_exact_union() {
    let mut w = World::new(PhysConfig::default());
    let r = 0.3f32;
    let bar = Vec3::new(0.6, 0.1, 0.1);
    let cid = w.add_compound(vec![
        CompoundChild {
            shape: Shape::Sphere { radius: r },
            offset: Vec3::new(-0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Sphere { radius: r },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box { half: bar },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        },
    ]);
    let density = 500.0f32;
    let i = w.spawn_compound_body(cid, Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY, density) as usize;

    let v_true = 2.0 * (4.0 / 3.0) * std::f32::consts::PI * r * r * r + 8.0 * bar.x * bar.y * bar.z;
    let m_true = density * v_true;
    let m_got = 1.0 / w.bodies.inv_mass[i];
    assert!(
        (m_got - m_true).abs() / m_true < 1e-4,
        "质量应为子形状精确并集 {m_true}，实得 {m_got}"
    );

    // 并集 AABB 盒（旧近似）：明显更重——万一有人把覆写删了，这条会红。
    let bb = Vec3::new(1.8, 0.6, 0.6);
    let m_bb = density * bb.x * bb.y * bb.z;
    assert!(
        m_got < 0.6 * m_bb,
        "精确并集质量应显著小于并集 AABB 盒 {m_bb}，实得 {m_got}"
    );

    // 惯量关系（物理事实）：哑铃绕**长轴 x** 最容易转 ⇒ inv_x > inv_y。
    let inv = w.bodies.local_inv_inertia[i];
    assert!(inv.x > inv.y, "哑铃绕长轴应更易转，实得 inv={inv:?}");
}
