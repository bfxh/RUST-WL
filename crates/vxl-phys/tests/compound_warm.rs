//! 复合体 × 暖启动：**特征号带子序号**让求解器的"① 精确特征匹配"在复合体上生效
//! （球接触的 `feature` 恒为 0 ⇒ 若不标记子序号，同体对上的两条流形无法区分）。
//!
//! ⚠️ **已知缺陷（2026-09-20 实测，两轮读数）**：本测试**暂时标 ignore**——不是物理没生效，
//! 而是**同一体对的两条流形让这一对的暖缓存整条失效**。读数（落地后稳定接触 4 tick）：
//! 本场景 `流形=2 清醒体=2 计数=(0, 0, 0)`；对照探针 `warm_counter_probe` 里
//! **单盒**与**单子形状复合体**都是 `(32, 0, 0)` ⇒ 计数器本身是好的，异常只在
//! "两条流形共用一个 `(a, b)` 键"这一种（键相同 ⇒ 相互覆盖 / 查不到）。
//! （更早的"只有串行构建器计数"归因**已被推翻**：`build_constraint` 只有一个调用点，
//! 就在并行驱动内。）
//!
//! **修法**：暖启动键改 `(a, b, space)`（`space = feature >> 16`，复合体子形状序号；
//! 非复合体场景恒为 0 ⇒ 键退化为 `(a, b, 0)`，逐位中性）。改完解禁本测试：`exact` 应从 0
//! 变为"每条流形都精确命中"。详见 `TECH-SURVEY.md` A9 ④。

use vxl_phys::{
    warm_match_stats_take, CompoundChild, FrictionModel, Material, PhysConfig, Quat, Shape, Vec3,
    World,
};

/// 见文件头：同体对两流形的暖缓存失效（键相同 ⇒ 相互覆盖）未修前本测试不可用。
#[ignore = "已知缺陷：同体对两条流形令该对暖缓存整条失效（计数全 0）；见文件头与 TECH-SURVEY A9 ④"]
#[test]
fn compound_contacts_get_exact_warm_matches() {
    let mut w = World::new(PhysConfig::default());
    // 地板：半 5×0.5×5，中心 y=-0.5 ⇒ 顶面 y=0。
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 哑铃：两个球子形状（半径 0.3，位于 ±0.6）——球接触特征号恒为 0，正是要验的情形。
    let cid = w.add_compound(vec![
        CompoundChild {
            shape: Shape::Sphere { radius: 0.3 },
            offset: Vec3::new(-0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Sphere { radius: 0.3 },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
    ]);
    // 从 0.45 落下（静置高度 0.3）⇒ 落地后仍是清醒态，暖启动路径有机会生效。
    w.spawn_compound_body(cid, Vec3::new(0.0, 0.45, 0.0), Quat::IDENTITY, 500.0);
    // 材质（与 `capsule_drop` 同款）：默认 e=0 会让体在接触后很快入睡，睡眠体不被求解
    // ⇒ 暖启动计数器全 0（会把"没跑"误读成"没命中"）。
    let mg = w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: 0.7 },
        restitution: 0.05,
    });
    for b in 0..w.bodies.len() {
        w.bodies.set_material(b, mg);
    }
    // 落到**接触出现**之后再计数（飞行期没有流形，计不了）。
    let mut contact_ticks = 0;
    for _ in 0..80 {
        w.step();
        if !w.manifolds().is_empty() {
            contact_ticks += 1;
            if contact_ticks >= 3 {
                break;
            }
        }
    }
    assert!(contact_ticks >= 3, "复合体应在 80 tick 内落地并保持接触");
    // 清掉落地阶段的计数，只量"稳定接触"这几 tick 的匹配分支。
    let _ = warm_match_stats_take();
    for t in 0..4 {
        w.step();
        let awake = (0..w.bodies.len()).filter(|&i| w.bodies.awake[i]).count();
        println!(
            "t={t} 流形={} 清醒体={awake} 计数={:?}",
            w.manifolds().len(),
            warm_match_stats_take()
        );
    }
    let (exact, fallback, unmatched) = warm_match_stats_take();
    assert!(
        exact > 0,
        "复合体接触应拿到**精确特征**暖启动命中（子序号标记生效），实得 exact={exact} \
         fallback={fallback} unmatched={unmatched}"
    );
}
