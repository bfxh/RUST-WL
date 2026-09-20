//! 复合体 × 暖启动：**特征号带子序号**让求解器的"① 精确特征匹配"在复合体上生效
//! （球接触的 `feature` 恒为 0 ⇒ 若不标记子序号，同体对上的两条流形无法区分）。
//!
//! ⚠️ **已知仪器缺口（2026-09-20 实测）**：本测试**暂时标 ignore**——不是物理没生效，而是
//! **这套计数器在引擎路径上不动**。诊断读数（4 tick，落地后稳定接触）：
//! `流形=2 清醒体=2 计数=(0, 0, 0)` ⇒ 求解器在处理流形（双体清醒），但 `warm_match_stats`
//! 三个分支全零。原因定位：`vxl-phys-solver` 里 **两个构建器变体**（`lib.rs` 两个
//! `warm_index: &HashMap<…>` 签名处）只有**串行那套**写计数；`World::step` 走并行那套
//! ⇒ 引擎路径无读数，而窄相测试（`SerialJobSystem`）有读数。
//!
//! **要拿这个读数需先补一处**：把 `warm_match_stats` 的计数也接到并行构建器（或给并行路径
//! 提供一个串行回退的读数通道）。补完后本测试解禁，并作为"暖启动键改 `(a, b, space)`"
//! （`TECH-SURVEY.md` A9 ④）的判据：届时 `exact` 应从 0 变成"每条流形都精确命中"。
//!
//! **已知边界（本测试建立的语境）**：暖启动缓存条目键是"体对" ⇒ 同一体对的多条流形里
//! 只有一条能拿到暖启动，其余冷启动（安全：位置回退对不上 ⇒ 不暖，但不会用错冲量）。

use vxl_phys::{
    warm_match_stats_take, CompoundChild, FrictionModel, Material, PhysConfig, Quat, Shape, Vec3,
    World,
};

/// 见文件头：仪器缺口未补前本测试不可用。
#[ignore = "仪器缺口：warm_match_stats 只在串行构建器里计数，引擎路径无读数（见文件头）"]
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
