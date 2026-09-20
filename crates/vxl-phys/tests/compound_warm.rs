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
//! ✅ **已修复（2026-09-20）**：暖启动键的第三维 = **显式特征空间**（`ContactPoints::space()`，
//! 窄相复合体展开时按子形状序号填；非复合体恒 0）⇒ 同体对两条流形不再相互覆盖。
//! 读数：本场景由 `(0, 0, 0)` 变为 `(4, 0, 0)`（4 tick × 2 子步 × 2 流形各 1 点）；
//! 对照探针 `warm_counter_probe` 保持 `(32, 0, 0)`；`default_tier_stability` 四个冻结读数
//! **逐位不变**（非复合体键退化为 `(a, b, 0)`）。
//!
//! ⚠️ **不要**用 `feature` 的高位当空间：窄相特征号高位被"侧别/裁剪路/哈希"编码占用，哈希逐帧
//! 漂移 ⇒ 普通场景暖启动随机失效（实测 `default_tier_stability` 的 top_y 漂 3 mm，已回退）。
//! 详见 `TECH-SURVEY.md` A9 ④。

use vxl_phys::{
    warm_match_stats_take, CompoundChild, FrictionModel, Material, PhysConfig, Quat, Shape, Vec3,
    World,
};

/// 回归门（2026-09-20 解除 ignore）：暖启动键已带**显式特征空间**（`ContactPoints::space()`，
/// 窄相复合体展开时按子形状序号填）⇒ 同体对两条流形不再相互覆盖。
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
    // 注：`warm_match_stats_take()` **会清零** ⇒ 循环里不能再调它（否则最后读到的总是 0；
    // 本文件先前正是踩了这条，读数被诊断打印吃掉，误判成"全 0"）。
    let _ = warm_match_stats_take();
    for _ in 0..4 {
        w.step();
    }
    let (exact, fallback, unmatched) = warm_match_stats_take();
    assert!(
        exact > 0,
        "复合体接触应拿到**精确特征**暖启动命中（子序号标记 + 显式空间通道生效），实得 \
         exact={exact} fallback={fallback} unmatched={unmatched}"
    );
}
