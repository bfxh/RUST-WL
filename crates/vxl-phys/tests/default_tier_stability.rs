//! **默认档长跑稳定性门**（`EXPERIMENTS.md` 记过的验证覆盖缺口）。
//!
//! 背景：金样配方自带 `16` 迭代参数（`col45/pile5/tower25 ... 16 0.01 4 ...`），
//! 所以**开启/关闭"默认档"的改动，金样读数不会变**——默认档（`PhysConfig::default()`
//! = 2 子步 × 3 迭代 × 内层 1 = 6 扫掠）的稳定性此前**只能靠人肉跑 arena 长跑**
//! （`--steps 3000`）来守。本文件把它变成门禁链的一部分。
//!
//! 两个测试：
//! 1. `default_tier_stays_stable_on_long_run`——默认档跑 3000 步堆叠，冻结读数。
//! 2. `gate_is_sensitive_to_sweep_reduction`——**金丝雀**：把扫掠降到 4（`EXPERIMENTS`
//!    记录过的"地板以下"档）读数必须**明显不同**，否则说明本门的场景选得不够灵敏
//!    （门形同虚设）。
//!
//! 更新冻结值：只应在**有意**改动默认档时进行，并按 ADR 0004 记录换代理由。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 长跑读数（确定性：同配置同机同状态应逐位可复现）。
#[derive(Debug, Clone, Copy)]
struct Readings {
    /// 堆顶中心 y（塌了就掉）。
    top_y: f32,
    /// 末态 Σ|v|²（嗡振强度）。
    sum_v2: f32,
    /// 末态清醒体数。
    awake: usize,
    /// 末态流形数。
    manifolds: usize,
}

/// 自包含堆叠场景（与 `diag_min` 同族但**不依赖 example**）：`layers` 层 × `side`²，
/// 盒半长 0.25、间距 0.52（留 2 cm 缝）、盒地板。返回末态读数。
fn run_stack(cfg: PhysConfig, layers: usize, side: usize, ticks: usize) -> Readings {
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(20.0, 0.5, 20.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let spacing = 0.52f32;
    let off = (side as f32 - 1.0) * 0.5 * spacing;
    for layer in 0..layers {
        let y = 0.25 + layer as f32 * spacing;
        for row in 0..side {
            for col in 0..side {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * spacing - off, y, row as f32 * spacing - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    for _ in 0..ticks {
        w.step();
    }
    let mut top_y = f32::MIN;
    let mut sum_v2 = 0.0f32;
    let mut awake = 0usize;
    for i in 0..w.bodies.len() {
        if !w.bodies.is_dynamic(i) {
            continue;
        }
        top_y = top_y.max(w.bodies.position[i].y);
        sum_v2 += w.bodies.linvel[i].length_squared();
        if w.bodies.awake[i] {
            awake += 1;
        }
    }
    Readings {
        top_y,
        sum_v2,
        awake,
        manifolds: w.manifolds().len(),
    }
}

/// **默认档**（`PhysConfig::default()` = 6 扫掠/tick）长跑 3000 步。
///
/// 冻结值见测试体（2026-09-20 标定，同机同状态逐位可复现）。判定：三项读数都必须在
/// 容差内——堆顶塌了 / 动能爆了 / 流形数变了，都说明默认档的稳定性退化了。
#[test]
fn default_tier_stays_stable_on_long_run() {
    let r = run_stack(PhysConfig::default(), 6, 6, 3000);
    // 冻结读数（6×6×6 = 216 体、3000 步、默认档；2026-09-20 标定，逐位可复现）。
    // 注：`awake = 216`（全醒）**反映的是已知的睡眠缺口**，不是"目标值"——本门的作用是
    // 让默认档长跑行为的**任何**变化都必须是有意为之（改了就来更新这四个数并按 ADR 0004 记录）。
    let (top_y, sum_v2, awake, manifolds) = (2.7223, 2.1488, 216, 835);
    // 标定用：`cargo test -- --nocapture` 可读实际读数。
    println!(
        "[默认档 3000 步] top_y {:.4} | Σv² {:.4} | awake {} | manifolds {}",
        r.top_y, r.sum_v2, r.awake, r.manifolds
    );
    assert!(
        (r.top_y - top_y).abs() < 1e-3,
        "堆顶 y 漂移：{} vs 冻结 {}（默认档稳定性退化？）",
        r.top_y,
        top_y
    );
    assert!(
        (r.sum_v2 - sum_v2).abs() < 1e-3,
        "末态 Σv² 漂移：{} vs 冻结 {}（嗡振变强？）",
        r.sum_v2,
        sum_v2
    );
    assert_eq!(r.awake, awake, "末态清醒体数变了（入睡行为退化？）");
    assert_eq!(r.manifolds, manifolds, "末态流形数变了（接触行为退化？）");
}

/// **金丝雀**：本门必须对"扫掠降到地板以下"敏感——`EXPERIMENTS` 记录过 4 扫掠是塔/堆
/// 的稳定性地板（再低就垮）。若本测试失败，说明 `run_stack` 的场景选得不够灵敏，
/// 主测试的通过就没有意义。
#[test]
fn gate_is_sensitive_to_sweep_reduction() {
    // 2 子步 × 2 迭代 × 内层 1 = 4 扫掠（地板以下）。
    let cfg = PhysConfig {
        velocity_iterations: 2,
        ..PhysConfig::default()
    };
    let r = run_stack(cfg, 6, 6, 3000);
    let full = run_stack(PhysConfig::default(), 6, 6, 3000);
    println!(
        "[4 扫掠 3000 步] top_y {:.4} | Σv² {:.4} | awake {} | manifolds {}  ← 对照默认档 \
         top_y {:.4} | Σv² {:.4} | awake {}",
        r.top_y, r.sum_v2, r.awake, r.manifolds, full.top_y, full.sum_v2, full.awake
    );
    let diverged = (r.top_y - full.top_y).abs() > 1e-2
        || (r.sum_v2 - full.sum_v2).abs() > 1e-2
        || r.awake != full.awake;
    assert!(
        diverged,
        "4 扫掠与默认档读数相同（top_y {} vs {}、Σv² {} vs {}、awake {} vs {}）\
         ⇒ 本场景对扫掠数不敏感，主测试的门无效",
        r.top_y, full.top_y, r.sum_v2, full.sum_v2, r.awake, full.awake
    );
}
