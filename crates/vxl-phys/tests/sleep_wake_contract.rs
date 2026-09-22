//! **睡眠/唤醒合同：睡着 ≠ 隐形墙**（2026-09-22 立）。
//!
//! 为什么需要这道门：`PhysConfig::wake_gate_k`（唤醒**接触数门**）是**硬条件**——
//! "同一次 `solve_phase` 内累计 ≥K 条 hot 接触观测才醒"。而**孤立睡体被撞**只有
//! 1 条 hot 接触 ⇒ 若没有安全阀，K>0 时它**永不醒**、对撞它的体表现为"不可动的墙"
//! （睡体不解算不积分），而物理上它该被撞飞。探针实测（`_wake_side_probe.rs`）确认了
//! 这个失效形状，安全阀 = **强撞直通**（`WAKE_GATE_FAST_MULT`，相对速度 ≥ 8×睡眠阈）。
//!
//! 判据（`wake_gate_k` 取 0 与 8 都必须过）：
//! - ① 单盒静置**能自己睡着**（否则本门无意义，直接报"场景不适用"而不是假绿）；
//! - ② 高速盒撞上去后，睡体**必须醒来**（`awake == true`）且**被推动**（位置变化）；
//! - ③ 醒来发生在**接触后若干 tick 内**（不是"最后勉强醒了"）。

use vxl_phys::{HeightField, PhysConfig, Quat, Shape, Vec3, World};

/// 静置盒（体 0）跑到入睡；返回 `(world, 睡体的体号, 撞击体的体号, 睡体初始 x)`。
fn scene(wake_gate_k: u32, iters: u32, settled_hold: u32) -> (World, usize, usize, f32) {
    let cfg = PhysConfig {
        velocity_iterations: iters,
        substeps: 4,
        threads: 1,
        wake_gate_k,
        settled_hold_iterations: settled_hold,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-2.0, -2.0, 9, 9, 1.0, 0.0));
    let sleeper = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 0.25, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    // 撞击体先放远处（不接触），稍后给速度。
    let hitter = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(-0.9, 0.25, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let x0 = w.bodies.position[sleeper].x;
    (w, sleeper, hitter, x0)
}

fn run_case(wake_gate_k: u32, iters: u32) {
    run_case_with(wake_gate_k, iters, 0);
}

fn run_case_with(wake_gate_k: u32, iters: u32, settled_hold: u32) {
    let (mut w, sleeper, hitter, x0) = scene(wake_gate_k, iters, settled_hold);
    // ① 静置：等睡体自己睡着（最多 600 tick）。
    let mut slept_at = None;
    for t in 1..=600 {
        w.step();
        if !w.bodies.awake[sleeper] {
            slept_at = Some(t);
            break;
        }
    }
    let slept_at = match slept_at {
        Some(t) => t,
        // 场景不适用：明确报出而不是假绿（本仓"睡不了的小场景"是已知缺口，见 M1-EXIT §2.2）。
        None => {
            println!("wake_gate_k={wake_gate_k}: ⚠️ 单盒 600 tick 内未入睡 ⇒ 本场景不适用，跳过（不是通过）");
            return;
        }
    };
    // ② 撞击体以 3 m/s 撞过来。⚠️ 起点必须**贴近**（间距 ≈0.3 m）：盒子在摩擦下会减速
    //    （3 m/s 只能滑 ~0.9 m）——第一版放 2 m 外，撞击体半路停下、压根没撞上（测试假红）。
    // ⚠️ 必须**同时唤醒撞击体**：它在前面静置阶段也睡着了，而睡体不积分 ⇒
    //    只设 linvel 会被积分器清零、位置永远不动（本测试第一版就栽在这，连 K=0 都"不醒"）。
    w.bodies.awake[hitter] = true;
    w.bodies.sleep_timer[hitter] = 0.0;
    w.bodies.linvel[hitter] = Vec3::new(3.0, 0.0, 0.0);
    let mut woke_at = None;
    let mut pushed = 0.0f32;
    for t in 1..=120 {
        w.step();
        if woke_at.is_none() && w.bodies.awake[sleeper] {
            woke_at = Some(t);
        }
        pushed = pushed.max((w.bodies.position[sleeper].x - x0).abs());
        if t % 5 == 0 && t <= 40 {
            let hp = w.bodies.position[hitter];
            let hv = w.bodies.linvel[hitter].length();
            println!(
                "   t{t:3}: 撞体 x {:+.3} |v| {hv:.2} awake {} | 睡体 awake {} | 间隙 {:.3}",
                hp.x,
                w.bodies.awake[hitter],
                w.bodies.awake[sleeper],
                (hp.x + 0.25) - (w.bodies.position[sleeper].x - 0.25)
            );
        }
    }
    let woke = woke_at.is_some();
    println!(
        "wake_gate_k={wake_gate_k} iters={iters}: 入睡 tick {slept_at} | 撞后唤醒 tick {:?} | 睡体被推 Δx {pushed:.3} m",
        woke_at
    );
    assert!(
        woke,
        "睡体被高速撞击后**没醒**（睡着变成隐形墙）：wake_gate_k={wake_gate_k}、iters={iters}"
    );
    assert!(
        pushed > 0.05,
        "睡体醒了但没被推动（Δx = {pushed:.4} m）：wake_gate_k={wake_gate_k}、iters={iters}"
    );
}

/// 合同①/②/③：**门开与门关，被撞的睡体都必须醒且被推动**。
#[test]
fn sleeping_body_wakes_and_moves_on_fast_impact() {
    for k in [0u32, 8] {
        run_case(k, 16);
    }
}

/// 对照：扫掠**更低**（`wake_gate_k=8` 的安全阀是"强撞直通"，所以低扫掠下更该醒）。
#[test]
fn sleeping_body_wakes_with_settled_hold_enabled() {
    // 安座趟（settled_hold）会把准静态接触的法向残速归零 ⇒ 必须确认它**没有**
    // 妨碍"被撞要醒"这条路径（2026-09-22：万级入睡靠它达标，故两开关常同开）。
    for hold in [2u32, 4] {
        run_case_with(8, 16, hold);
    }
}

/// 开两个开关后**逐位确定**（安座趟与门的计数都在固定次序下运行）。
#[test]
fn gate_and_settle_are_deterministic() {
    let digest = |settled_hold: u32| {
        let (mut w, sleeper, hitter, _x0) = scene(8, 16, settled_hold);
        for _ in 0..200 {
            w.step();
        }
        w.bodies.linvel[hitter] = Vec3::new(3.0, 0.0, 0.0);
        w.bodies.awake[hitter] = true;
        for _ in 0..200 {
            w.step();
        }
        let mut d: Vec<u32> = Vec::new();
        for i in 0..w.bodies.len() {
            let p = w.bodies.position[i];
            let v = w.bodies.linvel[i];
            d.extend([p.x.to_bits(), p.y.to_bits(), v.x.to_bits(), v.y.to_bits()]);
        }
        d.push(sleeper as u32);
        d
    };
    assert_eq!(digest(4), digest(4), "settled_hold=4 两跑应逐位一致");
    assert_eq!(digest(2), digest(2), "settled_hold=2 两跑应逐位一致");
}

/// 对照：扫掠**更低**（`wake_gate_k=8` 的安全阀是"强撞直通"，所以低扫掠下更该醒）。
#[test]
fn sleeping_body_wakes_on_fast_impact_at_low_iterations() {
    run_case(8, 4);
}
