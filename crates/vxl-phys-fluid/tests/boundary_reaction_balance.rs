//! 2b 反作用的**动量大账**守卫：流体受的成对力与边界粒子受的反作用**等大反向**。
//!
//! 为什么值得单独钉：GPU 侧的"反作用回读"（`docs/PLAN-gpu.md` §13.2）打算用 **gather** 形式
//! 重写这条力，它依赖的性质正是"成对力对称——两边都带乘积质量因子 `m_i·m_j`"。先在 CPU 侧把这个
//! 实施的量出来，GPU 侧照同一条写时才有可比对象；也免得"体被往反方向推"这类符号错在**真耦合**里
//! 才被发现（那时很难一眼看出是符号问题）。
//!
//! 判据：**零重力 + 无提供者（壁面）**时，除成对力外没有别的力 ⇒
//! `Σ_流体 m·a + Σ_边界 bforce ≈ 0`（两个场取自**同一子步**）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

#[test]
fn fluid_pair_forces_balance_boundary_reactions() {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    let s = 0.05f32;
    // 4³ 流体块（y ∈ [0.2, 0.35]）+ 一个"体"（盒，底面贴在流体顶面之上一个晶格间距内 ⇒ 有真实成对力）。
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [4, 4, 4], s);
    let body = (
        0u32,
        Shape::Box {
            half: Vec3::splat(0.15),
        },
        BodyPose {
            pos: Vec3::new(0.0, 0.55, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    );
    let nb = f.set_boundary_particles(&[body]);
    assert!(nb > 0, "应当造出边界粒子");
    assert_eq!(f.boundary_forces().len(), nb, "反作用数组长度 = 边界粒子数");
    for _ in 0..3 {
        f.step(1.0 / 60.0, &NoProviders);
    }

    let m = f.particle_mass();
    let mut sum = Vec3::ZERO;
    let mut scale = 0.0f32;
    for a in f.accelerations() {
        let fa = *a * m;
        sum += fa;
        scale += fa.length();
    }
    for b in f.boundary_forces() {
        sum += *b;
    }
    // 先保证用例**非平凡**（否则下面的断言退化成空断言）。
    assert!(
        scale > 0.0 && f.boundary_forces().iter().any(|b| b.length() > 0.0),
        "本用例要有非零的成对力才有意义：流体侧标尺 {scale}、边界侧最大 {}",
        f.boundary_forces()
            .iter()
            .map(|b| b.length())
            .fold(0.0f32, f32::max)
    );
    // 残差相对**流体侧标尺**：成对力严格等大反向 ⇒ 只剩浮点累加序的残差（口径 B 同源）。
    let rel = sum.length() / scale;
    eprintln!(
        "2b 动量大账：{} 流体 + {} 边界 | Σ|m·a| = {scale:.6e} N | |Σ m·a + Σ bforce| = {:.3e} N | 相对 {rel:.2e}",
        f.accelerations().len(),
        f.boundary_forces().len(),
        sum.length()
    );
    assert!(
        rel < 1e-3,
        "成对力必须等大反向：|Σ m·a + Σ bforce| / Σ|m·a| = {rel}（Σ = {:?}）",
        sum
    );
}
