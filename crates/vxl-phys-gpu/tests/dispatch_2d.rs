//! 二维分派的**字面量锁步**金丝雀。
//!
//! 放在独立集成测试文件里（而不是 `src/lib.rs` 的内联 `mod tests`）：上帝对象门对
//! **既有文件**"只准减"，把新测试塞进老文件会让它变胖 ⇒ 本仓惯例是**新文件只判阈值**。
//!
//! 判据：核里写死的展平步长必须等于 `probe::WG_X_CAP * 64`——两边漂移会让
//! "一维 ≡ 二维（逐粒索引不变）"的承诺悄悄失效，而且**只在大档上才暴露**。

/// 各核含展平步长字面量；`split_2d` 的形状是"装得下就一维、装不下才二维"。
#[test]
fn dispatch_2d_lockstep() {
    let stride = format!("{}u * 64u", vxl_phys_gpu::probe::WG_X_CAP);
    for (name, src) in [
        ("grid.wgsl", include_str!("../src/grid.wgsl")),
        ("density.wgsl", include_str!("../src/density.wgsl")),
        ("eos.wgsl", include_str!("../src/eos.wgsl")),
        ("force.wgsl", include_str!("../src/force.wgsl")),
        ("integrate.wgsl", include_str!("../src/integrate.wgsl")),
    ] {
        assert!(
            src.contains(&stride),
            "{name} 缺二维分派展平步长 `{stride}`（与 probe::WG_X_CAP 漂移了）"
        );
    }
    // 装得下就保持一维（既有档逐位不变），装不下才转二维。
    assert_eq!(vxl_phys_gpu::probe::split_2d(1), (1, 1));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65535), (65535, 1));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65536), (65535, 2));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65535 * 3), (65535, 3));
}
