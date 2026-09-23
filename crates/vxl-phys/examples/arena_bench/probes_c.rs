//! probes_c：从 arena_bench.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// **真实感基准（二）：斜面静摩擦**。μ 已知、倾角 θ：`tan θ < μ` 应静止，
/// `tan θ > μ` 应下滑。判据：静止档 240 步位移 < 0.02 m；下滑档位移 > 0.1 m。
///
/// 与 `slide`（水平面动摩擦）互补：这条测的是**静摩擦阈值**——"物体在斜面上
/// 慢慢出溜"是最常见的"不真实"观感之一。
pub(crate) fn scene_incline(cfg: PhysConfig) {
    let mu = 0.5f32;
    // 扫角度找**临界角** = atan(μ_eff)：判据是 240 步位移 0.02 m 分界。
    let mut critical = None;
    for deg in [12.0f32, 16.0, 20.0, 24.0, 28.0, 32.0, 36.0, 40.0] {
        let th = deg * std::f32::consts::PI / 180.0;
        let mut w = World::new(cfg.clone());
        ground(&mut w, 40.0);
        let m = mat(&mut w, mu, 0.0);
        let n = 6.0;
        let i = w.bodies.len();
        w.add_static(
            Shape::Box {
                half: Vec3::new(n, 0.5, n),
            },
            Vec3::new(0.0, 0.5, 0.0),
            vxl_phys_core::Quat::from_axis_angle(Vec3::Z, th),
        );
        w.bodies.set_material(i, m);
        let nrm = Vec3::new(-th.sin(), th.cos(), 0.0);
        // **落位必须算准**：斜面绕 Z 转 θ 后，其顶面过点
        // `center + R·(0,0.5,0) = (−0.5 sinθ, 0.5+0.5cosθ, 0)`，该点沿 nrm 到原点的
        // 距离是 `0.5(1+cosθ)`——**不是 0.5**。首版按 `nrm·(0.5+0.5)` 落位，盒心
        // 落在斜面**内部** 0.46 m ⇒ 测的是"从深穿透被顶出 + 滑走"，把静摩擦
        // 误判成失效（本会话第二次"测试场景自身违例"，上一次是关节探针用密度 0 当静态锚）。
        let surf = Vec3::new(-0.5 * th.sin(), 0.5 + 0.5 * th.cos(), 0.0);
        let p = surf + nrm * (0.5 + 0.03);
        let mb = mat(&mut w, mu, 0.0);
        let ib = w.bodies.len();
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            p,
            vxl_phys_core::Quat::from_axis_angle(Vec3::Z, th),
            800.0,
        );
        w.bodies.set_material(ib, mb);
        // 位移沿**斜面方向**量（避免把下沉计入）
        let tang = Vec3::new(th.cos(), th.sin(), 0.0);
        let p0 = w.bodies.position[ib];
        for _ in 0..240 {
            w.step();
        }
        let d = (w.bodies.position[ib] - p0).dot(tang);
        let held = d.abs() < 0.02;
        if held {
            critical = Some(deg);
        } else if critical.is_some() {
            println!(
                "  incline μ={mu}：临界角 {:.0}°（atan = {:.3}）⇒ **有效 μ ≈ {:.3}**（名义 {mu}）",
                deg,
                (deg * std::f32::consts::PI / 180.0).tan(),
                (deg * std::f32::consts::PI / 180.0).tan()
            );
            return;
        }
        println!(
            "  incline θ={deg:.0}°（tan θ={:.3}）：位移 {d:+.3} m → {}",
            th.tan(),
            if held { "静止" } else { "下滑" }
        );
    }
    if let Some(c) = critical {
        let th = c * std::f32::consts::PI / 180.0;
        println!(
            "  incline μ={mu}：扫完未见下滑，临界角 ≥ {c:.0}°（μ_eff ≥ {:.3}）",
            th.tan()
        );
    }
}
