//! 质量属性（§2.1）：解析惯性张量（盒/球/圆柱），复合体平行轴叠加留给 M1 的复合刚体。

use crate::math::Vec3;
use crate::shape::Shape;

#[derive(Clone, Copy, Debug)]
pub struct MassProps {
    pub mass: f32,
    pub inv_mass: f32,
    /// 局部（体轴）逆惯性张量对角线（盒/球/圆柱主轴均与体轴对齐）。
    pub local_inv_inertia: Vec3,
}

/// 按形状 + 密度计算质量属性。`density <= 0` 视为非法，回退 1.0。
pub fn mass_props(shape: &Shape, density: f32) -> MassProps {
    let density = if density > 0.0 { density } else { 1.0 };
    match *shape {
        Shape::Box { half } => {
            let ex = 2.0 * half.x;
            let ey = 2.0 * half.y;
            let ez = 2.0 * half.z;
            let m = density * ex * ey * ez;
            let ix = m / 12.0 * (ey * ey + ez * ez);
            let iy = m / 12.0 * (ex * ex + ez * ez);
            let iz = m / 12.0 * (ex * ex + ey * ey);
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::new(1.0 / ix, 1.0 / iy, 1.0 / iz),
            }
        }
        Shape::Sphere { radius } => {
            let m = density * 4.0 / 3.0 * core::f32::consts::PI * radius * radius * radius;
            let i = 2.0 / 5.0 * m * radius * radius;
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::splat(1.0 / i),
            }
        }
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let h = 2.0 * half_height;
            let m = density * core::f32::consts::PI * radius * radius * h;
            // 体轴 Y 为圆柱主轴。
            let iy = 0.5 * m * radius * radius;
            let ixz = m / 12.0 * (3.0 * radius * radius + h * h);
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::new(1.0 / ixz, 1.0 / iy, 1.0 / ixz),
            }
        }
        // 圆锥：实心锥（高 H = 2·half_height，m = ρ·πr²H/3）。惯量按**关于体原点**给
        // （本仓无质心偏移字段）：绕轴 3/10·m·r²；横向 = 质心项 (3/20·m·r² + 3/80·m·H²)
        // + 平行轴 m·(H/4)² = m·(0.15·r² + 0.40·h²)。⚠️ 锥质心在底面上方 H/4，与体原点
        // 不重合（见 `shape.rs` 的说明）。
        Shape::Cone {
            half_height,
            radius,
        } => {
            let h = 2.0 * half_height;
            let m = density * core::f32::consts::PI * radius * radius * h / 3.0;
            let iy = 0.3 * m * radius * radius;
            let ixz = m * (0.15 * radius * radius + 0.40 * half_height * half_height);
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::new(1.0 / ixz, 1.0 / iy, 1.0 / ixz),
            }
        }
        // 胶囊体：中段圆柱 + 两片半球（闭式解；退化自检：h→0 给出球 2/5·m·r²、
        // r→0 给出细杆 m·h²/3 ⇒ 两端极限都对）。
        Shape::Capsule {
            half_height,
            radius,
        } => {
            let (h, r) = (half_height, radius);
            let pi = core::f32::consts::PI;
            let m_c = density * pi * r * r * 2.0 * h; // 中段圆柱
            let m_h = density * (2.0 / 3.0) * pi * r * r * r; // 单片半球
            let m = m_c + 2.0 * m_h;
            // 轴向（Y）：圆柱 ½m_c r² + 两半球各 (2/5)m_h r²。
            let iy = 0.5 * m_c * r * r + 0.8 * m_h * r * r;
            // 横向：柱绕心 + 两半球「绕自身质心 83/320·r² + 平行轴到 h+3r/8」。
            let i_hemi_own = m_h * r * r * (83.0 / 320.0);
            let d = h + 3.0 * r / 8.0;
            let ixz = m_c * (3.0 * r * r + 4.0 * h * h) / 12.0 + 2.0 * (i_hemi_own + m_h * d * d);
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::new(1.0 / ixz, 1.0 / iy, 1.0 / ixz),
            }
        }
        // 凸体外壳：按局部 AABB 盒惯量近似（点云实惯量待 M1 复合体）。
        Shape::ConvexHull { half, .. } => {
            let ex = 2.0 * half.x.max(1e-4);
            let ey = 2.0 * half.y.max(1e-4);
            let ez = 2.0 * half.z.max(1e-4);
            let m = density * ex * ey * ez;
            let ix = m / 12.0 * (ey * ey + ez * ez);
            let iy = m / 12.0 * (ex * ex + ez * ez);
            let iz = m / 12.0 * (ex * ex + ey * ey);
            MassProps {
                mass: m,
                inv_mass: 1.0 / m,
                local_inv_inertia: Vec3::new(1.0 / ix, 1.0 / iy, 1.0 / iz),
            }
        }
        // 高度场 / 外部 provider 只作为静态地形存在，不参与质量属性。
        Shape::HeightField(_) | Shape::Provider(_) => MassProps {
            mass: 0.0,
            inv_mass: 0.0,
            local_inv_inertia: Vec3::ZERO,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_inertia_matches_formula() {
        let h = Vec3::new(0.5, 1.0, 2.0);
        let p = mass_props(&Shape::Box { half: h }, 2.0);
        let m = 2.0 * 8.0 * h.x * h.y * h.z;
        assert!((p.mass - m).abs() < 1e-4);
        let ix = m / 12.0 * ((2.0 * h.y) * (2.0 * h.y) + (2.0 * h.z) * (2.0 * h.z));
        assert!((1.0 / p.local_inv_inertia.x - ix).abs() < 1e-3);
    }

    #[test]
    fn sphere_inertia() {
        let p = mass_props(&Shape::Sphere { radius: 1.0 }, 1.0);
        let m = 4.0 / 3.0 * core::f32::consts::PI;
        assert!((p.mass - m).abs() < 1e-4);
        assert!((1.0 / p.local_inv_inertia.x - 2.0 / 5.0 * m).abs() < 1e-4);
    }

    #[test]
    fn cylinder_axis_is_y() {
        let p = mass_props(
            &Shape::Cylinder {
                half_height: 1.0,
                radius: 0.5,
            },
            1.0,
        );
        // Iy = 1/2 m r^2 < Ix —— 绕主轴更容易转。
        assert!(p.local_inv_inertia.y > p.local_inv_inertia.x);
    }
}
