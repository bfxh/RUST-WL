//! tests：从 `lib.rs` 拆出的单元测试（纯搬移 + 去一层缩进）。

use super::*;

use super::*;
use vxl_phys_core::{BodySet, SerialJobSystem};

fn manifolds_for(b: &BodySet, hf: &[HeightField]) -> Vec<Manifold> {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 全对暴力（测试用）。
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        b,
        &pairs,
        hf,
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    out
}

/// 胶囊 × 盒地板（竖直）：接触判据 = **线段端点到地面的距离 < radius**（线段本身
/// 并未碰到地面）；应给出 1 个接触点、法线向上、深度 ≈ 压入量。
#[test]
fn capsule_vs_box_floor_vertical() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 下帽端点 y = y0 − 0.4；下帽表面 = 端点 − 0.3 ⇒ 压住地面 1 cm 时 y0 = 0.69。
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.69, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert_eq!(
        out.len(),
        1,
        "应有 1 个流形（胶囊 × 地板），实得 {}",
        out.len()
    );
    let m = &out[0];
    // 法线约定 **a→b**：期望符号由流形自身推导（别假定地板一定是 `a`——
    // 配对索引是 `(i, j)` 生成序，而本测试的 push 序不保证与之对应）。
    let floor_is_a = m.a == 0;
    let want = if floor_is_a { 1.0 } else { -1.0 };
    assert!(
        m.normal.y * want > 0.99,
        "法线应指向 a→b（地板→胶囊），a={} b={} 实得 {:?}",
        m.a,
        m.b,
        m.normal
    );
    assert_eq!(
        m.points.len(),
        1,
        "竖直胶囊只有下帽接触，实得 {} 点",
        m.points.len()
    );
    let d = m.points[0].depth;
    assert!(
        (d - 0.01).abs() < 2e-3,
        "深度应 ≈1 cm（0.3 − 端点距 0.29），实得 {d}"
    );
}

/// 胶囊 × 地形：此前高度场分支**显式拒绝**本组合 ⇒ 静默无接触。
/// 现沿中心线取 5 个样本、每个按球处理。判据：有接触、法线竖直、压入 ≈1 cm。
#[test]
fn capsule_on_heightfield() {
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    // 竖直胶囊：下端点 y0 − 0.4、帽面再 −0.3 ⇒ 压入 1 cm 时 y0 = 0.69。
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.69, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert!(!m.is_empty(), "竖直胶囊 × 地形应有接触（此前为静默无接触）");
    assert!(
        m[0].normal.y.abs() > 0.99,
        "法线应竖直，实得 {:?}",
        m[0].normal
    );
    let dmax = m[0].points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!((dmax - 0.01).abs() < 5e-3, "压入应 ≈1 cm，实得 {dmax}");
}

/// 外壳 × 地形：此前 `hull_pair` 明确不受理（"列裁剪对任意凸壳未实现"）⇒ 静默无接触。
/// 现走**逐顶点采样**（与 `poly_heightfield` 同款）。判据：有接触、法线竖直、正压入。
#[test]
fn hull_on_heightfield() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 外壳：3×3×3 立方点云（半 0.3）。
    let mut pts: Vec<Vec3> = Vec::with_capacity(27);
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
            }
        }
    }
    let hid = np.add_hull(pts);
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    // 底面压入地面（y=0）1 cm ⇒ 中心 y = 0.29。
    b.push_dynamic(
        Shape::ConvexHull {
            hull: hid,
            half: Vec3::splat(0.3),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[hf],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(!out.is_empty(), "外壳 × 地形应有接触（此前为静默无接触）");
    let m = &out[0];
    assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
}

/// 外壳 × 圆柱：曾因 `support_of` 缺圆柱/锥分支而**静默无接触**（`TECH-SURVEY.md` A9 ④ 留档）。
/// 判据：有接触、法线竖直、有正压入。
#[test]
fn hull_on_cylinder_cap() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 外壳：3×3×3 立方点云（半 0.3）。
    let mut pts: Vec<Vec3> = Vec::with_capacity(27);
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
            }
        }
    }
    let hid = np.add_hull(pts);
    let mut b = BodySet::new();
    // 静立圆柱（半高 0.5、半径 0.4，中心 y=-0.5 ⇒ 顶面 y=0）。
    b.push_static(
        Shape::Cylinder {
            half_height: 0.5,
            radius: 0.4,
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 外壳落在顶面（底面压入 1 cm ⇒ 中心 y = 0.29）。
    b.push_dynamic(
        Shape::ConvexHull {
            hull: hid,
            half: Vec3::splat(0.3),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(!out.is_empty(), "外壳 × 圆柱应有接触（此前为静默无接触）");
    let m = &out[0];
    assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
}

/// 圆锥 × 盒地板（坐底）：底圆盘多面化 ⇒ 应给出**多点**支撑（单点会晃）、法线竖直、
/// 最深压入 ≈ 压入量。
#[test]
fn cone_on_box_floor() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 底面在 y0 − h；要让底圆盘压入地面（y=0）1 cm ⇒ y0 = 0.5 − 0.01 = 0.49。
    b.push_dynamic(
        Shape::Cone {
            half_height: 0.5,
            radius: 0.4,
        },
        Vec3::new(0.0, 0.49, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert!(!out.is_empty(), "圆锥坐底应有接触");
    let m = &out[0];
    let floor_is_a = m.a == 0;
    let want = if floor_is_a { 1.0 } else { -1.0 };
    assert!(
        m.normal.y * want > 0.99,
        "法线应竖直（a→b），实得 {:?}",
        m.normal
    );
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!((dmax - 0.01).abs() < 3e-3, "最深压入应 ≈1 cm，实得 {dmax}");
    assert!(
        m.points.len() >= 3,
        "坐底应为多点支撑（多点才不摇），实得 {} 点",
        m.points.len()
    );
}

/// 复合体（哑铃：两端盒 + 中间横杆）坐地：**每个接触的子形状各出一条流形**（≥2 条），
/// 且特征号按**子序号左移 16 位**编码 ⇒ 不同子形状的特征空间互不重叠（暖缓存不串号）。
#[test]
fn compound_dumbbell_on_floor() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 两端用**盒**（而非球）：盒-盒接触带 `feature`，才能验到"子序号并入特征号"这条路径
    // （球接触的 `feature` 恒为 0 = "无特征"哨兵，标记不碰它）。
    let cid = np.add_compound(vec![
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(-0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::new(0.6, 0.1, 0.1),
            },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        },
    ]);
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 两球半径 0.3、横杆 1.2×0.2×0.2；球压入地面（y=0）1 cm ⇒ y0 = 0.29。
    b.push_dynamic(
        Shape::Compound {
            compound: cid,
            half: np.compound_half_extents(cid),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(
        out.len() >= 2,
        "两个球应各出一条流形（同体对多条），实得 {} 条",
        out.len()
    );
    let mut tags = std::collections::BTreeSet::new();
    for m in &out {
        assert!(
            m.normal.y.abs() > 0.99,
            "地面接触法线应竖直，实得 {:?}",
            m.normal
        );
        let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!((dmax - 0.01).abs() < 3e-3, "压入应 ≈1 cm，实得 {dmax}");
        for p in m.points.iter() {
            assert!(
                p.feature >> 16 != 0,
                "特征号应带子序号标记，实得 {}",
                p.feature
            );
            tags.insert(p.feature >> 16);
        }
    }
    assert!(
        tags.len() >= 2,
        "不同子形状的特征空间应互不相同，实得 {tags:?}"
    );
}

/// 平躺胶囊：应给出**两个**接触点（两帽各一）——这是它稳定静置（不摇）的前提。
#[test]
fn capsule_flat_gives_two_points() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 绕 Z 转 90° ⇒ 局部 +Y 变成世界 +X ⇒ 胶囊水平平躺，压入 1 cm（0.3 − 0.29）。
    let rot = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.29, 0.0),
        rot,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert_eq!(out.len(), 1, "应有 1 个流形，实得 {}", out.len());
    assert_eq!(
        out[0].points.len(),
        2,
        "平躺胶囊应给 2 点支撑，实得 {}",
        out[0].points.len()
    );
}

#[test]
fn sphere_sphere_touch() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::ZERO,
        Quat::IDENTITY,
        1.0,
    );
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.9, 0.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!((m[0].normal.x - 1.0).abs() < 1e-5);
    assert!((m[0].points[0].depth - 0.1).abs() < 1e-5);
}

#[test]
fn sphere_above_box_normal_points_down() {
    let mut b = BodySet::new();
    // 球心在盒顶上方 0.4 → 穿透深度 = r - 0.4 = 0.1。
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    // a=球 在上，b=盒 → 法线 a→b 朝下。
    assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
    assert!((m[0].points[0].depth - 0.1).abs() < 1e-4);
}

#[test]
fn box_box_resting_manifold() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::new(0.5, 0.5, 0.5),
        },
        Vec3::new(0.0, 0.95, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!(m[0].normal.y < -0.99);
    assert!(!m[0].points.is_empty() && m[0].points.len() <= 4);
    assert!(m[0].points[0].depth > 0.0 && m[0].points[0].depth < 0.06);
}

#[test]
fn box_penetrating_deep_gives_points() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.7, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert_eq!(m[0].points.len(), 4);
}

#[test]
fn cylinder_on_ground() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Cylinder {
            half_height: 0.5,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!(m[0].normal.y < -0.99);
}

#[test]
fn sphere_on_heightfield() {
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 0.45, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    // a=球(0) 在上，b=marker(1) → 法线 a→b = -Y（推向地面）。
    assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
    assert!((m[0].points[0].depth - 0.05).abs() < 0.02);
}

/// 复合体 × 地形：子形状各自走地形路径（此前该组合在高度场分支被**显式拒绝**）。
/// 判据：两个球子形状各给出一条流形、法线竖直（含接触）。
#[test]
fn compound_on_heightfield() {
    let mut np = DefaultNarrowPhase::new(0.01);
    let cid = np.add_compound(vec![
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
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    b.push_dynamic(
        Shape::Compound {
            compound: cid,
            half: np.compound_half_extents(cid),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[hf],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(
        out.len() >= 2,
        "两个球子形状应各出一条地形流形，实得 {} 条",
        out.len()
    );
    for m in &out {
        assert!(
            m.normal.y.abs() > 0.99,
            "地形法线应竖直，实得 {:?}",
            m.normal
        );
        // 球子形状的特征号恒为 0 ⇒ 子序号标记必须对**全部**点位生效，否则两个子形状
        // 的接触点在同一个体对（暖启动缓存键）上无法区分。
        for p in m.points.iter() {
            assert!(
                p.feature >> 16 != 0,
                "地形接触也应带子序号标记，实得 {}",
                p.feature
            );
        }
    }
}

#[test]
fn box_on_heightfield_slope() {
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11 {
        for ix in 0..11 {
            // 以 x=0 为零点、沿 x 抬升的斜坡（h(0)=0）。
            hf.set_height(ix, iz, (ix as f32 - 5.0) * 0.2);
        }
    }
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 0.35, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    // a=盒(0) 在上，b=marker(1) → 流形法线 a→b 指向地面（-y 分量为主），
    // 且因地面沿 +x 抬升而偏向 +x；求解器给盒子的推力 = -n = 朝上偏 -x。
    assert!(m[0].normal.y < -0.9, "normal {:?}", m[0].normal);
    assert!(m[0].normal.x > 0.1, "normal {:?}", m[0].normal);
}

#[test]
fn separated_boxes_no_manifold() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    assert!(manifolds_for(&b, &[]).is_empty());
}

/// T3 盒对 SAT extents 快路径 vs 通用逐顶点路径：300 对随机盒
/// （重叠/分离/极端姿态混合）同帧对拍，断言 None/Some 类别一致、
/// sep 差 ≤1e-4、法线对齐 >0.999、来源分类一致。守门对象：`sat()`
/// 内盒对分支（extents 公式）与通用顶点 min/max 的等价性。
#[test]
fn box_sat_fast_matches_vertex_reference() {
    let mut np = DefaultNarrowPhase::new(0.01);
    let ha = Vec3::new(0.5, 0.3, 0.7);
    let hb = Vec3::new(0.4, 0.6, 0.2);
    let ia = np.poly_for(&Shape::Box { half: ha }).unwrap();
    let ib = np.poly_for(&Shape::Box { half: hb }).unwrap();
    let mut rng: u32 = 0x1234_5678;
    let mut next = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (rng >> 8) as f32 / 16_777_216.0
    };
    let mut max_delta = 0.0f32;
    for k in 0..300 {
        let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
        let pb = pa + Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0);
        let ax = Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize();
        let bx = Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize();
        let qa = Quat::from_axis_angle(ax, next() * core::f32::consts::TAU);
        let qb = Quat::from_axis_angle(bx, next() * core::f32::consts::TAU);
        np.poly_a.fill(&np.polys[ia], pa, qa);
        np.poly_b.fill(&np.polys[ib], pb, qb);
        let d = pb - pa;
        // 快路径（盒对 extents 公式）。
        np.box_a = Some((ha, pa));
        np.box_b = Some((hb, pb));
        let fast = np.sat(d);
        // 通用路径（逐顶点 min/max；轴序、取向、平局规则完全相同，
        // 唯一差异即投影计算方式）。
        np.box_a = None;
        np.box_b = None;
        let slow = np.sat(d);
        match (fast, slow) {
            (None, None) => {}
            (Some((s1, n1, r1)), Some((s2, n2, r2))) => {
                let d = (s1 - s2).abs();
                if d > max_delta {
                    max_delta = d;
                }
                assert!(d <= 1e-6, "k{k}: sep {s1} vs {s2}（Δ {d}，非 ULP 级）");
                assert!(n1.dot(n2).abs() > 0.999, "k{k}: normal {n1:?} vs {n2:?}");
                assert_eq!(r1, r2, "k{k}: src {r1:?} vs {r2:?}");
            }
            (f, s) => panic!(
                "k{k}: 类别不一致 fast={:?} slow={:?}",
                f.is_some(),
                s.is_some()
            ),
        }
    }
    eprintln!("快/通用路径 sep 最大 Δ = {max_delta:.3e}（f32 ULP 级；>0 表示快路径确被拉到）");
    assert!(max_delta > 0.0, "两路径逐位相同 ⇒ 快路径未被真正测到");
}

/// 内边（折痕）幽灵接触判据：盒正中骑在「平地面 / 斜坡」的折痕上时，
/// 采样法线**不许是两块面法线的混合**——混合即内边假接触（幽灵推力）。
/// 高度场路径取「最深点采样法线」作整条流形法线，折痕处的双线性采样
/// 天然会把两块面混在一起，故这里是该缺陷的天然复现位。
#[test]
fn heightfield_crease_normal_is_not_blended() {
    // ix≤5 平（h=0），ix>5 沿 +x 抬升 0.5/格 ⇒ 折痕在 x=0（spacing=1，x0=-5）。
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11 {
        for ix in 0..11 {
            hf.set_height(ix, iz, (ix as f32 - 5.0).max(0.0) * 0.5);
        }
    }
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 0.35, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _ = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    let n = m[0].normal; // 约定：a=盒 → b=地面（指向地面者为主）
    let floor = Vec3::new(0.0, -1.0, 0.0);
    let ramp = -Vec3::new(-0.5, 1.0, 0.0).normalize();
    let d_floor = n.dot(floor);
    let d_ramp = n.dot(ramp);
    assert!(n.z.abs() < 1e-3, "折痕法线出现 z 分量（邻格串扰）：{n:?}");
    assert!(
        d_floor.max(d_ramp) > 0.995,
        "折痕法线是两块面法线的混合 ⇒ 内边幽灵接触：{n:?}（floor 对齐 {d_floor}，ramp 对齐 {d_ramp}）"
    );
}

/// T3 盒对专用路径 vs 通用路径**全链对拍**：同一姿态下，唯一变量是
/// 「体轴直生（专用）还是多面体填充（通用）」，断言 SAT 的 sep/法线/来源
/// 三者一致，且 `clip` 的接触点集合逐点对应（点数相同、特征号逐位相同、
/// 位置差 ≤1e-5——两条路径的顶点算术序不同，只保证到 ULP 级）。
/// 守门对象：专用路径的轴序 / 面表环绕序 / 特征号编号。
#[test]
fn box_dedicated_matches_generic_full_chain() {
    let mut np = DefaultNarrowPhase::new(0.02);
    let ha = Vec3::new(0.5, 0.3, 0.7);
    let hb = Vec3::new(0.4, 0.6, 0.2);
    let ia = np.poly_for(&Shape::Box { half: ha }).unwrap();
    let ib = np.poly_for(&Shape::Box { half: hb }).unwrap();
    let mut rng: u32 = 0x51ED_2701;
    let mut next = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (rng >> 8) as f32 / 16_777_216.0
    };
    let mut contacts_seen = 0usize;
    for k in 0..300 {
        let qa = Quat::from_axis_angle(
            Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize(),
            next() * core::f32::consts::TAU,
        );
        let qb = Quat::from_axis_angle(
            Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize(),
            next() * core::f32::consts::TAU,
        );
        let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
        // 近半样本深度重叠（保证 clip 真的跑出接触点），其余随机。
        let d = if k % 2 == 0 {
            Vec3::new(
                next() * 0.3 - 0.15,
                next() * 0.3 - 0.15,
                next() * 0.3 - 0.15,
            )
        } else {
            Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0)
        };
        let pb = pa + d;

        // 通用路径：多面体填充 + 盒对 extents 快路径。
        np.poly_a.fill(&np.polys[ia], pa, qa);
        np.poly_b.fill(&np.polys[ib], pb, qb);
        np.box_axes_a = None;
        np.box_axes_b = None;
        np.box_a = Some((ha, pa));
        np.box_b = Some((hb, pb));
        let generic = match np.sat(d) {
            Some((sep, n, src)) if sep <= np.skin => {
                if np.clip(n, src) {
                    Some((sep, n, src, np.cand.clone()))
                } else {
                    Some((sep, n, src, Vec::new()))
                }
            }
            _ => None,
        };

        // 专用路径：体轴直生（不填多面体——与生产路径一致）。
        let aa = box_axes(qa);
        let ab = box_axes(qb);
        np.box_axes_a = Some(aa);
        np.box_axes_b = Some(ab);
        let dedicated = match np.sat(d) {
            Some((sep, n, src)) if sep <= np.skin => {
                if np.clip(n, src) {
                    Some((sep, n, src, np.cand.clone()))
                } else {
                    Some((sep, n, src, Vec::new()))
                }
            }
            _ => None,
        };

        match (generic, dedicated) {
            (None, None) => {}
            (Some((s1, n1, r1, c1)), Some((s2, n2, r2, c2))) => {
                assert_eq!(s1.to_bits(), s2.to_bits(), "k{k}: sep 位不等");
                assert!(n1.dot(n2).abs() > 0.9999, "k{k}: 法线不一致");
                assert_eq!(r1, r2, "k{k}: 来源分类不一致");
                assert_eq!(c1.len(), c2.len(), "k{k}: 接触点数不一致");
                for p in &c1 {
                    let hit = c2
                        .iter()
                        .any(|q| q.feature == p.feature && (q.point - p.point).length() <= 1e-5);
                    assert!(hit, "k{k}: 通用点 {:?} 在专用路径无对应", p.point);
                }
                if !c1.is_empty() {
                    contacts_seen += 1;
                }
            }
            (g, s) => panic!(
                "k{k}: 类别不一致 通用={:?} 专用={:?}",
                g.is_some(),
                s.is_some()
            ),
        }
    }
    assert!(
        contacts_seen >= 100,
        "接触样本仅 {contacts_seen}，鉴别力不足"
    );
}
