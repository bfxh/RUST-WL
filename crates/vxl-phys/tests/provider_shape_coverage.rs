//! **窄相"有形状、无接触"缺口的判据**（`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §4/§5）：
//! `capsule / cylinder / cone` 撞 **提供者（三角网/体素/平面集）** 曾**一个接触都不产** ⇒ 体直接穿过。
//! 布节点 / 角色肢体 / 绳索段最常用的几何就是**胶囊** ⇒ 这条缺口就是"布料有问题"能落到的具体症状。
//!
//! **为什么判据在这一级**：窄相入口 `process_pair_shaped` 是 `pub(crate)`（`crates/*/tests/` 够不到），
//! `narrow/src/tests.rs` 又受尺寸棘轮约束 ⇒ 只能在门面级用 `World` **黑盒**搭场景（本文件）。
//!
//! **口径（黑盒，不读接触集）**：对照组 `Box`/`Sphere` 停在地板上（证明场景与通道是通的）；
//! **已修复**：`Capsule` 直立（单点支撑）与侧躺（多样本线接触）都停在地板上；
//! **仍缺口**：`Cylinder`/`Cone`（同族修法"端面圆 + 母线采样"未做）⇒ 断言还是"一路下沉"。
//!
//! 已知取舍（写清不藏）：判据用的是**平地板**（`h ≡ 0`）——比 arena 那条起伏网简单，但对"有没有接触"
//! 这件事足够，且读数（停/穿）是二值的、不吃容差。纯 CPU ⇒ **CI 会真跑**。

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::mesh::TriMesh;

const TICKS: usize = 400;
const HALF_BOX: f32 = 0.35;
const RADIUS: f32 = 0.35;
const HALF_H: f32 = 0.35;

/// 平地板（y = 0）：8×8 格、±4 m，与 arena 那条地形**同一套三角化顺序**（只把高度函数换成 0）。
fn flat_mesh() -> TriMesh {
    const N: usize = 8;
    const S: f32 = 4.0;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=N {
        for ix in 0..=N {
            let x = -S + (2.0 * S * ix as f32) / N as f32;
            let z = -S + (2.0 * S * iz as f32) / N as f32;
            verts.push(Vec3::new(x, 0.0, z));
        }
    }
    for iz in 0..N as u32 {
        for ix in 0..N as u32 {
            let a = iz * (N as u32 + 1) + ix;
            let c = a + 1;
            let d = a + N as u32 + 1;
            let e = d + 1;
            tris.push([a, d, c]);
            tris.push([c, d, e]);
        }
    }
    TriMesh::new(verts, tris)
}

/// 场景：平地板 + 一个从 `y0` 掉下来的体（形状/姿态由调用方给）。
fn drop_scene(shape: Shape, y0: f32, rot: Quat) -> (World, usize) {
    let cfg = PhysConfig::default();
    let mut w = World::new(cfg);
    let _mesh_body = w.add_mesh(flat_mesh());
    let i = w.add_dynamic(shape, Vec3::new(0.0, y0, 0.0), rot, 1000.0) as usize;
    for _ in 0..TICKS {
        w.step();
    }
    (w, i)
}

/// 体的表面积最低点世界 y —— **比"质心高度"干净**（质心高度依赖姿态，最低点不依赖）。
///
/// **光滑形状走解析式，不走采样**（2026-09-26 修的是**仪器本身**）：旧版把"轴 + 周向"采样的
/// 径向偏移写成**单位 1**（与 `radius` 无关）⇒ 轴一躺平就把半径当成 1 m（实测侧躺胶囊读
/// −0.6501 = 0.35 − 1.0，而它其实正停在质心高 0.35 上、且已入睡）。判据仪器必须与姿态无关
/// ⇒ 改成闭式。记形状本地 +Y 的世界方向 `a = rot·Y`：胶囊 = 轴段两端的球
/// （`pos.y − half_height·|a.y| − radius`）；圆柱 = 两个端面圆盘（侧面极值都在端圈上，
/// `pos.y − half_height·|a.y| − radius·√(1 − a.y²)`）；圆锥 = 底面圆盘 与 顶点 取更低。
///
/// 等价性核对（换了仪器但**不能换读数**）：盒 −0.0009 / 球 −0.0006 / **直立**胶囊 −0.0004 ——
/// 三条与旧采样器逐位相同（旧版在这三种姿态下恰好算对，侧躺那条才是旧仪器错的）。
fn lowest_y(shape: &Shape, pos: Vec3, rot: Quat) -> f32 {
    // 盒：6 面 7×7 顶点采样（与 arena 同口径，浅盒的姿态耦合靠面采样覆盖）。
    if let Shape::Box { half } = *shape {
        let mut lo = f32::INFINITY;
        const K: usize = 9;
        for sgn in [-1.0f32, 1.0] {
            for i in 0..K {
                for j in 0..K {
                    let u = -half.x + 2.0 * half.x * i as f32 / (K as f32 - 1.0);
                    let v = -half.z + 2.0 * half.z * j as f32 / (K as f32 - 1.0);
                    lo = lo.min((rot.rotate_vec3(Vec3::new(u, sgn * half.y, v)) + pos).y);
                }
            }
        }
        return lo;
    }
    let ay = rot.rotate_vec3(Vec3::Y).y;
    let rim = |radius: f32| radius * (1.0 - ay * ay).max(0.0).sqrt();
    match *shape {
        Shape::Sphere { radius } => pos.y - radius,
        Shape::Capsule {
            half_height,
            radius,
        } => pos.y - half_height * ay.abs() - radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => pos.y - half_height * ay.abs() - rim(radius),
        Shape::Cone {
            half_height,
            radius,
        } => (pos.y - half_height * ay - rim(radius)).min(pos.y + half_height * ay),
        _ => pos.y - shape.bounding_sphere_radius(),
    }
}

/// 一个体的"静止/穿透"读数：`(最低点 y, 是否还在动)`。
fn reading(w: &World, i: usize) -> (f32, bool) {
    let (p, r) = w.bodies.pose(i);
    (
        lowest_y(&w.bodies.shape[i], p, r),
        w.bodies.linvel[i].length() > 0.05,
    )
}

/// **停住判据**：最低点贴地板（`|y| < 0.1`）**且已静止**。为什么"静止"也判：只判高度的话
/// "在地板上下弹跳/滚动"也能过，而"没接触"与"有接触但解不稳"是两种病 ⇒ 两条一起才算收敛。
fn assert_rests(name: &str, shape: Shape, rot: Quat) {
    let (w, i) = drop_scene(shape, 2.0, rot);
    let (y, moving) = reading(&w, i);
    println!(
        "  · {name}：体表最低点 y = {y:+.4}（应 ≈ 0）；质心 y = {:+.4}；还在动={moving}",
        w.bodies.position[i].y
    );
    assert!(
        y.abs() < 0.1,
        "{name} 应**停在地板上**（表面最低点 ≈ 0，实得 {y:+.4}）——这里红了说明\
         `provider_shape_contacts` 没给它建接触（缺口复发）或接触解不收敛"
    );
    assert!(
        !moving,
        "{name} 停住了但**还在动**（|v| > 0.05）⇒ 接触在但不收敛"
    );
}

/// **对照组 + 已修复组**：盒/球/胶囊都能停在三角网地板上。
#[test]
fn box_sphere_capsule_rest_on_provider_floor() {
    // —— 对照组①：盒停在地板上。口径：`lowest_y` 量的是**体表面最低点**（≈ 与地板面的间隙），
    //    停住时它 ≈ 0——盒的**质心**才在半高 0.35 处（第一版断言把两者混了，判据当场红了）。
    assert_rests(
        "box（对照）",
        Shape::Box {
            half: Vec3::splat(HALF_BOX),
        },
        Quat::IDENTITY,
    );
    // —— 对照组②：球停在地板上（最低点同样 ≈ 0）——
    assert_rests(
        "sphere（对照）",
        Shape::Sphere { radius: RADIUS },
        Quat::IDENTITY,
    );

    // —— 已修复组：胶囊（2026-09-26；`provider.rs::capsule_provider_contacts` 沿轴 5 球采样）——
    // ①**直立**：只有底端球落进接触带 ⇒ **单点支撑**（与球同难度的解算题）；
    assert_rests(
        "capsule 直立",
        Shape::Capsule {
            half_height: HALF_H,
            radius: RADIUS,
        },
        Quat::IDENTITY,
    );
    // ②**侧躺**（轴转水平）：多个样本同时命中同一张面 ⇒ **一条线接触**、取满 4 槽——只判直立的话，
    //    5 个样本里只有 1 个被测到（这条才压到"多样本"那半边）。
    assert_rests(
        "capsule 侧躺",
        Shape::Capsule {
            half_height: HALF_H,
            radius: RADIUS,
        },
        Quat::from_axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_2),
    );
}

/// **缺口组**：圆柱 / 圆锥今天仍**不产接触** ⇒ 一路下沉（同族修法未做）。
/// 修法方向（`SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §5.1 末）：**端面圆 + 母线采样**——记轴上球采样
/// **覆盖不到圆柱侧面**（球只在球心正对的轴向高度上碰到侧面）⇒ 样本要放到"端面圆周"与"母线"上去。
#[test]
fn cylinder_and_cone_still_fall_through_provider_floor() {
    for (name, shape) in [
        (
            "cylinder（圆柱）",
            Shape::Cylinder {
                half_height: HALF_H,
                radius: RADIUS,
            },
        ),
        (
            "cone（圆锥）",
            Shape::Cone {
                half_height: HALF_H,
                radius: RADIUS,
            },
        ),
    ] {
        let (w, i) = drop_scene(shape, 2.0, Quat::IDENTITY);
        let (y, moving) = reading(&w, i);
        println!("  · {name}：最低点 y = {y:+.4}（修好后应 ≈ 0）；还在动={moving}");
        assert!(
            y < -0.5,
            "{name} 今天**不该**停住（`provider_shape_contacts` 对它 `_ => false` ⇒ 无接触）——\
             若它停了，说明缺口已被修好 ⇒ **把这条断言翻过来**（改成 |y| < 0.1）"
        );
    }
}
