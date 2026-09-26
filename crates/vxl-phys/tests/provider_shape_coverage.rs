//! **窄相"有形状、无接触"缺口的判据**（`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §4）：
//! `capsule / cylinder / cone` 撞 **provider（三角网/体素/平面集）** 今天**一个接触都不产** ⇒ 体直接穿过。
//!
//! **为什么这条判据值得先立**：布节点 / 角色肢体 / 绳索段最常用的几何就是**胶囊**，而"胶囊 vs 任意几何"
//! 恰好是布料的主力用法。缺口在窄相里是"静默的"（`provider_pair` 走 `_ => false`，不报错、不产接触）
//! ⇒ 线上表现是"**胶囊穿过地形**"。而这条一直**没有判据**的原因也已查明：窄相入口 `process_pair_shaped`
//! 是 `pub(crate)`，集成测试够不到；crate 内部的 `narrow/src/tests.rs` 又受尺寸门棘轮约束
//! （加行须配"函数变短"）⇒ **判据只能抬到门面级**（本文件就是这么来的：新文件只判阈值，不撞棘轮）。
//!
//! **本文件的口径（黑盒，不读接触集）**：
//! - **对照组**（证明场景搭对了、落地路径是通的）：`Box` 与 `Sphere` 都能**停在地板上**（各自停在
//!   半高 / 半径附近）；
//! - **缺口组**（今天应当是"穿过去"）：`Capsule` / `Cylinder` / `Cone` 会**一路下沉**（无接触 ⇒ 不存在平衡）。
//!
//! ⇒ 修法（沿轴 N 球采样 + `contacts_sphere`，见 §4）落地后，**缺口组的三条断言要翻过来**
//! （改成"停在 radius / half_height 附近"）。这就是"判据先立、修法后到"。
//!
//! ⚠️ 本判据**不改任何物理**：它只钉现状，故不影响任何既有读数。纯 CPU ⇒ **CI 会真跑**。
//!
//! 已知取舍（写清不藏）：判据用的是**平地板**（`h ≡ 0`）——比 arena 那条起伏网简单，但对"有没有接触"
//! 这件事足够，且读数（停/穿）是二值的、不吃容差。

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

/// 场景：平地板 + 一个从 `y0` 掉下来的体（`Shape` 与投放高度由调用方给）。
fn drop_scene(shape: Shape, y0: f32) -> (World, usize) {
    let cfg = PhysConfig::default();
    let mut w = World::new(cfg);
    let _mesh_body = w.add_mesh(flat_mesh());
    let i = w.add_dynamic(shape, Vec3::new(0.0, y0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    for _ in 0..TICKS {
        w.step();
    }
    (w, i)
}

/// 体的表面积最低点世界 y —— **比"质心高度"干净**（质心高度依赖姿态，最低点不依赖）。
fn lowest_y(shape: &Shape, pos: Vec3, rot: Quat) -> f32 {
    // 采样面：盒取 6 面 7×7（与 arena 同口径）；球/胶囊/圆柱/圆锥取**轴向与周向**采样
    // （目的只是"最低点"，不需要精确等于解析极值 ⇒ 采样足够密即可）。
    let mut lo = f32::INFINITY;
    let mut push = |l: Vec3| {
        lo = lo.min((rot.rotate_vec3(l) + pos).y);
    };
    const K: usize = 9;
    let axis = match *shape {
        Shape::Box { half } => {
            for sgn in [-1.0f32, 1.0] {
                for i in 0..K {
                    for j in 0..K {
                        let u = -half.x + 2.0 * half.x * i as f32 / (K as f32 - 1.0);
                        let v = -half.z + 2.0 * half.z * j as f32 / (K as f32 - 1.0);
                        push(Vec3::new(u, sgn * half.y, v));
                    }
                }
            }
            return lo;
        }
        Shape::Sphere { radius } => -(radius),
        Shape::Capsule {
            half_height,
            radius,
        } => -(half_height + radius),
        Shape::Cylinder {
            half_height,
            radius,
        } => -(half_height.hypot(radius)).max(-(half_height + radius)),
        Shape::Cone {
            half_height,
            radius,
        } => -(half_height.hypot(radius)).max(-(half_height + radius)),
        _ => -(shape.bounding_sphere_radius()),
    };
    // 轴向 + 周向采样：对光滑形状求"最低点"足够（判据只用到"停/穿"的二值性质）。
    for i in 0..=K {
        let t = -1.0 + 2.0 * i as f32 / K as f32;
        push(Vec3::new(0.0, t * axis.abs().max(1e-6), 0.0));
        for j in 0..8 {
            let a = std::f32::consts::TAU * j as f32 / 8.0;
            push(Vec3::new(a.cos(), t * axis.abs().max(1e-6), a.sin()));
        }
    }
    lo
}

/// 一个体的"静止/穿透"读数：`(最低点 y, 是否还在动)`。
fn reading(w: &World, i: usize) -> (f32, bool) {
    let (p, r) = w.bodies.pose(i);
    (
        lowest_y(&w.bodies.shape[i], p, r),
        w.bodies.linvel[i].length() > 0.05,
    )
}

#[test]
fn box_and_sphere_rest_on_provider_but_capsule_falls_through() {
    // —— 对照组①：盒停在地板上（≈ 半高）——
    let (w, i) = drop_scene(
        Shape::Box {
            half: Vec3::splat(HALF_BOX),
        },
        2.0,
    );
    let (y_box, moving_box) = reading(&w, i);
    // 口径：`lowest_y` 是**体表面最低点**（≈ 与地板面的间隙）⇒ 停住时它 ≈ 0
    //（盒的**质心**才在半高 0.35 处——第一版断言把它俩混了，判据当场红了）。
    assert!(
        y_box.abs() < 0.1,
        "盒应**底贴地板**（最低点与地板面之差 ≈ 0，实得 {y_box:+.4}；还在动={moving_box}）\
         ——这里红了说明**场景或 provider 通道本身坏了**，下面那几条缺口读数也就没有意义"
    );

    // —— 对照组②：球停在地板上（最低点同样 ≈ 0）——
    let (w, i) = drop_scene(Shape::Sphere { radius: RADIUS }, 2.0);
    let (y_sph, _) = reading(&w, i);
    assert!(
        y_sph.abs() < 0.1,
        "球应停在地板上（最低点 ≈ 0，实得 {y_sph:+.4}）"
    );

    // —— 缺口组：胶囊 / 圆柱 / 圆锥今天**不产接触** ⇒ 一路下沉 ——
    // ⚠️ 这三条钉的是**现状（缺口）**：修法（沿轴 N 球采样 + `contacts_sphere`）落地后，
    //    三条都要改成"停在 expected 附近"（见 `SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §4）。
    for (name, shape) in [
        (
            "capsule（胶囊）",
            Shape::Capsule {
                half_height: HALF_H,
                radius: RADIUS,
            },
        ),
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
        let (w, i) = drop_scene(shape, 2.0);
        let (y, moving) = reading(&w, i);
        println!("  · {name}：最低点 y = {y:+.4}（修好后应 ≈ 0）；还在动={moving}");
        assert!(
            y < -0.5,
            "{name} 今天**不该**停住（`provider_pair` 对它 `_ => false` ⇒ 无接触）——\
             若它停了，说明缺口已被修好 ⇒ **把这条断言翻过来**（改成 |y| < 0.1）"
        );
    }
}
