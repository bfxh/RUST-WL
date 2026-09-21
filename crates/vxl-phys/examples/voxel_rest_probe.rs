//! **体素路径的静置高度探针**（P5 ⑳ 的同族复查）。
//!
//! 背景：网格路径的点/盒查询写作 `depth = skin − sd`（穿透量 + skin），实测让盒体
//! **静置时悬空 2–3.5 cm**（已修为 `depth = −sd`，见 P5 ⑳）。体素路径的
//! `contacts_point_voxel` 是**同一形状**：`depth = skin − sdf(p)`；
//! 而球那一路 `depth = radius − sdf` 无此偏置。本探针量**修前/修后**的静置高度，
//! 作为"是否该按同形改"的判据（本探针不改引擎，先量再改）。
//!
//! 场景：8×2×8 格、边长 0.5 的体素地板（顶面 y = 1.0），盒/球从面上 0.5 m 落下。
//! 期望（口径正确时）：盒 `y = 1 + 半高`、球 `y = 1 + r`，即离地间隙 ≈ 0。
//!
//! 跑法（release）：
//! ```text
//! cargo run --release -q -p vxl-phys --example voxel_rest_probe
//! ```

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

const TICKS: usize = 600;

/// 与 `voxel.rs` 测试同源的地板：填充 y ∈ [0,1) 两层 ⇒ **顶面 y = 1.0**。
fn floor_volume() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

fn main() {
    println!("【体素静置高度】8×2×8 格、边长 0.5、顶面 y = 1.0；{TICKS} tick、默认档、μ=0.9\n");
    println!("  形状           末态 y      离地间隙     清醒  |v|");

    // 盒（半高 0.35）：期望 y = 1.35
    for half in [0.25f32, 0.35, 0.5] {
        let mut w = World::new(PhysConfig::default());
        w.add_voxel(floor_volume());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let i = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(half),
            },
            Vec3::new(0.25, 1.0 + half + 0.5, 0.25),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, _) = w.bodies.pose(i);
        println!(
            "  盒(半高 {half:4.2})  {:8.4}   {:+9.4}   {}  {:.4}",
            pos.y,
            pos.y - (1.0 + half),
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }

    // 球（对照：`depth = radius − sdf`，本来就不该有偏置）
    for r in [0.25f32, 0.35, 0.5] {
        let mut w = World::new(PhysConfig::default());
        w.add_voxel(floor_volume());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let i = w.add_dynamic(
            Shape::Sphere { radius: r },
            Vec3::new(0.25, 1.0 + r + 0.5, 0.25),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, _) = w.bodies.pose(i);
        println!(
            "  球(半径 {r:4.2})  {:8.4}   {:+9.4}   {}  {:.4}",
            pos.y,
            pos.y - (1.0 + r),
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }

    // 对照：**凸包**（走"逐顶点 `contacts_point`"那一路，与盒的 `contacts_box_voxel` 不同）
    // ⇒ 若体素的点查询仍是 `depth = skin − d`，包体应**悬空 ≈ band**，而同一块板上的盒贴住。
    for half in [0.25f32, 0.35] {
        let mut w = World::new(PhysConfig::default());
        w.add_voxel(floor_volume());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let mut pts = Vec::new();
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    pts.push(Vec3::new(sx * half, sy * half, sz * half));
                }
            }
        }
        let hull = w.add_hull(pts);
        let i = w.spawn_hull_body(
            hull,
            Vec3::new(0.25, 1.0 + half + 0.5, 0.25),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, _) = w.bodies.pose(i);
        println!(
            "  包(半高 {half:4.2})  {:8.4}   {:+9.4}   {}  {:.4}",
            pos.y,
            pos.y - (1.0 + half),
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }

    // 对照：盒落在**高度场**平地上（同为"地面"，口径已知正确）⇒ 间隙应 ≈ 0
    let mut w = World::new(PhysConfig::default());
    w.add_heightfield(vxl_phys_narrow::heightfield::HeightField::flat(
        -8.0, -8.0, 17, 17, 1.0, 1.0,
    ));
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.35),
        },
        Vec3::new(0.0, 1.0 + 0.35 + 0.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    for _ in 0..TICKS {
        w.step();
    }
    let (pos, _) = w.bodies.pose(i);
    println!(
        "\n  【高度场对照】盒(半高 0.35) 末态 y {:8.4}、离地间隙 {:+9.4}、清醒 {}  ⇒ 已知正确口径的读数",
        pos.y,
        pos.y - (1.0 + 0.35),
        w.bodies.awake[i]
    );
}
