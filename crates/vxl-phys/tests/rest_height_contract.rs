//! **静置高度 / 可恢复性的回归门**（P5 ⑳㉑ 落地后的守卫）。
//!
//! 为什么要有这个文件：2026-09-21 修的三处 `depth` 口径（网格 `−sd`、体素点 `−sdf`、
//! 喷溅 `−f`）此前**只有 example 层探针**在看，而 example 不进 `cargo test` ⇒ 谁改回去
//! 都不会被门禁拦住。这里把两侧契约都钉成测试：
//!   ① **贴住**：盒在平网格 / 外壳在平体素地板上，静置后底面必须落在地表 ±1 cm 内
//!      （旧口径 `depth = skin − d` 会让它悬空约一个 skin = 2 cm ⇒ 本条会红）；
//!   ② **可恢复**：球**生成在网格内部**时必须被顶出（这是 ⑯ 认定的差异化能力，
//!      修口径时最容易连带打掉的就是它）。
//!
//! 这两条是同一个取舍的两侧，必须成对存在（见 `OPEN-PROBLEMS.md` P5 ⑯⑰⑳）。

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

const TICKS: usize = 400;

/// 平网格：8 m 见方、y = 0 的 2×2 四边形（法线 +Y）。
fn flat_mesh() -> vxl_phys_terrain::mesh::TriMesh {
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=2u32 {
        for ix in 0..=2u32 {
            verts.push(Vec3::new(
                -4.0 + 4.0 * ix as f32,
                0.0,
                -4.0 + 4.0 * iz as f32,
            ));
        }
    }
    for iz in 0..2u32 {
        for ix in 0..2u32 {
            let a = iz * 3 + ix;
            tris.push([a, a + 3, a + 1]);
            tris.push([a + 1, a + 3, a + 4]);
        }
    }
    vxl_phys_terrain::mesh::TriMesh::new(verts, tris)
}

/// 平体素地板：8×2×8 格、边长 0.5、**顶面 y = 1.0**。
fn flat_voxel() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

fn material(w: &mut World) -> vxl_phys_core::MaterialId {
    w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    })
}

/// ① 盒落在**平网格**上：底面必须贴住 y = 0（±1 cm），不得悬空一个皮肤带。
#[test]
fn mesh_box_rests_on_surface() {
    let half = 0.35f32;
    let mut w = World::new(PhysConfig::default());
    w.add_mesh(flat_mesh());
    let m = material(&mut w);
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(half),
        },
        Vec3::new(0.5, 0.0 + half + 0.5, -0.5),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    for _ in 0..TICKS {
        w.step();
    }
    let (pos, rot) = w.bodies.pose(i);
    // 逐角点取最低点（斜停也算对），再与地表 y = 0 比。
    let mut lowest = f32::INFINITY;
    for sx in [-1.0f32, 1.0] {
        for sz in [-1.0f32, 1.0] {
            lowest = lowest.min((rot.rotate_vec3(Vec3::new(sx * half, -half, sz * half)) + pos).y);
        }
    }
    assert!(
        (-0.012..=0.010).contains(&lowest),
        "盒底面最低角点 y = {lowest:+.4}（应贴住 y=0，±1 cm）——悬空 ≈ 一个皮肤带说明 \
         `contacts_point` 的 depth 又被写回 `skin − sd`（见 P5 ⑳）"
    );
}

/// ① 外壳落在**平体素地板**上：底面必须贴住顶面 y = 1.0（外壳走"逐顶点 `contacts_point`"
/// 那一路，是体素口径修正的守卫；盒走 `contacts_box_voxel`，本来就对）。
#[test]
fn voxel_hull_rests_on_surface() {
    let half = 0.35f32;
    let mut w = World::new(PhysConfig::default());
    w.add_voxel(flat_voxel());
    let m = material(&mut w);
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
    let (pos, rot) = w.bodies.pose(i);
    let mut lowest = f32::INFINITY;
    for sx in [-1.0f32, 1.0] {
        for sz in [-1.0f32, 1.0] {
            lowest = lowest.min((rot.rotate_vec3(Vec3::new(sx * half, -half, sz * half)) + pos).y);
        }
    }
    assert!(
        (-0.012..=0.012).contains(&(lowest - 1.0)),
        "外壳底面最低角点离地 {:+.4}（应贴住顶面 y=1.0，±1.2 cm）——悬空 ≈ 一个皮肤带说明 \
         `contacts_point_voxel` 的 depth 又被写回 `skin − sdf`（见 P5 ㉑）",
        lowest - 1.0
    );
}

/// ② 球**生成在网格内部**（球心在地表下 0.20 m）必须被顶出到 `y ≈ r`。
/// 这是"可恢复性"那一侧的守卫：修静置口径时最容易连带打掉它（见 P5 ⑯）。
#[test]
fn mesh_sphere_inside_is_ejected() {
    let r = 0.45f32;
    let mut w = World::new(PhysConfig::default());
    w.add_mesh(flat_mesh());
    let m = material(&mut w);
    let i = w.add_dynamic(
        Shape::Sphere { radius: r },
        Vec3::new(0.5, -0.20, -0.5), // 球心在地表**下方** 0.20 m
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    for _ in 0..TICKS {
        w.step();
    }
    let (pos, _) = w.bodies.pose(i);
    assert!(
        (pos.y - r).abs() < 0.1,
        "陷网球未被顶出：末态 y = {:.4}，期望 ≈ {r:.2}（差 {:.3} m）——可恢复性被打掉了（P5 ⑯）",
        pos.y,
        pos.y - r
    );
}
