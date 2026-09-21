//! **P5「球那一路」的决定性读数：用"间隙"（clearance）而不是"高度"。**
//!
//! 背景（`OPEN-PROBLEMS.md` P5 ⑪/⑫/⑭）：`trimesh_escape_probe` 的【D】段曾给出
//! "球 r=0.20 末态 y 0.599、地面 h 1.690 ⇒ 球心在地面下 1.09 m"，据此我推出过
//! **幽灵支撑自锁**（⑭）。但那条读数有两个可疑处：
//!   ① 【D】段拿**投放点**的 `h` 当参考，而球在粗网格（3.75 m 格距）上**会滚**——同一批
//!      接触打印里出现的接触点在 x ≈ **−25.33**，离投放点 26 m ⇒ 球很可能早就滚走了，
//!      于是"末态 y"与"投放点的 h"是**两个地方**的数（与起伏网盒体那次同类口径错误）；
//!   ② 收敛判据是"y vs h(投放点)"，对**会滚的球**本身不成立。
//!
//! 本探针改用**与位置无关**的量：`clearance = d_min − r`，
//! 其中 `d_min` = 球心到**全部三角形**的最小距离（本探针**自己暴力枚举**，不复用被测代码的
//! 最近面查询 ⇒ 是独立仪器）。判据：
//!   |clearance| ≲ 1 cm ⇒ **贴着/正常静置**（不管它滚到哪、拿哪个点当参考都一样）；
//!   clearance ≫ 0     ⇒ **悬空**（幽灵支撑把我们托起来，可疑方向 1）；
//!   clearance ≪ 0     ⇒ **埋进网格**（幽灵支撑自锁，⑭ 说的方向）。
//! 同时打印它**自己脚下**的 `h(x,z)` 与"末态 y − (h + r)"，用来当场展示旧口径的错法。
//!
//! 跑法（release）：
//! ```text
//! cargo run --release -q -p vxl-phys --example trimesh_sphere_rest_probe
//! ```

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::mesh::TriMesh;

/// 与 `arena_bench::scene_trimesh_terrain` / `trimesh_escape_probe` **逐字同一套**地形。
const SIZE: f32 = 90.0;
const SEG: usize = 24;
const TICKS: usize = 600;

fn h(x: f32, z: f32) -> f32 {
    (x * 0.13).sin() * 1.6 + (z * 0.11).cos() * 1.4 + ((x + z) * 0.05).sin() * 1.1
}

fn terrain() -> TriMesh {
    let step = SIZE / SEG as f32;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=SEG {
        for ix in 0..=SEG {
            let x = -SIZE / 2.0 + ix as f32 * step;
            let z = -SIZE / 2.0 + iz as f32 * step;
            verts.push(Vec3::new(x, h(x, z), z));
        }
    }
    let row = (SEG + 1) as u32;
    for iz in 0..SEG as u32 {
        for ix in 0..SEG as u32 {
            let a = iz * row + ix;
            let b = a + 1;
            let c = a + row;
            let d = c + 1;
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    TriMesh::new(verts, tris)
}

/// 点到三角形的最近点（Ericson《Real-Time Collision Detection》§5.1.5，与引擎实现无关）。
fn closest_on_tri(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

/// **暴力**最近距离：球心到全部三角形的最小距离（独立仪器，不复用被测代码的最近面查询）。
fn brute_min_dist(mesh: &TriMesh, p: Vec3) -> f32 {
    let mut best = f32::INFINITY;
    for t in mesh.tris() {
        let (a, b, c) = (
            mesh.verts()[t[0] as usize],
            mesh.verts()[t[1] as usize],
            mesh.verts()[t[2] as usize],
        );
        let q = closest_on_tri(p, a, b, c);
        best = best.min((p - q).length());
    }
    best
}

fn main() {
    let step = SIZE / SEG as f32;
    let mesh = terrain();
    println!(
        "【P5 球那一路】粗网格三角网（{SIZE} m / {SEG} 段 ⇒ 格距 {step:.3} m）、{TICKS} tick、\
         radius 0.2/0.4/0.7、从 `h + r + 0.05` 投放（贴面，避免砸出坑）\n\
         判据：**clearance = d_min(暴力) − r**（与位置无关）；同时打印旧口径用于当场对照\n"
    );
    println!(
        "  落点                      r     末态 (x, z)          h(自身)   末态 y    旧口径 Δ     clearance   清醒  |v|"
    );

    // 落点：① 四边形形心 ② 三角形**真内部**（形心的邻域，避开对角线） ③ 格点(顶点)正上方
    //       ④ 边中点。与 escape_probe 的四个落点同源，便于对照。
    let gx = step * 6.0 - SIZE / 2.0;
    let gz = step * 5.0 - SIZE / 2.0;
    let cases: [(&str, f32, f32); 4] = [
        ("① 四边形形心", gx + step * 0.5, gz + step * 0.5),
        ("② 三角形真内部", gx + step * 0.3, gz + step * 0.7),
        ("③ 格点(顶点)正上方", gx, gz),
        ("④ 边中点(x 向)", gx + step * 0.5, gz),
    ];

    for (name, x, z) in cases {
        for r in [0.20f32, 0.40, 0.70] {
            let cfg = PhysConfig::default();
            let mut w = World::new(cfg);
            w.add_mesh(mesh.clone());
            let m = w.add_material(vxl_phys_core::Material {
                friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
                restitution: 0.02,
            });
            let i = w.add_dynamic(
                Shape::Sphere { radius: r },
                Vec3::new(x, h(x, z) + r + 0.05, z),
                Quat::IDENTITY,
                1000.0,
            ) as usize;
            w.bodies.set_material(i, m);
            for _ in 0..TICKS {
                w.step();
            }
            let (pos, _rot) = w.bodies.pose(i);
            let d_min = brute_min_dist(&mesh, pos);
            let clearance = d_min - r;
            println!(
                "  {:20} {r:4.2}  ({:7.3},{:7.3})  {:8.4}  {:8.4}  {:+8.4}  {:+9.4}   {}  {:.4}",
                name,
                pos.x,
                pos.z,
                h(pos.x, pos.z),
                pos.y,
                pos.y - (h(x, z) + r),
                clearance,
                w.bodies.awake[i],
                w.bodies.linvel[i].length()
            );
        }
    }

    // 平面对照：同一套路径、同一判据 ⇒ 球在**平网格**上必须 clearance ≈ 0。
    println!("\n平网格对照（y = 0、3×3 顶点铺 8 m 见方）：");
    let mut v: Vec<Vec3> = Vec::new();
    let mut t: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=2u32 {
        for ix in 0..=2u32 {
            v.push(Vec3::new(
                -4.0 + 4.0 * ix as f32,
                0.0,
                -4.0 + 4.0 * iz as f32,
            ));
        }
    }
    for iz in 0..2u32 {
        for ix in 0..2u32 {
            let a = iz * 3 + ix;
            t.push([a, a + 3, a + 1]);
            t.push([a + 1, a + 3, a + 4]);
        }
    }
    let flat = TriMesh::new(v, t);
    for r in [0.20f32, 0.40, 0.70] {
        let mut w = World::new(PhysConfig::default());
        w.add_mesh(flat.clone());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let i = w.add_dynamic(
            Shape::Sphere { radius: r },
            Vec3::new(0.0, r + 0.05, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, _rot) = w.bodies.pose(i);
        let clearance = brute_min_dist(&flat, pos) - r;
        println!(
            "  r = {r:4.2}：末态 y {:8.4}（贴住期望 {:.4}）、旧口径 Δ {:+8.4}、**clearance {:+9.4}**、\
             清醒 {}、|v| {:.4}",
            pos.y,
            r,
            pos.y - r,
            clearance,
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }
}
