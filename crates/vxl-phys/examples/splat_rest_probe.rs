//! **喷溅域的静置高度探针**（P5 ⑳/㉑ 的同族复查，最后一块域）。
//!
//! 背景：网格路径的盒体曾悬空 2–3.5 cm（`depth = skin − sd`，已修为 `−sd`，P5 ⑳）；
//! 体素的**点查询**同形偏置让外壳悬空 0.022 m（已修，P5 ㉑）。喷溅域的点查询是
//! `depth = skin − f`（`f` = 到等值面的距离），**且它的盒查询是 14 点采样走点查询**
//! （与网格同形、与体素不同）⇒ 预测：**盒在喷溅场上悬空 ≈ band，球贴住**。
//!
//! 参考面**独立**：用场自己的公开 `sdf()` 沿 y 二分求零点（等值面高度），且**逐角点各算各的**
//! （避免"拿体中心的标高当参考"那类错误——本会话已因此翻了三次车）。
//!
//! 跑法（release）：
//! ```text
//! cargo run --release -q -p vxl-phys --example splat_rest_probe
//! ```

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_splat::{GaussianSplatField, Splat};

const TICKS: usize = 600;
const ISO: f32 = 0.5;

/// 平场：y = 0 平面上按 0.25 m 铺各向同性核（σ = 0.5、幅值 1.0）。
fn flat_field() -> GaussianSplatField {
    let mut f = GaussianSplatField::new(ISO);
    for ix in -10..=10 {
        for iz in -10..=10 {
            let x = ix as f32 * 0.25;
            let z = iz as f32 * 0.25;
            f.push(Splat::isotropic(Vec3::new(x, 0.0, z), 0.5, 1.0));
        }
    }
    f
}

/// 等值面高度：沿 (x, ·, z) 在 y ∈ [−1, 2] 上二分 `sdf` 的零点（**场自己的公开 API**）。
fn iso_height_at(f: &GaussianSplatField, x: f32, z: f32) -> f32 {
    let (mut lo, mut hi) = (-1.0f32, 2.0f32);
    let flo = f.sdf(Vec3::new(x, lo, z));
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        let fm = f.sdf(Vec3::new(x, mid, z));
        if (fm > 0.0) == (flo > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

fn main() {
    let field = flat_field();
    let y_ref = iso_height_at(&field, 0.0, 0.0);
    println!(
        "【喷溅域静置高度】平场（y=0 面铺 σ=0.5 核、iso={ISO}）⇒ 中心处等值面 y = {y_ref:.4}\n\
         （参考面由场自己的 `sdf()` 二分求得，逐角点各算）\n"
    );
    println!("  形状           末态 y      **最小离面间隙**   清醒  |v|");

    // 盒：底面 4 角各按**自己所在 (x,z)** 的等值面高度算间隙，取最小值
    for half in [0.25f32, 0.35] {
        let mut w = World::new(PhysConfig::default());
        w.add_splat_field(flat_field());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let i = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(half),
            },
            Vec3::new(0.0, y_ref + half + 0.5, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, rot) = w.bodies.pose(i);
        let mut min_gap = f32::INFINITY;
        for sx in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let c = rot.rotate_vec3(Vec3::new(sx * half, -half, sz * half)) + pos;
                min_gap = min_gap.min(c.y - iso_height_at(&field, c.x, c.z));
            }
        }
        println!(
            "  盒(半高 {half:4.2})  {:8.4}   {:+9.4}        {}  {:.4}",
            pos.y,
            min_gap,
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }

    // 球（对照：`depth = radius − f`，本就不该有偏置）
    for r in [0.25f32, 0.35] {
        let mut w = World::new(PhysConfig::default());
        w.add_splat_field(flat_field());
        let m = w.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let i = w.add_dynamic(
            Shape::Sphere { radius: r },
            Vec3::new(0.0, y_ref + r + 0.5, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.set_material(i, m);
        for _ in 0..TICKS {
            w.step();
        }
        let (pos, _) = w.bodies.pose(i);
        println!(
            "  球(半径 {r:4.2})  {:8.4}   {:+9.4}        {}  {:.4}",
            pos.y,
            pos.y - r - iso_height_at(&field, pos.x, pos.z),
            w.bodies.awake[i],
            w.bodies.linvel[i].length()
        );
    }
}
