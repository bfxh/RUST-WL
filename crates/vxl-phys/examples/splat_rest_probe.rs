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

/// 平场：y = 0 平面上按 0.25 m 铺各向同性核（幅值 1.0）。 越大 ⇒ 等值面越**平滑**
/// （核半径相对铺距越大，起伏越小）——用来把"球停不住"里的**场起伏**因素与接触行为分开。
fn flat_field(sigma: f32) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(ISO);
    for ix in -10..=10 {
        for iz in -10..=10 {
            let x = ix as f32 * 0.25;
            let z = iz as f32 * 0.25;
            f.push(Splat::isotropic(Vec3::new(x, 0.0, z), sigma, 1.0));
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
    let field = flat_field(0.5);
    let y_ref = iso_height_at(&field, 0.0, 0.0);
    // **死区判据**（P6 根因的量化形式）：截断半径 = σ·√cut；若它在等值面上方的余量
    // 小于体半径，则体心会落进"读不到任何核（s=0）"的死区 ⇒ 该体在此域上不可能稳定接触。
    // 3σ（cut=9）对**多核叠加**的场不够：等值面本身可以浮到接近 3σ 高（这里是 1.31 / 1.5）。
    let cut_r = 0.5 * field.cut.sqrt();
    println!(
        "【喷溅域静置高度】平场（y=0 面铺 σ=0.5 核、iso={ISO}）⇒ 中心处等值面 y = {y_ref:.4}\n\
         截断：cut = {:.1} ⇒ 截断半径 σ√cut = **{cut_r:.4}**、等值面上方的**有效余量 = {:.4} m**\n\
         （余量 < 体半径 ⇒ 体心落进\"s=0 死区\" ⇒ 该体在此域上不可能有稳定接触）\n\
         （参考面由场自己的 `sdf()` 二分求得，逐角点各算）\n",
        field.cut,
        cut_r - y_ref
    );
    println!("  形状           末态 y      **最小离面间隙**   清醒  |v|");

    // 盒：底面 4 角各按**自己所在 (x,z)** 的等值面高度算间隙，取最小值
    for half in [0.25f32, 0.35] {
        let mut w = World::new(PhysConfig::default());
        w.add_splat_field(flat_field(0.5));
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
        w.add_splat_field(flat_field(0.5));
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

    // ③ **σ 扫描**：把"场起伏"与"球接触行为"分开。σ=0.5 时等值面按 0.25 m 铺距起伏明显
    //    ⇒ 球会像在搓衣板上那样滚；σ 加大（核重叠更强）⇒ 等值面趋平。
    //    若球在**平滑场**上贴住，则此前那条"球沉 0.25 m"是**参考场太粗糙**，不是接触算错。
    println!("\n③ σ 扫描（球 r=0.35 / 盒半高 0.35，同一铺距 0.25 m）：");
    println!("   σ      等值面 y   球间隙    球|v|   球清醒   盒间隙   盒|v|");
    for sigma in [0.5f32, 0.75, 1.0, 1.5] {
        let field = flat_field(sigma);
        let y_ref = iso_height_at(&field, 0.0, 0.0);
        let mut row = format!("   {sigma:4.2}  {y_ref:8.4}  ");
        // 球
        {
            let mut w = World::new(PhysConfig::default());
            w.add_splat_field(flat_field(sigma));
            let m = w.add_material(vxl_phys_core::Material {
                friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
                restitution: 0.02,
            });
            let i = w.add_dynamic(
                Shape::Sphere { radius: 0.35 },
                Vec3::new(0.0, y_ref + 0.35 + 0.5, 0.0),
                Quat::IDENTITY,
                1000.0,
            ) as usize;
            w.bodies.set_material(i, m);
            for _ in 0..TICKS {
                w.step();
            }
            let (pos, _) = w.bodies.pose(i);
            row += &format!(
                "{:+9.4}  {:.4}  {}  ",
                pos.y - 0.35 - iso_height_at(&field, pos.x, pos.z),
                w.bodies.linvel[i].length(),
                w.bodies.awake[i]
            );
        }
        // 盒（对照）
        {
            let mut w = World::new(PhysConfig::default());
            w.add_splat_field(flat_field(sigma));
            let m = w.add_material(vxl_phys_core::Material {
                friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
                restitution: 0.02,
            });
            let i = w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.35),
                },
                Vec3::new(0.0, y_ref + 0.35 + 0.5, 0.0),
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
                    let c = rot.rotate_vec3(Vec3::new(sx * 0.35, -0.35, sz * 0.35)) + pos;
                    min_gap = min_gap.min(c.y - iso_height_at(&field, c.x, c.z));
                }
            }
            row += &format!("{min_gap:+9.4}  {:.4}", w.bodies.linvel[i].length());
        }
        println!("{row}");
    }

    // ④ **深埋顶出**（把"场太软/顶点不稳"与"接触没生成"分开的关键一步）：
    //    把球心放到等值面**下方** 0.02/0.10/0.20/0.30 m、零初速、**关重力**，看它能否被顶出来。
    //    ⚠️ **必须做"睡 vs 不睡"对照**：零初速 + 关重力 ⇒ 睡眠计时器跑满就睡，
    //    而**睡眠体不做检测**（本仓既有事实，`trimesh_escape_probe` 踩过同一个坑）
    //    ⇒ 不分开就会把"接触没生成"与"睡了所以不动"混为一谈。
    println!("\n④ 深埋顶出（σ=0.5，球 r=0.35，关重力，60 tick；睡 vs 不睡 对照）：");
    let field = flat_field(0.5);
    let y_ref = iso_height_at(&field, 0.0, 0.0);
    for no_sleep in [false, true] {
        println!(
            "   （{}）",
            if no_sleep {
                "sleep_time = 1e9（不睡）"
            } else {
                "默认睡眠"
            }
        );
        for depth in [0.02f32, 0.10, 0.20, 0.30] {
            let mut cfg = PhysConfig {
                gravity: Vec3::ZERO,
                ..PhysConfig::default()
            };
            if no_sleep {
                cfg.sleep_time = 1e9;
            }
            let mut w = World::new(cfg);
            w.add_splat_field(flat_field(0.5));
            let m = w.add_material(vxl_phys_core::Material {
                friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
                restitution: 0.02,
            });
            let i = w.add_dynamic(
                Shape::Sphere { radius: 0.35 },
                Vec3::new(0.0, y_ref + 0.35 - depth, 0.0),
                Quat::IDENTITY,
                1000.0,
            ) as usize;
            w.bodies.set_material(i, m);
            let y0 = w.bodies.pose(i).0.y;
            for _ in 0..60 {
                w.step();
            }
            let (pos, _) = w.bodies.pose(i);
            // 直接问提供者：那个埋深下它到底报了什么？（不靠猜）
            if !no_sleep {
                use vxl_phys_core::interop::ProviderColliders;
                let mut q: Vec<vxl_phys_core::interop::InteropContact> = Vec::new();
                let c = Vec3::new(0.0, y_ref + 0.35 - depth, 0.0);
                let _ = w.bodies; // 仅借用；下面直接问场
                flat_field(0.5).contacts_sphere(0, c, 0.35, 0.02, &mut q);
                let f_ctr = flat_field(0.5).sdf(c);
                match q.first() {
                    Some(cc) => println!(
                        "         [直接查询] sdf(center)={f_ctr:+.4}、depth={:+.4}、n=({:6.3},{:6.3},{:6.3})、feat={}",
                        cc.depth, cc.normal.x, cc.normal.y, cc.normal.z, cc.feature
                    ),
                    None => println!("         [直接查询] sdf(center)={f_ctr:+.4}、**无接触**"),
                }
            }
            println!(
                "      初埋 {depth:4.2} m ⇒ 上移 {:+8.4} m、末态间隙 {:+8.4}、|v| {:.4}、清醒 {}",
                pos.y - y0,
                pos.y - 0.35 - iso_height_at(&field, pos.x, pos.z),
                w.bodies.linvel[i].length(),
                w.bodies.awake[i]
            );
        }
    }

    // ⑤ **截断收敛 + 开销**（P6 根因修复的判据）：把"等值面位置"随 `cut` 的变化量
    //    与**无截断参考**比——参考由本探针**自己**按同一批核参数求和（不经过 crate 的 cut）。
    //    收敛说明"4σ 之后表面不再移动"；同时给查询开销（相对 cut=9）。
    println!("\n⑤ 截断收敛与开销（平场，无截断参考由本探针自算）：");
    let y_unt = iso_height_untruncated(0.0, 0.0);
    println!("   无截断等值面 y = {y_unt:.4}");
    println!("   cut  √cut   有截断等值面   Δ(有−无截断)   余量(√cut·σ−面)   200k 查询用时");
    for cut in [4.0f32, 9.0, 16.0, 25.0, 36.0] {
        let mut f = flat_field(0.5);
        f.cut = cut;
        let y = iso_height_at(&f, 0.0, 0.0);
        let t0 = std::time::Instant::now();
        let mut acc = 0.0f32;
        for k in 0..200_000u32 {
            let yq = 0.6 + (k % 64) as f32 * 0.02;
            // 量 **sdf**（接触真正用的入口；近场细化会在这里触发）
            acc += f.sdf(Vec3::new(0.1, yq, 0.1));
        }
        let dt = t0.elapsed().as_secs_f64() * 1000.0;
        println!(
            "   {cut:4.0} {:.2}   {:10.4}   {:+11.4}      {:+10.4}       {dt:8.1} ms  (acc {acc:.1})",
            cut.sqrt(),
            y,
            y - y_unt,
            0.5 * cut.sqrt() - y
        );
    }

    f_error_profile();
}

/// **无截断**参考密度：本探针按与 `flat_field` 同一批核参数自己求和
/// （σ=0.5、幅值 1.0、0.25 m 铺距、±2.5 m），不经过 crate 的 `cut` ⇒ 独立参考。
fn untruncated_density(x: f32, y: f32, z: f32) -> f32 {
    let mut s = 0.0f32;
    for ix in -10..=10 {
        for iz in -10..=10 {
            let d = ((x - ix as f32 * 0.25).powi(2) + y * y + (z - iz as f32 * 0.25).powi(2))
                / (2.0 * 0.25);
            s += (-d).exp();
        }
    }
    s
}

/// 无截断等值面高度（沿 (x, ·, z) 二分本探针自算的密度）。
fn iso_height_untruncated(x: f32, z: f32) -> f32 {
    let (mut lo, mut hi) = (-1.0f32, 3.0f32);
    let flo = untruncated_density(x, lo, z) - ISO;
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        let fm = untruncated_density(x, mid, z) - ISO;
        if (fm > 0.0) == (flo > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// ⑥ **`f` 的误差规律**（残留"球沉 ≈0.4·r"的量化）：把场给的 `sdf(p)` 与**真实距离**
/// （p 到等值面的竖直距离，由无截断参考二分求）逐深度对比。若 f 随深度**系统性偏大**，
/// 则球停在 `f = r` 处必然比真实面深 —— 这正是实测的 0.4·r，且解释"球越大沉得越多"。
fn f_error_profile() {
    println!("\n⑥ `f` 误差规律（平场；真实距离由无截断参考沿竖直二分求得）：");
    let mut f = flat_field(0.5);
    f.cut = 16.0;
    let y_surface = iso_height_untruncated(0.0, 0.0);
    println!("   无截断等值面 y = {y_surface:.4}");
    println!("   中心 y     深度(面下)    f(p)      真实距离   f/真实");
    for y in [2.0f32, 1.8, 1.6, 1.5, 1.3963, 1.3, 1.2, 1.0, 0.8, 0.5, 0.0] {
        let p = Vec3::new(0.0, y, 0.0);
        let fv = f.sdf(p);
        let d_true = y - y_surface; // 正 = 面之上
        let ratio = if d_true.abs() > 1e-6 {
            fv / d_true
        } else {
            f32::NAN
        };
        println!(
            "   {y:8.4}  {:+9.4}   {fv:+9.4}  {d_true:+9.4}   {ratio:7.3}",
            -d_true
        );
    }
}
