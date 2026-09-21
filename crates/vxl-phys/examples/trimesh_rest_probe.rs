//! **P5 代价侧最小复现：起伏三角网上"静置高度"**（`OPEN-PROBLEMS.md` P5 ⑰）。
//!
//! 背景：9 引擎对拍（PhysArena）里 vxl 的盒子在起伏三角网上读出**全身最低采样点离地
//! 0.094 m**，看着像真悬空，而 Rapier/Jolt/PhysX/Bullet/Havok/Crashcat 都在 ±0.02 m 内。
//! 同一份对拍里 vxl 是**唯一**能把陷进球体的体顶出来的引擎 ⇒ 一度怀疑这是
//! "可恢复性 ↔ 静置精度"取舍的代价侧。
//!
//! **本探针的结论（2026-09-21）：0.094 m 是瞬态，不是静置值。** 逐字同场景下盒子在
//! **下坡翻滚**（全身间隙在 ±0.5 m 间摆、|v| 1–4 m/s），t≈225 才安静、之后缓慢爬行；
//! arena 的 300 tick 采样窗口正好落在翻滚里（逐位确定性 ⇒ 三次读同一个数，但**不等于已静置**）。
//! 换 900 tick 窗口后**同一 arena 探针直接通过**。真代价是皮肤带那 ~2 cm（见 ⑤⑥ 段），
//! 且根源在 `contacts_point` 的 `depth = skin − sd`（偏离 `interop` 契约一个 skin）；
//! 球那一路没有这个偏移（`depth = radius − sd`）——这条形状差异也在下面的读数里。
//!
//! 本探针把 PhysArena 那条场景**逐字**搬进进程内（±4 m / 8×8 格 / 同一 H / 同一三角化顺序 /
//! 同一盒半高 / `PhysConfig::default()` / 按 arena 的 `vxl_body_material` 给**两个体**都上 μ=0.9），并且：
//!   ① 逐 tick 打印轨迹（看它什么时候安静、什么时候睡）+ 末态**全身采样**（6 面 × 7×7 = 294 点）
//!      相对解析面的最小竖直间隙（与 arena 判据同口径）；
//!   ② 在末态**直接调 `TriMesh::contacts_box`**，看提供者到底产生了哪些接触（q、n、depth）——
//!      "为什么撑住了"必须由接触集本身回答，不能靠猜；
//!   ③ 对**最低采样点**直接调提供者的最近面查询，打印它认为的最近距离/最近点/法线；
//!   ④ 不睡对照（排除"睡在半空"）；⑤ 贴面投放对照（把静置精度与翻滚轨迹分开）；
//!   ⑥ 平地网格对照；⑦ 均匀 11.3° 斜面「预旋贴合」对照（判"斜面上摩擦是否失效"）。
//!
//! 跑法（release；`PROBE_TICKS` 可改 tick 数，默认 300）：
//! ```text
//! PROBE_TICKS=900 cargo run --release -q -p vxl-phys --example trimesh_rest_probe
//! ```

use vxl_phys::*;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::mesh::TriMesh;

const N: usize = 8; // 8×8 格
const S: f32 = 4.0; // ±4 m ⇒ 格距 1 m（与 arena 探针一致）
const HALF: f32 = 0.35;
fn ticks() -> usize {
    std::env::var("PROBE_TICKS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(300)
}

/// 与 arena `shape-trimesh-hilly` **逐字同一套**高度函数。
fn h(x: f32, z: f32) -> f32 {
    0.25 * (0.8 * x).sin() + 0.2 * (0.9 * z).cos()
}

/// 与 arena 同一套顶点/索引顺序（对角线方向会影响落点邻域的面分布）。
fn terrain() -> TriMesh {
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=N {
        for ix in 0..=N {
            let x = -S + (2.0 * S * ix as f32) / N as f32;
            let z = -S + (2.0 * S * iz as f32) / N as f32;
            verts.push(Vec3::new(x, h(x, z), z));
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

/// 全身体表面采样：6 面 × 7×7 = 294 个**局部**采样点（含全部棱与角）。
/// 为什么必须全身采：盒子翻滚后可能躺在任何一面上（甚至翻过来），"底面"这个说法不成立；
/// 且起伏面上刚性盒**本来就不会多点贴地** ⇒ 只有"表面有没有任何一处贴地"这一个口径站得住。
fn body_samples() -> Vec<Vec3> {
    const K: usize = 7;
    let mut out = Vec::with_capacity(6 * K * K);
    for axis in 0..3usize {
        for sgn in [-1.0f32, 1.0] {
            for i in 0..K {
                for j in 0..K {
                    let u = -HALF + (2.0 * HALF * i as f32) / (K as f32 - 1.0);
                    let v = -HALF + (2.0 * HALF * j as f32) / (K as f32 - 1.0);
                    out.push(match axis {
                        0 => Vec3::new(sgn * HALF, u, v),
                        1 => Vec3::new(u, sgn * HALF, v),
                        _ => Vec3::new(u, v, sgn * HALF),
                    });
                }
            }
        }
    }
    out
}

/// 全身采样相对解析面的**最小竖直间隙**（与 arena 判据同口径）。
fn min_vertical_gap(pos: Vec3, rot: Quat) -> f32 {
    body_samples()
        .into_iter()
        .map(|l| {
            let w = rot.rotate_vec3(l) + pos;
            w.y - h(w.x, w.z)
        })
        .fold(f32::INFINITY, f32::min)
}

/// 全身采样里最低点的**世界 y**：对 y = 0 的平网格地形 ⇒ 直接就是离地间隙（与姿态无关，
/// 斜停也算对）。
fn min_sample_y(pos: Vec3, rot: Quat) -> f32 {
    body_samples()
        .into_iter()
        .map(|l| (rot.rotate_vec3(l) + pos).y)
        .fold(f32::INFINITY, f32::min)
}

/// 8 个角点相对解析面的最小竖直间隙（第四版判据，保留用于与历代读数对照）。
fn min_corner_gap(pos: Vec3, rot: Quat) -> f32 {
    let mut min_gap = f32::INFINITY;
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let w = rot.rotate_vec3(Vec3::new(sx * HALF, sy * HALF, sz * HALF)) + pos;
                min_gap = min_gap.min(w.y - h(w.x, w.z));
            }
        }
    }
    min_gap
}

fn main() {
    let cfg = PhysConfig::default();
    println!(
        "【P5 代价侧】起伏三角网静置（±{S} m / {N}×{N} 格 / 格距 1 m）\n  \
         配置：dt {:.5}、skin {:.4}、iters {}、substeps {}、sleep_time {:.3}\n  \
         投放：盒半高 {HALF}、落点 (0, H(0,0)+2.5, 0)、{} tick",
        cfg.dt,
        cfg.contact_skin,
        cfg.velocity_iterations,
        cfg.substeps,
        cfg.sleep_time,
        ticks()
    );

    let mesh = terrain();
    let mut w = World::new(cfg.clone());
    let mesh_body = w.add_mesh(mesh.clone()) as usize;
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(0.0, h(0.0, 0.0) + 2.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    // arena 的 vxl 适配器对**每个**体都调  ⇒ 静态三角网体也拿到同一摩擦
    w.bodies.set_material(mesh_body, m);
    println!(
        "
轨迹（每 25 tick 一次；看它**什么时候睡**、睡在多大的间隙上）："
    );
    for t in 0..ticks() {
        w.step();
        if t % 100 == 99 {
            let (p, r) = w.bodies.pose(i);
            println!(
                "   t={:3}  pos=({:7.3},{:7.3},{:7.3})  全身间隙 {:+.4}  清醒 {}  |v| {:.4}",
                t + 1,
                p.x,
                p.y,
                p.z,
                min_vertical_gap(p, r),
                w.bodies.awake[i],
                w.bodies.linvel[i].length()
            );
        }
    }

    let (pos, rot) = w.bodies.pose(i);
    let v = w.bodies.linvel[i];
    let av = w.bodies.angvel(i);
    println!(
        "\n末态：pos = ({:.4}, {:.4}, {:.4})、quat = ({:.4}, {:.4}, {:.4}, {:.4})",
        pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w
    );
    println!(
        "       |v| {:.5} |ω| {:.5} 清醒 {}  中心 − (H(0,0)+{HALF}) = {:+.4}",
        v.length(),
        av.length(),
        w.bodies.awake[i],
        pos.y - (h(0.0, 0.0) + HALF)
    );
    let gap6 = min_vertical_gap(pos, rot);
    let gap8 = min_corner_gap(pos, rot);
    println!(
        "       **全身 294 采样最小间隙 {gap6:+.4} m**；8 角点最小间隙 {gap8:+.4} m\n       \
         （arena 在 **300 tick** 读到的 0.094 m 是本盒**下坡翻滚中的瞬时采样**，不是静置值；\
         换 900 tick 窗口同一探针即通过 ⇒ 见 OPEN-PROBLEMS P5 ⑰）"
    );

    // ② 末态直接问提供者：它到底产生了哪些接触？（撑住的理由必须在接触集里）
    let mut out: Vec<InteropContact> = Vec::new();
    mesh.contacts_box(0, Vec3::splat(HALF), pos, rot, cfg.contact_skin, &mut out);
    println!(
        "\n② `contacts_box` 在末态的直接回答（skin {:.4}）：{} 条接触",
        cfg.contact_skin,
        out.len()
    );
    for c in out.iter().take(20) {
        println!(
            "   point=({:7.4},{:7.4},{:7.4}) n=({:6.3},{:6.3},{:6.3}) depth={:+.5} feat {}",
            c.point.x, c.point.y, c.point.z, c.normal.x, c.normal.y, c.normal.z, c.depth, c.feature
        );
    }

    // ③ 真正的最低采样点：把它拿出来，问最近面查询（距离/最近点/法线），
    //    再按 `depth = skin − (p−q)·n` 复算一遍提供者的判据。
    let mut best = (f32::INFINITY, Vec3::ZERO);
    for axis in 0..3usize {
        for sgn in [-1.0f32, 1.0] {
            for a in 0..2usize {
                for b in 0..2usize {
                    let u = if a == 0 { -HALF } else { HALF };
                    let v = if b == 0 { -HALF } else { HALF };
                    let l = match axis {
                        0 => Vec3::new(sgn * HALF, u, v),
                        1 => Vec3::new(u, sgn * HALF, v),
                        _ => Vec3::new(u, v, sgn * HALF),
                    };
                    let wpt = rot.rotate_vec3(l) + pos;
                    let gap = wpt.y - h(wpt.x, wpt.z);
                    if gap < best.0 {
                        best = (gap, wpt);
                    }
                }
            }
        }
    }
    let (gap_low, p_low) = best;
    println!(
        "\n③ 最低角点 p=({:.4},{:.4},{:.4})：相对解析面 {gap_low:+.4} m；\
         解析面处 h={:.4}、p.y={:.4}",
        p_low.x,
        p_low.y,
        p_low.z,
        h(p_low.x, p_low.z),
        p_low.y
    );
    let mut single: Vec<InteropContact> = Vec::new();
    mesh.contacts_point(0, p_low, cfg.contact_skin, &mut single);
    if single.is_empty() {
        println!("   `contacts_point` 在该点**无接触**（点已在接触带之外）");
    }
    for c in single.iter() {
        let sd = (p_low - c.point).dot(c.normal);
        let d = (p_low - c.point).length();
        println!(
            "   q=({:7.4},{:7.4},{:7.4}) n=({:6.3},{:6.3},{:6.3}) |p−q|={d:.4} \
             sd=(p−q)·n={sd:+.4} depth=skin−sd={:+.4}",
            c.point.x,
            c.point.y,
            c.point.z,
            c.normal.x,
            c.normal.y,
            c.normal.z,
            cfg.contact_skin - sd
        );
    }

    // ① 末态流形（睡眠体无流形 ⇒ 用极长 sleep_time 重跑一份作对照）
    println!("\n① 末态流形（默认配置 ⇒ 睡着时为 0 条属预期）");
    let mut printed = 0usize;
    for mf in w.manifolds() {
        let (a, b) = (mf.a as usize, mf.b as usize);
        if a != i && b != i {
            continue;
        }
        printed += 1;
        print!(
            "   manifold({a},{b}) n=({:.3},{:.3},{:.3})",
            mf.normal.x, mf.normal.y, mf.normal.z
        );
        for p in &mf.points {
            print!(
                " [p=({:.3},{:.3},{:.3}) depth {:.4} feat {}]",
                p.point.x, p.point.y, p.point.z, p.depth, p.feature
            );
        }
        println!();
    }
    if printed == 0 {
        println!("   （0 条）");
    }

    // ⑤ 静置对照（见文末）
    // ④ 对照：不睡（sleep_time 极大）时是否同样悬空 ⇒ 排除"睡在半空"
    let awake_cfg = PhysConfig {
        sleep_time: 1e9,
        ..PhysConfig::default()
    };
    let mut w2 = World::new(awake_cfg);
    let mesh_body2 = w2.add_mesh(mesh.clone()) as usize;
    let m2 = w2.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let j = w2.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(0.0, h(0.0, 0.0) + 2.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w2.bodies.set_material(j, m2);
    w2.bodies.set_material(mesh_body2, m2);
    for _ in 0..ticks() {
        w2.step();
    }
    let (pos2, rot2) = w2.bodies.pose(j);
    println!(
        "\n④ 不睡对照（sleep_time=1e9）：pos.y {:.4}、全身最小间隙 {:+.4} m、\
         清醒 {}、|v| {:.5}、流形 {} 条\n   ⇒ 若同样悬空 ⇒ 不是「睡在半空」，是接触几何本身",
        pos2.y,
        min_vertical_gap(pos2, rot2),
        w2.bodies.awake[j],
        w2.bodies.linvel[j].length(),
        w2.manifolds()
            .iter()
            .filter(|mf| {
                let (a, b) = (mf.a as usize, mf.b as usize);
                a == j || b == j
            })
            .count()
    );

    // ⑤ 静置对照（贴面投放）：底面离地 5 cm 投放、无翻滚 ⇒ 量的是**纯静置高度**，
    //    把"静置精度"与"翻滚轨迹"分开。这是本探针给 P5 的最终读数。
    for drop in [0.05f32, 0.5, 2.5] {
        let mut w3 = World::new(PhysConfig::default());
        let mb = w3.add_mesh(mesh.clone()) as usize;
        let m3 = w3.add_material(vxl_phys_core::Material {
            friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
            restitution: 0.02,
        });
        let k = w3.add_dynamic(
            Shape::Box {
                half: Vec3::splat(HALF),
            },
            Vec3::new(0.0, h(0.0, 0.0) + HALF + drop, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w3.bodies.set_material(k, m3);
        w3.bodies.set_material(mb, m3);
        for _ in 0..ticks() {
            w3.step();
        }
        let (p3, r3) = w3.bodies.pose(k);
        println!(
            "   投放高度 +{drop:.2} m：末态 pos=({:7.3},{:7.3},{:7.3}) 全身间隙 {:+.4} m              |v| {:.4} 清醒 {} 滑移 {:.3} m",
            p3.x, p3.y, p3.z,
            min_vertical_gap(p3, r3),
            w3.bodies.linvel[k].length(),
            w3.bodies.awake[k],
            ((p3.x - 0.0).powi(2) + (p3.z - 0.0).powi(2)).sqrt()
        );
    }

    // ⑥ 平地对照（同材质、同投放方式）：**平地若也滑 ⇒ 网格摩擦整体失效**；
    //    平地不滑、坡上滑 ⇒ 是切向漂移/静摩擦那一路（与 P1「塔不入睡」同族）。
    let mut flat_v: Vec<Vec3> = Vec::new();
    let mut flat_t: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=2u32 {
        for ix in 0..=2u32 {
            let x = -4.0 + 4.0 * ix as f32;
            let z = -4.0 + 4.0 * iz as f32;
            flat_v.push(Vec3::new(x, 0.0, z));
        }
        let _ = iz;
    }
    let row = 3u32;
    for iz in 0..2u32 {
        for ix in 0..2u32 {
            let a = iz * row + ix;
            flat_t.push([a, a + row, a + 1]);
            flat_t.push([a + 1, a + row, a + row + 1]);
        }
    }
    let mut w4 = World::new(PhysConfig::default());
    let mb4 = w4.add_mesh(TriMesh::new(flat_v, flat_t)) as usize;
    let m4 = w4.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let k4 = w4.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(0.0, 0.5 + HALF, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w4.bodies.set_material(k4, m4);
    w4.bodies.set_material(mb4, m4);
    for _ in 0..ticks() {
        w4.step();
    }
    let (p4, r4) = w4.bodies.pose(k4);
    println!(
        "\n⑥ 平地对照（μ=0.9、从 0.5 m 落到 y=0 的平网格）：末态 y {:.4}（平躺期望 0.35）、\
         全身最低采样点离地 {:+.4} m、水平漂移 {:.4} m、|v| {:.5} 清醒 {}",
        p4.y,
        min_sample_y(p4, r4),
        (p4.x * p4.x + p4.z * p4.z).sqrt(),
        w4.bodies.linvel[k4].length(),
        w4.bodies.awake[k4]
    );

    // ⑦ 斜面判别（零成本、可判别）：**均匀 11.3° 斜面的平网格**，盒子**预旋到与斜面贴合**
    //    投放（无翻滚、无曲率）⇒ 只测"斜面上的静摩擦"这一件事。
    //    预测：若滑走 ⇒ 斜面上的摩擦/切向基（法线是面法线）这一路有问题；
    //          若站住 ⇒ 起伏网上那 2.5 m 滑移是**地形曲率/翻滚**带来的，不是摩擦失效。
    let slope = 11.3f32.to_radians();
    let tan_s = slope.tan();
    let mut sv: Vec<Vec3> = Vec::new();
    let mut st: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=2u32 {
        for ix in 0..=2u32 {
            let x = -4.0 + 4.0 * ix as f32;
            let z = -4.0 + 4.0 * iz as f32;
            sv.push(Vec3::new(x, x * tan_s, z));
        }
    }
    for iz in 0..2u32 {
        for ix in 0..2u32 {
            let a = iz * 3 + ix;
            st.push([a, a + 3, a + 1]);
            st.push([a + 1, a + 3, a + 4]);
        }
    }
    let mut w5 = World::new(PhysConfig::default());
    let mb5 = w5.add_mesh(TriMesh::new(sv, st)) as usize;
    let m5 = w5.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    // 盒预旋到与斜面平行：绕 z 轴转 +slope，沿斜面法线抬起 0.35 + 2 cm 贴合间隙
    let q = Quat::from_axis_angle(Vec3::Z, slope);
    let n_ramp = Vec3::new(-slope.sin(), slope.cos(), 0.0);
    let p0 = n_ramp * (HALF + 0.02);
    let k5 = w5.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        p0,
        q,
        1000.0,
    ) as usize;
    w5.bodies.set_material(k5, m5);
    w5.bodies.set_material(mb5, m5);
    println!(
        "
⑦ 均匀 {:.1}° 斜面对照（预旋贴合投放，μ=0.9；tan θ = {:.3}）：",
        11.3, tan_s
    );
    for t in 0..ticks() {
        w5.step();
        if t % 100 == 99 {
            let (p5, _r5) = w5.bodies.pose(k5);
            let drift =
                ((p5.x - p0.x).powi(2) + (p5.y - p0.y).powi(2) + (p5.z - p0.z).powi(2)).sqrt();
            println!(
                "   t={:3}  位姿=({:7.3},{:7.3},{:7.3})  滑移 {:.4} m  |v| {:.5}  清醒 {}",
                t + 1,
                p5.x,
                p5.y,
                p5.z,
                drift,
                w5.bodies.linvel[k5].length(),
                w5.bodies.awake[k5]
            );
        }
    }
}
