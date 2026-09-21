//! **P5 最小复现：三角网地形穿网判别**（`OPEN-PROBLEMS.md` P5）。
//!
//! 假设：`arena_bench trimesh` 里约 5% 的体穿网，机理是**落在三角形边/顶点附近**
//! 时窄相退化（与本仓 `EXPERIMENTS` R.2/R.3 记录的「面平局 ⇒ 退化到角点见证」同族）。
//!
//! 做法：用与 `scene_trimesh_terrain` **同一套地形**（90 m / 24 段 / 同一 `h`），
//! 但只放**一个**盒子，落点分别取：
//!   ① 四边形**形心**（应正常落住）② **对角线中点**（三角内部，应正常）
//!   ③ 四边形**格点/顶点正上方**（顶点处 4 个三角形共享 ⇒ 退化候选）
//!   ④ **四边形边中点**（2 个三角形共享 ⇒ 退化候选）
//! 判据：落点 ①② 应停在 `y ≈ h + 半高`；若 ③④ 穿到地形之下 ⇒ 机理坐实。
//!
//! 跑法（release）：
//! ```text
//! cargo run --release -q -p vxl-phys --example trimesh_escape_probe
//! ```

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Shape, Vec3};

const SIZE: f32 = 90.0;
const SEG: usize = 24;

/// 与 `arena_bench::scene_trimesh_terrain` **逐字同一套高度函数**。
fn h(x: f32, z: f32) -> f32 {
    (x * 0.13).sin() * 1.6 + (z * 0.11).cos() * 1.4 + ((x + z) * 0.05).sin() * 1.1
}

fn terrain() -> vxl_phys_terrain::mesh::TriMesh {
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
    vxl_phys_terrain::mesh::TriMesh::new(verts, tris)
}

/// 单点投放：`shape` 给出形状与"半尺寸"（盒的半高 / 球半径），返回末态 (y, |v|, 清醒)。
///
/// **为什么用"半尺寸"做自变量**：假设是"网格接触按**体中心**而非**支撑面**"
/// ⇒ 预测**沉降量 ≈ 半尺寸**（尺寸变、沉降跟着变）。若沉降是常数（例如只与 skin 有关），
/// 假设不成立。这就是本探针的可判别点。
fn drop_one_cfg(
    x: f32,
    z: f32,
    shape: Shape,
    half_size: f32,
    ticks: usize,
    cfg: PhysConfig,
) -> (f32, f32, bool) {
    let mut w = World::new(cfg);
    w.add_mesh(terrain());
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.6 },
        restitution: 0.05,
    });
    let i = w.bodies.len();
    w.add_dynamic(
        shape,
        Vec3::new(x, h(x, z) + 2.0 + half_size, z),
        vxl_phys_core::Quat::IDENTITY,
        1000.0,
    );
    w.bodies.set_material(i, m);
    for _ in 0..ticks {
        w.step();
    }
    let i = w.bodies.len() - 1;
    let (pos, _) = w.bodies.pose(i);
    let v = w.bodies.linvel[i];
    // **故障域判别（P5）**：把稳定后**实际产生的接触**打出来。
    // 若接触报 depth ≈ 0（无穿透）而体已陷下去 ⇒ **提供者查询侧**；
    // 若接触报 depth ≈ 实际压入量（正）而体不出来 ⇒ **消费/求解侧**。
    if std::env::var("PROBE_CONTACTS").is_ok() {
        println!(
            "    接触明细（体 {i} 末态 y {:.4}）：{} 条流形",
            pos.y,
            w.manifolds().len()
        );
        for mf in w.manifolds() {
            let (a, b) = (mf.a as usize, mf.b as usize);
            if a != i && b != i {
                continue;
            }
            print!(
                "      manifold({a},{b}) n=({:.3},{:.3},{:.3})",
                mf.normal.x, mf.normal.y, mf.normal.z
            );
            for p in &mf.points {
                print!(
                    " [p=({:.2},{:.2},{:.2}) depth {:.4} feat {}]",
                    p.point.x, p.point.y, p.point.z, p.depth, p.feature
                );
            }
            println!();
        }
    }
    (pos.y, v.length(), w.bodies.awake[i])
}

fn main() {
    let step = SIZE / SEG as f32;
    // 取一个远离边界的四边形：格点为 (0,0)–(step, step) 那一格。
    let (gx, gz) = (0.0f32, 0.0f32);
    let cx = gx + step * 0.5;
    let cz = gz + step * 0.5;
    let ticks = 400usize;
    println!("地形：{SIZE} m / {SEG} 段 ⇒ 格距 {step:.3} m；投放高度 h+2+半尺寸；{ticks} tick");

    println!("【A】落点对照（盒半高 0.32，默认档）");
    // ⚠️ 要"看接触"就必须**不让它睡**：睡眠体不做检测 ⇒ 末态 0 条流形（本探针实测），
    //    而沉降是在醒着的时候形成的（体带着压入量睡进去）。故 PROBE_CONTACTS 时用
    //    `sleep_time` 极大的配置重跑本段。
    let probe_contacts = std::env::var("PROBE_CONTACTS").is_ok();
    let cfg_a = if probe_contacts {
        println!("   （PROBE_CONTACTS：sleep_time=1e9，保持清醒以便打印接触）");
        PhysConfig {
            sleep_time: 1e9,
            ..PhysConfig::default()
        }
    } else {
        PhysConfig::default()
    };
    let cases: [(&str, f32, f32); 4] = [
        ("① 四边形形心", cx, cz),
        ("② 对角线中点", gx + step * 0.25, gz + step * 0.25),
        ("③ 格点(顶点)正上方", gx, gz),
        ("④ 边中点（x 向）", gx + step * 0.5, gz),
    ];
    for (name, x, z) in cases {
        let (y, v, awake) = drop_one_cfg(
            x,
            z,
            Shape::Box {
                half: Vec3::splat(0.32),
            },
            0.32,
            ticks,
            cfg_a.clone(),
        );
        let surface = h(x, z);
        let rest = surface + 0.32;
        let sink = rest - y; // >0 = 比"支撑面贴地"更低（陷进网格）
        let verdict = if y < surface - 0.5 {
            "❌ 穿网（在地形之下）"
        } else if sink.abs() < 0.05 {
            "✅ 正常停住"
        } else {
            "⚠️ 沉降"
        };
        println!(
            "{name:16} x {x:6.2} z {z:6.2} | 地面 {surface:6.3} 期望静置 {rest:6.3} | 末态 y {y:9.3} 沉降 {sink:+.3} |v| {v:6.2} 清醒 {awake} ⇒ {verdict}"
        );
    }

    // 【B】尺寸扫描（形心处）：FALSIFY 点——"按体中心求接触"预测**沉降 ≈ 半尺寸**；
    //      若沉降是常数（例如只与 skin 有关），则预测不成立、假设被否证。
    println!("【B】尺寸扫描（形心 x {cx:.2} z {cz:.2}）：假设＝「按体中心求接触」⇒ 沉降 ≈ 半尺寸");
    println!("  形状        半尺寸   地面 h   末态 y   **沉降**   沉降/半尺寸");
    for half in [0.12f32, 0.24, 0.48, 0.80] {
        let (y, _v, _a) = drop_one_cfg(
            cx,
            cz,
            Shape::Box {
                half: Vec3::splat(half),
            },
            half,
            ticks,
            PhysConfig::default(),
        );
        let surface = h(cx, cz);
        let sink = surface + half - y;
        println!(
            "  盒(半高)    {half:5.2}   {surface:7.3}  {y:7.3}   {sink:+7.3}     {:6.2}",
            sink / half
        );
    }
    for r in [0.20f32, 0.40, 0.70] {
        let (y, _v, _a) = drop_one_cfg(
            cx,
            cz,
            Shape::Sphere { radius: r },
            r,
            ticks,
            PhysConfig::default(),
        );
        let surface = h(cx, cz);
        let sink = surface + r - y;
        println!(
            "  球(半径)    {r:5.2}   {surface:7.3}  {y:7.3}   {sink:+7.3}     {:6.2}",
            sink / r
        );
    }

    // 【D】球那一路的决定性读数（P5 唯一还站着的线索）：**接触自己的 depth 就是它的判词**。
    //      不需要"正确的参考面"——`depth` 是接触给出的自述：
    //        · 球沉下去而接触报 depth ≈ 0 ⇒ **提供者/窄相侧算错**（它以为贴着）
    //        · 球沉下去而接触报 depth ≈ 实际压入量（正） ⇒ **消费/求解侧没顶出去**
    //      必须**不睡**（睡眠体不做检测 ⇒ 末态无流形）。放**真正的三角内部点**（②，避开对角线）。
    println!("【D】球那一路：真内部点 (0.94, 0.94) 的接触自述（不睡：sleep_time=1e9）");
    for r in [0.20f32, 0.40, 0.70] {
        let no_sleep = PhysConfig {
            sleep_time: 1e9,
            ..PhysConfig::default()
        };
        let (y, _v, awake) = drop_one_cfg(
            gx + step * 0.25,
            gz + step * 0.25,
            Shape::Sphere { radius: r },
            r,
            ticks,
            no_sleep,
        );
        let surface = h(gx + step * 0.25, gz + step * 0.25);
        println!(
            "  球 r={r:.2}：末态 y {y:.3}、地面 h {surface:.3}、中心相对地面 {:+.3}（期望 +{r:.2}）、清醒 {awake}",
            y - surface
        );
        if std::env::var("PROBE_CONTACTS").is_ok() {
            println!("     （接触明细见上：本探针在 PROBE_CONTACTS 下逐次打印）");
        }
    }

    // 【C】迭代预算判别：假设＝"网格接触的去穿透受**迭代数**限制（软接触）"
    //      ⇒ 换到参考配方（16 迭代 / 16 子步）后**沉降应大幅缩小**。
    //      若沉降几乎不变 ⇒ 不是迭代预算问题，而是**几何/压入量本身算错**。
    let fine = PhysConfig {
        velocity_iterations: 16,
        normal_inner: 1,
        substeps: 16,
        ..PhysConfig::default()
    };
    println!("【C】参考配方对照（iters 16 / inner 1 / substeps 16，形心处）");
    println!("  形状        半尺寸  默认档沉降   **参考配方沉降**");
    for half in [0.24f32, 0.48, 0.80] {
        let (y0, _v0, _a0) = drop_one_cfg(
            cx,
            cz,
            Shape::Box {
                half: Vec3::splat(half),
            },
            half,
            ticks,
            PhysConfig::default(),
        );
        let (y1, _v1, _a1) = drop_one_cfg(
            cx,
            cz,
            Shape::Box {
                half: Vec3::splat(half),
            },
            half,
            ticks,
            fine.clone(),
        );
        let surface = h(cx, cz);
        println!(
            "  盒(半高)    {half:5.2}   {:+8.3}     {:+8.3}",
            surface + half - y0,
            surface + half - y1
        );
    }
    for r in [0.20f32, 0.40, 0.70] {
        let (y0, _v0, _a0) = drop_one_cfg(
            cx,
            cz,
            Shape::Sphere { radius: r },
            r,
            ticks,
            PhysConfig::default(),
        );
        let (y1, _v1, _a1) =
            drop_one_cfg(cx, cz, Shape::Sphere { radius: r }, r, ticks, fine.clone());
        let surface = h(cx, cz);
        println!(
            "  球(半径)    {r:5.2}   {:+8.3}     {:+8.3}",
            surface + r - y0,
            surface + r - y1
        );
    }
}
