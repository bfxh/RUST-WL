//! **2b 精度探针**（仪表，只打印不断言）：把"边界粒子反作用"的误差**按两个轴实测成表**，
//! 供 `OPEN-PROBLEMS.md` P7 定"适用边界"与"升级路径"用——此前的"体 ≲2h 失准"是**推断**，
//! 这里给出读数。
//!
//! 两轴：
//! - **体尺寸 / h**（固定 h = 0.1）：体半长 0.04 / 0.06 / 0.09 / 0.12 / 0.15（= 0.8h…3h）；
//! - **分辨率 h**（固定体半长 0.12）：h = 0.1 / 0.05（s = h/2 ⇒ 粒子数 ×8）。
//!
//! 每格的判据量（与 crate 门 #13 同源）：全潜盒的
//! `f.y / ρVg`（目标 1.0）、`max(|f.x|,|f.z|) / ρVg`（对称场景应近 0）、`|ω|`（静止体应近 0）、
//! 深潜位姿漂移 `Δy`、**边界粒子数/粒子数**（造价）。场景口径与门一致（铸装水块 + `Tank`）。
//!
//! 跑法（**探针只打印**；用 release 否则太慢）：
//! ```bash
//! cargo test --release -p vxl-phys-fluid --test boundary_accuracy_probe -- --ignored --nocapture
//! ```

use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

/// 水槽（半空间族）：地板 y=0 + 四壁内面 x/z = ±`CAV`，堰高 `WALL`。
/// 与 crate 测试里的 `Tank` 同约定（外法线朝流体、depth = probe − sdf），但腔口可调。
struct Basin {
    cav: f32,
    wall: f32,
}

impl Basin {
    /// 面表：`(轴, 平面位置, 内向法线符号)`。
    fn faces(&self) -> [(usize, f32, f32); 5] {
        [
            (1, 0.0, 1.0),
            (0, -self.cav, 1.0),
            (0, self.cav, -1.0),
            (2, -self.cav, 1.0),
            (2, self.cav, -1.0),
        ]
    }
}

impl ProviderColliders for Basin {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(Aabb {
            min: Vec3::new(-50.0, -50.0, -50.0),
            max: Vec3::new(50.0, 50.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, probe: f32, out: &mut Vec<InteropContact>) -> bool {
        for &(axis, plane, sign) in &self.faces() {
            let comp = [p.x, p.y, p.z][axis];
            // 墙只到堰顶：堰上不设墙（否则是"无限高筒"）。
            if axis != 1 && comp > self.wall {
                continue;
            }
            let sdf = (comp - plane) * sign;
            if sdf > 2.0 * probe {
                continue;
            }
            let point = match axis {
                0 => Vec3::new(plane, p.y, p.z),
                1 => Vec3::new(p.x, plane, p.z),
                _ => Vec3::new(p.x, p.y, plane),
            };
            let normal = match axis {
                0 => Vec3::new(sign, 0.0, 0.0),
                1 => Vec3::new(0.0, sign, 0.0),
                _ => Vec3::new(0.0, 0.0, sign),
            };
            out.push(InteropContact {
                point,
                normal,
                depth: probe - sdf,
                feature: 0,
            });
        }
        true
    }
}

fn still(pos: Vec3) -> BodyPose {
    BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    }
}

/// 一格读数（**窗口均值**——单帧快照不可信：反作用是大数之差，瞬时值可整号翻转）。
struct Reading {
    /// 窗口内 `mean(f.y) / ρVg`（目标 1.0）。
    ratio_y: f32,
    /// 窗口内 `mean(f.x) / ρVg`（对称场景应近 0）。
    ratio_x: f32,
    /// 窗口内 `mean(f.z) / ρVg`。
    ratio_z: f32,
    /// 窗口内 f.y 的**波动**（max−min）/2 / ρVg —— 单帧读数不可信的量化。
    fluct: f32,
    dy: f32,
    bfrac: f32,
    ms: f64,
}

/// 一次测量：`h` 分辨率、`half` 体半长。水柱**足够深**（体四周 ≥2h 净空）⇒ 读数只反映
/// 体尺寸/h，不掺"体被盆底或水面挤住"的场景效应（上一版探针就栽在这：水深写成了水的体积，
/// 大体被夹在地板与自由面之间 ⇒ size/h=2.4 那格虚高到 10×）。
fn measure(h: f32, half: f32) -> Reading {
    let s = h / 2.0;
    let cav = 0.4f32; // 盆腔内半宽（0.8 见方）
    let wall = 0.9f32; // 堰高（水柱 0.675 ⇒ 不溢）
                       // 水块：足印 0.6、深 1.2（按 h 折算格数）⇒ 摊到 0.8² 腔后深 0.675 m。
    let n = (0.6 / s).round() as usize;
    let nz = (1.2 / s).round() as usize;
    let mut f = FluidSystem::new(
        FluidConfig {
            smoothing_radius: h,
            ..FluidConfig::default()
        },
        Vec3::new(-0.3, 0.0, -0.3),
        [n, nz, n],
        s,
    );
    f.set_boundaries(&[0]);
    // 体放**水柱中部**：体积 / 腔面积 = 水深（上一版漏了这一步）。
    let vol = (n as f32 * s) * (n as f32 * s) * (nz as f32 * s);
    let depth = vol / ((2.0 * cav) * (2.0 * cav));
    let y_body = 0.5 * depth;
    let body = (
        7u32,
        Shape::Box {
            half: Vec3::splat(half),
        },
        still(Vec3::new(0.0, y_body, 0.0)),
    );
    let rho0 = f.config().rest_density;
    let v = (2.0 * half).powi(3);
    let expect = rho0 * v * 9.81;
    let t0 = std::time::Instant::now();
    // ① 静置 60 tick（水块摊开的瞬态衰减）——体作为静态几何一直在场。
    for _ in 0..60 {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Basin { cav, wall });
    }
    // ② 窗口 120 tick：逐 tick 采反作用（**窗口均值** + 波动）。
    let win = 120usize;
    let (mut sy, mut sx, mut sz) = (0.0f64, 0.0f64, 0.0f64);
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for _ in 0..win {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Basin { cav, wall });
        if let Some((fy, _t)) = f
            .boundary_reactions()
            .iter()
            .find(|x| x.0 == 7)
            .map(|x| (x.1, x.2))
        {
            sy += fy.y as f64;
            sx += fy.x as f64;
            sz += fy.z as f64;
            lo = lo.min(fy.y);
            hi = hi.max(fy.y);
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1e3 / (60 + win) as f64;
    let inv = 1.0 / (win as f64 * expect as f64);
    Reading {
        ratio_y: (sy * inv) as f32,
        ratio_x: (sx * inv) as f32,
        ratio_z: (sz * inv) as f32,
        fluct: (hi - lo) * 0.5 / expect,
        // 净空（体心到地板，单位 = 体半长）：检查"体没被挤住"。
        dy: y_body / half,
        bfrac: f.boundary_count() as f32 / f.len() as f32,
        ms,
    }
}

#[test]
#[ignore = "仪表（只打印）：跑法见文件头；release 档约数分钟"]
fn boundary_accuracy_vs_size_and_resolution() {
    println!("轴① 固定 h = 0.1：体半长 → 尺寸/h = 0.8 … 3.0");
    println!(
        "{:>6} {:>7} {:>8} {:>9} {:>9} {:>9} {:>8} {:>8}",
        "half", "尺寸/h", "净空/h", "f.y/ρVg", "f.x/ρVg", "f.z/ρVg", "波动±", "边界/粒子"
    );
    for half in [0.04f32, 0.06, 0.09, 0.12, 0.15] {
        let c = measure(0.1, half);
        println!(
            "{half:>6.3} {:>7.1} {:>8.1} {:>9.2} {:>9.2} {:>9.2} {:>8.2} {:>8.2}",
            half * 2.0 / 0.1,
            c.dy / 2.0,
            c.ratio_y,
            c.ratio_x,
            c.ratio_z,
            c.fluct,
            c.bfrac
        );
    }
    println!("轴② 固定体半长 0.12：分辨率 h → 尺寸/h = 1.2 … 2.4");
    for h in [0.1f32, 0.05, 0.04] {
        let c = measure(h, 0.12);
        println!(
            "h={h:.3} 尺寸/h={:.1} 净空/h={:.1} → f.y/ρVg {:.2}（波动±{:.2}）| f.x {:.2} | f.z {:.2} | 边界/粒子 {:.2} | {:.2} ms/tick",
            0.24 / h,
            c.dy / 2.0,
            c.ratio_y,
            c.fluct,
            c.ratio_x,
            c.ratio_z,
            c.bfrac,
            c.ms
        );
    }
}
