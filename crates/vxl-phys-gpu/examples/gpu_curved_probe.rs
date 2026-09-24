//! **曲面壁的投影精度**（`PLAN-gpu.md` §16）——卡上那张 **tick 起点**平面表 vs **当前点**重查（CPU 口径）。
//!
//! **为什么单独成探针**：平面壁（体素容器、盒子）上"用 tick 起点的表做投影"是**精确**的（平面不动）；
//! 曲面壁（三角网/喷溅/体素阶梯）上，表里存的是**t=0 那片最近三角形**的弦平面，粒子在一个子步里会
//! 滑到**相邻**三角形上 ⇒ 推的方向用的是"上一片"的平面。
//!
//! **判据构造（纯几何账 ⇒ 确定性，不含混沌）**：不跑仿真，直接对一组测试点算**两侧各自会给出的推出**：
//! - CPU 口径 = 在**当前点** `p` 调 `contacts_point_boundary`（逐子步重查）；
//! - 卡上口径 = 在**tick 起点** `p0` 收一次表（`gather_wall_project_contacts`），再拿它在 `p` 处
//!   按 `wall_project` 的同式算 `pen`/推出。
//!
//! 两者之差就是**停滞误差**。预期机制：**滑移跨过片边界**时两侧取到不同三角形 ⇒ 差 ≈ 片夹角 × 推量。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_curved_probe`

use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_fluid::FluidSystem;
use vxl_phys_terrain::mesh::TriMesh;

/// 槽的曲率半径（浅碗：R 越大越平；弦高 = R(1 − cos(半角))）。
const R: f32 = 2.0;
/// 槽沿 z 的半长。
const HALF_L: f32 = 0.25;
/// 静置线（与引擎同口径：`skin = 0.15h`，h = 0.1）。
const SKIN: f32 = 0.015;
/// 核半径（表的收集半径）。
const H: f32 = 0.1;

/// 多边逼近的**圆槽**：θ ∈ [π, 2π] 的圆弧（最低点在 (0,0,z)），用 `n` 片四边形拼。
/// 每片是**弦平面** ⇒ 片夹角 = π/n，这就是这套离散的固有角误差。
fn trough(n: usize) -> TriMesh {
    let (mut verts, mut tris): (Vec<Vec3>, Vec<[u32; 3]>) = (Vec::new(), Vec::new());
    let pt = |t: f32| Vec3::new(R * t.cos(), R + R * t.sin(), 0.0);
    for i in 0..n {
        let (t0, t1) = (
            std::f32::consts::PI * (1.0 + i as f32 / n as f32),
            std::f32::consts::PI * (1.0 + (i + 1) as f32 / n as f32),
        );
        let (a, b) = (pt(t0), pt(t1));
        let mid = (t0 + t1) * 0.5;
        // 朝外法线（指回流体侧 = 指向上）：−（cos θ, sin θ, 0）
        let nrm = Vec3::new(-mid.cos(), -mid.sin(), 0.0);
        let q = TriMesh::quad(
            (a + b) * 0.5,
            (b - a) * 0.5,
            Vec3::new(0.0, 0.0, HALF_L),
            nrm,
        );
        let off = verts.len() as u32;
        verts.extend_from_slice(q.verts());
        for t in q.tris() {
            tris.push([t[0] + off, t[1] + off, t[2] + off]);
        }
    }
    TriMesh::new(verts, tris)
}

/// 第 `i` 片的**外法线**（与 `trough` 同式）。
fn quad_normal(n: usize, i: usize) -> Vec3 {
    let mid = std::f32::consts::PI * (1.0 + (i as f32 + 0.5) / n as f32);
    Vec3::new(-mid.cos(), -mid.sin(), 0.0)
}

/// 弧上角度 `t` 处的**真实曲面点**的静置线点（沿真实法线外移 `SKIN`）。
fn rest_point(t: f32) -> Vec3 {
    let c = Vec3::new(R * t.cos(), R + R * t.sin(), 0.0);
    let nrm = Vec3::new(-t.cos(), -t.sin(), 0.0); // 指向圆心（流体侧）
    c + nrm * SKIN
}

/// 一侧的**推出**（`pen > 0` 才推，`min(pen + SKIN, H)`；与 `wall_project` 同式）。
fn push_of(plane: Option<(Vec3, Vec3)>, p: Vec3) -> Vec3 {
    let Some((pt, n)) = plane else {
        return Vec3::ZERO;
    };
    let sdf = (p - pt).dot(n);
    let pen = -sdf;
    if pen <= 0.0 {
        return Vec3::ZERO;
    }
    n * (pen + SKIN).min(H)
}

/// CPU 口径：在**当前点** `p` 重查（取第一笔 `pen > 0` 的接触）。
fn cpu_plane(prov: &dyn ProviderColliders, id: u32, p: Vec3) -> Option<(Vec3, Vec3)> {
    let mut out: Vec<InteropContact> = Vec::new();
    if !prov.contacts_point_boundary(id, p, SKIN, &mut out) {
        return None;
    }
    out.first().map(|c| (c.point, c.normal))
}

/// 卡上口径：**tick 起点** `p0` 收一次表，取该粒的第一片平面。
fn gpu_plane(prov: &dyn ProviderColliders, id: u32, p0: Vec3) -> Option<(Vec3, Vec3)> {
    let (ids, _st, pl) = FluidSystem::gather_wall_project_contacts(&[id], H, &[p0], prov);
    if ids.is_empty() {
        return None;
    }
    pl.first().copied()
}

fn main() {
    println!("== 曲面壁的**投影停滞误差**（纯几何账：tick 起点表 vs 当前点重查）==");
    println!("  圆槽 R={R} | 静置线 sdf=+{SKIN} | 表半径 h={H}");
    println!("  ── 逐片夹角与最坏停滞误差（测试点扫过整条弧；滑移 = 切向 Δs）──");
    for n in [4usize, 8, 16, 32, 64, 128] {
        let mesh = trough(n);
        let mut w = World::new(PhysConfig::default());
        let id = w.add_mesh(mesh);
        let prov = w.providers();
        let dth = std::f32::consts::PI / n as f32;
        let sag = R * (1.0 - dth.cos());
        // ① **典型**（滑移 mm 级，几乎不跨片）：扫整条弧 × 若干滑移量
        let mut worst = 0.0f32;
        for i in 0..n * 4 {
            let t0 = std::f32::consts::PI * (1.0 + i as f32 / (n * 4) as f32);
            let p0 = rest_point(t0);
            for ds in [0.0f32, 0.001, 0.002, 0.005, 0.01] {
                let dt = ds / R;
                let p = rest_point(t0 + dt);
                let a = push_of(cpu_plane(prov, id, p), p);
                let b = push_of(gpu_plane(prov, id, p0), p);
                worst = worst.max((a - b).length());
            }
        }
        // ② **刻意跨片边界**（最坏情况尖峰）：p0 在边界前 0.5 mm、p 在边界后 0.5 mm 且下沉 pen
        let mut worst_cross = 0.0f32;
        let d = 0.0005 / R;
        for k in 0..n {
            let tb = std::f32::consts::PI * (1.0 + (k + 1) as f32 / n as f32);
            let p0 = rest_point(tb - d);
            let nb = Vec3::new(-tb.cos(), -tb.sin(), 0.0);
            for pen in [0.001f32, 0.005, 0.01] {
                let p = rest_point(tb + d) - nb * pen;
                let a = push_of(cpu_plane(prov, id, p), p);
                let b = push_of(gpu_plane(prov, id, p0), p);
                worst_cross = worst_cross.max((a - b).length());
            }
        }
        // 尖峰的量级预测：片夹角 × 推量（pen=5 mm ⇒ pen+skin = 20 mm）
        let predict = dth * (0.005 + SKIN);
        println!(
            "     n={n:>3} 片：片夹角 {:>5.2}° | 弦高 {:>6.2} mm | 典型 **{:.2e} m** | **跨片尖峰 {:.3e} m** | 量级预测 {:>6.2} mm",
            dth.to_degrees(),
            sag * 1e3,
            worst,
            worst_cross,
            predict * 1e3
        );
        // 静默 `quad_normal` 的用途：留一个"法线可查"的入口给将来的逐片核对
        let _ = quad_normal(n, 0);
    }
}
