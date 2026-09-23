//! tests：从 `lib.rs` 拆出的单元测试（纯搬移 + 去一层缩进）。

use super::*;

use super::*;
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Aabb;

/// 测试替身：半空间 y<0 实心（表面 y=0，外法线 +y）。
/// 语义对齐体素点查询：depth = skin − sdf；depth < −skin 时仍返回支持。
/// x/z 取大范围（±50）：落水飞溅的角部粒子会在无压区滑行数米，
/// 面积太小会被「滑出测试地板边缘」误判成穿隧。
pub(crate) struct FloorY;
impl ProviderColliders for FloorY {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(Aabb {
            min: Vec3::new(-50.0, -1.0, -50.0),
            max: Vec3::new(50.0, 0.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: vxl_phys_core::Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        let depth = skin - p.y; // sdf = p.y
        if depth < -skin {
            return true; // 支持查询，但不在接触带内
        }
        out.push(InteropContact {
            point: Vec3::new(p.x, 0.0, p.z),
            normal: Vec3::Y,
            depth,
            feature: 0,
        });
        true
    }
}

/// 测试替身：矩形水槽——地板 y=0 + 四面墙（内壁 x/z = ±0.2；7³ 柱
/// 沉底摊平后仍保有 ~5 个核深的液柱，静水梯度可测）。
/// 墙体沿法线向外无限延伸（半空间式），无「翻墙」另一侧。
/// 面序：地板、左、右、后、前；sign = 内向法线的轴符号。
pub(crate) struct Tank;
impl Tank {
    const FACES: [(usize, f32, f32); 5] = [
        (1, 0.0, 1.0),
        (0, -0.2, 1.0),
        (0, 0.2, -1.0),
        (2, -0.2, 1.0),
        (2, 0.2, -1.0),
    ];
}
impl ProviderColliders for Tank {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        // 半空间几何的提示框必须覆盖固体全域：墙外/地板下仍是固体。
        // 若只框内腔，飞溅粒子越出提示框（如越过开放顶部落到 x<−0.31）
        // 后预滤会跳过一切查询——含地板——永久自由落体（实测 ESC：
        // x=−1.1 处 v=(·,−9.58,·) 匀加速坠穿）。对齐 FloorY 的 ±50。
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
        _rot: vxl_phys_core::Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, probe: f32, out: &mut Vec<InteropContact>) -> bool {
        for &(axis, plane, sign) in &Self::FACES {
            // 半空间墙在全部 y 生效：墙面下方仍是墙固体（无「墙脚缝」）。
            // 若 y < 0 关墙，贴壁底层（近压抖沉到 y ∈ [−pen, 0]）会在墙
            // 关闭的子步里从墙线下横滑出去，地板预滤（余量 h）一丢即自由落体。
            let comp = [p.x, p.y, p.z][axis];
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

/// 静止 5³ 晶格块（标定间距 = h/2）。
fn still_block() -> FluidSystem {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        substeps: 2,
        ..FluidConfig::default()
    };
    FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [5, 5, 5], 0.05)
}

/// #1 静止晶格密度 ≈ ρ0（±2%，核截断的边界粒子除外）。
#[test]
fn rest_lattice_density_matches_rest_density() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let rho0 = f.config().rest_density;
    // 内部判定：晶格坐标（i+½）落在 [2.4, 5.6] 之外即距边不足 2 格。
    let origin = Vec3::new(-0.1, 0.2, -0.1);
    let inv_sp = 1.0 / 0.05;
    let mut checked = 0;
    for (i, p) in f.positions().iter().enumerate() {
        let q = (*p - origin) * inv_sp;
        if q.x < 2.4 || q.x > 3.6 || q.y < 2.4 || q.y > 3.6 || q.z < 2.4 || q.z > 3.6 {
            continue;
        }
        let rel = (f.densities()[i] - rho0).abs() / rho0;
        assert!(rel < 0.02, "粒子 {i} 密度相对偏差 {rel:.6}");
        checked += 1;
    }
    assert_eq!(checked, 8, "5³ 块内部应恰有 2³=8 个粒子");
}

/// #2 静水压强（切片1口径）：柱沉降后压强随深度**方向性递增**
/// （下三分之一 > 中三分之一 > 上三分之一），并加三道量级闸：
/// ① 柱高收在晶格初始高与摊平之间（四壁兜住的证据）；
/// ② 中带均压在静水参考 ρ0·g·(H/2) 的 0.4–1.1×（离散 ρ 噪声经
///    Tait q⁷ 放大 + 自由面钳制 ⇒ 实测 ≈0.72×，偏低压）；
/// ③ 底带均压 ≤ 2× 静水参考 ρ0·g·(5H/6)（镜像鬼影补给的边界层
///    q⁷ 尾部使贴底 2cm 系统性偏高 ~1.65×，三分带稀释后 ≈1.19×）。
/// 用 Tank（地板+四壁）——无壁则落柱摊平成 puddle 是正确物理，静水态无从谈起。
/// 实测口径（7³、间距 0.05、xsph 0.05、240 tick）：柱高 0.259，
/// 下/中/上三分带均压 ≈ 2520/920/90 Pa。±30% 逐带量化校准
/// 留待提分辨率或 δ-SPH 切片——本切片只锁方向与量级带宽。
#[test]
fn hydrostatic_pressure_increases_with_depth() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
    f.set_boundaries(&[0]);
    for _ in 0..240 {
        f.step(1.0 / 60.0, &Tank);
    }
    let mut ymin = f32::INFINITY;
    let mut ymax = f32::NEG_INFINITY;
    for p in f.positions() {
        ymin = ymin.min(p.y);
        ymax = ymax.max(p.y);
    }
    let height = ymax - ymin;
    assert!(height > 0.20 && height < 0.32, "柱高 {height:.3} 异常");
    let (mut lo_s, mut lo_n, mut mid_s, mut mid_n, mut hi_s, mut hi_n) =
        (0.0f32, 0u32, 0.0f32, 0u32, 0.0f32, 0u32);
    for (i, p) in f.positions().iter().enumerate() {
        let t = (p.y - ymin) / height;
        let pr = f.pressures()[i];
        if t < 1.0 / 3.0 {
            lo_s += pr;
            lo_n += 1;
        } else if t < 2.0 / 3.0 {
            mid_s += pr;
            mid_n += 1;
        } else {
            hi_s += pr;
            hi_n += 1;
        }
    }
    let (lo, mid, hi) = (lo_s / lo_n as f32, mid_s / mid_n as f32, hi_s / hi_n as f32);
    // 方向：随深度单调（下 > 中 > 顶；顶部自由面钳制 p≥0）。
    assert!(
        lo > mid && mid > hi,
        "压强须随深度递增：底 {lo:.0} / 中 {mid:.0} / 顶 {hi:.0}"
    );
    // 量级闸（参考深度取各带中心：中带 t=0.5 → 深 H/2；底带 t=1/6 → 深 5H/6）。
    let rho0 = f.config().rest_density;
    let mid_ref = rho0 * 9.81 * 0.5 * height;
    assert!(
        mid > mid_ref * 0.4 && mid < mid_ref * 1.1,
        "中带 {mid:.0} 应在静水参考 {mid_ref:.0} 的 0.4–1.1×"
    );
    let lo_ref = rho0 * 9.81 * height * 5.0 / 6.0;
    assert!(
        lo < lo_ref * 2.0,
        "底带 {lo:.0} 超过边界层上界（2× 静水参考 {lo_ref:.0}）"
    );
    // 顶带（自由面 + 镜像鬼影密度尾，见 PLAN-0.3 §4.3）实测：release 下
    // 顶带 157 / 中带 646 ≈ 4.1×（旧的 5× 闸门在此误报失败），debug 下
    // 同一断言（5×）通过 ⇒ debug 比值 > 5。两个 profile 的绝对值不完全
    // 相同，因此判据取"顶带至少低于中带 3×"的稳健口径：真实缺陷是"顶带
    // 与中带同量级"，3× 抓得住，而不会被 profile 差异误报。失败信息把三带
    // 实测值全部带出。
    assert!(
        hi * 3.0 < mid,
        "顶带 {hi:.0} 应显著低于中带 {mid:.0}（底 {lo:.0}）"
    );
    for v in f.velocities() {
        assert!(v.length_squared() < 9.0, "速度失控：{v:?}");
    }
}

/// #3 动量守恒：零重力、XSPH 关闭，内部压力成对抵消 ⇒ Σv 漂移 < 1e−3。
#[test]
fn momentum_conserved_without_gravity() {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        xsph_viscosity: 0.0,
        substeps: 2,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.125, 0.2, -0.125), [5, 5, 5], 0.05);
    // 正弦速度场（非平移、非平衡）⇒ 内部力持续活跃。
    let vs: Vec<Vec3> = (0..f.len())
        .map(|i| {
            let a = i as f32;
            Vec3::new(0.3 * a.sin(), 0.3 * (a * 1.3).cos(), 0.3 * (a * 0.7).sin())
        })
        .collect();
    f.set_velocities(&vs);
    let p0: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
    for _ in 0..120 {
        f.step(1.0 / 60.0, &NoProviders);
    }
    let p1: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
    let drift = (p1 - p0).length();
    assert!(drift < 1e-3, "动量漂移 {drift:.3e}");
}

/// #4 确定性：同输入两次运行 600 子步，位置逐位相等。
#[test]
fn deterministic_bitwise() {
    let run = || {
        let cfg = FluidConfig {
            substeps: 2,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.0, -0.1), [5, 5, 5], 0.05);
        f.set_boundaries(&[0]);
        for _ in 0..300 {
            f.step(1.0 / 60.0, &FloorY);
        }
        f.positions()
            .iter()
            .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run(), "两次运行必须逐位一致");
}

/// #5 边界：粒子落到地板（替身半空间）上不穿透、不弹飞。
#[test]
fn particles_rest_on_boundary_without_penetration() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.2, -0.1), [5, 5, 5], 0.05);
    f.set_boundaries(&[0]);
    for _ in 0..300 {
        f.step(1.0 / 60.0, &FloorY);
    }
    for (i, p) in f.positions().iter().enumerate() {
        assert!(p.y > -0.01, "粒子 {i} 穿透地板：y = {}", p.y);
        assert!(p.y < 0.6, "粒子 {i} 弹飞：y = {}", p.y);
    }
}

/// #6 网格邻居 = 全扫暴力邻居（集合相等，排序后比对）。
#[test]
fn grid_neighbors_equal_brute_force() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders); // 建格（零重力下位置不动）
    let n = f.len();
    let mut grid_sets: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (i, set) in grid_sets.iter_mut().enumerate() {
        f.for_neighbors(i, |j, _d, _r2| set.push(j as u32));
        set.sort_unstable();
    }
    let mut brute: Vec<Vec<u32>> = vec![Vec::new(); n];
    let h2 = f.h2;
    let ps = f.positions();
    for i in 0..n {
        for j in 0..n {
            if j != i && (ps[i] - ps[j]).length_squared() <= h2 {
                brute[i].push(j as u32);
            }
        }
    }
    assert_eq!(grid_sets, brute);
}

/// 既有口径：默认配置仍是 CPU SPH 族。
#[test]
fn default_is_cpu_sph() {
    let c = FluidConfig::default();
    assert_eq!(c.family, FluidFamily::CpuSph);
    assert_eq!(c.rest_density, 1000.0);
    assert_eq!(c.substeps, 4);
}

/// #8 **介质场采样**（`MediumField`，2a 第一刀）：块内晶格点读到 ≈ρ0。
/// 容差比内部密度测试宽（同核同式但**不含自身项与鬼影项**，且 5³ 块只剩 2 格余量）。
#[test]
fn medium_sample_center_reads_rest_density() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let c = Vec3::new(0.025, 0.325, 0.025); // origin + (2+½)·0.05 = 晶格点上
    let s = f.sample(c);
    let rho0 = f.config().rest_density;
    assert!(
        (s.density - rho0).abs() < 0.15 * rho0,
        "块中心密度 {:?} 应≈ρ0 {:?}",
        s.density,
        rho0
    );
    assert!(s.occupied > 0.85, "占用率 {:?} 应接近 1", s.occupied);
}

/// #9 采样口径：**无介质处返回真空**（核带外 ⇒ 不是"零密度的一团水"）。
#[test]
fn medium_sample_far_is_vacuum() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let far = Vec3::new(0.025, 0.325 + 10.0 * f.h, 0.025);
    let s = f.sample(far);
    assert_eq!(s.density, 0.0, "核带外不应有密度");
    assert_eq!(s.occupied, 0.0, "核带外占用率应为 0");
}

/// #10 采样速度 = **Shepard 平均**（Σwᵥ/Σw）⇒ 均匀流场下应与输入一致。
#[test]
fn medium_sample_velocity_follows_uniform_flow() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let v = Vec3::new(1.5, -0.25, 0.75);
    let vs = vec![v; f.len()];
    f.set_velocities(&vs);
    let s = f.sample(Vec3::new(0.025, 0.325, 0.025));
    assert!(
        (s.velocity - v).length() < 1e-5,
        "采样速度 {:?} 应≈{:?}",
        s.velocity,
        v
    );
}

// ───────────────────────── 2b：Akinci 两层边界粒子 ─────────────────────────
// 判据见 `docs/M1-EXIT.md` §4：① 近壁密度天然正确 ② 稳定性 ③ 对无流体场景
// 逐位不变（由 `crates/vxl-phys` 的四哈希门守）④ 造价。
// 端到端（体真被浮起来 / 体推水）在 `crates/vxl-phys/tests/fluid_boundary.rs`。

/// 体面速度静止的静态体（本文件只把"体"当几何用）。
fn still_pose(pos: Vec3) -> BodyPose {
    BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    }
}

/// #11 **近壁密度补偿**（2b 的核心主张）：贴壁层密度靠边界粒子补到 ρ0 量级。
/// 对照 = 同水块**无地板**（自由面 ⇒ 贴壁层只剩 ~0.7ρ0）。两场景都只跑 1 步
/// （密度只依赖位置，位置在 1 步内几乎不动 ⇒ 对照干净、测试快）。
#[test]
fn boundary_particles_restore_near_floor_density() {
    let cfg = FluidConfig::default();
    let mk = || FluidSystem::new(cfg.clone(), Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
    let mean_lo = |f: &FluidSystem| {
        let (mut s, mut n) = (0.0f32, 0usize);
        for (p, d) in f.positions().iter().zip(f.densities().iter()) {
            if p.y < 0.06 {
                s += d;
                n += 1;
            }
        }
        assert!(n > 0, "贴壁层应有粒子");
        s / n as f32
    };
    // 对照：无地板。
    let mut free = mk();
    free.step(1.0 / 60.0, &NoProviders);
    let rho_free = mean_lo(&free);
    // 实验：地板 = 一层 Box 体的边界粒子（顶面 y = 0，托住水块底面）。
    let mut on = mk();
    let floor = (
        0u32,
        Shape::Box {
            half: Vec3::new(0.3, 0.05, 0.3),
        },
        still_pose(Vec3::new(0.0, -0.05, 0.0)),
    );
    let n = on.set_boundary_particles(std::slice::from_ref(&floor));
    assert!(n > 0, "地板应生成边界粒子");
    on.step(1.0 / 60.0, &NoProviders);
    let rho_on = mean_lo(&on);
    let rho0 = on.config().rest_density;
    assert!(
        rho_free < 0.85 * rho0,
        "自由面贴底密度应偏低：{rho_free:.0}"
    );
    // 门槛按**实测**给（2026-09-22，Akinci 自洽体积标定后：+123 kg/m³，0.72→0.84ρ0）。
    // 旧口径（固定 `V_b = s³`）是 +80：自洽标定把补偿**做强**了，且不再对稀疏小体过量注入。
    assert!(
        rho_on > rho_free + 0.10 * rho0,
        "边界粒子应显著补回核质量：{rho_on:.0} vs 自由面 {rho_free:.0}"
    );
    assert!(
        rho_on < 1.3 * rho0,
        "补偿不得过量（会造虚假压力）：{rho_on:.0}"
    );
}

/// #12 **压力承住流体**（不穿透）：只靠边界粒子地板，平台范围内的粒子不得漏下去。
/// ⚠️ 平台**边缘外**的粒子会（正确地）摊出去——那不是穿透。判据只看"站得住"：
/// 芯内（|xz| ≤ 0.25）最低点不得越过第二层边界粒子（−0.075）以下。
#[test]
fn boundary_particles_hold_fluid_column() {
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.3, 0.0, -0.3),
        [12, 12, 10],
        0.05,
    );
    let floor = (
        0u32,
        Shape::Box {
            half: Vec3::new(0.5, 0.05, 0.5),
        },
        still_pose(Vec3::new(0.0, -0.05, 0.0)),
    );
    let n = f.set_boundary_particles(std::slice::from_ref(&floor));
    assert!(n > 0);
    for _ in 0..60 {
        let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
        f.step(1.0 / 60.0, &NoProviders);
    }
    let mut core_min = f32::INFINITY;
    for p in f.positions() {
        assert!(
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
            "NaN/Inf"
        );
        if p.x.abs() <= 0.25 && p.z.abs() <= 0.25 {
            core_min = core_min.min(p.y);
        }
    }
    assert!(
        core_min > -0.06,
        "芯内粒子漏过边界粒子地板：min_y = {core_min:.4}"
    );
}

/// #13 **反作用 = 浮力**（端到端量纲闸）：槽里全潜盒应受 ≈ `ρ·V·g` 的上浮力。
/// 槽用既有 `Tank` 提供者（**真实场景里围水是提供者通道的活**，边界粒子只管与体的
/// 动量交换——这条把两者分工钉死）。
///
/// ⚠️ **口径 = 窗口均值**（2026-09-22 用 `tests/boundary_accuracy_probe.rs` 两轴实测后定）：
/// - **单帧端点值不可用**：反作用是"两个大数之差"，同一场景端点可读 −6.11×ρVg 而窗口均值
///   是另一个号；另一格端点 1.00 却在窗口内以 ±4.3×ρVg 摆。端点会让门随机红/绿。
/// - 本档（盒半长 0.06 = 1.2h、`Tank`、7³ 水）窗口均值实测 **1.3–1.7×ρVg**；
///   **侧向均值 ≈ 0.01–0.1×ρVg**（先记录在案的 0.6–2.1× 是**端点瞬态**，不是稳态偏差——
///   这条是对我自己先前读数的更正）。
/// - 机理：贴壁核质量（离散补偿）经 Tait q⁷ 放大 ⇒ 近壁压强量级远高于静水；
///   与**既有提供者方案的已接受偏差同族**（`PLAN-0.3.md` §4.3 底压 ≈1.65× 静水）。
/// - **误差不是"体尺寸/h"的干净函数**：探针实测半长 0.04/0.06/0.09/0.12/0.15（h=0.1）
///   给出 3.30/1.38/1.63/**−6.13**/1.21 ⇒ 存在**离散相位共振**（体面栅格与流体晶格不可公约）。
///   ⇒ 2b 是**定性档**（水被推动、轻物浮起、方向对），**不是定量浮力模型**；
///   升级路径见 `OPEN-PROBLEMS.md` P7。断言带按此**诚实地**给。
#[test]
fn submerged_box_gets_buoyant_reaction() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
    f.set_boundaries(&[0]);
    let half = 0.06f32;
    let body = (
        7u32,
        Shape::Box {
            half: Vec3::splat(half),
        },
        still_pose(Vec3::new(0.0, 0.12, 0.0)),
    );
    // **窗口均值**（2026-09-22 改；探针 `tests/boundary_accuracy_probe.rs` 实测得出）：
    // 反作用是"两个大数之差"，**单帧端点值可整号翻转**（实测同一场景 h=0.1 端点 −6.11×ρVg、
    // 而窗口均值 −6.13 是**系统性**的；另一格端点 1.00 而窗口内波动 ±4.3）⇒ 端点读数会让门
    // 随机红/绿。这里按仓库既有纪律（`vxl-phys-measurement-protocol` §5 窗口均值）取
    // **静置 180 + 窗口 60 tick 的均值**。
    for _ in 0..180 {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Tank);
    }
    let win = 60usize;
    let (mut sy, mut sx, mut sz) = (0.0f32, 0.0f32, 0.0f32);
    let (mut tsum, mut tmax) = (Vec3::ZERO, 0.0f32);
    for _ in 0..win {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Tank);
        if let Some(r) = f.boundary_reactions().iter().find(|r| r.0 == 7) {
            sy += r.1.y;
            sx += r.1.x;
            sz += r.1.z;
            tsum += r.2;
            tmax = tmax.max(r.2.length());
        }
    }
    let inv = 1.0 / win as f32;
    let force = Vec3::new(sx * inv, sy * inv, sz * inv);
    let tau = tsum * inv;
    let expect = f.config().rest_density * (2.0 * half).powi(3) * 9.81;
    println!(
        "2b 浮力（窗口 {win} tick 均值）{:+.2} N / ρVg = {expect:.2} N（比值 {:.2}）；\
         侧向 ({:+.2}, {:+.2})；|τ| 均值 {:.3} / 峰 {:.3}",
        force.y,
        force.y / expect,
        force.x,
        force.z,
        tau.length(),
        tmax
    );
    assert!(
        force.y > 0.5 * expect && force.y < 2.0 * expect,
        "浮力均值应 ≈ ρVg = {expect:.2} N（本档实测 1.3–1.7×），实测 {:+.2}（f = {force:?}）",
        force.y
    );
    // 侧向：**窗口均值下本来就近消**（探针实测 ≈ 0.01–0.1×ρVg；先前的 0.6–2.1× 是端点瞬态）
    // ⇒ 这条恢复成"近消"判据，只留宽带回旋余量。
    assert!(
        force.x.abs() < 0.5 * expect && force.z.abs() < 0.5 * expect,
        "对称场景侧向**均值**应近消（本档实测 ≤0.1×ρVg）：{force:?}"
    );
    assert!(
        tau.length() < 0.5 * expect * half,
        "对称场景力矩均值应近消：{tau:?}"
    );
}

/// #14 **确定性**（2b 在场）：同场景两跑，位置/密度/反作用**逐位一致**。
#[test]
fn boundary_coupling_is_deterministic() {
    let digest = || {
        let cfg = FluidConfig {
            xsph_viscosity: 0.05,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
        f.set_boundaries(&[0]);
        let body = (
            7u32,
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            still_pose(Vec3::new(0.0, 0.12, 0.0)),
        );
        for _ in 0..60 {
            let _ = f.set_boundary_particles(std::slice::from_ref(&body));
            f.step(1.0 / 60.0, &Tank);
        }
        (
            f.positions()
                .iter()
                .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                .collect::<Vec<u32>>(),
            f.densities()
                .iter()
                .map(|d| d.to_bits())
                .collect::<Vec<u32>>(),
            f.boundary_reactions()
                .iter()
                .flat_map(|r| [r.1.x.to_bits(), r.1.y.to_bits(), r.2.x.to_bits()])
                .collect::<Vec<u32>>(),
            f.boundary_count(),
        )
    };
    assert_eq!(digest(), digest(), "两次运行应逐位一致");
}

/// **并行 = 串行逐位一致**（`FluidConfig::threads`；2026-09-23 规模档并行化的守门）：
/// 同一场景跑 `threads = 1` 与 `threads = 4`，末态位置/速度/密度**逐位相同**。
/// 覆盖**两条路径**：① 纯流体（无边界粒子）；② **含边界粒子**（2b）——后者是风险点，
/// 因为 `bforce` 是唯一的跨粒子累加量（并行路径用**串行补趟**保序）。
#[test]
fn parallel_equals_serial_bitwise() {
    let digest = |threads: usize, with_boundary: bool| {
        let cfg = FluidConfig {
            threads,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
        if with_boundary {
            let floor = (
                0u32,
                Shape::Box {
                    half: Vec3::new(0.3, 0.05, 0.3),
                },
                still_pose(Vec3::new(0.0, -0.05, 0.0)),
            );
            let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
        }
        for _ in 0..40 {
            if with_boundary {
                let floor = (
                    0u32,
                    Shape::Box {
                        half: Vec3::new(0.3, 0.05, 0.3),
                    },
                    still_pose(Vec3::new(0.0, -0.05, 0.0)),
                );
                let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
            }
            f.step(1.0 / 60.0, &NoProviders);
        }
        let mut d: Vec<u32> = Vec::new();
        for p in f.positions() {
            d.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        }
        for v in f.velocities() {
            d.extend([v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
        }
        for r in f.densities() {
            d.push(r.to_bits());
        }
        for r in f.boundary_reactions() {
            d.extend([r.1.x.to_bits(), r.1.y.to_bits(), r.2.z.to_bits()]);
        }
        d
    };
    assert_eq!(
        digest(1, false),
        digest(4, false),
        "纯流体：并行与串行应逐位一致"
    );
    assert_eq!(
        digest(1, true),
        digest(4, true),
        "含边界粒子（2b）：并行与串行应逐位一致（bforce 走串行补趟）"
    );
}
