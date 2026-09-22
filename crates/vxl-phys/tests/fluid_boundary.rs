//! **刚体↔液体双向耦合（2b：Akinci 两层边界粒子）** 的端到端门：`ROUTE.md` §4
//! 「液体 → 刚体」格的另一半（刚体 → 液体）。规格见 `docs/M1-EXIT.md` §4。
//!
//! 判据（照 SPEC/§4 逐条落地，不看旋钮）：
//! - **① 体推水**：水下的体扫过时**在前方造出流速场**。判据用**差分**：
//!   同场景 `add_fluid`（2a 粗档，单向）⇒ 流体**不**被推动；`add_fluid_with_boundary_coupling`
//!   （2b）⇒ 前方流体获得同向速度。这条把"双向真的通了"钉死（单向耦合下体穿水而过、
//!   水一动不动）。
//! - **② 稳定性**：无 NaN、水不越堰、速度有界（CFL 钳制）。
//! - **③ 无流体场景四哈希逐位不变**：2b 默认关（`add_fluid` 不填 `fluid_2b`）
//!   ⇒ 由 `m0_gates`/`determinism`/金样门守，本文件不重复跑。
//! - **④ 造价**：边界粒子数 / 粒子数、ms/tick ⇒ 读在示例 `fluid_buoyancy` 的对照表里
//!   （测试只机读断言量级：边界粒子数 ≤ 粒子数的若干倍）。
//! - **⑤ 确定性**：开 2b 后同场景两跑末态逐位一致。
//!
//! ⚠️ 场景**铸装**（`PLAN-0.3.md` §4.2）：水块按沉降后几何直接就位 `[8,8,8]@0.05`
//! 对 0.5 m 盆腔，**任何"带落差入盆"的水块**都会触发顶心喷泉并把断言变成在测失控粒子。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 水槽：5×5 格（外沿 2.5 m）地板 + **中心一格**围堰 ⇒ 内腔 0.5×0.5 m。
/// 与 `tests/fluid_coupling.rs`（2a 门）同款场景，便于对照。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// 铸装水块 `[8,8,8]@0.05`（足印 0.4、深 0.4）在 0.5 m 盆腔里近平衡就位。
fn water() -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

/// 造一个开 2b 的场景：`couple=true` ⇒ 边界粒子耦合；false ⇒ 2a 单向粗档（对照）。
fn scene(couple: bool) -> (World, u32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water();
    if couple {
        w.add_fluid_with_boundary_coupling(sys, &[v]);
    } else {
        w.add_fluid(sys, &[v]);
    }
    (w, v)
}

/// 柱区（体的入水柱）内流体的**峰值速度**。
fn column_peak_v(w: &World) -> f32 {
    let f = &w.fluids()[0].0;
    let mut peak = 0.0f32;
    for (p, v) in f.positions().iter().zip(f.velocities().iter()) {
        if p.x.abs() < 0.15 && p.z.abs() < 0.15 {
            peak = peak.max(v.length());
        }
    }
    peak
}

/// ① **体推水**（`M1-EXIT.md` §4 判据①）：体入水时**把水推开**——2b 开 ⇒ 柱区出现
/// 明显的流场；2a 单向（对照档）⇒ 体像幽灵一样穿过水，柱区几乎不动。
///
/// 口径（判据与场景都踩过几轮才定下）：
/// - 体**从水面上方落下**（不落在已有水格上：重合会让边界粒子与流体粒子近同位 ⇒ ρ 爆 ⇒
///   CFL 尖峰，判据就变成在测尖峰——debug 档那次尖峰读成 +0.97 m/s，release 档读成
///   −0.41 m/s，两侧都不是"推水"）。
/// - 先静置 60 tick（`[8,8,8]` 足印 0.4 < 盆 0.5 ⇒ 水要摊开，瞬态自身 ~2 m/s）。
/// - 判据取**柱区峰值速度**（不用"体前方一层"的平均 vx：体在 2b 档被流体拖得很慢
///   ——20 tick 只走 0.029 m——平均量被残波淹没；入水柱的峰值差是**量级差**，稳）。
/// - 阈值宽（3× + 绝对值下限）⇒ 对 debug/release 的浮点路径差免疫。
#[test]
fn body_pushes_water_only_with_boundary_coupling() {
    let mut res = Vec::new();
    for couple in [false, true] {
        let (mut w, _v) = scene(couple);
        for _ in 0..60 {
            w.step(); // 静置：摊开瞬态衰减
        }
        let quiet = column_peak_v(&w);
        // 从水面上方入水，给一点向下初速 ⇒ 尽快扎进去（体与水格不重合）。
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, 1.50, 0.0),
            Quat::IDENTITY,
            1200.0,
        );
        w.bodies.linvel[b as usize] = Vec3::new(0.0, -0.6, 0.0);
        let mut peak = quiet;
        for _ in 0..40 {
            w.step();
            peak = peak.max(column_peak_v(&w));
        }
        let y_end = w.bodies.position[b as usize].y;
        res.push((quiet, peak - quiet, y_end));
    }
    let (q_off, d_off, y_off) = res[0];
    let (q_on, d_on, y_on) = res[1];
    println!(
        "体推水（入水柱峰值速度）：2a 单向 静置 {q_off:.3} → 增量 {d_off:+.3} m/s（末态 y {y_off:.3}）；         2b 双向 静置 {q_on:.3} → 增量 {d_on:+.3} m/s（末态 y {y_on:.3}）"
    );
    // 对照：单向耦合下体穿水而过，柱区只剩残波。
    assert!(
        d_off < 0.3,
        "单向耦合下柱区不该被搅动（残波以内）：Δ = {d_off:+.3}"
    );
    // 2b：体把水推开，柱区出现量级更大的流场。
    assert!(
        d_on > 0.4 && d_on > 3.0 * d_off,
        "2b 应把入水柱的水推开（且压倒对照）：Δ = {d_on:+.3} vs {d_off:+.3}"
    );
    // 对照档的体会**穿过**水落到盆底；2b 档的体被水托住/减速（不穿底）。
    assert!(y_off < 1.10, "单向档的体应落到盆底：y = {y_off:.3}");
    assert!(y_on > 1.10, "2b 档的体不该穿过水落到盆底：y = {y_on:.3}");
}

/// ② **稳定性 + ④ 造价量级**：2b 开、水先静置、体**从水面上方落入**（轻盒 ⇒ 漂在水面）
/// 泡 150 tick ⇒ 无 NaN、速度有界、水基本留在盆里、边界粒子数 ≤ 8× 粒子数。
///
/// ⚠️ 口径：体**不能直接落在已有的水格上开算**——体与水格重合 ⇒ 边界粒子与流体粒子
/// 近同位 ⇒ ρ 爆 ⇒ CFL 钳制把粒子以 9.6 m/s 抛出去（实测 max|xz| → 2.17 m、vmax 恰好
/// 顶到钳制线）。那是**初值重叠**，不是耦合失稳；干净开局的两种写法：水格按体几何挖空
/// （铸装），或体从空处落入（本测试）。`fluid_buoyancy` 的 2b 档取后者。
#[test]
fn boundary_coupling_is_stable_and_bounded() {
    let (mut w, _v) = scene(true);
    for _ in 0..60 {
        w.step(); // 先让铸装水块自己摊平
    }
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.50, 0.0), // 水面上方（堰顶 1.5）
        Quat::IDENTITY,
        300.0,
    );
    for _ in 0..150 {
        w.step();
    }
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        assert!(p.is_finite(), "体 {i} 位姿非有限：{p:?}");
    }
    let f = &w.fluids()[0].0;
    let (mut max_v, mut max_r, mut outside) = (0.0f32, 0.0f32, 0usize);
    for (p, v) in f.positions().iter().zip(f.velocities().iter()) {
        assert!(p.is_finite() && v.is_finite(), "流体非有限：{p:?} {v:?}");
        max_v = max_v.max(v.length());
        max_r = max_r.max(p.x.abs().max(p.z.abs()));
        if p.x.abs() > 0.3 || p.z.abs() > 0.3 {
            outside += 1;
        }
    }
    let nb = f.boundary_count();
    println!(
        "2b 稳定性：粒子 {}、边界粒子 {nb}（{:.2}×）、vmax {max_v:.2} m/s、max|xz| {max_r:.3}、盆外 {outside}",
        f.len(),
        nb as f32 / f.len() as f32
    );
    assert!(max_v < 6.0, "速度失控：vmax = {max_v:.2}");
    assert!(
        outside <= f.len() / 10,
        "盆外粒子过多（应 ≤10%）：{outside}/{}",
        f.len()
    );
    assert!(nb > 0, "2b 应生成边界粒子");
    assert!(
        nb <= 8 * f.len(),
        "边界粒子数应 ≤ 8× 粒子数：{nb} vs {}",
        f.len()
    );
    assert!(b < u32::MAX);
}

/// ⑤ **确定性**（2b 开）：同场景两跑末态逐位一致（体 + 流体全部 `to_bits`）。
#[test]
fn boundary_coupling_is_deterministic_end_to_end() {
    let digest = || {
        let (mut w, _v) = scene(true);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, 1.10, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        for _ in 0..60 {
            w.step();
        }
        let f = &w.fluids()[0].0;
        let mut d: Vec<u32> = Vec::new();
        for i in 0..w.bodies.len() {
            let p = w.bodies.position[i];
            d.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        }
        for p in f.positions() {
            d.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        }
        for v in f.velocities() {
            d.extend([v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
        }
        d.push(b);
        d
    };
    assert_eq!(digest(), digest(), "两跑应逐位一致");
}

/// **2b 单独给出浮力**（端到端）：比水轻的盒（300 kg/m³）在水下**上浮**。
/// 这条与 `fluid_coupling`（2a）对应——那里浮力来自介质场，这里**只**来自
/// Akinci 反作用（2b 覆盖的体 2a 已让位），所以它是"双向耦合能产生浮力"的证据。
#[test]
fn light_box_floats_on_akinci_reaction_alone() {
    let (mut w, _v) = scene(true);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.10, 0.0), // 深潜：必须靠浮力升上来
        Quat::IDENTITY,
        300.0,
    );
    let y0 = w.bodies.position[b as usize].y;
    let mut peak = y0;
    for _ in 0..180 {
        w.step();
        peak = peak.max(w.bodies.position[b as usize].y);
    }
    let y1 = w.bodies.position[b as usize].y;
    println!("2b 全潜轻盒：y {y0:.3} → 峰值 {peak:.3}（末态 {y1:.3}）");
    assert!(
        peak > y0 + 0.02,
        "轻盒应被浮力推上来：y0 = {y0:.3}、峰值 = {peak:.3}"
    );
    assert!(y1.is_finite(), "末态非有限");
}
