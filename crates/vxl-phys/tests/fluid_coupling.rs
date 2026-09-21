//! **流体 ↔ 刚体耦合（2a）** 的集成门：`ROUTE.md` §4「刚体↔液体」格。
//!
//! 判据（物理量纲齐备，不看旋钮）：
//! - 比水轻的盒**上浮**、比水重的盒**下沉**；
//! - **对照**：同密度但落在"干区"（介质包围盒外）的盒**照常受重力下落**
//!   ⇒ 变化来自介质耦合，不是重力被改；
//! - **确定性**：同场景两次运行末态哈希逐位一致（耦合是状态函数，不是随机项）。
//!
//! ⚠️ 场景必须按**铸装口径**搭（`PLAN-0.3.md` §4.2 的负面结论）：水块要按**沉降后几何**
//! 直接就位——`[8,8,5]@0.05` 对 ~0.5 m 盆腔（与 `showcase` 石盆/门禁同款）。任何"带落差
//! 入盆"的水块都会触发 WCSPH 驻留瞬态：压实波在块顶心聚焦、粒子以 ~9 m/s 喷出盆地
//! （实测：`[14,10,14]@0.05` 摊在 1.5 m 槽里 ⇒ t=20 时水面被抛到 y=2.54，随后抛出的
//! 粒子越出流体 AABB 预滤 ⇒ 自由落体到 y=−3.53）。**那不是边界失效，也不是本耦合的问题**，
//! 但会让任何"水里有盒子"的断言失效 ⇒ 本门用铸装口径。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 水槽：5×5 格（外沿 2.5 m）地板（y∈[0,1]）+ **中心一格**围堰 ⇒ 内腔 0.5×0.5 m
/// （与 `showcase` 石盆/门禁同款量级）；外圈地板留给"干区对照盒"。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    // 围堰：只留中心格 (2,2) 敞开 ⇒ 腔体 x,z ∈ [-0.25, 0.25]。
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

/// 铸装水块：`[8,8,8]@0.05`（足印 0.4、深 0.4）在 0.5 m 盆腔里近平衡就位。
/// 深 0.4 是为了让"浮"有位可浮（摊到 0.5×0.5 后仍有 ~0.26 m 水深 ⇒ 轻盒能从深潜位升起）。
fn water(w: &mut World, voxel_id: u32) {
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    let _ = w.add_fluid(sys, &[voxel_id]);
}

/// 轻盒**从深潜位上浮**到水面；干区同密度盒照常下落（对照 ⇒ 变化来自介质）。
#[test]
fn buoyancy_raises_light_box() {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    water(&mut w, v);
    let half = Vec3::splat(0.06);
    // 水中：起点在水面以下 ~0.15 m（深潜 ⇒ 必须靠浮力升上来）。
    let light = w.add_dynamic(
        Shape::Box { half },
        Vec3::new(0.0, 1.10, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    // 干区对照：同密度、**槽外无几何处** ⇒ 只受重力（自由落体）。
    // 注：不能放在"槽内地板上"——围堰盖满中心格以外的所有格，那位置其实是墙体内部。
    let dry = w.add_dynamic(
        Shape::Box { half },
        Vec3::new(-3.0, 1.40, -3.0),
        Quat::IDENTITY,
        300.0,
    );
    for _ in 0..180 {
        w.step();
    }
    let y_light = w.bodies.position[light as usize].y;
    let y_dry = w.bodies.position[dry as usize].y;
    assert!(
        y_light > 1.20,
        "轻盒应从深潜位上浮：y {y_light:.3}（起点 1.100）"
    );
    assert!(
        y_dry < 1.00,
        "干区同密度盒应只受重力继续下落：y {y_dry:.3}（起点 1.400）"
    );
    let h = w.health();
    assert_eq!(h.nan_bodies, 0, "出现 NaN");
    assert_eq!(h.deep_penetrations, 0, "出现深穿透");
}

/// 重盒下沉到盆底。
#[test]
fn dense_box_sinks() {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    water(&mut w, v);
    let heavy = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.20, 0.0),
        Quat::IDENTITY,
        2000.0,
    );
    for _ in 0..180 {
        w.step();
    }
    let y = w.bodies.position[heavy as usize].y;
    assert!(
        y < 1.15,
        "重盒应下沉到盆底（地板顶 y=1.0）：y {y:.3}（起点 1.200）"
    );
    let h = w.health();
    assert_eq!(h.nan_bodies, 0, "出现 NaN");
    assert_eq!(h.deep_penetrations, 0, "出现深穿透");
}

/// 耦合是**状态函数**：同场景两次运行，末态哈希逐位一致。
#[test]
fn medium_coupling_is_deterministic() {
    let run = || {
        let mut w = World::new(PhysConfig::default());
        let v = tank(&mut w);
        water(&mut w, v);
        let _ = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            Vec3::new(0.0, 1.20, 0.0),
            Quat::IDENTITY,
            300.0,
        );
        for _ in 0..90 {
            w.step();
        }
        w.state_hash()
    };
    let (a, b) = (run(), run());
    assert_eq!(a, b, "两次运行末态哈希应逐位一致");
}
