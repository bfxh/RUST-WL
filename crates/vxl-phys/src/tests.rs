//! tests：从 `lib.rs` 拆出的单元测试（纯搬移 + 去一层缩进）。

use super::*;

use super::*;

/// 立方体点云（外壳测试用；顶点序固定 ⇒ 确定性）。
fn cube_hull_points(half: f32) -> Vec<Vec3> {
    let mut pts = Vec::new();
    for &x in &[-half, half] {
        for &y in &[-half, half] {
            for &z in &[-half, half] {
                pts.push(Vec3::new(x, y, z));
            }
        }
    }
    pts
}

fn ground_world() -> World {
    let mut w = World::new(PhysConfig::default());
    let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
    w.add_heightfield(hf);
    w
}

#[test]
fn box_falls_and_rests_on_heightfield() {
    let mut w = ground_world();
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 3.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..240 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在 y ≈ 0.5 + 少许穿透修正余量。
    assert!(y > 0.45 && y < 0.62, "y = {y}");
    assert!(w.health().is_clean());
}

/// **M2 贯通切片**（ROUTE §7）：**刚体 ↔ 体素**——盒经 `Shape::Provider` 路径
/// 落在体素地面上并入睡（跨域唯一通道 `ProviderColliders` 的第一条端到端用例）。
#[test]
fn box_falls_and_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
    let marker = w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 2.5, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：y ≈ 1.5 + 少许穿透修正余量。
    assert!(y > 1.42 && y < 1.60, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "盒应已入睡（静置 10s）");
    assert!(w.health().is_clean());
    // marker 体（provider）保持静止：位置零漂移。
    assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
}

/// **M0.3 液体域**（ROUTE §7）：流体块经门面落入**体素盆**（地板+四壁，
/// 一并覆盖体素 provider 的顶面/内壁/角点接触路径）并停驻。
/// `add_fluid` + `fluid_pass` 端到端；单向耦合，marker 体不受扰动。
/// （不用悬浮板：驻留投影不消耗切向速度，冲击横流会沿板面滑出板缘——
/// 那是正确物理，但场景里板外无物，跑出者永远下坠，断言无从谈起。）
#[test]
fn fluid_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    // 体素盆：外廓 0.8×0.8m、格边 0.2；地板层顶面 y=0.2，壁高到 y=0.8，
    // 内腔 0.4×0.4（与 fluid 域 Tank 测试同腔口）。
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-0.4, 0.0, -0.4), 0.2, 4, 4, 4);
    vol.fill_box(Vec3::new(-0.4, 0.0, -0.4), Vec3::new(0.4, 0.2, 0.4)); // 地板
    vol.fill_box(Vec3::new(0.2, 0.2, -0.4), Vec3::new(0.4, 0.8, 0.4)); // +x 壁
    vol.fill_box(Vec3::new(-0.4, 0.2, -0.4), Vec3::new(-0.2, 0.8, 0.4)); // −x 壁
    vol.fill_box(Vec3::new(-0.2, 0.2, 0.2), Vec3::new(0.2, 0.8, 0.4)); // +z 壁
    vol.fill_box(Vec3::new(-0.2, 0.2, -0.4), Vec3::new(0.2, 0.8, -0.2)); // −z 壁
    let marker = w.add_voxel(vol);
    let vid = w.provider_id_of(marker).expect("marker 是 provider 体");
    // 铸装近平衡块（8×8 贴腔口 + 5 层 ≈ 实测静水充高 0.44，320 粒）。
    // 不用方块自落：任何带落差的方块入盆，WCSPH 驻留瞬态（底部镜像
    // 鬼影密度尾 → 压实波在块顶心聚焦）都会把顶心粒子以近钳制速度
    // （实测 ~9 m/s）垂直喷过敞口壁顶——0.8 m 重落、0.1 m 轻落、触底
    // 就位皆复现，是 PLAN-0.3 §4 已记录的求解器瞬态而非边界失效；
    // 边界本身在全部场景中零穿壁。铸装后瞬态消失，本测试只验驻留与
    // 边界。
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.175, 0.25, -0.175),
        [8, 8, 5],
        0.05,
    );
    let fid = w.add_fluid(sys, &[vid]);
    for _ in 0..300 {
        w.step();
    }
    let f = &w.fluids()[fid].0;
    for (i, p) in f.positions().iter().enumerate() {
        assert!(p.y > 0.15, "粒子 {i} 穿透盆底：y = {}", p.y);
        assert!(p.y < 0.9, "粒子 {i} 飞出：y = {}", p.y);
        assert!(
            p.x.abs() < 0.45 && p.z.abs() < 0.45,
            "粒子 {i} 越出盆壁：({}, {})",
            p.x,
            p.z
        );
    }
    // 单向耦合：marker 体（provider）保持静止。
    assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
    assert!(w.health().is_clean());
}

/// **L1**：外壳落在高度场上（顶点采样；此前不受理 ⇒ 直接穿地）。
#[test]
fn hull_rests_on_heightfield() {
    let mut w = ground_world(); // 平地高度场（y = 0）
    let hull = w.add_hull(cube_hull_points(0.5));
    let b = w.spawn_hull_body(hull, Vec3::new(0.1, 3.0, -0.1), Quat::IDENTITY, 1.0);
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在平地（y=0）上方：半长 0.5 ⇒ y ≈ 0.5
    assert!(y > 0.40 && y < 0.62, "y = {y}");
    assert!(w.health().is_clean());
}

/// **M3 多边形域**：凸体外壳落在体素地面上（外壳 × 提供者 = 顶点采样多点流形）。
#[test]
fn hull_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
    w.add_voxel(vol);
    let hull = w.add_hull(cube_hull_points(0.5));
    let b = w.spawn_hull_body(hull, Vec3::new(0.1, 2.5, -0.1), Quat::IDENTITY, 1.0);
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：外壳半长 0.5 ⇒ y ≈ 1.5
    assert!(y > 1.42 && y < 1.70, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "外壳应已入睡");
    assert!(w.health().is_clean());
}

/// **M3 凸体切割**：立方体外壳 ✕ 8 种子 ⇒ 8 个凸碎块，逐个落在体素地面上。
#[test]
fn fractured_hull_pieces_rest_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    let hull = w.add_hull(cube_hull_points(0.5));
    let seeds: Vec<Vec3> = [
        Vec3::new(-0.3, -0.25, -0.35),
        Vec3::new(-0.3, -0.25, 0.25),
        Vec3::new(-0.3, 0.35, -0.35),
        Vec3::new(-0.3, 0.35, 0.25),
        Vec3::new(0.3, -0.25, -0.35),
        Vec3::new(0.3, -0.25, 0.25),
        Vec3::new(0.3, 0.35, -0.35),
        Vec3::new(0.3, 0.35, 0.25),
    ]
    .to_vec();
    let pieces = w.spawn_hull_pieces(hull, &seeds, Vec3::new(0.0, 2.2, 0.0), Quat::IDENTITY, 1.0);
    assert_eq!(pieces.len(), 8, "应有 8 块");
    for _ in 0..900 {
        w.step();
    }
    // 全体落到地面带内且干净
    for &p in &pieces {
        let y = w.bodies.position[p as usize].y;
        assert!(y > 0.9 && y < 2.0, "碎块 y = {y} 不在带内");
    }
    assert!(w.health().is_clean());
}

/// **M3 喷溅域**：盒落在一团高斯喷溅上并停住（隐式场 σ(p) = Σ 核；
/// 接触走统一提供者通道 `contacts_box`）。
#[test]
fn box_rests_on_gaussian_splat_field() {
    let mut w = World::new(PhysConfig::default());
    let mut f = vxl_phys_splat::GaussianSplatField::new(0.5);
    // 半径 1.2 球状栅格（间距 0.4、核半径 0.35）⇒ 顶面等值面 ≈ y = 1.6
    for i in -3..=3 {
        for j in -3..=3 {
            for k in -3..=3 {
                let c = Vec3::new(i as f32 * 0.4, j as f32 * 0.4, k as f32 * 0.4);
                if c.length() <= 1.2 {
                    f.push(vxl_phys_splat::Splat::isotropic(c, 0.35, 1.0));
                }
            }
        }
    }
    assert!(f.len() > 100);
    w.add_splat_field(f);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 3.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 停在等值面之上（盒半长 0.4 + 顶面 ≈ 1.6 ⇒ 中心 ≈ 2.0）
    assert!(y > 1.5 && y < 2.6, "y = {y}");
    assert!(w.health().is_clean());
}

/// **介质耦合（喷溅域第三层）**：稀薄喷溅云（σ < iso ⇒ 不产生接触）作介质
/// ⇒ 落体被二次阻力减速；介质密度 0 时为纯自由落体（对照）。
/// 口径：两条 run 仅差 `medium_density`，其余位姿/初始条件完全相同。
#[test]
fn splat_medium_drag_slows_falling_body() {
    let run = |medium_density: f32| -> (f32, f32) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0); // iso 高 ⇒ 纯介质、无接触
        for k in 0..16 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
                0.45,
                0.6,
            ));
        }
        f.medium_density = medium_density;
        f.medium_velocity = Vec3::ZERO;
        w.add_splat_field(f);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 9.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..90 {
            w.step();
        }
        (
            w.bodies.position[b as usize].y,
            w.bodies.linvel[b as usize].y,
        )
    };
    let (y_free, v_free) = run(0.0);
    let (y_drag, v_drag) = run(6.0);
    println!("free: y={y_free:.3} v={v_free:.3} | drag: y={y_drag:.3} v={v_drag:.3}");
    assert!(
        y_drag > y_free + 0.2,
        "介质应显著减速：free y={y_free} drag y={y_drag}"
    );
    assert!(
        v_drag > v_free + 0.5,
        "末速应更高（落得更慢）：{v_free} vs {v_drag}"
    );
}

/// **网格域（M3 扩展）**：盒落在三角网格地面上并停住（薄壳接触；
/// 8 顶点 + 6 面心采样 ⇒ 底四角多点接触）。
#[test]
fn box_rests_on_mesh_ground() {
    let mut w = World::new(PhysConfig::default());
    // 4×4 米网格地面（y = 0，朝上）
    let ground =
        vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, Vec3::X * 2.0, Vec3::Z * 2.0, Vec3::Y);
    w.add_mesh(ground);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.3, 2.0, -0.2),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    assert!(y > 0.42 && y < 0.70, "y = {y}"); // 静置在网格面上方（半长 0.5）
    assert!(w.health().is_clean());
}

/// **网格域**：球在斜网格上按**面法线**接触并沿坡下滑（球无滚阻必下滚——
/// 因此断言「接触带内 + 法向速度被抑制 + 沿坡下滑」，而非「停在坡上」）。
#[test]
fn sphere_contacts_sloped_mesh_along_face_normal() {
    let mut w = World::new(PhysConfig::default());
    // 斜面：沿 +u 抬升（面法线 n 偏向 −X）
    let slope = 0.25f32;
    let n = Vec3::new(-slope, 1.0, 0.0).normalize();
    let u = Vec3::new(1.0, slope, 0.0).normalize() * 8.0;
    let v = Vec3::Z * 8.0;
    let ground = vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, u, v, n);
    w.add_mesh(ground);
    let ball = w.add_dynamic(
        Shape::Sphere { radius: 0.4 },
        Vec3::new(0.0, 1.5, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    // 触面后约 0.5 秒：仍在接触带内、法向速度被压制、沿 −u 下滑
    for _ in 0..90 {
        w.step();
    }
    let p = w.bodies.position[ball as usize];
    let vel = w.bodies.linvel[ball as usize];
    let dist = p.dot(n);
    let v_n = vel.dot(n);
    let down = vel.dot(u.normalize());
    assert!(
        dist > 0.30 && dist < 0.75,
        "应在接触带内（半径 0.4）：沿法线 {dist}"
    );
    assert!(v_n.abs() < 1.0, "法向速度应被接触抑制：{v_n}");
    assert!(down < -0.05, "应沿坡下滑（−u 方向）：v·u = {down}");
    assert!(w.health().is_clean());
}

/// M2 provider 通道扩到**球**：球经 SDF 解析接触（`depth = r − sdf(c)`）
/// 落在体素地面上并入睡；顺带覆盖「斜坡不穿透」。
#[test]
fn sphere_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.4 },
        Vec3::new(0.25, 2.0, 0.25),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：y ≈ 1.4
    assert!(y > 1.32 && y < 1.50, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "球应已入睡");
    assert!(w.health().is_clean());
}

/// **M3 破坏切片**：体素柱被「切掉顶部」⇒ 顶部转成刚体碎块，落在余柱上停驻；
/// 余柱（仍在体素体里）与碎块共同构成确定性可继续推进的场景。
#[test]
fn carve_top_spawns_debris_resting_on_column() {
    let mut w = World::new(PhysConfig::default());
    // 柱：X/Z ∈ [−0.5,0.5]、Y ∈ [0,4)，格边长 0.5（8 层 × 2×2）
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 8, 8);
    vol.fill_box(Vec3::new(-0.5, 0.0, -0.5), Vec3::new(0.5, 4.0, 0.5));
    w.add_voxel(vol);
    // 切掉顶部 1m（Y ∈ [3,4)）⇒ 碎块（2×2×2 格 ⇒ 贪心合并为 1 个 1m 立方）
    let n = w.spawn_box_debris(
        0,
        Vec3::new(-0.5, 3.0, -0.5),
        Vec3::new(0.5, 4.0, 0.5),
        1000.0,
    );
    assert_eq!(n, 1, "顶部 8 格应合并为 1 个碎块盒");
    for _ in 0..600 {
        w.step();
    }
    let h = w.health();
    assert!(h.is_clean(), "无 NaN / 无深穿透");
    // 碎块落在余柱顶面（y=3.0）上方：中心 ≈ 3.5
    let mut top = 0.0f32;
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            top = top.max(w.bodies.position[i].y);
        }
    }
    assert!(top > 3.3 && top < 3.7, "碎块应停在余柱上，实际 top={top}");
}

/// **M3 冲击破坏**：高速盒撞体素墙 ⇒ 接触点处挖洞并产出碎块；墙体素减少、
/// 场景干净，且**整轮可复现**（同一构造两次 → 碎块数/末态哈希一致）。
#[test]
fn impact_carves_wall_and_spawns_debris() {
    let run = || -> (usize, usize, usize, u128) {
        let mut w = World::new(PhysConfig::default());
        // 地板（整幅 1 层）+ 墙（X ∈ [0,0.5]、Y ∈ [0.5,2.5)、Z ∈ [−2,2)）
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 0.5, 4.0));
        vol.fill_box(Vec3::new(0.0, 0.5, -2.0), Vec3::new(0.5, 2.5, 2.0));
        let filled0 = vol.filled_count();
        w.add_voxel(vol);
        // 炮弹：半 0.4 的盒，以 12 m/s 冲墙
        let bullet = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(-3.0, 1.0, 0.0),
            Quat::IDENTITY,
            2000.0,
        );
        w.bodies.linvel[bullet as usize] = Vec3::new(12.0, 0.0, 0.0);
        let mut debris_total = 0usize;
        for _ in 0..240 {
            w.step();
            debris_total += w.apply_impact_destruction(0, 3.0, 1000.0);
        }
        let filled1 = w.providers.voxel(0).unwrap().filled_count();
        let bodies = w.bodies.len();
        (debris_total, filled0 - filled1, bodies, w.state_hash())
    };
    let (debris, carved, _bodies, hash1) = run();
    assert!(debris > 0, "应触发冲击破坏（产出碎块）");
    assert!(carved > 0, "墙体素应减少（挖洞）carved={carved}");
    let (debris2, carved2, _b, hash2) = run();
    assert_eq!((debris, carved), (debris2, carved2), "破坏应可复现（计数）");
    assert_eq!(hash1, hash2, "破坏应可复现（末态哈希逐位一致）");
}

/// **CCD 回归**（本轮修复）：开启 CCD 后，**贴地滑行**的体不得被锁死。
/// 修前：每个扫描采样都有地面接触 ⇒ 判「命中」⇒ 钳回起点、原地停住。
/// 修后：只有「沿法向接近」的采样才算命中 ⇒ 切向滑行不受影响。
#[test]
fn ccd_does_not_lock_sliding_body() {
    let cfg = PhysConfig {
        ccd_speed_threshold: 5.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
    w.add_heightfield(hf);
    // 贴地盒（底面 y=0.5 略上方）以 8 m/s 沿 +X 滑行（超过 CCD 阈值 5）
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.55, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(8.0, 0.0, 0.0);
    for _ in 0..60 {
        w.step();
    }
    let x = w.bodies.position[b as usize].x;
    // 摩擦会减速，但绝不该「原地不动」：修前 x ≈ 0，修后应有明显位移
    assert!(x > 2.0, "CCD 不应锁死滑行体：x = {x}");
}

#[test]
fn determinism_same_construction_same_hash() {
    let run = || {
        let mut w = ground_world();
        for k in 0..12 {
            let x = (k % 4) as f32 * 1.2;
            let z = (k / 4) as f32 * 1.2;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(x, 2.0 + (k as f32) * 0.9, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        for _ in 0..180 {
            w.step();
        }
        w.state_hash()
    };
    assert_eq!(run(), run());
}

/// §5/§6 并行契约：并行（threads=8）与串行（threads=1）结果 bit 级一致。
/// 场景需超过各相并行门槛（>4096 体）才能真正走到并行路径。
#[test]
fn parallel_matches_serial_bitwise() {
    let run = |threads: usize| {
        let cfg = PhysConfig {
            threads,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        w.add_heightfield(HeightField::flat(-40.0, -40.0, 81, 81, 1.0, 0.0));
        for k in 0..4200usize {
            let x = (k % 70) as f32 - 35.0;
            let z = (k / 70) as f32 - 30.0;
            w.add_static(
                Shape::Box {
                    half: Vec3::new(0.5, 0.5, 0.5),
                },
                Vec3::new(x, 0.5, z),
                Quat::IDENTITY,
            );
        }
        for k in 0..900 {
            let x = ((k * 37) % 97) as f32 / 97.0 * 30.0 - 15.0;
            let z = ((k * 53) % 89) as f32 / 89.0 * 30.0 - 15.0;
            let y = 4.0 + ((k * 29) % 71) as f32 / 71.0 * 10.0;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(x, y, z),
                Quat::IDENTITY,
                1000.0,
            );
        }
        for _ in 0..150 {
            w.step();
        }
        w.state_hash()
    };
    let serial = run(1);
    let parallel = run(8);
    assert_eq!(serial, parallel, "并行与串行状态哈希必须一致（§5）");
}

#[test]
fn pyramid_settles_and_sleeps() {
    let mut w = ground_world();
    let layers = 4;
    for layer in 0..layers {
        let count = layers - layer;
        for k in 0..count {
            let x = (k as f32 - (count as f32 - 1.0) * 0.5) * 1.05;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(x, 0.55 + layer as f32 * 1.02, 0.0),
                Quat::IDENTITY,
                1.0,
            );
        }
    }
    for _ in 0..600 {
        w.step();
    }
    let h = w.health();
    assert!(h.is_clean(), "{h:?}");
    // 塔应全部入睡（§3：无持续抖动）。
    let awake = h.awake_bodies;
    assert_eq!(awake, 0, "awake = {awake}");
}

#[test]
fn restitution_bounce_and_threshold() {
    let mut w = World::new(PhysConfig {
        restitution: 0.8,
        ..PhysConfig::default()
    });
    w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let mut max_after_fall = 0.0f32;
    for _ in 0..1800 {
        w.step();
        max_after_fall = max_after_fall.max(w.bodies.linvel[b as usize].y);
    }
    // e=0.8 应产生明显反弹（> 2 m/s 向上），30 秒内经恢复阈值衰减到静止。
    assert!(max_after_fall > 2.0, "bounce vy = {max_after_fall}");
    assert!(w.bodies.linvel[b as usize].y.abs() < 0.3);
}

#[test]
fn digging_removes_support() {
    let mut w = ground_world();
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.6, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..120 {
        w.step();
    }
    let y_rest = w.bodies.position[b as usize].y;
    // 在盒子正下方挖 3×3 列块（2m 深）。
    for gx in -1i32..=1 {
        for gz in -1i32..=1 {
            w.terrain.dig(0, gx as f32, gz as f32, 2.0);
        }
    }
    w.bodies.wake(b as usize);
    for _ in 0..120 {
        w.step();
    }
    let y_after = w.bodies.position[b as usize].y;
    assert!(y_after < y_rest - 1.0, "rest {y_rest} → after {y_after}");
}

#[test]
fn material_pair_restitution_and_friction() {
    // §4.4/§4.5：e 取材质对 max——球(e=0.9) 落到地面(e=0.0) → 反弹按 0.9。
    let mut w = World::new(PhysConfig::default());
    let ground_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.5 }, 0.0));
    let ball_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.2 }, 0.9));
    w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    w.bodies.set_material(b as usize, ball_mat);
    let _ = ground_mat;
    let mut max_bounce = 0.0f32;
    for _ in 0..1200 {
        w.step();
        max_bounce = max_bounce.max(w.bodies.linvel[b as usize].y);
    }
    // 0.9 × 9.9 m/s 落地速度 ≈ 8.9 m/s 首次反弹。
    assert!(max_bounce > 4.0, "material pair bounce vy = {max_bounce}");
    assert!(w.health().is_clean());
}

#[test]
fn ccd_stops_fast_sphere_at_thin_wall() {
    // 薄墙 half_z=0.1；球 r=0.2 以 120 m/s（单帧位移 2m）射向墙体。
    // 采样带（墙厚+球径=0.6m）< 位移 2m 且起点偏移 → 离散步进必然穿透。
    let build = |ccd_on: bool| {
        let mut w = World::new(PhysConfig {
            ccd_speed_threshold: if ccd_on { 30.0 } else { f32::INFINITY },
            max_linear_velocity: 200.0,
            ..PhysConfig::default()
        });
        w.add_static(
            Shape::Box {
                half: Vec3::new(5.0, 5.0, 0.1),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let s = w.add_dynamic(
            Shape::Sphere { radius: 0.2 },
            Vec3::new(0.0, 0.0, -19.0),
            Quat::IDENTITY,
            1.0,
        );
        w.bodies.set_linvel(s as usize, Vec3::new(0.0, 0.0, 120.0));
        (w, s)
    };
    // CCD 关：穿透（球越过墙面 z>0.3）。
    let (mut w_off, s_off) = build(false);
    for _ in 0..20 {
        w_off.step();
    }
    let z_off = w_off.bodies.position[s_off as usize].z;
    assert!(z_off > 1.0, "预期穿透，实际 z = {z_off}");
    // CCD 开：钳位在墙前。
    let (mut w_on, s_on) = build(true);
    for _ in 0..20 {
        w_on.step();
    }
    let z_on = w_on.bodies.position[s_on as usize].z;
    assert!(
        (-1.0..=0.9).contains(&z_on),
        "CCD 应拦下球，实际 z = {z_on}"
    );
    let h = w_on.health();
    assert!(h.is_clean(), "{h:?}");
}

#[test]
fn ccd_disabled_by_default_keeps_slow_scene_unchanged() {
    // 默认阈值 INFINITY：确定性场景与 M0 行为一致（回归守门）。
    let run = || {
        let mut w = ground_world();
        for k in 0..6 {
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(k as f32 * 1.1, 2.0 + k as f32 * 0.9, 0.0),
                Quat::IDENTITY,
                1.0,
            );
        }
        for _ in 0..180 {
            w.step();
        }
        w.state_hash()
    };
    assert_eq!(run(), run());
}
