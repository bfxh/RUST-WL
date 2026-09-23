//! 稀疏体素体（可破坏地形的**物理表示**；ROUTE §3 体素域的第一步）。
//!
//! 表示：均匀网格 + 占据位图（1 bit/格）+ 占据包围盒增量维护。查询：
//! **局域 SDF**（±1 格邻域到最近占据格盒子的带符号距离）+ 有限差分法线
//! ⇒ 实现 `vxl_phys_core::interop::CollisionProvider`，即可作为**碰撞提供者**
//! 被刚体接触消费（跨域唯一通道，见 ROUTE §2.1/§5）。
//!
//! 确定性：查询只读位图与常数；无哈希迭代、无浮点归约顺序问题（逐格循环固定序）。
//! 性能：局域扫描是 O(27)/查询——первый版够用；上量时应换距离场或 BVH
//! （与「体素→SDF→Provider」的专用解法一并做，见 ROUTE §3 体素行）。

use vxl_phys_core::interop::{CollisionProvider, SurfaceHit};
use vxl_phys_core::{Aabb, Quat, Vec3};

// ── 按域拆出的子模块（子目录 voxel/）
mod voxel_contacts;
mod voxel_provider;
mod voxel_volume;
pub use self::{voxel_contacts::*, voxel_volume::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn floor_volume() -> VoxelVolume {
        // 8×2×8 格、边长 0.5：填充 y ∈ [0,1) 两层 ⇒ 顶面 y = 1.0
        let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
        v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
        v
    }

    #[test]
    pub(crate) fn sdf_signs_and_surface() {
        let v = floor_volume();
        // 地面上方 0.25 ⇒ +0.25
        let d = v.sdf(Vec3::new(0.0, 1.25, 0.0));
        assert!((d - 0.25).abs() < 1e-5, "d={d}");
        // 地面内一格的**中心**（0.25,0.75,0.25 ⇒ 格 y=[0.5,1.0] 的中心）⇒ 负；
        // 到最近面 = 半格 0.25（注意别取在格边界上：那里距离恰为 0）
        let d = v.sdf(Vec3::new(0.25, 0.75, 0.25));
        assert!(d < 0.0, "d={d}");
        assert!((d + 0.25).abs() < 1e-5, "d={d}");
        // 表面最近点与法线
        let hit = v.closest_point(Vec3::new(0.25, 1.25, 0.25)).unwrap();
        assert!((hit.point.y - 1.0).abs() < 1e-5, "py={}", hit.point.y);
        assert!((hit.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-4);
        assert!((hit.signed_dist - 0.25).abs() < 1e-5);
    }

    #[test]
    pub(crate) fn dug_hole_reads_as_empty_with_walls() {
        let mut v = floor_volume();
        // 挖掉 (0,0,0) 与 (0,1,0) 两格（x∈[0,0.5), y∈[0,1), z∈[0,0.5)）
        v.set(4, 0, 4, false);
        v.set(4, 1, 4, false);
        assert_eq!(v.filled_count(), 8 * 2 * 8 - 2);
        // 洞里（y=0.5, x=z=0.25）现在是空：SDF 应为正（到洞壁/洞底的最近距离）
        let d = v.sdf(Vec3::new(0.25, 0.5, 0.25));
        assert!(d > 0.0, "d={d}");
        // 洞底 y=0.25 处在洞里；正下方无占据格（挖穿到底）⇒ 仍是空
        assert!(v.sdf(Vec3::new(0.25, 0.1, 0.25)) > 0.0);
    }

    #[test]
    pub(crate) fn box_contacts_on_voxel_floor() {
        let v = floor_volume();
        // 盒（半 0.5）落在地面 y=1.0 上、穿透 0.1 ⇒ 底面 4 角 depth ≈ 0.1
        let mut out = Vec::new();
        let any = contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 1.4, 0.25),
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any);
        // **逐面发射**（2026-09-15 起）：每张「带内样本数 > 0」的面各自发点；
        // 共享角点会在相邻面里重复出现，因此断言改为：
        //  ① 贴地面的 5 点（面号 3 ⇒ feature 48..52）必须在，且深度 ≈0.1、法线 +Y；
        //  ② 其它面只允许出现**角点**（feature % 16 != 0）——面心样本只有真贴着
        //     的底面才有（窄相据此在多面候选里排除"只有角点的伪面"）。
        let bottom: Vec<_> = out
            .iter()
            .filter(|c| (48..53).contains(&c.feature))
            .collect();
        assert_eq!(bottom.len(), 5, "贴地面应有 5 点；out={}", out.len());
        for c in &bottom {
            assert!((c.depth - 0.1).abs() < 1e-4, "depth={}", c.depth);
            assert!((c.normal.y - 1.0).abs() < 1e-4, "normal={:?}", c.normal);
        }
        for c in out.iter().filter(|c| !(48..53).contains(&c.feature)) {
            assert_ne!(
                c.feature % 16,
                0,
                "非贴地面不得有面心样本：feature={}",
                c.feature
            );
        }
        // 面中心点（feature 48）也应在（对齐落面时中心才给得出正确法线语义）
        assert!(out.iter().any(|c| c.feature == 48));
        // 提升到 y=2.5（远离表面）⇒ 无接触
        let mut out2 = Vec::new();
        assert!(!contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 2.5, 0.25),
            Quat::IDENTITY,
            0.02,
            &mut out2
        ));
    }

    #[test]
    pub(crate) fn box_fully_embedded_reports_contact() {
        // 回归：盒**完全嵌入**体积内部时也必须报接触——否则碎块会「自由落体
        // 穿地」（实测逃逸机制：体素挖出的碎块嵌在残余结构里却拿不到接触）。
        let v = floor_volume(); // 顶面 y=1.0
        let mut out = Vec::new();
        let any = contacts_box_voxel(
            &v,
            Vec3::splat(0.5),
            Vec3::new(0.25, 0.75, 0.25), // 底 0.25、顶 1.25 ⇒ 完全在体内
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any, "完全嵌入的盒必须报接触（out={}）", out.len());
        assert!(!out.is_empty());
        // 深度应为正（穿透）
        assert!(out[0].depth > 0.0, "depth={}", out[0].depth);
    }

    #[test]
    pub(crate) fn sphere_sdf_contact_depth_and_normal() {
        let v = floor_volume();
        // 球心在 y=1.3（地面顶 1.0）、半径 0.4 ⇒ 穿透 0.1
        let mut out = Vec::new();
        assert!(contacts_sphere_voxel(
            &v,
            Vec3::new(0.25, 1.3, 0.25),
            0.4,
            0.02,
            &mut out
        ));
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.1).abs() < 1e-4, "depth={}", out[0].depth);
        assert!((out[0].normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-4);
        // 球心在 y=1.5、半径 0.4 ⇒ 缝 0.1 > skin ⇒ 无接触
        let mut out2 = Vec::new();
        assert!(!contacts_sphere_voxel(
            &v,
            Vec3::new(0.25, 1.5, 0.25),
            0.4,
            0.02,
            &mut out2
        ));
    }

    #[test]
    pub(crate) fn sphere_extract_carves_crater() {
        // 8×8×8 实体块（格边长 0.5、origin 0）：球域挖洞 ⇒ 洞内变空、盒数>0、
        // 移除格数 ≈ 球体积/格体积（格心判据 ⇒ 数量级一致即可）
        let mut v = VoxelVolume::new(Vec3::ZERO, 0.5, 8, 8, 8);
        v.fill_box(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0));
        let filled0 = v.filled_count();
        let boxes = v.extract_sphere(Vec3::new(2.0, 2.0, 2.0), 1.0);
        let removed = filled0 - v.filled_count();
        assert!(!boxes.is_empty(), "球域应提取出碎块盒");
        let vol_cells = (4.0 / 3.0 * std::f32::consts::PI * 1.0f32.powi(3)) / 0.5f32.powi(3);
        assert!(
            (removed as f32) > vol_cells * 0.6 && (removed as f32) < vol_cells * 1.4,
            "移除格数 {removed} 应接近球体积格数 {vol_cells:.1}"
        );
        // 球心处已空
        assert!(!v.get(4, 4, 4), "球心格应被挖掉");
        // 球外的角点仍在
        assert!(v.get(0, 0, 0));
        assert!(v.get(7, 7, 7));
    }

    #[test]
    pub(crate) fn voronoi_fracture_tiles_region_exactly() {
        // 守恒：Voronoi 分区是对域内占据格的**划分** ⇒ 提取总格数 = 原占据格数
        let mut v = VoxelVolume::new(Vec3::ZERO, 0.5, 8, 8, 8);
        v.fill_box(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0));
        let filled0 = v.filled_count();
        let seeds =
            VoxelVolume::seeds_jittered(Vec3::new(0.5, 0.5, 0.5), Vec3::new(3.5, 3.5, 3.5), 8, 0.6);
        assert_eq!(seeds.len(), 8);
        let cells = v.fracture_voronoi(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0), &seeds);
        assert!(!cells.is_empty(), "应至少产出一个碎块簇");
        // 守恒：提取到的格数（按盒体积折算）× 全部 = 原格数
        let mut removed = 0usize;
        for (_, boxes) in &cells {
            for (_, h) in boxes {
                // 盒体积 / 格体积 = 覆盖格数（贪心合并的盒都是整格并集）
                removed += ((2.0 * h.x / 0.5).round()
                    * (2.0 * h.y / 0.5).round()
                    * (2.0 * h.z / 0.5).round()) as usize;
            }
        }
        assert_eq!(removed, filled0, "Voronoi 分区必须恰好覆盖域内全部占据格");
        assert_eq!(v.filled_count(), 0, "域内应被全部提取");
        // 种子数 ≥ 2 时通常至少 2 个非空簇（8 个种子 + 抖动 ⇒ 必然多簇）
        assert!(cells.len() >= 2, "多种子应产出多个簇；实际 {}", cells.len());
    }

    #[test]
    pub(crate) fn bounds_track_occupied_cells() {
        let mut v = VoxelVolume::new(Vec3::ZERO, 1.0, 4, 4, 4);
        assert!(v.occupied_bounds().is_none());
        v.set(1, 0, 2, true);
        let b = v.occupied_bounds().unwrap();
        assert!((b.min.x - 1.0).abs() < 1e-6 && (b.max.x - 2.0).abs() < 1e-6);
        assert!((b.min.y - 0.0).abs() < 1e-6 && (b.max.y - 1.0).abs() < 1e-6);
        assert!((b.min.z - 2.0).abs() < 1e-6 && (b.max.z - 3.0).abs() < 1e-6);
        let _ = v.bounds(); // 覆盖 provider 路径
    }
}
