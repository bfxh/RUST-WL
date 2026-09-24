//! wall_gather：从 `fluid_density.rs` 按域拆出——**批量收集壁面接触**（主机侧 SDF 查询的批量入口）。
//!
//! 单独成档的理由：`fluid_density.rs` 在尺寸门里是棘轮档（加行要配"函数变短"），而新文件只判阈值。
//! 本档给 **GPU 档的壁面镜像 + 投影**（`wall_ghost.wgsl` / `wall_project`）与将来的 **facade 壁面档**
//! 共用——那里要的是"稀疏三段"（只有近壁粒子有条目），而 CPU 密度轮要的是逐粒定长 8 槽。

use super::*;

/// **收集口径**：镜像与投影要的不是同一份东西 ⇒ **必须各拿一张表**（`PLAN-gpu.md` §15 补记五/六）。
///
/// 混用过的代价有读数：让**投影**也吃 `Mirror` 口径 ⇒ 近壁带差 **1.26 m**、`max|Δpos|` 12.2 m
/// ——穿透粒子的截断 SDF 在固体内部梯度指向错误，沿错法线推出壁外。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum WallFlavor {
    /// **镜像口径**＝ CPU 密度轮的 `wall_planes_in`：`contacts_point` + 只留 `0 < sdf < h`。
    Mirror,
    /// **投影口径**＝ CPU `boundary_pass`：`contacts_point_boundary` + 留 `sdf < h`（**含穿透**）。
    Project,
}

impl FluidSystem {
    /// **批量收集镜像壁面接触**（稀疏三段 `(ids, start, planes)`）：只有"`h` 带内有壁面"的粒子才有
    /// 条目（`start[e]..start[e+1]` 是第 `ids[e]` 粒的平面区间）。**口径与 CPU `wall_planes_in` 同款**
    /// （`contacts_point` + `0 < sdf < h`）——镜像要的是 CPU 密度轮看的那份数据。
    pub fn gather_wall_contacts(
        boundaries: &[u32],
        h: f32,
        ps: &[Vec3],
        providers: &dyn ProviderColliders,
    ) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
        Self::gather_in(boundaries, h, ps, providers, WallFlavor::Mirror)
    }

    /// **批量收集投影壁面接触**（同上形态）：**口径与 CPU `boundary_pass` 同款**——用
    /// `contacts_point_boundary` 且**含穿透**（`sdf < h`）：穿透粒子的截断 SDF 内部梯度指向错误，
    /// 只有边界口径给得出"最近真表面"（否则沿错法线投影 ⇒ 穿壁）。
    pub fn gather_wall_project_contacts(
        boundaries: &[u32],
        h: f32,
        ps: &[Vec3],
        providers: &dyn ProviderColliders,
    ) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
        Self::gather_in(boundaries, h, ps, providers, WallFlavor::Project)
    }

    /// 两个口径共用的收集体（域与遍历序逐字相同，只有查询函数与 `sdf` 过滤不同 ⇒ 两个消费者各拿一张表）。
    fn gather_in(
        boundaries: &[u32],
        h: f32,
        ps: &[Vec3],
        providers: &dyn ProviderColliders,
        flavor: WallFlavor,
    ) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
        let (mut ids, mut start, mut planes) = (Vec::new(), vec![0u32], Vec::new());
        let mut scratch = Vec::new();
        for (i, p) in ps.iter().enumerate() {
            let mut np = 0usize;
            for &bid in boundaries {
                if let Some(bb) = providers.bounds(bid) {
                    // 预滤余量 = h（与 `wall_planes_in` 同款；穿透粒子仍在盒内 ⇒ 不会被滤掉）。
                    let m = h;
                    if p.x < bb.min.x - m
                        || p.x > bb.max.x + m
                        || p.y < bb.min.y - m
                        || p.y > bb.max.y + m
                        || p.z < bb.min.z - m
                        || p.z > bb.max.z + m
                    {
                        continue;
                    }
                }
                scratch.clear();
                let hit = match flavor {
                    WallFlavor::Mirror => providers.contacts_point(bid, *p, h, &mut scratch),
                    WallFlavor::Project => {
                        providers.contacts_point_boundary(bid, *p, h, &mut scratch)
                    }
                };
                if hit {
                    for c in scratch.iter() {
                        // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，provider 无关）。
                        let sdf = (*p - c.point).dot(c.normal);
                        let keep = match flavor {
                            // 穿透（≤ 0）不补质量：交给投影（与 CPU 同款）。
                            WallFlavor::Mirror => sdf > 0.0 && sdf < h,
                            WallFlavor::Project => sdf < h,
                        };
                        if keep && np < 8 {
                            planes.push((c.point, c.normal));
                            np += 1;
                        }
                    }
                }
            }
            if np > 0 {
                ids.push(i as u32);
                start.push(planes.len() as u32);
            }
        }
        (ids, start, planes)
    }
}
