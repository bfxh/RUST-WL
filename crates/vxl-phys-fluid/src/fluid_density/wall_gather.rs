//! wall_gather：从 `fluid_density.rs` 按域拆出——**批量收集壁面接触**（主机侧 SDF 查询的批量入口）。
//!
//! 单独成档的理由：`fluid_density.rs` 在尺寸门里是棘轮档（加行要配"函数变短"），而新文件只判阈值。
//! 本档给 **GPU 档的壁面镜像 + 投影**（`wall_ghost.wgsl` / `wall_project`）与将来的 **facade 壁面档**
//! 共用——那里要的是"稀疏三段"（只有近壁粒子有条目），而 CPU 密度轮要的是逐粒定长 8 槽。

use super::*;

impl FluidSystem {
    /// **批量收集壁面接触**：返回**稀疏三段** `(ids, start, planes)`——只有"`h` 带内有壁面"的粒子
    /// 才有条目（`start[e]..start[e+1]` 是第 `ids[e]` 粒的平面区间）。
    ///
    /// 与 [`FluidSystem::wall_planes_in`] 的分工：那个是**逐粒 ≤8 面**的定长形态（CPU 密度轮用，只收
    /// `0 < sdf < h`）；本函数**含穿透**（收 `sdf < h` 全部），且用 `contacts_point_boundary`
    /// （**流体边界口径**：对 `sdf > 0` 与 `contacts_point` 等价，对 `sdf ≤ 0` 只有它给得对
    /// —— 穿透粒子的投影靠它）。
    pub fn gather_wall_contacts(
        boundaries: &[u32],
        h: f32,
        ps: &[Vec3],
        providers: &dyn ProviderColliders,
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
                if providers.contacts_point_boundary(bid, *p, h, &mut scratch) {
                    for c in scratch.iter() {
                        if (*p - c.point).dot(c.normal) < h && np < 8 {
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
