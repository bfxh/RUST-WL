//! walls：**提供者壁面的镜像鬼影**（GPU 侧）——CPU 侧那条在
//! `vxl-phys-fluid/src/fluid_density.rs`（`wall_planes_in` 收集 + `density_pass` 补核质量）与
//! `fluid_density/wall_gather.rs`（稀疏表批量入口 `gather_wall_contacts`）。
//!
//! **分工（为什么不是纯卡上）**：provider 是体素/三角网/喷溅，SDF 查询只有主机能做 ⇒
//! 主机把"每粒 ≤8 面"压成**稀疏三段**（`ids` / `start` / `planes`）上传；卡上只做邻居遍历 + 镜像求和
//! （`wall_ghost.wgsl`）。
//!
//! **两张表（`PLAN-gpu.md` §15 补记五/六）**：两个入口的**收集口径不同** ⇒ 各存一张、各绑各的：
//! - [`WallSide::Mirror`]（密度鬼影）＝ CPU `wall_planes_in`：`contacts_point` + 只留 `0 < sdf < h`；
//! - [`WallSide::Project`]（穿透推回）＝ CPU `boundary_pass`：`contacts_point_boundary` + 留 `sdf < h`。
//!
//! 混用的代价有读数：**让投影也吃普通口径 ⇒ 近壁带差 1.26 m、`max|Δpos|` 12.2 m**（穿壁隧逃）。
//!
//! **两处与 CPU 的已知差别**（都记在 `PLAN-gpu.md`）：
//! - **求和序**：CPU 把鬼影项与流体项交错累加，本核单独累加后一次性加到 `dens[i]` ⇒ 口径 B；
//! - **调用点**：本档由**调用方**在每子步的"密度之后、EOS 之前"显式 `encode`（`Packet` 字段已顶门，
//!   不往里塞缓冲；见 §13.7 的"棘轮管既有文件"）。

use super::*;

use super::wall_table::WallTable;
use vxl_phys_core::Vec3;

/// 建包时**没被 `Packet` 持有**的两把缓冲句柄（`items` / `dens`）——壁面表要绑它们。
/// 由 `Packet::new_with_walls` 在建包那一刻转交（见该函数注：`Packet` 字段数已顶门）。
/// 定义在 [`wall_table`](super::wall_table)，这里只做转发以免动 `pipeline.rs` 的路径。
pub(crate) use super::wall_table::ExtraBufs;

/// 壁面档的**两侧**：镜像（密度鬼影）与投影（穿透推回）。两侧的**收集口径不同** ⇒ 各有一张表；
/// 本枚举同时用于"收集哪一侧"（[`gather_wall_contacts`]）与"上传哪一侧"（[`WallStage::upload`]）。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WallSide {
    /// 密度鬼影侧（`contacts_point` + `0 < sdf < h`）。
    Mirror,
    /// 穿透投影侧（`contacts_point_boundary` + `sdf < h`）。
    Project,
}

/// 两张稀疏平面表 + 两个入口的管线。
pub struct WallStage {
    pipe: wgpu::ComputePipeline,
    /// **投影入口**（同一 `wall_ghost.wgsl` 的第二个入口）：每子步积分之后跑，把穿透粒子
    /// 推回静置线、法向速度归零（CPU `substep` 末尾的 `boundary_pass` 对应物）。
    pipe_proj: wgpu::ComputePipeline,
    /// **镜像表**（[`WallSide::Mirror`] 口径收集）：`wall_ghost` 入口用。
    mirror: WallTable,
    /// **投影表**（[`WallSide::Project`] 口径收集）：`wall_project` 入口用。
    proj: WallTable,
    /// 是否投影（金丝雀/消融用；默认开）。
    project: bool,
}

impl WallStage {
    /// 建阶段（两张表都留空；`upload` 时按需重建并重绑）。uniform 复用包的**相位 uniform** ⇒
    /// 网格映射与密度核逐字一致；`extra` 提供建包时留下的 `items`/`dens` 句柄。
    pub(crate) fn new(pkt: &Packet, extra: ExtraBufs) -> Self {
        let bgl = mk_layout(
            &pkt.device,
            "p.wall_bgl",
            &[
                (0, Kind::Uniform),
                // 1 pos：鬼影只读、**投影要写** ⇒ 声明 Rw（两个入口共用同一张布局）。
                (1, Kind::Rw),
                (2, Kind::Ro),
                (3, Kind::Ro),
                (4, Kind::Rw),
                (5, Kind::Ro),
                (6, Kind::Ro),
                (7, Kind::Ro),
                // 8 vel：投影把法向速度归零（鬼影不碰）。
                (8, Kind::Rw),
            ],
        );
        let sh = pkt
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("wall_ghost.wgsl"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../wall_ghost.wgsl").into()),
            });
        let pipe = mk_pipe(&pkt.device, &bgl, &sh, "p.wall_ghost", "wall_ghost");
        let pipe_proj = mk_pipe(&pkt.device, &bgl, &sh, "p.wall_project", "wall_project");
        let mirror = WallTable::empty(pkt, &bgl, &extra);
        let proj = WallTable::empty(pkt, &bgl, &extra);
        Self {
            pipe,
            pipe_proj,
            mirror,
            proj,
            project: true,
        }
    }

    /// 开关**壁面投影**（默认开；关掉 = 只补密度、不挡穿透 ⇒ 金丝雀/消融档）。
    pub fn set_project(&mut self, on: bool) {
        self.project = on;
    }

    /// 上传**一侧**的稀疏平面表：`ids`/`start`（长度 = 条目数 + 1）与 `planes`（`(接触点, 外法线)`）。
    /// 返回该表的条目数（0 = 这一侧本 tick 没东西可干 ⇒ 对应那趟直接跳过）。
    ///
    /// **口径由调用方负责**：镜像侧要 [`WallSide::Mirror`] 口径收集的表、投影侧要
    /// [`WallSide::Project`] 口径——两张表串了就是档首那 1.26 m。
    pub fn upload(
        &mut self,
        pkt: &Packet,
        side: WallSide,
        ids: &[u32],
        start: &[u32],
        planes: &[(Vec3, Vec3)],
    ) -> usize {
        match side {
            WallSide::Mirror => self.mirror.upload(pkt, ids, start, planes),
            WallSide::Project => self.proj.upload(pkt, ids, start, planes),
        }
    }

    /// **读回密度**（前 `n` 粒；诊断/验收用）：与 CPU 的 `densities()` 逐粒对拍。
    /// 密度只看镜像侧 ⇒ 走镜像表（投影不改 `dens`）。
    pub fn read_dens(&self, pkt: &Packet, n: usize) -> Vec<f32> {
        self.mirror.read_dens(pkt, n)
    }

    /// 追加一趟**壁面投影**（**每子步积分之后**；投影表空或关了投影 ⇒ 空操作）。
    pub fn encode_project(&self, enc: &mut wgpu::CommandEncoder) {
        if self.project {
            self.proj.encode(enc, &self.pipe_proj);
        }
    }

    /// 追加一趟鬼影分派（**必须排在密度相位之后、EOS 之前**；镜像表空 ⇒ 空操作）。
    pub fn encode(&self, enc: &mut wgpu::CommandEncoder) {
        self.mirror.encode(enc, &self.pipe);
    }
}

/// **收集镜像壁面接触**（主机侧；provider 的 SDF 查询只有主机能做）⇒ 稀疏三段表。
/// **口径**＝ CPU `wall_planes_in`：`contacts_point` + 只留 `0 < sdf < h`。
///
/// ⚠️ **与 `FluidSystem::gather_wall_contacts` 是同一实现的两份**（本 crate 对 `vxl-phys-fluid`
/// 只有 **dev-dependency** ⇒ lib 侧不能转发）⇒ **改一处要改两处**。探针走 fluid 那份（facade 将来
/// 也用那份）；本函数留给"只想用 GPU crate、不引 fluid"的调用方。
pub fn gather_wall_contacts(
    boundaries: &[u32],
    h: f32,
    ps: &[Vec3],
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
    gather_in(boundaries, h, ps, providers, WallSide::Mirror)
}

/// **收集投影壁面接触**（同形态）。**口径**＝ CPU `boundary_pass`：`contacts_point_boundary`
/// 且**含穿透**（`sdf < h`）——穿透粒子的截断 SDF 内部梯度指向错误，只有边界口径给得出
/// "最近真表面"（否则沿错法线投影 ⇒ 穿壁）。同上一份两处实现，**改一处要改两处**。
pub fn gather_wall_project_contacts(
    boundaries: &[u32],
    h: f32,
    ps: &[Vec3],
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
    gather_in(boundaries, h, ps, providers, WallSide::Project)
}

/// 两个口径共用的收集体（域与遍历序逐字相同，只有查询函数与 `sdf` 过滤不同）。
fn gather_in(
    boundaries: &[u32],
    h: f32,
    ps: &[Vec3],
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
    side: WallSide,
) -> (Vec<u32>, Vec<u32>, Vec<(Vec3, Vec3)>) {
    let mut out_ids = Vec::new();
    let mut start = vec![0u32];
    let mut planes = Vec::new();
    let mut scratch = Vec::new();
    for (i, p) in ps.iter().enumerate() {
        let mut np = 0usize;
        for &bid in boundaries {
            if let Some(bb) = providers.bounds(bid) {
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
            let hit = match side {
                WallSide::Mirror => providers.contacts_point(bid, *p, h, &mut scratch),
                WallSide::Project => providers.contacts_point_boundary(bid, *p, h, &mut scratch),
            };
            if hit {
                for c in scratch.iter() {
                    let sdf = (*p - c.point).dot(c.normal);
                    let keep = match side {
                        WallSide::Mirror => sdf > 0.0 && sdf < h,
                        WallSide::Project => sdf < h,
                    };
                    if keep && np < 8 {
                        planes.push((c.point, c.normal));
                        np += 1;
                    }
                }
            }
        }
        if np > 0 {
            out_ids.push(i as u32);
            start.push(planes.len() as u32);
        }
    }
    (out_ids, start, planes)
}
