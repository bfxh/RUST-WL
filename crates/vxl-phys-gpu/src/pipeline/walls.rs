//! walls：**提供者壁面的镜像鬼影**（GPU 侧）——CPU 侧那条在
//! `vxl-phys-fluid/src/fluid_density.rs`（`wall_planes_in` 收集 + `density_pass` 里补核质量）。
//!
//! **分工（为什么不是纯卡上）**：provider 是体素/三角网/喷溅，SDF 查询只有主机能做 ⇒
//! 主机跑 `FluidSystem::wall_planes_in`（与 CPU 档**同一个函数**），把"每粒 ≤8 面"压成**稀疏三段**
//! （`ids` / `start` / `planes`）上传；卡上只做邻居遍历 + 镜像求和（`wall_ghost.wgsl`）。
//!
//! **两处与 CPU 的已知差别**（都记在 `PLAN-gpu.md`）：
//! - **求和序**：CPU 把鬼影项与流体项交错累加，本核单独累加后一次性加到 `dens[i]` ⇒ 口径 B；
//! - **调用点**：本档由**调用方**在每子步的"密度之后、EOS 之前"显式 `encode`（`Packet` 字段已顶门，
//!   不往里塞缓冲；见 §13.7 的"棘轮管既有文件"）。

use super::*;

use vxl_phys_core::Vec3;

/// 稀疏平面表（只有近壁粒子有条目）与那趟鬼影分派。
pub struct WallStage {
    pipe: wgpu::ComputePipeline,
    bgl: wgpu::BindGroupLayout,
    /// 建包时留下的 `items`/`dens` 句柄（重绑时要再用 ⇒ 自己存一份）。
    extra: ExtraBufs,
    ids_b: wgpu::Buffer,
    start_b: wgpu::Buffer,
    planes_b: wgpu::Buffer,
    bg: wgpu::BindGroup,
    /// **投影入口**（同一 `wall_ghost.wgsl` 的第二个入口）：每子步积分之后跑，把穿透粒子
    /// 推回静置线、法向速度归零（CPU `substep` 末尾的 `boundary_pass` 对应物）。
    pipe_proj: wgpu::ComputePipeline,
    entries: usize,
    cap_entries: usize,
    cap_planes: usize,
    /// 是否投影（金丝雀/消融用；默认开）。
    project: bool,
}

/// 每个壁面在卡上的 32 B：`point.xyz | 填充 | normal.xyz | 填充`（vec3 的 16 对齐）。
const WALL_STRIDE: usize = 32;

/// **收集壁面接触**（主机侧；provider 的 SDF 查询只有主机能做）⇒ 稀疏三段表。
///
/// ⚠️ **与 `FluidSystem::gather_wall_contacts` 是同一实现的两份**（本 crate 对 `vxl-phys-fluid`
/// 只有 **dev-dependency** ⇒ lib 侧不能转发）⇒ **改一处要改两处**。探针走 fluid 那份（facade 将来
/// 也用那份）；本函数留给"只想用 GPU crate、不引 fluid"的调用方。
///
/// 域：保留 `sdf < h` 的接触（**含穿透** `sdf ≤ 0`）——同一张表给两个入口用：
/// - `wall_ghost`（密度镜像）自己按 CPU 口径过滤 `0 < sdf < h`；
/// - `wall_project`（投影）用 `pen > 0` 那部分（CPU 侧对应 `boundary_pass`）。
///
/// 用 `contacts_point_boundary`（**流体边界口径**：内点鲁棒、给出"最近真表面"）：对 `sdf > 0`
/// 它与 CPU 镜像用的 `contacts_point` 等价，对 `sdf ≤ 0` 只有它给得对（穿透粒子的投影靠它）。
pub fn gather_wall_contacts(
    boundaries: &[u32],
    h: f32,
    ps: &[Vec3],
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
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
            if providers.contacts_point_boundary(bid, *p, h, &mut scratch) {
                for c in scratch.iter() {
                    let sdf = (*p - c.point).dot(c.normal);
                    if sdf < h && np < 8 {
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

/// 建包时**没被 `Packet` 持有**的两把缓冲句柄（`items` / `dens`）——壁面鬼影阶段要绑它们。
/// 由 `Packet::new_with_walls` 在建包那一刻转交（见该函数注：`Packet` 字段数已顶门）。
pub(crate) struct ExtraBufs {
    pub(crate) items_b: wgpu::Buffer,
    pub(crate) dens_b: wgpu::Buffer,
}

impl WallStage {
    /// 建阶段（缓冲留空；`upload` 时按需重建并重绑）。uniform 复用包的**相位 uniform** ⇒
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
        let mk = |label: &str, size: u64| {
            pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: size.max(4),
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let ids_b = mk("wall.ids", 4);
        let start_b = mk("wall.start", 8);
        let planes_b = mk("wall.planes", 32);
        let bg = Self::make_bg(pkt, &bgl, &extra, &ids_b, &start_b, &planes_b);
        Self {
            pipe,
            bgl,
            extra,
            ids_b,
            start_b,
            planes_b,
            bg,
            pipe_proj,
            entries: 0,
            cap_entries: 1,
            cap_planes: 1,
            project: true,
        }
    }

    /// 开关**壁面投影**（默认开；关掉 = 只补密度、不挡穿透 ⇒ 金丝雀/消融档）。
    pub fn set_project(&mut self, on: bool) {
        self.project = on;
    }

    fn make_bg(
        pkt: &Packet,
        bgl: &wgpu::BindGroupLayout,
        extra: &ExtraBufs,
        ids_b: &wgpu::Buffer,
        start_b: &wgpu::Buffer,
        planes_b: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        pkt.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_wall"),
            layout: bgl,
            entries: &[
                ent(0, &pkt.phase_params_b),
                ent(1, &pkt.pos_b),
                ent(2, &pkt.start_b),
                ent(3, &extra.items_b),
                ent(4, &extra.dens_b),
                ent(5, ids_b),
                ent(6, start_b),
                ent(7, planes_b),
                ent(8, &pkt.vel_b),
            ],
        })
    }

    /// 上传稀疏平面表：`ids`/`start`（长度 = 条目数 + 1）与 `planes`（`(接触点, 外法线)`）。
    /// 返回条目数（0 = 本 tick 没有近壁粒子 ⇒ `encode` 直接跳过）。
    pub fn upload(
        &mut self,
        pkt: &Packet,
        ids: &[u32],
        start: &[u32],
        planes: &[(Vec3, Vec3)],
    ) -> usize {
        let (ne, np) = (ids.len(), planes.len());
        if ne != start.len().saturating_sub(1) {
            return 0; // 形状不符：宁可不干活（调用方 bug，见 §13.7 的同款纪律）
        }
        self.entries = ne;
        if ne == 0 {
            return 0;
        }
        let mut rebind = false;
        if ne > self.cap_entries {
            self.cap_entries = ne;
            self.ids_b = pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("wall.ids"),
                size: (ne as u64) * 4,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.start_b = pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("wall.start"),
                size: ((ne + 1) as u64) * 4,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            rebind = true;
        }
        if np > self.cap_planes {
            self.cap_planes = np;
            self.planes_b = pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("wall.planes"),
                size: (np as u64) * WALL_STRIDE as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            rebind = true;
        }
        if rebind {
            self.bg = Self::make_bg(
                pkt,
                &self.bgl,
                &self.extra,
                &self.ids_b,
                &self.start_b,
                &self.planes_b,
            );
        }
        pkt.queue.write_buffer(&self.ids_b, 0, &u32_bytes(ids));
        pkt.queue.write_buffer(&self.start_b, 0, &u32_bytes(start));
        pkt.queue
            .write_buffer(&self.planes_b, 0, &wall_bytes(planes));
        ne
    }

    /// **读回密度**（前 `n` 粒；诊断/验收用）：与 CPU 的 `densities()` 逐粒对拍。
    /// 走的是建包时留下的 `dens` 句柄（`Packet` 自己不持有它，见 `new_with_walls` 注）。
    pub fn read_dens(&self, pkt: &Packet, n: usize) -> Vec<f32> {
        let bytes = (n as u64) * 4;
        let rb = pkt.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("wall.dens_rb"),
            size: bytes.max(4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = pkt
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.extra.dens_b, 0, &rb, 0, bytes);
        pkt.queue.submit(Some(enc.finish()));
        pkt.poll_wait().ok();
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        pkt.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        // 按索引解码（**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判"constant chunk size"）。
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let o = k * 4;
            out.push(f32::from_le_bytes([
                data[o],
                data[o + 1],
                data[o + 2],
                data[o + 3],
            ]));
        }
        drop(data);
        rb.unmap();
        out
    }

    /// 追加一趟**壁面投影**（**每子步积分之后**；条目为 0 时是空操作）。
    /// 与 `encode`（鬼影）共用同一张平面表与同一张 bind group。
    pub fn encode_project(&self, enc: &mut wgpu::CommandEncoder) {
        if self.entries == 0 || !self.project {
            return;
        }
        let groups = (self.entries as u32).div_ceil(64);
        dispatch(enc, &self.pipe_proj, &self.bg, groups);
    }

    /// 追加一趟鬼影分派（**必须排在密度相位之后、EOS 之前**；条目为 0 时是空操作）。
    pub fn encode(&self, enc: &mut wgpu::CommandEncoder) {
        if self.entries == 0 {
            return;
        }
        let groups = (self.entries as u32).div_ceil(64);
        dispatch(enc, &self.pipe, &self.bg, groups);
    }
}

/// `u32` 切片 → 小端字节（上传用）。
fn u32_bytes(v: &[u32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// 平面表 → 卡上布局（每面 32 B：point.xyz + 填充 + normal.xyz + 填充）。
fn wall_bytes(planes: &[(Vec3, Vec3)]) -> Vec<u8> {
    let mut b = Vec::with_capacity(planes.len() * WALL_STRIDE);
    for &(p, n) in planes {
        for c in [p.x, p.y, p.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [n.x, n.y, n.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    b
}
