//! wall_table：**稀疏壁面表**（`ids` / `start` / `planes` + 自己的 bind group）——壁面档的两张表共用同一形态。
//!
//! 为什么要两张：镜像（`WallSide::Mirror`）与投影（`WallSide::Project`）的**收集口径**不同
//! （`PLAN-gpu.md` §15 补记五/六），共用一张必有一方吃错口径——投影吃普通口径的读数是
//! **近壁带差 1.26 m**（穿壁隧逃）。本档只管"表"这件事（缓冲增长 / 重绑 / 分派），
//! 两个入口的编排在 `walls.rs` 的 `WallStage`。

use super::*;

use vxl_phys_core::Vec3;

/// 每个壁面在卡上的 32 B：`point.xyz | 填充 | normal.xyz | 填充`（vec3 的 16 对齐）。
const WALL_STRIDE: usize = 32;

/// 建包时**没被 `Packet` 持有**的两把缓冲句柄（`items` / `dens`）——壁面表要绑它们。
/// 由 `Packet::new_with_walls` 在建包那一刻转交（见该函数注：`Packet` 字段数已顶门）。
pub(crate) struct ExtraBufs {
    pub(crate) items_b: wgpu::Buffer,
    pub(crate) dens_b: wgpu::Buffer,
}

/// 一张稀疏平面表（只有近壁粒子有条目）+ 它的 bind group。条目为 0 ⇒ `encode` 是空操作。
pub(crate) struct WallTable {
    /// 建包时转交的 `items`/`dens`（重绑时要再用 ⇒ 各留一份句柄；`wgpu::Buffer` 克隆是引用计数）。
    items_b: wgpu::Buffer,
    dens_b: wgpu::Buffer,
    bgl: wgpu::BindGroupLayout,
    ids_b: wgpu::Buffer,
    start_b: wgpu::Buffer,
    planes_b: wgpu::Buffer,
    bg: wgpu::BindGroup,
    entries: usize,
    cap_entries: usize,
    cap_planes: usize,
}

impl WallTable {
    /// 空表（缓冲留空；`upload` 时按需重建并重绑）。布局与 `pkt` 的相位 uniform 共用
    /// ⇒ 网格映射与密度核逐字一致。
    pub(crate) fn empty(pkt: &Packet, bgl: &wgpu::BindGroupLayout, extra: &ExtraBufs) -> Self {
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
        let bg = make_bg(pkt, bgl, extra, &ids_b, &start_b, &planes_b);
        Self {
            items_b: extra.items_b.clone(),
            dens_b: extra.dens_b.clone(),
            bgl: bgl.clone(),
            ids_b,
            start_b,
            planes_b,
            bg,
            entries: 0,
            cap_entries: 1,
            cap_planes: 1,
        }
    }

    /// 上传稀疏表：`ids`/`start`（长度 = 条目数 + 1）与 `planes`（`(接触点, 外法线)`）。
    /// 形状不符 ⇒ 记 0 条并返回 0（宁可不干活；调用方 bug，见 §13.7 的同款纪律）。
    pub(crate) fn upload(
        &mut self,
        pkt: &Packet,
        ids: &[u32],
        start: &[u32],
        planes: &[(Vec3, Vec3)],
    ) -> usize {
        let (ne, np) = (ids.len(), planes.len());
        if ne != start.len().saturating_sub(1) {
            self.entries = 0;
            return 0;
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
            self.bg = make_bg(
                pkt,
                &self.bgl,
                &ExtraBufs {
                    items_b: self.items_b.clone(),
                    dens_b: self.dens_b.clone(),
                },
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
    /// 走的是建包时转交的 `dens` 句柄（`Packet` 自己不持有它，见 `new_with_walls` 注）。
    pub(crate) fn read_dens(&self, pkt: &Packet, n: usize) -> Vec<f32> {
        pkt.read_f32_head(&self.dens_b, n)
    }

    /// 分派这趟（`pipe` = 要跑的入口；条目为 0 ⇒ 空操作）。
    pub(crate) fn encode(&self, enc: &mut wgpu::CommandEncoder, pipe: &wgpu::ComputePipeline) {
        if self.entries == 0 {
            return;
        }
        let groups = (self.entries as u32).div_ceil(64);
        dispatch(enc, pipe, &self.bg, groups);
    }
}

/// 建 bind group（**两个入口共用同一张布局**：`pos`/`dens`/`vel` 声明为 `Rw` 以同时容纳
/// 只读的鬼影与要写回位置的投影）。
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
