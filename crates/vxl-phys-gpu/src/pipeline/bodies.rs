//! bodies：**刚体前缀的积分**（`integrate.wgsl` 的对应物）的卡上阶段。
//!
//! 与其它阶段同款：**调用方持有**（`Packet` 字段数已顶门，见 §13.7）；`upload` → `encode` →
//! `download` 三步显式。本档**只做"给定 (F, τ, g) ⇒ 推进一个 dt"**（接触/解算/睡眠仍在主机侧）
//! ——它是"全卡上跑一个世界"的**积分腿**，不是完整刚体管线。
//!
//! 与 CPU 同式（`vxl-phys-integrate/src/lib.rs` + `vxl-phys-core/src/body.rs::apply_world_inv_inertia`）；
//! 世界系逆惯量在核内由"局部对角 + 姿态"现算（CPU 也是每次现算，无缓存依赖）。

use super::*;

use vxl_phys_core::{Quat, Vec3};

/// 一步积分的常量（重力 / dt / 限速）——攒成一个结构体，免得参数表过长。
#[derive(Clone, Copy)]
pub struct StepCfg {
    pub gravity: Vec3,
    pub dt: f32,
    pub max_lin: f32,
    pub max_ang: f32,
}

/// 一个体在卡上的 112 B（与 `body_integrate.wgsl` 的 `struct Body` 逐字段同形）。
#[derive(Clone, Copy)]
pub struct BodyState {
    pub pos: Vec3,
    pub inv_mass: f32,
    pub rot: Quat,
    pub linvel: Vec3,
    pub angvel: Vec3,
    /// **局部**逆惯量对角（世界系那份由核内旋转得到）。
    pub loc_inv_i: Vec3,
    pub force: Vec3,
    pub torque: Vec3,
}

/// 体状态在卡上的跨度（字节）：7 个 16 B 槽 = 112。
const BODY_STRIDE: usize = 112;

/// 刚体积分阶段（缓冲按体数增长；体数不变时复用）。
pub struct BodyStage {
    pipe: wgpu::ComputePipeline,
    bgl: wgpu::BindGroupLayout,
    state_b: wgpu::Buffer,
    params_b: wgpu::Buffer,
    rb: wgpu::Buffer,
    bg: wgpu::BindGroup,
    n: usize,
    cap: usize,
}

impl BodyStage {
    /// 建阶段（初始容量 1 体；`upload` 时按需增长并重绑）。
    pub fn new(device: &wgpu::Device) -> Self {
        let bgl = mk_layout(device, "b.bgl", &[(0, Kind::Uniform), (1, Kind::Rw)]);
        let sh = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("body_integrate.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../body_integrate.wgsl").into()),
        });
        let pipe = mk_pipe(device, &bgl, &sh, "b.integrate", "integrate");
        let mk = |label: &str, size: u64, extra: wgpu::BufferUsages| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC
                    | extra,
                mapped_at_creation: false,
            })
        };
        let state_b = mk("b.state", BODY_STRIDE as u64, wgpu::BufferUsages::empty());
        let params_b = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("b.params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        // 回读缓冲：`MAP_READ` 只能与**对向**的 `COPY` 组合（不能再叠 STORAGE/COPY_SRC）⇒ 单独建。
        let rb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("b.rb"),
            size: BODY_STRIDE as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let bg = Self::make_bg(device, &bgl, &params_b, &state_b);
        Self {
            pipe,
            bgl,
            state_b,
            params_b,
            rb,
            bg,
            n: 0,
            cap: 1,
        }
    }

    fn make_bg(
        device: &wgpu::Device,
        bgl: &wgpu::BindGroupLayout,
        params_b: &wgpu::Buffer,
        state_b: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("b.bg"),
            layout: bgl,
            entries: &[ent(0, params_b), ent(1, state_b)],
        })
    }

    /// 上传（重力 / dt / 限速 / 体数 + 全体状态）。返回体数。
    pub fn upload(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cfg: StepCfg,
        st: &[BodyState],
    ) -> usize {
        self.n = st.len();
        if self.n == 0 {
            return 0;
        }
        if self.n > self.cap {
            self.cap = self.n;
            self.state_b = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("b.state"),
                size: (self.n * BODY_STRIDE) as u64,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            self.rb = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("b.rb"),
                size: (self.n * BODY_STRIDE) as u64,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            self.bg = Self::make_bg(device, &self.bgl, &self.params_b, &self.state_b);
        }
        let mut prm = Vec::with_capacity(32);
        for x in [
            cfg.gravity.x,
            cfg.gravity.y,
            cfg.gravity.z,
            cfg.dt,
            cfg.max_lin,
            cfg.max_ang,
        ] {
            prm.extend_from_slice(&x.to_le_bytes());
        }
        prm.extend_from_slice(&(self.n as u32).to_le_bytes());
        prm.extend_from_slice(&[0u8; 4]);
        queue.write_buffer(&self.params_b, 0, &prm);
        queue.write_buffer(&self.state_b, 0, &body_bytes(st));
        self.n
    }

    /// 一趟积分分派（**每子步一次**，与 CPU 的 `integrate_velocities` + `integrate_positions` 对位）。
    pub fn encode(&self, enc: &mut wgpu::CommandEncoder) {
        if self.n == 0 {
            return;
        }
        dispatch(enc, &self.pipe, &self.bg, (self.n as u32).div_ceil(64));
    }

    /// 读回状态（验收/诊断用）。
    pub fn download(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Vec<BodyState> {
        if self.n == 0 {
            return Vec::new();
        }
        let bytes = (self.n * BODY_STRIDE) as u64;
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("b.dl"),
        });
        enc.copy_buffer_to_buffer(&self.state_b, 0, &self.rb, 0, bytes);
        queue.submit(Some(enc.finish()));
        device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
        let slice = self.rb.slice(..bytes);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        // 按索引解码（**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判"constant chunk size"）。
        let f = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let mut out = Vec::with_capacity(self.n);
        for k in 0..self.n {
            let o = k * BODY_STRIDE;
            out.push(BodyState {
                pos: Vec3::new(f(o), f(o + 4), f(o + 8)),
                inv_mass: f(o + 12),
                rot: Quat::new(f(o + 16), f(o + 20), f(o + 24), f(o + 28)),
                linvel: Vec3::new(f(o + 32), f(o + 36), f(o + 40)),
                angvel: Vec3::new(f(o + 48), f(o + 52), f(o + 56)),
                loc_inv_i: Vec3::new(f(o + 64), f(o + 68), f(o + 72)),
                force: Vec3::new(f(o + 80), f(o + 84), f(o + 88)),
                torque: Vec3::new(f(o + 96), f(o + 100), f(o + 104)),
            });
        }
        drop(data);
        self.rb.unmap();
        out
    }
}

/// 体状态 → 卡上布局（每体 112 B）。
fn body_bytes(st: &[BodyState]) -> Vec<u8> {
    let mut b = Vec::with_capacity(st.len() * BODY_STRIDE);
    for s in st {
        for c in [s.pos.x, s.pos.y, s.pos.z, s.inv_mass] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.rot.x, s.rot.y, s.rot.z, s.rot.w] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.linvel.x, s.linvel.y, s.linvel.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.angvel.x, s.angvel.y, s.angvel.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.loc_inv_i.x, s.loc_inv_i.y, s.loc_inv_i.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.force.x, s.force.y, s.force.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
        for c in [s.torque.x, s.torque.y, s.torque.z, 0.0] {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    b
}
