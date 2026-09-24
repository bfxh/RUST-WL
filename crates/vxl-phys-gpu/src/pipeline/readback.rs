//! readback：从 pipeline.rs 按域拆出（纯搬移，语义未改）。
//!
//! 主机侧的回读胶水：边界反作用力、overflow 护栏计数、状态（`pos`/`vel`）与格表句柄。
//! 它们只做「映射缓冲 → 解码 → 等待」，与相位编排无关 ⇒ 单独成档（也让 pipeline.rs
//! 回到尺寸门以内）。

use super::*;

impl Packet {
    /// **回读边界粒子的反作用力**（`out_b` 的后缀；末子步的值，每粒 6 个 f32：`(力, xsph)`）。
    ///
    /// 只回读边界段（`(n − n_fluid)` 粒）——这是"反作用回读"的最小代价形态，验收时与 CPU 的
    /// `FluidSystem::boundary_forces()` 对拍（见 `PLAN-gpu.md` §13.2）。
    pub fn read_boundary_forces(&self, n_fluid: u32) -> Vec<f32> {
        let off = (n_fluid as u64) * 24;
        let bytes = ((self.n - n_fluid) as u64) * 24;
        if bytes == 0 {
            return Vec::new();
        }
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.bforce_rb"),
            size: bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("p.bforce.enc"),
            });
        enc.copy_buffer_to_buffer(&self.out_b, off, &rb, 0, bytes);
        self.queue.submit(Some(enc.finish()));
        self.poll_wait().ok();
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        // 按索引解码（**别用 `chunks_exact(4)`**：CI 的 clippy 比本机新，会判
        // "using `chunks_exact` with a constant chunk size" ⇒ 门红。`bbox.rs` 的 `decode_bb` 同写法。）
        let n4 = (bytes / 4) as usize;
        let mut out = Vec::with_capacity(n4);
        for k in 0..n4 {
            let o = k * 4;
            out.push(f32::from_le_bytes([
                data[o],
                data[o + 1],
                data[o + 2],
                data[o + 3],
            ]));
        }
        // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
        drop(data);
        rb.unmap();
        out
    }

    /// 格表句柄（诊断/复核用）：`(start, cursor, 活的格数)`。
    /// 两者都由 `scan` 写（见 `grid.wgsl`），主机侧要核对"网格表是不是按预期建的"时取它们。
    pub fn grid_tables(&self) -> (&wgpu::Buffer, &wgpu::Buffer, u32) {
        (&self.start_b, &self.cursor_b, self.total)
    }

    /// 读回**未规范化的格数**（`grid.wgsl` 的 `cap` 护栏计数）：= 0 才说明网格表和 CPU 同规则。
    pub fn read_overflow(&self) -> u32 {
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.of_rb"),
            size: 4,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.overflow_b, 0, &rb, 0, 4);
        self.queue.submit(Some(enc.finish()));
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        drop(data);
        rb.unmap();
        v
    }

    /// 读回 `pos` / `vel`（各 `n×3` f32 扁平）。
    pub fn read_state(&self) -> (Vec<f32>, Vec<f32>) {
        let bytes = (self.n as u64) * 12;
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.state_rb"),
            size: bytes * 2,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.pos_b, 0, &rb, 0, bytes);
        enc.copy_buffer_to_buffer(&self.vel_b, 0, &rb, bytes, bytes);
        self.queue.submit(Some(enc.finish()));
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        let parse = |off: usize| -> Vec<f32> {
            (0..self.n as usize * 3)
                .map(|i| {
                    let c = &data[off + i * 4..off + i * 4 + 4];
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]])
                })
                .collect()
        };
        let pos = parse(0);
        let vel = parse(bytes as usize);
        drop(data);
        rb.unmap();
        (pos, vel)
    }
}
