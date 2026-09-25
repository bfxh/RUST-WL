//! readback：从 pipeline.rs 按域拆出（纯搬移，语义未改）。
//!
//! 主机侧的回读胶水：边界反作用力、overflow 护栏计数、状态（`pos`/`vel`）与格表句柄。
//! 它们只做「映射缓冲 → 解码 → 等待」，与相位编排无关 ⇒ 单独成档（也让 pipeline.rs
//! 回到尺寸门以内）。

use super::*;

impl Packet {
    /// **回读边界粒子的反作用力**（`out_b` 的后缀；末子步的值，每粒 6 个 f32：`(力, xsph)`）。
    pub fn read_boundary_forces(&self, n_fluid: u32) -> Vec<f32> {
        self.read_out(n_fluid)
    }

    /// **读回 `out`（力相输出）的第 `from` 粒起直到末尾**（每粒 6 个 f32：`(acc.xyz, xsph.xyz)` 交错）。
    ///
    /// `from = 0` ⇒ **全体**：分相定位用——力相输出是积分的**直接输入**，所以"密度逐粒同、位移却不同"
    /// 这条矛盾要么卡在它身上，要么卡在它下游的积分上；`from = n_fluid` ⇒ 边界段（反作用回读）。
    pub fn read_out(&self, from: u32) -> Vec<f32> {
        let off = (from as u64) * 24;
        let bytes = ((self.n - from) as u64) * 24;
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

    /// **通用回读**：把 `buf` 的前 `n` 个 f32 读回来（`read_dens` 用；
    /// 按索引解码——**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判"constant chunk size"）。
    pub(crate) fn read_f32_head(&self, buf: &wgpu::Buffer, n: usize) -> Vec<f32> {
        let bytes = (n as u64) * 4;
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.head_rb"),
            size: bytes.max(4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(buf, 0, &rb, 0, bytes);
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

    /// 格表句柄（诊断/复核用）：`(start, cursor, 活的格数)`。
    /// 两者都由 `scan` 写（见 `grid.wgsl`），主机侧要核对"网格表是不是按预期建的"时取它们。
    pub fn grid_tables(&self) -> (&wgpu::Buffer, &wgpu::Buffer, u32) {
        (&self.start_b, &self.cursor_b, self.total)
    }

    /// **读回卡上每子步算好的包围盒**（`bbox.wgsl` 的 `box_out`，48 B：
    /// `min.xyz | inv | dims.xyz | total | hi.xyz`，**后七槽是 u32 位模式**）。返回 `(min, hi)` ——
    /// **紧凑 AABB**（`min` 槽与 8..10 的 `hi` 都是精确的粒子极值），与 CPU 侧 `particle_bounds`
    /// 同口径；近域过滤要的就是它（网格盒的 max 被 dims 上取整放大过，最多差 1 bin）。
    /// 坏值/非有限 ⇒ `None`（调用方退回主机侧那份）。
    pub fn read_box(&self) -> Option<([f32; 3], [f32; 3])> {
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.box_rb"),
            size: 48,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("p.box.enc"),
            });
        enc.copy_buffer_to_buffer(self.bbox.box_out(), 0, &rb, 0, 48);
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
        // 按索引解码（**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判"constant chunk size"）。
        let f = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let inv = f(12);
        let min = [f(0), f(4), f(8)];
        let hi = [f(32), f(36), f(40)];
        let bad = !inv.is_finite()
            || inv <= 0.0
            || !min.iter().all(|x| x.is_finite())
            || !hi.iter().all(|x| x.is_finite());
        drop(data);
        rb.unmap();
        if bad {
            return None;
        }
        Some((min, hi))
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

    /// **取走"管线档"回读的数据**（配套 `Packet::run_deferred`，`PLAN-gpu` §19.5）：等完待完成的
    /// GPU 工作 → 映射 `readback_b` → 拷出 `pos`（n×3 f32）→ 解除映射。宿主典型用法：
    /// `pk.run_deferred(cfg, 1, substeps); /* 帧内其它工作 */; let pos = pk.take_deferred_state();`
    /// ⇒ 回读的等待被中间那段工作遮住。与 `read_state` 的区别：**不再新起一趟拷贝**（用
    /// `run_deferred` 已发起的那趟），且只回 `pos`（管线档的消费方只需要位置）。
    /// ⚠️ 拿到的就是**上一 tick** 的状态（见 `run_deferred` 注）——耦合解算要"本 tick"就别用本方法。
    pub fn take_deferred_state(&self) -> Vec<f32> {
        self.poll_wait().ok();
        let slice = self.readback_b.slice(..(self.n as u64) * 12);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        let out = (0..self.n as usize * 3)
            .map(|i| {
                let c = &data[i * 4..i * 4 + 4];
                f32::from_le_bytes([c[0], c[1], c[2], c[3]])
            })
            .collect();
        drop(data);
        self.readback_b.unmap();
        out
    }
}
