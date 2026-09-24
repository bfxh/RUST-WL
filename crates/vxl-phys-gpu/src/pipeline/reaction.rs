//! reaction：边界粒子反作用的**卡上聚合**（`PLAN-gpu.md` §13.2 的"剩半步"）。
//!
//! CPU 侧的口径（`vxl-phys-fluid/src/fluid_boundary.rs::aggregate_reactions`）：逐粒 `bforce`
//! 按**段表**（`spans`）聚合成每体 `(力, 绕体原点的力矩)`，求和序 = 段序 × 段内索引序。
//! 本档把同一步搬到卡上（`reduce.wgsl`），于是耦合回路要回读的不再是"边界段"（20k × 24 B
//! ≈ 480 KB/tick），而是**每体 6 个 f32**（5 体 = 120 B）。
//!
//! **判据**（§13.2 三条，实现里逐条对应）：
//! - **求和序**：一个体 = 一个 workgroup、段内升序串行 ⇒ 与 CPU 的 `for k in start..end` 同序
//!   （不用原子加：原子浮点加不定序 ⇒ 比口径 B 更差）；
//! - **符号/参照系**：核里 `d = p_k − origin`、`τ = Σ d × f_k`，与 CPU 逐字同式；
//! - **回读量**：每体 6 个 f32（本档的 `aggregate` 就回读这个）。

use super::*;

use vxl_phys_core::Vec3;

/// 段表条目在卡上的跨度（字节）：8 个 32 位槽 —— `origin.xyz | start | end | 3 个填充`。
/// 与 `reduce.wgsl` 的 `struct Span` 布局一致（vec3 后接 u32 按 WGSL 规则落在偏移 12）。
const SPAN_STRIDE: usize = 32;

/// 反作用聚合阶段：管线建一次；段表/输出缓冲按**体数**变化时重建（体数不变则复用）。
pub struct ReactionStage {
    pipe: wgpu::ComputePipeline,
    bgl: wgpu::BindGroupLayout,
    spans_b: Option<wgpu::Buffer>,
    react_b: Option<wgpu::Buffer>,
    rb: Option<wgpu::Buffer>,
    n_bodies: usize,
}

impl ReactionStage {
    /// 建管线（`reduce.wgsl` 的 `reduce` 入口）。缓冲留到 `aggregate` 按体数分配。
    pub fn new(pkt: &Packet) -> Self {
        let sh = pkt
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("reduce.wgsl"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../reduce.wgsl").into()),
            });
        // 0 pos(只读) | 1 out(只读) | 2 spans(只读) | 3 react(读写) —— 无 uniform（段表长度由
        // 缓冲字节数给出 ⇒ `arrayLength`），所以这张布局只有 storage 槽。
        let bgl = mk_layout(
            &pkt.device,
            "p.bl_reduce",
            &[(0, Kind::Ro), (1, Kind::Ro), (2, Kind::Ro), (3, Kind::Rw)],
        );
        let pipe = mk_pipe(&pkt.device, &bgl, &sh, "p.reduce", "reduce");
        Self {
            pipe,
            bgl,
            spans_b: None,
            react_b: None,
            rb: None,
            n_bodies: 0,
        }
    }

    /// 段表/输出缓冲按体数（重）建；体数不变时复用（耦合回路每 tick 调用也不重建）。
    fn prepare(&mut self, pkt: &Packet, n: usize) {
        if self.n_bodies == n && self.spans_b.is_some() {
            return;
        }
        let mk = |label: &str, size: u64, usage: wgpu::BufferUsages| {
            pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        self.spans_b = Some(mk(
            "r.spans",
            (n * SPAN_STRIDE) as u64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        ));
        self.react_b = Some(mk(
            "r.react",
            (n * 24) as u64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        ));
        self.rb = Some(mk(
            "r.rb",
            (n * 24) as u64,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        ));
        self.n_bodies = n;
    }

    /// 段表上传（每体 32 B）：`(体 id, 体原点, 起始, 结束)`——体 id 只留在主机侧做还原，
    /// 卡上只读 `origin/start/end`（体 id 对求和没有贡献）。
    fn upload_spans(&self, pkt: &Packet, spans: &[(u32, Vec3, u32, u32)]) {
        let mut raw = Vec::with_capacity(spans.len() * SPAN_STRIDE);
        for &(_, origin, start, end) in spans {
            for c in [origin.x, origin.y, origin.z] {
                raw.extend_from_slice(&c.to_le_bytes());
            }
            for v in [start, end, 0u32, 0u32, 0u32] {
                raw.extend_from_slice(&v.to_le_bytes());
            }
        }
        if let Some(b) = self.spans_b.as_ref() {
            pkt.queue.write_buffer(b, 0, &raw);
        }
    }

    /// **聚合**：`out` 的边界段 → 每体 `(体 id, 力, 绕体原点的力矩)`。
    ///
    /// 返回序 = 传入 `spans` 的段序（与 CPU `boundary_reactions()` 同序 ⇒ 可直接对拍）。
    /// 只在**末子步之后**调用（读的就是末子步的 `out` 与当时的 `pos`，与 CPU 的
    /// `aggregate_reactions` 同一时刻口径）。
    pub fn aggregate(
        &mut self,
        pkt: &Packet,
        spans: &[(u32, Vec3, u32, u32)],
    ) -> Vec<(u32, Vec3, Vec3)> {
        if spans.is_empty() {
            return Vec::new();
        }
        self.prepare(pkt, spans.len());
        self.upload_spans(pkt, spans);
        let (Some(spans_b), Some(react_b), Some(rb)) = (
            self.spans_b.as_ref(),
            self.react_b.as_ref(),
            self.rb.as_ref(),
        ) else {
            // `prepare` 之后不可能走到这里（缓冲三件套同生同灭）——留个明确出口而不是 unwrap。
            return Vec::new();
        };
        let bg = pkt.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_reduce"),
            layout: &self.bgl,
            entries: &[
                ent(0, &pkt.pos_b),
                ent(1, &pkt.out_b),
                ent(2, spans_b),
                ent(3, react_b),
            ],
        });
        let bytes = (spans.len() * 24) as u64;
        let mut enc = pkt
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("p.reduce.enc"),
            });
        // 一个体一个 workgroup（段内并行会改求和序 ⇒ 只有 gid.x < 体数 的那条线程干活）。
        dispatch(&mut enc, &self.pipe, &bg, spans.len() as u32);
        enc.copy_buffer_to_buffer(react_b, 0, rb, 0, bytes);
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
        // 按索引解码（**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判
        // "using `chunks_exact` with a constant chunk size" ⇒ 门红）。
        let g = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let mut out = Vec::with_capacity(spans.len());
        for (b, &(body, _, _, _)) in spans.iter().enumerate() {
            let o = b * 24;
            out.push((
                body,
                Vec3::new(g(o), g(o + 4), g(o + 8)),
                Vec3::new(g(o + 12), g(o + 16), g(o + 20)),
            ));
        }
        // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
        drop(data);
        rb.unmap();
        out
    }

    /// **验收报告**（`gpu_tick_probe --tank` 用）：GPU 逐粒 vs CPU 逐粒 + GPU 每体聚合 vs
    /// CPU `breact`，外加一条"同一批项换序求和"的自洽核对。
    ///
    /// 参数取**朴素类型**（本 crate 对 `vxl-phys-fluid` 只有 `[dev-dependencies]` ⇒ 不引用它的
    /// 类型）；CPU 侧直接喂 `boundary_forces()` / `boundary_reactions()` / `boundary_spans()`。
    pub fn report(
        &mut self,
        pkt: &Packet,
        n_fluid: u32,
        cpu_force: &[Vec3],
        cpu_react: &[(u32, Vec3, Vec3)],
        spans: &[(u32, Vec3, u32, u32)],
    ) -> String {
        let gf = pkt.read_boundary_forces(n_fluid);
        if gf.len() < cpu_force.len() * 6 {
            return format!(
                "  ├ 反作用：GPU 回读不足（{} < {}）——跳过报数\n",
                gf.len(),
                cpu_force.len() * 6
            );
        }
        // ① 逐粒（与 §13.2 的验收口径同一组数）
        let (mut mx, mut scale) = (0.0f32, 0.0f32);
        let (mut sc, mut sg) = (Vec3::ZERO, Vec3::ZERO);
        for (k, b) in cpu_force.iter().enumerate() {
            let g = Vec3::new(gf[k * 6], gf[k * 6 + 1], gf[k * 6 + 2]);
            mx = mx.max((*b - g).length());
            scale += b.length();
            sc += *b;
            sg += g;
        }
        let mut s = format!(
            "  ├ 反作用【逐粒】{} 粒边界：max |ΔF| = {mx:.3e} N（相对 Σ|F_cpu| = {:.2e}）| \
             ΣF：CPU {:.4e} N vs GPU {:.4e} N（差 {:.2e}）\n",
            cpu_force.len(),
            mx / scale.max(1e-30),
            sc.length(),
            sg.length(),
            (sc - sg).length()
        );
        // ② 每体（卡上聚合）——相对量用 CPU 侧的 Σ|F|、Σ|τ| 标定
        let gb = self.aggregate(pkt, spans);
        let (mut mf, mut mt) = (0.0f32, 0.0f32);
        let (mut sf, mut st) = (0.0f32, 0.0f32);
        let mut tf = Vec3::ZERO;
        let mut rows = String::new();
        for (i, &(body, f, tau)) in cpu_react.iter().enumerate() {
            let (g, gt) = match gb.get(i) {
                Some(&(_, g, gt)) => (g, gt),
                None => (Vec3::ZERO, Vec3::ZERO),
            };
            mf = mf.max((f - g).length());
            mt = mt.max((tau - gt).length());
            sf += f.length();
            st += tau.length();
            tf += g;
            if i < 8 {
                rows.push_str(&format!(
                    "  │   体 {body}：|ΔF| {:.3e} N / |Δτ| {:.3e}\n",
                    (f - g).length(),
                    (tau - gt).length()
                ));
            }
        }
        s.push_str(&format!(
            "  ├ 反作用【每体】{} 体（卡上聚合）：max |ΔF| = {mf:.3e} N（相对 Σ|F_cpu| = {:.2e}）\
             / max |Δτ| = {mt:.3e}（相对 Σ|τ_cpu| = {:.2e}）\n",
            cpu_react.len(),
            mf / sf.max(1e-30),
            mt / st.max(1e-30)
        ));
        s.push_str(&rows);
        s.push_str(&format!(
            "  └ 自洽：Σ_体 F_gpu − Σ_粒 F_gpu = {:.3e} N（同一批项换序求和 ⇒ 只该差舍入）\n",
            (tf - sg).length()
        ));
        s
    }
}
