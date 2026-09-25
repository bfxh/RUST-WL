//! **重排 pass 的成本探针**（`PLAN-gpu.md` §21.5 第 1 条 —— 那条"按格重排"杠杆的**代价侧**）。
//!
//! 背景：`gpu_layout_probe` 量出"数据按格序摆"在两个邻域相位上值 **0.64×**（10M 档 44.49 →
//! 28.49 ms/子步），但**数据得先搬过去**。本探针只量这一步搬运：
//! - `gather` ：索引序 → 格序（`x_c[m] = x[items[m]]`，三张表：pos/vel/press = 28 B/粒）；
//! - `scatter`：把两相位输出（acc+xsph 交错 = 24 B/粒）从格序**散写回**索引序。
//!
//! 判读口径：`gather` 是"每子步都搬"的代价；`scatter` 只在"格序只当输入副本、下游仍吃索引序"那条路上
//! 才每子步出现（若整条链都留在格序，它退化成**每次回读一次**）。
//! `--shuffle` 把置换换成随机置换 ⇒ 量"索引序与空间完全解耦"的**上界档**（不量它就会把收益报高）。
//!
//! 计时：两档 repeats 解 `t = c + R/r`（不读回结果 ⇒ 没有大块回读项）；**开头必须显式预热一次**
//! ——`queue.write_buffer` 的上传是在**下一个提交**里落地的，不预热会把 320 MB 上传算进第一次计时
//! （实测把 `gather` 抬到 39.63 ms/轮、两档读数反过来、解出负 `R`）。
//! 另加金丝雀：抽样校验 `x_c[m] == x[items[m]]`（接线错了会算出一个"很快"的假数）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_reorder_probe -- [n] [--cpu-threads K]
//!        [--shuffle] [--r1 K] [--r2 K]`

use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::probe;

/// 与 `reorder.wgsl` 的 `Params` 逐字节对应。
#[repr(C)]
#[derive(Clone, Copy)]
struct RParams {
    n: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

/// 一档（`gather` / `scatter`）的计时：`c` = 每轮稳态，`R` = 固定项（排空 + 首次提交）。
struct Meas {
    c: f64,
    r: f64,
    ok: bool,
}

fn f32s(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

fn u32s(v: &[u32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// 场景与 `gpu_layout_probe` / `gpu_tick_probe` **同一套**（零重力 + 5 趟静置 + 剪切初速）。
fn build_scene(n: usize, cpu_threads: usize) -> FluidSystem {
    let spacing = 0.05f32;
    let cfg = FluidConfig {
        threads: cpu_threads,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    f
}

/// 扁平化的位置/速度（xyz 交错）。
fn flatten(f: &FluidSystem) -> (Vec<f32>, Vec<f32>) {
    let np = f.len();
    let mut pos: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        let p = f.positions()[k];
        pos.extend_from_slice(&[p.x, p.y, p.z]);
        let v = f.velocities()[k];
        vel.extend_from_slice(&[v.x, v.y, v.z]);
    }
    (pos, vel)
}

/// 固定种子的 xorshift 置换（可复现；不引外部 crate）。
fn shuffle(n: usize) -> Vec<u32> {
    let mut s = 0x9E37_79B9_7F4A_7C15u64;
    let mut v: Vec<u32> = (0..n as u32).collect();
    for i in (1..n).rev() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        let j = (s % (i as u64 + 1)) as usize;
        v.swap(i, j);
    }
    v
}

/// 两档 repeats 解 `t = c + R/r`。`R` 必须为正且占比小；否则报两档较大者当**上界**。
fn solve(r1: usize, p1: f64, r2: usize, p2: f64) -> (f64, f64, bool) {
    let (a, b) = (r1 as f64, r2 as f64);
    let c = (p1 * a - p2 * b) / (a - b);
    let r = p1 * a - c * a;
    let ok = r > 0.0 && r < 0.5 * p2 * b;
    (if ok { c } else { p1.max(p2) }, r, ok)
}

/// 全部缓冲（`gather` 的源/目标 + `scatter` 的源/目标 + 置换 + uniform）。
struct Buffers {
    items: wgpu::Buffer,
    pos: wgpu::Buffer,
    vel: wgpu::Buffer,
    press: wgpu::Buffer,
    pos_c: wgpu::Buffer,
    vel_c: wgpu::Buffer,
    press_c: wgpu::Buffer,
    out_c: wgpu::Buffer,
    out: wgpu::Buffer,
    params: wgpu::Buffer,
}

fn make_buffers(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    np: usize,
    items: &[u32],
    pos: &[f32],
    vel: &[f32],
    press: &[f32],
) -> Buffers {
    let mk = |label: &str, size: u64| -> wgpu::Buffer {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let items_b = mk("items", (np * 4) as u64);
    queue.write_buffer(&items_b, 0, &u32s(items));
    let pos_b = mk("pos", (np * 12) as u64);
    queue.write_buffer(&pos_b, 0, &f32s(pos));
    let vel_b = mk("vel", (np * 12) as u64);
    queue.write_buffer(&vel_b, 0, &f32s(vel));
    let press_b = mk("press", (np * 4) as u64);
    queue.write_buffer(&press_b, 0, &f32s(press));
    let params_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("reorder.params"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let p = RParams {
        n: np as u32,
        _pad0: 0,
        _pad1: 0,
        _pad2: 0,
    };
    let mut b = Vec::with_capacity(16);
    for x in [p.n, p._pad0, p._pad1, p._pad2] {
        b.extend_from_slice(&x.to_le_bytes());
    }
    queue.write_buffer(&params_b, 0, &b);
    Buffers {
        items: items_b,
        pos: pos_b,
        vel: vel_b,
        press: press_b,
        pos_c: mk("pos_c", (np * 12) as u64),
        vel_c: mk("vel_c", (np * 12) as u64),
        press_c: mk("press_c", (np * 4) as u64),
        out_c: mk("out_c", (np * 24) as u64),
        out: mk("out", (np * 24) as u64),
        params: params_b,
    }
}

/// 两个入口 + 两个 bind group（一张布局：0 uniform、1..4 只读、5..7 读写）。
struct Pipes {
    gather: wgpu::ComputePipeline,
    scatter: wgpu::ComputePipeline,
    bg_gather: wgpu::BindGroup,
    bg_scatter: wgpu::BindGroup,
}

fn make_pipes(device: &wgpu::Device, b: &Buffers) -> Pipes {
    let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..8u32)
        .map(|binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if binding == 0 {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: binding <= 4,
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("reorder.bgl"),
        entries: &entries,
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("reorder.pl"),
        bind_group_layouts: &[&bgl],
        push_constant_ranges: &[],
    });
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("reorder.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("reorder.wgsl").into()),
    });
    let pipe = |label: &str, entry: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
        wgpu::BindGroupEntry {
            binding,
            resource: b.as_entire_binding(),
        }
    }
    let bg = |label: &str, e: &[wgpu::BindGroupEntry<'_>]| {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(label),
            layout: &bgl,
            entries: e,
        })
    };
    let bg_gather = bg(
        "reorder.bg.gather",
        &[
            ent(0, &b.params),
            ent(1, &b.items),
            ent(2, &b.pos),
            ent(3, &b.vel),
            ent(4, &b.press),
            ent(5, &b.pos_c),
            ent(6, &b.vel_c),
            ent(7, &b.press_c),
        ],
    );
    // scatter：2 out_c → 5 out（3/4/6/7 本入口不用，但布局里在 ⇒ 绑同尺寸占位）
    let bg_scatter = bg(
        "reorder.bg.scatter",
        &[
            ent(0, &b.params),
            ent(1, &b.items),
            ent(2, &b.out_c),
            ent(3, &b.vel),
            ent(4, &b.press),
            ent(5, &b.out),
            ent(6, &b.vel_c),
            ent(7, &b.press_c),
        ],
    );
    Pipes {
        gather: pipe("reorder.gather", "gather"),
        scatter: pipe("reorder.scatter", "scatter"),
        bg_gather,
        bg_scatter,
    }
}

/// 计时/金丝雀的公共上下文（包起来免得 8 参以上）。
struct Runner<'a> {
    device: &'a wgpu::Device,
    queue: &'a wgpu::Queue,
    gx: u32,
    gy: u32,
    r1: usize,
    r2: usize,
}

impl Runner<'_> {
    fn wait(&self) {
        self.device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
    }

    /// **显式预热一次、丢弃计时**：`write_buffer` 的上传在下一个提交里落地 ⇒ 不预热会把上传
    /// （10M 档 320 MB）算进第一次计时，两档读数会反过来、解出负 `R`。
    fn warmup(&self, pipes: &Pipes) {
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("reorder.warmup"),
            });
        for (pipeline, bg) in [
            (&pipes.gather, &pipes.bg_gather),
            (&pipes.scatter, &pipes.bg_scatter),
        ] {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("reorder.warmup.pass"),
                timestamp_writes: None,
            });
            cp.set_pipeline(pipeline);
            cp.set_bind_group(0, bg, &[]);
            cp.dispatch_workgroups(self.gx, self.gy, 1);
        }
        self.queue.submit(Some(enc.finish()));
        self.wait();
    }

    /// 一档计时 + 解 `t = c + R/r`。
    fn time(&self, name: &str, pipeline: &wgpu::ComputePipeline, bg: &wgpu::BindGroup) -> Meas {
        let mut ps: Vec<f64> = Vec::new();
        for reps in [self.r1, self.r2] {
            let t = std::time::Instant::now();
            let mut enc = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("reorder.enc"),
                });
            for _ in 0..reps {
                let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("reorder.pass"),
                    timestamp_writes: None,
                });
                cp.set_pipeline(pipeline);
                cp.set_bind_group(0, bg, &[]);
                cp.dispatch_workgroups(self.gx, self.gy, 1);
            }
            self.queue.submit(Some(enc.finish()));
            self.wait();
            ps.push(t.elapsed().as_secs_f64() * 1e3 / reps as f64);
        }
        let (c, r, ok) = solve(self.r1, ps[0], self.r2, ps[1]);
        println!(
            "  · {name:<8} r={} → {:.2} | r={} → {:.2} ms ⇒ c = {c:.2} ms{}，R = {r:.2} ms",
            self.r1,
            ps[0],
            self.r2,
            ps[1],
            if ok { "" } else { "（R 不可解 †）" }
        );
        Meas { c, r, ok }
    }

    /// 金丝雀：抽样校验 `x_c[m] == x[items[m]]`（返回 `(抽样分量数, 不符数)`）。
    fn canary(&self, pos_c: &wgpu::Buffer, items: &[u32], pos: &[f32]) -> (usize, usize) {
        let k = 4096usize;
        let rb = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("reorder.readback"),
            size: (k * 4) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(pos_c, 0, &rb, 0, (k * 4) as u64);
        self.queue.submit(Some(enc.finish()));
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.wait();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        let mut bad = 0usize;
        for (m, it) in items.iter().take(k / 3).enumerate() {
            let ki = *it as usize;
            for c in 0..3 {
                let o = (m * 3 + c) * 4;
                let got = f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
                if got.to_bits() != pos[ki * 3 + c].to_bits() {
                    bad += 1;
                }
            }
        }
        drop(data);
        rb.unmap();
        (k, bad)
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let rest: Vec<String> = args.collect();
    let get = |key: &str, dflt: usize| -> usize {
        rest.iter()
            .position(|a| a == key)
            .and_then(|k| rest.get(k + 1))
            .and_then(|v| v.parse().ok())
            .unwrap_or(dflt)
    };
    let (cpu_threads, r1, r2) = (get("--cpu-threads", 1), get("--r1", 8), get("--r2", 32));

    let (adapter, device, queue) = match probe::device_for(0) {
        Ok(v) => v,
        Err(e) => {
            println!("GPU 路径不可用：{e}");
            return;
        }
    };
    println!("适配器：{adapter}");

    let f = build_scene(n, cpu_threads);
    let np = f.len();
    let items: Vec<u32> = if rest.iter().any(|a| a == "--shuffle") {
        println!("（`--shuffle`：置换换成随机置换 ⇒ 搬运成本的**上界档**）");
        shuffle(np)
    } else {
        f.neighbor_grid().items.to_vec()
    };
    let (pos, vel) = flatten(&f);
    let press: Vec<f32> = f.pressures().to_vec();
    println!(
        "== 重排成本（{np} 粒；`gather` 搬 pos/vel/press = 28 B/粒，`scatter` 搬 acc+xsph = 24 B/粒）=="
    );

    let bufs = make_buffers(&device, &queue, np, &items, &pos, &vel, &press);
    let pipes = make_pipes(&device, &bufs);
    let (gx, gy) = probe::split_2d((np as u32).div_ceil(64u32));
    let runner = Runner {
        device: &device,
        queue: &queue,
        gx,
        gy,
        r1,
        r2,
    };
    runner.warmup(&pipes);
    let g = runner.time("gather", &pipes.gather, &pipes.bg_gather);
    let s = runner.time("scatter", &pipes.scatter, &pipes.bg_scatter);

    let (k, bad) = runner.canary(&bufs.pos_c, &items, &pos);
    println!(
        "  金丝雀：`pos_c[m] == pos[items[m]]` 抽样 {} 个分量不符 {}",
        k - bad,
        if bad == 0 {
            "✅"
        } else {
            "❌ 接线错，读数作废"
        }
    );

    // —— 对账：毛收益 = §21.2 的 natural − sorted = 44.49 − 28.49 = 16.00 ms/子步 ——
    let gross = 16.00f64;
    println!("\n== 对账（10M 档口径）==");
    println!("  毛收益（两相位 natural − sorted）    ：{gross:.2} ms/子步");
    println!(
        "  每子步都搬（gather + scatter）       ：{:.2} ms/子步 ⇒ 净 {:.2}",
        g.c + s.c,
        gross - g.c - s.c
    );
    println!(
        "  只在重排时搬（gather，摊销到 N 子步）：{:.2}/N ms/子步",
        g.c
    );
    println!(
        "  每 tick 回读一次才搬（scatter）      ：{:.2} ms/tick",
        s.c
    );
    for (name, m) in [("gather", &g), ("scatter", &s)] {
        println!("  [{name}] c={:.2} R={:.2} ok={}", m.c, m.r, m.ok);
    }
}
