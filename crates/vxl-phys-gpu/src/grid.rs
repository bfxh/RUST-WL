//! **GPU 网格重建探针**：把 CPU 的计数排序原样搬到卡上，判据 = **与 CPU 逐位同表**
//! （`start` / `items` 两数组全等；`grid.wgsl` 头注写了四个入口与 CPU 四步的对应关系）。
//!
//! 纪律（同 `probe.rs`）：
//! - **口径 A 用在网格上是可达的**：这里没有浮点累加，只有"一次减、一次乘、一次 floor"的
//!   整数分箱 + 整数前缀和 + 整数排序 ⇒ 与 CPU 的位级一致**不依赖** FMA 收缩那类运气；
//! - **不绑厂商**：同一份 WGSL 在任何适配器上跑（含 Intel iGPU）；
//! - **两个测量坑**照旧：提交是异步的（计时必须含同步回读）、一次性 setup 与稳态分开报。

use wgpu::util::DeviceExt;

/// 与 `grid.wgsl` 的 `GridParams` **逐字节对应**。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GridParams {
    pub gmin: [f32; 3],
    pub inv: f32,
    pub nx: u32,
    pub ny: u32,
    pub nz: u32,
    pub n: u32,
    pub total: u32,
    /// 逐格规范化（插入排序）的**段长上限**：超过则不规范化并记 `overflow`（最坏情况护栏）。
    pub cap: u32,
    pub _pad0: u32,
    pub _pad1: u32,
}

/// 输入：位置（扁平 xyz）+ 同一套箱子参数（`min`/`inv`/`dims` 由调用方给，
/// 与 CPU `rebuild` 自己算出来的那组**完全相同**才能谈同表——箱子计算是另一件事）。
pub struct GridInputs<'a> {
    pub pos_flat: &'a [f32],
}

/// GPU 网格表 + 读数。
pub struct GridOut {
    /// 每格起点（长度 `total + 1`，与 CPU `start` 同义）。
    pub start: Vec<u32>,
    /// 按格分组的粒子索引（长度 `n`；格内 = **粒子索引升序**）。
    pub items: Vec<u32>,
    /// 逐粒的格线性下标（诊断用：CPU 侧可从 `start/items` 反推同一张表）。
    pub bins: Vec<u32>,
    /// 未规范化的格数（**判据是 0**；非 0 ⇒ 表不再与 CPU 同表）。
    pub overflow: u32,
    pub setup_ms: f32,
    pub per_run_ms: f32,
    pub adapter: String,
    pub error: Option<String>,
}

impl GridOut {
    fn err(msg: String) -> Self {
        Self {
            start: Vec::new(),
            items: Vec::new(),
            bins: Vec::new(),
            overflow: 0,
            setup_ms: 0.0,
            per_run_ms: 0.0,
            adapter: String::new(),
            error: Some(msg),
        }
    }
}

/// 网格探针的缓冲 + 回读偏移（`grid_on_adapter` 第一段）。
pub(crate) struct GridBuffs {
    pub pos_b: wgpu::Buffer,
    pub params_b: wgpu::Buffer,
    pub bins_b: wgpu::Buffer,
    pub counts_b: wgpu::Buffer,
    pub start_b: wgpu::Buffer,
    pub items_b: wgpu::Buffer,
    pub cursor_b: wgpu::Buffer,
    pub overflow_b: wgpu::Buffer,
    pub readback: wgpu::Buffer,
    pub start_off: u64,
    pub items_off: u64,
    pub bins_off: u64,
    pub of_off: u64,
}

pub(crate) fn make_grid_buffs(
    device: &wgpu::Device,
    n: usize,
    total: usize,
    params: GridParams,
    inputs: &GridInputs<'_>,
) -> GridBuffs {
    let params_bytes = {
        let mut b = Vec::with_capacity(48);
        for x in params.gmin {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b.extend_from_slice(&params.inv.to_le_bytes());
        for x in [
            params.nx,
            params.ny,
            params.nz,
            params.n,
            params.total,
            params.cap,
            params._pad0,
            params._pad1,
        ] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let mut pos_bytes = Vec::with_capacity(inputs.pos_flat.len() * 4);
    for x in inputs.pos_flat {
        pos_bytes.extend_from_slice(&x.to_le_bytes());
    }

    let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("grid.pos"),
        contents: &pos_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });
    let params_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("grid.params"),
        contents: &params_bytes,
        usage: wgpu::BufferUsages::UNIFORM,
    });
    // 需要 `clear_buffer` / `copy_buffer_to_buffer` ⇒ 一律带 COPY_DST；结果要回读 ⇒ 带 COPY_SRC。
    let mk = |label: &str, size: u64| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let bins_b = mk("grid.bins", (n * 4) as u64);
    let counts_b = mk("grid.counts", ((total + 1) * 4) as u64);
    let start_b = mk("grid.start", ((total + 1) * 4) as u64);
    let items_b = mk("grid.items", (n * 4) as u64);
    let cursor_b = mk("grid.cursor", ((total + 1) * 4) as u64);
    let overflow_b = mk("grid.overflow", 4);
    // 回读：start | items | bins | overflow（每轮尾同步回读一次，只为校验）
    let start_off = 0u64;
    let items_off = start_off + ((total + 1) * 4) as u64;
    let bins_off = items_off + (n * 4) as u64;
    let of_off = bins_off + (n * 4) as u64;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("grid.readback"),
        size: of_off + 4,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    GridBuffs {
        pos_b,
        params_b,
        bins_b,
        counts_b,
        start_b,
        items_b,
        cursor_b,
        overflow_b,
        readback,
        start_off,
        items_off,
        bins_off,
        of_off,
    }
}

/// 网格四相位管线 + 绑定组（一次性 setup 的产物；`submit_grid_pass` 里只读引用）。
pub(crate) struct GridPipes {
    bin: wgpu::ComputePipeline,
    scan: wgpu::ComputePipeline,
    place: wgpu::ComputePipeline,
    canon: wgpu::ComputePipeline,
    bg: wgpu::BindGroup,
}

pub(crate) fn make_grid_pipes(device: &wgpu::Device, bufs: &GridBuffs) -> GridPipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("grid.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("grid.wgsl").into()),
    });
    // 一份布局覆盖四个入口（除 `pos` 外全部 read_write；见 `grid.wgsl` 头注）
    let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..8u32)
        .map(|binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if binding == 0 {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: binding == 1,
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("grid.bgl"),
        entries: &entries,
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("grid.pl"),
        bind_group_layouts: &[&bgl],
        push_constant_ranges: &[],
    });
    let mk_pipe = |label: &str, entry: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let p_bin = mk_pipe("grid.bin_count", "bin_count");
    let p_scan = mk_pipe("grid.scan", "scan");
    let p_place = mk_pipe("grid.place", "place");
    let p_canon = mk_pipe("grid.canon", "canon");
    fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
        wgpu::BindGroupEntry {
            binding,
            resource: b.as_entire_binding(),
        }
    }
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("grid.bg"),
        layout: &bgl,
        entries: &[
            ent(0, &bufs.params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.bins_b),
            ent(3, &bufs.counts_b),
            ent(4, &bufs.start_b),
            ent(5, &bufs.items_b),
            ent(6, &bufs.cursor_b),
            ent(7, &bufs.overflow_b),
        ],
    });

    GridPipes {
        bin: p_bin,
        scan: p_scan,
        place: p_place,
        canon: p_canon,
        bg,
    }
}

pub(crate) fn parse_grid(
    n: usize,
    total: usize,
    bufs: &GridBuffs,
    data: &[u8],
) -> (Vec<u32>, Vec<u32>, Vec<u32>, u32) {
    let u32_at = |i: usize| -> u32 {
        let c = &data[i * 4..i * 4 + 4];
        u32::from_le_bytes([c[0], c[1], c[2], c[3]])
    };
    let s_base = (bufs.start_off / 4) as usize;
    let i_base = (bufs.items_off / 4) as usize;
    let b_base = (bufs.bins_off / 4) as usize;
    let start: Vec<u32> = (0..total + 1).map(|k| u32_at(s_base + k)).collect();
    let items: Vec<u32> = (0..n).map(|k| u32_at(i_base + k)).collect();
    let bins: Vec<u32> = (0..n).map(|k| u32_at(b_base + k)).collect();
    let overflow = u32_at(bufs.of_off as usize / 4);

    (start, items, bins, overflow)
}

fn submit_grid_pass(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    bufs: &GridBuffs,
    pipes: &GridPipes,
    groups_n: u32,
    groups_total: u32,
    total: usize,
) {
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("grid.enc"),
    });
    // ① 清零（counts / overflow）：`clear_buffer` 是**确定**的（不是"未定义内容"）
    enc.clear_buffer(&bufs.counts_b, 0, None);
    enc.clear_buffer(&bufs.overflow_b, 0, None);
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.bin_count"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.bin);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(groups_n, 1, 1);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.scan"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.scan);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(1, 1, 1);
    }
    enc.copy_buffer_to_buffer(
        &bufs.start_b,
        0,
        &bufs.cursor_b,
        0,
        ((total + 1) * 4) as u64,
    );
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.place"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.place);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(groups_n, 1, 1);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.canon"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.canon);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(groups_total, 1, 1);
    }
    queue.submit(Some(enc.finish()));
}

/// 在**指定适配器序号**上跑「网格重建」（四个入口一条命令链，缓冲/管线复用，`repeats` 轮）。
pub fn grid_on_adapter(
    adapter_index: usize,
    inputs: &GridInputs<'_>,
    params: GridParams,
    repeats: usize,
) -> GridOut {
    let t0 = std::time::Instant::now();
    let (adapter_name, device, queue) = match crate::probe::device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => return GridOut::err(e),
    };
    let n = params.n as usize;
    let total = params.total as usize;

    let bufs = make_grid_buffs(&device, n, total, params, inputs);
    let pipes = make_grid_pipes(&device, &bufs);
    let setup_ms = (t0.elapsed().as_secs_f64() * 1e3) as f32;
    let reps = repeats.max(1);
    let wg = 64u32;
    let groups_n = (n as u32).div_ceil(wg);
    let groups_total = (total as u32).div_ceil(wg);
    let t_run = std::time::Instant::now();
    submit_grid_pass(
        &device,
        &queue,
        &bufs,
        &pipes,
        groups_n,
        groups_total,
        total,
    );
    {
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("grid.bufs.readback"),
        });
        enc.copy_buffer_to_buffer(
            &bufs.start_b,
            0,
            &bufs.readback,
            bufs.start_off,
            ((total + 1) * 4) as u64,
        );
        enc.copy_buffer_to_buffer(
            &bufs.items_b,
            0,
            &bufs.readback,
            bufs.items_off,
            (n * 4) as u64,
        );
        enc.copy_buffer_to_buffer(
            &bufs.bins_b,
            0,
            &bufs.readback,
            bufs.bins_off,
            (n * 4) as u64,
        );
        enc.copy_buffer_to_buffer(&bufs.overflow_b, 0, &bufs.readback, bufs.of_off, 4);
        queue.submit(Some(enc.finish()));
    }
    let slice = bufs.readback.slice(..);
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
    let per_run_ms = (t_run.elapsed().as_secs_f64() * 1e3) as f32 / reps as f32;

    let data = slice.get_mapped_range();
    let (start, items, bins, overflow) = parse_grid(n, total, &bufs, &data);
    // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
    drop(data);
    bufs.readback.unmap();
    GridOut {
        start,
        items,
        bins,
        overflow,
        setup_ms,
        per_run_ms,
        adapter: adapter_name,
        error: None,
    }
}
