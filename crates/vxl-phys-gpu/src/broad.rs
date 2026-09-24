//! **均匀格宽相（卡上）**：与 `vxl-phys-broad/src/grid_phase.rs::GridBroadPhase` **同规则**
//! （见 `broad.wgsl` 的档头），判据 = **配对集合逐位相同**（探针 `gpu_broad_probe`）。
//!
//! 为什么这一片选宽相（`PLAN-gpu.md` §17.2）：① 负载下占比第二大（29%）；② 只吃**逐体 AABB**
//! （主机用同一份 `shape_aabb` 算好 ⇒ 与 CPU 同数据）+ 整数格坐标 + f32 比较 ⇒ **全是精确运算
//! ⇒ 可以逐位可复现**，判据比"口径 B 容差"强一档；③ 回读只有配对表（8 B/对）。
//!
//! **与 CPU 的两处有意差别**（都不影响判据）：稠密网格 vs 稀疏 HashMap；格内条目序不定
//! （最终 `sort+dedup` ⇒ 输出与序无关）。
//!
//! **本档是显式档**：不接进 `World` 的默认路径（默认档仍是 `BvhBroadPhase`）——接线的形状见 §17.3。

use crate::probe::device_for;

const WG: u32 = 64;
/// 稠密网格格数上限（超了直接报错回退；CPU 的稀疏表没这个限制）。
const MAX_CELLS: u64 = 1 << 22;

/// 卡上宽相读数。
pub struct BroadOut {
    pub adapter: String,
    /// 非空 = 不可用（无适配器/超出本档护栏），调用方按"回退 CPU"处理。
    pub error: Option<String>,
    /// 归一化（`a < b`）有序对，已 `sort_unstable + dedup`（**与 CPU 的收尾同操作**）。
    pub pairs: Vec<(u32, u32)>,
    /// 稠密网格格数（诊断）。
    pub cells: u32,
    /// >0 ⇒ 护栏触发（条目或配对超容量）⇒ **本帧不可信**，调用方不得使用 `pairs`。
    pub overflow: u32,
}

impl BroadOut {
    fn err(msg: String) -> Self {
        Self {
            adapter: String::new(),
            error: Some(msg),
            pairs: Vec::new(),
            cells: 0,
            overflow: 0,
        }
    }
}

/// 主机侧的格规则（**与 CPU `GridBroadPhase::cell_of` 逐字同式**）。本档只用它定稠密网格的
/// 范围与容量（判定用的那份在核里、同表达式）；范围取"全局 AABB 的格范围"⇒ 是所有体覆盖格的
/// **超集**（多出来的空格无影响），因此这里与核里的差一两个格也不会改变结果。
fn cell_of(v: f32, inv: f32) -> i32 {
    (v * inv).floor().clamp(-1_000_000.0, 1_000_000.0) as i32
}

/// 稠密网格（主机侧算出、与核同规则）。
struct Grid {
    lo: [i32; 3],
    dims: [u32; 3],
    cells: u32,
    /// 条目容量 = Σ 每体覆盖格数 + 64（超了会被核里的 `overflow` 抓到）。
    items_cap: u32,
    inv: f32,
}

/// 全局 AABB 的格范围 ⇒ 网格三件套 + 容量；超上限返回 `Err`（调用方回退 CPU）。
fn plan(boxes: &[f32], n: u32, cell_size: f32) -> Result<Grid, String> {
    if cell_size <= 1e-4 {
        return Err(format!("cell_size={cell_size} 太小（CPU 会退回 1.0）"));
    }
    let inv = 1.0f32 / cell_size;
    let (mut lo, mut hi) = ([i32::MAX; 3], [i32::MIN; 3]);
    let mut spans: u64 = 0;
    for i in 0..n as usize {
        let o = i * 8;
        let (bmin, bmax) = (&boxes[o..o + 3], &boxes[o + 4..o + 7]);
        let cmin = [
            cell_of(bmin[0], inv),
            cell_of(bmin[1], inv),
            cell_of(bmin[2], inv),
        ];
        let cmax = [
            cell_of(bmax[0], inv),
            cell_of(bmax[1], inv),
            cell_of(bmax[2], inv),
        ];
        for a in 0..3 {
            lo[a] = lo[a].min(cmin[a]);
            hi[a] = hi[a].max(cmax[a]);
        }
        spans += ((cmax[0] - cmin[0] + 1) as u64)
            * ((cmax[1] - cmin[1] + 1) as u64)
            * ((cmax[2] - cmin[2] + 1) as u64);
    }
    let dims = [
        (hi[0] - lo[0] + 1) as u32,
        (hi[1] - lo[1] + 1) as u32,
        (hi[2] - lo[2] + 1) as u32,
    ];
    let cells = dims[0] as u64 * dims[1] as u64 * dims[2] as u64;
    if cells > MAX_CELLS {
        return Err(format!("稠密网格 {cells} 格超上限 {MAX_CELLS}（回退 CPU）"));
    }
    Ok(Grid {
        lo,
        dims,
        cells: cells as u32,
        items_cap: (spans as u32).saturating_add(64),
        inv,
    })
}

/// 全部缓冲 + uniform 参数（`boxes` 与 params 在这里就写进去）。
struct Bufs {
    params_b: wgpu::Buffer,
    boxes_b: wgpu::Buffer,
    counts_b: wgpu::Buffer,
    start_b: wgpu::Buffer,
    cursor_b: wgpu::Buffer,
    items_b: wgpu::Buffer,
    pairs_b: wgpu::Buffer,
    pcnt_b: wgpu::Buffer,
    ovf_b: wgpu::Buffer,
}

fn make_bufs(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    boxes: &[f32],
    n: u32,
    g: &Grid,
    cap_pairs: u32,
) -> Bufs {
    let mk = |label: &str, size: u64, extra: wgpu::BufferUsages| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: size.max(4),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC
                | extra,
            mapped_at_creation: false,
        })
    };
    let plain = wgpu::BufferUsages::empty();
    let boxes_b = mk("broad.boxes", (n as u64) * 32, plain);
    let bytes: Vec<u8> = boxes[..n as usize * 8]
        .iter()
        .flat_map(|x| x.to_le_bytes())
        .collect();
    queue.write_buffer(&boxes_b, 0, &bytes);
    let params_b = mk("broad.params", 48, wgpu::BufferUsages::UNIFORM);
    // 48 B：`cell_min(i32×3) | n | dims(u32×3) | total | inv_cell | cap_pairs | pad×2`
    let mut p: Vec<u8> = Vec::with_capacity(48);
    for x in g.lo {
        p.extend_from_slice(&x.to_le_bytes());
    }
    p.extend_from_slice(&n.to_le_bytes());
    for x in g.dims {
        p.extend_from_slice(&x.to_le_bytes());
    }
    p.extend_from_slice(&g.cells.to_le_bytes());
    p.extend_from_slice(&g.inv.to_le_bytes());
    p.extend_from_slice(&cap_pairs.to_le_bytes());
    p.extend_from_slice(&[0u8; 8]);
    queue.write_buffer(&params_b, 0, &p);
    Bufs {
        params_b,
        boxes_b,
        counts_b: mk("broad.counts", (g.cells as u64) * 4, plain),
        start_b: mk("broad.start", ((g.cells + 1) as u64) * 4, plain),
        cursor_b: mk("broad.cursor", (g.cells as u64) * 4, plain),
        items_b: mk("broad.items", (g.items_cap as u64) * 4, plain),
        pairs_b: mk("broad.pairs", (cap_pairs as u64) * 8, plain),
        pcnt_b: mk("broad.pcnt", 4, plain),
        ovf_b: mk("broad.ovf", 4, plain),
    }
}

/// 布局 + 管线 + bind group（+ 回读缓冲）。
struct Parts {
    bg: wgpu::BindGroup,
    pipes: [wgpu::ComputePipeline; 4],
    bufs: Bufs,
    rb: wgpu::Buffer,
}

fn build(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    boxes: &[f32],
    n: u32,
    g: &Grid,
    cap_pairs: u32,
) -> Parts {
    let bufs = make_bufs(device, queue, boxes, n, g, cap_pairs);
    let ro = wgpu::BufferBindingType::Storage { read_only: true };
    let rw = wgpu::BufferBindingType::Storage { read_only: false };
    let types = [
        wgpu::BufferBindingType::Uniform,
        ro,
        rw,
        rw,
        rw,
        rw,
        rw,
        rw,
        rw,
    ];
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("broad.bgl"),
        entries: &types
            .iter()
            .enumerate()
            .map(|(b, ty)| wgpu::BindGroupLayoutEntry {
                binding: b as u32,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: *ty,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>(),
    });
    // ⚠️ 这个变量**必须叫 `shader`**：本仓的词汇门只豁免"计算管线描述符里那一字段取形参 `shader`"
    // 这一种写法（见 `scripts/vocab_scan.sh` 头部）——改名会红。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("broad.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("broad.wgsl").into()),
    });
    let mk = |entry: &str| {
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("broad.pl"),
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let pipes = [mk("bin_count"), mk("scan"), mk("place"), mk("gen_pairs")];
    let bg = {
        fn e<'a>(b: u32, bf: &'a wgpu::Buffer) -> wgpu::BindGroupEntry<'a> {
            wgpu::BindGroupEntry {
                binding: b,
                resource: bf.as_entire_binding(),
            }
        }
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("broad.bg"),
            layout: &layout,
            entries: &[
                e(0, &bufs.params_b),
                e(1, &bufs.boxes_b),
                e(2, &bufs.counts_b),
                e(3, &bufs.start_b),
                e(4, &bufs.cursor_b),
                e(5, &bufs.items_b),
                e(6, &bufs.pairs_b),
                e(7, &bufs.pcnt_b),
                e(8, &bufs.ovf_b),
            ],
        })
    };
    let rb = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("broad.rb"),
        size: (cap_pairs as u64) * 8 + 8,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    Parts {
        bg,
        pipes,
        bufs,
        rb,
    }
}

/// 一趟提交：清零 → 四遍 → 回读；返回 `(配对表（已截断+排序+去重）, overflow)`。
fn run(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    p: &Parts,
    n: u32,
    cap_pairs: u32,
) -> (Vec<(u32, u32)>, u32) {
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("broad.enc"),
    });
    enc.clear_buffer(&p.bufs.counts_b, 0, None);
    enc.clear_buffer(&p.bufs.cursor_b, 0, None);
    enc.clear_buffer(&p.bufs.pcnt_b, 0, None);
    enc.clear_buffer(&p.bufs.ovf_b, 0, None);
    let ng = n.div_ceil(WG).max(1);
    for (pipe, groups) in p.pipes.iter().zip([ng, 1, ng, ng]) {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: None,
            timestamp_writes: None,
        });
        cp.set_pipeline(pipe);
        cp.set_bind_group(0, &p.bg, &[]);
        cp.dispatch_workgroups(groups, 1, 1);
    }
    enc.copy_buffer_to_buffer(&p.bufs.pairs_b, 0, &p.rb, 0, (cap_pairs as u64) * 8);
    enc.copy_buffer_to_buffer(&p.bufs.pcnt_b, 0, &p.rb, (cap_pairs as u64) * 8, 4);
    enc.copy_buffer_to_buffer(&p.bufs.ovf_b, 0, &p.rb, (cap_pairs as u64) * 8 + 4, 4);
    queue.submit(Some(enc.finish()));
    let slice = p.rb.slice(..);
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
    let f = |o: usize| u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
    let cap = cap_pairs as usize;
    let cnt = (f(cap * 8) as usize).min(cap);
    let overflow = f(cap * 8 + 4);
    let mut pairs: Vec<(u32, u32)> = (0..cnt).map(|k| (f(k * 8), f(k * 8 + 4))).collect();
    drop(data);
    p.rb.unmap();
    // 与 CPU 的收尾**同操作**（`sort_unstable + dedup`）⇒ 格内条目序不定也不漏到结果里。
    pairs.sort_unstable();
    pairs.dedup();
    (pairs, overflow)
}

/// **在指定适配器上跑一趟宽相**。`boxes` = 逐体 AABB（**8 个 f32/体**：`min.xyz | pad | max.xyz | dyn`，
/// `dyn` = `inv_mass > 0` 的 0/1，与 CPU 的 `BodySet::is_dynamic` 同一判据）。
pub fn broad_on_adapter(
    adapter_index: usize,
    boxes: &[f32],
    n: u32,
    cell_size: f32,
    cap_pairs: u32,
) -> BroadOut {
    if n == 0 || boxes.len() < (n as usize) * 8 {
        return BroadOut::err(format!(
            "输入不合法：n={n}、boxes={}（要 ≥ {}）",
            boxes.len(),
            n as usize * 8
        ));
    }
    let g = match plan(boxes, n, cell_size) {
        Ok(g) => g,
        Err(e) => return BroadOut::err(e),
    };
    let (adapter, device, queue) = match device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => return BroadOut::err(e),
    };
    let cells = g.cells;
    let parts = build(&device, &queue, boxes, n, &g, cap_pairs);
    let (pairs, overflow) = run(&device, &queue, &parts, n, cap_pairs);
    BroadOut {
        adapter,
        error: None,
        pairs,
        cells,
        overflow,
    }
}
