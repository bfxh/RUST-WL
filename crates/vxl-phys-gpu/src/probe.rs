//! **GPU 度相位探针**（首里程碑第一片：整条计算管线跑通 + 与 CPU 对拍）。
//!
//! 纪律（`docs/PLAN-gpu.md` §4/§6）：
//! - **口径 A 优先**：GPU 内核与 CPU **同式、同遍历序**（邻域用 CPU 侧那张计数排序表），
//!   目标是**逐位一致**；做不到时退到口径 B（量化哈希 + 容差）并在档里逐相位写明。
//! - **不绑厂商**：用 `enumerate_adapters` 枚举全部后端，任何适配器（含 Intel iGPU）跑同一份内核。
//! - **零额外依赖**：不等 `pollster`（自带 20 行 `block_on`）、不用 `bytemuck`（手写 f32/u32 字节化）。
//! - **本仓纪律**：`src/**` 不许出现双精度浮点（`scripts/discipline_scan.sh` 按字面量扫）⇒ 本文件全单精度。

use wgpu::util::DeviceExt;

/// 与 `density.wgsl` 的 `Params` **逐字节对应**（48 B，16 B 对齐）。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DensityParams {
    pub gmin: [f32; 3],
    pub inv: f32,
    pub h2: f32,
    pub k6: f32,
    pub w0: f32,
    pub mass: f32,
    pub n_fluid: u32,
    pub nx: u32,
    pub ny: u32,
    pub nz: u32,
}

/// **极简 `block_on`**（不引 `pollster`）：wgpu 的 `request_device` 在原生后端是
/// "立即就绪或一次性等待" ⇒ 用标准库的**空 waker**（`Waker::noop()`，stable）轮询即可。
fn block_on<F: std::future::Future>(fut: F) -> F::Output {
    use std::task::{Context, Poll, Waker};
    let mut cx = Context::from_waker(Waker::noop());
    let mut fut = std::pin::pin!(fut);
    loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(v) => return v,
            Poll::Pending => std::thread::yield_now(),
        }
    }
}

/// 适配器清单（名字 / 后端 / 类型）——用于"不绑厂商"核对（Intel iGPU 也要在列）。
pub fn adapters() -> Vec<String> {
    let instance = wgpu::Instance::default();
    instance
        .enumerate_adapters(wgpu::Backends::all())
        .into_iter()
        .map(|a| {
            let info = a.get_info();
            format!(
                "{} | {:?} | {:?}",
                info.name, info.backend, info.device_type
            )
        })
        .collect()
}

/// 一次密度相位的 GPU 运行结果。
pub struct DensityOut {
    pub dens: Vec<f32>,
    /// **一次性**成本（冷启动）：设备/缓冲/管线创建 + 上传 + 回读。
    pub setup_ms: f32,
    /// **稳态**每轮成本：只含"分派 + 提交"（缓冲与管线复用），`repeats` 次均值。
    pub per_dispatch_ms: f32,
    pub adapter: String,
    /// 若指定了适配器序号而枚举为空 ⇒ 返回错误字符串（CI 无 GPU 时的正常路径）。
    pub error: Option<String>,
}

/// 在**指定适配器序号**上跑一遍密度相位（`pos` 为扁平 xyz，`pmass` 逐粒）。
/// `cell_start` / `cell_items` = CPU 侧计数排序网格（保证与 CPU 同遍历序）。
#[allow(clippy::too_many_arguments)]
pub fn density_on_adapter(
    adapter_index: usize,
    pos_flat: &[f32],
    pmass: &[f32],
    cell_start: &[u32],
    cell_items: &[u32],
    params: DensityParams,
    n: usize,
    repeats: usize,
) -> DensityOut {
    let t0 = std::time::Instant::now();
    let instance = wgpu::Instance::default();
    let list = instance.enumerate_adapters(wgpu::Backends::all());
    let Some(adapter) = list.into_iter().nth(adapter_index) else {
        return DensityOut {
            dens: Vec::new(),
            setup_ms: 0.0,
            per_dispatch_ms: 0.0,
            adapter: String::new(),
            error: Some(format!(
                "没有第 {adapter_index} 个适配器（本机无 GPU 或后端不可用）"
            )),
        };
    };
    let info = adapter.get_info();
    let adapter_name = format!(
        "{} | {:?} | {:?}",
        info.name, info.backend, info.device_type
    );
    let (device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
        .expect("request_device");

    let bytes_f32 = |v: &[f32]| -> Vec<u8> {
        let mut b = Vec::with_capacity(v.len() * 4);
        for x in v {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let bytes_u32 = |v: &[u32]| -> Vec<u8> {
        let mut b = Vec::with_capacity(v.len() * 4);
        for x in v {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let params_bytes = {
        let mut b = Vec::with_capacity(48);
        for x in params.gmin {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in [params.inv, params.h2, params.k6, params.w0, params.mass] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in [params.n_fluid, params.nx, params.ny, params.nz] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let storage = |label: &str, data: &[u8], read_only: bool| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage: if read_only {
                wgpu::BufferUsages::STORAGE
            } else {
                wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC
            },
        })
    };
    let pos_b = storage("pos", &bytes_f32(pos_flat), true);
    let pmass_b = storage("pmass", &bytes_f32(pmass), true);
    let start_b = storage("cell_start", &bytes_u32(cell_start), true);
    let items_b = storage("cell_items", &bytes_u32(cell_items), true);
    let dens_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("dens"),
        size: (n * 4) as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let params_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("params"),
        contents: &params_bytes,
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (n * 4) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("density.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("density.wgsl").into()),
    });
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("density.bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("density"),
        layout: Some(
            &device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("density.pl"),
                bind_group_layouts: &[&layout],
                push_constant_ranges: &[],
            }),
        ),
        module: &shader,
        entry_point: Some("density"),
        compilation_options: Default::default(),
        cache: None,
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("density.bg"),
        layout: &layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: params_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: pos_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: pmass_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: start_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: items_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: dens_b.as_entire_binding(),
            },
        ],
    });
    // **稳态**：反复提交"分派"（缓冲/管线复用），**计时含最后一次同步读回**
    // ⚠️ 只给 `queue.submit` 计时是错的（提交是**异步**的：实测 125k 粒反比 64k "快"，
    //    那只是入队成本）⇒ 必须让"提交 + 同步读回"一起计时才等于**核执行**成本。
    let setup_ms = (t0.elapsed().as_secs_f64() * 1e3) as f32;
    let reps = repeats.max(1);
    let t_disp = std::time::Instant::now();
    for _ in 0..reps {
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("density.enc"),
        });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("density.pass"),
                timestamp_writes: None,
            });
            cp.set_pipeline(&pipeline);
            cp.set_bind_group(0, &bind, &[]);
            let wg = 64u32;
            cp.dispatch_workgroups((n as u32).div_ceil(wg), 1, 1);
        }
        queue.submit(Some(enc.finish()));
    }
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("density.readback"),
    });
    enc.copy_buffer_to_buffer(&dens_b, 0, &readback, 0, (n * 4) as u64);
    queue.submit(Some(enc.finish()));

    let slice = readback.slice(..);
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
    // 同步完成点：从第一次提交到最后一次读回映射返回 ⇒ 这才是 reps 轮的**真实**耗时
    // （`queue.submit` 是异步的，只给它计时会把入队当执行）。
    let per_dispatch_ms = (t_disp.elapsed().as_secs_f64() * 1e3) as f32 / reps as f32;
    let data = slice.get_mapped_range();
    let mut dens = Vec::with_capacity(n);
    for c in data.chunks_exact(4) {
        dens.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
    }
    drop(data);
    readback.unmap();
    DensityOut {
        dens,
        setup_ms,
        per_dispatch_ms,
        adapter: adapter_name,
        error: None,
    }
}
