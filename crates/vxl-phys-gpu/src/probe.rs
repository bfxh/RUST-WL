//! **GPU 相位探针**（首里程碑：密度 + 力/黏度两相位，与 CPU 对拍）。
//!
//! 纪律（`docs/PLAN-gpu.md` §4/§6）：
//! - **口径 A 优先**：GPU 内核与 CPU **同式、同遍历序**（邻域用 CPU 侧那张计数排序表），
//!   目标是**逐位一致**；首片实测未达 ⇒ 退到口径 B（量化哈希 + 容差），档里逐相位写明。
//! - **不绑厂商**：`enumerate_adapters` 枚举全部后端，任何适配器（含 Intel iGPU）跑同一份内核。
//! - **零额外依赖**：不等 `pollster`（用标准库 `Waker::noop()`）、不引 `bytemuck`（手写字节化）。
//! - **本仓纪律**：`src/**` 不许出现双精度浮点（`scripts/discipline_scan.sh` 按字面量扫）⇒ 全单精度；
//!   常量块大小写法一律避开（CI 的 clippy 1.98 有 `chunks_exact_to_as_chunks`）。
//!
//! **两个测量坑**（首片踩过，勿重犯）：
//! 1. `queue.submit` 是**异步**的 ⇒ 只给它计时会把"入队"当"执行"（实测 125k 竟"快"于 64k）
//!    ⇒ 必须让「提交 + **同步回读**」一起计时；
//! 2. **一次性 setup**（设备/着色器编译/上传/回读）必须与**稳态每轮**分开报。

use wgpu::util::DeviceExt;

/// 与 `density.wgsl` / `force.wgsl` 的 `Params` **逐字节对应**（两核共用一份 = 字段并集）。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PhaseParams {
    pub gmin: [f32; 3],
    pub inv: f32,
    pub h2: f32,
    pub k6: f32,
    pub w0: f32,
    pub mass: f32,
    pub ks: f32,
    pub h: f32,
    pub alpha_c: f32,
    /// WGSL uniform 布局：`vec3` 16 字节对齐 ⇒ `gvec` 前必须补 4 字节（否则参数错位）。
    pub _pad0: f32,
    pub gvec: [f32; 3],
    pub n_fluid: u32,
    pub nx: u32,
    pub ny: u32,
    pub nz: u32,
    pub _pad: u32,
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

/// 输入状态（与 CPU 侧**同一份数组**：扁平 xyz / 逐粒质量 / 逐粒 ρ、p）。
pub struct PhaseInputs<'a> {
    pub pos_flat: &'a [f32],
    pub vel_flat: &'a [f32],
    pub pmass: &'a [f32],
    pub press: &'a [f32],
    pub cell_start: &'a [u32],
    pub cell_items: &'a [u32],
}

/// 两相位（密度 + 力/黏度）在 GPU 上跑一轮的结果。
pub struct PhasesOut {
    /// 密度（`n_fluid` 个）。
    pub dens: Vec<f32>,
    /// 加速度（3 个 f32/粒）。
    pub acc: Vec<f32>,
    /// XSPH 速度平滑增量（3 个 f32/粒）。
    pub xsph: Vec<f32>,
    /// **一次性**成本：设备/缓冲/管线创建 + 上传 + 首个回读（冷启动）。
    pub setup_ms: f32,
    /// **稳态**每轮成本：两核一起分派 + 同步读回，`repeats` 次均值。
    pub per_dispatch_ms: f32,
    pub adapter: String,
    pub error: Option<String>,
}

/// 取第 `adapter_index` 个适配器并建设备（**两探针共用**的选卡路径）。
/// 返回 `(适配器名字, device, queue)`；不可用时给中文错误串（调用方原样报出）。
pub(crate) fn device_for(
    adapter_index: usize,
) -> Result<(String, wgpu::Device, wgpu::Queue), String> {
    let instance = wgpu::Instance::default();
    let list = instance.enumerate_adapters(wgpu::Backends::all());
    let Some(adapter) = list.into_iter().nth(adapter_index) else {
        return Err(format!(
            "没有第 {adapter_index} 个适配器（本机无 GPU 或后端不可用）"
        ));
    };
    let info = adapter.get_info();
    let name = format!(
        "{} | {:?} | {:?}",
        info.name, info.backend, info.device_type
    );
    let (device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
        .map_err(|e| format!("request_device 失败：{e:?}"))?;
    Ok((name, device, queue))
}

/// 在**指定适配器序号**上跑「密度 + 力/黏度」两相位（缓冲/管线复用，`repeats` 轮）。
pub fn phases_on_adapter(
    adapter_index: usize,
    inputs: &PhaseInputs<'_>,
    params: PhaseParams,
    n: usize,
    repeats: usize,
) -> PhasesOut {
    let t0 = std::time::Instant::now();
    let (adapter_name, device, queue) = match device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => {
            return PhasesOut {
                dens: Vec::new(),
                acc: Vec::new(),
                xsph: Vec::new(),
                setup_ms: 0.0,
                per_dispatch_ms: 0.0,
                adapter: String::new(),
                error: Some(e),
            };
        }
    };

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
        let mut b = Vec::with_capacity(64);
        for x in params.gmin {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in [
            params.inv,
            params.h2,
            params.k6,
            params.w0,
            params.mass,
            params.ks,
            params.h,
            params.alpha_c,
            params._pad0,
        ] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in params.gvec {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in [params.n_fluid, params.nx, params.ny, params.nz, params._pad] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let ro = |label: &str, data: &[u8]| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage: wgpu::BufferUsages::STORAGE,
        })
    };
    let rw = |label: &str, size: u64| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        })
    };
    let pos_b = ro("pos", &bytes_f32(inputs.pos_flat));
    let vel_b = ro("vel", &bytes_f32(inputs.vel_flat));
    let pmass_b = ro("pmass", &bytes_f32(inputs.pmass));
    let press_b = ro("press", &bytes_f32(inputs.press));
    let start_b = ro("cell_start", &bytes_u32(inputs.cell_start));
    let items_b = ro("cell_items", &bytes_u32(inputs.cell_items));
    // 7 = dens（密度写 / 力读）；8 = out（acc.xyz + xsph.xyz 交错）
    let dens_b = rw("dens", (n * 4) as u64);
    let out_b = rw("out", (n * 24) as u64);
    let params_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("params"),
        contents: &params_bytes,
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (n * 28) as u64, // dens(4) + acc(12) + xsph(12)
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let dens_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("density.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("density.wgsl").into()),
    });
    let force_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("force.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("force.wgsl").into()),
    });
    // **两核各一张绑定布局**：wgpu 对 storage 的访问权限要求**精确匹配**（实测报错：
    // `Storage { access: LOAD }` 与 `LOAD | STORE` 不匹配）⇒「密度核写 `dens` / 力核只读 `dens`」
    // 不可能共用一张布局。两张布局 + 两个 bind group 都属**一次性 setup**，不进出稳态计时。
    let mk_bgl = |label: &str, list: &[(u32, bool)]| {
        let entries: Vec<wgpu::BindGroupLayoutEntry> = list
            .iter()
            .map(|&(binding, rw)| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if binding == 0 {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage { read_only: !rw }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some(label),
            entries: &entries,
        })
    };
    // 密度核用到的槽位：0 params | 1 pos | 3 pmass | 5 cell_start | 6 cell_items | 7 dens(读写)
    let dens_bgl = mk_bgl(
        "density.bgl",
        &[
            (0, false),
            (1, false),
            (3, false),
            (5, false),
            (6, false),
            (7, true),
        ],
    );
    // 力核用到的槽位：0 | 1 pos | 2 vel | 3 pmass | 4 press | 5 | 6 | 7 dens(只读) | 8 out(读写)
    let force_bgl = mk_bgl(
        "force.bgl",
        &[
            (0, false),
            (1, false),
            (2, false),
            (3, false),
            (4, false),
            (5, false),
            (6, false),
            (7, false),
            (8, true),
        ],
    );
    let mk =
        |label: &str, shader: &wgpu::ShaderModule, entry: &str, bgl: &wgpu::BindGroupLayout| {
            let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some(label),
                bind_group_layouts: &[bgl],
                push_constant_ranges: &[],
            });
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(label),
                layout: Some(&pl),
                module: shader,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
    let p_dens = mk("density", &dens_shader, "density", &dens_bgl);
    let p_force = mk("force", &force_shader, "force", &force_bgl);
    // 嵌套 fn 而非闭包：`BindGroupEntry` 借了缓冲 ⇒ 返回类型里的生命周期必须显式写出
    // （闭包无法标注入参生命周期，会撞 "lifetime may not live long enough"）。
    fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
        wgpu::BindGroupEntry {
            binding,
            resource: b.as_entire_binding(),
        }
    }
    let dens_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("density.bg"),
        layout: &dens_bgl,
        entries: &[
            ent(0, &params_b),
            ent(1, &pos_b),
            ent(3, &pmass_b),
            ent(5, &start_b),
            ent(6, &items_b),
            ent(7, &dens_b),
        ],
    });
    let force_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("force.bg"),
        layout: &force_bgl,
        entries: &[
            ent(0, &params_b),
            ent(1, &pos_b),
            ent(2, &vel_b),
            ent(3, &pmass_b),
            ent(4, &press_b),
            ent(5, &start_b),
            ent(6, &items_b),
            ent(7, &dens_b),
            ent(8, &out_b),
        ],
    });
    // 稳态：两核各分派一次 × reps 轮；计时含最后一次**同步回读**（见文件头"两个测量坑"）。
    let setup_ms = (t0.elapsed().as_secs_f64() * 1e3) as f32;
    let reps = repeats.max(1);
    let groups = (n as u32).div_ceil(64u32);
    let t_disp = std::time::Instant::now();
    for _ in 0..reps {
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("phases.enc"),
        });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("density.pass"),
                timestamp_writes: None,
            });
            cp.set_pipeline(&p_dens);
            cp.set_bind_group(0, &dens_bg, &[]);
            cp.dispatch_workgroups(groups, 1, 1);
        }
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("force.pass"),
                timestamp_writes: None,
            });
            cp.set_pipeline(&p_force);
            cp.set_bind_group(0, &force_bg, &[]);
            cp.dispatch_workgroups(groups, 1, 1);
        }
        queue.submit(Some(enc.finish()));
    }
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("phases.readback"),
    });
    enc.copy_buffer_to_buffer(&dens_b, 0, &readback, 0, (n * 4) as u64);
    enc.copy_buffer_to_buffer(&out_b, 0, &readback, (n * 4) as u64, (n * 24) as u64);
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
    let per_dispatch_ms = (t_disp.elapsed().as_secs_f64() * 1e3) as f32 / reps as f32;
    let data = slice.get_mapped_range();
    let mut dens = Vec::with_capacity(n);
    for i in 0..n {
        let c = &data[i * 4..i * 4 + 4];
        dens.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
    }
    let out_base = n;
    let mut acc = Vec::with_capacity(n * 3);
    let mut xsph = Vec::with_capacity(n * 3);
    for i in 0..n {
        for k in 0..3 {
            let c = &data[(out_base + i * 6 + k) * 4..(out_base + i * 6 + k) * 4 + 4];
            acc.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
        }
        for k in 3..6 {
            let c = &data[(out_base + i * 6 + k) * 4..(out_base + i * 6 + k) * 4 + 4];
            xsph.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
        }
    }
    drop(data);
    readback.unmap();
    PhasesOut {
        dens,
        acc,
        xsph,
        setup_ms,
        per_dispatch_ms,
        adapter: adapter_name,
        error: None,
    }
}
