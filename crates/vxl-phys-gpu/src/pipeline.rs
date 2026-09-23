//! **常驻缓冲的完整 GPU 子步**（`docs/PLAN-gpu.md` §9.4 ③ 的骨料）：一条命令链跑
//! `网格重建 → 密度 → 压力 EOS → 力/黏度 → 半隐式欧拉`，缓冲与管线**只建一次**。
//!
//! 五个相位与 CPU 侧的对应关系（都在这仓里）：
//! | 相位 | WGSL | CPU 对应 |
//! |---|---|---|
//! | 网格 | `grid.wgsl`（四入口） | `UniformGrid::rebuild`（与 CPU **逐位同表**，见 §11） |
//! | 密度 | `density.wgsl` | `density_pass` |
//! | 压力 | `eos.wgsl` | `pressure_pass` |
//! | 力/黏度 | `force.wgsl` | `force_pass` |
//! | 积分 | `integrate.wgsl` | `substep` 末段（半隐式欧拉 + CFL 缩回） |
//!
//! **口径（重要）**：
//! - 浮点相位走**口径 B**（同式不逐位，见 §9.6 的容差表）；**网格相位逐位同表**（全整数）；
//! - **箱子（`min`/`inv`/`dims`）在本片是固定的**：由调用方给一次。真正的后端每 tick 要按
//!   当前包围盒重算（CPU 的"`h` 起、超预算翻倍"规则），那是**另加一个小核**（减 min/max + 单线程
//!   算 dims）——本片先固定，理由是"边界钳位不影响邻域正确性"（钳进边缘格的粒子仍会被 `r ≤ h`
//!   判据刷掉，只是边缘格变挤、变慢），见 §12。
//! - **每 tick 只回读一次**（真要耦合才回读）：稳态数字用"多 tick 只回读一次"量；再单独量一次
//!   "回读+同步"的成本，两者相加才是"含耦合"的口径。

use wgpu::util::DeviceExt;

/// 建管线所需的静态参数（一次给全；运行时只有 `dt` 会变）。
#[derive(Clone, Copy, Debug)]
pub struct PacketCfg {
    pub n: u32,
    pub total: u32,
    pub gmin: [f32; 3],
    pub inv: f32,
    pub dims: [u32; 3],
    /// 逐格规范化段长上限（`grid.wgsl` 的护栏）。
    pub cap: u32,
    // —— 密度/力相位的常量（与 `probe::PhaseParams` 同义）——
    pub h: f32,
    pub h2: f32,
    pub k6: f32,
    pub w0: f32,
    pub ks: f32,
    pub mass: f32,
    pub alpha_c: f32,
    pub gravity: [f32; 3],
    // —— EOS ——
    pub b_tait: f32,
    pub rho0: f32,
    pub gamma: f32,
    pub clamp_neg: bool,
    // —— 积分 ——
    pub xsph_eps: f32,
    pub max_speed_frac: f32,
}

/// 一轮（一个 tick）跑完的耗时（毫秒）。
#[derive(Clone, Copy, Debug, Default)]
pub struct TickMs {
    /// `ticks` 个 tick 的**总**耗时（无逐 tick 回读，末尾一次同步）。
    pub total: f32,
    /// 每 tick 均值。
    pub per_tick: f32,
    /// 单独量的"一次状态回读 + 同步"成本（毫秒）——耦合接口的代价。
    pub readback_ms: f32,
}

pub struct Packet {
    device: wgpu::Device,
    queue: wgpu::Queue,
    n: u32,
    total: u32,
    groups_n: u32,
    groups_total: u32,
    pos_b: wgpu::Buffer,
    vel_b: wgpu::Buffer,
    counts_b: wgpu::Buffer,
    start_b: wgpu::Buffer,
    cursor_b: wgpu::Buffer,
    overflow_b: wgpu::Buffer,
    int_params_b: wgpu::Buffer,
    p_bin: wgpu::ComputePipeline,
    p_scan: wgpu::ComputePipeline,
    p_place: wgpu::ComputePipeline,
    p_canon: wgpu::ComputePipeline,
    p_dens: wgpu::ComputePipeline,
    p_eos: wgpu::ComputePipeline,
    p_force: wgpu::ComputePipeline,
    p_int: wgpu::ComputePipeline,
    bg_grid: wgpu::BindGroup,
    bg_dens: wgpu::BindGroup,
    bg_force: wgpu::BindGroup,
    bg_eos: wgpu::BindGroup,
    bg_int: wgpu::BindGroup,
    readback_b: wgpu::Buffer,
}

/// 绑定类型（写布局用）。
#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Uniform,
    Ro,
    Rw,
}

fn mk_layout(device: &wgpu::Device, label: &str, list: &[(u32, Kind)]) -> wgpu::BindGroupLayout {
    let entries: Vec<wgpu::BindGroupLayoutEntry> = list
        .iter()
        .map(|&(binding, kind)| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: match kind {
                    Kind::Uniform => wgpu::BufferBindingType::Uniform,
                    Kind::Ro | Kind::Rw => wgpu::BufferBindingType::Storage {
                        read_only: kind == Kind::Ro,
                    },
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
}

fn mk_pipe(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
    shader: &wgpu::ShaderModule,
    label: &str,
    entry: &str,
) -> wgpu::ComputePipeline {
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
}

/// 一个 compute pass + 一次分派（**自由函数**：闭包会独占借用 `enc`，后续就没法再编码拷贝）。
fn dispatch(
    enc: &mut wgpu::CommandEncoder,
    pipe: &wgpu::ComputePipeline,
    bg: &wgpu::BindGroup,
    groups: u32,
) {
    let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
        label: None,
        timestamp_writes: None,
    });
    cp.set_pipeline(pipe);
    cp.set_bind_group(0, bg, &[]);
    cp.dispatch_workgroups(groups.max(1), 1, 1);
}

fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: b.as_entire_binding(),
    }
}

fn f32_bytes(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

impl Packet {
    /// 建全部缓冲/管线（**一次性**），并上传初始 `pos` / `vel` / `pmass`。
    pub fn new(
        adapter_index: usize,
        cfg: PacketCfg,
        pos_flat: &[f32],
        vel_flat: &[f32],
        pmass: &[f32],
    ) -> Result<Self, String> {
        let (_, device, queue) = crate::probe::device_for(adapter_index)?;
        let n = cfg.n;
        let total = cfg.total;
        let storage = |label: &str, size: u64, extra: wgpu::BufferUsages| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_SRC
                    | wgpu::BufferUsages::COPY_DST
                    | extra,
                mapped_at_creation: false,
            })
        };
        let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("p.pos"),
            contents: &f32_bytes(pos_flat),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
        });
        let vel_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("p.vel"),
            contents: &f32_bytes(vel_flat),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
        });
        let pmass_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("p.pmass"),
            contents: &f32_bytes(pmass),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let press_b = storage("p.press", (n as u64) * 4, wgpu::BufferUsages::empty());
        let dens_b = storage("p.dens", (n as u64) * 4, wgpu::BufferUsages::empty());
        let out_b = storage("p.out", (n as u64) * 24, wgpu::BufferUsages::empty());
        let bins_b = storage("p.bins", (n as u64) * 4, wgpu::BufferUsages::empty());
        let counts_b = storage(
            "p.counts",
            ((total + 1) as u64) * 4,
            wgpu::BufferUsages::empty(),
        );
        let start_b = storage(
            "p.start",
            ((total + 1) as u64) * 4,
            wgpu::BufferUsages::empty(),
        );
        let items_b = storage("p.items", (n as u64) * 4, wgpu::BufferUsages::empty());
        let cursor_b = storage(
            "p.cursor",
            ((total + 1) as u64) * 4,
            wgpu::BufferUsages::empty(),
        );
        let overflow_b = storage("p.overflow", 4, wgpu::BufferUsages::empty());
        let readback_b = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("p.readback"),
            size: (n as u64) * 24,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // —— 四组 uniform ——
        let grid_params = {
            let mut b = Vec::with_capacity(48);
            for x in cfg.gmin {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b.extend_from_slice(&cfg.inv.to_le_bytes());
            for x in [
                cfg.dims[0],
                cfg.dims[1],
                cfg.dims[2],
                n,
                total,
                cfg.cap,
                0,
                0,
            ] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b
        };
        let phase_params = {
            let mut b = Vec::with_capacity(80);
            for x in cfg.gmin {
                b.extend_from_slice(&x.to_le_bytes());
            }
            for x in [
                cfg.inv,
                cfg.h2,
                cfg.k6,
                cfg.w0,
                cfg.mass,
                cfg.ks,
                cfg.h,
                cfg.alpha_c,
                0.0,
            ] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            for x in cfg.gravity {
                b.extend_from_slice(&x.to_le_bytes());
            }
            for x in [n, cfg.dims[0], cfg.dims[1], cfg.dims[2], 0] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b
        };
        let eos_params = {
            let mut b = Vec::with_capacity(32);
            for x in [cfg.b_tait, cfg.rho0, cfg.gamma] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b.extend_from_slice(&u32::from(cfg.clamp_neg).to_le_bytes());
            for x in [n, 0, 0, 0] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b
        };
        let uniform = |label: &str, bytes: &[u8]| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: bytes,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            })
        };
        let grid_params_b = uniform("p.grid_params", &grid_params);
        let phase_params_b = uniform("p.phase_params", &phase_params);
        let eos_params_b = uniform("p.eos_params", &eos_params);
        let int_params_b = uniform(
            "p.int_params",
            &[0u8; 32], // 每子步 `write_buffer` 更新
        );

        // —— 五个着色器模块 + 八条管线 ——
        let sh = |label: &str, src: &str| {
            device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(src.into()),
            })
        };
        let m_grid = sh("grid.wgsl", include_str!("grid.wgsl"));
        let m_dens = sh("density.wgsl", include_str!("density.wgsl"));
        let m_force = sh("force.wgsl", include_str!("force.wgsl"));
        let m_eos = sh("eos.wgsl", include_str!("eos.wgsl"));
        let m_int = sh("integrate.wgsl", include_str!("integrate.wgsl"));

        // 网格：0 uniform + 1(pos,只读) + 2..=7(读写)
        let bl_grid = mk_layout(
            &device,
            "p.bl_grid",
            &[
                (0, Kind::Uniform),
                (1, Kind::Ro),
                (2, Kind::Rw),
                (3, Kind::Rw),
                (4, Kind::Rw),
                (5, Kind::Rw),
                (6, Kind::Rw),
                (7, Kind::Rw),
            ],
        );
        // 密度：0 uniform | 1 pos(只读) | 3 pmass(只读) | 5/6 格表(只读) | 7 dens(读写)
        let bl_dens = mk_layout(
            &device,
            "p.bl_dens",
            &[
                (0, Kind::Uniform),
                (1, Kind::Ro),
                (3, Kind::Ro),
                (5, Kind::Ro),
                (6, Kind::Ro),
                (7, Kind::Rw),
            ],
        );
        // 力：0 uniform | 1..=7 只读（含 dens）| 8 out(读写)
        let bl_force = mk_layout(
            &device,
            "p.bl_force",
            &[
                (0, Kind::Uniform),
                (1, Kind::Ro),
                (2, Kind::Ro),
                (3, Kind::Ro),
                (4, Kind::Ro),
                (5, Kind::Ro),
                (6, Kind::Ro),
                (7, Kind::Ro),
                (8, Kind::Rw),
            ],
        );
        let bl_eos = mk_layout(
            &device,
            "p.bl_eos",
            &[(0, Kind::Uniform), (1, Kind::Ro), (2, Kind::Rw)],
        );
        let bl_int = mk_layout(
            &device,
            "p.bl_int",
            &[
                (0, Kind::Uniform),
                (1, Kind::Rw),
                (2, Kind::Rw),
                (3, Kind::Ro),
            ],
        );

        let p_bin = mk_pipe(&device, &bl_grid, &m_grid, "p.bin_count", "bin_count");
        let p_scan = mk_pipe(&device, &bl_grid, &m_grid, "p.scan", "scan");
        let p_place = mk_pipe(&device, &bl_grid, &m_grid, "p.place", "place");
        let p_canon = mk_pipe(&device, &bl_grid, &m_grid, "p.canon", "canon");
        let p_dens = mk_pipe(&device, &bl_dens, &m_dens, "p.density", "density");
        let p_force = mk_pipe(&device, &bl_force, &m_force, "p.force", "force");
        let p_eos = mk_pipe(&device, &bl_eos, &m_eos, "p.eos", "eos");
        let p_int = mk_pipe(&device, &bl_int, &m_int, "p.integrate", "integrate");

        let bg_grid = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_grid"),
            layout: &bl_grid,
            entries: &[
                ent(0, &grid_params_b),
                ent(1, &pos_b),
                ent(2, &bins_b),
                ent(3, &counts_b),
                ent(4, &start_b),
                ent(5, &items_b),
                ent(6, &cursor_b),
                ent(7, &overflow_b),
            ],
        });
        let bg_dens = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_dens"),
            layout: &bl_dens,
            entries: &[
                ent(0, &phase_params_b),
                ent(1, &pos_b),
                ent(3, &pmass_b),
                ent(5, &start_b),
                ent(6, &items_b),
                ent(7, &dens_b),
            ],
        });
        let bg_force = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_force"),
            layout: &bl_force,
            entries: &[
                ent(0, &phase_params_b),
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
        let bg_eos = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_eos"),
            layout: &bl_eos,
            entries: &[ent(0, &eos_params_b), ent(1, &dens_b), ent(2, &press_b)],
        });
        let bg_int = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_int"),
            layout: &bl_int,
            entries: &[
                ent(0, &int_params_b),
                ent(1, &pos_b),
                ent(2, &vel_b),
                ent(3, &out_b),
            ],
        });

        Ok(Self {
            device,
            queue,
            n,
            total,
            groups_n: n.div_ceil(64),
            groups_total: total.div_ceil(64),
            pos_b,
            vel_b,
            counts_b,
            start_b,
            cursor_b,
            overflow_b,
            int_params_b,
            p_bin,
            p_scan,
            p_place,
            p_canon,
            p_dens,
            p_eos,
            p_force,
            p_int,
            bg_grid,
            bg_dens,
            bg_force,
            bg_eos,
            bg_int,
            readback_b,
        })
    }

    /// 一个子步（**一条命令链**）：网格四入口 + 密度 + EOS + 力 + 积分。
    /// `stages` 位掩码（诊断用，`run` 传全 1）：
    /// 0 `bin_count` / 1 `scan`+拷贝+`place` / 2 `canon` / 3 密度 / 4 EOS / 5 力 / 6 积分。
    fn encode_substep(
        &self,
        enc: &mut wgpu::CommandEncoder,
        cfg: &PacketCfg,
        dt_sub: f32,
        stages: u32,
    ) {
        let vmax = cfg.max_speed_frac * cfg.h / dt_sub;
        let mut ip = Vec::with_capacity(32);
        for x in [dt_sub, vmax, cfg.xsph_eps] {
            ip.extend_from_slice(&x.to_le_bytes());
        }
        ip.extend_from_slice(&self.n.to_le_bytes());
        ip.extend_from_slice(&[0u8; 16]);
        self.queue.write_buffer(&self.int_params_b, 0, &ip);

        if stages & 0b000_0001 != 0 {
            enc.clear_buffer(&self.counts_b, 0, None);
            enc.clear_buffer(&self.overflow_b, 0, None);
            dispatch(enc, &self.p_bin, &self.bg_grid, self.groups_n);
        }
        if stages & 0b000_0010 != 0 {
            dispatch(enc, &self.p_scan, &self.bg_grid, 1);
            enc.copy_buffer_to_buffer(
                &self.start_b,
                0,
                &self.cursor_b,
                0,
                ((self.total + 1) as u64) * 4,
            );
            dispatch(enc, &self.p_place, &self.bg_grid, self.groups_n);
        }
        if stages & 0b000_0100 != 0 {
            dispatch(enc, &self.p_canon, &self.bg_grid, self.groups_total);
        }
        if stages & 0b000_1000 != 0 {
            dispatch(enc, &self.p_dens, &self.bg_dens, self.groups_n);
        }
        if stages & 0b001_0000 != 0 {
            dispatch(enc, &self.p_eos, &self.bg_eos, self.groups_n);
        }
        if stages & 0b010_0000 != 0 {
            dispatch(enc, &self.p_force, &self.bg_force, self.groups_n);
        }
        if stages & 0b100_0000 != 0 {
            dispatch(enc, &self.p_int, &self.bg_int, self.groups_n);
        }
    }

    /// 跑 `ticks` 个 tick（每 tick `substeps` 个子步，子步 `dt = 1/(60·substeps)`）。
    /// `tick_readback = true` 时**每 tick** 回读一次状态（模拟耦合接口），否则只在末尾同步一次。
    pub fn run(
        &mut self,
        cfg: &PacketCfg,
        ticks: usize,
        substeps: usize,
        tick_readback: bool,
    ) -> TickMs {
        self.run_stages(cfg, 0b111_1111, ticks, substeps, tick_readback)
    }

    /// **消融计时**（诊断）：只跑 `stages` 打开的相位，返回每 tick 毫秒。
    /// 位：0 分箱 / 1 扫描+拷贝+占位 / 2 规范化 / 3 密度 / 4 EOS / 5 力 / 6 积分
    /// ——用来把"整 tick 为什么这么慢"拆开看。
    pub fn run_stages(
        &mut self,
        cfg: &PacketCfg,
        stages: u32,
        ticks: usize,
        substeps: usize,
        tick_readback: bool,
    ) -> TickMs {
        let dt_tick = 1.0 / 60.0;
        let dt_sub = dt_tick / substeps as f32;
        let mut out = TickMs::default();
        // ⚠️ **这里不做"预热"**：预热会顺带把状态多推进一个 tick。踩过——漂移表里表现为
        // "整块刚性位移 + Δv 恰好 = g·dt"（看着像混沌/接线错，其实是被测对象被提前推进了）。
        // 要摊掉首轮编译/首触成本，请由**调用方**显式跑一次丢弃计时的 `run`。
        let t0 = std::time::Instant::now();
        for _ in 0..ticks {
            let mut enc = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
            for _ in 0..substeps {
                self.encode_substep(&mut enc, cfg, dt_sub, stages);
            }
            if tick_readback {
                enc.copy_buffer_to_buffer(
                    &self.pos_b,
                    0,
                    &self.readback_b,
                    0,
                    (self.n as u64) * 12,
                );
            }
            self.queue.submit(Some(enc.finish()));
            if tick_readback {
                self.poll_wait().ok();
            }
        }
        self.poll_wait().ok();
        out.total = (t0.elapsed().as_secs_f64() * 1e3) as f32;
        out.per_tick = out.total / ticks.max(1) as f32;
        out
    }

    fn poll_wait(&self) -> Result<wgpu::PollStatus, wgpu::PollError> {
        self.device.poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        })
    }

    /// 量一次"状态回读 + 同步"的成本（耦合接口的代价）。
    pub fn measure_readback_ms(&self) -> f32 {
        let t = std::time::Instant::now();
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.pos_b, 0, &self.readback_b, 0, (self.n as u64) * 12);
        self.queue.submit(Some(enc.finish()));
        self.poll_wait().ok();
        let slice = self.readback_b.slice(..(self.n as u64) * 12);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.poll_wait().ok();
        rx.recv().ok();
        slice.get_mapped_range();
        self.readback_b.unmap();
        (t.elapsed().as_secs_f64() * 1e3) as f32
    }

    /// **状态快照**（`pos` / `vel` 扁平拷贝）：**计时类测量必须先冻结状态再比**——
    /// 踩过：消融计时逐项推进流体，剪切把粒子压实 ⇒ 邻域候选变多 ⇒ 后面的项天然更慢，
    /// 读数里混进的是"状态演化"而不是"相位成本"。
    pub fn snapshot(&self) -> (Vec<f32>, Vec<f32>) {
        self.read_state()
    }

    /// 把状态写回（配合 `snapshot`：每次计时前还原到同一状态）。
    pub fn restore(&self, pos: &[f32], vel: &[f32]) {
        self.queue.write_buffer(&self.pos_b, 0, &f32_bytes(pos));
        self.queue.write_buffer(&self.vel_b, 0, &f32_bytes(vel));
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
