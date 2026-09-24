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

// ── 按域拆出的子模块（子目录 pipeline/）
mod gpu_setup;
mod gpu_types;
pub(crate) use self::gpu_setup::*;
pub use self::gpu_types::*;
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

/// `Packet` 的**缓冲清单**（`new` 的第一段：建 + 上传初值）。
pub(crate) struct Bufs {
    pub pos_b: wgpu::Buffer,
    pub vel_b: wgpu::Buffer,
    pub pmass_b: wgpu::Buffer,
    pub press_b: wgpu::Buffer,
    pub dens_b: wgpu::Buffer,
    pub out_b: wgpu::Buffer,
    pub bins_b: wgpu::Buffer,
    pub counts_b: wgpu::Buffer,
    pub start_b: wgpu::Buffer,
    pub items_b: wgpu::Buffer,
    pub cursor_b: wgpu::Buffer,
    pub overflow_b: wgpu::Buffer,
    pub readback_b: wgpu::Buffer,
}

pub(crate) fn make_buffers(
    device: &wgpu::Device,
    n: u32,
    cap_total: u32,
    pos_flat: &[f32],
    vel_flat: &[f32],
    pmass: &[f32],
) -> Bufs {
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
    // ⚠️ 这三张**按格数**的缓冲是一次性分配的（与 CPU 侧 `Vec` 会自动增长不同）：
    // `cap_total` = 分配额度，`refresh_box` 按它夹"总格数预算"⇒ 箱子跟随时不会越界写。
    let counts_b = storage(
        "p.counts",
        ((cap_total + 1) as u64) * 4,
        wgpu::BufferUsages::empty(),
    );
    let start_b = storage(
        "p.start",
        ((cap_total + 1) as u64) * 4,
        wgpu::BufferUsages::empty(),
    );
    let items_b = storage("p.items", (n as u64) * 4, wgpu::BufferUsages::empty());
    let cursor_b = storage(
        "p.cursor",
        ((cap_total + 1) as u64) * 4,
        wgpu::BufferUsages::empty(),
    );
    let overflow_b = storage("p.overflow", 4, wgpu::BufferUsages::empty());
    let readback_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("p.readback"),
        size: (n as u64) * 24,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    Bufs {
        pos_b,
        vel_b,
        pmass_b,
        press_b,
        dens_b,
        out_b,
        bins_b,
        counts_b,
        start_b,
        items_b,
        cursor_b,
        overflow_b,
        readback_b,
    }
}

/// `Packet` 的四组 uniform（`new` 的第二段：字节布局 + 建缓冲）。
pub(crate) struct Params {
    pub grid_params_b: wgpu::Buffer,
    pub phase_params_b: wgpu::Buffer,
    pub eos_params_b: wgpu::Buffer,
    pub int_params_b: wgpu::Buffer,
}

pub(crate) fn make_params(device: &wgpu::Device, cfg: &PacketCfg, n: u32, total: u32) -> Params {
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
        // 末段首字段是相位核读的 **`n_fluid`**（不是总粒子数）：`density.wgsl`/`force.wgsl` 用它
        // 区分"流体邻居 ⇒ 计入 sum"与"边界邻居 ⇒ 计入 sum_b"（Akinci 口径，与 CPU 同式）。
        // ⚠️ 首版这里写的是 `n`（全量）⇒ 边界粒子上卡后全被当流体 ⇒ 一 tick 就炸（动能比 289）。
        for x in [cfg.n_fluid, cfg.dims[0], cfg.dims[1], cfg.dims[2], n] {
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

    Params {
        grid_params_b,
        phase_params_b,
        eos_params_b,
        int_params_b,
    }
}

/// `Packet` 的绑定布局 + 计算管线（`new` 的第三段）。
pub(crate) struct Pipes {
    pub bl_grid: wgpu::BindGroupLayout,
    pub bl_dens: wgpu::BindGroupLayout,
    pub bl_force: wgpu::BindGroupLayout,
    pub bl_eos: wgpu::BindGroupLayout,
    pub bl_int: wgpu::BindGroupLayout,
    pub p_bin: wgpu::ComputePipeline,
    pub p_scan: wgpu::ComputePipeline,
    pub p_place: wgpu::ComputePipeline,
    pub p_canon: wgpu::ComputePipeline,
    pub p_dens: wgpu::ComputePipeline,
    pub p_eos: wgpu::ComputePipeline,
    pub p_force: wgpu::ComputePipeline,
    pub p_int: wgpu::ComputePipeline,
}

pub(crate) fn make_pipelines(device: &wgpu::Device) -> Pipes {
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
        device,
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
        device,
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
        device,
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
        device,
        "p.bl_eos",
        &[(0, Kind::Uniform), (1, Kind::Ro), (2, Kind::Rw)],
    );
    let bl_int = mk_layout(
        device,
        "p.bl_int",
        &[
            (0, Kind::Uniform),
            (1, Kind::Rw),
            (2, Kind::Rw),
            (3, Kind::Ro),
        ],
    );

    let p_bin = mk_pipe(device, &bl_grid, &m_grid, "p.bin_count", "bin_count");
    let p_scan = mk_pipe(device, &bl_grid, &m_grid, "p.scan", "scan");
    let p_place = mk_pipe(device, &bl_grid, &m_grid, "p.place", "place");
    let p_canon = mk_pipe(device, &bl_grid, &m_grid, "p.canon", "canon");
    let p_dens = mk_pipe(device, &bl_dens, &m_dens, "p.density", "density");
    let p_force = mk_pipe(device, &bl_force, &m_force, "p.force", "force");
    let p_eos = mk_pipe(device, &bl_eos, &m_eos, "p.eos", "eos");
    let p_int = mk_pipe(device, &bl_int, &m_int, "p.integrate", "integrate");

    Pipes {
        bl_grid,
        bl_dens,
        bl_force,
        bl_eos,
        bl_int,
        p_bin,
        p_scan,
        p_place,
        p_canon,
        p_dens,
        p_eos,
        p_force,
        p_int,
    }
}

/// `Packet` 的五张 bind group（`new` 的第四段：把缓冲/参数/管线绑起来）。
pub(crate) struct Binds {
    pub bg_grid: wgpu::BindGroup,
    pub bg_dens: wgpu::BindGroup,
    pub bg_force: wgpu::BindGroup,
    pub bg_eos: wgpu::BindGroup,
    pub bg_int: wgpu::BindGroup,
}

pub(crate) fn make_bind_groups(
    device: &wgpu::Device,
    bufs: &Bufs,
    prm: &Params,
    pipes: &Pipes,
) -> Binds {
    let bg_grid = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_grid"),
        layout: &pipes.bl_grid,
        entries: &[
            ent(0, &prm.grid_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.bins_b),
            ent(3, &bufs.counts_b),
            ent(4, &bufs.start_b),
            ent(5, &bufs.items_b),
            ent(6, &bufs.cursor_b),
            ent(7, &bufs.overflow_b),
        ],
    });
    let bg_dens = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_dens"),
        layout: &pipes.bl_dens,
        entries: &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.pos_b),
            ent(3, &bufs.pmass_b),
            ent(5, &bufs.start_b),
            ent(6, &bufs.items_b),
            ent(7, &bufs.dens_b),
        ],
    });
    let bg_force = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_force"),
        layout: &pipes.bl_force,
        entries: &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.vel_b),
            ent(3, &bufs.pmass_b),
            ent(4, &bufs.press_b),
            ent(5, &bufs.start_b),
            ent(6, &bufs.items_b),
            ent(7, &bufs.dens_b),
            ent(8, &bufs.out_b),
        ],
    });
    let bg_eos = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_eos"),
        layout: &pipes.bl_eos,
        entries: &[
            ent(0, &prm.eos_params_b),
            ent(1, &bufs.dens_b),
            ent(2, &bufs.press_b),
        ],
    });
    let bg_int = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_int"),
        layout: &pipes.bl_int,
        entries: &[
            ent(0, &prm.int_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.vel_b),
            ent(3, &bufs.out_b),
        ],
    });

    Binds {
        bg_grid,
        bg_dens,
        bg_force,
        bg_eos,
        bg_int,
    }
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
        // 四段各进一个 helper（`new` 由 310 行降到 ~90）：**持有结构体而不是就地解构**——
        // 后一段（bind group）要借前几段（`&bufs`/`&prm`/`&pipes`），解构会把它们移走。
        let cap_total = total.max(cfg.grid_bins_cap);
        let bufs = make_buffers(&device, n, cap_total, pos_flat, vel_flat, pmass);
        let prm = make_params(&device, &cfg, n, total);
        let pipes = make_pipelines(&device);
        let binds = make_bind_groups(&device, &bufs, &prm, &pipes);
        // 常驻包围盒阶段（借用 `bufs.pos_b`；借用在此结束，随后字段被移进 `Self`）。
        let bbox = crate::bbox::BboxStage::new(
            &device,
            &bufs.pos_b,
            n,
            cfg.h,
            vxl_phys_core::grid::GRID_MAX_BINS.min(cap_total as usize),
        );
        Ok(Self {
            device,
            queue,
            n,
            // `recompute_box` 时活格数由 GPU 侧决定（主机不再回读）⇒ 规范化分派与"扫描→占位"
            // 拷贝都按**分配额度**来：核函数按 uniform 里的活 `total` 自限 ⇒ 多出的线程/字节只是
            // 空转、不改变结果（4 MB 的拷贝换来"零回读"，实测净赚）。
            total: if cfg.recompute_box { cap_total } else { total },
            groups_n: n.div_ceil(64),
            groups_fluid: cfg.n_fluid.div_ceil(64),
            groups_total: if cfg.recompute_box {
                cap_total.div_ceil(64)
            } else {
                total.div_ceil(64)
            },
            pos_b: bufs.pos_b,
            vel_b: bufs.vel_b,
            counts_b: bufs.counts_b,
            start_b: bufs.start_b,
            cursor_b: bufs.cursor_b,
            overflow_b: bufs.overflow_b,
            int_params_b: prm.int_params_b,
            grid_params_b: prm.grid_params_b,
            phase_params_b: prm.phase_params_b,
            bbox,
            p_bin: pipes.p_bin,
            p_scan: pipes.p_scan,
            p_place: pipes.p_place,
            p_canon: pipes.p_canon,
            p_dens: pipes.p_dens,
            p_eos: pipes.p_eos,
            p_force: pipes.p_force,
            p_int: pipes.p_int,
            bg_grid: binds.bg_grid,
            bg_dens: binds.bg_dens,
            bg_force: binds.bg_force,
            bg_eos: binds.bg_eos,
            bg_int: binds.bg_int,
            readback_b: bufs.readback_b,
        })
    }

    /// 一个子步（**一条命令链**）：网格四入口 + 密度 + EOS + 力 + 积分。
    /// `stages` 位掩码（诊断用，`run` 传全 1）：
    /// 0 `bin_count` / 1 `scan`+拷贝+`place` / 2 `canon` / 3 密度 / 4 EOS / 5 力 / 6 积分。
    pub(crate) fn encode_substep(
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
            // 游标由 `scan` 自己写（见 `grid.wgsl`）⇒ 这里不再 `copy_buffer_to_buffer`。
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
            // 积分**只跑流体前缀**：边界粒子（2b）是运动学冻结的，被积分会飘走——
            // 与 CPU `FluidSystem::substep` 的 `for i in 0..nf` 逐条对应。
            dispatch(enc, &self.p_int, &self.bg_int, self.groups_fluid);
        }
    }

    /// **每子步重算箱子**（`cfg.recompute_box`）：卡上"归约 → setup"写出**与 uniform 同形**的箱子，
    /// 再用**设备内拷贝**搬进两组 uniform —— **不需要任何回读**（这正是卡上 setup 的意义：主机侧
    /// 规则那条路每子步要一次 24 B 往返，实测占整 tick 的 **88%**）。返回这一步的墙钟毫秒。
    ///
    /// 为什么必须每子步：CPU 引擎在 `substep()` 开头就 `self.grid.rebuild(&self.pos, self.h)`
    /// （见 `vxl-phys-fluid/src/fluid_step.rs`）⇒ GPU 若不与其同频，两条链的分箱/邻域就不同，
    /// 逐 tick 漂移表失去意义。
    fn refresh_box(&mut self) -> f32 {
        let t = std::time::Instant::now();
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("bbox"),
            });
        self.bbox.encode(&self.queue, &mut enc);
        // 搬进 uniform 的**动态字段**（偏移口径见 `make_params`，其余静态字段不动）：
        //   grid_params ：gmin 0..12 | inv 12..16 | dims 16..28 | n 28..32 | total 32..36
        //   phase_params：gmin 0..12 | inv 12..16 | … | n 60..64 | dims 64..76
        let src = self.bbox.box_out();
        enc.copy_buffer_to_buffer(src, 0, &self.grid_params_b, 0, 16);
        enc.copy_buffer_to_buffer(src, 16, &self.grid_params_b, 16, 12);
        enc.copy_buffer_to_buffer(src, 28, &self.grid_params_b, 32, 4);
        enc.copy_buffer_to_buffer(src, 0, &self.phase_params_b, 0, 16);
        enc.copy_buffer_to_buffer(src, 16, &self.phase_params_b, 64, 12);
        self.queue.submit(Some(enc.finish()));
        (t.elapsed().as_secs_f64() * 1e3) as f32
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
            if cfg.recompute_box {
                // **每子步重算箱子**（与 CPU 的 `substep()` 同频）：代价 = 每子步一次提交 + 回读往返，
                // 单独记进 `out.box_ms` 以便读数里能看见这笔开销。
                for si in 0..substeps {
                    out.box_ms += self.refresh_box();
                    let mut enc = self
                        .device
                        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
                    self.encode_substep(&mut enc, cfg, dt_sub, stages);
                    if tick_readback && si + 1 == substeps {
                        enc.copy_buffer_to_buffer(
                            &self.pos_b,
                            0,
                            &self.readback_b,
                            0,
                            (self.n as u64) * 12,
                        );
                    }
                    self.queue.submit(Some(enc.finish()));
                }
                if tick_readback {
                    self.poll_wait().ok();
                }
                continue;
            }
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

    pub(crate) fn poll_wait(&self) -> Result<wgpu::PollStatus, wgpu::PollError> {
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
