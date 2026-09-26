//! gpu_types：从 pipeline.rs 按域拆出（纯搬移，语义未改）。

/// 建管线所需的静态参数（一次给全；运行时只有 `dt` 会变）。
#[derive(Clone, Copy, Debug)]
pub struct PacketCfg {
    pub n: u32,
    /// **流体粒子数**（索引前缀；含 2b 边界粒子时 `n_fluid < n`）。
    /// 语义与 CPU 的 `FluidSystem::substep` 逐条对应：密度/压力/力跑**全部**粒子（邻域必须看得见
    /// 边界粒子），而**积分（含 XSPH/CFL）只跑 `0..n_fluid`**——边界粒子是运动学冻结的。
    /// 纯流体场景令 `n_fluid == n`（此时与旧口径逐位一致）。
    pub n_fluid: u32,
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
    /// **每子步从位置重算箱子**（`gmin`/`inv`/`dims`）：CPU 引擎就是每子步 `rebuild` 一次
    /// ⇒ `true` = 与 CPU 同频（自由落体等"跑出箱子"的场景必须开）；`false` = 固定箱子口径
    /// （与 `PLAN-gpu.md` §12.1 的读数一致，留给消融/计时对比）。
    pub recompute_box: bool,
    /// **格表分配额度**（格数）：`counts`/`start`/`cursor` 三张缓冲按它一次性分配。
    /// `recompute_box` 打开时，每子步的"总格数预算"取 `min(GRID_MAX_BINS, 本额度)`
    /// ——GPU 的缓冲不会像 CPU 的 `Vec` 那样增长 ⇒ 必须显式给额度，否则箱子跟随会越界写。
    /// 给 0 或给了比 `total` 小的值都按 `total` 处理。
    pub grid_bins_cap: u32,
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
    /// `recompute_box` 打开时：每子步重算箱子（归约 + 回读 + 写 uniform）的**累计**毫秒。
    pub box_ms: f32,
}

pub struct Packet {
    pub(crate) device: wgpu::Device,
    pub(crate) queue: wgpu::Queue,
    pub(crate) n: u32,
    /// 活的格数（**诊断/未来用途**：`recompute_box` 时它是分配额度——真值由 GPU 侧决定）。
    pub(crate) total: u32,
    pub(crate) groups_n: u32,
    pub(crate) groups_total: u32,
    /// 格表：`start`（每格起点，长度 `total + 1`）与 `cursor`（`place` 用的可变游标）。
    /// 二者现在**只由 `grid.wgsl` 写**（`scan` 一次写两份，省掉每子步的 `copy_buffer_to_buffer`），
    /// 主机侧只在诊断里取用。
    pub(crate) start_b: wgpu::Buffer,
    pub(crate) cursor_b: wgpu::Buffer,
    pub(crate) pos_b: wgpu::Buffer,
    pub(crate) vel_b: wgpu::Buffer,
    pub(crate) counts_b: wgpu::Buffer,
    pub(crate) overflow_b: wgpu::Buffer,
    pub(crate) int_params_b: wgpu::Buffer,
    /// 网格 uniform（每子步重算箱子时要改它前 36 字节）。
    pub(crate) grid_params_b: wgpu::Buffer,
    /// 密度/力相位 uniform（同样含箱子三件套，偏移见 `make_params`）。
    pub(crate) phase_params_b: wgpu::Buffer,
    /// 常驻的包围盒归约阶段（`cfg.recompute_box` 时每子步用一次）。
    pub(crate) bbox: crate::bbox::BboxStage,
    /// 力/积分相位的输出缓冲（每粒 6 个 f32：`(加速度或反作用力, XSPH)`）——"反作用回读"从它取边界段。
    pub(crate) out_b: wgpu::Buffer,
    pub(crate) p_bin: wgpu::ComputePipeline,
    pub(crate) p_scan: wgpu::ComputePipeline,
    pub(crate) p_place: wgpu::ComputePipeline,
    pub(crate) p_canon: wgpu::ComputePipeline,
    pub(crate) p_dens: wgpu::ComputePipeline,
    pub(crate) p_eos: wgpu::ComputePipeline,
    pub(crate) p_force: wgpu::ComputePipeline,
    pub(crate) p_int: wgpu::ComputePipeline,
    pub(crate) bg_grid: wgpu::BindGroup,
    pub(crate) bg_dens: wgpu::BindGroup,
    pub(crate) bg_force: wgpu::BindGroup,
    pub(crate) bg_eos: wgpu::BindGroup,
    pub(crate) bg_int: wgpu::BindGroup,
    pub(crate) readback_b: wgpu::Buffer,
    /// **格序副本档**的常驻物（`cfg.sort_copies` 且适用时才有；见 `sorted.rs`）。
    pub(crate) sorted: Option<super::sorted::Sorted>,
}
