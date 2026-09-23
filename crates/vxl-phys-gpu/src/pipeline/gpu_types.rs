//! gpu_types：从 pipeline.rs 按域拆出（纯搬移，语义未改）。

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
    pub(crate) device: wgpu::Device,
    pub(crate) queue: wgpu::Queue,
    pub(crate) n: u32,
    pub(crate) total: u32,
    pub(crate) groups_n: u32,
    pub(crate) groups_total: u32,
    pub(crate) pos_b: wgpu::Buffer,
    pub(crate) vel_b: wgpu::Buffer,
    pub(crate) counts_b: wgpu::Buffer,
    pub(crate) start_b: wgpu::Buffer,
    pub(crate) cursor_b: wgpu::Buffer,
    pub(crate) overflow_b: wgpu::Buffer,
    pub(crate) int_params_b: wgpu::Buffer,
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
}
