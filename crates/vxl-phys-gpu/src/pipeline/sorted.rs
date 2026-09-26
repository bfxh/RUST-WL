//! **格序副本档**（`PacketCfg::sort_copies`，**默认关**）——`PLAN-gpu.md` §23.1 的落地。
//!
//! 做法：每子步在 `canon` 之后、`density` 之前插一趟 `gather`（索引序 → 格序的三张副本
//! pos/vel/press），`force` 之后插一趟 `scatter`（输出从格序写回索引序）。**两个核一字不改**——
//! 只把绑定换成副本 + **恒等表**（`id[k] = k`）当 `cell_items`：因为两档的 `cell_start` 逐项相同、
//! 邻域枚举序列逐条相同（判据 §23），核里 `j = cell_items[k]` 拿到的就是格序副本的下标。
//!
//! 适用范围（**守死，宁可退回平铺档也不许静默错**）：
//! - `cfg.n_fluid == cfg.n`（纯流体）：边界粒子会让 `j < P.n_fluid` 这条**标签**判断失效（空间排序
//!   会把流体与边界混在一起），要支持它得把两类的格表分开建（见 §23.1 的后续项）；
//! - 与壁面档**互斥**：壁面档改的是 `dens`（索引序），格序档下 `dens` 是副本 ⇒ 不搬它会**静默失效**。
//!
//! 这两条不满足时 `Packet::build` 直接不建（`PacketCfg::sort_copies` 被忽略），走平铺档。

use super::*;

/// 格序副本档的全部常驻物：2 条管线 + 5 个绑定组（缓冲由绑定组自己持有——wgpu 的绑定组会
/// 引用计数住缓冲，主机侧不必再存句柄）。
pub(crate) struct Sorted {
    pub p_gather: wgpu::ComputePipeline,
    pub p_scatter: wgpu::ComputePipeline,
    pub bg_gather: wgpu::BindGroup,
    pub bg_scatter: wgpu::BindGroup,
    pub bg_dens: wgpu::BindGroup,
    pub bg_force: wgpu::BindGroup,
    pub bg_eos: wgpu::BindGroup,
}

/// 建格序副本档。`bufs`/`prm` 提供源与 uniform、`pipes` 提供三个相位的**既有布局**（副本档共用布局、
/// 只换缓冲 —— 这正是"核不改"在接口侧的对应物）。
pub(crate) fn make_sorted(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    n: u32,
    mass: f32,
    bufs: &Bufs,
    prm: &Params,
    pipes: &Pipes,
) -> Sorted {
    let c = CopyBufs::new(device, queue, n, mass);
    let sp = make_sorted_pipelines(device);
    let b = make_sorted_binds(device, &c, bufs, prm, pipes, &sp);
    Sorted {
        p_gather: sp.gather,
        p_scatter: sp.scatter,
        bg_gather: b.0,
        bg_scatter: b.1,
        bg_dens: b.2,
        bg_force: b.3,
        bg_eos: b.4,
    }
}

/// 六张副本缓冲 + 一张恒等表（`pmass` 纯流体是常量 ⇒ 填一次，不必每子步重排）。
struct CopyBufs {
    pos_c: wgpu::Buffer,
    vel_c: wgpu::Buffer,
    press_c: wgpu::Buffer,
    pmass_c: wgpu::Buffer,
    dens_c: wgpu::Buffer,
    out_c: wgpu::Buffer,
    items_id: wgpu::Buffer,
}

impl CopyBufs {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, n: u32, mass: f32) -> Self {
        let storage = |label: &str, size: u64| -> wgpu::Buffer {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_SRC
                    | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let (nf, n3) = ((n as u64) * 4, (n as u64) * 12);
        // 恒等表：把核里那行 `j = cell_items[k]` 变成"取自己"⇒ 读格序副本的下标。
        let mut id: Vec<u8> = Vec::with_capacity(n as usize * 4);
        let mut pm: Vec<u8> = Vec::with_capacity(n as usize * 4);
        for k in 0..n {
            id.extend_from_slice(&k.to_le_bytes());
            pm.extend_from_slice(&mass.to_le_bytes());
        }
        let items_id = storage("s.items_id", (n as u64) * 4);
        let pmass_c = storage("s.pmass_c", nf);
        queue.write_buffer(&items_id, 0, &id);
        queue.write_buffer(&pmass_c, 0, &pm);
        Self {
            items_id,
            pmass_c,
            pos_c: storage("s.pos_c", n3),
            vel_c: storage("s.vel_c", n3),
            press_c: storage("s.press_c", nf),
            dens_c: storage("s.dens_c", nf),
            out_c: storage("s.out_c", (n as u64) * 24),
        }
    }
}

/// 两条新管线（重排 / 回写）+ 各自的布局。
struct SortedPipes {
    gather: wgpu::ComputePipeline,
    scatter: wgpu::ComputePipeline,
    bl_gather: wgpu::BindGroupLayout,
    bl_scatter: wgpu::BindGroupLayout,
}

fn make_sorted_pipelines(device: &wgpu::Device) -> SortedPipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("reorder.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("../reorder.wgsl").into()),
    });
    let mk = |label: &str,
              slots: &[(u32, bool)],
              entry: &str|
     -> (wgpu::ComputePipeline, wgpu::BindGroupLayout) {
        let bl = make_layout(device, label, slots);
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some(label),
            bind_group_layouts: &[&bl],
            push_constant_ranges: &[],
        });
        let p = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        });
        (p, bl)
    };
    // 槽位表按**入口实际用到**的绑定给（`scatter` 复用 `src_a`/`dst_a` ⇒ 是 1/2/5 这组不连续的）。
    let gslots = [
        (1, true),
        (2, true),
        (3, true),
        (4, true),
        (5, false),
        (6, false),
        (7, false),
    ];
    let (gather, bl_gather) = mk("s.gather", &gslots, "gather");
    let (scatter, bl_scatter) = mk("s.scatter", &[(1, true), (2, true), (5, false)], "scatter");
    SortedPipes {
        gather,
        scatter,
        bl_gather,
        bl_scatter,
    }
}

/// 五个绑定组：重排 / 回写，以及**三个相位（共用既有布局、只换缓冲）**。
#[allow(clippy::type_complexity)] // 五个同型绑定组的元组：拆成新类型只加间接、不改复杂度
fn make_sorted_binds(
    device: &wgpu::Device,
    c: &CopyBufs,
    bufs: &Bufs,
    prm: &Params,
    pipes: &Pipes,
    sp: &SortedPipes,
) -> (
    wgpu::BindGroup,
    wgpu::BindGroup,
    wgpu::BindGroup,
    wgpu::BindGroup,
    wgpu::BindGroup,
) {
    let bg = |label: &str, layout: &wgpu::BindGroupLayout, e: &[wgpu::BindGroupEntry<'_>]| {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(label),
            layout,
            entries: e,
        })
    };
    let bg_gather = bg(
        "s.bg_gather",
        &sp.bl_gather,
        &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.items_b),
            ent(2, &bufs.pos_b),
            ent(3, &bufs.vel_b),
            ent(4, &bufs.press_b),
            ent(5, &c.pos_c),
            ent(6, &c.vel_c),
            ent(7, &c.press_c),
        ],
    );
    let bg_scatter = bg(
        "s.bg_scatter",
        &sp.bl_scatter,
        &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.items_b),
            ent(2, &c.out_c),
            ent(5, &bufs.out_b),
        ],
    );
    let bg_dens = bg(
        "s.bg_dens",
        &pipes.bl_dens,
        &[
            ent(0, &prm.phase_params_b),
            ent(1, &c.pos_c),
            ent(3, &c.pmass_c),
            ent(5, &bufs.start_b),
            ent(6, &c.items_id),
            ent(7, &c.dens_c),
        ],
    );
    let bg_eos = bg(
        "s.bg_eos",
        &pipes.bl_eos,
        &[
            ent(0, &prm.eos_params_b),
            ent(1, &c.dens_c),
            ent(2, &c.press_c),
        ],
    );
    let bg_force = bg(
        "s.bg_force",
        &pipes.bl_force,
        &[
            ent(0, &prm.phase_params_b),
            ent(1, &c.pos_c),
            ent(2, &c.vel_c),
            ent(3, &c.pmass_c),
            ent(4, &c.press_c),
            ent(5, &bufs.start_b),
            ent(6, &c.items_id),
            ent(7, &c.dens_c),
            ent(8, &c.out_c),
        ],
    );
    (bg_gather, bg_scatter, bg_dens, bg_force, bg_eos)
}

/// 一张布局 = `0` 号 uniform + **显式给出的 storage 槽位表**（`(槽位, 只读?)`）。
/// 为什么要显式：两个入口在同一个着色器模块里，`scatter` 用的是 `1/2/5` 这组**不连续**的槽位
/// （它复用 `src_a`/`dst_a` 两个名字），而 wgpu 只校验**该入口实际用到的**绑定，
/// 且要求 storage 的访问权限**精确匹配**。
fn make_layout(
    device: &wgpu::Device,
    label: &str,
    storage: &[(u32, bool)],
) -> wgpu::BindGroupLayout {
    let mut entries = vec![wgpu::BindGroupLayoutEntry {
        binding: 0,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }];
    for &(binding, read_only) in storage {
        entries.push(wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        });
    }
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &entries,
    })
}
