//! gpu_setup：从 pipeline.rs 按域拆出（纯搬移，语义未改）。

/// 绑定类型（写布局用）。
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum Kind {
    Uniform,
    Ro,
    Rw,
}

pub(crate) fn mk_layout(
    device: &wgpu::Device,
    label: &str,
    list: &[(u32, Kind)],
) -> wgpu::BindGroupLayout {
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

pub(crate) fn mk_pipe(
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
pub(crate) fn dispatch(
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

pub(crate) fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: b.as_entire_binding(),
    }
}

pub(crate) fn f32_bytes(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}
