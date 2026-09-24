//! **包围盒归约的主机侧**：参考实现 + 上卡探针（`bbox.wgsl`）。
//!
//! 用途（`PLAN-gpu.md` §12.3 第一条）：GPU 常驻管线的箱子本片是**固定**的，真后端要每子步
//! 从位置重算。本模块给两件东西：
//! 1. `host_box` —— 主机侧参考（自己归约 min/max，再走**同一条**箱子规则
//!    `vxl_phys_core::grid::grid_box`），供对拍；
//! 2. `box_on_adapter` —— 上卡跑 `bbox.wgsl` 的 `reduce`、读回 24 字节累加器，再走上面
//!    同一条规则，供探针与测试。
//!
//! **等价口径**（测试按这个断言）：`min`/`max`/`bin`/`dims`/`inv` **逐位相同**（min/max 是精确
//! 运算、与求值顺序无关；三件套由主机按同一表达式算）。
//!
//! 首版的教训（留痕）：另写了一个单线程 `box_setup` 核在卡上算三件套，实测它写出的箱子缓冲是
//! **未初始化内存** —— 而同一轮里归约累加器（`bb`）逐项正确 ⇒ 问题在那一遍 pass 的落写，不在
//! 归约。为不阻塞研发：**规则只在主机侧算一次**（`grid_box`），卡上只做归约（这一半已被
//! `gpu_reduce_covers_all_particles` 逐位钉住）。等管线接线时若发现每子步回读 24 字节吃不消，
//! 再回头做卡上 setup（届时先用"写哨兵看它落不落"的方式定位那一遍 pass）。

use vxl_phys_core::grid::grid_box;
use vxl_phys_core::Vec3;
use wgpu::util::DeviceExt;

/// 上卡读数；`error` 非空表示不可用（例如本机无适配器）。
pub struct BoxOut {
    pub adapter: String,
    pub error: Option<String>,
    /// 归约出的粒子角点（世界系）。
    pub min: [f32; 3],
    pub max: [f32; 3],
    /// 箱子三件套（主机侧按 `grid_box` 算）。
    pub bin: f32,
    pub inv: f32,
    pub dims: [u32; 3],
    pub total: u32,
    /// **原始原子累加器**（有序映射下的 6 个 u32）——诊断用：判"归约错"还是"下游接线错"。
    pub bb: [u32; 6],
    /// **卡上 setup** 的产物：`inv`（f32）/ `dims` / `total`（主机那份在上面 `bin`/`inv`/`dims`）。
    pub gpu_inv: f32,
    pub gpu_dims: [u32; 3],
    pub gpu_total: u32,
}

impl BoxOut {
    fn err(msg: String) -> Self {
        Self {
            adapter: String::new(),
            error: Some(msg),
            min: [0.0; 3],
            max: [0.0; 3],
            bin: 0.0,
            inv: 0.0,
            dims: [0; 3],
            total: 0,
            bb: [0; 6],
            gpu_inv: 0.0,
            gpu_dims: [0; 3],
            gpu_total: 0,
        }
    }
}

/// 有序映射的逆（与 `bbox.wgsl` 的 `f2o` 对偶）：`u32 → f32`。
pub fn ordered_to_f32(o: u32) -> f32 {
    if o & 0x8000_0000 != 0 {
        f32::from_bits(o ^ 0x8000_0000)
    } else {
        f32::from_bits(!o)
    }
}

/// 主机侧参考：归约 `pos_flat`（xyz 扁平）的角点，再走**同一条**箱子规则。
/// 返回 `(min, max, bin, dims)`。
pub fn host_box(pos_flat: &[f32], h: f32, max_bins: usize) -> ([f32; 3], [f32; 3], f32, [u32; 3]) {
    let n = pos_flat.len() / 3;
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for i in 0..n {
        for a in 0..3 {
            let v = pos_flat[i * 3 + a];
            lo[a] = lo[a].min(v);
            hi[a] = hi[a].max(v);
        }
    }
    let (_, bin, dims) = grid_box(
        Vec3::new(lo[0], lo[1], lo[2]),
        Vec3::new(hi[0], hi[1], hi[2]),
        h,
        max_bins,
    );
    (lo, hi, bin, dims)
}

/// 归约的一次性建置产物（绑定组 + 管线；`box_on_adapter` 只读引用）。
struct BboxPipes {
    bg: wgpu::BindGroup,
    reduce: wgpu::ComputePipeline,
    setup: wgpu::ComputePipeline,
}

/// 一次性建置：着色器 / 绑定布局 / 绑定组 / 两条管线（不进出稳态计时）。
fn make_bbox_pipes(
    device: &wgpu::Device,
    pos_b: &wgpu::Buffer,
    bb_b: &wgpu::Buffer,
    rp_b: &wgpu::Buffer,
    box_out_b: &wgpu::Buffer,
    sp_b: &wgpu::Buffer,
) -> BboxPipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("bbox.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("bbox.wgsl").into()),
    });
    // 5 个槽位：0 pos(read) | 1 bb(rw，原子) | 2 rp(uniform) | 3 box_out(rw) | 4 sp(uniform)
    let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..5u32)
        .map(|binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if binding == 2 || binding == 4 {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: binding == 0,
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("bbox.bgl"),
        entries: &entries,
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("bbox.bg"),
        layout: &bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: pos_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: bb_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: rp_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: box_out_b.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: sp_b.as_entire_binding(),
            },
        ],
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("bbox.pl"),
        bind_group_layouts: &[&bgl],
        push_constant_ranges: &[],
    });
    let p_reduce = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("bbox.reduce"),
        layout: Some(&pl),
        module: &shader,
        entry_point: Some("reduce"),
        compilation_options: Default::default(),
        cache: None,
    });
    let p_setup = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("bbox.setup"),
        layout: Some(&pl),
        module: &shader,
        entry_point: Some("box_setup"),
        compilation_options: Default::default(),
        cache: None,
    });

    BboxPipes {
        bg,
        reduce: p_reduce,
        setup: p_setup,
    }
}

fn f32_bytes(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// `reduce` 的工作组大小（与 `bbox.wgsl` 的 `WG` 一致）。
const WG: u32 = 64;
/// `bb` 的中性元：`[+∞, +∞, +∞, −∞, −∞, −∞]` **在有序映射下**的位模式
/// （⚠️ `+∞` 映射后是 `0xFF800000`：有序映射把正数的符号位翻成 1）。
const BB_INIT: [u32; 6] = [
    0xFF80_0000,
    0xFF80_0000,
    0xFF80_0000,
    0x007F_FFFF,
    0x007F_FFFF,
    0x007F_FFFF,
];

/// `bb` 的中性元字节（`BB_INIT` 的 LE 展开）。
fn bb_init_bytes() -> Vec<u8> {
    let mut b = Vec::with_capacity(24);
    for x in BB_INIT {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// 探针的一次性建置（`box_on_adapter` 的第一段）：`pos`/`bb`/`box_out`/`sp`/`rp` 与回读缓冲 + 管线。
/// 返回 `(bb, box_out, readback, pipes)`——`pos` 与两个 uniform 只被 bind group 持有。
fn make_probe(
    device: &wgpu::Device,
    pos_flat: &[f32],
    n: u32,
    ngroups: u32,
    h: f32,
    max_bins: usize,
) -> (wgpu::Buffer, wgpu::Buffer, wgpu::Buffer, BboxPipes) {
    let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.pos"),
        contents: &f32_bytes(pos_flat),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let bb_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.bb"),
        contents: &bb_init_bytes(),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
    });
    let rp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.rp"),
        contents: &{
            let mut b = Vec::with_capacity(16);
            for x in [n, ngroups, 0, 0] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b
        },
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let box_out_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bbox.box_out"),
        size: 48, // 8 槽（uniform 形）+ 3 槽紧凑 `hi`（近域过滤用的真 AABB，见 `bbox.wgsl` 头注）
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let sp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.sp"),
        contents: &f32_bytes(&[h, max_bins as f32, 0.0, 0.0]),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bbox.readback"),
        size: 72, // bb（24 B）+ box_out（48 B）
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let pipes = make_bbox_pipes(device, &pos_b, &bb_b, &rp_b, &box_out_b, &sp_b);
    (bb_b, box_out_b, readback, pipes)
}

/// 在**指定适配器序号**上跑归约 + setup，读回角点与卡上箱子。
/// `error` 非空即未测到（无适配器 / 空输入）。
pub fn box_on_adapter(adapter_index: usize, pos_flat: &[f32], h: f32, max_bins: usize) -> BoxOut {
    let n = (pos_flat.len() / 3) as u32;
    if n == 0 {
        return BoxOut::err("空输入（n = 0）：CPU 侧此时把格数置 0，GPU 路径无意义".to_string());
    }
    let (adapter, device, queue) = match crate::probe::device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => return BoxOut::err(e),
    };
    let ngroups = n.div_ceil(WG).max(1);
    let (bb_b, box_out_b, readback, pipes) = make_probe(&device, pos_flat, n, ngroups, h, max_bins);

    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("bbox.enc"),
    });
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("bbox.reduce"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.reduce);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(ngroups, 1, 1);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("bbox.setup"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.setup);
        cp.set_bind_group(0, &pipes.bg, &[]);
        cp.dispatch_workgroups(1, 1, 1);
    }
    // 回读布局：bb 0..24 | box_out 24..72
    enc.copy_buffer_to_buffer(&bb_b, 0, &readback, 0, 24);
    enc.copy_buffer_to_buffer(&box_out_b, 0, &readback, 24, 48);
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
    let data = slice.get_mapped_range();
    let bb = decode_bb(&data);
    let (lo, hi) = bb_to_corners(bb);
    let u = |k: usize| {
        let o = 24 + k * 4;
        u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]])
    };
    // 卡上 setup 的产物：[3]=inv(f32 位模式) | [4..6]=dims | [7]=total。
    let gpu_inv = f32::from_bits(u(3));
    let gpu_dims = [u(4), u(5), u(6)];
    let gpu_total = u(7);
    // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
    drop(data);
    readback.unmap();

    let (_, bin, dims) = grid_box(
        Vec3::new(lo[0], lo[1], lo[2]),
        Vec3::new(hi[0], hi[1], hi[2]),
        h,
        max_bins,
    );
    BoxOut {
        adapter,
        error: None,
        min: lo,
        max: hi,
        bin,
        inv: 1.0 / bin,
        dims,
        total: dims[0] * dims[1] * dims[2],
        bb,
        gpu_inv,
        gpu_dims,
        gpu_total,
    }
}

/// `bb` 的有序 u32 → 角点 `(min, max)`。
fn bb_to_corners(bb: [u32; 6]) -> ([f32; 3], [f32; 3]) {
    (
        [
            ordered_to_f32(bb[0]),
            ordered_to_f32(bb[1]),
            ordered_to_f32(bb[2]),
        ],
        [
            ordered_to_f32(bb[3]),
            ordered_to_f32(bb[4]),
            ordered_to_f32(bb[5]),
        ],
    )
}

/// 回读字节 → `bb`（6 个有序 u32）。
fn decode_bb(data: &[u8]) -> [u32; 6] {
    let mut bb = [0u32; 6];
    for (k, slot) in bb.iter_mut().enumerate() {
        let o = k * 4;
        *slot = u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
    }
    bb
}

/// **常驻**包围盒阶段（`Packet` 用）：缓冲/管线只建一次，每子步只"写中性元 + 归约 + setup"。
pub struct BboxStage {
    groups: u32,
    bb_b: wgpu::Buffer,
    /// setup 的产物（与 uniform 动态字段同形，见 `bbox.wgsl`）——主机侧用 `copy_buffer_to_buffer` 搬。
    box_out_b: wgpu::Buffer,
    readback: wgpu::Buffer,
    bg: wgpu::BindGroup,
    reduce: wgpu::ComputePipeline,
    setup: wgpu::ComputePipeline,
}

impl BboxStage {
    /// 建常驻件（`pos_b` 复用管线自己的位置缓冲，不复制；`sp` 用 `(h, max_bins)`）。
    pub fn new(
        device: &wgpu::Device,
        pos_b: &wgpu::Buffer,
        n: u32,
        h: f32,
        max_bins: usize,
    ) -> Self {
        let groups = n.div_ceil(WG).max(1);
        let bb_b = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bbox.stage.bb"),
            size: 24,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let box_out_b = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bbox.stage.box_out"),
            size: 48, // 同探针：8 槽 uniform 形 + 3 槽紧凑 `hi`
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let sp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bbox.stage.sp"),
            contents: &f32_bytes(&[h, max_bins as f32, 0.0, 0.0]),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bbox.stage.readback"),
            size: 56,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let rp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bbox.stage.rp"),
            contents: &{
                let mut b = Vec::with_capacity(16);
                for x in [n, groups, 0, 0] {
                    b.extend_from_slice(&x.to_le_bytes());
                }
                b
            },
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let pipes = make_bbox_pipes(device, pos_b, &bb_b, &rp_b, &box_out_b, &sp_b);
        Self {
            groups,
            bb_b,
            box_out_b,
            readback,
            bg: pipes.bg,
            reduce: pipes.reduce,
            setup: pipes.setup,
        }
    }

    /// 一次"写中性元 → 归约 → setup"（调用方负责 `submit`；三步在同一 encoder 内有序）。
    pub fn encode(&self, queue: &wgpu::Queue, enc: &mut wgpu::CommandEncoder) {
        // ⚠️ `write_buffer` 是**提交前**生效的队列操作 ⇒ 它一定落在下面的归约之前。
        queue.write_buffer(&self.bb_b, 0, &bb_init_bytes());
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bbox.stage.reduce"),
                timestamp_writes: None,
            });
            cp.set_pipeline(&self.reduce);
            cp.set_bind_group(0, &self.bg, &[]);
            cp.dispatch_workgroups(self.groups, 1, 1);
        }
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bbox.stage.setup"),
                timestamp_writes: None,
            });
            cp.set_pipeline(&self.setup);
            cp.set_bind_group(0, &self.bg, &[]);
            cp.dispatch_workgroups(1, 1, 1);
        }
    }

    /// setup 的产物缓冲（与 uniform 的动态字段同形）——调用方按需 `copy_buffer_to_buffer` 搬到 uniform。
    pub fn box_out(&self) -> &wgpu::Buffer {
        &self.box_out_b
    }

    /// 同步回读角点（**调用方需先 `submit` + `poll_wait`**；需先自己把 `bb` 拷进 `readback`）。
    pub fn read_corners(&self, device: &wgpu::Device) -> ([f32; 3], [f32; 3]) {
        let slice = self.readback.slice(..);
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
        let corners = bb_to_corners(decode_bb(&data));
        // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
        drop(data);
        self.readback.unmap();
        corners
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// GPU 用例**必须串行**：同一块卡上并发跑「建缓冲 + 提交 + 回读」会**挂死**（实测多次；
    /// `cargo test` 默认多线程跑测试 ⇒ 不加这把锁会偶发挂住整轮测试、连带门链与 CI）。
    /// 无适配器的机器上用例会快速跳过 ⇒ 锁不会造成等待。
    static GPU_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn gpu_guard() -> std::sync::MutexGuard<'static, ()> {
        GPU_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// 确定性伪随机位置（LCG；含负坐标与一条极扁的分布）。
    fn positions(n: usize) -> Vec<f32> {
        let mut s = 0x2545_F491_4F6C_DD1Du64;
        let mut out = Vec::with_capacity(n * 3);
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / 16_777_216.0
        };
        for i in 0..n {
            out.push(next() * 4.0 - 2.0);
            out.push(next() * 0.25 - 0.1);
            out.push(next() * 4.0 - 2.0 + (i % 7) as f32 * 0.01);
        }
        out
    }

    /// 小输入 + 手算期望：把归约的"覆盖"与"有序映射"两件事分开验（n < WG / n > WG 各一档）。
    /// 位置取 `(i, -2i, 1)` ⇒ min = `(0, -(n-1)*2, 1)`、max = `(n-1, 0, 1)`。
    #[test]
    fn gpu_reduce_covers_all_particles() {
        let _g = gpu_guard();
        for n in [3usize, 64, 65, 1000] {
            let mut pos = Vec::with_capacity(n * 3);
            for i in 0..n {
                pos.push(i as f32);
                pos.push(-(i as f32) * 2.0);
                pos.push(1.0);
            }
            let (hmin, hmax, _, _) = host_box(&pos, 0.05, 1 << 20);
            let out = box_on_adapter(0, &pos, 0.05, 1 << 20);
            if let Some(e) = out.error {
                eprintln!("跳过（本机无可用适配器）：{e}");
                return;
            }
            assert_eq!(out.min, hmin, "n={n}：min（bb={:?}）", out.bb);
            assert_eq!(out.max, hmax, "n={n}：max（bb={:?}）", out.bb);
        }
    }

    /// 上卡箱子必须与主机规则**逐位相同**（min/max/bin/dims/inv）。本机无适配器时打印跳过
    /// （CI 上 GPU 不可用 ⇒ 不能红）。
    #[test]
    fn gpu_box_matches_host_rule() {
        let _g = gpu_guard();
        let pos = positions(5_000);
        let h = 0.05f32;
        let max_bins = 1usize << 20;
        let (hmin, hmax, hbin, hdims) = host_box(&pos, h, max_bins);
        let out = box_on_adapter(0, &pos, h, max_bins);
        if let Some(e) = out.error {
            eprintln!("跳过（本机无可用适配器）：{e}");
            return;
        }
        assert_eq!(
            out.min, hmin,
            "min 角必须逐位相同（适配器：{}）",
            out.adapter
        );
        assert_eq!(out.max, hmax, "max 角必须逐位相同");
        assert_eq!(out.bin, hbin, "bin 必须逐位相同");
        assert_eq!(out.dims, hdims, "dims（整数）必须逐位相同");
        assert_eq!(
            out.total,
            hdims[0] * hdims[1] * hdims[2],
            "total = dims 之积"
        );
        assert_eq!(out.inv, 1.0 / hbin, "inv 与主机同式 ⇒ 逐位相同");
        // **卡上 setup**（`box_setup`）必须与主机规则一致：dims/total 逐位、inv ≤1 ulp。
        // 这条同时钉住 `o2f` 的**分支顺序**（首版写反 ⇒ 核解出的角点全乱，归约却是对的）。
        assert_eq!(
            out.gpu_dims, hdims,
            "卡上 setup 的 dims 必须逐位相同（核看到 lo/hi 见 gpu_* 诊断）"
        );
        assert_eq!(
            out.gpu_total,
            hdims[0] * hdims[1] * hdims[2],
            "卡上 setup 的 total 必须逐位相同"
        );
        assert!(
            (out.gpu_inv - 1.0 / hbin).abs() <= f32::EPSILON * (1.0 / hbin).abs(),
            "卡上 setup 的 inv 只允许 ≤1 ulp：gpu={} host={}",
            out.gpu_inv,
            1.0 / hbin
        );
    }

    /// 主机参考自身：与 `grid_box` 一致（薄包装，防"参考实现自己写歪"）。
    #[test]
    fn host_reference_uses_the_single_source_rule() {
        let pos = positions(1_000);
        let (lo, hi, bin, dims) = host_box(&pos, 0.05, 1 << 20);
        let (_, bin2, dims2) = grid_box(
            Vec3::new(lo[0], lo[1], lo[2]),
            Vec3::new(hi[0], hi[1], hi[2]),
            0.05,
            1 << 20,
        );
        assert_eq!(bin, bin2);
        assert_eq!(dims, dims2);
    }
}
