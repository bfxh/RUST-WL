//! **窄相（卡上）：固定槽 + 主机回填**（`PLAN-gpu.md` §17.5 第二片）。
//!
//! 形状（与宽相那一片同族）：**逐对一槽、对序即槽序** ⇒ 流形表结构天然确定，不需要重排；
//! 卡上只接**能对上的那一批几何**（本片 = 球×球），其余对由核写 `NOT_HANDLED` 哨兵、
//! **主机回填同一槽位**（调用方逐对走：槽有流形就用槽、是哨兵就调 CPU 窄相）⇒ 最终表仍是对序。
//!
//! **判据形态**（比宽相弱一档，弱在哪里要清楚）：宽相只吃整数格坐标 + f32 比较 ⇒ 逐位可同；
//! 窄相要走 `sqrt`/除法/乘加 ⇒ 卡上浮点**只能到口径 B**（§9.6：naga 无 `precise`，FMA 收缩
//! 挡不住）⇒ 判据 = 逐对 `max|Δpoint|/Δdepth/Δnormal|` **容差** + `feature`/点数/`a`/`b`
//! **逐位**（那几项是整数）。
//!
//! **本档是显式档**：不接进 `World` 的默认路径（默认档一行都不走新路径，§17.3 铁律）。
//! 与宽相那片**有意的接口差别**：这里给的是**可复用档**（`NarrowTier`）而不是一次性函数——
//! 缓冲要跨 tick 常驻（每 tick 只重写体表/对表 + 读槽），否则一次 `Device` 创建就贵过核本身。

use crate::probe::device_for;

const WG: u32 = 64;
/// 槽字数（u32）：`a|b|normal.xyz|count` + 4×`(point.xyz|depth|feature)` = 6 + 20。
pub const SLOT_WORDS: usize = 26;
/// 槽字节数（104 B/对）。**与 `narrow.wgsl` 的 `SLOT_WORDS` 必须同值**。
pub const SLOT_BYTES: usize = SLOT_WORDS * 4;
/// 逐体输入字数（48 B/体）：`pos.xyz | rot.xyzw | kind | p0 | p1 | p2 | pad`。
pub const BODY_WORDS: usize = 12;
/// 体记录里 `kind` 的字偏移（裸整数：1 = 球、0 = 本档不接手）。
pub const BODY_KIND_AT: usize = 7;
/// 体记录里 `p0`（形状参数 0；球 = 半径）的字偏移（f32 位模式）。
pub const BODY_P0_AT: usize = 8;
/// `count` 哨兵：本档不接手该对 ⇒ 调用方按**主机回填**处理（别当成"无流形"）。
pub const NOT_HANDLED: u32 = u32::MAX;
/// 体种类：球（`p0` = 半径）。其余形状/复合体一律 0 = 不接手（留给后续族按分派表补）。
pub const KIND_SPHERE: u32 = 1;
/// 护栏：体数/对数上限（超了直接报错，让调用方回退 CPU）。
const MAX_ITEMS: u32 = 1 << 22;

/// 逐对槽（主机侧视图 = 卡上 26 字的**逐字镜像** ⇒ 比较与装配都在同一份数据上做）。
#[derive(Clone, Copy, Debug)]
pub struct Slot {
    pub words: [u32; SLOT_WORDS],
}

impl Slot {
    #[inline]
    fn f(&self, w: usize) -> f32 {
        f32::from_bits(self.words[w])
    }

    /// 对的两个体号（核原样回写 ⇒ 与 CPU 流形可逐位对）。
    #[inline]
    pub fn pair(&self) -> (u32, u32) {
        (self.words[0], self.words[1])
    }

    /// 点数：0 = 无流形、1..=4 = 点数、`NOT_HANDLED` = 本档不接手。
    #[inline]
    pub fn count(&self) -> u32 {
        self.words[5]
    }

    /// 流形法线（a→b）。
    #[inline]
    pub fn normal(&self) -> [f32; 3] {
        [self.f(2), self.f(3), self.f(4)]
    }

    /// 第 `k` 个接触点 = `(point, depth, feature)`（`k < count`）。
    #[inline]
    pub fn point(&self, k: usize) -> ([f32; 3], f32, u32) {
        let o = 6 + k * 5;
        (
            [self.f(o), self.f(o + 1), self.f(o + 2)],
            self.f(o + 3),
            self.words[o + 4],
        )
    }
}

/// 卡上窄相档：设备/管线/缓冲常驻，`run` 只重写体表/对表并回读槽。
pub struct NarrowTier {
    adapter: String,
    device: wgpu::Device,
    queue: wgpu::Queue,
    bg: wgpu::BindGroup,
    pipe: wgpu::ComputePipeline,
    params_b: wgpu::Buffer,
    bodies_b: wgpu::Buffer,
    pairs_b: wgpu::Buffer,
    slots_b: wgpu::Buffer,
    rb: wgpu::Buffer,
    cap_bodies: u32,
    cap_pairs: u32,
}

/// u32 表 → 小端字节（`write_buffer` 要 `&[u8]`）。
fn words_to_bytes(w: &[u32]) -> Vec<u8> {
    w.iter().flat_map(|x| x.to_le_bytes()).collect()
}

/// 建全部缓冲（体表/对表按**容量**开 ⇒ 每 tick 只重写前缀）。
fn make_bufs(
    device: &wgpu::Device,
    cap_bodies: u32,
    cap_pairs: u32,
) -> (
    wgpu::Buffer,
    wgpu::Buffer,
    wgpu::Buffer,
    wgpu::Buffer,
    wgpu::Buffer,
) {
    let mk = |label: &str, size: u64| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: size.max(4),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        })
    };
    let slots_bytes = (cap_pairs as u64) * (SLOT_BYTES as u64);
    (
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("narrow.params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
        mk(
            "narrow.bodies",
            (cap_bodies as u64) * (BODY_WORDS as u64) * 4,
        ),
        mk("narrow.pairs", (cap_pairs as u64) * 8),
        mk("narrow.slots", slots_bytes),
        // ⚠️ 回读缓冲**单独建**：`MAP_READ` 只能与 `COPY_DST` 组合（把它当 `mk` 的附加位塞进去
        // ⇒ wgpu 校验当场报错：`MAP` usage can only be combined with the opposite `COPY`）。
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("narrow.rb"),
            size: slots_bytes.max(4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        }),
    )
}

/// 布局 + 管线 + bind group。
fn build_pipeline(
    device: &wgpu::Device,
    params_b: &wgpu::Buffer,
    bodies_b: &wgpu::Buffer,
    pairs_b: &wgpu::Buffer,
    slots_b: &wgpu::Buffer,
) -> (wgpu::BindGroup, wgpu::ComputePipeline) {
    let ro = wgpu::BufferBindingType::Storage { read_only: true };
    let rw = wgpu::BufferBindingType::Storage { read_only: false };
    let types = [wgpu::BufferBindingType::Uniform, ro, ro, rw];
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("narrow.bgl"),
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
    // ⚠️ 这个变量**必须叫 `shader`**：本仓的词汇门只豁免"计算管线描述符里那一字段取形参
    // `shader`"这一种写法（见 `scripts/vocab_scan.sh` 头部）——改名会红。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("narrow.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("narrow.wgsl").into()),
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("narrow.pl"),
        bind_group_layouts: &[&layout],
        push_constant_ranges: &[],
    });
    let pipe = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("narrow.main"),
        layout: Some(&pl),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: Default::default(),
        cache: None,
    });
    let bg = {
        fn e<'a>(b: u32, bf: &'a wgpu::Buffer) -> wgpu::BindGroupEntry<'a> {
            wgpu::BindGroupEntry {
                binding: b,
                resource: bf.as_entire_binding(),
            }
        }
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("narrow.bg"),
            layout: &layout,
            entries: &[e(0, params_b), e(1, bodies_b), e(2, pairs_b), e(3, slots_b)],
        })
    };
    (bg, pipe)
}

impl NarrowTier {
    /// 在**指定适配器序号**上建档（`cap_bodies`/`cap_pairs` = 缓冲容量，必须 ≥ 后续每次 `run` 的规模）。
    pub fn new(adapter_index: usize, cap_bodies: u32, cap_pairs: u32) -> Result<Self, String> {
        if cap_bodies == 0 || cap_pairs == 0 || cap_bodies > MAX_ITEMS || cap_pairs > MAX_ITEMS {
            return Err(format!(
                "容量不合法：cap_bodies={cap_bodies}、cap_pairs={cap_pairs}（要 1..={MAX_ITEMS}）"
            ));
        }
        let (adapter, device, queue) = device_for(adapter_index)?;
        let (params_b, bodies_b, pairs_b, slots_b, rb) = make_bufs(&device, cap_bodies, cap_pairs);
        let (bg, pipe) = build_pipeline(&device, &params_b, &bodies_b, &pairs_b, &slots_b);
        Ok(Self {
            adapter,
            device,
            queue,
            bg,
            pipe,
            params_b,
            bodies_b,
            pairs_b,
            slots_b,
            rb,
            cap_bodies,
            cap_pairs,
        })
    }

    /// 适配器名（诊断）。
    pub fn adapter(&self) -> &str {
        &self.adapter
    }

    /// 一趟：写体表/对表 → 派发 → 回读槽。返回**对序**的槽表（长度 = 对数）。
    ///
    /// `bodies` = 逐体 12 字（`BODY_WORDS`）、`pairs` = 逐对 2 字（`(a, b)` 裸 u32）。
    pub fn run(&self, bodies: &[u32], pairs: &[u32]) -> Result<Vec<Slot>, String> {
        if !bodies.len().is_multiple_of(BODY_WORDS) || !pairs.len().is_multiple_of(2) {
            return Err(format!(
                "输入长度不是整条：bodies={}（{} 字/体）、pairs={}（2 字/对）",
                bodies.len(),
                BODY_WORDS,
                pairs.len()
            ));
        }
        let n_bodies = (bodies.len() / BODY_WORDS) as u32;
        let n_pairs = (pairs.len() / 2) as u32;
        if n_bodies > self.cap_bodies || n_pairs > self.cap_pairs {
            return Err(format!(
                "超容量：体 {n_bodies}/{cap_b}、对 {n_pairs}/{cap_p}（调用方应回退 CPU）",
                cap_b = self.cap_bodies,
                cap_p = self.cap_pairs
            ));
        }
        // 空对表直接返回：**别去映射 0 字节**（`map_async` 对零长区间不保证成立），
        // 且真实 tick 里"一只都没碰"是常态（不必为它冒一次校验风险）。
        if n_pairs == 0 {
            return Ok(Vec::new());
        }
        // 每 tick 重写参数与两张表（容量不变 ⇒ 缓冲不重建）。
        let mut prm: Vec<u8> = Vec::with_capacity(16);
        prm.extend_from_slice(&n_bodies.to_le_bytes());
        prm.extend_from_slice(&n_pairs.to_le_bytes());
        prm.extend_from_slice(&[0u8; 8]);
        self.queue.write_buffer(&self.params_b, 0, &prm);
        self.queue
            .write_buffer(&self.bodies_b, 0, &words_to_bytes(bodies));
        self.queue
            .write_buffer(&self.pairs_b, 0, &words_to_bytes(pairs));
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("narrow.enc"),
            });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: None,
                timestamp_writes: None,
            });
            cp.set_pipeline(&self.pipe);
            cp.set_bind_group(0, &self.bg, &[]);
            cp.dispatch_workgroups(n_pairs.div_ceil(WG).max(1), 1, 1);
        }
        let used = (n_pairs as u64) * (SLOT_BYTES as u64);
        if used > 0 {
            enc.copy_buffer_to_buffer(&self.slots_b, 0, &self.rb, 0, used);
        }
        self.queue.submit(Some(enc.finish()));
        let slice = self.rb.slice(..used);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        let out = (0..n_pairs as usize)
            .map(|i| {
                let b = i * SLOT_BYTES;
                let mut words = [0u32; SLOT_WORDS];
                for (k, w) in words.iter_mut().enumerate() {
                    let o = b + k * 4;
                    *w = u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
                }
                Slot { words }
            })
            .collect();
        drop(data);
        self.rb.unmap();
        Ok(out)
    }
}
