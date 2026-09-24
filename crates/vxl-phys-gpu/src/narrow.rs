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
/// 体种类：球（`p0` = 半径）。其余形状一律 0 = 不接手（留给后续族按分派表补）。
pub const KIND_SPHERE: u32 = 1;
/// 体种类：盒（`p0..p2` = `half.xyz`；`rot` 走记录里的四元数）。
pub const KIND_BOX: u32 = 2;
/// 护栏：体数/对数上限（超了直接报错，让调用方回退 CPU）。
const MAX_ITEMS: u32 = 1 << 22;
/// 诊断字个数（`[0]` = 裁剪多边形越界次数）。
pub const DIAG_WORDS: usize = 2;

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
    bufs: Buffers,
    cap_bodies: u32,
    cap_pairs: u32,
    skin: f32,
    /// 选点去重间距（与 CPU `DefaultNarrowPhase::new` 同式：`max(skin*2, 0.01)`）。
    min_sep: f32,
}

/// 常驻缓冲清单（体表/对表按**容量**开 ⇒ 每 tick 只重写前缀）。
struct Buffers {
    params: wgpu::Buffer,
    bodies: wgpu::Buffer,
    pairs: wgpu::Buffer,
    slots: wgpu::Buffer,
    rb: wgpu::Buffer,
    diag: wgpu::Buffer,
}

/// 一趟读数：对序槽表 + 诊断字（`[0]` = 裁剪多边形越界次数，**必须 0**）。
pub struct NarrowRun {
    pub slots: Vec<Slot>,
    pub diag: [u32; DIAG_WORDS],
}

/// u32 表 → 小端字节（`write_buffer` 要 `&[u8]`）。
fn words_to_bytes(w: &[u32]) -> Vec<u8> {
    w.iter().flat_map(|x| x.to_le_bytes()).collect()
}

/// 建全部缓冲（体表/对表按**容量**开 ⇒ 每 tick 只重写前缀）。
fn make_bufs(device: &wgpu::Device, cap_bodies: u32, cap_pairs: u32) -> Buffers {
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
    Buffers {
        params: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("narrow.params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
        bodies: mk(
            "narrow.bodies",
            (cap_bodies as u64) * (BODY_WORDS as u64) * 4,
        ),
        pairs: mk("narrow.pairs", (cap_pairs as u64) * 8),
        slots: mk("narrow.slots", slots_bytes),
        // ⚠️ 回读缓冲**单独建**：`MAP_READ` 只能与 `COPY_DST` 组合（把它当 `mk` 的附加位塞进去
        // ⇒ wgpu 校验当场报错：`MAP` usage can only be combined with the opposite `COPY`）。
        rb: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("narrow.rb"),
            // 尾部留 DIAG_WORDS 个字的槽给诊断（跟槽表同一次回读，省一趟同步）。
            size: (slots_bytes + (DIAG_WORDS as u64) * 4).max(4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        }),
        // 诊断（原子累加）：`[0]` = 裁剪多边形越界次数 —— 每趟在 `run` 里清零。
        diag: mk("narrow.diag", (DIAG_WORDS as u64) * 4),
    }
}

/// 布局 + 管线 + bind group。
fn build_pipeline(
    device: &wgpu::Device,
    bufs: &Buffers,
) -> (wgpu::BindGroup, wgpu::ComputePipeline) {
    let ro = wgpu::BufferBindingType::Storage { read_only: true };
    let rw = wgpu::BufferBindingType::Storage { read_only: false };
    let types = [wgpu::BufferBindingType::Uniform, ro, ro, rw, rw];
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
            entries: &[
                e(0, &bufs.params),
                e(1, &bufs.bodies),
                e(2, &bufs.pairs),
                e(3, &bufs.slots),
                e(4, &bufs.diag),
            ],
        })
    };
    (bg, pipe)
}

impl NarrowTier {
    /// 在**指定适配器序号**上建档（`cap_bodies`/`cap_pairs` = 缓冲容量，必须 ≥ 后续每次 `run` 的规模）。
    /// `skin` = 接触投机带（与 CPU `DefaultNarrowPhase::new(skin)` **同一个值**，否则两侧判据不同源）。
    pub fn new(
        adapter_index: usize,
        cap_bodies: u32,
        cap_pairs: u32,
        skin: f32,
    ) -> Result<Self, String> {
        if cap_bodies == 0 || cap_pairs == 0 || cap_bodies > MAX_ITEMS || cap_pairs > MAX_ITEMS {
            return Err(format!(
                "容量不合法：cap_bodies={cap_bodies}、cap_pairs={cap_pairs}（要 1..={MAX_ITEMS}）"
            ));
        }
        let (adapter, device, queue) = device_for(adapter_index)?;
        let bufs = make_bufs(&device, cap_bodies, cap_pairs);
        let (bg, pipe) = build_pipeline(&device, &bufs);
        Ok(Self {
            adapter,
            device,
            queue,
            bg,
            pipe,
            bufs,
            cap_bodies,
            cap_pairs,
            skin,
            // 与 CPU `DefaultNarrowPhase::new` 同式。
            min_sep: (skin * 2.0).max(0.01),
        })
    }

    /// 适配器名（诊断）。
    pub fn adapter(&self) -> &str {
        &self.adapter
    }

    /// 一趟：写体表/对表 → 派发 → 回读槽 + 诊断。返回**对序**的槽表（长度 = 对数）。
    ///
    /// `bodies` = 逐体 12 字（`BODY_WORDS`）、`pairs` = 逐对 2 字（`(a, b)` 裸 u32）。
    pub fn run(&self, bodies: &[u32], pairs: &[u32]) -> Result<NarrowRun, String> {
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
            return Ok(NarrowRun {
                slots: Vec::new(),
                diag: [0; DIAG_WORDS],
            });
        }
        // 每 tick 重写参数与两张表（容量不变 ⇒ 缓冲不重建）。参数 = `n_bodies|n_pairs|skin|min_sep`
        // （后两个是 f32 位模式：两者都进几何判定 ⇒ 必须是**同一份** CPU 侧值的位）。
        let mut prm: Vec<u8> = Vec::with_capacity(16);
        prm.extend_from_slice(&n_bodies.to_le_bytes());
        prm.extend_from_slice(&n_pairs.to_le_bytes());
        prm.extend_from_slice(&self.skin.to_bits().to_le_bytes());
        prm.extend_from_slice(&self.min_sep.to_bits().to_le_bytes());
        self.queue.write_buffer(&self.bufs.params, 0, &prm);
        self.queue
            .write_buffer(&self.bufs.bodies, 0, &words_to_bytes(bodies));
        self.queue
            .write_buffer(&self.bufs.pairs, 0, &words_to_bytes(pairs));
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("narrow.enc"),
            });
        // 诊断字每趟清零（原子累加 ⇒ 不清就会跨趟累计，读数失去"本趟"含义）。
        enc.clear_buffer(&self.bufs.diag, 0, None);
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
        let diag_at = used;
        if used > 0 {
            enc.copy_buffer_to_buffer(&self.bufs.slots, 0, &self.bufs.rb, 0, used);
        }
        enc.copy_buffer_to_buffer(
            &self.bufs.diag,
            0,
            &self.bufs.rb,
            diag_at,
            (DIAG_WORDS as u64) * 4,
        );
        self.queue.submit(Some(enc.finish()));
        let slice = self.bufs.rb.slice(..used + (DIAG_WORDS as u64) * 4);
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
        let rd = |o: usize| u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let mut diag = [0u32; DIAG_WORDS];
        for (k, d) in diag.iter_mut().enumerate() {
            *d = rd(diag_at as usize + k * 4);
        }
        drop(data);
        self.bufs.rb.unmap();
        Ok(NarrowRun { slots: out, diag })
    }
}
