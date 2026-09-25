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

// 布局常量**只有一份**：定义在 `vxl_phys_core::narrow_tier`（卡上档的接口契约），这里重导出
// ⇒ 探针/门面写 `vxl_phys_gpu::narrow::SLOT_WORDS` 与写 core 那份是同一个值。
pub use vxl_phys_core::narrow_tier::{
    flat_pairs, BODY_WORDS, KIND_BOX, KIND_NONE, KIND_SPHERE, NOT_HANDLED, SLOT_BYTES, SLOT_WORDS,
};

/// 一排线程数（64）。
/// ⚠️ **已否证的假设**（§17.9 补记五）：曾疑"动态索引的局部数组（`sat_boxes` 的 `axes`、裁剪多边形）
/// 被后端降到共享内存 ⇒ 同 workgroup 内 invocation 互相踩"。把本值改成 **1** 重跑 m1 档：24 条几何不符
/// **一条不少、数值完全相同** ⇒ **invocation 间干扰被排除**（容器布局无关）。
const WG: u32 = 64;
/// 体记录里 `kind` 的字偏移（裸整数：1 = 球、2 = 盒、0 = 本档不接手）。
pub const BODY_KIND_AT: usize = 7;
/// 体记录里 `p0`（形状参数 0）的字偏移（f32 位模式）。
pub const BODY_P0_AT: usize = 8;
/// 诊断字个数（`[0]` = 裁剪多边形越界次数）。
pub const DIAG_WORDS: usize = 2;
/// 护栏：体数/对数上限（超了直接报错，让调用方回退 CPU）。
const MAX_ITEMS: u32 = 1 << 22;

/// **卡上档的分项计时**（诊断；`PLAN-gpu.md` §17.10 补记）。
///
/// 为什么要**进程级**而不是放进 `NarrowTier`：门面把档 `Box<dyn Trait>` 移进了 `World`，探针拿不到
/// 那个对象 ⇒ 只有在卡上档内部累加、任何探针都能读，才能把"World 路径比直调慢 2 ms"这件事拆开。
/// 代价：每趟 8 次 `fetch_add`（~100 ns），相对 ms 级分项可忽略。
///
/// 索引含义：`[0]` 变动打包(idx/rec) `[1]` 两次 `write_buffer` `[2]` 参数/对表写
/// `[3]` 编码+提交 `[4]` `map_async`+`poll(Wait)`（含等 GPU）`[5]` 取映射+解码 `[6]` `flatten`
/// `[7]` 调用次数。
pub mod prof {
    use std::sync::atomic::{AtomicU64, Ordering};

    pub static W: [AtomicU64; 8] = [
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
    ];

    /// 累加一项（µs）。
    pub fn add(i: usize, us: u64) {
        W[i].fetch_add(us, Ordering::Relaxed);
    }

    /// 读一项（µs）。
    pub fn us(i: usize) -> u64 {
        W[i].load(Ordering::Relaxed)
    }

    /// 清零（探针在量某一条链**之前**调）。
    pub fn reset() {
        for a in &W {
            a.store(0, Ordering::Relaxed);
        }
    }
}

/// 分项计时的**读数**（`prof` 的 8 项，µs）。
pub fn prof_snapshot() -> [u64; 8] {
    let mut o = [0u64; 8];
    for (i, v) in o.iter_mut().enumerate() {
        *v = prof::us(i);
    }
    o
}

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
    /// `[0]` = 主核（窄相）、`[1]` = `scatter_records`（常驻体表的变动散写）。
    pipes: [wgpu::ComputePipeline; 2],
    bufs: Buffers,
    cap_bodies: u32,
    cap_pairs: u32,
    skin: f32,
    /// 选点去重间距（与 CPU `DefaultNarrowPhase::new` 同式：`max(skin*2, 0.01)`）。
    min_sep: f32,
    /// **卡上体表的当前行数**（常驻档用：`run_resident` 不再传体表，行数从这里取）。
    resident_n: std::sync::atomic::AtomicU32,
    /// 待散写的**变动记录数**（`upload_records` 记账；下一趟在 `run_common` 里消费）。
    resident_upd: std::sync::atomic::AtomicU32,
}

/// 常驻缓冲清单（体表/对表按**容量**开 ⇒ 每 tick 只重写前缀）。
struct Buffers {
    params: wgpu::Buffer,
    bodies: wgpu::Buffer,
    pairs: wgpu::Buffer,
    slots: wgpu::Buffer,
    rb: wgpu::Buffer,
    diag: wgpu::Buffer,
    /// 变动记录（常驻体表的增量）：`upd_idx` = 体号、`upd_rec` = 12 字/条。
    upd_idx: wgpu::Buffer,
    upd_rec: wgpu::Buffer,
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
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
        bodies: mk(
            "narrow.bodies",
            (cap_bodies as u64) * (BODY_WORDS as u64) * 4,
        ),
        pairs: mk("narrow.pairs", (cap_pairs as u64) * 8),
        slots: mk("narrow.slots", slots_bytes),
        upd_idx: mk("narrow.upd_idx", (cap_bodies as u64) * 4),
        upd_rec: mk(
            "narrow.upd_rec",
            (cap_bodies as u64) * (BODY_WORDS as u64) * 4,
        ),
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
) -> (wgpu::BindGroup, [wgpu::ComputePipeline; 2]) {
    let ro = wgpu::BufferBindingType::Storage { read_only: true };
    let rw = wgpu::BufferBindingType::Storage { read_only: false };
    let types = [wgpu::BufferBindingType::Uniform, rw, ro, rw, rw, ro, ro];
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
    let mk_pipe = |entry: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let pipes = [mk_pipe("main"), mk_pipe("scatter_records")];
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
                e(5, &bufs.upd_idx),
                e(6, &bufs.upd_rec),
            ],
        })
    };
    (bg, pipes)
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
        let (bg, pipes) = build_pipeline(&device, &bufs);
        Ok(Self {
            adapter,
            device,
            queue,
            bg,
            pipes,
            bufs,
            cap_bodies,
            cap_pairs,
            skin,
            // 与 CPU `DefaultNarrowPhase::new` 同式。
            min_sep: (skin * 2.0).max(0.01),
            resident_n: std::sync::atomic::AtomicU32::new(0),
            resident_upd: std::sync::atomic::AtomicU32::new(0),
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
        if !bodies.len().is_multiple_of(BODY_WORDS) {
            return Err(format!(
                "体表长度不是整条：bodies={}（{} 字/体）",
                bodies.len(),
                BODY_WORDS
            ));
        }
        self.run_common((bodies.len() / BODY_WORDS) as u32, pairs, Some(bodies))
    }

    /// **常驻体表**：登记"哪些体的记录变了"，由下一趟**同一次提交**里的 `scatter_records` 散写进卡上
    /// 常驻体表（体表此后不必整表上传）。
    ///
    /// 为什么要"一次大上传 + 卡上散写"而不是逐段 `write_buffer`：**代价按调用次数算**（实测 515 KB
    /// 一次 ≈ 0.243 ms，而 35 KB 分多次 ≈ 0.176 ms ⇒ 只快 1.4×）——把变动折成 `(体号, 记录)` 两段
    /// 连续数据、一次写完，散写在卡上是 µs 级（§17.10）。
    pub fn upload_records(&self, bodies: &[u32], changed: &[u32]) -> Result<(), String> {
        if !bodies.len().is_multiple_of(BODY_WORDS) {
            return Err(format!("体表长度不是整条：bodies={}", bodies.len()));
        }
        let n_bodies = (bodies.len() / BODY_WORDS) as u32;
        if n_bodies > self.cap_bodies || changed.len() as u32 > self.cap_bodies {
            return Err(format!("超容量：体 {n_bodies}/{}", self.cap_bodies));
        }
        if changed.is_empty() {
            self.resident_upd
                .store(0, std::sync::atomic::Ordering::Relaxed);
            self.resident_n
                .store(n_bodies, std::sync::atomic::Ordering::Relaxed);
            return Ok(());
        }
        // 打包成两段**连续**数据：`idx`（体号）与 `rec`（12 字/条）⇒ 两次 `write_buffer`。
        let t0 = std::time::Instant::now();
        let mut idx: Vec<u8> = Vec::with_capacity(changed.len() * 4);
        let mut rec: Vec<u8> = Vec::with_capacity(changed.len() * BODY_WORDS * 4);
        for &i in changed {
            idx.extend_from_slice(&i.to_le_bytes());
            let a = (i as usize) * BODY_WORDS;
            rec.extend_from_slice(&words_to_bytes(&bodies[a..a + BODY_WORDS]));
        }
        let d = t0.elapsed().as_micros() as u64;
        prof::add(0, d);
        let t1 = std::time::Instant::now();
        self.queue.write_buffer(&self.bufs.upd_idx, 0, &idx);
        self.queue.write_buffer(&self.bufs.upd_rec, 0, &rec);
        prof::add(1, t1.elapsed().as_micros() as u64);
        self.resident_upd
            .store(changed.len() as u32, std::sync::atomic::Ordering::Relaxed);
        self.resident_n
            .store(n_bodies, std::sync::atomic::Ordering::Relaxed);
        Ok(())
    }

    /// **常驻模式的一趟**：体表已在卡上（见 `upload_records`）⇒ 只吃对表。
    pub fn run_resident(&self, pairs: &[u32]) -> Result<NarrowRun, String> {
        let n = self.resident_n.load(std::sync::atomic::Ordering::Relaxed);
        self.run_common(n, pairs, None)
    }

    /// 公共体：校验 → 写参数/对表（体表**可选**）→ 派发 → 回读。
    fn run_common(
        &self,
        n_bodies: u32,
        pairs: &[u32],
        bodies: Option<&[u32]>,
    ) -> Result<NarrowRun, String> {
        if !pairs.len().is_multiple_of(2) {
            return Err(format!(
                "对表长度不是整条：pairs={}（2 字/对）",
                pairs.len()
            ));
        }
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
        // （后两个是 f32 位模式：两者都进几何判定 ⇒ 必须是**同一份** CPU 侧值的位）；第 5 槽 = 本趟要
        // 散写的**变动记录数**（0 = 跳过；由 `upload_records` 记账）。
        let n_upd = self.resident_upd.load(std::sync::atomic::Ordering::Relaxed);
        let mut prm: Vec<u8> = Vec::with_capacity(32);
        prm.extend_from_slice(&n_bodies.to_le_bytes());
        prm.extend_from_slice(&n_pairs.to_le_bytes());
        prm.extend_from_slice(&self.skin.to_bits().to_le_bytes());
        prm.extend_from_slice(&self.min_sep.to_bits().to_le_bytes());
        prm.extend_from_slice(&n_upd.to_le_bytes());
        prm.extend_from_slice(&[0u8; 12]);
        self.queue.write_buffer(&self.bufs.params, 0, &prm);
        // 体表：常驻档**不写**（已在卡上，见 `upload_records`）。
        if let Some(b) = bodies {
            self.queue
                .write_buffer(&self.bufs.bodies, 0, &words_to_bytes(b));
        }
        let t_prm = std::time::Instant::now();
        self.queue
            .write_buffer(&self.bufs.pairs, 0, &words_to_bytes(pairs));
        prof::add(2, t_prm.elapsed().as_micros() as u64);
        prof::add(7, 1); // 趟数（每趟 `run_common` 记 1 ⇒ 读数分项都是"每趟均值"）
        let t_enc = std::time::Instant::now();
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("narrow.enc"),
            });
        // 诊断字每趟清零（原子累加 ⇒ 不清就会跨趟累计，读数失去"本趟"含义）。
        enc.clear_buffer(&self.bufs.diag, 0, None);
        // **0) 变动记录散写**（常驻体表；`n_upd = 0` 时整趟跳过 ⇒ 非常驻档零影响）：
        // 与主核**同一次提交**（省一趟同步），且在上传的同一编码器里、主核之前。
        if n_upd > 0 {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: None,
                timestamp_writes: None,
            });
            cp.set_pipeline(&self.pipes[1]);
            cp.set_bind_group(0, &self.bg, &[]);
            cp.dispatch_workgroups(n_upd.div_ceil(WG).max(1), 1, 1);
        }
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: None,
                timestamp_writes: None,
            });
            cp.set_pipeline(&self.pipes[0]);
            cp.set_bind_group(0, &self.bg, &[]);
            cp.dispatch_workgroups(n_pairs.div_ceil(WG).max(1), 1, 1);
        }
        prof::add(3, t_enc.elapsed().as_micros() as u64);
        let (out, diag) = self.finish(enc, n_pairs);
        Ok(NarrowRun { slots: out, diag })
    }

    /// **收尾一趟**：`slots → rb` 拷贝 + `diag` 拷贝 + 提交 + `map_async` + `poll(Wait)` + 解码。
    ///
    /// 单独成函数有两个理由：① 尺寸门（`run_common` 曾 129 行 > 120）；② 这一段的**每一项**都是
    /// 分项计时（`prof`）的对象——"提交/等"与"解码"拆开后才能各归各的账（§17.10 补记的实测：
    /// 这一档每趟代价的 70–88% 就在 `提交+映射+等`，不是算、不是带宽、不是解码）。
    ///
    /// **解码走一次过**：逐字 `data[o]`、`data[o+1]`… 的取法实测慢一倍（m1 档 0.124 → 0.061 ms，
    /// m4 档 1.09 → 0.50 ms；输出字表**逐位相同**，见 `gpu_narrow_up_probe` 的 ⑨/⑩）。
    fn finish(
        &self,
        mut enc: wgpu::CommandEncoder,
        n_pairs: u32,
    ) -> (Vec<Slot>, [u32; DIAG_WORDS]) {
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
        let t_wait = std::time::Instant::now();
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
        prof::add(4, t_wait.elapsed().as_micros() as u64);
        let t_dec = std::time::Instant::now();
        let data = slice.get_mapped_range();
        let mut out: Vec<Slot> = Vec::with_capacity(n_pairs as usize);
        let mut words_it = data[..(n_pairs as usize) * SLOT_BYTES].chunks_exact(4);
        for _ in 0..n_pairs {
            let mut words = [0u32; SLOT_WORDS];
            for w in words.iter_mut() {
                // `chunks_exact(4)` 已保证每块 4 字节 ⇒ 这里的 `expect` 不会触发（不留 panic 面）。
                let c = words_it.next().expect("chunks_exact 每块 4 字节");
                *w = u32::from_le_bytes([c[0], c[1], c[2], c[3]]);
            }
            out.push(Slot { words });
        }
        let rd = |o: usize| u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let mut diag = [0u32; DIAG_WORDS];
        for (k, d) in diag.iter_mut().enumerate() {
            *d = rd(diag_at as usize + k * 4);
        }
        drop(data);
        self.bufs.rb.unmap();
        prof::add(5, t_dec.elapsed().as_micros() as u64);
        (out, diag)
    }
}

/// **卡上档的接口实现**（`vxl_phys_core::narrow_tier` 的契约）：把逐槽的 `Slot` 摊成**裸字表**，
/// 门面那边只认字、不认几何类型 ⇒ 卡上档不把任何物理知识带进 core。
impl vxl_phys_core::narrow_tier::NarrowTierBackend for NarrowTier {
    fn narrow_run(
        &self,
        bodies: &[u32],
        pairs: &[u32],
    ) -> Result<vxl_phys_core::narrow_tier::NarrowSlots, String> {
        let r = self.run(bodies, pairs)?;
        Ok(flatten(r))
    }

    /// 常驻体表：只写 `changed` 的记录（§17.10；整表 528 KB/tick 在 m1 档比窄相本身还贵）。
    fn narrow_resident(&self, bodies: &[u32], changed: &[u32]) -> Option<()> {
        self.upload_records(bodies, changed).ok()
    }

    fn narrow_run_resident(
        &self,
        pairs: &[u32],
    ) -> Result<vxl_phys_core::narrow_tier::NarrowSlots, String> {
        Ok(flatten(self.run_resident(pairs)?))
    }
}

/// 逐槽 → 裸字表（门面只认字）。
fn flatten(r: NarrowRun) -> vxl_phys_core::narrow_tier::NarrowSlots {
    let t = std::time::Instant::now();
    let mut words = Vec::with_capacity(r.slots.len() * SLOT_WORDS);
    for s in &r.slots {
        words.extend_from_slice(&s.words);
    }
    let out = vxl_phys_core::narrow_tier::NarrowSlots {
        words,
        diag: r.diag,
    };
    prof::add(6, t.elapsed().as_micros() as u64);
    out
}
