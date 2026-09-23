//! types：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 单接触点约束（预计算质量项与软接触目标/正则化）。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct PointConstraint {
    pub(crate) ra: Vec3,
    pub(crate) rb: Vec3,
    pub(crate) t1: Vec3,
    pub(crate) t2: Vec3,
    pub(crate) nmass: f32,
    pub(crate) tmass1: f32,
    pub(crate) tmass2: f32,
    /// 切向 2×2 矩阵交叉项 k12（联立解用；见 `contact_mass_cross`）。
    pub(crate) tcross: f32,
    /// 切向锚点漂移回拉目标（Rapier 切向 rhs 同义：`v_t → (pa−pb)·t·inv_dt`，
    /// 由摩擦在锥内执行；回收锚点使其为真实材料点错位，非裁剪几何噪声）。
    pub(crate) trhs1: f32,
    pub(crate) trhs2: f32,
    /// 局部锚点与烘焙深度（warm 回写 / 下一帧距离推进）；`depth0` =
    /// 烘焙时深度（回写时必须存它，见 `WarmPoint::depth0` 注）。
    pub(crate) la: Vec3,
    pub(crate) lb: Vec3,
    pub(crate) depth0: f32,
    /// M1 软接触（TGS-Soft 语义，与 Rapier 0.35 同构）：法向目标分离速度
    /// rhs = 去穿透速率（erp·depth，钳 ±max_corrective_velocity）− speculative
    /// 项（浅缝允许一个 tick 内闭合）+ 弹性目标（e·vn，e>0 时）。
    /// 旧「硬约束 + 分裂冲量独立通道」方案已被本形态取代（金样定标见
    /// docs/M1-PLAN.md 第五段：软接触是深堆稳定的结构性关键）。
    pub(crate) rhs: f32,
    /// 正则化因子（CFM）：穿透接触 cfm=1（硬投影，保证支撑刚性）；
    /// speculative（depth<0）接触 cfm<1（等效柔度 ω/ζ，Rapier 默认
    /// 30 Hz 动/60 Hz 静态、ζ=10）——限制迭代增益、深堆不依赖跨层链收敛。
    pub(crate) cfm: f32,
    pub(crate) friction: f32,
    pub(crate) pn: f32,
    pub(crate) pt1: f32,
    pub(crate) pt2: f32,
    /// 接触特征 ID（窄相来；warm 缓存回写用）。
    pub(crate) feature: u32,
    pub(crate) warm: Option<WarmPoint>,
}

/// 软接触求解参数（每步预计算；由 `PhysConfig` 的 ω/ζ 档导出，Rapier 同构）。
pub(crate) struct SolverParams {
    pub(crate) slop: f32,
    pub(crate) inv_dt: f32,
    /// erp 偏置速率（1/s）：动态对 / 静态侧（更硬，防挤压穿过）。
    pub(crate) erp_inv_dt_dyn: f32,
    pub(crate) erp_inv_dt_static: f32,
    /// CFM 正则化因子：动态对 / 静态侧。
    pub(crate) cfm_dyn: f32,
    pub(crate) cfm_static: f32,
    pub(crate) max_corr: f32,
}

impl SolverParams {
    /// ω = 2πf；erp_inv_dt = ω/(dt·ω + 2ζ)；erp = dt·erp_inv_dt；
    /// cfm_coeff = inv²/((1+inv)·4ζ²)（inv = 1/erp − 1）；cfm = 1/(1+cfm_coeff)。
    /// 公式逐行对齐 Rapier `SpringCoefficients::{erp_inv_dt, cfm_coeff, cfm_factor}`。
    pub(crate) fn from_config(cfg: &PhysConfig, dt: f32) -> Self {
        fn erp_cfm(freq_hz: f32, zeta: f32, dt: f32) -> (f32, f32) {
            let omega = core::f32::consts::TAU * freq_hz;
            let two = 2.0 * zeta;
            let erp_inv_dt = omega / (dt * omega + two);
            let erp = dt * erp_inv_dt;
            let (erp_inv_dt, cfm) = if erp != 0.0 {
                let inv = 1.0 / erp - 1.0;
                let cfm_coeff = inv * inv / ((1.0 + inv) * 4.0 * zeta * zeta);
                (erp_inv_dt, 1.0 / (1.0 + cfm_coeff))
            } else {
                (erp_inv_dt, 1.0)
            };
            (erp_inv_dt, cfm)
        }
        let (erp_inv_dt_dyn, cfm_dyn) = erp_cfm(cfg.contact_freq_hz, cfg.contact_damping_ratio, dt);
        let (erp_inv_dt_static, cfm_static) =
            erp_cfm(cfg.static_contact_freq_hz, cfg.contact_damping_ratio, dt);
        Self {
            slop: cfg.linear_slop,
            inv_dt: 1.0 / dt,
            erp_inv_dt_dyn,
            erp_inv_dt_static,
            cfm_dyn,
            cfm_static,
            max_corr: cfg.max_corrective_velocity,
        }
    }
}

/// 单流形约束。
pub(crate) struct ContactConstraint {
    pub(crate) a: u32,
    pub(crate) b: u32,
    pub(crate) normal: Vec3,
    /// warm 槽号（`u32::MAX` = 本 tick 新接触；回写按此**原位写**，零哈希）。
    pub(crate) warm_slot: u32,
    /// warm 键的**特征空间**（`ContactPoints::space()`；非复合体恒 0）——回写键 = `(a, b, space)`。
    pub(crate) warm_space: u32,
    /// 点数据**定长内联**（流形点 ≤4，窄相已截断）：`Vec<PointConstraint>` 时
    /// 每流形一次 `with_capacity(4)` 堆分配 + 释放——金字塔实测每帧 1140 次
    /// （570 流形 × 2 子步），是构建相位的固定开销之一。内联后零堆。
    /// 与 `WarmManifold.points` 同一课（那次是 22 万次小 Vec ≈11.4 ms）。
    pub(crate) points: [PointConstraint; 4],
    /// 本流形实际点数（≤4）。
    pub(crate) npts: u8,
}

/// **岛级并行的内部拆分**（诊断；T4「碎片雨扩展比」为何饱和就看这几个数）。
///
/// 求解相位 = 建岛/分组 gather → `thread::scope`（**唯一并行段**）→ 散射回写 → warm 回写。
/// `gather_us + scatter_us` 是**串行**的，它们占求解相位的比例就是扩展比的**上限**
/// （Amdahl：并行段再完美也拉不动这两块）。
/// `group_us` 是**每组**的解算墙钟：组间最大值决定 scope 的墙钟 ⇒ 它的离散度就是**负载不均**。
/// `group_manifs` 是每组的流形数（工作量代理；与 `group_us` 一起看才能判"均不均"）。
#[derive(Clone, Default, Debug)]
pub struct IslandDiag {
    /// 本次参与的岛数与流形数、并行组数（`g_count == 1` ⇒ 走单组串行）。
    pub islands: u32,
    pub manifolds: u32,
    pub g_count: u32,
    /// ⚠️ **实测是「建岛」部分本身、不含 `fill_us`**（`gather_us == island_build_us`，两者同值）：
    /// 值 = `d_island_all − d_fill`（见 `solve_phase`）。**原先的注释写作
    /// "建岛 + gather = island_build_us + fill_us" 与实现不符**，2026-09-22 核正
    /// （实测同帧：`gather_us 0.05` vs `fill_us 3.47` ⇒ 差一个数量级，不可能相加关系）。
    pub gather_us: u64,
    /// **建岛**部分（并查集分岛 + 岛桶 + 流形归岛）——实测可忽略（36000 体约 13 µs；
    /// 10 万体单体岛场景 0.05 ms/子步）。
    pub island_build_us: u64,
    /// 其**按组 gather** 部分（每体 4 次 push + 世界逆惯量矩阵）——实测占求解相位约 24%。
    /// ⚠️ **不要试图并行化它**：与解算同属访存带宽受限，并发反而更慢（见 `EXPERIMENTS` C8）。
    pub fill_us: u64,
    /// `thread::scope` 墙钟 = 最慢组 + spawn/join（并行段）。
    pub scope_us: u64,
    /// 散射回写 + warm 回写/剪枝（串行）。
    pub scatter_us: u64,
    /// 每组的解算墙钟（µs）与流形数（工作量代理）。
    pub group_us: Vec<u64>,
    pub group_manifs: Vec<u32>,
    /// **本次解算**（= 每子步）的三段 CPU 时间合计（**各组之和**）：
    /// 约束构建 / 热启动预施加 / 迭代扫掠。与 `last_detail_us`（**进程内累计**，
    /// 供 `arena_bench` 除以步数取均值用）**不是一回事**——这三个字段与
    /// `manifolds`/`points` **同源同帧**，只有它们能做"每流形 / 每点成本"的归一化比较。
    /// ⚠️ 并发时它们是"各组墙钟之和"（受带宽竞争放大）⇒ **判占比**可靠；
    /// 判绝对量要拿串行跑的同一字段比。
    pub build_us: u64,
    pub warm_us: u64,
    pub iter_us: u64,
    /// 本次解算的接触点总数（与 `manifolds` 同帧；归一化用）。
    pub points: u32,
    /// warm 槽表当前条目数（工作集读数用：× `size_of::<WarmManifold>()` 即该表占用）。
    pub warm_count: u32,
    /// 单个 warm 槽的字节数（`size_of::<WarmManifold>()`；工作集读数用，避免外部估算偏差）。
    pub warm_bytes_per_slot: u32,
}

/// 顺序冲量求解器。
#[derive(Default)]
pub struct ImpulseSolver {
    /// **warm 槽位表（稠密）**：(键, 数据) 连续存储；回写按槽号原位写、
    /// 剪枝对稠密槽单遍扫——`HashMap` 桶遍历曾是 33.5ms/tick 的大头（DESIGN §11）。
    pub(crate) warm_slots: Vec<(WarmKey, WarmManifold)>,
    /// 空闲槽号（LIFO 复用，避免周期压实）。
    pub(crate) warm_free: Vec<u32>,
    /// 键 → 槽号（查找用；只在查找/分配时触达）。
    pub(crate) warm_index: HashMap<WarmKey, u32>,
    /// 求解印章（自增）。
    pub(crate) warm_stamp: u32,
    /// **世界逆惯量矩阵缓存（按组紧凑）**：`M = R·diag(inv_local)·Rᵀ`，一帧内姿态
    /// 不变 ⇒ 每帧 gather 一次、求解环内复用（每点每轮由两次矩阵乘降为**一次**
    /// `M·(r×imp)`）。**索引 = 组内局部索引**（`local_of[i]`）：原先是全局数组
    /// （36B × 全体数，8B 档 7.2 MB）且求解环里按随机全局索引访问 ⇒ 缓存不友好；
    /// 按组紧凑后工作集 = 组内体数（金字塔档 ≈26 体，落进 L1）。
    /// 纯搬运：同值、同运算序 ⇒ **逐位不变**（2026-09-18 A3）。
    pub(crate) group_iw: Vec<Vec<Mat3>>,
    /// **逆质量缓存（按组紧凑）**：`group_apply` 每次调用读一次（热路径），
    /// 同样按组内局部索引。纯搬运 ⇒ 逐位不变。
    pub(crate) group_im: Vec<Vec<f32>>,
    /// 上一帧岛（诊断/调试用）。
    pub island_count: usize,
    /// warm starting 点匹配距离（= 4×skin，构造时可调）。
    pub match_dist: f32,
    /// 诊断：本帧计时器被清零的体数。
    pub sleep_resets: u32,
    /// **唤醒接触数门**的逐体计数（`PhysConfig::wake_gate_k`）：**每次 `solve_phase`
    /// 调用开头清零**（寿命 = 语义：同一次调用内累计，见该字段文档）；`k == 0` 时不参与。
    pub(crate) wake_streak: Vec<u32>,
    /// 并行分组复用缓冲（§6）：组 → 约束构建 / warm 更新 / 速度 scratch。
    pub(crate) build_bufs: Vec<Vec<ContactConstraint>>,
    pub(crate) warm_outs: Vec<Vec<WarmOutEntry>>,
    pub(crate) group_lv: Vec<Vec<Vec3>>,
    pub(crate) group_av: Vec<Vec<Vec3>>,
    /// 体 → 组内局部索引（u32::MAX = 静态/不在清醒岛）。
    pub(crate) local_of: Vec<u32>,
    /// 诊断：上一帧内部阶段耗时（µs）=(岛构建, 约束构建+迭代, 休眠/其他, 保留)。
    pub last_phase_us: (u64, u64, u64, u64),
    /// 诊断：上一帧解算细分（µs）=(建岛, **约束构建**, **热启动预施加**, **迭代扫掠**)。
    /// 用途：求解相位线性拟合出"每扫掠成本"与"每帧固定成本"两半之后，定位固定
    /// 成本落在哪（金字塔实测固定 ≈762 µs vs 每扫掠 ≈124 µs，固定是大头）。
    pub last_detail_us: [u64; 4],
    /// 诊断计数器（零开销）：上一次解算的 (接触点总数, 法向冲量≈0 的点数)。
    /// 用途：回答"**流形选点是否选多了**"——解算后不承载任何法向冲量的点，
    /// 每扫掠仍要付一次迭代。本段五次微优化证伪（全是"减指令数"）之后，
    /// 这是唯一还开着的**减工作量**入口。
    pub last_points: (u64, u64),
    /// 诊断：岛级并行的**内部拆分**（T4 扩展比的上限就写在串行占比里）。
    /// 每次 `solve_phase` 覆盖；内部 `Vec` 跨帧复用容量。
    pub island_diag: IslandDiag,
    /// 并查集缓冲（跨帧复用）。
    pub(crate) parent: Vec<u32>,
    /// 岛池（跨帧复用：Vec 容量保留，全清醒场景不逐帧分配）。
    pub(crate) island_pool: Vec<Island>,
}

pub(crate) struct Island {
    pub(crate) bodies: Vec<u32>,
    /// 流形索引（岛内按全局流形序，§4.14）。
    pub(crate) manifs: Vec<usize>,
}
