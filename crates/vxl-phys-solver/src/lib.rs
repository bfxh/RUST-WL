//! # vxl-phys-solver
//!
//! 刚体约束求解（§2.5/§2.6）：
//! - M0：顺序冲量（warm starting + Baumgarte 位置修正 + 摩擦锥）——即 M0 里程碑
//!   明确要求的「顺序冲量」；TGS-Soft（time-of-impact 分级 + 软约束）在 M1 升级，
//!   本文件求解循环骨架与岛/休眠/缓存层保持不变。
//! - 岛：并查集分岛；岛内约束按 (a, b, 点序) 固定排序（§4.14 确定性模式）。
//! - 休眠（§4.11）：线性 0.04 / 角速 0.05 rad/s、计时 0.5 s，岛级判定。
//! - CCD（§4.12/§2.7）：`ccd` 模块——选择性判据 + 保守推进扫描（M1）。
//! - 并行（§6）：清醒岛分组并行（组内串行、组间体集合不相交），
//!   gather→solve→scatter 走组内 scratch 速度缓冲，与串行 bit 级一致（§5）。

#![forbid(unsafe_code)]

pub mod ccd;
pub mod joints;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, Mat3, PhysConfig};
use vxl_phys_narrow::Manifold;

/// 单接触点的已求解冲量缓存（warm starting + 接触回收）。
#[derive(Clone, Copy, Debug, Default)]
struct WarmPoint {
    pn: f32,
    pt1: f32,
    pt2: f32,
    /// 接触特征 ID（窄相命名，跨帧稳定；0 = 无特征）。
    feature: u32,
    /// 局部锚点（双方体局部坐标；M1 接触回收核心——锚点固定在材料点上，
    /// 帧间只推进距离；Rapier `contact_recycling` 同族）。
    la: Vec3,
    lb: Vec3,
    /// **烘焙时**的接触深度（正 = 穿透）。本帧有效深度 = 此值 + 锚点
    /// 当前累计分离量沿法向的投影（存更新值会复合累计 → 二次增长，实测
    /// 推出 0.44 m/s 虚假分离速度；必须存烘焙值）。
    depth0: f32,
}

impl WarmPoint {
    /// 零点哨兵（定长数组初始化用）。
    const EMPTY: WarmPoint = WarmPoint {
        pn: 0.0,
        pt1: 0.0,
        pt2: 0.0,
        feature: 0,
        la: Vec3::ZERO,
        lb: Vec3::ZERO,
        depth0: 0.0,
    };
}

/// 接触回收半径（m；超过则锚点重烘焙）。**0.01**（收窄自 Rapier 的 0.05）：
/// 累计漂移更小 ⇒ 回拉修正更小、各场景代价大降（门槛场景末态 38→7 体醒、
/// p50 9.1→0.95 ms；塔 600 tick 0.28、2000 tick 全程平静；col45 末态
/// 2.06→1.52 cm）——M1 第十一段决定性验证。
const RECYCLE_DIST: f32 = 0.01;

/// 切向锚点漂移回拉的强度标度（1 = Rapier 原式；0 = 关闭回拉只保留回收）。
/// 切向锚点漂移回拉的强度标度（1 = Rapier 原式；0 = 关闭回拉只保留回收）。
/// **长时程决定性数据（M1 第十一段）**：关闭时塔蠕动在 t≈1400 持续升级为
/// 弹射（t=2000 |v| 97、盒飞出世界）；满强度使蠕动衰减（0.34→0.26）。
/// 各场景代价（门槛 38 体醒/8.4ms、5 层堆 0.09 微抖）见消融记录；
/// 消融中满强度还会在 5 层堆留下 ~0.09 m/s 永久微抖（不入睡）。
const DRIFT_BIAS_SCALE: f32 = 1.0;

/// 切向漂移回拉的死区（m）：|漂移| ≤ 死区时不回拉——只纠正真实材料滑移，
/// 不响应微倾/裁剪抖动引起的毫米级锚点错位。**默认 0（关闭）**：消融实测
/// 1mm 死区利于 5 层堆（0.09→0.05）但让塔蠕动从「衰减」退回「微增」
/// （0.34→0.26 变 0.22→0.29）——M1 验收主体是塔，取无死区。
const DRIFT_DEADZONE: f32 = 0.0;

/// **切向回拉的粘着判定阈值**：`|pt| ≥ 此值 × μ·pn` ⇒ 该接触视为**滑动**、不做切向回拉。
///
/// 依据（`OPEN-PROBLEMS.md` P1）：滑动接触的锚点分离**就是**真实材料滑移，回拉等于
/// **抹掉真实滑动**、且因力臂而 ∝ μ 地注入转矩——那是角向残差的主项（实测塔角向超阈
/// 202→93、总数 406→269）；粘着接触的分离才是数值错位，才是回拉的正当对象。
/// 判定用**上一子步的暖启动冲量**（零额外状态、不需材料查表）。
/// 实测该阈值 0.99 在各轴上最好（0.90 全面变差：角 122→137、总数 247→315）。
const DRIFT_STICK_RATIO: f32 = 0.99;

/// **warm 匹配分支计数**（诊断；atomic 宽松序，只计数、不改行为 ⇒ 哈希不变）。
///
/// 用途：判定「本仓流形是否**特征稳定**」——这是"检测每步一次 / 便宜子步"能否成立的
/// 前提。§9（P1：检测上提，塔崩 |v| 44.4）与 §10（复用约束 + prepare，全变体崩）把失败
/// 归因于"本引擎流形是**裁剪产物** ⇒ 材料点配对跨帧漂移"；但窄相的 `feature` 设计本
/// 就是**几何特征哈希**（`feat_intersect`：入射棱对 × 参考侧平面，"同三元组重现同 ID"），
/// 即**该归因缺实测支撑**。本计数给出依据：
/// - **精确命中率高** ⇒ 配对稳定 ⇒ 归因错误，§9/§10 的失败须另找机理
///   （便宜子步这条最值钱的结构杠杆应重新评估）；
/// - **命中率低** ⇒ 归因成立（且原因多半是参考面翻转 / 裁剪输出churn）。
static WARM_EXACT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WARM_FALLBACK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WARM_MISS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// **回退命中的成因分解**（诊断；只计数）：特征是"ID 变了但材料点还在附近"，
/// 拆开看变了哪一种 ⇒ 判定修复入口（见 vxl_phys_narrow::feature_kind 注）。
static WB_SIDE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WB_CLIP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WB_HASH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WB_SAME: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// **参考面变化的代理**（诊断；只计数）：匹配上的点里，"接触法向变了"（`dot < 0.99999`）
/// 与"法向几乎不变"各占多少。
///
/// 用途（A6 参考面稳定性）：`feature` 的哈希部分 = `入射棱对 × 参考侧平面 k`
/// （`k = ref_base + k'`）。**换参考面**（`ref_base` 变）通常伴随法向改变；而在**同一面内**
/// 换裁剪侧平面（`k'` 变）则法向不变、只有哈希变 ⇒ 用"法向是否变"即可把 K.1 的
/// "哈希变 55.9%" 拆成「换面」与「同面换裁剪面」两块。
static WN_FLIP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static WN_SAME: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 读取并清零参考面代理计数：`(法向变了, 法向几乎不变)`。
pub fn warm_normal_flip_take() -> (u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (WN_FLIP.swap(0, Relaxed), WN_SAME.swap(0, Relaxed))
}

/// 读取并清零回退成因分解：`(侧别翻转, 裁剪路变化, 哈希变化, 特征相同)`。
pub fn warm_fallback_kind_take() -> (u64, u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        WB_SIDE.swap(0, Relaxed),
        WB_CLIP.swap(0, Relaxed),
        WB_HASH.swap(0, Relaxed),
        WB_SAME.swap(0, Relaxed),
    )
}

/// 读取并清零 warm 匹配计数：`(精确特征命中, 近邻回退命中, 未匹配)`。
pub fn warm_match_stats_take() -> (u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        WARM_EXACT.swap(0, Relaxed),
        WARM_FALLBACK.swap(0, Relaxed),
        WARM_MISS.swap(0, Relaxed),
    )
}

/// **解算记账**（诊断；只计数）：点总数 / 其中分离点（`depth < 0`，吃 spec 额度）数 /
/// `spec` 之和 ×1000 / `bias` 之和 ×1000。
///
/// 用途（`EXPERIMENTS.md` 末节 N/P 的复用余价）：判定"复用是否让去穿透/闭合额度被
/// **重复发放**"——复用下几何冻结，若同一份 `sep` 被逐子步当额度发出去，就是系统性的
/// 能量注入；"每子步 spec 之和"与"每子步 bias 之和"在复用开关下的对比即可判。
static DA_PTS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static DA_SEP: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static DA_SPEC: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static DA_BIAS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 读取并清零解算记账：`(点总数, 分离点数, spec 和 ×1000, bias 和 ×1000)`。
pub fn solve_accounting_take() -> (u64, u64, u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        DA_PTS.swap(0, Relaxed),
        DA_SEP.swap(0, Relaxed),
        DA_SPEC.swap(0, Relaxed),
        DA_BIAS.swap(0, Relaxed),
    )
}

/// 回退匹配的**接触状态门**（米）：只有本帧裁剪深度 > 此阈值的回退匹配才允许/// 暖启动——**分离/预期接触（depth ≤ 0）拒配**（按新接触处理，暖冲量清零、
/// 锚点重烘焙）。依据：错配锚点的暖冲量会过驱动「间歇角点接触」（125 体族的
/// 触发画像 = 留缝角点间歇接触，见 OPEN-PROBLEMS P1）；而塔的承重腿是持续
/// 正深度接触、不受影响。**非距离判别式**（距离类门已实测分不开有害/必需匹配）。
const FB_DEPTH_MIN: f32 = 0.0;

/// 回退匹配的**深度跳变门**（米）：`|本帧裁剪深度 − 锚点烘焙深度|` 超限 ⇒ 拒配。
/// 这是 OPEN-PROBLEMS P1 候选①的正确形式——**注意别写成「有效深度 vs 裁剪深度」**：
/// 锚点随体刚性移动时 `depth0 + sep·n ≡ cp.depth` 恒成立（实测差 ≤1e-6），
/// 那样写等于不设门（本轮踩过）。直接量 `cp.depth − depth0` 有真信号：插桩实测
/// 「刚触地型」匹配 = 锚点在分离态烘焙（d0 = −3.7mm）后被拿到触地帧用
/// （d = +0.2mm）⇒ 跳变 3.9mm、且携带加载相冲量。**阈值扫描（125 体复现 dE 峰值）**：
/// 1mm → 63.8、0.5mm → 27.8、**0.25mm → 18.2（取此档，验收线 ≤20）**、0.1mm → 17.7
/// （更紧开始逼近「全拒」；全拒 = 16.2 但塔崩）。物理含义：dd ≈ v·dt ⇒ 只对
/// 「深度变化快于 ~1.5 cm/s」的接触拒配，与睡眠阈（4 cm/s）同量级。
const FB_DEPTH_JUMP: f32 = 0.00025;

#[derive(Clone, Copy, Debug)]
struct WarmManifold {
    normal: Vec3,
    /// 点数据**定长内联**（流形点 ≤4，窄相已截断）——此前是 `Vec<WarmPoint>`，
    /// 每 tick 22 万次小 Vec 的分配 + 释放实测 ≈11.4ms（泄漏探针）；内联后零堆。
    points: [WarmPoint; 4],
    n: u8,
    /// 求解印章（本 solve 调用是否刷新过；剪枝用，见 `warm 槽位表` 注）。
    seen: u32,
}

impl WarmManifold {
    const EMPTY: WarmManifold = WarmManifold {
        normal: Vec3::ZERO,
        points: [WarmPoint::EMPTY; 4],
        n: 0,
        seen: 0,
    };

    #[inline]
    fn pts(&self) -> &[WarmPoint] {
        &self.points[..self.n as usize]
    }
}

/// **warm 缓存键**：`(体 a, 体 b, 特征空间)`。
///
/// 特征空间来自 `ContactPoints::space()`（**显式通道**，窄相复合体展开时按子形状序号填；
/// 非复合体恒 0）。**为什么必须有它**：同一体对会同时存在多条流形（复合体每个子形状一条），
/// 共用 `(a, b)` 键会相互覆盖 ⇒ 该对的暖缓存**整条失效**（实测 `warm_counter_probe`：
/// 两球复合体计数全 0，单子形状复合体 32）。非复合体场景空间恒为 0 ⇒ 键退化为 `(a, b, 0)`。
///
/// ⚠️ **不要**改用 `feature` 的高位当空间：窄相特征号高位被"侧别/裁剪路/哈希"编码占用，
/// 哈希逐帧漂移 ⇒ 普通场景暖启动会随机失效（实测 `default_tier_stability` 的 top_y 漂 3 mm，
/// 见 `TECH-SURVEY.md` A9 ④）。
type WarmKey = (u32, u32, u32);

/// 死槽键（槽位表空洞哨兵）。
const DEAD_KEY: WarmKey = (u32::MAX, u32::MAX, u32::MAX);

/// warm 回写条目：(槽号（`u32::MAX` = 新键）, 键, 数据)。
type WarmOutEntry = (u32, WarmKey, WarmManifold);

use vxl_phys_core::Vec3;

/// 单接触点约束（预计算质量项与软接触目标/正则化）。
#[derive(Clone, Copy, Debug, Default)]
struct PointConstraint {
    ra: Vec3,
    rb: Vec3,
    t1: Vec3,
    t2: Vec3,
    nmass: f32,
    tmass1: f32,
    tmass2: f32,
    /// 切向 2×2 矩阵交叉项 k12（联立解用；见 `contact_mass_cross`）。
    tcross: f32,
    /// 切向锚点漂移回拉目标（Rapier 切向 rhs 同义：`v_t → (pa−pb)·t·inv_dt`，
    /// 由摩擦在锥内执行；回收锚点使其为真实材料点错位，非裁剪几何噪声）。
    trhs1: f32,
    trhs2: f32,
    /// 局部锚点与烘焙深度（warm 回写 / 下一帧距离推进）；`depth0` =
    /// 烘焙时深度（回写时必须存它，见 `WarmPoint::depth0` 注）。
    la: Vec3,
    lb: Vec3,
    depth0: f32,
    /// M1 软接触（TGS-Soft 语义，与 Rapier 0.35 同构）：法向目标分离速度
    /// rhs = 去穿透速率（erp·depth，钳 ±max_corrective_velocity）− speculative
    /// 项（浅缝允许一个 tick 内闭合）+ 弹性目标（e·vn，e>0 时）。
    /// 旧「硬约束 + 分裂冲量独立通道」方案已被本形态取代（金样定标见
    /// docs/M1-PLAN.md 第五段：软接触是深堆稳定的结构性关键）。
    rhs: f32,
    /// 正则化因子（CFM）：穿透接触 cfm=1（硬投影，保证支撑刚性）；
    /// speculative（depth<0）接触 cfm<1（等效柔度 ω/ζ，Rapier 默认
    /// 30 Hz 动/60 Hz 静态、ζ=10）——限制迭代增益、深堆不依赖跨层链收敛。
    cfm: f32,
    friction: f32,
    pn: f32,
    pt1: f32,
    pt2: f32,
    /// 接触特征 ID（窄相来；warm 缓存回写用）。
    feature: u32,
    warm: Option<WarmPoint>,
}

/// 软接触求解参数（每步预计算；由 `PhysConfig` 的 ω/ζ 档导出，Rapier 同构）。
struct SolverParams {
    slop: f32,
    inv_dt: f32,
    /// erp 偏置速率（1/s）：动态对 / 静态侧（更硬，防挤压穿过）。
    erp_inv_dt_dyn: f32,
    erp_inv_dt_static: f32,
    /// CFM 正则化因子：动态对 / 静态侧。
    cfm_dyn: f32,
    cfm_static: f32,
    max_corr: f32,
}

impl SolverParams {
    /// ω = 2πf；erp_inv_dt = ω/(dt·ω + 2ζ)；erp = dt·erp_inv_dt；
    /// cfm_coeff = inv²/((1+inv)·4ζ²)（inv = 1/erp − 1）；cfm = 1/(1+cfm_coeff)。
    /// 公式逐行对齐 Rapier `SpringCoefficients::{erp_inv_dt, cfm_coeff, cfm_factor}`。
    fn from_config(cfg: &PhysConfig, dt: f32) -> Self {
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
struct ContactConstraint {
    a: u32,
    b: u32,
    normal: Vec3,
    /// warm 槽号（`u32::MAX` = 本 tick 新接触；回写按此**原位写**，零哈希）。
    warm_slot: u32,
    /// warm 键的**特征空间**（`ContactPoints::space()`；非复合体恒 0）——回写键 = `(a, b, space)`。
    warm_space: u32,
    /// 点数据**定长内联**（流形点 ≤4，窄相已截断）：`Vec<PointConstraint>` 时
    /// 每流形一次 `with_capacity(4)` 堆分配 + 释放——金字塔实测每帧 1140 次
    /// （570 流形 × 2 子步），是构建相位的固定开销之一。内联后零堆。
    /// 与 `WarmManifold.points` 同一课（那次是 22 万次小 Vec ≈11.4 ms）。
    points: [PointConstraint; 4],
    /// 本流形实际点数（≤4）。
    npts: u8,
}

/// 顺序冲量求解器。
#[derive(Default)]
pub struct ImpulseSolver {
    /// **warm 槽位表（稠密）**：(键, 数据) 连续存储；回写按槽号原位写、
    /// 剪枝对稠密槽单遍扫——`HashMap` 桶遍历曾是 33.5ms/tick 的大头（DESIGN §11）。
    warm_slots: Vec<(WarmKey, WarmManifold)>,
    /// 空闲槽号（LIFO 复用，避免周期压实）。
    warm_free: Vec<u32>,
    /// 键 → 槽号（查找用；只在查找/分配时触达）。
    warm_index: HashMap<WarmKey, u32>,
    /// 求解印章（自增）。
    warm_stamp: u32,
    /// **世界逆惯量矩阵缓存（按组紧凑）**：`M = R·diag(inv_local)·Rᵀ`，一帧内姿态
    /// 不变 ⇒ 每帧 gather 一次、求解环内复用（每点每轮由两次矩阵乘降为**一次**
    /// `M·(r×imp)`）。**索引 = 组内局部索引**（`local_of[i]`）：原先是全局数组
    /// （36B × 全体数，8B 档 7.2 MB）且求解环里按随机全局索引访问 ⇒ 缓存不友好；
    /// 按组紧凑后工作集 = 组内体数（金字塔档 ≈26 体，落进 L1）。
    /// 纯搬运：同值、同运算序 ⇒ **逐位不变**（2026-09-18 A3）。
    group_iw: Vec<Vec<Mat3>>,
    /// **逆质量缓存（按组紧凑）**：`group_apply` 每次调用读一次（热路径），
    /// 同样按组内局部索引。纯搬运 ⇒ 逐位不变。
    group_im: Vec<Vec<f32>>,
    /// 上一帧岛（诊断/调试用）。
    pub island_count: usize,
    /// warm starting 点匹配距离（= 4×skin，构造时可调）。
    pub match_dist: f32,
    /// 诊断：本帧计时器被清零的体数。
    pub sleep_resets: u32,
    /// 并行分组复用缓冲（§6）：组 → 约束构建 / warm 更新 / 速度 scratch。
    build_bufs: Vec<Vec<ContactConstraint>>,
    warm_outs: Vec<Vec<WarmOutEntry>>,
    group_lv: Vec<Vec<Vec3>>,
    group_av: Vec<Vec<Vec3>>,
    /// 体 → 组内局部索引（u32::MAX = 静态/不在清醒岛）。
    local_of: Vec<u32>,
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
    /// 并查集缓冲（跨帧复用）。
    parent: Vec<u32>,
    /// 岛池（跨帧复用：Vec 容量保留，全清醒场景不逐帧分配）。
    island_pool: Vec<Island>,
}

struct Island {
    bodies: Vec<u32>,
    /// 流形索引（岛内按全局流形序，§4.14）。
    manifs: Vec<usize>,
}

#[allow(clippy::too_many_arguments)] // 接触质量项的规范参数集（M1 TGS-Soft 重构时收敛为结构体）
fn contact_mass(
    im_a: f32,
    im_b: f32,
    ra: Vec3,
    rb: Vec3,
    dir: Vec3,
    bodies: &BodySet,
    a: usize,
    b: usize,
) -> f32 {
    let mut k = im_a + im_b;
    if im_a > 0.0 {
        let rn = ra.cross(dir);
        let w = bodies.apply_world_inv_inertia(a, rn);
        k += w.cross(ra).dot(dir);
    }
    if im_b > 0.0 {
        let rn = rb.cross(dir);
        let w = bodies.apply_world_inv_inertia(b, rn);
        k += w.cross(rb).dot(dir);
    }
    if k > 1e-12 {
        1.0 / k
    } else {
        0.0
    }
}

/// 切向 2×2 有效质量矩阵的**交叉项** k12（Rapier `ContactConstraintTangentPart.r[2]`
/// 同源）：k12 = J1ᵀ·M⁻¹·J2 = (im_a+im_b)·(d1·d2) + Σ (r×d1)ᵀ·I⁻¹·(r×d2)。
/// 各向异性 K 下逐轴（对角）解会留正交残差并旋转能量——摩擦模式在大堆里
/// 被泵成弹射（金样源码注释原文）；必须联立解。
#[allow(clippy::too_many_arguments)] // 与 contact_mass 同参数集（热路径内联）
fn contact_mass_cross(
    im_a: f32,
    im_b: f32,
    ra: Vec3,
    rb: Vec3,
    d1: Vec3,
    d2: Vec3,
    bodies: &BodySet,
    a: usize,
    b: usize,
) -> f32 {
    let mut k = (im_a + im_b) * d1.dot(d2);
    if im_a > 0.0 {
        let w = bodies.apply_world_inv_inertia(a, ra.cross(d2));
        k += ra.cross(d1).dot(w);
    }
    if im_b > 0.0 {
        let w = bodies.apply_world_inv_inertia(b, rb.cross(d2));
        k += rb.cross(d1).dot(w);
    }
    k
}

fn tangents(n: Vec3) -> (Vec3, Vec3) {
    let refr = if n.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let t1 = n.cross(refr).normalize();
    let t2 = n.cross(t1);
    (t1, t2)
}

/// 并查集 find（路径减半）。固定规则：小索引为根（确定性）。
fn find_small_root(parent: &mut [u32], x: u32) -> u32 {
    let mut x = x;
    while parent[x as usize] != x {
        let g = parent[x as usize];
        parent[x as usize] = parent[g as usize];
        x = parent[x as usize];
    }
    x
}

fn union_small_root(parent: &mut [u32], x: u32, y: u32) {
    let rx = find_small_root(parent, x);
    let ry = find_small_root(parent, y);
    if rx != ry && rx < ry {
        parent[ry as usize] = rx;
    } else if rx != ry {
        parent[rx as usize] = ry;
    }
}

/// 组内速度读取：局部索引 u32::MAX = 静态侧。静止体速度恒为精确 0
/// （inv_mass=0 的旧路径空写不改变 0 值），读 Vec3::ZERO 与其 bit 级等价。
#[inline]
fn group_vel(lv: &[Vec3], av: &[Vec3], local_of: &[u32], i: usize, r: Vec3) -> Vec3 {
    let k = local_of[i];
    if k == u32::MAX {
        Vec3::ZERO
    } else {
        let q = k as usize;
        lv[q] + av[q].cross(r)
    }
}

/// 组内冲量施加：动态侧写 scratch；静态/越组侧跳过（旧路径为 inv_mass=0
/// 的空写，数值结果相同）。
#[inline]
#[allow(clippy::too_many_arguments)] // 组内热路径内联目标：避免引入打包结构体的额外构造成本
fn group_apply(
    lv: &mut [Vec3],
    av: &mut [Vec3],
    local_of: &[u32],
    iw: &[Mat3],
    im: &[f32],
    i: usize,
    ra: Vec3,
    imp: Vec3,
    minus: bool,
) {
    let k = local_of[i];
    if k == u32::MAX {
        return;
    }
    let q = k as usize;
    let m = im[q];
    // 世界逆惯量矩阵每帧预积（`R·diag(inv_local)·Rᵀ`）⇒ 这里只剩**一次**矩阵乘；
    // 旧路径每点每轮要 `Rᵀ·x → diag → R·(...)` 两次矩阵乘。数学等价、舍入不同
    // （启用即换代；见 EXPERIMENTS 2026-09-15「每点求解成本」）。
    // A3：`iw`/`im` 都按**组内局部索引**读（原为全局散点：36B×全体数 + 4B×全体数）。
    let dw = iw[q].mul_vec3(ra.cross(imp));
    if minus {
        lv[q] -= imp * m;
        av[q] -= dw;
    } else {
        lv[q] += imp * m;
        av[q] += dw;
    }
}

impl ImpulseSolver {
    pub fn new(skin: f32) -> Self {
        Self {
            warm_slots: Vec::new(),
            warm_free: Vec::new(),
            warm_index: HashMap::new(),
            warm_stamp: 0,
            island_count: 0,
            match_dist: (skin * 4.0).max(0.02),
            sleep_resets: 0,
            build_bufs: Vec::new(),
            warm_outs: Vec::new(),
            group_lv: Vec::new(),
            group_av: Vec::new(),
            group_iw: Vec::new(),
            group_im: Vec::new(),
            local_of: Vec::new(),
            last_phase_us: (0, 0, 0, 0),
            last_detail_us: [0; 4],
            last_points: (0, 0),
            parent: Vec::new(),
            island_pool: Vec::new(),
        }
    }

    /// 求解 + 岛级休眠。调用方顺序：积分速度 → 检测 → 本函数 → 积分位置。
    /// 唤醒语义：岛整体睡/醒（Box2D 同族）——岛内任一成员被外部唤醒（用户冲量、
    /// 新接触带入的动体）即全岛唤醒；杜绝"醒体反复唤醒睡体"。
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
    ) {
        self.solve_phase(bodies, manifolds, config, dt, jobs, false);
    }

    /// **无偏置趟**（Rapier TGS-Soft 的末趟，`rhs_wo_bias` 同义）：位置积分之后
    /// 对同一批流形重解，去掉**去穿透偏置**（`max_corr = 0`），把"修正速度"从
    /// **最终速度**里移除——位置已由带偏置趟的积分推进，去穿透不受影响；最终速度
    /// 只留"不接近"（speculative）语义。切向漂移回拉**保留**（Rapier 的切向
    /// `rhs_wo_bias` = 材料点漂移率；实测归零会让 125 体留缝角点场景退化）。
    /// `config.stabilization_iterations == 0` 时本函数不应被调用。
    pub fn solve_unbiased(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
    ) {
        self.solve_phase(bodies, manifolds, config, dt, jobs, true);
    }

    fn solve_phase(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
        cleanup: bool,
    ) {
        let e_threshold = config.restitution_threshold;
        let iters = if cleanup {
            config.stabilization_iterations.max(1)
        } else {
            config.velocity_iterations.max(1)
        };
        let shock = if cleanup { 0 } else { config.shock_iterations };
        let normal_inner = if cleanup {
            1
        } else {
            config.normal_inner.max(1)
        };
        let mut sp = SolverParams::from_config(config, dt);
        if cleanup {
            // 无偏置趟：法向只留 speculative（去穿透偏置 0）——对齐 Rapier
            // `rhs_wo_bias`。**切向漂移回拉保留**：Rapier 的切向 `rhs_wo_bias` =
            // `solver_contact.tangent_velocity`（材料点漂移率），即在无偏置趟里
            // 仍然生效；本仓实测把它一并归零会让 125 体留缝角点场景退化
            // （|ω| 0.13→0.45、末态 KE 1.6→19.0）——它正是那个场景的承重件。
            sp.max_corr = 0.0;
        }
        let threads = jobs.threads().max(1);
        let match_dist = self.match_dist;
        // 计时走跨目标探针（wasm32 无时钟；原生不变）。
        let t_island = vxl_phys_core::probe::start();

        // 1) 并查集分岛（直接对流形；双静态对不入岛）。固定规则：小索引为根（确定性）。
        //    缓冲跨帧复用（self.parent），避免每帧 20 万级 alloc/fill。
        let n = bodies.len();
        let mut parent = std::mem::take(&mut self.parent);
        parent.clear();
        parent.extend(0..n as u32);
        for m in manifolds {
            let (a, b) = (m.a as usize, m.b as usize);
            if bodies.is_dynamic(a) && bodies.is_dynamic(b) {
                union_small_root(&mut parent, m.a, m.b);
            }
        }

        // 2) 岛桶。体按索引升序；岛内流形按全局流形序（§4.14 确定性模式）。
        //    只有「与清醒体连通」的体参与建岛：
        //    - 第一遍：清醒动体建岛（睡眠体不建岛 → 全睡眠帧岛构建 ≈ O(查是否为空)）；
        //    - 第二遍：睡眠动体若其连通分量已被激活（root 已在槽位表）则并入——
        //      保证「被撞唤醒」的接触对里有沉睡侧的体（求解冲量要施加到它并唤醒）。
        //    岛池跨帧复用（Vec 容量保留），全清醒场景（10 万岛）不再逐帧分配。
        let mut root_slot: HashMap<u32, usize> = HashMap::new();
        let mut pool = std::mem::take(&mut self.island_pool);
        let mut islands_used = 0usize;
        for i in 0..n {
            if !(bodies.is_dynamic(i) && bodies.awake[i]) {
                continue;
            }
            let r = find_small_root(&mut parent, i as u32);
            let slot = *root_slot.entry(r).or_insert_with(|| {
                if islands_used == pool.len() {
                    pool.push(Island {
                        bodies: Vec::new(),
                        manifs: Vec::new(),
                    });
                }
                let s = islands_used;
                pool[s].bodies.clear();
                pool[s].manifs.clear();
                islands_used += 1;
                s
            });
            pool[slot].bodies.push(i as u32);
        }
        if !root_slot.is_empty() {
            // 睡眠侧并入（其根已被清醒体激活的连通分量）。
            for i in 0..n {
                if !bodies.is_dynamic(i) || bodies.awake[i] {
                    continue;
                }
                let r = find_small_root(&mut parent, i as u32);
                if let Some(&slot) = root_slot.get(&r) {
                    pool[slot].bodies.push(i as u32);
                }
            }
            for (mi, m) in manifolds.iter().enumerate() {
                let (a, b) = (m.a as usize, m.b as usize);
                if !bodies.is_dynamic(a) && !bodies.is_dynamic(b) {
                    continue;
                }
                let root = if bodies.is_dynamic(a) {
                    find_small_root(&mut parent, m.a)
                } else {
                    find_small_root(&mut parent, m.b)
                };
                if let Some(&slot) = root_slot.get(&root) {
                    pool[slot].manifs.push(mi);
                }
            }
        }
        let islands = &pool[..islands_used];
        self.parent = parent;
        self.island_count = islands.len();

        // 3) 清醒岛（任一成员 awake → 全岛解算）；沉睡岛整体跳过（上面的建岛已
        //    只收「与清醒体连通」的岛，因此这里恒为全清醒）。分组（连续岛段）：
        //    组内岛串行、组间体集合不相交 → 并行（§6），gather→solve→scatter
        //    走组内 scratch 速度缓冲（岛间本就无浮点交互，§5 → 与串行 bit 级一致）。
        let awake: Vec<usize> = (0..islands.len()).collect();
        // 并行门槛：spawn ≈ 90µs/个（Windows 实测）；流形 < 4096 时并行不划算
        // （解算工作量 ≈ 1µs/接触/帧），走单组串行（数值路径不变）。
        let g_count = if threads <= 1 || manifolds.len() < 4096 {
            1
        } else {
            awake.len().min(threads).max(1)
        };

        let mut warm_slots = std::mem::take(&mut self.warm_slots);
        let mut warm_free = std::mem::take(&mut self.warm_free);
        let mut warm_index = std::mem::take(&mut self.warm_index);
        let mut build_bufs = std::mem::take(&mut self.build_bufs);
        let mut warm_outs = std::mem::take(&mut self.warm_outs);
        let mut group_lv = std::mem::take(&mut self.group_lv);
        let mut group_av = std::mem::take(&mut self.group_av);
        let mut local_of = std::mem::take(&mut self.local_of);
        let mut group_iw = std::mem::take(&mut self.group_iw);
        let mut group_im = std::mem::take(&mut self.group_im);

        build_bufs.resize_with(g_count, Vec::new);
        warm_outs.resize_with(g_count, Vec::new);
        group_lv.resize_with(g_count, Vec::new);
        group_av.resize_with(g_count, Vec::new);
        group_iw.resize_with(g_count, Vec::new);
        group_im.resize_with(g_count, Vec::new);
        local_of.clear();
        local_of.resize(n, u32::MAX);
        let mut groups: Vec<(usize, usize)> = Vec::with_capacity(g_count);
        // gather：组 g 的岛体速度拷入组内 scratch（顺序 = 岛序 = scatter 序）。
        // 分组切分用比例式（g·n/g_count）：`岛数 < 组数` 时尾部组为空区间
        // ——旧式 `g*ceil(n/g)` 会产出 start > n 的越界区间（9 岛 8 组实测 panic）。
        for g in 0..g_count {
            build_bufs[g].clear();
            warm_outs[g].clear();
            group_lv[g].clear();
            group_av[g].clear();
            group_iw[g].clear();
            group_im[g].clear();
            let s0 = g * awake.len() / g_count;
            let e0 = (g + 1) * awake.len() / g_count;
            groups.push((s0, e0));
            for &ii in &awake[s0..e0] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    local_of[i] = group_lv[g].len() as u32;
                    group_lv[g].push(bodies.linvel[i]);
                    group_av[g].push(bodies.angvel(i));
                    // 每帧一次：世界逆惯量矩阵（帧内姿态不变，求解只改速度；
                    // M = R·diag(inv_local)·Rᵀ ⇒ 求解环每点每轮只需一次矩阵乘）。
                    group_iw[g].push(Mat3::world_inv_inertia(
                        bodies.rot(i),
                        bodies.local_inv_inertia[i],
                    ));
                    group_im[g].push(bodies.inv_mass[i]);
                }
            }
        }

        let d_island = vxl_phys_core::probe::us(t_island);
        let t_solve = vxl_phys_core::probe::start();
        // 4) 并行解算（§6 契约：组间写槽位不相交，组内 = 串行语义）。
        if g_count > 1 {
            let bodies_ref: &BodySet = bodies;
            let awake_ref: &[usize] = &awake;
            let islands_ref: &[Island] = islands;
            let warm_index_ref: &HashMap<WarmKey, u32> = &warm_index;
            let warm_slots_ref: &[(WarmKey, WarmManifold)] = &warm_slots;
            let local_ref: &[u32] = &local_of;
            let iw_ref: &[Vec<Mat3>] = &group_iw;
            let im_ref: &[Vec<f32>] = &group_im;
            let sp_ref: &SolverParams = &sp;
            // 诊断细分：每组一份累加器（闭包按 move 捕获，不能共享一个可变借用）。
            let mut details: Vec<[u64; 5]> = vec![[0; 5]; g_count];
            std::thread::scope(|s| {
                // iter_mut 逐容器取出元素可变借用（按 g 索引整体借用会跨迭代重叠）。
                for (g, (((lv, av), (cbuf, wout)), det)) in group_lv
                    .iter_mut()
                    .zip(group_av.iter_mut())
                    .zip(build_bufs.iter_mut().zip(warm_outs.iter_mut()))
                    .zip(details.iter_mut())
                    .enumerate()
                {
                    let (s0, e0) = groups[g];
                    let iw_g: &[Mat3] = &iw_ref[g];
                    let im_g: &[f32] = &im_ref[g];
                    let job = move || {
                        solve_island_group(
                            &awake_ref[s0..e0],
                            islands_ref,
                            manifolds,
                            bodies_ref,
                            warm_index_ref,
                            warm_slots_ref,
                            local_ref,
                            iw_g,
                            im_g,
                            lv,
                            av,
                            cbuf,
                            wout,
                            iters,
                            e_threshold,
                            match_dist,
                            shock,
                            normal_inner,
                            sp_ref,
                            det,
                        );
                    };
                    if g + 1 == g_count {
                        let mut job = job;
                        job();
                    } else {
                        s.spawn(job);
                    }
                }
            });
            let mut tot = [0u64; 5];
            for d in &details {
                for (k, v) in d.iter().enumerate() {
                    tot[k] += v;
                }
            }
            for (k, v) in tot.iter().enumerate().take(3) {
                self.last_detail_us[k + 1] += v;
            }
            self.last_points = (tot[3], tot[4]);
        } else if !awake.is_empty() {
            let mut det = [0u64; 5];
            solve_island_group(
                &awake,
                islands,
                manifolds,
                bodies,
                &warm_index,
                &warm_slots,
                &local_of,
                &group_iw[0],
                &group_im[0],
                &mut group_lv[0],
                &mut group_av[0],
                &mut build_bufs[0],
                &mut warm_outs[0],
                iters,
                e_threshold,
                match_dist,
                shock,
                normal_inner,
                &sp,
                &mut det,
            );
            for (k, v) in det.iter().enumerate().take(3) {
                self.last_detail_us[k + 1] += v;
            }
            self.last_points = (det[3], det[4]);
        }

        // scatter：组序 = gather 序 → 局部索引一一对应（确定性）。
        for g in 0..g_count {
            let mut k = 0usize;
            for &ii in &awake[groups[g].0..groups[g].1] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    bodies.linvel[i] = group_lv[g][k];
                    bodies.set_angvel_raw(i, group_av[g][k]);
                    k += 1;
                }
            }
        }

        // （M1 软接触形态起，位置修正走 erp 偏置速度进速度通道 + CFM 正则化，
        //  独立「分裂冲量偏置通道 + 位移写回」已退役——见 SolverParams。）

        // —— warm 槽位表：回写（按槽号原位写）+ 剪枝（对稠密槽单遍扫）——
        //
        // 设计（DESIGN-staged-solver §11）：`HashMap` 桶遍历/插入曾占 33.5ms/tick
        // （合并 13.1 + 剪枝 20.7，均为桶访存）。槽位表把「值」搬到连续内存：
        // 回写零哈希（直接按槽号写）、剪枝单遍顺序扫、索引只存 (键 → u32)。
        //
        // 剪枝规则不变：流形已消失的**双清醒**对才删——睡眠体不移动，其接触
        // 不会真正消失（睡眠期不被检测只是省算力），若一并剪掉，唤醒后 warm
        // 起点归零会导致数帧收敛变弱（穿透加深）。以「本调用是否刷新过」的
        // 印章判定「流形是否仍在」：有流形且≥1 体清醒 ⇒ 必属清醒岛 ⇒ 必被
        // 求解盖章；双睡的有流形但不被求解，两条规则都因「非双清醒」保留。
        self.warm_stamp = self.warm_stamp.wrapping_add(1);
        let stamp = self.warm_stamp;
        for wo in warm_outs.drain(..) {
            for (slot, key, mut v) in wo {
                v.seen = stamp;
                if slot != u32::MAX {
                    warm_slots[slot as usize] = (key, v); // 原位写：零哈希
                } else if let Some(free) = warm_free.pop() {
                    warm_slots[free as usize] = (key, v);
                    warm_index.insert(key, free);
                } else {
                    warm_slots.push((key, v));
                    warm_index.insert(key, (warm_slots.len() - 1) as u32);
                }
            }
        }
        if manifolds.is_empty() {
            warm_slots.clear();
            warm_index.clear();
            warm_free.clear();
        } else {
            // 稠密单遍剪枝（顺序访存）。
            for (i, (key, v)) in warm_slots.iter_mut().enumerate() {
                let (a, b, _space) = *key;
                if a == u32::MAX {
                    continue; // 已是空洞
                }
                let both_awake = bodies.awake[a as usize] && bodies.awake[b as usize];
                if v.seen != stamp && both_awake {
                    warm_index.remove(key);
                    *key = DEAD_KEY;
                    warm_free.push(i as u32);
                }
            }
        }
        self.warm_slots = warm_slots;
        self.warm_free = warm_free;
        self.warm_index = warm_index;
        self.build_bufs = build_bufs;
        self.warm_outs = warm_outs;
        self.group_lv = group_lv;
        self.group_av = group_av;
        self.local_of = local_of;
        self.group_iw = group_iw;
        self.group_im = group_im;

        let d_solve = vxl_phys_core::probe::us(t_solve);
        let t_sleep = vxl_phys_core::probe::start();
        // 5) 岛级休眠与唤醒（§4.11 / §3 稳定性）。
        //    - 建岛阶段已只收「与清醒体连通」的岛（含被牵连的睡眠体），
        //      遗漏的睡眠体天然保持冻结（不解算不积分）；
        //    - 清醒岛：任一成员 awake → 全岛同步为 awake（外部唤醒传播）；
        //    - 全员速度低于阈值持续 sleep_time → 岛内**原子**入睡（同帧全员睡），
        //      不存在"部分睡部分醒"状态，从机制上排除反复唤醒；
        //    - 无接触的孤立清醒动体 = 单成员岛，走同一套休眠判定。
        //    无偏置趟不做休眠判定（同一子步内带偏置趟已判过；重复判定会让
        //    sleep_timer 每次子步双倍累积 ⇒ 入睡提前，属非本意行为改变）。
        let sleep_islands: &[Island] = if cleanup { &[] } else { islands };
        for island in sleep_islands {
            for &bi in &island.bodies {
                let i = bi as usize;
                if !bodies.awake[i] {
                    bodies.awake[i] = true;
                    bodies.sleep_timer[i] = 0.0;
                }
            }
            let mut all_slow = true;
            for &bi in &island.bodies {
                let i = bi as usize;
                let lin = bodies.linvel[i].length();
                let ang = bodies.angvel(i).length();
                if lin >= config.sleep_linear || ang >= config.sleep_angular {
                    all_slow = false;
                    break;
                }
            }
            if all_slow {
                let mut min_timer = f32::MAX;
                for &bi in &island.bodies {
                    let i = bi as usize;
                    bodies.sleep_timer[i] += dt;
                    min_timer = min_timer.min(bodies.sleep_timer[i]);
                }
                if min_timer >= config.sleep_time {
                    for &bi in &island.bodies {
                        let i = bi as usize;
                        bodies.awake[i] = false;
                        bodies.linvel[i] = Vec3::ZERO;
                        bodies.set_angvel_raw(i, Vec3::ZERO);
                    }
                }
            } else {
                for &bi in &island.bodies {
                    let i = bi as usize;
                    bodies.sleep_timer[i] = 0.0;
                    self.sleep_resets += 1;
                }
            }
        }
        self.island_pool = pool;
        self.last_phase_us = (d_island, d_solve, vxl_phys_core::probe::us(t_sleep), 0);
    }
}

/// 构建单个流形的接触约束（预计算质量项/bias/warm）。摩擦/恢复按材质对
/// 组合（§4.4/§4.5）；静+动档 μ 随预解相对切向速度在 [μk, μs] 线化过渡
/// （vt 取首个接触点，确定性）。切向基提升到流形级（原为每点重算）。
#[allow(clippy::too_many_arguments)]
fn build_constraint(
    out: &mut Vec<ContactConstraint>,
    m: &Manifold,
    bodies: &BodySet,
    warm_index: &HashMap<WarmKey, u32>,
    warm_slots: &[(WarmKey, WarmManifold)],
    match_dist: f32,
    e_threshold: f32,
    sp: &SolverParams,
) {
    let (a, b) = (m.a as usize, m.b as usize);
    let da = bodies.is_dynamic(a);
    let db = bodies.is_dynamic(b);
    if !da && !db {
        return;
    }
    let mat_a = bodies
        .material
        .get(a)
        .and_then(|&id| bodies.materials.get(id as usize))
        .copied()
        .unwrap_or_default();
    let mat_b = bodies
        .material
        .get(b)
        .and_then(|&id| bodies.materials.get(id as usize))
        .copied()
        .unwrap_or_default();
    let (mu, e) = {
        let stribeck = matches!(
            mat_a.friction,
            vxl_phys_core::FrictionModel::StaticKinetic { .. }
        ) || matches!(
            mat_b.friction,
            vxl_phys_core::FrictionModel::StaticKinetic { .. }
        );
        if stribeck {
            let cp = &m.points[0];
            let ra0 = cp.point - bodies.position[a];
            let rb0 = cp.point - bodies.position[b];
            let vrel = bodies.velocity_at(b, rb0) - bodies.velocity_at(a, ra0);
            let vt = (vrel - m.normal * vrel.dot(m.normal)).length();
            (
                vxl_phys_core::Material::combine_friction_stribeck(
                    &mat_a.friction,
                    &mat_b.friction,
                    vt,
                ),
                mat_a.restitution.max(mat_b.restitution),
            )
        } else {
            vxl_phys_core::Material::combine(&mat_a, &mat_b)
        }
    };

    // 槽位表查找：索引给槽号，数据在稠密槽位里（回写按槽号原位写）。
    //
    // **键的第三维 = 特征空间**（`ContactPoints::space()`，**显式通道**）：同一体对会同时有多条
    // 流形（复合体每个子形状一条），共用 `(a, b)` 键会相互覆盖 ⇒ 该对暖缓存整条失效。
    // 非复合体恒 0 ⇒ 键退化为 `(a, b, 0)`。
    let warm_space = m.points.space() as u32;
    let warm_slot = warm_index
        .get(&(m.a, m.b, warm_space))
        .copied()
        .unwrap_or(u32::MAX);
    let warmm = if warm_slot == u32::MAX {
        None
    } else {
        Some(&warm_slots[warm_slot as usize].1)
    };
    let (t1, t2) = tangents(m.normal);
    let pos_a = bodies.position[a];
    let pos_b = bodies.position[b];
    // 接触回收（Rapier `contact_recycling` 同族）：锚点存双方体局部坐标，
    // 帧间还原到世界系并只推进距离——锚点固定在材料点上 ⇒ 力臂/暖启动/
    // 切向漂移判断跨帧稳定（裁剪点每帧重算的几何噪声被消除）。
    let rot_a = vxl_phys_core::Mat3::from_quat(bodies.rot(a));
    let rot_b = vxl_phys_core::Mat3::from_quat(bodies.rot(b));
    // 锚点当前世界位置（≤4 点定长数组，零分配；匹配/复用判定共用）。
    let mut warm_world = [Vec3::ZERO; 4];
    let mut warm_n = 0usize;
    if let Some(wm) = warmm {
        for w in wm.pts() {
            warm_world[warm_n] =
                (pos_a + rot_a.mul_vec3(w.la) + pos_b + rot_b.mul_vec3(w.lb)) * 0.5;
            warm_n += 1;
        }
    }
    let mut pts = [PointConstraint::default(); 4];
    let mut npts = 0u8;
    for cp in &m.points {
        // warm starting 匹配：① 特征 ID 精确匹配（带距离护栏，锚点当前世界
        // 位置量距）；② 近邻回退（无特征或 ID 未命中）。
        let mut warm_pt: Option<WarmPoint> = None;
        if let Some(wm) = warmm {
            if warm_n > 0 && wm.normal.dot(m.normal) > 0.95 {
                if cp.feature != 0 {
                    warm_pt = (0..warm_n)
                        .find(|&k| {
                            wm.points[k].feature == cp.feature
                                && (warm_world[k] - cp.point).length_squared()
                                    < match_dist * match_dist
                        })
                        .map(|k| wm.points[k]);
                    if warm_pt.is_some() {
                        WARM_EXACT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    }
                }
                if warm_pt.is_none() {
                    // **回退分支单独收紧接受半径**（上一轮实测结论的直接产物）：
                    // 回退（特征未命中）按 4×skin ≈4cm 接受时会把**错配锚点**也认下来
                    // ⇒ 帧间「漂移」= 两点真实错位 ⇒ 暖启动冲量过驱动速度
                    // ⇒ 堆族永不入睡（实测：整个关掉回退使 col45 45/45、
                    // pile5 1985/2000 全睡且位置更准，但塔崩）。⇒ 收到 **1×skin**：
                    // 只认「确实还是同一个材料点」的候选；拒配 ⇒ 按新接触处理、不暖启动。
                    let fb = match_dist * 0.25;
                    warm_pt = (0..warm_n)
                        .filter(|&k| {
                            // 接触状态门（非距离信号）：分离/预期接触不继承
                            // 上一帧的暖冲量（错配冲量过驱动间歇角点接触）。
                            if cp.depth <= FB_DEPTH_MIN {
                                return false;
                            }
                            if (warm_world[k] - cp.point).length_squared() >= fb * fb {
                                return false;
                            }
                            // 深度跳变门（非距离信号）：|本帧裁剪深度 − 锚点烘焙深度|
                            // 超限 ⇒ 拒配。见 FB_DEPTH_JUMP 注（含「恒等式陷阱」教训）。
                            let w = wm.points[k];
                            (cp.depth - w.depth0).abs() <= FB_DEPTH_JUMP
                        })
                        .min_by(|&x, &y| {
                            (warm_world[x] - cp.point)
                                .length_squared()
                                .total_cmp(&(warm_world[y] - cp.point).length_squared())
                        })
                        .map(|k| wm.points[k]);
                    if let Some(w) = warm_pt {
                        use std::sync::atomic::Ordering::Relaxed;
                        WARM_FALLBACK.fetch_add(1, Relaxed);
                        // 参考面代理：法向是否变（见 `warm_normal_flip_take` 注）。
                        if wm.normal.dot(m.normal) < 0.99999 {
                            WN_FLIP.fetch_add(1, Relaxed);
                        } else {
                            WN_SAME.fetch_add(1, Relaxed);
                        }
                        // 成因分解（诊断，只计数）：见 `vxl_phys_narrow::feature_kind` 注。
                        let (s_new, c_new) = vxl_phys_narrow::feature_kind(cp.feature);
                        let (s_old, c_old) = vxl_phys_narrow::feature_kind(w.feature);
                        if s_new != s_old {
                            WB_SIDE.fetch_add(1, Relaxed);
                        } else if c_new != c_old {
                            WB_CLIP.fetch_add(1, Relaxed);
                        } else if vxl_phys_narrow::feature_hash_part(cp.feature)
                            != vxl_phys_narrow::feature_hash_part(w.feature)
                        {
                            WB_HASH.fetch_add(1, Relaxed);
                        } else {
                            WB_SAME.fetch_add(1, Relaxed);
                        }
                    }
                }
                if warm_pt.is_none() {
                    WARM_MISS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                }
            }
        }

        // 复用判定 + 有效几何：锚点世界分离 ≤ 回收半径 → 沿用锚点（深度/漂移
        // 由锚点分离量推进）；否则按本帧裁剪点烘焙新锚点（la/lb 同源 ⇒ 初始分离 0）。
        let (pt_world, depth, depth0, la, lb, drift) = match warm_pt {
            Some(w) => {
                let pa = pos_a + rot_a.mul_vec3(w.la);
                let pb = pos_b + rot_b.mul_vec3(w.lb);
                let sep_v = pa - pb;
                if sep_v.length_squared() <= RECYCLE_DIST * RECYCLE_DIST {
                    // 有效深度 = 烘焙深度 + 当前累计分离沿法向的投影（非增量式
                    // 复合——烘焙深度全程不变，见 WarmPoint::depth0）。
                    (
                        (pa + pb) * 0.5,
                        w.depth0 + sep_v.dot(m.normal),
                        w.depth0,
                        w.la,
                        w.lb,
                        sep_v,
                    )
                } else {
                    let (la, lb) = (
                        rot_a.transpose_mul_vec3(cp.point - pos_a),
                        rot_b.transpose_mul_vec3(cp.point - pos_b),
                    );
                    (cp.point, cp.depth, cp.depth, la, lb, Vec3::ZERO)
                }
            }
            None => {
                let (la, lb) = (
                    rot_a.transpose_mul_vec3(cp.point - pos_a),
                    rot_b.transpose_mul_vec3(cp.point - pos_b),
                );
                (cp.point, cp.depth, cp.depth, la, lb, Vec3::ZERO)
            }
        };
        let ra = pt_world - pos_a;
        let rb = pt_world - pos_b;
        let nmass = contact_mass(
            bodies.inv_mass[a],
            bodies.inv_mass[b],
            ra,
            rb,
            m.normal,
            bodies,
            a,
            b,
        );
        let tmass1 = contact_mass(
            bodies.inv_mass[a],
            bodies.inv_mass[b],
            ra,
            rb,
            t1,
            bodies,
            a,
            b,
        );
        let tmass2 = contact_mass(
            bodies.inv_mass[a],
            bodies.inv_mass[b],
            ra,
            rb,
            t2,
            bodies,
            a,
            b,
        );
        // 切向联立解的交叉项（对角逐轴解的残差泵能问题，见 contact_mass_cross）。
        let tcross = contact_mass_cross(
            bodies.inv_mass[a],
            bodies.inv_mass[b],
            ra,
            rb,
            t1,
            t2,
            bodies,
            a,
            b,
        );

        // —— M1 软接触目标（TGS-Soft 语义，Rapier 0.35 同构；深度为回收后有效值）——
        // 弹性：预解相对法向速度。
        let va = bodies.velocity_at(a, ra);
        let vb = bodies.velocity_at(b, rb);
        let vn = (vb - va).dot(m.normal);
        let bounce = if vn < -e_threshold { -e * vn } else { 0.0 };
        // speculative：浅缝（depth<0）允许一个 tick 内闭合剩余间隙（不再提前悬停）；
        // 去穿透：erp·(depth−slop)，钳 max_corrective_velocity（穿透→分离速度目标）。
        let sep = -depth;
        let spec = sep.max(0.0) * sp.inv_dt;
        let is_static_pair = bodies.inv_mass[a] == 0.0 || bodies.inv_mass[b] == 0.0;
        let erp_inv_dt = if is_static_pair {
            sp.erp_inv_dt_static
        } else {
            sp.erp_inv_dt_dyn
        };
        let pen = (depth - sp.slop).max(0.0);
        let bias = (erp_inv_dt * pen).min(sp.max_corr);
        // 诊断记账（只计数，不改行为；见 `solve_accounting_take`）。
        {
            use std::sync::atomic::Ordering::Relaxed;
            DA_PTS.fetch_add(1, Relaxed);
            if depth < 0.0 {
                DA_SEP.fetch_add(1, Relaxed);
            }
            DA_SPEC.fetch_add((spec.max(0.0) * 1000.0) as u64, Relaxed);
            DA_BIAS.fetch_add((bias * 1000.0) as u64, Relaxed);
        }
        let rhs = bias - spec + bounce;
        // 正则化：穿透接触 cfm=1（硬投影，支撑刚性）；speculative 接触 cfm<1
        // （等效柔度 ω/ζ，限制迭代增益，深堆不依赖跨层链收敛——金样定标结论）。
        let cfm = if depth >= 0.0 {
            1.0
        } else if is_static_pair {
            sp.cfm_static
        } else {
            sp.cfm_dyn
        };
        // 切向锚点漂移回拉（Rapier 切向 rhs 同义）：目标 `v_t = (pa−pb)·t·inv_dt`，
        // 由摩擦在锥内执行——粘着接触的材料点错位被回正（回收锚点 ⇒ 漂移为
        // 真实滑移量，非裁剪几何噪声；旧 shortcut 无锚点实测变差已回退）。
        // **切向回拉只在"非滑动"接触上做**（见 `DRIFT_STICK_RATIO` 注）：滑动接触的锚点
        // 分离是**真实材料滑移**，回拉等于抹掉真实滑动、并 ∝ μ 地注转矩（角向残差主项）；
        // 粘着接触的分离才是数值错位。判定用上一子步的暖启动冲量（零额外状态）。
        let sliding = warm_pt.is_none_or(|w| {
            (w.pt1 * w.pt1 + w.pt2 * w.pt2).sqrt() >= DRIFT_STICK_RATIO * mu * w.pn.max(0.0)
        });
        let drift_eff = if !sliding && drift.length_squared() > DRIFT_DEADZONE * DRIFT_DEADZONE {
            drift * (DRIFT_BIAS_SCALE * sp.inv_dt)
        } else {
            Vec3::ZERO
        };
        let trhs1 = drift_eff.dot(t1);
        let trhs2 = drift_eff.dot(t2);

        pts[npts as usize] = PointConstraint {
            ra,
            rb,
            t1,
            t2,
            nmass,
            tmass1,
            tmass2,
            tcross,
            trhs1,
            trhs2,
            la,
            lb,
            depth0,
            rhs,
            cfm,
            friction: mu,
            pn: warm_pt.map(|w| w.pn).unwrap_or(0.0),
            pt1: warm_pt.map(|w| w.pt1).unwrap_or(0.0),
            pt2: warm_pt.map(|w| w.pt2).unwrap_or(0.0),
            feature: cp.feature,
            warm: warm_pt,
        };
        npts += 1;
    }
    if npts == 0 {
        return;
    }
    out.push(ContactConstraint {
        a: m.a,
        b: m.b,
        normal: m.normal,
        warm_slot,
        warm_space,
        points: pts,
        npts,
    });
}

/// 单约束一次顺序冲量迭代（法向 × 内层扫掠 + 两切向 + 摩擦锥 + 偏置）。
///
/// 两通道：**歧管内层 `inner` 次扫掠（法向 + 摩擦一起）**。法向/切向慢模
/// 都在歧管内部（4 点冗余约束 + 强转动耦合 + 锥耦合）；内层 K 遍把歧管内
/// 收敛提到 ≈K×外层（代价只加歧管局部工作量）。法向为 M1 软接触更新式
/// （正则化 cfm + 目标 rhs，见 `SolverParams`）。`rev` = 逆序遍历
/// （对称扫掠，配合外层正/反交替 ≈ ρ²）。
#[inline]
#[allow(clippy::too_many_arguments)] // 热路径内联目标：避免打包结构体的构造成本
/// 收敛早退阈值（速度级残差，m/s）：一次外层扫掠内**最大速度修正**低于它即视
/// 该岛已收敛、停止剩余外层迭代。标定与判据见 EXPERIMENTS「求解预算标定」：
/// 安静堆叠期 12 次外层是纯浪费；判据是状态的纯函数（同状态 ⇒ 同退出点），
/// 不引入任何时序/线程依赖，串并行逐位一致仍成立。0 = 关闭（等价纯定档）。
fn early_exit_eps() -> f32 {
    0.002
}

/// 早退前至少跑满的外层迭代数（堆叠建立期不早退；须为偶数以保持正/反扫掠对称）。
fn early_min_iters() -> u32 {
    4
}

/// **参与式降点**的起始轮次（0 = 关闭）。从第 N 轮外层扫掠起，跳过"至今零冲量
/// 的浅缝接触点"（`pn == 0 && pt1 == 0 && pt2 == 0 && cfm < 1.0`）。
///
/// 动机（实测，见 EXPERIMENTS「零冲量点占比」）：解算末态有 **22–35% 的接触点
/// 法向冲量为 0**（金字塔 2233 点里 784 个），而它们在每个扫掠里都要付"法向 +
/// 摩擦"两段迭代（扫掠占帧 ≈50%）。前几轮全点参与、之后跳过零冲量点 ⇒ 理论上
/// 省下这部分空转。
///
/// 保守设计（为什么不是硬删点）：
/// - 只跳**浅缝/软接触**点（`cfm < 1.0` ⇒ speculative，非穿透硬投影）：穿透接触
///   承载刚性与抗转，绝不跳；
/// - 前 `N-1` 轮全点参与：需要承载的点已经在早期轮次载上（4 点冗余约束的
///   抗转刚度在早期建立）；
/// - 点仍在 warm 槽表里、`pn/pt1/pt2` 保留 ⇒ 下一帧若被 feature 匹配回来，
///   起点仍是它的历史值，不会"复活式弹跳"。
///
/// 确定性：判据只依赖本帧已累积的冲量与轮次序号（纯状态的函数），不引入时序/线程
/// 依赖 ⇒ 串并行逐位一致仍成立。
///
/// **默认 3（实测落地，2026-09-15）**：金字塔交错 A/B 三轮 p50
/// 1.528/1.525/1.491 → **1.397/1.414/1.403 ms（−7.3%）**；三道保真门都过——
/// 金样 col45 与关闭态**完全一致**（45/45 入睡、最深 0.000）、pile5 **更好**
/// （1735/2000 入睡 vs 1690、|v| 0.075 vs 0.096）、tower25 |v| 0.224（关闭态 0.200，
/// 两者都优于记录基线 0.236）；长跑 3000 步堆顶 19.426（关闭态 19.407）、末态动能
/// **1.47 vs 1.74（更稳）**。换代哈希：`0x6219d186…` / `0x63e5eb35…` / `0xd8601988…`。
/// 0 = 关闭（回到旧行为，逐位复现旧世代哈希）。
fn point_reduce_after() -> u32 {
    3
}

/// 顺序冲量解算一条约束；返回本次扫掠施加的**最大速度级修正**（m/s，
/// 法向 + 摩擦通道的最大值），供收敛早退判据使用。
#[allow(clippy::too_many_arguments)] // 热路径内联目标：避免打包结构体的构造成本
fn solve_constraint(
    c: &mut ContactConstraint,
    lv: &mut [Vec3],
    av: &mut [Vec3],
    local_of: &[u32],
    iw: &[Mat3],
    im: &[f32],
    rev: bool,
    inner: u32,
    reduce: bool,
) -> f32 {
    let (ai, bi) = (c.a as usize, c.b as usize);
    let normal = c.normal;
    let npts = c.npts as usize;
    let mut resid = 0.0f32;

    // —— 歧管内层扫掠（法向 + 摩擦两通道一起）——
    for _ in 0..inner.max(1) {
        for k in 0..npts {
            let idx = if rev { npts - 1 - k } else { k };
            let p = &mut c.points[idx];
            // 参与式降点：后期轮次跳过"至今零冲量的浅缝点"（见 `point_reduce_after`）。
            if reduce && p.pn == 0.0 && p.pt1 == 0.0 && p.pt2 == 0.0 && p.cfm < 1.0 {
                continue;
            }
            // —— 法向（M1 软接触）：λ ← cfm·(λ + m·(rhs − vn))，钳 ≥ 0 ——
            let va = group_vel(lv, av, local_of, ai, p.ra);
            let vb = group_vel(lv, av, local_of, bi, p.rb);
            let vn = (vb - va).dot(normal);
            let new_pn = (p.cfm * (p.pn + p.nmass * (p.rhs - vn))).max(0.0);
            let dl = new_pn - p.pn;
            p.pn = new_pn;
            if dl != 0.0 {
                // 速度级残差：冲量增量 × 有效质量 = 该点速度修正（m/s）。
                resid = resid.max((dl * p.nmass).abs());
                let imp = normal * dl;
                group_apply(lv, av, local_of, iw, im, ai, p.ra, imp, true);
                group_apply(lv, av, local_of, iw, im, bi, p.rb, imp, false);
            }
            // —— 摩擦（**精确 2×2 联立切向解** + 径向锥投影）——
            // Rapier `contact_constraint_element.rs` 同式（Δ = −K⁻¹·dvel 后
            // cap_magnitude(μ·pn)）：K = 切向有效质量矩阵（含交叉项 k12）。
            // 对角逐轴解在 K 各向异性时留正交残差、旋转能量——「把大堆的
            // 摩擦模式泵成弹射/蠕动」（金样源码注释原文，即本引擎塔蠕动的根因）。
            // k12=0 时退化为原逐轴解（逐式等价）。
            {
                let va = group_vel(lv, av, local_of, ai, p.ra);
                let vb = group_vel(lv, av, local_of, bi, p.rb);
                let dv = vb - va;
                // 目标 v_t = trhs（锚点漂移回拉；无漂移时为 0 = 原「抑制滑动」语义）。
                let vt1 = dv.dot(p.t1) - p.trhs1;
                let vt2 = dv.dot(p.t2) - p.trhs2;
                let k11 = if p.tmass1 > 0.0 { 1.0 / p.tmass1 } else { 0.0 };
                let k22 = if p.tmass2 > 0.0 { 1.0 / p.tmass2 } else { 0.0 };
                let k12 = p.tcross;
                let det = k11 * k22 - k12 * k12;
                if det > 1e-12 {
                    let inv = 1.0 / det;
                    let dv1 = (-vt1 * k22 + vt2 * k12) * inv;
                    let dv2 = (-vt2 * k11 + vt1 * k12) * inv;
                    let (old1, old2) = (p.pt1, p.pt2);
                    let (mut a1, mut a2) = (old1 + dv1, old2 + dv2);
                    let max_f = p.friction * p.pn;
                    let f2 = a1 * a1 + a2 * a2;
                    if f2 > max_f * max_f && f2 > 1e-20 {
                        let s = max_f / f2.sqrt();
                        a1 *= s;
                        a2 *= s;
                    }
                    p.pt1 = a1;
                    p.pt2 = a2;
                    let d1 = a1 - old1;
                    let d2 = a2 - old2;
                    if d1 != 0.0 || d2 != 0.0 {
                        resid = resid.max((d1 * p.tmass1).abs()).max((d2 * p.tmass2).abs());
                        let imp = p.t1 * d1 + p.t2 * d2;
                        group_apply(lv, av, local_of, iw, im, ai, p.ra, imp, true);
                        group_apply(lv, av, local_of, iw, im, bi, p.rb, imp, false);
                    }
                }
            }
        }
    }
    resid
}

/// 解算一组清醒岛（组内岛串行；岛间体集合不相交）。速度读写走组内 scratch
/// （gather 已填充），warm 更新收集到 `warm_out`（调用方按组序合并）。
#[allow(clippy::too_many_arguments)]
fn solve_island_group(
    awake: &[usize],
    islands: &[Island],
    manifolds: &[Manifold],
    bodies: &BodySet,
    warm_index: &HashMap<WarmKey, u32>,
    warm_slots: &[(WarmKey, WarmManifold)],
    local_of: &[u32],
    iw: &[Mat3],
    im: &[f32],
    lv: &mut [Vec3],
    av: &mut [Vec3],
    cbuf: &mut Vec<ContactConstraint>,
    warm_out: &mut Vec<WarmOutEntry>,
    iters: u32,
    e_threshold: f32,
    match_dist: f32,
    shock_iterations: u32,
    normal_inner: u32,
    sp: &SolverParams,
    detail: &mut [u64; 5],
) {
    for &ii in awake {
        let isl = &islands[ii];
        // 每岛构建约束（岛内流形序 = 全局流形序，§4.14）。
        let t_build = vxl_phys_core::probe::start();
        cbuf.clear();
        for &mi in &isl.manifs {
            build_constraint(
                cbuf,
                &manifolds[mi],
                bodies,
                warm_index,
                warm_slots,
                match_dist,
                e_threshold,
                sp,
            );
        }
        detail[0] += vxl_phys_core::probe::us(t_build);
        // warm starting 预施加（每约束一次）。
        let t_warm = vxl_phys_core::probe::start();
        for c in cbuf.iter() {
            let (ai, bi) = (c.a as usize, c.b as usize);
            for p in &c.points[..c.npts as usize] {
                if let Some(w) = p.warm {
                    if w.pn == 0.0 && w.pt1 == 0.0 && w.pt2 == 0.0 {
                        continue;
                    }
                    let impulse = c.normal * w.pn + p.t1 * w.pt1 + p.t2 * w.pt2;
                    group_apply(lv, av, local_of, iw, im, ai, p.ra, impulse, true);
                    group_apply(lv, av, local_of, iw, im, bi, p.rb, impulse, false);
                }
            }
        }
        detail[1] += vxl_phys_core::probe::us(t_warm);
        // 顺序冲量迭代（岛内顺序 = 流形序 = 约束构建序，§4.14）。
        // 对称扫掠：偶数迭代正序、奇数迭代反序（约束序 + 接触点序同时反转）——
        // 4 点面接触是冗余约束 + 强转动耦合，单向 GS 16 次迭代后残留可观
        // （重堆实测微抖永不入睡）；正/反交替后收敛率 ≈ ρ²。
        // 确定性：方向仅由迭代序决定。
        //
        // **收敛早退**（2026-09-15 标定）：每次扫掠累计"最大速度级修正"残差，
        // 跑满 `early_min_iters` 后残差 < eps 即提前结束剩余外层迭代——安静
        // 堆叠期（金字塔/砖墙的稳态段）12 次外层纯属浪费；判据只依赖状态，
        // 同状态必在同一迭代退出 ⇒ 确定性不受影响。
        let t_it = vxl_phys_core::probe::start();
        let eps = early_exit_eps();
        let min_iters = early_min_iters();
        let reduce_after = point_reduce_after();
        for it in 0..iters {
            let mut resid = 0.0f32;
            // **参与式降点**：从第 `reduce_after` 轮起，跳过"至今零冲量"的浅缝接触点
            // （判据见 `point_reduce_after`）。前几轮全点参与 ⇒ 需要承载的点已经
            // 载上，之后跳过它们只是省下空转。
            let reduce = reduce_after > 0 && it + 1 >= reduce_after;
            if it % 2 == 0 {
                for c in cbuf.iter_mut() {
                    resid = resid.max(solve_constraint(
                        c,
                        lv,
                        av,
                        local_of,
                        iw,
                        im,
                        false,
                        normal_inner,
                        reduce,
                    ));
                }
            } else {
                for c in cbuf.iter_mut().rev() {
                    resid = resid.max(solve_constraint(
                        c,
                        lv,
                        av,
                        local_of,
                        iw,
                        im,
                        true,
                        normal_inner,
                        reduce,
                    ));
                }
            }
            if it + 1 >= min_iters && resid < eps {
                break;
            }
        }
        // 堆叠 shock 附加迭代（M1 稳定性；Jolt shock propagation 同思路）：
        // 反序再过一遍约束，使「底层承载」的载荷沿约束图反向传播一次——
        // 深层堆叠的正向迭代需 ≈ 2×层数 次才能收敛，反序一遍等效多收敛若干层。
        // 确定性：反序为固定次序、纯数据驱动，与线程数无关。
        for _ in 0..shock_iterations {
            for c in cbuf.iter_mut().rev() {
                let _ = solve_constraint(c, lv, av, local_of, iw, im, true, normal_inner, false);
            }
        }
        // 收集 warm 更新（接触点锚点回推；位置在解算中不变）。
        detail[2] += vxl_phys_core::probe::us(t_it);
        for c in cbuf.iter() {
            // 诊断（零开销）：被解算的点数 / 其中法向冲量≈0 的点数（padding 槽
            // 不算——只数 `npts` 内的有效点，否则补零槽会被当成"零冲量点"）。
            detail[3] += c.npts as u64;
            detail[4] += c.points[..c.npts as usize]
                .iter()
                .filter(|p| p.pn <= 1e-6)
                .count() as u64;
            let mut wm = WarmManifold::EMPTY;
            wm.normal = c.normal;
            wm.n = c.npts;
            for (k, p) in c.points.iter().enumerate().take(4) {
                wm.points[k] = WarmPoint {
                    pn: p.pn,
                    pt1: p.pt1,
                    pt2: p.pt2,
                    feature: p.feature,
                    la: p.la,
                    lb: p.lb,
                    depth0: p.depth0,
                };
            }
            warm_out.push((c.warm_slot, (c.a, c.b, c.warm_space), wm));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{Quat, SerialJobSystem, Shape};
    use vxl_phys_narrow::{ContactPoint, Manifold};

    #[test]
    fn resting_box_velocities_damp_to_zero() {
        // 迷你闭环：每帧按当前位置重建流形（模拟窄相），验证顺序冲量把
        // 下落盒收敛到静置高度（y ≈ 1.0）且速度趋零、最终入睡。
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 2.5, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        let g = b.push_static(
            Shape::Box {
                half: Vec3::new(10.0, 0.5, 10.0),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let cfg = PhysConfig::default();
        let mut solver = ImpulseSolver::new(cfg.contact_skin);
        let dt = cfg.dt;
        // 简化重力（真实管线经力场→积分器；此处直接并入速度积分）。
        let mut y = 2.5f32;
        for _ in 0..300 {
            // 引擎契约：积分器只对清醒体施加重力（沉睡体由岛冻结）。
            if b.awake[0] {
                b.linvel[0] += Vec3::new(0.0, -9.81 * dt, 0.0);
            }
            let depth = 1.0 - y;
            if depth > -cfg.contact_skin {
                let m = Manifold {
                    a: 0,
                    b: g,
                    normal: Vec3::new(0.0, -1.0, 0.0),
                    points: [ContactPoint {
                        point: Vec3::new(0.0, y - 0.5, 0.0),
                        depth,
                        feature: 0,
                    }]
                    .into(),
                };
                solver.solve(&mut b, &[m], &cfg, dt, &SerialJobSystem);
            } else {
                solver.solve(&mut b, &[], &cfg, dt, &SerialJobSystem);
            }
            y += b.linvel[0].y * dt;
            b.position[0].y = y;
        }
        assert!((y - 1.0).abs() < 0.05, "rest y = {y}");
        assert!(b.linvel[0].y.abs() < 0.05, "linvel {:?}", b.linvel[0]);
    }
}
