//! phase：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 默认窄相（SAT + 解析球 + 高度场采样）。工作缓冲全程复用（无逐步分配）。
#[derive(Clone)]
pub struct DefaultNarrowPhase {
    /// 凸体外壳仓库（多边形域；点云注册后由 shape 引用）。
    pub(crate) hulls: HullStore,
    /// 复合体仓库（子形状表；由 `Shape::Compound { compound, .. }` 引用）。
    pub(crate) compounds: CompoundStore,
    /// 子形状表 scratch（`kids_take`/`kids_put` 借出，避开 `&self`/`&mut self` 借用冲突）。
    pub(crate) kids_buf: Vec<CompoundChild>,
    pub(crate) skin: f32,
    /// **速度充气视野的预测时长**（s；0 = 不预测 ＝ 现行行为）。由 `World` 每子步设为
    /// **检测间隔**（每子步检测时为 `dt`、每 tick 检测时为整 tick）：窄相的接受判据
    /// 从 `sep ≤ skin` 放宽为 `sep ≤ skin + max(0, 接近速度)·predict_dt`，使
    /// "**下一次检测之前会碰上的接触**"提前成流形——求解器的 spec 项
    /// （`sep·inv_dt`）负责把逼近平滑拦停，不需要额外机制。
    ///
    /// 依据（`EXPERIMENTS.md` 末节 L/M）：检测每步一次时塔崩（KE 30 万）的真实机理是
    /// **逼近中的接触晚生**；把 skin 从 0.01 加到 0.02/0.04/0.08 即可让 KE 降到
    /// 2 386/1 992/1 271 且 y 带完整 ⇒ 视野不足而非语义不可行。
    /// `0` 时整条路径逐位不变（含三哈希）。
    pub(crate) predict_dt: f32,
    /// 本对的充气量（scratch）：由 SAT 预筛处按 (a, b, n) 计算，`clip` 的逐点过滤共用。
    pub(crate) inflate: f32,
    /// 接触点空间去重最小间距（m）：2×skin，且 ≥ 1 cm。
    pub(crate) min_point_sep: f32,
    pub(crate) polys: Vec<ConvexPolytope>,
    pub(crate) poly_index: HashMap<u64, usize>,
    pub(crate) poly_a: WorldPoly,
    pub(crate) poly_b: WorldPoly,
    /// 世界多面体填充缓存（T3 快路径）：pair 按 (a,b) 排序 ⇒ 同一体的对连续，
    /// 单条缓存即可让每体每帧只填一次（大场景实测同一体每帧被填 ~50 次）。
    /// 纯函数（poly, pos, rot）→ 命中即跳过 33 次旋转；逐位一致。
    pub(crate) cached_a: (u32, u64),
    pub(crate) cached_b: (u32, u64),
    pub(crate) axes: Vec<Vec3>,
    pub(crate) clip_in: Vec<(Vec3, u32)>,
    pub(crate) clip_out: Vec<(Vec3, u32)>,
    pub(crate) cand: Vec<ContactPoint>,
    /// select_contacts 去重取点 scratch（T3：免每接触一次堆分配）。
    pub(crate) kept_buf: Vec<ContactPoint>,
    /// 盒对 SAT 快路径参数（half, 世界中心）：两 poly 均为盒时用 extents
    /// 投影公式（O(1)/体/轴）替代逐顶点 min/max（24 点/体/轴）。由盒-盒
    /// 分支设置；圆柱等其它形状为 None（走通用逐顶点路径）。
    pub(crate) box_a: Option<(Vec3, Vec3)>,
    pub(crate) box_b: Option<(Vec3, Vec3)>,
    /// 盒对专属体轴（T3 专用路径）：由 `(rot, half)` 直生（每体 3 次旋转），
    /// 与预筛共用；置位时 SAT/clip 走免填充路径，其余形状为 None。
    pub(crate) box_axes_a: Option<[Vec3; 3]>,
    pub(crate) box_axes_b: Option<[Vec3; 3]>,
    /// 体轴缓存（T3）：键 = (体号, 姿态指纹)。盒对路径原先**每对**都重算两体的
    /// **外壳世界点缓存**（两侧各一份）：键 = (体号, 姿态指纹)，与 `cached_ax_*`
    /// 同机制 ⇒ 同体同帧多对时只做一次 O(n) 世界变换（顶点采样/EPA 细化共用）。
    pub(crate) hull_pts: [Vec<Vec3>; 2],
    pub(crate) cached_hull: [(u32, u64); 2],
    /// 3 次旋转（同体在 (a,b) 序下连续出现，与 `cached_a/cached_b` 同机制）
    /// ⇒ 纯函数、命中即同值 ⇒ **逐位透明**。两槽分别给对的两个侧别。
    pub(crate) cached_ax_a: (u32, u64, [Vec3; 3]),
    pub(crate) cached_ax_b: (u32, u64, [Vec3; 3]),
    /// 裁剪顶点 scratch（参考面顶点）：通用与专用路径共用同一裁剪实现，
    /// 避免两份易漂移的裁剪代码。
    pub(crate) ref_v: Vec<Vec3>,
    /// 上一帧产出的流形数（下一帧并行块输出缓冲的容量提示）。纯性能提示：
    /// 不参与任何判定，故与确定性无关。
    pub(crate) out_hint: usize,
    /// 诊断计数器（**零开销**，仅整数自增）：裁剪调用数 / 内层顶点迭代总数 /
    /// 交点插值总数 / 进 `select_contacts` 前的候选点数。用途：用"每次调用的
    /// 迭代数"反推成本落在**固定开销**（面选择 + 装配 + 归约）还是**内层行走**
    /// ——计时探针在 1361 次/步下自身就要 ~200 µs/步，会淹没被测段。
    pub probe_clip_calls: u64,
    pub probe_clip_iters: u64,
    pub probe_clip_xings: u64,
    pub probe_cand_pts: u64,
    /// 观测到的**最大裁剪多边形长度**（`clip_in` 峰值）。用途：为"把
    /// `clip_in`/`clip_out` 换成定长数组"提供**实测上界**——盒对快路径理论上界
    /// 是 8（4 顶点入射面 + 4 次半平面裁剪，每次凸多边形裁剪 ≤ m+1），但通用
    /// 路径的面顶点数无小常数上界（圆柱面 = n 边形、凸包面可达数十顶点），
    /// 只有实测峰值才能支撑"定长 16 是否安全"的判断。
    pub probe_clip_max: u64,
}

impl DefaultNarrowPhase {
    /// 诊断读数：(裁剪调用数, 内层迭代总数, 插值总数, 候选点总数, 裁剪多边形峰值)。
    pub fn probe_stats(&self) -> (u64, u64, u64, u64, u64) {
        (
            self.probe_clip_calls,
            self.probe_clip_iters,
            self.probe_clip_xings,
            self.probe_cand_pts,
            self.probe_clip_max,
        )
    }
}
