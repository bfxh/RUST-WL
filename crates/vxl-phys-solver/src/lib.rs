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

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, PhysConfig};
use vxl_phys_narrow::Manifold;

/// 单接触点的已求解冲量缓存（warm starting + 接触回收）。
#[derive(Clone, Copy, Debug)]
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

/// 接触回收半径（m；超过则锚点重烘焙。Rapier `normalized_contact_recycle_distance` 0.05）。
const RECYCLE_DIST: f32 = 0.05;

/// 切向锚点漂移回拉的强度标度（1 = Rapier 原式；0 = 关闭回拉只保留回收）。
/// 消融中发现满强度会在 5 层堆留下 ~0.09 m/s 永久微抖（不入睡），
/// 半强度为实测折中（塔蠕动停止增长 + 堆可入睡）。
/// 切向锚点漂移回拉的强度标度（1 = Rapier 原式；0 = 关闭回拉只保留回收）。
/// **默认 0（关闭）**：消融实测（M1 第十段）——满强度可让塔蠕动停止增长
/// （0.34→0.26 衰减）但代价明显：5 层堆留 ~0.09 m/s 永久微抖（不入睡）、
/// 门槛场景末态 38 体不睡且 p50 0.48→8.4ms。回收本身（scale=0）各场景行为
/// 保持（门槛干净入睡/0.88ms、5 层堆全睡），故先落地回收、回拉留待精调。
const DRIFT_BIAS_SCALE: f32 = 0.0;

/// 切向漂移回拉的死区（m）：|漂移| ≤ 死区时不回拉——只纠正真实材料滑移，
/// 不响应微倾/裁剪抖动引起的毫米级锚点错位。**默认 0（关闭）**：消融实测
/// 1mm 死区利于 5 层堆（0.09→0.05）但让塔蠕动从「衰减」退回「微增」
/// （0.34→0.26 变 0.22→0.29）——M1 验收主体是塔，取无死区。
const DRIFT_DEADZONE: f32 = 0.0;

#[derive(Clone, Debug)]
struct WarmManifold {
    normal: Vec3,
    points: Vec<WarmPoint>,
}

use vxl_phys_core::Vec3;

/// 单接触点约束（预计算质量项与软接触目标/正则化）。
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
    points: Vec<PointConstraint>,
}

/// 顺序冲量求解器。
#[derive(Default)]
pub struct ImpulseSolver {
    warm_cache: HashMap<(u32, u32), WarmManifold>,
    /// 上一帧岛（诊断/调试用）。
    pub island_count: usize,
    /// warm starting 点匹配距离（= 4×skin，构造时可调）。
    pub match_dist: f32,
    /// 诊断：本帧计时器被清零的体数。
    pub sleep_resets: u32,
    /// 并行分组复用缓冲（§6）：组 → 约束构建 / warm 更新 / 速度 scratch。
    build_bufs: Vec<Vec<ContactConstraint>>,
    warm_outs: Vec<Vec<((u32, u32), WarmManifold)>>,
    group_lv: Vec<Vec<Vec3>>,
    group_av: Vec<Vec<Vec3>>,
    /// 体 → 组内局部索引（u32::MAX = 静态/不在清醒岛）。
    local_of: Vec<u32>,
    /// 诊断：上一帧内部阶段耗时（µs）=(岛构建, 约束构建+迭代, 休眠/其他, 保留)。
    pub last_phase_us: (u64, u64, u64, u64),
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
    i: usize,
    ra: Vec3,
    imp: Vec3,
    minus: bool,
    bodies: &BodySet,
) {
    let k = local_of[i];
    if k == u32::MAX {
        return;
    }
    let q = k as usize;
    let im = bodies.inv_mass[i];
    if minus {
        lv[q] -= imp * im;
        av[q] -= bodies.apply_world_inv_inertia(i, ra.cross(imp));
    } else {
        lv[q] += imp * im;
        av[q] += bodies.apply_world_inv_inertia(i, ra.cross(imp));
    }
}

impl ImpulseSolver {
    pub fn new(skin: f32) -> Self {
        Self {
            warm_cache: HashMap::new(),
            island_count: 0,
            match_dist: (skin * 4.0).max(0.02),
            sleep_resets: 0,
            build_bufs: Vec::new(),
            warm_outs: Vec::new(),
            group_lv: Vec::new(),
            group_av: Vec::new(),
            local_of: Vec::new(),
            last_phase_us: (0, 0, 0, 0),
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
        let e_threshold = config.restitution_threshold;
        let iters = config.velocity_iterations.max(1);
        let shock = config.shock_iterations;
        let normal_inner = config.normal_inner.max(1);
        let sp = SolverParams::from_config(config, dt);
        let threads = jobs.threads().max(1);
        let match_dist = self.match_dist;
        let t_island = std::time::Instant::now();

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

        let mut warm = std::mem::take(&mut self.warm_cache);
        let mut build_bufs = std::mem::take(&mut self.build_bufs);
        let mut warm_outs = std::mem::take(&mut self.warm_outs);
        let mut group_lv = std::mem::take(&mut self.group_lv);
        let mut group_av = std::mem::take(&mut self.group_av);
        let mut local_of = std::mem::take(&mut self.local_of);

        build_bufs.resize_with(g_count, Vec::new);
        warm_outs.resize_with(g_count, Vec::new);
        group_lv.resize_with(g_count, Vec::new);
        group_av.resize_with(g_count, Vec::new);
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
            let s0 = g * awake.len() / g_count;
            let e0 = (g + 1) * awake.len() / g_count;
            groups.push((s0, e0));
            for &ii in &awake[s0..e0] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    local_of[i] = group_lv[g].len() as u32;
                    group_lv[g].push(bodies.linvel[i]);
                    group_av[g].push(bodies.angvel(i));
                }
            }
        }

        let d_island = t_island.elapsed().as_micros() as u64;
        let t_solve = std::time::Instant::now();
        // 4) 并行解算（§6 契约：组间写槽位不相交，组内 = 串行语义）。
        if g_count > 1 {
            let bodies_ref: &BodySet = bodies;
            let awake_ref: &[usize] = &awake;
            let islands_ref: &[Island] = islands;
            let warm_ref: &HashMap<(u32, u32), WarmManifold> = &warm;
            let local_ref: &[u32] = &local_of;
            let sp_ref: &SolverParams = &sp;
            std::thread::scope(|s| {
                // iter_mut 逐容器取出元素可变借用（按 g 索引整体借用会跨迭代重叠）。
                for (g, ((lv, av), (cbuf, wout))) in group_lv
                    .iter_mut()
                    .zip(group_av.iter_mut())
                    .zip(build_bufs.iter_mut().zip(warm_outs.iter_mut()))
                    .enumerate()
                {
                    let (s0, e0) = groups[g];
                    let job = move || {
                        solve_island_group(
                            &awake_ref[s0..e0],
                            islands_ref,
                            manifolds,
                            bodies_ref,
                            warm_ref,
                            local_ref,
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
        } else if !awake.is_empty() {
            solve_island_group(
                &awake,
                islands,
                manifolds,
                bodies,
                &warm,
                &local_of,
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
            );
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

        // warm 合并（组序 = 岛序；键唯一）+ 剪枝失效键。
        // 剪枝规则：流形已消失的**双清醒**对才删——睡眠体不移动，其接触
        // 不会真正消失（睡眠期不被检测只是省算力），若一并剪掉，唤醒后
        // warm 起点归零会导致数帧收敛变弱（穿透加深）。睡眠体的条目在
        // 醒来且流形真正消失时自然被清。
        for wo in warm_outs.drain(..) {
            for (k, v) in wo {
                warm.insert(k, v);
            }
        }
        if manifolds.is_empty() {
            warm.clear();
        } else {
            let mut cur: Vec<(u32, u32)> = manifolds.iter().map(|m| (m.a, m.b)).collect();
            cur.sort_unstable();
            cur.dedup();
            warm.retain(|&(a, b), _| {
                cur.binary_search(&(a, b)).is_ok()
                    || !(bodies.awake[a as usize] && bodies.awake[b as usize])
            });
        }
        self.warm_cache = warm;
        self.build_bufs = build_bufs;
        self.warm_outs = warm_outs;
        self.group_lv = group_lv;
        self.group_av = group_av;
        self.local_of = local_of;

        let d_solve = t_solve.elapsed().as_micros() as u64;
        let t_sleep = std::time::Instant::now();
        // 5) 岛级休眠与唤醒（§4.11 / §3 稳定性）。
        //    - 建岛阶段已只收「与清醒体连通」的岛（含被牵连的睡眠体），
        //      遗漏的睡眠体天然保持冻结（不解算不积分）；
        //    - 清醒岛：任一成员 awake → 全岛同步为 awake（外部唤醒传播）；
        //    - 全员速度低于阈值持续 sleep_time → 岛内**原子**入睡（同帧全员睡），
        //      不存在"部分睡部分醒"状态，从机制上排除反复唤醒；
        //    - 无接触的孤立清醒动体 = 单成员岛，走同一套休眠判定。
        for island in islands {
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
        self.last_phase_us = (d_island, d_solve, t_sleep.elapsed().as_micros() as u64, 0);
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
    warm: &HashMap<(u32, u32), WarmManifold>,
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

    let warmm = warm.get(&(m.a, m.b));
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
        for w in wm.points.iter().take(4) {
            warm_world[warm_n] =
                (pos_a + rot_a.mul_vec3(w.la) + pos_b + rot_b.mul_vec3(w.lb)) * 0.5;
            warm_n += 1;
        }
    }
    let mut pts: Vec<PointConstraint> = Vec::with_capacity(m.points.len());
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
                }
                if warm_pt.is_none() {
                    warm_pt = (0..warm_n)
                        .filter(|&k| {
                            (warm_world[k] - cp.point).length_squared() < match_dist * match_dist
                        })
                        .min_by(|&x, &y| {
                            (warm_world[x] - cp.point)
                                .length_squared()
                                .total_cmp(&(warm_world[y] - cp.point).length_squared())
                        })
                        .map(|k| wm.points[k]);
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
        let drift_eff = if drift.length_squared() > DRIFT_DEADZONE * DRIFT_DEADZONE {
            drift * (DRIFT_BIAS_SCALE * sp.inv_dt)
        } else {
            Vec3::ZERO
        };
        let trhs1 = drift_eff.dot(t1);
        let trhs2 = drift_eff.dot(t2);

        pts.push(PointConstraint {
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
        });
    }
    if pts.is_empty() {
        return;
    }
    out.push(ContactConstraint {
        a: m.a,
        b: m.b,
        normal: m.normal,
        points: pts,
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
fn solve_constraint(
    c: &mut ContactConstraint,
    lv: &mut [Vec3],
    av: &mut [Vec3],
    local_of: &[u32],
    bodies: &BodySet,
    rev: bool,
    inner: u32,
) {
    let (ai, bi) = (c.a as usize, c.b as usize);
    let normal = c.normal;
    let npts = c.points.len();

    // —— 歧管内层扫掠（法向 + 摩擦两通道一起）——
    for _ in 0..inner.max(1) {
        for k in 0..npts {
            let idx = if rev { npts - 1 - k } else { k };
            let p = &mut c.points[idx];
            // —— 法向（M1 软接触）：λ ← cfm·(λ + m·(rhs − vn))，钳 ≥ 0 ——
            let va = group_vel(lv, av, local_of, ai, p.ra);
            let vb = group_vel(lv, av, local_of, bi, p.rb);
            let vn = (vb - va).dot(normal);
            let new_pn = (p.cfm * (p.pn + p.nmass * (p.rhs - vn))).max(0.0);
            let dl = new_pn - p.pn;
            p.pn = new_pn;
            if dl != 0.0 {
                let imp = normal * dl;
                group_apply(lv, av, local_of, ai, p.ra, imp, true, bodies);
                group_apply(lv, av, local_of, bi, p.rb, imp, false, bodies);
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
                        let imp = p.t1 * d1 + p.t2 * d2;
                        group_apply(lv, av, local_of, ai, p.ra, imp, true, bodies);
                        group_apply(lv, av, local_of, bi, p.rb, imp, false, bodies);
                    }
                }
            }
        }
    }
}

/// 解算一组清醒岛（组内岛串行；岛间体集合不相交）。速度读写走组内 scratch
/// （gather 已填充），warm 更新收集到 `warm_out`（调用方按组序合并）。
#[allow(clippy::too_many_arguments)]
fn solve_island_group(
    awake: &[usize],
    islands: &[Island],
    manifolds: &[Manifold],
    bodies: &BodySet,
    warm: &HashMap<(u32, u32), WarmManifold>,
    local_of: &[u32],
    lv: &mut [Vec3],
    av: &mut [Vec3],
    cbuf: &mut Vec<ContactConstraint>,
    warm_out: &mut Vec<((u32, u32), WarmManifold)>,
    iters: u32,
    e_threshold: f32,
    match_dist: f32,
    shock_iterations: u32,
    normal_inner: u32,
    sp: &SolverParams,
) {
    for &ii in awake {
        let isl = &islands[ii];
        // 每岛构建约束（岛内流形序 = 全局流形序，§4.14）。
        cbuf.clear();
        for &mi in &isl.manifs {
            build_constraint(
                cbuf,
                &manifolds[mi],
                bodies,
                warm,
                match_dist,
                e_threshold,
                sp,
            );
        }
        // warm starting 预施加（每约束一次）。
        for c in cbuf.iter() {
            let (ai, bi) = (c.a as usize, c.b as usize);
            for p in &c.points {
                if let Some(w) = p.warm {
                    if w.pn == 0.0 && w.pt1 == 0.0 && w.pt2 == 0.0 {
                        continue;
                    }
                    let impulse = c.normal * w.pn + p.t1 * w.pt1 + p.t2 * w.pt2;
                    group_apply(lv, av, local_of, ai, p.ra, impulse, true, bodies);
                    group_apply(lv, av, local_of, bi, p.rb, impulse, false, bodies);
                }
            }
        }
        // 顺序冲量迭代（岛内顺序 = 流形序 = 约束构建序，§4.14）。
        // 对称扫掠：偶数迭代正序、奇数迭代反序（约束序 + 接触点序同时反转）——
        // 4 点面接触是冗余约束 + 强转动耦合，单向 GS 16 次迭代后残留可观
        // （重堆实测微抖永不入睡）；正/反交替后收敛率 ≈ ρ²。
        // 确定性：方向仅由迭代序决定。
        for it in 0..iters {
            if it % 2 == 0 {
                for c in cbuf.iter_mut() {
                    solve_constraint(c, lv, av, local_of, bodies, false, normal_inner);
                }
            } else {
                for c in cbuf.iter_mut().rev() {
                    solve_constraint(c, lv, av, local_of, bodies, true, normal_inner);
                }
            }
        }
        // 堆叠 shock 附加迭代（M1 稳定性；Jolt shock propagation 同思路）：
        // 反序再过一遍约束，使「底层承载」的载荷沿约束图反向传播一次——
        // 深层堆叠的正向迭代需 ≈ 2×层数 次才能收敛，反序一遍等效多收敛若干层。
        // 确定性：反序为固定次序、纯数据驱动，与线程数无关。
        for _ in 0..shock_iterations {
            for c in cbuf.iter_mut().rev() {
                solve_constraint(c, lv, av, local_of, bodies, true, normal_inner);
            }
        }
        // 收集 warm 更新（接触点锚点回推；位置在解算中不变）。
        for c in cbuf.iter() {
            let pts = c
                .points
                .iter()
                .map(|p| WarmPoint {
                    pn: p.pn,
                    pt1: p.pt1,
                    pt2: p.pt2,
                    feature: p.feature,
                    la: p.la,
                    lb: p.lb,
                    depth0: p.depth0,
                })
                .collect();
            warm_out.push((
                (c.a, c.b),
                WarmManifold {
                    normal: c.normal,
                    points: pts,
                },
            ));
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
                    points: vec![ContactPoint {
                        point: Vec3::new(0.0, y - 0.5, 0.0),
                        depth,
                        feature: 0,
                    }],
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
