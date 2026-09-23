//! island：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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
pub(crate) fn early_exit_eps() -> f32 {
    0.002
}

/// 早退前至少跑满的外层迭代数（堆叠建立期不早退；须为偶数以保持正/反扫掠对称）。
pub(crate) fn early_min_iters() -> u32 {
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
pub(crate) fn point_reduce_after() -> u32 {
    3
}

/// 顺序冲量解算一条约束；返回本次扫掠施加的**最大速度级修正**（m/s，
/// 法向 + 摩擦通道的最大值），供收敛早退判据使用。
#[allow(clippy::too_many_arguments)] // 热路径内联目标：避免打包结构体的构造成本
pub(crate) fn solve_constraint(
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
pub(crate) fn solve_island_group(
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
    settled_hold: u32,
    hold_max_vn: f32,
    detail: &mut [u64; 5],
) {
    for &ii in awake {
        let isl = &islands[ii];
        // 每岛构建约束（岛内流形序 = 全局流形序，§4.14）。
        let t_build = ISLAND_SEG_PROBE.then(vxl_phys_core::probe::start);
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
        if let Some(t) = t_build {
            detail[0] += vxl_phys_core::probe::us(t);
        }
        // warm starting 预施加（每约束一次）。
        let t_warm = ISLAND_SEG_PROBE.then(vxl_phys_core::probe::start);
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
        if let Some(t) = t_warm {
            detail[1] += vxl_phys_core::probe::us(t);
        }
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
        let t_it = ISLAND_SEG_PROBE.then(vxl_phys_core::probe::start);
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
        // —— 准静态"安座"趟（`settled_hold`）——
        // 把准静态接触（|vn| < hold_max_vn）的**法向**相对速度精确归零：逐点求有效质量
        // （与主迭代同一套组内缓存），施加 λ = −vn·m_eff（钳 pn+λ ≥ 0：只推不拉）。
        // 只动法向 ⇒ 不改变摩擦/滑动语义；大 vn（真撞击、真分离）一概不碰。
        // ⚠️ 两条纪律（第一版栽过）：
        // ① **不动 `p.pn`**：warm 缓存必须只装"求解器自己的冲量"，掺进外来量会污染
        //    下一子步的暖启动 ⇒ 实测爆掉（6152 快体、|v|max 81、深穿透 347 tick）；
        // ② **欠松弛**：流形 ≤4 点是**冗余**约束，逐点各自"精确归零"会叠加（×点数×2 趟
        //    ×子步数）⇒ 过冲。取 ω = 0.25（残差 0.16 → 一遍 0.12 → 四遍 ≈0.05）。
        const HOLD_OMEGA: f32 = 0.25;
        for _ in 0..settled_hold {
            for c in cbuf.iter() {
                let (ai, bi) = (c.a as usize, c.b as usize);
                let normal = c.normal;
                for p in c.points.iter().take(c.npts as usize) {
                    let ka = local_of[ai];
                    let kb = local_of[bi];
                    let im_a = if ka == u32::MAX { 0.0 } else { im[ka as usize] };
                    let im_b = if kb == u32::MAX { 0.0 } else { im[kb as usize] };
                    let mut k = im_a + im_b;
                    if im_a > 0.0 {
                        let w = iw[ka as usize].mul_vec3(p.ra.cross(normal));
                        k += w.cross(p.ra).dot(normal);
                    }
                    if im_b > 0.0 {
                        let w = iw[kb as usize].mul_vec3(p.rb.cross(normal));
                        k += w.cross(p.rb).dot(normal);
                    }
                    if k <= 1e-12 {
                        continue;
                    }
                    let m_eff = 1.0 / k;
                    let va = group_vel(lv, av, local_of, ai, p.ra);
                    let vb = group_vel(lv, av, local_of, bi, p.rb);
                    let vn = (vb - va).dot(normal);
                    if vn.abs() >= hold_max_vn {
                        continue; // 真撞击/真分离：不碰
                    }
                    // ⚠️ 目标 = **0**（不是 `rhs`）：投到 `rhs` 等于把"去穿透分离速度"加回去
                    // ⇒ 实测比不开还差（默认路径 awake 7491 → 9756）。
                    // 深穿透由**下面的深度守卫**处理：只在**浅接触**（`depth0 ≤ 0.05`）上安座，
                    // 深穿透接触一概不碰、照常让求解器把它们推出来（首版无守卫 ⇒ 实测
                    // 默认路径深穿透 5 tick / 0.295 m：把恢复速度一起冻掉了）。
                    if p.depth0 > 0.05 {
                        continue;
                    }
                    let dl = -vn * m_eff * HOLD_OMEGA;
                    if dl != 0.0 {
                        let imp = normal * dl;
                        group_apply(lv, av, local_of, iw, im, ai, p.ra, imp, true);
                        group_apply(lv, av, local_of, iw, im, bi, p.rb, imp, false);
                    }
                }
            }
        }

        // 收集 warm 更新（接触点锚点回推；位置在解算中不变）。
        if let Some(t) = t_it {
            detail[2] += vxl_phys_core::probe::us(t);
        }
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
