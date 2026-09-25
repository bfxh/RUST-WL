//! constraint：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;
use vxl_phys_narrow::ContactPoint;

/// 材质对的有效摩擦/恢复：摩擦按材质对组合（§4.4/§4.5）；静+动档 μ 随预解
/// 相对切向速度在 [μk, μs] 线化过渡（vt 取首个接触点，确定性）。
fn material_pair(bodies: &BodySet, m: &Manifold, a: usize, b: usize) -> (f32, f32) {
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
}

/// warm starting 匹配：① 特征 ID 精确匹配（带距离护栏，锚点当前世界位置量距）；
/// ② 近邻回退（无特征或 ID 未命中）。命中/回退/未命中都进诊断计数。
fn match_warm_point(
    cp: &ContactPoint,
    m: &Manifold,
    warmm: Option<&WarmManifold>,
    warm_world: &[Vec3; 4],
    warm_n: usize,
    match_dist: f32,
) -> Option<WarmPoint> {
    let mut warm_pt: Option<WarmPoint> = None;
    if let Some(wm) = warmm {
        if warm_n > 0 && wm.normal.dot(m.normal) > 0.95 {
            if cp.feature != 0 {
                warm_pt = (0..warm_n)
                    .find(|&k| {
                        wm.points[k].feature == cp.feature
                            && (warm_world[k] - cp.point).length_squared() < match_dist * match_dist
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
    warm_pt
}

/// 复用判定后的有效几何：世界点、有效深度、烘焙深度、双局部锚点、分离向量。
struct AnchorGeom {
    pt_world: Vec3,
    depth: f32,
    depth0: f32,
    la: Vec3,
    lb: Vec3,
    drift: Vec3,
}

/// 复用判定 + 有效几何：锚点世界分离 ≤ 回收半径 → 沿用锚点（深度/漂移
/// 由锚点分离量推进）；否则按本帧裁剪点烘焙新锚点（la/lb 同源 ⇒ 初始分离 0）。
#[allow(clippy::too_many_arguments)]
fn resolve_anchor(
    cp: &ContactPoint,
    warm_pt: Option<WarmPoint>,
    pos_a: Vec3,
    pos_b: Vec3,
    rot_a: &vxl_phys_core::Mat3,
    rot_b: &vxl_phys_core::Mat3,
    normal: Vec3,
) -> AnchorGeom {
    let bake = || {
        (
            rot_a.transpose_mul_vec3(cp.point - pos_a),
            rot_b.transpose_mul_vec3(cp.point - pos_b),
        )
    };
    match warm_pt {
        Some(w) => {
            let pa = pos_a + rot_a.mul_vec3(w.la);
            let pb = pos_b + rot_b.mul_vec3(w.lb);
            let sep_v = pa - pb;
            if sep_v.length_squared() <= RECYCLE_DIST * RECYCLE_DIST {
                // 有效深度 = 烘焙深度 + 当前累计分离沿法向的投影（非增量式
                // 复合——烘焙深度全程不变，见 WarmPoint::depth0）。
                AnchorGeom {
                    pt_world: (pa + pb) * 0.5,
                    depth: w.depth0 + sep_v.dot(normal),
                    depth0: w.depth0,
                    la: w.la,
                    lb: w.lb,
                    drift: sep_v,
                }
            } else {
                let (la, lb) = bake();
                AnchorGeom {
                    pt_world: cp.point,
                    depth: cp.depth,
                    depth0: cp.depth,
                    la,
                    lb,
                    drift: Vec3::ZERO,
                }
            }
        }
        None => {
            let (la, lb) = bake();
            AnchorGeom {
                pt_world: cp.point,
                depth: cp.depth,
                depth0: cp.depth,
                la,
                lb,
                drift: Vec3::ZERO,
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn contact_masses(
    bodies: &BodySet,
    a: usize,
    b: usize,
    ra: Vec3,
    rb: Vec3,
    normal: Vec3,
    t1: Vec3,
    t2: Vec3,
) -> (f32, f32, f32, f32) {
    let nmass = contact_mass(
        bodies.inv_mass[a],
        bodies.inv_mass[b],
        ra,
        rb,
        normal,
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
    (nmass, tmass1, tmass2, tcross)
}

/// —— M1 软接触目标（TGS-Soft 语义，Rapier 0.35 同构；深度为回收后有效值）——
/// 返回 `(rhs, cfm)`：弹性 + speculative + 去穿透 bias 合成右端项与正则化系数。
/// 弹性取预解相对法向速度；speculative 让浅缝（depth<0）一个 tick 内闭合剩余间隙
/// （不再提前悬停）；去穿透用 `erp·(depth−slop)` 并钳 `max_corrective_velocity`。
fn soft_contact(
    depth: f32,
    vn: f32,
    e: f32,
    e_threshold: f32,
    sp: &SolverParams,
    is_static_pair: bool,
) -> (f32, f32) {
    let bounce = if vn < -e_threshold { -e * vn } else { 0.0 };
    let sep = -depth;
    let spec = sep.max(0.0) * sp.inv_dt;
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
    (rhs, cfm)
}

/// 切向锚点漂移回拉（Rapier 切向 rhs 同义）：目标 `v_t = (pa−pb)·t·inv_dt`，
/// 由摩擦在锥内执行——粘着接触的材料点错位被回正（回收锚点 ⇒ 漂移为
/// 真实滑移量，非裁剪几何噪声；旧 shortcut 无锚点实测变差已回退）。
/// **切向回拉只在"非滑动"接触上做**（见 `DRIFT_STICK_RATIO` 注）：滑动接触的锚点
/// 分离是**真实材料滑移**，回拉等于抹掉真实滑动、并 ∝ μ 地注转矩（角向残差主项）；
/// 粘着接触的分离才是数值错位。判定用上一子步的暖启动冲量（零额外状态）。
fn tangential_drift(
    warm_pt: Option<WarmPoint>,
    mu: f32,
    drift: Vec3,
    sp: &SolverParams,
    t1: Vec3,
    t2: Vec3,
) -> (f32, f32) {
    let sliding = warm_pt.is_none_or(|w| {
        (w.pt1 * w.pt1 + w.pt2 * w.pt2).sqrt() >= DRIFT_STICK_RATIO * mu * w.pn.max(0.0)
    });
    if !sliding && drift.length_squared() > DRIFT_DEADZONE * DRIFT_DEADZONE {
        let eff = drift * (DRIFT_BIAS_SCALE * sp.inv_dt);
        (eff.dot(t1), eff.dot(t2))
    } else {
        (0.0, 0.0)
    }
}

/// 构建单个流形的接触约束（预计算质量项/bias/warm）。摩擦/恢复按材质对
/// 组合（§4.4/§4.5）；静+动档 μ 随预解相对切向速度在 [μk, μs] 线化过渡
/// （vt 取首个接触点，确定性）。切向基提升到流形级（原为每点重算）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_constraint(
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
    if !bodies.is_dynamic(a) && !bodies.is_dynamic(b) {
        return;
    }
    let (mu, e) = material_pair(bodies, m, a, b);

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
        let warm_pt = match_warm_point(cp, m, warmm, &warm_world, warm_n, match_dist);
        let g = resolve_anchor(cp, warm_pt, pos_a, pos_b, &rot_a, &rot_b, m.normal);
        let ra = g.pt_world - pos_a;
        let rb = g.pt_world - pos_b;
        let (nmass, tmass1, tmass2, tcross) =
            contact_masses(bodies, a, b, ra, rb, m.normal, t1, t2);

        let va = bodies.velocity_at(a, ra);
        let vb = bodies.velocity_at(b, rb);
        let vn = (vb - va).dot(m.normal);
        let is_static_pair = bodies.inv_mass[a] == 0.0 || bodies.inv_mass[b] == 0.0;
        let (rhs, cfm) = soft_contact(g.depth, vn, e, e_threshold, sp, is_static_pair);
        let (trhs1, trhs2) = tangential_drift(warm_pt, mu, g.drift, sp, t1, t2);

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
            la: g.la,
            lb: g.lb,
            depth0: g.depth0,
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
