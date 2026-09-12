//! # vxl-phys-solver
//!
//! 刚体约束求解（§2.5/§2.6）：
//! - M0：顺序冲量（warm starting + Baumgarte 位置修正 + 摩擦锥）——即 M0 里程碑
//!   明确要求的「顺序冲量」；TGS-Soft（time-of-impact 分级 + 软约束）在 M1 升级，
//!   本文件求解循环骨架与岛/休眠/缓存层保持不变。
//! - 岛：并查集分岛；岛内约束按 (a, b, 点序) 固定排序（§4.14 确定性模式）。
//! - 休眠（§4.11）：线性 0.04 / 角速 0.05 rad/s、计时 0.5 s，岛级判定。
//! - CCD（§4.12/§2.7）：接口预留，M1 接入扫掠式。

#![forbid(unsafe_code)]

use std::collections::HashMap;

use vxl_phys_core::{BodySet, PhysConfig};
use vxl_phys_narrow::Manifold;

/// 单接触点的已求解冲量缓存（warm starting）。
#[derive(Clone, Copy, Debug)]
struct WarmPoint {
    point: Vec3,
    pn: f32,
    pt1: f32,
    pt2: f32,
}

#[derive(Clone, Debug)]
struct WarmManifold {
    normal: Vec3,
    points: Vec<WarmPoint>,
}

use vxl_phys_core::Vec3;

/// 单接触点约束（预计算质量项与 bias）。
struct PointConstraint {
    ra: Vec3,
    rb: Vec3,
    t1: Vec3,
    t2: Vec3,
    nmass: f32,
    tmass1: f32,
    tmass2: f32,
    /// 速度目标（Baumgarte + 弹性）：约束要求新 vn ≥ bias。
    bias: f32,
    friction: f32,
    pn: f32,
    pt1: f32,
    pt2: f32,
    warm: Option<WarmPoint>,
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
}

struct Island {
    bodies: Vec<u32>,
    contacts: Vec<usize>,
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

fn tangents(n: Vec3) -> (Vec3, Vec3) {
    let refr = if n.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let t1 = n.cross(refr).normalize();
    let t2 = n.cross(t1);
    (t1, t2)
}

impl ImpulseSolver {
    pub fn new(skin: f32) -> Self {
        Self {
            warm_cache: HashMap::new(),
            island_count: 0,
            match_dist: (skin * 4.0).max(0.02),
            sleep_resets: 0,
        }
    }

    /// 求解 + 岛级休眠。调用方顺序：积分速度 → 检测 → 本函数 → 积分位置。
    /// 唤醒语义：岛整体睡/醒（Box2D 同族）——岛内任一成员被外部唤醒（用户冲量、
    /// 新接触带入的动体）即全岛唤醒；杜绝"醒体反复唤醒睡体"。
    pub fn solve(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
    ) {
        let mu = config.friction.effective_mu();
        let e = config.restitution;
        let e_threshold = config.restitution_threshold;
        let bias_rate = config.baumgarte / dt;
        let slop = config.linear_slop;

        // 1) 构建约束（预计算质量项/bias/warm）。含沉睡体（休眠岛整体跳过解算）。
        let mut constraints: Vec<ContactConstraint> = Vec::with_capacity(manifolds.len());
        for m in manifolds {
            let (a, b) = (m.a as usize, m.b as usize);
            let da = bodies.is_dynamic(a);
            let db = bodies.is_dynamic(b);
            if !da && !db {
                continue;
            }

            let warm = self.warm_cache.get(&(m.a, m.b));
            let mut pts: Vec<PointConstraint> = Vec::with_capacity(m.points.len());
            for cp in &m.points {
                let ra = cp.point - bodies.position[a];
                let rb = cp.point - bodies.position[b];
                let (t1, t2) = tangents(m.normal);
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

                // 弹性：预解相对法向速度。
                let va = bodies.velocity_at(a, ra);
                let vb = bodies.velocity_at(b, rb);
                let vn = (vb - va).dot(m.normal);
                let bounce = if vn < -e_threshold { -e * vn } else { 0.0 };
                // 位置修正（Baumgarte，上限 2 m/s 防能量泵）。与弹性目标取大者
                // （单通道修正）：若相加，bg ≈ 0.2·v 恰好抵消 (1-e) 损耗 → 永动弹跳。
                let bg = (bias_rate * (cp.depth - slop).max(0.0)).min(2.0);
                let bias = bounce.max(bg);

                // warm starting 匹配。
                let mut warm_pt: Option<WarmPoint> = None;
                if let Some(wm) = warm {
                    if wm.normal.dot(m.normal) > 0.95 {
                        if let Some(wp) = wm
                            .points
                            .iter()
                            .filter(|wp| {
                                (wp.point - cp.point).length_squared()
                                    < self.match_dist * self.match_dist
                            })
                            .min_by(|x, y| {
                                let dx = (x.point - cp.point).length_squared();
                                let dy = (y.point - cp.point).length_squared();
                                dx.total_cmp(&dy)
                            })
                        {
                            warm_pt = Some(*wp);
                        }
                    }
                }

                pts.push(PointConstraint {
                    ra,
                    rb,
                    t1,
                    t2,
                    nmass,
                    tmass1,
                    tmass2,
                    bias,
                    friction: mu,
                    pn: warm_pt.map(|w| w.pn).unwrap_or(0.0),
                    pt1: warm_pt.map(|w| w.pt1).unwrap_or(0.0),
                    pt2: warm_pt.map(|w| w.pt2).unwrap_or(0.0),
                    warm: warm_pt,
                });
            }
            if pts.is_empty() {
                continue;
            }
            constraints.push(ContactConstraint {
                a: m.a,
                b: m.b,
                normal: m.normal,
                points: pts,
            });
        }

        // 3) 并查集分岛（仅动体互联；静态为锚）。
        let n = bodies.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(parent: &mut [u32], x: u32) -> u32 {
            let mut x = x;
            while parent[x as usize] != x {
                let g = parent[x as usize];
                parent[x as usize] = parent[g as usize];
                x = parent[x as usize];
            }
            x
        }
        fn union(parent: &mut [u32], x: u32, y: u32) {
            let rx = find(parent, x);
            let ry = find(parent, y);
            if rx != ry {
                // 固定规则：小索引为根（确定性）。
                if rx < ry {
                    parent[ry as usize] = rx;
                } else {
                    parent[rx as usize] = ry;
                }
            }
        }
        for c in &constraints {
            let (a, b) = (c.a as usize, c.b as usize);
            if bodies.is_dynamic(a) && bodies.is_dynamic(b) {
                union(&mut parent, c.a, c.b);
            }
        }

        // 岛桶：root → (bodies, contacts)。按体索引升序首次出现顺序建岛（确定性）。
        let mut root_order: Vec<u32> = Vec::new();
        let mut islands: Vec<Island> = Vec::new();
        let mut root_slot: HashMap<u32, usize> = HashMap::new();
        let mut in_island = vec![false; n];
        for (i, used) in in_island.iter_mut().enumerate() {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let r = find(&mut parent, i as u32);
            let slot = *root_slot.entry(r).or_insert_with(|| {
                root_order.push(r);
                islands.push(Island {
                    bodies: Vec::new(),
                    contacts: Vec::new(),
                });
                islands.len() - 1
            });
            islands[slot].bodies.push(i as u32);
            *used = true;
        }
        for (ci, c) in constraints.iter().enumerate() {
            let a = c.a as usize;
            let root = if bodies.is_dynamic(a) {
                find(&mut parent, c.a)
            } else {
                find(&mut parent, c.b)
            };
            if let Some(&slot) = root_slot.get(&root) {
                islands[slot].contacts.push(ci);
            }
        }
        self.island_count = islands.len();

        // 4) 逐岛求解：休眠岛整体跳过（**不得**施加 warm 冲量，否则沉睡体会被
        //    缓存的支撑冲量逐帧加速发射）；清醒岛先施加 warm，再迭代求解
        //    （岛内顺序 = 接触构建顺序 = 流形字典序，§4.14）。
        let iters = config.velocity_iterations.max(1);
        for island in &islands {
            if island.bodies.iter().all(|&bi| !bodies.awake[bi as usize]) {
                continue;
            }
            // warm starting 预施加（每约束一次）。
            for &ci in &island.contacts {
                let c = &constraints[ci];
                let (ai, bi) = (c.a as usize, c.b as usize);
                for p in &c.points {
                    if let Some(w) = p.warm {
                        if w.pn == 0.0 && w.pt1 == 0.0 && w.pt2 == 0.0 {
                            continue;
                        }
                        let impulse = c.normal * w.pn + p.t1 * w.pt1 + p.t2 * w.pt2;
                        bodies.linvel[ai] = bodies.linvel[ai] - impulse * bodies.inv_mass[ai];
                        bodies.angvel[ai] = bodies.angvel[ai]
                            - bodies.apply_world_inv_inertia(ai, p.ra.cross(impulse));
                        bodies.linvel[bi] = bodies.linvel[bi] + impulse * bodies.inv_mass[bi];
                        bodies.angvel[bi] = bodies.angvel[bi]
                            + bodies.apply_world_inv_inertia(bi, p.rb.cross(impulse));
                    }
                }
            }
            for _ in 0..iters {
                for &ci in &island.contacts {
                    let c = &mut constraints[ci];
                    let (ai, bi) = (c.a as usize, c.b as usize);
                    let normal = c.normal;
                    let ima = bodies.inv_mass[ai];
                    let imb = bodies.inv_mass[bi];
                    for p in c.points.iter_mut() {
                        // —— 法向 ——
                        let va = bodies.velocity_at(ai, p.ra);
                        let vb = bodies.velocity_at(bi, p.rb);
                        let vn = (vb - va).dot(normal);
                        let lambda = p.nmass * (p.bias - vn);
                        let new_pn = (p.pn + lambda).max(0.0);
                        let dl = new_pn - p.pn;
                        p.pn = new_pn;
                        if dl != 0.0 {
                            let imp = normal * dl;
                            bodies.linvel[ai] -= imp * ima;
                            bodies.angvel[ai] = bodies.angvel[ai]
                                - bodies.apply_world_inv_inertia(ai, p.ra.cross(imp));
                            bodies.linvel[bi] += imp * imb;
                            bodies.angvel[bi] = bodies.angvel[bi]
                                + bodies.apply_world_inv_inertia(bi, p.rb.cross(imp));
                        }
                        // —— 摩擦（两切向 + 锥 radial clamp）——
                        for (t, tmass, key) in [(p.t1, p.tmass1, 0usize), (p.t2, p.tmass2, 1usize)]
                        {
                            let va = bodies.velocity_at(ai, p.ra);
                            let vb = bodies.velocity_at(bi, p.rb);
                            let vt = (vb - va).dot(t);
                            let lam = tmass * (-vt);
                            let (acc, dl) = match key {
                                0 => {
                                    let nv =
                                        (p.pt1 + lam).clamp(-p.friction * p.pn, p.friction * p.pn);
                                    let d = nv - p.pt1;
                                    p.pt1 = nv;
                                    (p.pt1, d)
                                }
                                _ => {
                                    let nv =
                                        (p.pt2 + lam).clamp(-p.friction * p.pn, p.friction * p.pn);
                                    let d = nv - p.pt2;
                                    p.pt2 = nv;
                                    (p.pt2, d)
                                }
                            };
                            let _ = acc;
                            if dl != 0.0 {
                                let imp = t * dl;
                                bodies.linvel[ai] -= imp * ima;
                                bodies.angvel[ai] = bodies.angvel[ai]
                                    - bodies.apply_world_inv_inertia(ai, p.ra.cross(imp));
                                bodies.linvel[bi] += imp * imb;
                                bodies.angvel[bi] = bodies.angvel[bi]
                                    + bodies.apply_world_inv_inertia(bi, p.rb.cross(imp));
                            }
                        }
                        // 摩擦锥（radial）：|pt_vec| ≤ μ·pn。
                        let max_f = p.friction * p.pn;
                        let f2 = p.pt1 * p.pt1 + p.pt2 * p.pt2;
                        if f2 > max_f * max_f && f2 > 1e-20 {
                            let s = max_f / f2.sqrt();
                            let d1 = p.pt1 * s - p.pt1;
                            let d2 = p.pt2 * s - p.pt2;
                            p.pt1 *= s;
                            p.pt2 *= s;
                            let imp = p.t1 * d1 + p.t2 * d2;
                            bodies.linvel[ai] -= imp * ima;
                            bodies.angvel[ai] = bodies.angvel[ai]
                                - bodies.apply_world_inv_inertia(ai, p.ra.cross(imp));
                            bodies.linvel[bi] += imp * imb;
                            bodies.angvel[bi] = bodies.angvel[bi]
                                + bodies.apply_world_inv_inertia(bi, p.rb.cross(imp));
                        }
                    }
                }
            }
        }

        // 5) 岛级休眠与唤醒（§4.11 / §3 稳定性）。
        //    - 休眠岛（全员 asleep）保持冻结，不解算不积分（积分器跳过 asleep）；
        //    - 清醒岛：任一成员 awake → 全岛同步为 awake（外部唤醒传播）；
        //    - 全员速度低于阈值持续 sleep_time → 岛内**原子**入睡（同帧全员睡），
        //      不存在"部分睡部分醒"状态，从机制上排除反复唤醒。
        for island in &islands {
            let has_awake = island.bodies.iter().any(|&bi| bodies.awake[bi as usize]);
            if !has_awake {
                continue;
            }
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
                let ang = bodies.angvel[i].length();
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
                        bodies.angvel[i] = Vec3::ZERO;
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
        // 无接触的孤立动体（不在任何岛内）：单独计时。
        for (i, used) in in_island.iter().enumerate() {
            if *used || !bodies.is_dynamic(i) || !bodies.awake[i] {
                continue;
            }
            let lin = bodies.linvel[i].length();
            let ang = bodies.angvel[i].length();
            if lin < config.sleep_linear && ang < config.sleep_angular {
                bodies.sleep_timer[i] += dt;
                if bodies.sleep_timer[i] >= config.sleep_time {
                    bodies.awake[i] = false;
                    bodies.linvel[i] = Vec3::ZERO;
                    bodies.angvel[i] = Vec3::ZERO;
                }
            } else {
                bodies.sleep_timer[i] = 0.0;
            }
        }

        // 6) 更新 warm 缓存。
        self.warm_cache.clear();
        for c in &constraints {
            let pts = c
                .points
                .iter()
                .map(|p| WarmPoint {
                    point: {
                        // 接触点世界坐标（锚点回推）。
                        bodies.position[c.a as usize] + p.ra
                    },
                    pn: p.pn,
                    pt1: p.pt1,
                    pt2: p.pt2,
                })
                .collect();
            self.warm_cache.insert(
                (c.a, c.b),
                WarmManifold {
                    normal: c.normal,
                    points: pts,
                },
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{Quat, Shape};
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
                    }],
                };
                solver.solve(&mut b, &[m], &cfg, dt);
            } else {
                solver.solve(&mut b, &[], &cfg, dt);
            }
            y += b.linvel[0].y * dt;
            b.position[0].y = y;
        }
        assert!((y - 1.0).abs() < 0.05, "rest y = {y}");
        assert!(b.linvel[0].y.abs() < 0.05, "linvel {:?}", b.linvel[0]);
    }
}
