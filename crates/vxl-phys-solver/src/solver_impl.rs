//! solver_impl：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

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
            wake_streak: Vec::new(),
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
            island_diag: IslandDiag::default(),
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

    pub(crate) fn solve_phase(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
        cleanup: bool,
    ) {
        let e_threshold = config.restitution_threshold;
        // 准静态"安座"趟（`settled_hold_iterations`；0 = 关闭 = 不走该路径，逐位同旧）。
        // **两趟都跑**：睡眠判定看的是**带偏置趟**之后的速度（本函数末尾逐岛判），
        // 而无偏置趟的速度才是下一子步的初值 ⇒ 两边都要清。
        let settled_hold = config.settled_hold_iterations;
        let hold_max_vn = 4.0 * config.sleep_linear;
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
        // 诊断拆点（T4 第三刀）：把「建岛」与「按组 gather（每体 4 次 push + 世界逆惯量矩阵）」
        // 分开计时——后者实测占求解相位约 24%（36000 体 × 约 41 ns），是唯一还串行的重活。
        // ⚠️ 并行化它**反而更慢**（见下 `fill` 处注）：该段与解算同属**访存带宽受限**。
        let t_fill = vxl_phys_core::probe::start();
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
        let mut group_manifs_diag: Vec<u32> = Vec::with_capacity(g_count);
        let mut group_us_diag: Vec<u64> = vec![0; g_count];
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
            // 诊断（T4）：每组流形数 = 工作量代理（与 group_us 一起判负载不均）。
            let mut mf_here = 0u32;
            for &ii in &awake[s0..e0] {
                mf_here += islands[ii].manifs.len() as u32;
            }
            group_manifs_diag.push(mf_here);
            for &ii in &awake[s0..e0] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    local_of[i] = group_lv[g].len() as u32;
                    if SUBISLAND_SLEEP && !bodies.awake[i] {
                        // 睡眠体落在清醒岛里（子块睡眠开启时会发生）：**求解期按静态处理**
                        // （质量/惯量置 0 + 速度置 0），否则它会被每子步写入速度
                        // ——"位置冻结、速度被写"的僵尸体，唤醒时会跳。
                        group_lv[g].push(Vec3::ZERO);
                        group_av[g].push(Vec3::ZERO);
                        group_iw[g].push(Mat3::world_inv_inertia(bodies.rot(i), Vec3::ZERO));
                        group_im[g].push(0.0);
                        continue;
                    }
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

        let d_fill = vxl_phys_core::probe::us(t_fill);
        let d_island_all = vxl_phys_core::probe::us(t_island);
        let d_island = d_island_all.saturating_sub(d_fill);
        self.island_diag.fill_us = d_fill;
        self.island_diag.island_build_us = d_island;
        let t_solve = vxl_phys_core::probe::start();
        let islands_len = islands.len() as u32;
        let mut scope_us_diag = 0u64;
        // 每**解算**（子步）的三段合计（各组之和）——与 manifolds/points 同帧，做归一化用。
        let mut build_us_diag = 0u64;
        let mut warm_us_diag = 0u64;
        let mut iter_us_diag = 0u64;
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
            let t_scope = vxl_phys_core::probe::start();
            std::thread::scope(|s| {
                // iter_mut 逐容器取出元素可变借用（按 g 索引整体借用会跨迭代重叠）。
                for (g, ((((lv, av), (cbuf, wout)), det), t_slot)) in group_lv
                    .iter_mut()
                    .zip(group_av.iter_mut())
                    .zip(build_bufs.iter_mut().zip(warm_outs.iter_mut()))
                    .zip(details.iter_mut())
                    .zip(group_us_diag.iter_mut())
                    .enumerate()
                {
                    let (s0, e0) = groups[g];
                    let iw_g: &[Mat3] = &iw_ref[g];
                    let im_g: &[f32] = &im_ref[g];
                    let job = move || {
                        // 诊断（T4）：本组墙钟（组间最大值 = scope 墙钟 ⇒ 离散度 = 负载不均）。
                        let t0 = vxl_phys_core::probe::start();
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
                            settled_hold,
                            hold_max_vn,
                            det,
                        );
                        *t_slot = vxl_phys_core::probe::us(t0);
                    };
                    if g + 1 == g_count {
                        let mut job = job;
                        job();
                    } else {
                        s.spawn(job);
                    }
                }
            });
            scope_us_diag = vxl_phys_core::probe::us(t_scope);
            let mut tot = [0u64; 5];
            for d in &details {
                for (k, v) in d.iter().enumerate() {
                    tot[k] += v;
                }
            }
            for (k, v) in tot.iter().enumerate().take(3) {
                self.last_detail_us[k + 1] += v;
            }
            build_us_diag = tot[0];
            warm_us_diag = tot[1];
            iter_us_diag = tot[2];
            self.last_points = (tot[3], tot[4]);
        } else if !awake.is_empty() {
            let mut det = [0u64; 5];
            let t_ser = vxl_phys_core::probe::start();
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
                settled_hold,
                hold_max_vn,
                &mut det,
            );
            for (k, v) in det.iter().enumerate().take(3) {
                self.last_detail_us[k + 1] += v;
            }
            build_us_diag = det[0];
            warm_us_diag = det[1];
            iter_us_diag = det[2];
            self.last_points = (det[3], det[4]);
            group_us_diag[0] = vxl_phys_core::probe::us(t_ser);
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
        // 诊断收尾（T4）：并行段墙钟 / 每组耗时 / 每组流形数 → `island_diag`。
        // 串行占比 = 求解相位 − scope；它就是扩展比的上限（判据写在 `IslandDiag` 文档里）。
        self.island_diag.islands = islands_len;
        self.island_diag.manifolds = manifolds.len() as u32;
        self.island_diag.g_count = g_count as u32;
        self.island_diag.gather_us = d_island;
        self.island_diag.scope_us = scope_us_diag;
        // `d_solve` 只覆盖 scope 之后的部分（t_solve 在 gather 之后才起）⇒ 别重复减 gather。
        self.island_diag.scatter_us = d_solve.saturating_sub(scope_us_diag);
        self.island_diag.group_us = std::mem::take(&mut group_us_diag);
        self.island_diag.group_manifs = std::mem::take(&mut group_manifs_diag);
        self.island_diag.build_us = build_us_diag;
        self.island_diag.warm_us = warm_us_diag;
        self.island_diag.iter_us = iter_us_diag;
        self.island_diag.points = self.last_points.0 as u32;
        self.island_diag.warm_count = self.warm_slots.len() as u32;
        self.island_diag.warm_bytes_per_slot = std::mem::size_of::<WarmManifold>() as u32;
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
        // 唤醒接触数门：**每次本函数调用清零**（寿命 = 语义，见 `wake_streak` 字段注）。
        // `k == 0` 时整段不参与 ⇒ 与现行行为**逐位一致**（不含任何算术语义改动）。
        let wake_gate_k = config.wake_gate_k;
        if wake_gate_k > 0 && !sleep_islands.is_empty() {
            if self.wake_streak.len() < bodies.len() {
                self.wake_streak.resize(bodies.len(), 0);
            }
            self.wake_streak[..bodies.len()].fill(0);
        }
        for island in sleep_islands {
            if SUBISLAND_SLEEP {
                // —— 实验：子块睡眠（见 `SUBISLAND_SLEEP` 注）——
                // ① 唤醒 = "实质相互作用"：与**清醒**邻居的相对运动显著才唤醒（连通本身不唤醒），
                //    并带**滞回**（阈值 ×`SUBISLAND_WAKE_MULT`），否则边界体被闪烁体每子步叫醒。
                for &mi in &island.manifs {
                    let m = &manifolds[mi];
                    let (a, b) = (m.a as usize, m.b as usize);
                    let (sa, sb) = (bodies.awake[a], bodies.awake[b]);
                    if sa == sb {
                        continue; // 双醒：无需唤醒；双睡：不在清醒岛内
                    }
                    let (s, w) = if sa { (b, a) } else { (a, b) };
                    let rel = (bodies.linvel[w] - bodies.linvel[s]).length();
                    let hot = rel > SUBISLAND_WAKE_MULT * config.sleep_linear
                        || bodies.angvel(w).length() > SUBISLAND_WAKE_MULT * config.sleep_angular;
                    let fast = rel > WAKE_GATE_FAST_MULT * config.sleep_linear;
                    if wake_gate_k == 0 || fast {
                        // 现行（或强撞直通）：任一 hot 邻居立即唤醒
                        if hot {
                            bodies.awake[s] = true;
                            bodies.sleep_timer[s] = 0.0;
                        }
                    } else if hot {
                        // 接触数门：同一次调用内累计 ≥K 条 hot 观测才唤醒；断一次清零。
                        let n = &mut self.wake_streak[s];
                        *n = n.saturating_add(1);
                        if *n >= wake_gate_k {
                            bodies.awake[s] = true;
                            bodies.sleep_timer[s] = 0.0;
                            *n = 0;
                        }
                    } else {
                        self.wake_streak[s] = 0;
                    }
                }
                // ② 逐体计时 + 逐体入睡（不要求整岛齐）
                for &bi in &island.bodies {
                    let i = bi as usize;
                    if !bodies.awake[i] {
                        continue; // 睡着的：本轮不动它（上面的唤醒已判过）
                    }
                    let lin = bodies.linvel[i].length();
                    let ang = bodies.angvel(i).length();
                    if lin < config.sleep_linear && ang < config.sleep_angular {
                        bodies.sleep_timer[i] += dt;
                        if bodies.sleep_timer[i] >= config.sleep_time {
                            bodies.awake[i] = false;
                            bodies.linvel[i] = Vec3::ZERO;
                            bodies.set_angvel_raw(i, Vec3::ZERO);
                        }
                    } else {
                        bodies.sleep_timer[i] = 0.0;
                    }
                }
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
            let mut fast_n = 0u64;
            let mut body_n = 0u64;
            for &bi in &island.bodies {
                let i = bi as usize;
                body_n += 1;
                let lin = bodies.linvel[i].length();
                let ang = bodies.angvel(i).length();
                if lin >= config.sleep_linear || ang >= config.sleep_angular {
                    all_slow = false;
                    fast_n += 1;
                    if !SLEEP_DIAG_DEEP {
                        break;
                    }
                }
            }
            if SLEEP_DIAG_DEEP {
                use std::sync::atomic::Ordering::Relaxed;
                SLEEP_D_FAST_BODY.fetch_add(fast_n, Relaxed);
                SLEEP_D_BODY_ALL.fetch_add(body_n, Relaxed);
            }
            if all_slow {
                let mut min_timer = f32::MAX;
                for &bi in &island.bodies {
                    let i = bi as usize;
                    bodies.sleep_timer[i] += dt;
                    min_timer = min_timer.min(bodies.sleep_timer[i]);
                }
                use std::sync::atomic::Ordering::Relaxed;
                SLEEP_D_WAIT.fetch_add(1, Relaxed);
                SLEEP_D_WAIT_MAX_MS.fetch_max((min_timer * 1000.0) as u64, Relaxed);
                if min_timer >= config.sleep_time {
                    SLEEP_D_SLEPT.fetch_add(1, Relaxed);
                    for &bi in &island.bodies {
                        let i = bi as usize;
                        bodies.awake[i] = false;
                        bodies.linvel[i] = Vec3::ZERO;
                        bodies.set_angvel_raw(i, Vec3::ZERO);
                    }
                }
            } else {
                SLEEP_D_FAST.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
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
