//! bvh_phase：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 增量 BVH 宽相（§2.3 主路径）。
///
/// - 代理更新按体索引升序（确定性）；树跨帧保持（增量 refit/平衡）；
/// - fat margin = 2×skin（≥ 2cm），静置帧零树操作；
/// - M1 不提供删体 API（`World` 无删体），叶子与体索引一一对应。
///
/// **宽节点（8 路）实测结论（T2 尾，2026-09-13）**：`wide::WideBvh` 已按其
/// 验收接入过一版，树峰 7.34 → **3.10ms**（叶含 8 体 ⇒ 多数移动落在叶盒内、
/// 免 refit），但查询均 7.47 → **14.68ms**、峰 22.64 → **33.50ms**——
/// 叶粒度让候选数 8×（实测 候选均 **208 万/tick**，真实接触对仅 ≈2.25/体），
/// 查询是**候选受限**而非遍历受限 ⇒ 净负收益，**回退**（详见 docs/M1-PLAN.md）。
pub struct BvhBroadPhase {
    pub(crate) tree: DynamicBvh,
    pub(crate) skin: f32,
    /// 当前子步 dt（`set_step` 注入；速度自适应 fat 边距用）。
    pub(crate) dt: f32,
    pub(crate) leaves: Vec<u32>,
    /// 精确 AABB（含 skin 膨胀），与树内 fat AABB 分离。
    pub(crate) aabbs: Vec<Aabb>,
    pub(crate) pairs: Vec<(u32, u32)>,
    /// 查询缓存（M1 T2 查询提速）：每体候选列表（arena 偏移/长度）+ 上次查询
    /// 所用 fat 盒。完备性：树不变式「精确盒 ⊆ fat 盒」+ 缓存 fat 盒自上次
    /// 查询未变 ⇒ 两体 fat 盒若在末态相交则在上次查询时已相交——候选必然
    /// 已入表（逃出缓存盒或树代理被重插时失效重查；配对双侧发射 + 去重）。
    pub(crate) cache_fat: Vec<Aabb>,
    pub(crate) cand_off: Vec<u32>,
    pub(crate) cand_len: Vec<u32>,
    pub(crate) cand_arena: Vec<u32>,
    /// 上一帧「参与查询」位（动态且清醒）——睡眠状态翻转检测用（见 `compute_pairs`
    /// 的不变式注：翻转帧必须全缓存失效，否则睡眠侧不查询 + 清醒侧缓存陈旧
    /// 会漏掉新接近对；不变式测试 `bvh_pairs_match_brute_force_across_frames` 守门）。
    pub(crate) prev_awake: Vec<bool>,
    /// 诊断：上一帧各子阶段耗时（µs）：(AABB, 树更新/重建, 查询, 排序)。
    pub last_breakdown_us: (u64, u64, u64, u64),
    /// 诊断：上一帧候选总数（查询返回的候选条目数之和）——候选粒度/b 因子审计。
    pub last_cand_total: usize,
}

impl BvhBroadPhase {
    pub fn new(skin: f32) -> Self {
        Self {
            tree: DynamicBvh::new((skin * 2.0).max(0.02)),
            skin,
            dt: 1.0 / 60.0,
            leaves: Vec::new(),
            aabbs: Vec::new(),
            pairs: Vec::new(),
            cache_fat: Vec::new(),
            cand_off: Vec::new(),
            cand_len: Vec::new(),
            cand_arena: Vec::new(),
            prev_awake: Vec::new(),
            last_breakdown_us: (0, 0, 0, 0),
            last_cand_total: 0,
        }
    }

    /// 速度自适应 fat 边距（M1 宽相提速的核心开关之一）：
    /// `base + |v_lin|·dt·1.5`，上钳 0.5m。旧档上限 0.25 在查询缓存 + 就地
    /// 生长到位后放宽（T2 第十四段）：代价结构已变——宽 fat 盒只增**候选数**
    /// 不再增**遍历数**（缓存命中时零遍历），快体（少）的多占候选换逃逸率降。
    /// 确定性：只由（速度, dt）决定；同一状态 → 同一边距 → 同一树形。
    #[inline]
    /// 速度自适应边距（`base + v·dt·K`，上限 0.5）。
    ///
    /// **K 的标定（2026-09-14 第三段实测，8B 同窗口）**：查询相位拆段计时发现
    /// 成本**几乎全在「逃逸重查」**（refresh 均 8.08ms）而精确过滤只有 1.22ms
    /// （3.8ns/条，与隔离档一致）⇒ 杠杆是**逃逸频率**，不是过滤局部性
    /// （修正此前的 32ns/条归因；W2/W7「边距不是杠杆」的结论也据此修正为
    /// 「收窄边距有害、放大才是方向」）。K 扫描：1.5（旧值）→ 14.67/43.02（K=6，
    /// 取此档）/ 14.94/48.97（K=12，峰反涨——快体大盒把候选推高）。
    /// `K=6` 实测 broad 均 17.03→**14.67**、峰 43.50→43.02、树 均 2.65→**1.77**；
    /// **逐位中性**（配对仍精确过滤 ⇒ 物理不变；`m0_gates` 哈希不变、twin_match ✓）。
    pub(crate) fn fat_margin_for(&self, v: Vec3) -> f32 {
        let base = (self.skin * 2.0).max(0.02);
        let speed = v.length();
        (base + speed * self.dt * 6.0).min(0.5)
    }

    /// 树高（诊断/负载审计：健康树 ≈ 1.4·log2(n)）。
    pub fn tree_height(&self) -> u32 {
        self.tree.root_height()
    }

    /// 诊断：宽相存储的精确 AABB（可视化/验证用，§12.3 调试可视化钩子）。
    pub fn stored_aabb(&self, i: usize) -> Option<Aabb> {
        self.aabbs.get(i).copied()
    }
}

impl BvhBroadPhase {
    /// 0.5) 睡眠状态翻转检测（T2 查询缓存完备性的关键补丁）：任一体的
    ///      「参与查询」位（动态且清醒）帧间变化 ⇒ 本帧全缓存失效。
    ///      原因：睡眠侧不查询，其新邻居只能靠清醒侧查询兜底；而清醒侧
    ///      可能因未逃出自身 fat 盒而复用旧候选表 → 漏对（不变式测试
    ///      `bvh_pairs_match_brute_force_across_frames` 实测抓出）。
    ///      完备性再证：两体 fat 盒均冻结且不相交时，精确盒（⊆ fat）不可能
    ///      新相交——故「翻转帧」是唯一漏洞，翻转帧全体重查即封闭。
    ///      确定性：只读 awake 位（纯状态），与线程数无关。
    fn detect_sleep_flip(&mut self, bodies: &BodySet) {
        let n = bodies.len();
        self.prev_awake.resize(n, true);
        let mut flip = false;
        for i in 0..n {
            let participates = bodies.is_dynamic(i) && bodies.awake[i];
            if self.prev_awake[i] != participates {
                self.prev_awake[i] = participates;
                flip = true;
            }
        }
        if flip {
            for f in self.cache_fat.iter_mut() {
                *f = Aabb::EMPTY;
            }
        }
    }

    /// 0) AABB 计算：纯函数按下标写槽位（§6 并行契约）。分块并行且
    ///    spawn 数受控（for_each_chunk_mut：≤ threads−1，禁止线程爆炸）。
    ///    门槛 32768：单体内 AABB ≈ 30ns，低于此并行开销（≈0.6ms 启动）不划算。
    fn update_aabbs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        provider_bounds: &[Aabb],
        threads: usize,
        full: bool,
    ) {
        let aabbs = &mut self.aabbs;
        let skin = self.skin;
        let bodies_ref: &BodySet = bodies;
        let hfs: &[Aabb] = hf_bounds;
        vxl_phys_core::schedule::for_each_chunk_mut(aabbs, threads, 32768, |start, _len, slot| {
            for (k, s) in slot.iter_mut().enumerate() {
                let i = start + k;
                if !full && (!bodies_ref.is_dynamic(i) || !bodies_ref.awake[i]) {
                    continue; // 睡眠/静态：位置未变，沿用上帧 AABB。
                }
                *s = shape_aabb(
                    &bodies_ref.shape[i],
                    bodies_ref.position[i],
                    bodies_ref.rot(i),
                    skin,
                    hfs,
                    provider_bounds,
                );
            }
        });
    }

    /// 1) 代理更新（需要重建：中位分裂全量重建；否则增量只动清醒体）。
    ///    面积启发式对结构化插入序（网格行优先）会链化，
    ///    阈值 = 3·log2(n) + 16（确定性纯函数，不依赖时序）。
    ///    树变异按体索引升序串行（结构操作不可并行）。
    fn update_proxies(&mut self, bodies: &BodySet, n: usize, rebuild_due: bool) {
        if n >= 32 && (self.leaves.is_empty() || rebuild_due) {
            let items: Vec<(u32, Aabb)> = (0..n).map(|i| (i as u32, self.aabbs[i])).collect();
            self.leaves = self.tree.rebuild(&items);
            // 全量重建：所有缓存失效（代理盒全部重设）。
            self.cache_fat.clear();
            self.cache_fat.resize(n, Aabb::EMPTY);
        } else {
            for i in 0..n {
                if (i as u32) >= self.leaves.len() as u32 {
                    self.leaves.push(self.tree.insert(i as u32, self.aabbs[i]));
                    self.cache_fat.resize(n, Aabb::EMPTY);
                } else if bodies.is_dynamic(i) && bodies.awake[i] {
                    // M1：速度自适应边距——快速体在其 fat 盒内连续多帧零结构操作。
                    let m = self.fat_margin_for(bodies.linvel[i]);
                    let (nl, changed) =
                        self.tree
                            .move_proxy_scaled(self.leaves[i], self.aabbs[i], m);
                    if changed {
                        // 代理盒变化（就地生长或重插）→ 查询缓存失效（T2）。
                        self.leaves[i] = nl;
                        self.cache_fat[i] = Aabb::EMPTY;
                    }
                }
            }
        }
    }

    /// 查询块数（刷新段与过滤段**共用同一划分**）：块序 = 体区间序 ⇒ 合并确定性。
    fn query_chunks(threads: usize, n_dyns: usize) -> usize {
        if threads > 1 && n_dyns >= 4096 {
            n_dyns.div_ceil(n_dyns.div_ceil(threads))
        } else {
            1
        }
    }

    /// 2a) 候选重查：仅对逃出缓存 fat 盒的体重走树（只读树；每块本地 arena + 条目表），
    ///     随后串行合并。条目 = (体, 本地偏移, 长度, fat 盒)。
    fn refresh_candidates(&mut self, bodies: &BodySet, threads: usize, dyns: &[u32]) {
        let n_chunks = Self::query_chunks(threads, dyns.len());
        let chunk_len = dyns.len().div_ceil(n_chunks);
        // 条目 = (体, 本地偏移, 长度, fat 盒)。
        type RefreshChunk = (Vec<u32>, Vec<(u32, u32, u32, Aabb)>);
        let mut refresh: Vec<RefreshChunk> =
            (0..n_chunks).map(|_| (Vec::new(), Vec::new())).collect();
        {
            let this = &*self;
            let dyns_ref: &[u32] = dyns;
            let bodies_ref: &BodySet = bodies;
            vxl_phys_core::schedule::for_each_chunk_mut(
                &mut refresh,
                threads,
                2,
                |start_slot, _len, slots| {
                    let mut tmp: Vec<u32> = Vec::new();
                    for (k, (arena, entries)) in slots.iter_mut().enumerate() {
                        let oi = start_slot + k;
                        let start = oi * chunk_len;
                        let end = ((oi + 1) * chunk_len).min(dyns_ref.len());
                        for &i in &dyns_ref[start..end] {
                            let iu = i as usize;
                            let exact = this.aabbs[iu];
                            if this.cache_fat[iu].contains(&exact) {
                                continue; // 未逃出缓存盒：候选复用。
                            }
                            let m = this.fat_margin_for(bodies_ref.linvel[iu]);
                            let fat = exact.grown(m);
                            this.tree.query(&fat, &mut tmp);
                            let base = arena.len() as u32;
                            arena.extend_from_slice(&tmp);
                            entries.push((i, base, tmp.len() as u32, fat));
                        }
                    }
                },
            );
        }
        let mut cand_total = 0usize;
        for (arena, entries) in refresh {
            let arena_base = self.cand_arena.len() as u32;
            self.cand_arena.extend_from_slice(&arena);
            for (i, off, len, fat) in entries {
                let iu = i as usize;
                self.cand_off[iu] = arena_base + off;
                self.cand_len[iu] = len;
                self.cache_fat[iu] = fat;
                cand_total += len as usize;
            }
        }
        self.last_cand_total = cand_total;
    }

    /// 2b) 精确过滤 + 并行收集（dyn-dyn 双侧发射由最终排序去重收敛；dyn-static 由动体侧发起）。
    fn collect_pairs(&mut self, threads: usize, dyns: &[u32]) {
        let n_chunks = Self::query_chunks(threads, dyns.len());
        let chunk_len = dyns.len().div_ceil(n_chunks);
        let mut outs: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n_chunks];
        {
            let this = &*self;
            let dyns_ref: &[u32] = dyns;
            vxl_phys_core::schedule::for_each_chunk_mut(
                &mut outs,
                threads,
                2,
                |start_slot, _len, slots| {
                    // slots[k] = outs[start_slot + k]（块内逐槽对应各自的体区间）。
                    for (k, co) in slots.iter_mut().enumerate() {
                        let oi = start_slot + k;
                        let start = oi * chunk_len;
                        let end = ((oi + 1) * chunk_len).min(dyns_ref.len());
                        for &i in &dyns_ref[start..end] {
                            let iu = i as usize;
                            let off = this.cand_off[iu] as usize;
                            let len = this.cand_len[iu] as usize;
                            for &j in &this.cand_arena[off..off + len] {
                                let ju = j as usize;
                                if ju == iu {
                                    continue;
                                }
                                if this.aabbs[iu].overlaps(&this.aabbs[ju]) {
                                    let (a, b) = if iu < ju { (i, j) } else { (j, i) };
                                    co.push((a, b));
                                }
                            }
                        }
                    }
                },
            );
        }
        for mut co in outs {
            self.pairs.append(&mut co);
        }
    }

    /// arena 压实（少见：仅当垃圾占比高时；重建各体偏移）。
    fn compact_arena(&mut self, dyns: &[u32]) {
        if self.cand_arena.len() > 4_000_000 {
            let mut fresh: Vec<u32> = Vec::with_capacity(self.cand_arena.len());
            for &i in dyns {
                let iu = i as usize;
                let off = self.cand_off[iu] as usize;
                let len = self.cand_len[iu] as usize;
                if len == 0 {
                    continue;
                }
                let noff = fresh.len() as u32;
                fresh.extend_from_slice(&self.cand_arena[off..off + len]);
                self.cand_off[iu] = noff;
            }
            self.cand_arena = fresh;
        }
    }
}

impl BroadPhase for BvhBroadPhase {
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        provider_bounds: &[Aabb],
        jobs: &dyn JobSystem,
    ) -> &[(u32, u32)] {
        // 计时走跨目标探针（wasm32 无时钟；原生不变）。
        let t_aabb = vxl_phys_core::probe::start();
        self.pairs.clear();
        let n = bodies.len();
        // 注：绝不 clear——AABB 数组跨帧保留（睡眠/静态体沿用上帧值，
        // 增量分支只重算清醒动体；clear 会把它们清成零 AABB）。
        if self.aabbs.len() > n {
            self.aabbs.truncate(n);
        }
        self.aabbs.resize(
            n,
            Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            },
        );
        let threads = jobs.threads();
        // 0.5) 睡眠状态翻转检测（理由与完备性再证见私有方法的文档）：翻转帧全缓存失效。
        self.detect_sleep_flip(bodies);
        // 全量 AABB 分支与「是否重建树」**解耦**（T2）：AABB 本就增量维护——
        // 仅首次（叶子未建）/ 体数变化（新体）需要全量重算（否则静态体 /
        // 睡眠体会带着零 AABB 进树）；纯「树链化超限」的重建 tick 复用现有
        // AABB（旧实现让重建 tick 白付一次 20 万体全量 AABB ≈16ms）。
        // 非全量分支 = 增量：只处理「清醒动体」（睡眠体与静态体位置不变，
        // AABB 与树内代理均无需更新——稳态零树操作，M1 规模档的成败手）。
        let leaves_missing = self.leaves.len() != n;
        let rebuild_due = !leaves_missing && n >= 32 && {
            let limit = 3.0 * (n as f32).log2() + 16.0;
            self.tree.root_height() as f32 > limit
        };
        let full = leaves_missing;
        // 0) AABB 计算：纯函数按下标写槽位（§6 并行契约）。分块并行且
        //    spawn 数受控（for_each_chunk_mut：≤ threads−1，禁止线程爆炸）。
        //    门槛 32768：单体内 AABB ≈ 30ns，低于此并行开销（≈0.6ms 启动）不划算。
        self.update_aabbs(bodies, hf_bounds, provider_bounds, threads, full);
        let d_aabb = vxl_phys_core::probe::us(t_aabb);
        let t_tree = vxl_phys_core::probe::start();
        // 1) 代理更新（需要重建：中位分裂全量重建；否则增量只动清醒体）。
        //    面积启发式对结构化插入序（网格行优先）会链化，
        //    阈值 = 3·log2(n) + 16（确定性纯函数，不依赖时序）。
        //    树变异按体索引升序串行（结构操作不可并行）。
        self.update_proxies(bodies, n, rebuild_due);
        let d_tree = vxl_phys_core::probe::us(t_tree);
        let t_query = vxl_phys_core::probe::start();
        // 2) 清醒动体查询（dyn-dyn 双侧发射由最终排序去重收敛；dyn-static 由
        //    动体侧发起）。睡眠体不查询：沉睡体不产生新接触；被唤醒/被撞由
        //    对方（清醒体）的查询反向命中（睡眠叶仍在树内），唤醒语义不变。
        //    查询缓存（T2）：仅对逃出缓存 fat 盒的体重走树（候选安全复用，
        //    见 `cache_fat` 注）；随后并行只读消费候选 + 精确过滤。
        let dyns: Vec<u32> = (0..n as u32)
            .filter(|&i| {
                let i = i as usize;
                bodies.is_dynamic(i) && bodies.awake[i]
            })
            .collect();
        self.cache_fat.resize(n, Aabb::EMPTY);
        self.cand_off.resize(n, 0);
        self.cand_len.resize(n, 0);
        // 分块并行重查（只读树；每块本地 arena + 条目表），随后串行合并。
        self.refresh_candidates(bodies, threads, &dyns);
        self.collect_pairs(threads, &dyns);
        self.compact_arena(&dyns);
        let d_query = vxl_phys_core::probe::us(t_query);
        let t_sort = vxl_phys_core::probe::start();
        self.pairs.sort_unstable();
        self.pairs.dedup();
        self.last_breakdown_us = (d_aabb, d_tree, d_query, vxl_phys_core::probe::us(t_sort));
        &self.pairs
    }

    fn query_aabb(&mut self, aabb: &Aabb, out: &mut Vec<u32>) {
        self.tree.query(aabb, out);
    }

    fn set_step(&mut self, dt: f32) {
        if dt > 0.0 && dt.is_finite() {
            self.dt = dt;
        }
    }

    fn breakdown_us(&self) -> (u64, u64, u64, u64) {
        self.last_breakdown_us
    }

    fn tree_height(&self) -> u32 {
        self.tree.root_height()
    }

    fn cand_total(&self) -> usize {
        self.last_cand_total
    }
}
