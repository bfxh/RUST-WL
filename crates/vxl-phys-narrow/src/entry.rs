//! entry：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 内部时「最大平面距」（全部为负；面数小，代价可忽略）。
pub(crate) fn max_plane_d_of(poly: &WorldPoly, p: Vec3) -> f32 {
    let mut max_d = f32::MIN;
    for f in 0..poly.face_normal.len() {
        let s = poly.face_start[f] as usize;
        let d = (p - poly.verts[s]).dot(poly.face_normal[f]);
        if d > max_d {
            max_d = d;
        }
    }
    max_d
}

impl NarrowPhase for DefaultNarrowPhase {
    fn collide(
        &mut self,
        bodies: &vxl_phys_core::BodySet,
        pairs: &[(u32, u32)],
        heightfields: &[HeightField],
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
        jobs: &dyn JobSystem,
    ) {
        out.clear();
        // 每对各自的充气量（`clip` 逐点过滤复用）；默认 0 ⇒ 逐位同现行。
        self.inflate = 0.0;
        // 世界多面体填充缓存跨帧失效（体在帧间移动；键只含体号+形状）。
        self.cached_a = (u32::MAX, u64::MAX);
        self.cached_b = (u32::MAX, u64::MAX);
        self.cached_ax_a = (u32::MAX, u64::MAX, [Vec3::ZERO; 3]);
        self.cached_ax_b = (u32::MAX, u64::MAX, [Vec3::ZERO; 3]);
        self.cached_hull = [(u32::MAX, u64::MAX), (u32::MAX, u64::MAX)];
        let threads = jobs.threads();
        // 小规模串行（线程启动开销 > 收益）；并行 = 每块独立 clone（含自有
        // scratch），结果按块序拼接 = pair 序（§5 确定性契约）。
        if threads <= 1 || pairs.len() < 2048 {
            for &(a, b) in pairs {
                self.process_pair(a, b, bodies, heightfields, providers, out);
            }
            return;
        }
        let this = &*self;
        let n_chunks = threads.min(pairs.len().div_ceil(2048));
        let chunk = pairs.len().div_ceil(n_chunks);
        // 输出缓冲预分配（T3 结构项②的第一片）：8B 场景实测对/流形 ≈ 15:1，
        // 旧实现每块从空 Vec 逐次增长（每块 ~8 次重分配 + memcpy），且最终
        // 拼接要把全部流形再搬一遍。按下界预留容量即免掉这一段。
        let out_hint = self.out_hint.max(16);
        let mut outs: Vec<Vec<Manifold>> = (0..n_chunks)
            .map(|_| Vec::with_capacity(out_hint / n_chunks + 8))
            .collect();
        out.reserve(out_hint);
        vxl_phys_core::schedule::for_each_chunk_mut(
            &mut outs,
            threads,
            2,
            |start_slot, _len, slots| {
                // slots[k] = outs[start_slot + k]（块内逐槽对应各自的 pair 区间）。
                for (k, co) in slots.iter_mut().enumerate() {
                    let oi = start_slot + k;
                    let range = oi * chunk..((oi + 1) * chunk).min(pairs.len());
                    let mut np = this.clone();
                    for &(a, b) in &pairs[range] {
                        np.process_pair(a, b, bodies, heightfields, providers, co);
                    }
                }
            },
        );
        let mut produced = 0usize;
        for mut co in outs {
            produced += co.len();
            out.append(&mut co);
        }
        // 下一帧的容量提示（纯性能提示，不参与任何判定 ⇒ 确定性无关）。
        self.out_hint = produced;
    }
}
