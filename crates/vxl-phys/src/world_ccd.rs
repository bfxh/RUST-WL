//! world_ccd：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl World {
    /// 选择性 CCD（§4.12）：保守推进采样。
    ///
    /// 位置积分已把体推到 p1；这里从 p0 = p1 − v·dt 回扫到 p1，找到首个与
    /// 静态/沉睡目标接触的采样段，把体钳位到上一安全采样点，并清零指向
    /// 表面的法向速度（M1 非弹性停下）。动-动 CCD 在 M2 接入。
    pub(crate) fn ccd_pass(&mut self, dt: f32) {
        if !self.config.ccd_speed_threshold.is_finite() {
            return;
        }
        let n = self.bodies.len();
        let flagged: Vec<usize> = (0..n)
            .filter(|&i| ccd::needs_ccd(&self.bodies, i, &self.config, dt))
            .collect();
        for &i in &flagged {
            let v = self.bodies.linvel[i];
            let steps = ccd::step_count(&self.bodies, i, &self.config, dt);
            let sub = dt / steps as f32;
            let p1 = self.bodies.position[i];
            let p0 = p1 - v * dt;

            // 扫掠 AABB → 候选（静态/沉睡体）+ 高度场。
            let lo = p0.min(p1);
            let hi = p0.max(p1);
            let swept = Aabb {
                min: lo - Vec3::splat(self.config.contact_skin),
                max: hi + Vec3::splat(self.config.contact_skin),
            };
            let mut candidates: Vec<u32> = Vec::new();
            self.broad.query_aabb(&swept, &mut candidates);
            candidates.retain(|&j| {
                let j = j as usize;
                j != i && (!self.bodies.is_dynamic(j) || !self.bodies.awake[j])
            });

            let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(candidates.len());
            for &j in &candidates {
                let (a, b) = if (j as usize) < i {
                    (j, i as u32)
                } else {
                    (i as u32, j)
                };
                pairs.push((a, b));
            }
            // 高度场 marker 体本身是静态叶子，已在 candidates 内。
            let mut hit: Option<(f32, Vec3)> = None;
            for s in 1..=steps {
                let t = sub * s as f32;
                self.bodies.position[i] = p0 + v * t;
                // CCD 逐采样已在时间维上走路径 ⇒ 不再叠加视野预测（显式清零，
                // 否则会继承主窄相调用设的 `predict_dt`）。
                self.narrow.set_predict_dt(0.0);
                self.narrow.collide(
                    &self.bodies,
                    &pairs,
                    self.terrain.slice(),
                    &self.providers,
                    &mut self.ccd_manifolds,
                    self.jobs.as_ref(),
                );
                if let Some(m) = self.ccd_manifolds.first() {
                    // 法线 a→b；换算成「推离表面、指向动体」的方向。
                    let n_into_body = if m.a == i as u32 { -m.normal } else { m.normal };
                    // **只有「正在接近表面」的采样才算命中**（命中判据细化，本轮修复）：
                    // 贴地滑行/静置的体在每个采样都天生有接触，按「有接触即命中」会
                    // 把它钳回起点、原地锁死（实测：弹体滑到墙前 0.1 m 停住）。
                    // 接近判据：法线指向动体 ⇒ 速度沿它 < 0 即压向表面。
                    let closing = n_into_body.dot(v) < 0.0;
                    if closing {
                        hit = Some((t, n_into_body));
                        break;
                    }
                }
            }
            if let Some((t_hit, n_into_body)) = hit {
                let t_safe = (t_hit - sub).max(0.0);
                self.bodies.position[i] = p0 + v * t_safe;
                let vn = self.bodies.linvel[i].dot(n_into_body);
                if vn < 0.0 {
                    self.bodies.linvel[i] -= n_into_body * vn;
                }
            } else {
                self.bodies.position[i] = p1;
            }
        }
    }
}
