//! world_health：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl World {
    /// §3 稳定性指标采集。
    pub fn health(&self) -> HealthReport {
        let mut rep = HealthReport {
            max_depth: 0.0,
            ..HealthReport::default()
        };
        for i in 0..self.bodies.len() {
            if !self.bodies.position[i].is_finite() || !self.bodies.linvel[i].is_finite() {
                rep.nan_bodies += 1;
            }
            // 活跃度只对动体有意义（静态 marker 恒置 awake=true 但不参与休眠）。
            if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                rep.awake_bodies += 1;
            }
        }
        let limit = self.config.contact_skin * 4.0;
        for m in &self.manifolds {
            rep.contacts += m.points.len() as u32;
            for p in &m.points {
                rep.max_depth = rep.max_depth.max(p.depth);
                if p.depth > limit {
                    rep.deep_penetrations += 1;
                }
            }
        }
        rep
    }
}
