//! pair：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl DefaultNarrowPhase {
    /// 薄入口：从 `bodies` 取形状/位姿后交给 `process_pair_shaped`（复合体递归走后者）。
    pub(crate) fn process_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        heightfields: &[HeightField],
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
    ) {
        let (sa, sb) = (&bodies.shape[a as usize], &bodies.shape[b as usize]);
        let pa = bodies.position[a as usize];
        let pb = bodies.position[b as usize];
        let ra = bodies.rot(a as usize);
        let rb = bodies.rot(b as usize);
        self.process_pair_shaped(
            a,
            b,
            bodies,
            sa,
            sb,
            pa,
            ra,
            pb,
            rb,
            heightfields,
            providers,
            out,
        );
    }
}
