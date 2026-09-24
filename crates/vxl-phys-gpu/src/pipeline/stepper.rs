//! stepper：门面「流体步进后端」的 **GPU 实现**（`vxl_phys_core::interop::FluidStepper`）。
//!
//! 用法：宿主用**全量**粒子（流体 + 边界段）建好 `Packet`（一次），交给门面；此后每 tick
//! 门面重建好边界段后调 [`GpuFluidStepper::step`]，本档只把**边界段**写回卡上
//! （`upload_boundary_segment`）→ 卡上推进一个 tick → 聚合每体 `(F, τ)`。
//!
//! **三条不变式**（对应 `PLAN-gpu.md` §13.7 的契约）：
//! - **粒子数固定**：`n_fluid` 与边界段长度必须与建 `Packet` 时一致（段长变了要重建 `Packet`；
//!   `upload_boundary_segment` 会**显式报错**而不是静默截断半段）；
//! - **逐粒质量 `pmass` 不变**（形状与晶格间距不变时是常量）——`step` 收下只做**长度核对**；
//! - `bounds()` 读**卡上**算好的包围盒（`read_box`）：宿主那份流体状态启用后端后不再推进，
//!   拿它做近域过滤会偏旧。

use super::*;

use vxl_phys_core::interop::FluidStepper;
use vxl_phys_core::Vec3;

/// 卡上步进后端：`Packet` + 聚合阶段 + 最近一次反作用（门面按同口径读）。
pub struct GpuFluidStepper {
    pk: Packet,
    stage: ReactionStage,
    pc: PacketCfg,
    substeps: usize,
    n_fluid: u32,
    nb: usize,
    reactions: Vec<(u32, Vec3, Vec3)>,
}

impl GpuFluidStepper {
    /// 建后端。`packet` 必须已按**全量**粒子建好；`n_fluid`/`nb` 与包内一致，`substeps`
    /// 与门面侧 `FluidConfig::substeps` 同值。
    pub fn new(pk: Packet, pc: PacketCfg, substeps: usize, n_fluid: u32, nb: usize) -> Self {
        // `stage` 要在 `pk` 移进结构体**之前**建（结构体字面量按字段序求值）。
        let stage = ReactionStage::new(&pk);
        Self {
            pk,
            stage,
            pc,
            substeps: substeps.max(1),
            n_fluid,
            nb,
            reactions: Vec::new(),
        }
    }

    /// 后端持有的包（诊断/复核用）。
    pub fn packet(&self) -> &Packet {
        &self.pk
    }
}

impl FluidStepper for GpuFluidStepper {
    fn step(
        &mut self,
        dt_tick: f32,
        n_fluid: usize,
        pos: &[Vec3],
        vel: &[Vec3],
        pmass: &[f32],
        spans: &[(u32, Vec3, u32, u32)],
    ) {
        // 卡上的 dt 是 `run` 内定的 `(1/60)/substeps` ⇒ 门面的 tick 步长必须同值（否则静默漂移）。
        assert!(
            (dt_tick - 1.0 / 60.0).abs() < 1e-9,
            "卡上步进要求 tick = 1/60 s（实得 {dt_tick}）——`FluidConfig`/`PhysConfig` 的 dt 要同值"
        );
        assert_eq!(
            n_fluid, self.n_fluid as usize,
            "流体前缀长度与后端不符（{n_fluid} ≠ {}）——粒子集变了要重建 Packet",
            self.n_fluid
        );
        assert_eq!(
            pos.len(),
            n_fluid + self.nb,
            "全粒子数与后端不符（{} ≠ {n_fluid}+{}）——要重建 Packet",
            pos.len(),
            self.nb
        );
        assert_eq!(pmass.len(), pos.len(), "pmass 长度与全粒子数不符");
        self.pk
            .upload_boundary_segment(n_fluid as u32, &pos[n_fluid..], &vel[n_fluid..]);
        self.pk.run(&self.pc, 1, self.substeps, false);
        self.reactions.clear();
        self.reactions.extend(self.stage.aggregate(&self.pk, spans));
    }

    fn reactions(&self) -> &[(u32, Vec3, Vec3)] {
        &self.reactions
    }

    fn bounds(&self) -> Option<(Vec3, Vec3)> {
        self.pk.read_box().map(|(lo, hi)| {
            (
                Vec3::new(lo[0], lo[1], lo[2]),
                Vec3::new(hi[0], hi[1], hi[2]),
            )
        })
    }
}
