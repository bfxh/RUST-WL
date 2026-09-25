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

/// 卡上步进后端：`Packet` + 聚合阶段 + 最近一次反作用（门面按同口径读）；
/// `walls` = 可选的**壁面档**（`wall_ghost` 镜像 + `wall_project` 投影，宿主每 tick 喂接触表）。
pub struct GpuFluidStepper {
    pk: Packet,
    stage: ReactionStage,
    walls: Option<WallStage>,
    /// 壁面 provider id 表（`gather_walls` 用；空 = 纯 2b 场景 ⇒ 不开销）。
    boundaries: Vec<u32>,
    pc: PacketCfg,
    substeps: usize,
    n_fluid: u32,
    nb: usize,
    reactions: Vec<(u32, Vec3, Vec3)>,
}

impl GpuFluidStepper {
    /// 建后端。`packet` 必须已按**全量**粒子建好；`n_fluid`/`nb` 与包内一致，`substeps`
    /// 与门面侧 `FluidConfig::substeps` 同值。`walls` 只在用 provider 壁面时给
    /// （用 `Packet::new_with_walls` 拿到的那个阶段），`boundaries` 给对应的 provider id 表。
    pub fn new(
        pk: Packet,
        walls: Option<WallStage>,
        boundaries: Vec<u32>,
        pc: PacketCfg,
        substeps: usize,
        n_fluid: u32,
        nb: usize,
    ) -> Self {
        // `stage` 要在 `pk` 移进结构体**之前**建（结构体字面量按字段序求值）。
        let stage = ReactionStage::new(&pk);
        Self {
            pk,
            stage,
            walls,
            boundaries,
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
        self.pk.run_stages_with(
            &self.pc,
            0b111_1111,
            1,
            self.substeps,
            false,
            self.walls.as_ref(),
            // 门面默认同步（管线档由调用方按需走 `run_deferred`；见 `PLAN-gpu` §19.5）。
            false,
        );
        self.reactions.clear();
        self.reactions.extend(self.stage.aggregate(&self.pk, spans));
    }

    fn reactions(&self) -> &[(u32, Vec3, Vec3)] {
        &self.reactions
    }

    /// **按 provider 收集壁面接触**并喂进壁面档：**从卡上读位置**（主机那份脚手架是陈的 ⇒ 位置必须以
    /// 卡上为准，这是 facade 路径第一版踩的坑：差 0.177 m）；空 `boundaries` ⇒ 直接返回（纯 2b 零开销）。
    fn gather_walls(&mut self, h: f32, providers: &dyn vxl_phys_core::interop::ProviderColliders) {
        if self.boundaries.is_empty() {
            return;
        }
        let Some(w) = self.walls.as_mut() else {
            return;
        };
        let (gp, _) = self.pk.read_state();
        let nf = self.n_fluid as usize;
        let ps: Vec<Vec3> = (0..nf)
            .map(|k| Vec3::new(gp[k * 3], gp[k * 3 + 1], gp[k * 3 + 2]))
            .collect();
        // **两张表**（§15 补记五/六）：镜像 ← 普通口径（与 CPU `wall_planes_in` 同款）、
        // 投影 ← 边界口径（穿透鲁棒）。两份各查一次、各上一次（代价 = 每 tick 两次查询 + 两次上传）。
        let (ids, start, planes) = gather_wall_contacts(&self.boundaries, h, &ps, providers);
        w.upload(&self.pk, WallSide::Mirror, &ids, &start, &planes);
        let (ids, start, planes) =
            gather_wall_project_contacts(&self.boundaries, h, &ps, providers);
        w.upload(&self.pk, WallSide::Project, &ids, &start, &planes);
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
