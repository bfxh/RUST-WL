//! fluid_stepper：**卡上步进档**的门面侧（`PLAN-gpu.md` §13.7 选项 A）。
//!
//! 单独成档的理由（与 `pipeline/reaction.rs`、`pipeline/readback.rs` 同款）：接线代码集中一处，
//! `world_step.rs` 只留调用点——既有文件加行要配"函数变短"（尺寸门），新文件只判阈值。
//!
//! 时序（与 `World::fluid_pass` 一致）：宿主已按**当前**体姿态重建好边界段 ⇒ 后端推进一个 tick
//! ⇒ 门面在体子步里按 `reactions()` 施加 `(F, τ)`（量纲 = 力，与 CPU 档同口径）。

use super::*;

/// 一个流体槽：`(CPU 引擎, 边界 provider id 表, 可选的**卡上步进后端**)`。
///
/// 抽成别名是**尺寸门 + clippy 两条约束的交点**：裸写这个三元组会撞 `clippy::type_complexity`
/// （`-D warnings` 判红），而 `world_struct.rs` 是零函数的档（加行即红）⇒ 类型必须能写在一行内，
/// 且要有个**可以在别处按路径引用**的名字。别名定义在新文件里（新文件只判阈值）。
pub(crate) type FluidSlot = (
    vxl_phys_fluid::FluidSystem,
    Vec<u32>,
    Option<Box<dyn vxl_phys_core::interop::FluidStepper>>,
);

/// 反作用来源：卡上步进档读后端最近一次的聚合，否则读本机流体（两者同口径）。
///
/// 自由函数而非 `&self` 方法：调用点要**同时**改 `self.bodies`（施加反作用）⇒ 方法会把整个
/// `&self` 借走而与 `self.bodies` 的可变借用冲突；自由函数只借 `self.fluids[fi]` 这一个元素。
pub(super) fn reactions_of(slot: &FluidSlot) -> &[(u32, Vec3, Vec3)] {
    match &slot.2 {
        Some(st) => st.reactions(),
        None => slot.0.boundary_reactions(),
    }
}

/// 近域过滤用的盒子：卡上步进档优先用后端报的（卡上每子步算好的那个，见 §12.4）；
/// `None` 则由调用方退回主机侧粒子范围（可能偏旧 ⇒ 只影响"多造/少造边界粒子"）。
pub(super) fn bounds_of(slot: &FluidSlot) -> Option<(Vec3, Vec3)> {
    slot.2.as_ref().and_then(|st| st.bounds())
}

impl World {
    /// **注册卡上步进后端**（§13.7 选项 A）：把某流体的"推进一个 tick + 每体反作用聚合"整段交给
    /// `FluidStepper`（如 GPU 档）。主机侧此后**只**做两件事：按近域体重建边界粒子
    /// （`refresh_fluid_boundary`）、拿后端的 AABB 做近域过滤；2a 介质采样对该流体**自动让位**
    /// （主机状态不再推进）。返回是否注册成功（索引越界 = false）。
    ///
    /// **逐位不变性**：未注册后端的流体一行都不走新路径（与 2b 当初落地同款显式档）。
    pub fn set_fluid_stepper(
        &mut self,
        fi: usize,
        st: Box<dyn vxl_phys_core::interop::FluidStepper>,
    ) -> bool {
        match self.fluids.get_mut(fi) {
            Some(slot) => {
                slot.2 = Some(st);
                true
            }
            None => false,
        }
    }

    /// **卡上步进一趟**（`fluid_pass` 的分支体）：把重建好的边界段交给后端；主机**不**推进自己的
    /// 流体状态（那份只当边界重建的脚手架）。返回 `true` = 本流体走了卡上路径。
    ///
    /// `take` 出来再放回：`step` 要读本流体系统的粒子切片（不可变借用），而 `st` 是同一元组的
    /// 另一槽 ⇒ 直接 `as_mut()` 会同时借同一元素的两处；`Option::take` 把 `st` 挪成局部变量，
    /// 借用关系就干净了（也不额外复制粒子数组）。
    pub(crate) fn fluid_stepper_pass(&mut self, fi: usize) -> bool {
        let dt = self.config.dt;
        let Some(mut st) = self.fluids[fi].2.take() else {
            return false;
        };
        // **壁面**：让后端**自己**去收集（它从卡上读位置 ⇒ 主机那份脚手架是陈的，不能拿它收集）。
        // 空 `boundaries`（纯 2b 场景）时后端直接返回 ⇒ 零开销。
        st.gather_walls(self.fluids[fi].0.config().smoothing_radius, &self.providers);
        let (pos, vel, pmass, nf) = self.fluids[fi].0.raw_particles();
        let spans = self.fluids[fi].0.boundary_spans();
        st.step(dt, nf, pos, vel, pmass, spans);
        self.fluids[fi].2 = Some(st);
        true
    }
}
