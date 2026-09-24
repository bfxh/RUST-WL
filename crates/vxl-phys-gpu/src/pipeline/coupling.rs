//! coupling：耦合回路的**写侧**（每 tick 边界段重建后的上传）。
//!
//! 三条腿各占一档：回读在 `readback.rs`、聚合在 `reaction.rs`，而"把宿主每 tick 按体姿态
//! 重建的边界粒子写回卡上"是第三条。它**只写后缀**：流体状态（`0..n_fluid`）活在卡上，
//! 每 tick 回读再写回等于白送一次往返（§12 的账：那条路占整 tick 的 88%）。

use super::*;

use vxl_phys_core::Vec3;

impl Packet {
    /// **上传边界段**（`n_fluid..n` 的 `pos` / `vel`）：耦合回路每 tick 重建边界粒子后调用。
    ///
    /// - `pos`/`vel` 只给**边界粒子那一段**（长度 = `n − n_fluid`，序 = `raw_particles()` 的
    ///   `[n_fluid..]`）。长度不符 ⇒ **直接报错**：写半段会让一部分边界粒子停在上一 tick 的姿态
    ///   （物理静默错），比 panic 难查得多。
    /// - **逐粒质量 `pmass` 不在这里传**：形状与晶格间距不变时它是常量（`ρ0·V_b`）；
    ///   换形状/换间距要新建 `Packet`（缓冲按粒子数一次性分配）。
    /// - 段表（体原点随体姿态变）不在这里传：`ReactionStage::aggregate` 每次调用都会上传它。
    pub fn upload_boundary_segment(&self, n_fluid: u32, pos: &[Vec3], vel: &[Vec3]) {
        let nb = (self.n - n_fluid) as usize;
        assert_eq!(
            pos.len(),
            nb,
            "边界段长度与 Packet 不符（pos {} ≠ {nb}）",
            pos.len()
        );
        assert_eq!(
            vel.len(),
            nb,
            "边界段长度与 Packet 不符（vel {} ≠ {nb}）",
            vel.len()
        );
        if nb == 0 {
            return;
        }
        let mut pb: Vec<u8> = Vec::with_capacity(nb * 12);
        let mut vb: Vec<u8> = Vec::with_capacity(nb * 12);
        for k in 0..nb {
            for c in [pos[k].x, pos[k].y, pos[k].z] {
                pb.extend_from_slice(&c.to_le_bytes());
            }
            for c in [vel[k].x, vel[k].y, vel[k].z] {
                vb.extend_from_slice(&c.to_le_bytes());
            }
        }
        let off = (n_fluid as u64) * 12;
        self.queue.write_buffer(&self.pos_b, off, &pb);
        self.queue.write_buffer(&self.vel_b, off, &vb);
    }
}
