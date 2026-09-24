//! 均匀网格的**箱子规则**（单一来源）：CPU 流体网格与 GPU 常驻管线共用同一条
//! 「格边从 h 起、超预算则加倍粗化」的规则 ⇒ 两侧的 `(min, bin, dims)` 必须**逐位相同**，
//! 否则同一初态在两条链上会分箱不同、邻域不同 ⇒ 漂移表失去意义。

use crate::math::Vec3;

/// 单维格数上限（钳位；与原 `UniformGrid::rebuild` 里的 `1 << 14` 一致）。
pub const GRID_DIM_MAX: u32 = 1 << 14;

/// 总格数上限（预算；与原 `vxl_phys_fluid::config::GRID_MAX_BINS` 一致）。
/// **放在 core**：CPU 侧与 GPU 侧（每子步重算箱子的预算判据）必须用同一个数。
pub const GRID_MAX_BINS: usize = 1 << 20;

/// 由粒子包围盒 + 平滑长度算箱子三件套：`(min_corner, bin, dims)`，`bin` = **格边**。
///
/// 规则（唯一来源；CPU 侧 `UniformGrid::rebuild` 调它，GPU 侧同样调它）：
/// - 格边 `bin` 从 `h` 起（下限 `1e-6`）；
/// - `dims[a] = floor(ext[a] / bin) + 1`，逐维钳到 `[1, GRID_DIM_MAX]`；
/// - 总格数 `> max_bins` ⇒ `bin *= 2` 后重算（幂次翻倍 ⇒ `1.0 / bin` 逐位可复现）。
///
/// 返回的 `min_corner` = **粒子最小角本身**（不留边）：边缘粒子靠分箱的钳位落进边界格。
/// 钳位不破坏邻域正确性（钳进边缘格的粒子仍被 `r ≤ h` 的邻域判据刷掉），代价是边缘格变挤。
///
/// 返回 `bin` 而不是 `inv`：`1.0 / bin` 在调用方各算一次 ⇒ 与旧实现**逐位相同**
/// （若返回 `inv`，调用方再取倒数会引入一次舍入，可能差 1 ulp）。
pub fn grid_box(lo: Vec3, hi: Vec3, h: f32, max_bins: usize) -> (Vec3, f32, [u32; 3]) {
    let mut bin = h.max(1e-6);
    let ext = hi - lo;
    let exts = [ext.x, ext.y, ext.z];
    let mut dims = [1u32; 3];
    loop {
        for a in 0..3 {
            dims[a] =
                (((exts[a] / bin).floor() as usize + 1).clamp(1, GRID_DIM_MAX as usize)) as u32;
        }
        if dims[0] as usize * dims[1] as usize * dims[2] as usize <= max_bins {
            break;
        }
        bin *= 2.0;
    }
    (lo, bin, dims)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MB: usize = 1 << 20;

    #[test]
    fn single_particle_gets_one_cell() {
        let p = Vec3::new(1.0, 2.0, 3.0);
        let (min, bin, dims) = grid_box(p, p, 0.05, MB);
        assert_eq!(min, p);
        assert_eq!(bin, 0.05);
        assert_eq!(dims, [1, 1, 1], "ext=0 ⇒ 每维 1 格");
    }

    #[test]
    fn dims_are_floor_plus_one() {
        // ext = 0.25、bin = 0.05 ⇒ floor(5) + 1 = 6 格/维。
        let (_, bin, dims) = grid_box(Vec3::ZERO, Vec3::splat(0.25), 0.05, MB);
        assert_eq!(bin, 0.05);
        assert_eq!(dims, [6, 6, 6]);
    }

    #[test]
    fn oversize_budget_doubles_cell() {
        // ext = 5、h = 0.05、预算 10_000：dims 101³ → 51³ → 26³ 都超预算，bin 翻到 **0.4**
        // （5/0.4 = 12.5 ⇒ floor 12 ⇒ 13³ = 2197 ≤ 10_000 ✓）
        let (_, bin, dims) = grid_box(Vec3::ZERO, Vec3::splat(5.0), 0.05, 10_000);
        assert_eq!(dims, [13, 13, 13]);
        assert_eq!(bin, 0.4);
        assert!(dims[0] as usize * dims[1] as usize * dims[2] as usize <= 10_000);
    }

    #[test]
    fn per_dim_clamp_to_14_bits() {
        // 极大范围：单维钳到 GRID_DIM_MAX，而总格数远超预算 ⇒ 一直翻倍直到进预算。
        let (_, bin, dims) = grid_box(Vec3::ZERO, Vec3::splat(1.0e6), 0.05, MB);
        for (a, d) in dims.iter().enumerate() {
            assert!(
                *d <= GRID_DIM_MAX,
                "第 {a} 维必须钳在 {} 以内",
                GRID_DIM_MAX
            );
        }
        assert!(dims[0] as usize * dims[1] as usize * dims[2] as usize <= MB);
        assert!(bin > 0.05, "必然发生过翻倍");
    }

    #[test]
    fn tiny_h_is_floored() {
        // 退化点（ext = 0 ⇒ 每维 1 格）不触发翻倍 ⇒ 直接暴露 h 的下限。
        let p = Vec3::new(1.0, 2.0, 3.0);
        let (_, bin, dims) = grid_box(p, p, 0.0, MB);
        assert_eq!(bin, 1e-6, "h 的下限 1e-6（防 0 格边）");
        assert_eq!(dims, [1, 1, 1]);
        // 非退化时 h=0 会一路翻倍进预算（下限只保证"不比 1e-6 更细"）。
        let (_, bin2, _) = grid_box(Vec3::ZERO, Vec3::splat(1.0), 0.0, MB);
        assert!(bin2 > 1e-6 && bin2.is_finite(), "翻倍到进预算，bin={bin2}");
    }
}
