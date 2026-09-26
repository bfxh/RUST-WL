//! 两件小工具（都为了**不动默认档语义**地修 §25 那条根因 / 压 `pipeline.rs` 的行数）。
//!
//! ## 1. `covering_box`：**盒按全量粒子现算**（`PLAN-gpu.md` §25.2 的修法）
//!
//! 根因（§25）：`PacketCfg` 的 `gmin/inv/dims/total` 是**调用方抄引擎那张表**的，而那张表可能是
//! `set_boundary_particles`/最后一次 `step()` **之前**建的 ⇒ 盒偏小 ⇒ 盒外粒子被 `axis_bin` 钳进边缘格
//! ⇒ 那几个格爆满（> `cap`）⇒ `canon` 跳过规范化 ⇒ 格内序留在 `place` 的原子占位序 ⇒ **求和序随运行变**
//! ⇒ 2b 路径 run-to-run 不可复现（实测 `items` 5/6 遍不同、`overflow` 4）。
//!
//! 修法：**盒只取决于"实际有哪些粒子"**，所以在这里按**上传的全量 `pos_flat`** 现算
//! （`grid_box` 是箱子规则的单一来源，CPU 侧每子步也调它）⇒ 与 CPU 同一条规则、且必然覆盖全部粒子。
//!
//! ⚠️ **这是换代级改动**（它改的是物理：盒外粒子原先的邻域搜索是错的），所以：默认档读数会变、
//! 必须显式拍板并重冻判据（§25.2/§25.3 记了账）。
//!
//! ## 2. `spec`：把布局规格写成**紧凑串**（"u r _ w" ⇒ 槽位=位置，`_` = 不声明）
//!
//! 只为让 `make_pipelines` 变短（god 门是棘轮：本文件的改动让 `pipeline.rs` 长了几行 ⇒ 必须让它的
//! **最长函数严格变短**）。规格串与原逐行写法**逐项等价**：写错槽位会在建管线时**大声报错**
//! （wgpu 要求 storage 访问权限精确匹配），不会静默错。

use super::*;

/// 按**全量粒子**重算格盒，覆盖 `cfg` 里那三件套（`PLAN-gpu.md` §25.2）。
/// `pos_flat` 必须是**全部**粒子（含 2b 边界粒子）的扁平位置。
pub(crate) fn covering_box(cfg: PacketCfg, pos_flat: &[f32]) -> PacketCfg {
    let mut lo = vxl_phys_core::Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let mut hi = vxl_phys_core::Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    for p in pos_flat.as_chunks::<3>().0 {
        let v = vxl_phys_core::Vec3::new(p[0], p[1], p[2]);
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if !(lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z) {
        return cfg; // 空输入：保持原样（建包会在别处报"没有粒子"）
    }
    let (gmin, bin, dims) =
        vxl_phys_core::grid::grid_box(lo, hi, cfg.h, vxl_phys_core::grid::GRID_MAX_BINS);
    PacketCfg {
        gmin: [gmin.x, gmin.y, gmin.z],
        inv: 1.0 / bin,
        dims,
        total: dims[0] * dims[1] * dims[2],
        ..cfg
    }
}

/// 紧凑布局规格：`'u'` = uniform、`'r'` = 只读 storage、`'w'` = 读写 storage、`'_'`/空格 = 不声明。
/// 槽位 = 字符位置（从 0 数）；末尾未写的槽位一律不声明。
pub(crate) fn spec(s: &str) -> Vec<(u32, Kind)> {
    let mut out = Vec::new();
    for (i, c) in s.chars().enumerate() {
        let k = match c {
            'u' => Kind::Uniform,
            'r' => Kind::Ro,
            'w' => Kind::Rw,
            _ => continue,
        };
        out.push((i as u32, k));
    }
    out
}
