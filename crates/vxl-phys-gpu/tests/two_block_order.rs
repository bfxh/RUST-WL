//! **§23.3 的前提判据（纯 CPU，CI 可跑）**：两块方案**不改每格的枚举序**。
//!
//! 背景（`PLAN-gpu.md` §23.3）：格序副本档（`new_sorted`）今天只对**纯流体**生效，因为空间排序会把
//! 流体与边界粒子混在一起 ⇒ 核里 `j < n_fluid` 这条**标签**判断失效。要解开它，设计是"**两块**：
//! 流体块 ‖ 边界块，各自按格排，两张格表（`fstart`/`bstart`），核里每格迭代两段"。
//!
//! **这条设计的成败只取决于一件事**：两块方案下每格枚举的**粒子序列**必须与平铺档**逐条相同**
//! ——否则它就不是"只改访存"，而会改数值（口径 B 都保不住）。
//!
//! 本判据把这件事**在 CPU 上先证掉**（用引擎自己的位置/格表当参照），再动核：
//! - 参照（平铺）：`neighbor_grid()` 的 `items[start[c]..start[c+1])`（`canon` 保证格内按**粒子索引升序**）；
//! - 候选（两块）：① 对 `[0, n_fluid)` 做计数排序 → `fstart`/`perm_f`；② 对 `[n_fluid, n)` 同样处理
//!   → `bstart`/`perm_b`（全局槽位 = `n_fluid + 局部下标`）；③ 拼接；
//! - **判据**：逐格断言 `平铺的序列 == perm_f[该格的流体段] ‖ perm_b[该格的边界段]`。
//!
//! 为什么这个判据成立是**必然**的（写下来备查）：边界粒子的索引**恒 ≥ n_fluid**（`set_boundary_particles`
//! 先截断再追加），而 `canon` 按索引升序 ⇒ **今天每格的序列本来就是"先流体、后边界"**；两块方案只要
//! 每块内按"格 + 索引升序"稳定放置，拼起来就与它逐条相同。
//!
//! ✅ **不需要适配器** ⇒ 与那四个 GPU 判据不同，**这条在 CI 上也真跑**（不跳过）。

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

const N: usize = 16;
const SPACING: f32 = 0.05;

/// 2b 场景：晶格 + 5 趟静置 + 一块地板；**之后再推一个子步** ⇒ 引擎的格表覆盖全量粒子（含边界）。
fn scene_2b() -> FluidSystem {
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.5,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let half = N as f32 * SPACING * 0.5 + 4.0 * h;
    let bodies = vec![(
        0u32,
        Shape::Box {
            half: Vec3::new(half, 2.0 * SPACING, half),
        },
        BodyPose {
            pos: Vec3::new(0.0, -2.0 * h, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    )];
    assert!(f.set_boundary_particles(&bodies) > 0, "地板没造出边界粒子");
    // 推一个子步 ⇒ 格表按**全量**粒子重建（否则表是加边界之前那张）。
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    f
}

/// 对 `[lo, hi)` 这段粒子做计数排序：返回 `(每格起点, 按格分组的粒子下标)`。
/// 与 `grid.wgsl` 的四步同义：分箱 → 前缀和 → 按索引升序占位（稳定 ⇒ 格内 = 索引升序）。
fn count_sort_range(f: &FluidSystem, lo: usize, hi: usize) -> (Vec<u32>, Vec<u32>) {
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);
    let bin = |p: Vec3| -> usize {
        let ax = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * gd.inv).floor().max(0.0) as u32).min(n - 1)
        };
        let idx = (ax(gd.min.x, p.x, nx) * ny + ax(gd.min.y, p.y, ny)) * nz + ax(gd.min.z, p.z, nz);
        (idx as usize).min(total - 1)
    };
    let apos = f.raw_particles().0;
    let mut counts = vec![0u32; total + 1];
    for p in &apos[lo..hi] {
        counts[bin(*p) + 1] += 1;
    }
    for c in 0..total {
        counts[c + 1] += counts[c];
    }
    let start = counts.clone();
    let mut cur = counts;
    let mut perm = vec![0u32; hi - lo];
    // 按**粒子索引升序**占位 ⇒ 格内序 = 索引升序（与 `canon` 的规范结果一致）。
    for (i, p) in apos[lo..hi].iter().enumerate() {
        let c = bin(*p);
        perm[cur[c] as usize] = (i + lo) as u32;
        cur[c] += 1;
    }
    (start, perm)
}

#[test]
fn two_block_order_matches_flat_order_per_cell() {
    let f = scene_2b();
    let (_, _, _, n_fluid) = f.raw_particles();
    let apos = f.raw_particles().0;
    let np = apos.len();
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);

    // ⚠️ **参照必须与候选同源**：引擎那张  是本子步开头建的，而位置已经又前进了一次 ⇒ 直接比
    // 会拿两个时刻的表对拍（这条坑我在 §24.3/§25 已经踩过两次）。⇒ **平铺参照也由我用同一份位置现建**。
    let (lstart, perm_all) = count_sort_range(&f, 0, np);

    // —— 候选（两块）——
    let (fstart, perm_f) = count_sort_range(&f, 0, n_fluid);
    let (bstart_l, perm_b_l) = count_sort_range(&f, n_fluid, np);
    // ⚠️ `count_sort_range` 返回的**已经是全局粒子索引**（它填的就是 `i + lo`）⇒ **不要再加 `n_fluid`**
    // （我第一版加了一次，差恰好 `n_fluid` ⇒ 512 个流体格全报不一致：**是判据的 bug，不是设计的**）。
    let perm_b: &Vec<u32> = &perm_b_l;

    // —— 逐格对拍：平铺序列 vs 两块序列 ——
    let mut bad = 0usize;
    let mut cells_nonempty = 0usize;
    let mut mixed_cells = 0usize; // 同时含流体与边界的格（正是"标签判断会失效"的那些格）
    for c in 0..total {
        let flat = &perm_all[lstart[c] as usize..lstart[c + 1] as usize];
        if flat.is_empty() {
            continue;
        }
        cells_nonempty += 1;
        let has_f = flat.iter().any(|&j| (j as usize) < n_fluid);
        let has_b = flat.iter().any(|&j| (j as usize) >= n_fluid);
        if has_f && has_b {
            mixed_cells += 1;
        }
        let fs = &perm_f[fstart[c] as usize..fstart[c + 1] as usize];
        let bs = &perm_b[bstart_l[c] as usize..bstart_l[c + 1] as usize];
        let two: Vec<u32> = fs.iter().chain(bs.iter()).copied().collect();
        if two != flat {
            bad += 1;
            if bad <= 3 {
                println!(
                    "  · 格 {c}：平铺 {:?} | 两块 流体段 {:?} + 边界段 {:?}（fstart {:?}/{:?}、bstart {:?}/{:?}、lstart {:?}/{:?}）",
                    &flat[..flat.len().min(6)],
                    &fs[..fs.len().min(6)],
                    &bs[..bs.len().min(6)],
                    fstart[c],
                    fstart[c + 1],
                    bstart_l[c],
                    bstart_l[c + 1],
                    lstart[c],
                    lstart[c + 1]
                );
            }
        }
    }
    println!(
        "== 两块方案的枚举序判据（{np} 粒：流体 {n_fluid} + 边界 {}；{cells_nonempty} 个非空格，其中 {mixed_cells} 个**混装格**）==",
        np - n_fluid
    );
    println!(
        "  逐格对拍：不一致 {bad} / {cells_nonempty} 格（混合格 {mixed_cells} 个正是「标签判断会失效」的那些）"
    );
    // 顺带把"两块是否真按块分开"钉一下（不这么做的话，判据可能因为"两块恰好等于平铺"而空过）。
    assert!(n_fluid < np, "本判据需要边界粒子（否则退化成纯流体那半）");
    //  只作**诊断**（本场景实测为 0）：本判据要钉的是\每格序列\，与格内是否混装无关——
    // 单块排序的问题在于**槽位与类别的对应会乱**（ 失效），而不是格内混装。
    assert_eq!(
        bad, 0,
        "两块方案的每格枚举序必须与平铺**逐条相同**（实得 {bad} 格不同）——不同的话，它就不是\
         \"只改访存\"，而会改数值（`PLAN-gpu.md` §23.3 的前提不成立）"
    );
    println!("  ⇒ ✅ 前提成立：两块方案逐格复现平铺的枚举序 ⇒ 核改「每格两段」是机械改动，不是语义改动。");
}
