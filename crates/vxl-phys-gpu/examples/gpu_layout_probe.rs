//! **排布判别探针** —— 回答一个问题：两个邻域相位（密度 + 力）的代价，有多少来自
//! **「槽位 → 空间位置」的映射**（访存局部性），有多少是每粒的固有工作量？
//!
//! 为什么需要它：`PLAN-gpu.md` §20 把 10M 档的 209.75 ms 解成「52% 每粒固定 + 48% 候选收集」，
//! 然后打算用「workgroup 按格分块 + 共享内存」去砍那 48%。**那个前提要验**：粒子数组是
//! **索引序**（`grid.wgsl` 的 `items[slot] = i` 是置换，`pos` 不按格重排）⇒ 一个 workgroup 的
//! 64 个线程在空间上是散的，**它们不共享邻域**。本探针用**零内核改动**的方式把这件事量出来。
//!
//! **做法（同位置、同工作量、只改映射）**：把同一批位置重新分配到"槽位"上——
//! - `natural`：槽位 k ← 第 k 个粒子（**今天的数据路径**，`cell_items` 是任意置换）；
//! - `sorted`：槽位 k ← 格序第 k 个粒子（**按格重排的理想档** ⇒ "把数据按格重排"的上限）；
//! - `random`：槽位 k ← 随机置换（**完全没有局部性**的下界）。
//!
//! 三档的**位置集合完全相同** ⇒ 每粒候选量与算术同量（用「候选对数」自检守住），差别只剩
//! "哪些线程在读哪些地址" ⇒ 计时差就是局部性的纯效应，且能顺带回答"探针场景的读数有没有
//! 被初始晶格序美化"（`natural` 与 `random` 的差就是这个红利）。
//!
//! ⚠️ **本探针不实现"按格重排"的生产路径**，只量它的**上限**；也不含网格重建与积分
//! （只跑两相位核，见 `probe::phases_on_adapter`）。读数用**两档 repeats 解 `t = c + R/r`**：
//! `R` 是那次同步回读（dens+acc+xsph = 28 B/粒，10M 档 ≈180 ms），它**与排布无关**，所以要解出来
//! 而不是让它污染 `c`（r=16 时会把 c 虚高 ≈11 ms）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_layout_probe -- [n] [--adapter K]
//!        [--cpu-threads K] [--rounds K]`

use vxl_phys_core::grid::{grid_box, GRID_MAX_BINS};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem, NeighborGrid};
use vxl_phys_gpu::probe::{self, PhaseInputs, PhaseParams};

/// 箱子三件套（`min`/`inv`/`dims`）——与管线**同一时刻**的那一套（见 `main` 里的说明）。
#[derive(Clone, Copy)]
struct Box3 {
    min: Vec3,
    inv: f32,
    dims: (u32, u32, u32),
}

impl Box3 {
    fn total(&self) -> usize {
        self.dims.0 as usize * self.dims.1 as usize * self.dims.2 as usize
    }
    fn split(&self, c: u32) -> (i32, i32, i32) {
        let z = (c % self.dims.2) as i32;
        let y = ((c / self.dims.2) % self.dims.1) as i32;
        let x = (c / (self.dims.2 * self.dims.1)) as i32;
        (x, y, z)
    }
    fn cell_of(&self, x: i32, y: i32, z: i32) -> usize {
        ((x * self.dims.1 as i32 + y) * self.dims.2 as i32 + z) as usize
    }
}

/// 三种「槽位 → 空间位置」映射。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Layout {
    /// 槽位 k ← 原粒子 k（今天的路径：`cell_items` 是任意置换）。
    Natural,
    /// 槽位 k ← 格序第 k 个粒子（按格重排的理想档）。
    Sorted,
    /// 槽位 k ← 随机置换（无局部性下界）。
    Random,
}

impl Layout {
    fn name(self) -> &'static str {
        match self {
            Layout::Natural => "natural",
            Layout::Sorted => "sorted",
            Layout::Random => "random",
        }
    }
    fn all() -> [Layout; 3] {
        [Layout::Natural, Layout::Sorted, Layout::Random]
    }
}

/// 一份排布：槽位化的输入数组 + 重建的网格表 + 解释用诊断量。
struct Arrangement {
    pos_flat: Vec<f32>,
    vel_flat: Vec<f32>,
    press: Vec<f32>,
    cell_start: Vec<u32>,
    cell_items: Vec<u32>,
    /// 逐槽位的格下标（自检用）。
    bins: Vec<u32>,
    /// 采样：连续 32 槽的邻域并集 / 32（1.0 = 整 warp 共享同一片格 ⇒ 理想局部性）。
    nbr_per_warp: f64,
    /// 采样：槽 k 与 k+1 的空间距离均值（米）。
    step_dist_m: f64,
    /// 候选对数（三档必须相同 ⇒ "工作量相同"这条前提被守住）。
    pairs: u64,
    /// 跑出来的一份 dens（跨档对拍：排布不该改物理）。
    dens: Vec<f32>,
}

/// 与 `vxl-phys-fluid/src/grid.rs::bin_index_of` **同式**（格下标 + 越界钳位）。
fn bin_of(b: &Box3, p: Vec3, total: usize) -> u32 {
    let f =
        |o: f32, v: f32, n: u32| -> u32 { (((v - o) * b.inv).floor().max(0.0) as u32).min(n - 1) };
    let idx = (f(b.min.x, p.x, b.dims.0) * b.dims.1 + f(b.min.y, p.y, b.dims.1)) * b.dims.2
        + f(b.min.z, p.z, b.dims.2);
    (idx as usize).min(total - 1) as u32
}

/// 计数排序（与 `UniformGrid::rebuild` 同式）：逐槽位分箱 → 前缀和 → 占位。
fn count_sort(bins: &[u32], total: usize) -> (Vec<u32>, Vec<u32>) {
    let mut counts = vec![0u32; total + 1];
    for &c in bins {
        counts[c as usize + 1] += 1;
    }
    for c in 0..total {
        counts[c + 1] += counts[c];
    }
    let start = counts.clone();
    let mut cur = counts;
    let mut items = vec![0u32; bins.len()];
    for (k, &c) in bins.iter().enumerate() {
        let slot = cur[c as usize] as usize;
        items[slot] = k as u32;
        cur[c as usize] += 1;
    }
    (start, items)
}

/// 逐槽位的「27 格邻域里的候选数」之和（减自身）⇒ 全局候选对数。
fn candidate_pairs(b: &Box3, bins: &[u32], start: &[u32]) -> u64 {
    let (nx, ny, nz) = (b.dims.0 as i32, b.dims.1 as i32, b.dims.2 as i32);
    let mut pairs = 0u64;
    for &bi in bins {
        let (x, y, z) = b.split(bi);
        let mut s = 0u64;
        for dz in -1..=1i32 {
            if z + dz < 0 || z + dz >= nz {
                continue;
            }
            for dy in -1..=1i32 {
                if y + dy < 0 || y + dy >= ny {
                    continue;
                }
                for dx in -1..=1i32 {
                    if x + dx < 0 || x + dx >= nx {
                        continue;
                    }
                    let ci = b.cell_of(x + dx, y + dy, z + dz);
                    s += (start[ci + 1] - start[ci]) as u64;
                }
            }
        }
        pairs += s.saturating_sub(1);
    }
    pairs
}

/// 把 27 格 stencil 的格下标压进 `buf`（采样用）。
fn stencil_into(b: &Box3, c: u32, buf: &mut Vec<u32>) {
    let (nx, ny, nz) = (b.dims.0 as i32, b.dims.1 as i32, b.dims.2 as i32);
    let (x, y, z) = b.split(c);
    for dz in -1..=1i32 {
        if z + dz < 0 || z + dz >= nz {
            continue;
        }
        for dy in -1..=1i32 {
            if y + dy < 0 || y + dy >= ny {
                continue;
            }
            for dx in -1..=1i32 {
                if x + dx < 0 || x + dx >= nx {
                    continue;
                }
                buf.push(b.cell_of(x + dx, y + dy, z + dz) as u32);
            }
        }
    }
}

/// 采样两条解释性诊断：warp 内邻域并集（格数/线程）、相邻槽的空间步距。
fn locality_diag(b: &Box3, pos: &[Vec3], bins: &[u32]) -> (f64, f64) {
    let np = bins.len();
    let mut buf: Vec<u32> = Vec::with_capacity(32 * 27);
    let groups = 2000usize.min(np / 32).max(1);
    let stride = (np / 32 / groups).max(1);
    let mut sum_cells = 0usize;
    let mut used = 0usize;
    for g in 0..groups {
        let k0 = g * stride * 32;
        if k0 + 32 > np {
            break;
        }
        buf.clear();
        for t in 0..32 {
            stencil_into(b, bins[k0 + t], &mut buf);
        }
        buf.sort_unstable();
        buf.dedup();
        sum_cells += buf.len();
        used += 1;
    }
    let nbr_per_warp = sum_cells as f64 / (used.max(1) * 32) as f64;
    // 相邻槽步距（抽样，避免大档 O(n) 的整段扫描）
    let step = (np / 200_000).max(1);
    let mut acc = 0.0f64;
    let mut m = 0usize;
    let mut k = 0usize;
    while k + 1 < np {
        acc += (pos[k + 1] - pos[k]).length() as f64;
        m += 1;
        k += step;
    }
    (nbr_per_warp, acc / m.max(1) as f64)
}

/// 建一档排布：按 `perm` 把位置/速度/压力搬到槽位，再重建网格表。
fn build_arrangement(
    b: &Box3,
    layout: Layout,
    gd: &NeighborGrid<'_>,
    f: &FluidSystem,
) -> Arrangement {
    let pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    let total = b.total();
    // 槽位 k ← 原粒子 perm[k]
    let perm: Vec<u32> = match layout {
        Layout::Natural => (0..np as u32).collect(),
        // 格序：槽位 k 放"引擎表里第 k 个"粒子 ⇒ 槽位序 = 空间序。
        Layout::Sorted => gd.items.to_vec(),
        Layout::Random => {
            // 固定种子的 xorshift 洗牌（可复现；不引外部 crate）。
            let mut s = 0x9E37_79B9_7F4A_7C15u64;
            let mut v: Vec<u32> = (0..np as u32).collect();
            for i in (1..np).rev() {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                let j = (s % (i as u64 + 1)) as usize;
                v.swap(i, j);
            }
            v
        }
    };
    let vel: Vec<Vec3> = f.velocities().to_vec();
    let press: Vec<f32> = f.pressures().to_vec();
    let mut slot_pos: Vec<Vec3> = Vec::with_capacity(np);
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut pr: Vec<f32> = Vec::with_capacity(np);
    for &p in &perm {
        let p = p as usize;
        slot_pos.push(pos[p]);
        pos_flat.extend_from_slice(&[pos[p].x, pos[p].y, pos[p].z]);
        vel_flat.extend_from_slice(&[vel[p].x, vel[p].y, vel[p].z]);
        pr.push(press[p]);
    }
    let bins: Vec<u32> = slot_pos.iter().map(|&p| bin_of(b, p, total)).collect();
    let (cell_start, cell_items) = count_sort(&bins, total);
    let pairs = candidate_pairs(b, &bins, &cell_start);
    let (nbr_per_warp, step_dist_m) = locality_diag(b, &slot_pos, &bins);
    Arrangement {
        pos_flat,
        vel_flat,
        press: pr,
        cell_start,
        cell_items,
        bins,
        nbr_per_warp,
        step_dist_m,
        pairs,
        dens: Vec::new(),
    }
}

/// **与时机无关**的 `bin_of` 判据：引擎那张表对应的是**上一个子步的位置**（`substep()` 开头建、
/// `step()` 末尾又积分了一次；`neighbor_grid()` 只导出缓存、不重建）⇒ 不能比"同格"，只能比
/// "引擎放在哪一格"与"我按当前位置算哪一格"**差几格**。返回 `(同格%, 差 ≤1 格%)`。
fn bin_agreement(stale: &[u32], gd: &NeighborGrid<'_>, np: usize) -> (f64, f64) {
    let (nx, ny, nz) = (gd.dims.0 as i32, gd.dims.1 as i32, gd.dims.2 as i32);
    let total = (nx * ny * nz) as usize;
    // 引擎表 → 每粒子的格下标
    let mut eng_cell = vec![u32::MAX; np];
    for c in 0..total {
        for k in gd.start[c]..gd.start[c + 1] {
            let p = gd.items[k as usize] as usize;
            if p < np {
                eng_cell[p] = c as u32;
            }
        }
    }
    let split = |c: u32| -> (i32, i32, i32) {
        let z = (c % gd.dims.2) as i32;
        let y = ((c / gd.dims.2) % gd.dims.1) as i32;
        let x = (c / (gd.dims.2 * gd.dims.1)) as i32;
        (x, y, z)
    };
    let mut exact = 0u64;
    let mut within1 = 0u64;
    let mut n = 0u64;
    for k in 0..np {
        let e = eng_cell[k];
        if e == u32::MAX {
            continue;
        }
        n += 1;
        let (ex, ey, ez) = split(e);
        let (mx, my, mz) = split(stale[k]);
        if (ex - mx).abs() <= 1 && (ey - my).abs() <= 1 && (ez - mz).abs() <= 1 {
            within1 += 1;
            if e == stale[k] {
                exact += 1;
            }
        }
    }
    let d = n.max(1) as f64;
    (100.0 * exact as f64 / d, 100.0 * within1 as f64 / d)
}

/// 越界槽位数：与 `bin_of` 的钳位同式但**去掉** `max(0)`/`min(n-1)`。
/// 非 0 ⇒ 有粒子在格盒外 ⇒ 27 格 stencil 不再覆盖全邻域 ⇒ 实验前提作废。
fn oob_count(b: &Box3, pos: &[Vec3]) -> u64 {
    let f = |o: f32, v: f32| -> f32 { ((v - o) * b.inv).floor() };
    pos.iter()
        .filter(|p| {
            f(b.min.x, p.x) < 0.0
                || f(b.min.x, p.x) >= b.dims.0 as f32
                || f(b.min.y, p.y) < 0.0
                || f(b.min.y, p.y) >= b.dims.1 as f32
                || f(b.min.z, p.z) < 0.0
                || f(b.min.z, p.z) >= b.dims.2 as f32
        })
        .count() as u64
}

/// 自检 ④：喂给核的那张表必须自洽——每格 `[start[c], start[c+1])` 里的槽位，其 `bins` 必须就是 `c`
/// （`count_sort` 的正向验证；表不自洽的话三档比的就不是"同一件事"）。
/// 顺带返回 `sorted` 档的 `items[m] == m` 命中率（那正是"理想档"的定义：槽位序 = 空间序）。
fn table_consistent(bins: &[u32], start: &[u32], items: &[u32], total: usize) -> (u64, u64) {
    let mut bad = 0u64;
    let mut identity = 0u64;
    for c in 0..total {
        for k in start[c]..start[c + 1] {
            let m = k as usize;
            if bins[items[m] as usize] != c as u32 {
                bad += 1;
            }
            if items[m] as usize == m {
                identity += 1;
            }
        }
    }
    (bad, identity)
}

/// **长跑下的局部性衰减**（回答"10M 档那个 226 ms/tick 能不能代表跑久了的场景"）：
/// 同一条场景往前推，在给定的 tick 检查点上重算两条诊断（`warp 邻域并集` 与 `相邻槽步距`）。
/// 索引↔空间的关联一旦衰减，warp 邻域并集就会从 ~5 往 27 爬 ⇒ 代价往 `random` 档那侧跑。
fn diag_curve(n: usize, cpu_threads: usize, checkpoints: &[usize]) {
    let mut f = build_scene(n, cpu_threads);
    let mut pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    println!("== 局部性随时间的衰减（{np} 粒；诊断口径：32 槽/组 × 2000 组采样）==");
    println!("  tick | warp 邻域并集（格/线程）| 相邻槽步距 m | 格边 m");
    let mut done = 5usize; // `build_scene` 已经推了 5 趟
    for &s in checkpoints {
        for _ in done..s {
            f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
        }
        done = done.max(s);
        pos = f.positions().to_vec();
        let (lo, hi) = pos
            .iter()
            .fold((pos[0], pos[0]), |(l, h), p| (l.min(*p), h.max(*p)));
        let (min, bin, dims) = grid_box(lo, hi, f.config().smoothing_radius, GRID_MAX_BINS);
        let b = Box3 {
            min,
            inv: 1.0 / bin,
            dims: (dims[0], dims[1], dims[2]),
        };
        let total = b.total();
        let bins: Vec<u32> = pos.iter().map(|&p| bin_of(&b, p, total)).collect();
        let (nbr, step_m) = locality_diag(&b, &pos, &bins);
        println!("  {s:>4} | {nbr:>24.2} | {step_m:>12.4} | {bin:.4}");
    }
    let _ = np;
}

/// 四条**前提判据**（任一不过就不许读数）：① 三档格表逐项同 `start`；② 候选对数三档相同；
/// ③ `bin_of` 与引擎同式（与时机无关的判据，见 `bin_agreement`）+ 越界 0；④ 表自洽 + `sorted`
/// 档必须是恒等置换（"槽位序 = 空间序"的定义）。
fn self_checks(
    arrs: &[(Layout, Arrangement)],
    gd: &NeighborGrid<'_>,
    pos: &[Vec3],
    b: &Box3,
    np: usize,
    total: usize,
) {
    let base_start = &arrs[0].1.cell_start;
    for (l, a) in arrs {
        assert_eq!(&a.cell_start, base_start, "{:?} 的 start 不同", l.name());
    }
    let p0 = arrs[0].1.pairs;
    for (l, a) in arrs {
        assert_eq!(a.pairs, p0, "{:?} 的候选对数不同", l.name());
    }
    let stale: Vec<u32> = pos.iter().map(|&p| bin_of(b, p, total)).collect();
    let (exact, within1) = bin_agreement(&stale, gd, np);
    let oob = oob_count(b, pos);
    println!(
        "  自检：三档 `start` 逐项同（{} 项）、候选对数同为 {p0}（{:.1}/粒）",
        base_start.len(),
        p0 as f64 / np as f64
    );
    println!(
        "        `bin_of` 对引擎（旧一子步的）表：同格 {exact:.2}%、差 ≤1 格 {within1:.4}%；越界槽位 {oob}"
    );
    assert_eq!(oob, 0, "有粒子落在格盒外 ⇒ 27 格 stencil 不再覆盖全邻域");
    assert!(within1 > 99.9, "bin_of 与引擎公式不一致（差 >1 格）");
    for (l, a) in arrs {
        let (bad, ident) = table_consistent(&a.bins, &a.cell_start, &a.cell_items, total);
        assert_eq!(bad, 0, "{:?} 的表不自洽：{bad} 项", l.name());
        if l.name() == Layout::Sorted.name() {
            assert_eq!(ident as usize, np, "sorted 档不是恒等置换（{ident}/{np}）");
        }
        println!(
            "        表的自洽性 {:<7}：错位 {bad} 项；`items[m] == m` 命中 {ident}/{np}",
            l.name()
        );
    }
}

/// 交错轮计时：每轮把三档都过一遍（两档 repeats），档内按 `ok` 取更保守的那次。
fn measure_all(
    arrs: &mut [(Layout, Arrangement)],
    params: PhaseParams,
    np: usize,
    r1: usize,
    r2: usize,
    rounds: usize,
) -> Vec<Meas> {
    let mut best: Vec<Option<Meas>> = vec![None; arrs.len()];
    for round in 0..rounds.max(1) {
        for (i, (l, a)) in arrs.iter_mut().enumerate() {
            let (p1, setup1) = run_once(a, params, np, r1);
            let (p2, _) = run_once(a, params, np, r2);
            let (c, r, ok) = solve(r1, p1, r2, p2);
            println!(
                "  · 轮 {} {:<6}：r={r1} → {p1:.2} | r={r2} → {p2:.2} ms ⇒ c = {c:.2} ms{}，R = {r:.1} ms（setup {setup1:.0} ms）",
                round + 1,
                l.name(),
                if ok { "" } else { "（R 不可解 †）" }
            );
            let cand = Meas { c, r, p1, p2, ok };
            best[i] = Some(match best[i] {
                // 可解档之间取**更小**的 `c`；不可解档（†）保留**更大**的那次当上界
                // （保守：别把不确定读成收益）。
                Some(prev) if prev.ok == ok && (prev.c <= c) == ok => prev,
                _ => cand,
            });
        }
    }
    best.into_iter().map(|b| b.unwrap()).collect()
}

/// 读数表 + `†` 脚注 + 标定提示。
fn report_table(arrs: &[(Layout, Arrangement)], best: &[Meas]) {
    println!("\n== 读数（密度+力两相位合计，每子步口径；`c` 已扣掉与排布无关的固定项 `R`）==");
    println!(
        "  档      | 每轮 c ms | 相对 natural | p1/p2 ms   | R ms  | warp 邻域并集 | 相邻槽步距 m"
    );
    let cn = best[0].c;
    for (i, (l, a)) in arrs.iter().enumerate() {
        let m = best[i];
        println!(
            "  {:<7} | {:>8.2}{} | {:>11.2}× | {:>6.1}/{:<6.1} | {:>5.1} | {:>13.2} | {:.4}",
            l.name(),
            m.c,
            if m.ok { " " } else { "†" },
            m.c / cn,
            m.p1,
            m.p2,
            m.r,
            a.nbr_per_warp,
            a.step_dist_m
        );
    }
    if best.iter().any(|m| !m.ok) {
        println!(
            "  † 该档的 `R` 解不出来（两档读数反过来 ⇒ 长跑降频）⇒ 表里给的是**两档原始值的较大者**，\n    即 c 的**上界**（保守侧）。"
        );
    }
}

/// 跑一次 `phases_on_adapter`，返回 `(per_dispatch_ms, setup_ms)`（= `c + R/r`）。
fn run_once(arr: &mut Arrangement, params: PhaseParams, np: usize, reps: usize) -> (f64, f32) {
    let pmass = vec![params.mass; np];
    let inputs = PhaseInputs {
        pos_flat: &arr.pos_flat,
        vel_flat: &arr.vel_flat,
        pmass: &pmass,
        press: &arr.press,
        cell_start: &arr.cell_start,
        cell_items: &arr.cell_items,
    };
    let out = probe::phases_on_adapter(0, &inputs, params, np, reps);
    if let Some(e) = out.error {
        panic!("GPU 路径不可用：{e}");
    }
    if arr.dens.is_empty() {
        arr.dens = out.dens.clone();
    }
    (out.per_dispatch_ms as f64, out.setup_ms)
}

/// 一档排布的一次测量结果。
#[derive(Clone, Copy)]
struct Meas {
    /// 每轮稳态（两相位合计，已扣固定项 `R`）；`ok == false` 时是**上界**。
    c: f64,
    /// 固定项（同步回读 + 首轮预热）；`ok == false` 时不可信。
    r: f64,
    p1: f64,
    p2: f64,
    /// `R` 是否解得出来（见 `solve` 的护栏）。
    ok: bool,
}

/// 两档 repeats 解 `t = c + R/r`：`c` = 每轮稳态（两相位合计），`R` = 固定项（同步回读 + 首轮预热）。
///
/// **护栏**：`R` 与排布无关，必须**为正**，且它在 `r2` 档读数里的占比要小（否则"解出 c"等于在噪声里
/// 外推）。`random` 档在 10M 会慢到 3.4 s/子步 ⇒ 一发跑满上百秒 ⇒ **降频/时钟漂移**，实测两档反过来
/// （p1 < p2 ⇒ 解出负 R）。此时**不报解**，改报"两档原始值的较大者"当上界（`ok = false`，表里带 †）。
fn solve(r1: usize, p1: f64, r2: usize, p2: f64) -> (f64, f64, bool) {
    let (a, b) = (r1 as f64, r2 as f64);
    let c = (p1 * a - p2 * b) / (a - b);
    let r = p1 * a - c * a;
    let ok = r > 0.0 && r < 0.5 * p2 * b;
    (if ok { c } else { p1.max(p2) }, r, ok)
}

/// 排布不该改物理：**分位对齐**比密度（槽位口径不同、求和序不同 ⇒ 只判"同分布"）。
fn physics_check(arrs: &[(Layout, Arrangement)]) {
    let mut s0 = arrs[0].1.dens.clone();
    s0.sort_unstable_by(|p, q| p.partial_cmp(q).unwrap());
    for (l, a) in arrs.iter().skip(1) {
        let mut s1 = a.dens.clone();
        s1.sort_unstable_by(|p, q| p.partial_cmp(q).unwrap());
        let mut rel = 0.0f32;
        for (x, y) in s0.iter().zip(s1.iter()) {
            rel = rel.max((x - y).abs() / x.abs().max(1e-3));
        }
        println!(
            "  物理对拍 {:<7}：分位对齐后密度最大相对差 {rel:.3e}（⇒ 工作量与物理一致）",
            l.name()
        );
    }
}

fn parse_args() -> (usize, usize, usize, Vec<String>) {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let rest: Vec<String> = args.collect();
    let get = |key: &str, dflt: usize| -> usize {
        rest.iter()
            .position(|a| a == key)
            .and_then(|k| rest.get(k + 1))
            .and_then(|v| v.parse().ok())
            .unwrap_or(dflt)
    };
    (n, get("--cpu-threads", 1), get("--rounds", 2), rest)
}

fn get(rest: &[String], key: &str, dflt: usize) -> usize {
    rest.iter()
        .position(|a| a == key)
        .and_then(|k| rest.get(k + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(dflt)
}

/// `--diag 5 60 300 600`：取 `--diag` 后面的整数串当检查点。
fn diag_checkpoints(rest: &[String]) -> Option<Vec<usize>> {
    let k = rest.iter().position(|a| a == "--diag")?;
    let v: Vec<usize> = rest[k + 1..]
        .iter()
        .take_while(|a| !a.starts_with("--"))
        .filter_map(|a| a.parse().ok())
        .collect();
    Some(if v.is_empty() {
        vec![5, 60, 300, 600]
    } else {
        v
    })
}

fn build_scene(n: usize, cpu_threads: usize) -> FluidSystem {
    let spacing = 0.05f32;
    let cfg = FluidConfig {
        threads: cpu_threads,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    // 与 `gpu_tick_probe` **同一套**：零重力 + 5 趟静置 + 剪切初速（零重力下完美晶格会全程静止）。
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    f
}

fn main() {
    let (n, cpu_threads, rounds, rest) = parse_args();
    // `--diag-only` 纯 CPU：只跑局部性衰减曲线（不碰 GPU、不需要独占锁）。
    let diag = diag_checkpoints(&rest);
    if rest.iter().any(|a| a == "--diag-only") {
        diag_curve(
            n,
            cpu_threads,
            &diag.unwrap_or_else(|| vec![5, 60, 300, 600]),
        );
        return;
    }
    let list = probe::adapters();
    if list.is_empty() {
        println!("（本机无可用适配器 ⇒ CI 下的正常路径；本探针不构成门禁）");
        return;
    }
    for (i, a) in list.iter().enumerate() {
        println!("  [{i}] {a}");
    }

    let f = build_scene(n, cpu_threads);
    let gd = f.neighbor_grid();
    let pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    // **箱子按当前位置现算**（与管线 `substep()` 同一条规则、同一时刻）——不能直接借引擎那份：
    // 它是上一个子步建的，粒子已外移 ⇒ 会有一批粒子落在盒外（实测 1.8%），27 格 stencil 就
    // 不再覆盖全邻域。顺带这也让 `natural` 档与管线的访问模式逐条对齐。
    let (lo, hi) = pos
        .iter()
        .fold((pos[0], pos[0]), |(l, h), p| (l.min(*p), h.max(*p)));
    let (min, bin, dims) = grid_box(lo, hi, f.config().smoothing_radius, GRID_MAX_BINS);
    let b = Box3 {
        min,
        inv: 1.0 / bin,
        dims: (dims[0], dims[1], dims[2]),
    };
    let total = b.total();
    println!(
        "== 排布判别（{np} 粒；箱 {dims:?} = {total} 格 ⇒ 格边 {bin:.4} m，h={}）==",
        f.config().smoothing_radius
    );

    let mut arrs: Vec<(Layout, Arrangement)> = Layout::all()
        .into_iter()
        .map(|l| (l, build_arrangement(&b, l, &gd, &f)))
        .collect();
    self_checks(&arrs, &gd, &pos, &b, np, total);

    // —— 计时：交错轮（两轮取最小），每档两档 repeats 解 `c` 与 `R` ——
    let h = f.config().smoothing_radius;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let g = f.config().gravity;
    let params = PhaseParams {
        gmin: [min.x, min.y, min.z],
        inv: b.inv,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        mass: f.particle_mass(),
        ks: 45.0 / (std::f32::consts::PI * h.powi(6)),
        h,
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        _pad0: 0.0,
        gvec: [g.x, g.y, g.z],
        n_fluid: np as u32,
        nx: dims[0],
        ny: dims[1],
        nz: dims[2],
        n_total: np as u32,
    };

    // 小档 repeats：`random` 档在 10M 会慢到 ~3.4 s/子步 ⇒ r 越大越久（一发 ≈ r×c）。
    // `(8, 32)` 的固定项占比已足够小，且一发只 3–110 s。
    let (r1, r2) = (get(&rest, "--r1", 8), get(&rest, "--r2", 32));
    let best = measure_all(&mut arrs, params, np, r1, r2, rounds);
    report_table(&arrs, &best);
    physics_check(&arrs);
    if let Some(cp) = diag {
        println!();
        diag_curve(n, cpu_threads, &cp);
    }
    if n == 215 {
        // **标定**：n=215 正是 10M 档（215³ = 9 938 375 粒）⇒ `natural` 档应当落在真实管线
        // 相位账的同一个数上（§18.2 抬预算后：密度 93.82 + 力段 115.93 = 209.75 ms/tick ÷ 4 子步）。
        // 对上了 ⇒ 本探针复现的就是管线的访问模式，读数可直接与 tick 读数换算。
        println!(
            "  标定：真实管线 10M 档相位账 (93.82 + 115.93)/4 子步 = 52.44 ms/子步 ⇒ natural 档应 ≈ 该值"
        );
    }
    println!(
        "  ⇒ 判读：`sorted` = 「把数据按格重排」的上限（含去掉 `cell_items` 间接层）；`random` ≈ 生产上\n     流动后索引序与空间解耦的档；`natural` 与 `random` 的差 = 探针场景自带的局部性红利。"
    );
}
