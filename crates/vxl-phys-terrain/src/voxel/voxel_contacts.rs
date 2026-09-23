//! voxel_contacts：从 voxel.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 盒的世界 AABB 半长：`|R|·half`（与宽相同式）。
pub(crate) fn box_aabb_half(half: Vec3, rot: Quat) -> Vec3 {
    let m = vxl_phys_core::Mat3::from_quat(rot);
    Vec3::new(
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).x.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).x.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).x.abs(),
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).y.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).y.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).y.abs(),
        (m.mul_vec3(Vec3::new(half.x, 0.0, 0.0))).z.abs()
            + (m.mul_vec3(Vec3::new(0.0, half.y, 0.0))).z.abs()
            + (m.mul_vec3(Vec3::new(0.0, 0.0, half.z))).z.abs(),
    )
}

/// 盒形包络的体素专用接触（**按盒的面聚合**，标准「参考面」做法）。
///
/// 采样 = 6 个面的面中心 + 4 角（每面 5 点，共 30 点）。**主导面** = skin 带内
/// 采样数最多的面（并列取最深的）；法线取**该面翻转**（= 表面外向法线；平面
/// 接触精确，斜面/棱边站立时是标准近似——球路径仍用精确 SDF 梯度）。
///
/// 实测教训：早先版本按「采样点的 SDF 梯度」逐点出法线，块体对齐落在柱顶时
/// 4 个角都落在柱角上 ⇒ 梯度是斜向的 ⇒ 窄相取到斜法线，盒沿斜向滑走。
pub fn contacts_box_voxel(
    v: &VoxelVolume,
    half: Vec3,
    pos: Vec3,
    rot: Quat,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let m = vxl_phys_core::Mat3::from_quat(rot);
    // **自适应查询范围（本轮修复）**：先收集「盒 AABB（±1 格）范围内的占据格」，
    // 采样点对这批格求最近距离——而不是只扫点周围 ±1 格（后者会让大碎块的采样点
    // 找不到最近的体素格、拿不到接触 ⇒ 自由落体穿地，实测 74/79 逃逸）。
    let ah = box_aabb_half(half, rot);
    let lo = v.grid_of(pos - ah - Vec3::splat(v.step));
    let hi = v.grid_of(pos + ah + Vec3::splat(v.step));
    let mut cells: Vec<(i32, i32, i32)> = Vec::new();
    for iz in lo.2.max(0)..=hi.2.min(v.nz as i32 - 1) {
        for iy in lo.1.max(0)..=hi.1.min(v.ny as i32 - 1) {
            for ix in lo.0.max(0)..=hi.0.min(v.nx as i32 - 1) {
                if v.get(ix as u32, iy as u32, iz as u32) {
                    cells.push((ix, iy, iz));
                }
            }
        }
    }
    if cells.is_empty() {
        return false;
    }
    // 到「这批格」的带符号距离（同 `sdf` 公式，但遍历给定列表）
    let sd = |p: Vec3| -> f32 {
        let mut best = v.step * 2.0;
        for &(ix, iy, iz) in &cells {
            let lo = v.grid_center(ix as u32, iy as u32, iz as u32) - Vec3::splat(v.step * 0.5);
            let hi = lo + Vec3::splat(v.step);
            let q = Vec3::new(
                (lo.x - p.x).max(p.x - hi.x).max(0.0),
                (lo.y - p.y).max(p.y - hi.y).max(0.0),
                (lo.z - p.z).max(p.z - hi.z).max(0.0),
            );
            let outside = q.length();
            let d = if outside > 0.0 {
                outside
            } else {
                let inx = (p.x - lo.x).min(hi.x - p.x);
                let iny = (p.y - lo.y).min(hi.y - p.y);
                let inz = (p.z - lo.z).min(hi.z - p.z);
                -inx.min(iny).min(inz)
            };
            if d < best {
                best = d;
            }
        }
        best
    };
    // 6 个面（局部轴向外法线）：±X/±Y/±Z
    let dirs = [
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let h = [half.x, half.y, half.z];
    let d3 = [
        [
            dirs[0].x, dirs[1].x, dirs[2].x, dirs[3].x, dirs[4].x, dirs[5].x,
        ],
        [
            dirs[0].y, dirs[1].y, dirs[2].y, dirs[3].y, dirs[4].y, dirs[5].y,
        ],
        [
            dirs[0].z, dirs[1].z, dirs[2].z, dirs[3].z, dirs[4].z, dirs[5].z,
        ],
    ];
    // 每面：(skin 带内采样数, 最深 signed_dist)
    let mut count = [0usize; 6];
    let mut deepest = [f32::INFINITY; 6];
    // 逐面采样（面中心 + 4 角）
    for k in 0..6 {
        let (fcx, fcy, fcz) = (d3[0][k] * h[0], d3[1][k] * h[1], d3[2][k] * h[2]);
        let axes = match k {
            0 | 1 => [1usize, 2],
            2 | 3 => [0, 2],
            _ => [0, 1],
        };
        // 采样点（局部）：中心 + 4 角
        let mut samples: [(f32, f32, f32); 5] = [(fcx, fcy, fcz); 5];
        for (ci, (su, sv)) in [(-1.0f32, -1.0f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
            .into_iter()
            .enumerate()
        {
            let mut l = [fcx, fcy, fcz];
            l[axes[0]] = su * h[axes[0]];
            l[axes[1]] = sv * h[axes[1]];
            samples[ci + 1] = (l[0], l[1], l[2]);
        }
        for s in &samples {
            let p = pos + m.mul_vec3(Vec3::new(s.0, s.1, s.2));
            let d = sd(p);
            if d < skin {
                count[k] += 1;
                deepest[k] = deepest[k].min(d);
            }
        }
    }
    // 主导面：skin 带内采样数最多 → 并列取最深
    let mut best = usize::MAX;
    for k in 0..6 {
        if count[k] == 0 {
            continue;
        }
        if best == usize::MAX
            || count[k] > count[best]
            || (count[k] == count[best] && deepest[k] < deepest[best])
        {
            best = k;
        }
    }
    if best == usize::MAX {
        return false;
    }
    // **逐面发射**（2026-09-15 修复）：此前只发「主导面」（按带内采样数、并列取
    // 最深选一张），墙角下会按深度选中**地板面**而把**墙面整张丢掉**——体的水平
    // 动量被倾斜的地板法线吸收、撞墙事件不登记（实测 `arena_bench wall_provider`：
    // 12 m/s 弹体在墙前 0.14 m 处 vx 11.4 → −0.02，墙面法线缺失 ⇒ 破坏管线无冲击，
    // 2 子步档下更明显）。现在 6 张面各自「带内即发」（法线按面、特征 = 面号×16 +
    // 采样号），主导面选择从 provider 内移到窄相，依据从"深度"换成"闭合速度"
    // （见 narrow 的 CLOSING_MIN 逻辑）——两处合起来才修好"墙角丢面"。
    let _ = best;
    let mut any = false;
    for k in 0..6usize {
        if count[k] == 0 {
            continue;
        }
        let n_world = m.mul_vec3(dirs[k] * -1.0);
        let (fcx, fcy, fcz) = (d3[0][k] * h[0], d3[1][k] * h[1], d3[2][k] * h[2]);
        let axes = match k {
            0 | 1 => [1usize, 2],
            2 | 3 => [0, 2],
            _ => [0, 1],
        };
        for ci in 0..5usize {
            let (lx, ly, lz) = if ci == 0 {
                (fcx, fcy, fcz)
            } else {
                let (su, sv) = match ci {
                    1 => (-1.0f32, -1.0f32),
                    2 => (-1.0, 1.0),
                    3 => (1.0, -1.0),
                    _ => (1.0, 1.0),
                };
                let mut l = [fcx, fcy, fcz];
                l[axes[0]] = su * h[axes[0]];
                l[axes[1]] = sv * h[axes[1]];
                (l[0], l[1], l[2])
            };
            let p = pos + m.mul_vec3(Vec3::new(lx, ly, lz));
            let d = sd(p);
            if d >= skin {
                continue;
            }
            out.push(vxl_phys_core::interop::InteropContact {
                point: p,
                normal: n_world,
                depth: -d,
                // 面号(1..6)×16 + 采样号(0 = 中心, 1..4 = 角)：跨帧稳定，供 warm 匹配
                feature: (k as u32 + 1) * 16 + ci as u32,
            });
            any = true;
        }
    }
    any
}

/// 球 vs 体素的**解析**接触（SDF 语义）：`depth = r − sdf(center)`，法线取
/// SDF 梯度；接触点取「球面点与 provider 表面点的中点」。只保留 `depth ≥ −skin`。
pub fn contacts_sphere_voxel(
    v: &VoxelVolume,
    center: Vec3,
    radius: f32,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let d = v.sdf(center);
    let depth = radius - d;
    if depth < -skin {
        return false;
    }
    let e = v.step * 0.5;
    let gx = v.sdf(center + Vec3::new(e, 0.0, 0.0)) - v.sdf(center - Vec3::new(e, 0.0, 0.0));
    let gy = v.sdf(center + Vec3::new(0.0, e, 0.0)) - v.sdf(center - Vec3::new(0.0, e, 0.0));
    let gz = v.sdf(center + Vec3::new(0.0, 0.0, e)) - v.sdf(center - Vec3::new(0.0, 0.0, e));
    let g = Vec3::new(gx, gy, gz);
    let n = if g.length_squared() > 1e-12 {
        g.normalize()
    } else {
        Vec3::Y
    };
    out.push(vxl_phys_core::interop::InteropContact {
        point: center - n * ((radius + d) * 0.5),
        normal: n,
        depth,
        feature: 1,
    });
    true
}

/// **点查询**：`depth = −sdf(p)`，即 **depth = 穿透量**（正 = 点已在固体里侧）；
/// 带内判据 `depth > −skin`（等价 `sdf < skin`）⇒ 表面外 `skin` 内仍生成**预期接触**。
/// 与 `contacts_box_voxel`（`depth = −d`）和网格路径（2026-09-21 起 `depth = −sd`）**同一口径**。
///
/// **偏置修正**（2026-09-21，P5 ⑳ 同族）：旧式 `depth = skin − sdf` 在**外侧**也给正 depth
/// ⇒ 走点查询的**外壳顶点采样**把体顶到 `sdf ≈ skin` ⇒ **包体在体素地面上悬空 ≈ 0.022 m**
/// （实测 `voxel_rest_probe`：同一块板上盒 −0.0004、球 −0.0000、**包 +0.0220**）。
/// 流体不受影响：它**不读 `depth`**，只用返回点的 `point`/`normal` 自算 sdf，
/// 且它过滤的上界就是 `self.h`（新口径的带正好等于管道半径 h，旧口径是 2h 后又被它滤掉）。
pub fn contacts_point_voxel(
    v: &VoxelVolume,
    p: Vec3,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let d = v.sdf(p);
    let depth = -d;
    if depth < -skin {
        return true; // 支持查询，但该点不在接触带内（无接触）
    }
    let e = v.step * 0.5;
    let gx = v.sdf(p + Vec3::new(e, 0.0, 0.0)) - v.sdf(p - Vec3::new(e, 0.0, 0.0));
    let gy = v.sdf(p + Vec3::new(0.0, e, 0.0)) - v.sdf(p - Vec3::new(0.0, e, 0.0));
    let gz = v.sdf(p + Vec3::new(0.0, 0.0, e)) - v.sdf(p - Vec3::new(0.0, 0.0, e));
    let g = Vec3::new(gx, gy, gz);
    let n = if g.length_squared() > 1e-12 {
        g.normalize()
    } else {
        Vec3::Y
    };
    out.push(vxl_phys_core::interop::InteropContact {
        point: p - n * d,
        normal: n,
        depth,
        feature: 0,
    });
    true
}

/// **点查询·流体边界口径**（`contacts_point_voxel` 的内点鲁棒变体）。
///
/// 所在格为空或出界（域外/腔内驻留点）与 [`contacts_point_voxel`] 逐位
/// 一致——外点的截断 SDF 梯度可信（刚体通道既有行为不变）。内点判定按
/// **所在格占据**而非 sdf 符号：截断 SDF 把出界当「空」，靠近网格外壳的
/// 内点（距外壳面不足一格）sdf 会误报为正 ⇒ 若按符号分流，会走旧外点
/// 路径沿截断梯度被持续外推——穿壁隧逃引信之一（实测 escapee 钉在
/// x≈−0.385 外壳驻留带上）。
///
/// 内点**不用**截断 SDF 的内部梯度：±1 格截断在薄壁/角部内部被格间内面
/// （占据格之间的共享面）主导，中心差分符号可翻转——轻则沿壁面滑推
/// （穿透轴永不修正），重则把粒子推出远侧表面（穿壁隧逃）。改为在 ±1 格
/// 邻域内搜最近**真表面面片**（占据格朝空邻格/出界的面）：距离² = 轴向
/// 差² + 两径向轴**各归各轴**的钳制距离²；法线 = 面轴外向，
/// `depth = skin − (−距离)` 与外点同口径。
///
/// 推回侧：内点一律推往最近的**流体可达面**（邻格在网格内且空）——外壳
/// 面通向域外，推过去等于把粒子逐出模拟域（隧逃引信之二：距外壳面不足
/// 0.25·step 的「新穿透原路推回」在壁内命中外壳面，与 sdf 误报叠加成
/// 外推棘轮，故不作「原路推回」特判）。邻域内无流体可达面才退外壳面；
/// 再无（深陷大固体，流体浅穿透机制下不会发生）退回截断 SDF 梯度路径
/// （次优但有限）。扫描序 (dz,dy,dx) × 面序 (轴,±) 固定，严格 `<` 取
/// 最近 ⇒ 确定。
pub fn contacts_point_voxel_solid(
    v: &VoxelVolume,
    p: Vec3,
    skin: f32,
    out: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let (cx, cy, cz) = v.grid_of(p);
    // 内点判定：所在格被占据 ⇒ 固体内部（格占据是精确判据，不受截断
    // SDF 在外壳附近的符号噪声影响）；否则（空格/出界）走旧外点路径。
    if !v.in_range(cx, cy, cz) || !v.get(cx as u32, cy as u32, cz as u32) {
        return contacts_point_voxel(v, p, skin, out);
    }
    let step = v.step;
    // 两轨最近真表面面片：open = 邻格在网格内且空（流体可达面）；
    // shell = 邻格出界（网格外壳面）。扫描序 (dz,dy,dx) × 面序 (轴,±)
    // 固定 + 严格 `<` 取最近 ⇒ 确定。
    let mut open_d2 = f32::INFINITY;
    let mut open_n = Vec3::Y;
    let mut shell_d2 = f32::INFINITY;
    let mut shell_n = Vec3::Y;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (ix, iy, iz) = (cx + dx, cy + dy, cz + dz);
                if !v.in_range(ix, iy, iz) || !v.get(ix as u32, iy as u32, iz as u32) {
                    continue;
                }
                let lo = v.grid_center(ix as u32, iy as u32, iz as u32) - Vec3::splat(step * 0.5);
                let hi = lo + Vec3::splat(step);
                // 两径向轴到格区间外的钳制距离（各归各轴；面片矩形距离的径向项）。
                let qx = (lo.x - p.x).max(p.x - hi.x).max(0.0);
                let qy = (lo.y - p.y).max(p.y - hi.y).max(0.0);
                let qz = (lo.z - p.z).max(p.z - hi.z).max(0.0);
                // 6 面：面为「真表面」⇔ 该方向邻格空或出界。面序固定
                // （−x,+x,−y,+y,−z,+z），法线 = 外向；到面片矩形的距离²
                // = 轴向差² + 两径向项²。
                let faces = [
                    (Vec3::new(-1.0, 0.0, 0.0), (lo.x - p.x).abs(), qy, qz),
                    (Vec3::new(1.0, 0.0, 0.0), (hi.x - p.x).abs(), qy, qz),
                    (Vec3::new(0.0, -1.0, 0.0), (lo.y - p.y).abs(), qx, qz),
                    (Vec3::new(0.0, 1.0, 0.0), (hi.y - p.y).abs(), qx, qz),
                    (Vec3::new(0.0, 0.0, -1.0), (lo.z - p.z).abs(), qx, qy),
                    (Vec3::new(0.0, 0.0, 1.0), (hi.z - p.z).abs(), qx, qy),
                ];
                for (dir, da, dr1, dr2) in faces {
                    let (nx, ny, nz) = (ix + dir.x as i32, iy + dir.y as i32, iz + dir.z as i32);
                    if v.in_range(nx, ny, nz) && v.get(nx as u32, ny as u32, nz as u32) {
                        continue; // 内面（贴着占据格）：截断 SDF 的噪声源，跳过
                    }
                    let d2 = da * da + dr1 * dr1 + dr2 * dr2;
                    if v.in_range(nx, ny, nz) {
                        if d2 < open_d2 {
                            open_d2 = d2;
                            open_n = dir;
                        }
                    } else if d2 < shell_d2 {
                        shell_d2 = d2;
                        shell_n = dir;
                    }
                }
            }
        }
    }
    // 推回侧：一律优先流体可达面（open），外壳面仅作邻域内无 open 时的
    // 兜底（见函数文档——「原路推回」特判在外壳附近会与 sdf 误报叠加成
    // 外推棘轮，已删除）；两者皆无退回截断 SDF 梯度路径（次优但有限）。
    let (face_n, dist) = if open_d2.is_finite() {
        (open_n, open_d2.sqrt())
    } else if shell_d2.is_finite() {
        (shell_n, shell_d2.sqrt())
    } else {
        return contacts_point_voxel(v, p, skin, out);
    };
    let depth = skin + dist; // 内点：sdf = −dist ⇒ depth = skin − (−dist)
    out.push(vxl_phys_core::interop::InteropContact {
        point: p + face_n * dist,
        normal: face_n,
        depth,
        feature: 0,
    });
    true
}
