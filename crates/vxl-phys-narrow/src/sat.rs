//! sat：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 盒对体轴 `(a, b)`：盒×盒专用路径给 `Some`；其余组合（盒×柱/锥…）走通用多面体路径给 `None`。
type BoxAxes = Option<([Vec3; 3], [Vec3; 3])>;

impl DefaultNarrowPhase {
    /// 参考面：与「ref → incident」方向最对齐的面（盒对读体轴、通用读多面体）。
    fn select_ref_face(
        &self,
        ref_is_a: bool,
        dir_to_incident: Vec3,
        box_axes: BoxAxes,
    ) -> (usize, Vec3) {
        match box_axes {
            Some((aa, ab)) => {
                let axes = if ref_is_a { aa } else { ab };
                let mut bi = 0;
                let mut bd = f32::MIN;
                for (i, f) in BOX_FACES.iter().enumerate() {
                    let d = face_normal_of(&axes, f.0).dot(dir_to_incident);
                    if d > bd {
                        bd = d;
                        bi = i;
                    }
                }
                (bi, face_normal_of(&axes, BOX_FACES[bi].0))
            }
            None => {
                let p = if ref_is_a { &self.poly_a } else { &self.poly_b };
                let mut bi = 0;
                let mut bd = f32::MIN;
                for (i, &n) in p.face_normal.iter().enumerate() {
                    let d = n.dot(dir_to_incident);
                    if d > bd {
                        bd = d;
                        bi = i;
                    }
                }
                (bi, p.face_normal[bi])
            }
        }
    }

    /// 参考面的世界顶点入 scratch，返回**全局基准号**（供侧平面特征号复用）。
    fn collect_ref_verts(&mut self, box_axes: BoxAxes, ref_is_a: bool, ref_face_idx: usize) -> u32 {
        self.ref_v.clear();
        match box_axes {
            Some((aa, ab)) => {
                let (axes, half, pos) = if ref_is_a {
                    let (h, p) = self.box_a.expect("盒对专用路径必置 box_a");
                    (aa, h, p)
                } else {
                    let (h, p) = self.box_b.expect("盒对专用路径必置 box_b");
                    (ab, h, p)
                };
                for &sg in BOX_FACES[ref_face_idx].1.iter() {
                    self.ref_v.push(face_vertex(pos, &axes, half, sg));
                }
                ref_face_idx as u32 * 4
            }
            None => {
                let (s, e) = if ref_is_a {
                    let p = &self.poly_a;
                    (
                        p.face_start[ref_face_idx] as usize,
                        p.face_start[ref_face_idx + 1] as usize,
                    )
                } else {
                    let p = &self.poly_b;
                    (
                        p.face_start[ref_face_idx] as usize,
                        p.face_start[ref_face_idx + 1] as usize,
                    )
                };
                if ref_is_a {
                    self.ref_v.extend_from_slice(&self.poly_a.verts[s..e]);
                } else {
                    self.ref_v.extend_from_slice(&self.poly_b.verts[s..e]);
                }
                s as u32
            }
        }
    }

    /// 入射面：与 n_ref 最逆平行的面；顶点带特征号入裁剪多边形。
    fn collect_incident(&mut self, box_axes: BoxAxes, ref_is_a: bool, n_ref: Vec3) {
        self.clip_in.clear();
        let side_bit = if ref_is_a { FEAT_SIDE_B } else { 0 };
        match box_axes {
            Some((aa, ab)) => {
                let (axes, half, pos) = if ref_is_a {
                    let (h, p) = self.box_b.expect("盒对专用路径必置 box_b");
                    (ab, h, p)
                } else {
                    let (h, p) = self.box_a.expect("盒对专用路径必置 box_a");
                    (aa, h, p)
                };
                let mut inc_face = 0;
                let mut inc_dot = f32::MAX;
                for (i, f) in BOX_FACES.iter().enumerate() {
                    let d = face_normal_of(&axes, f.0).dot(n_ref);
                    if d < inc_dot {
                        inc_dot = d;
                        inc_face = i;
                    }
                }
                let base = inc_face as u32 * 4;
                for (i, &sg) in BOX_FACES[inc_face].1.iter().enumerate() {
                    let v = face_vertex(pos, &axes, half, sg);
                    self.clip_in.push((v, side_bit | (base + i as u32)));
                }
            }
            None => {
                let inc_poly = if ref_is_a { &self.poly_b } else { &self.poly_a };
                let mut inc_face = 0;
                let mut inc_dot = f32::MAX;
                for (i, &n) in inc_poly.face_normal.iter().enumerate() {
                    let d = n.dot(n_ref);
                    if d < inc_dot {
                        inc_dot = d;
                        inc_face = i;
                    }
                }
                let (is_, ie) = {
                    let p = inc_poly;
                    (
                        p.face_start[inc_face] as usize,
                        p.face_start[inc_face + 1] as usize,
                    )
                };
                for k in is_..ie {
                    let v = inc_poly.verts[k];
                    self.clip_in.push((v, side_bit | (k as u32)));
                }
            }
        }
    }

    /// 逐侧平面裁剪入射多边形（侧平面向量**不归一化**）；`false` = 已裁空。
    fn clip_by_side_planes(&mut self, n_ref: Vec3, ref_base: u32) -> bool {
        let mut centroid = Vec3::ZERO;
        for &v in &self.ref_v {
            centroid += v;
        }
        centroid *= 1.0 / self.ref_v.len() as f32;

        // 逐侧平面裁剪入射多边形。侧平面向量**不归一化**：保留判定
        // （`da <= 0`）、穿越判定（`da*db < 0`）与插值参数
        // （`t = da/(da−db)`）全部与平面向量尺度无关，故省掉每面一次
        // sqrt + 除法（旧实现每 clip 4 次）。
        let nref_v = self.ref_v.len();
        for k in 0..nref_v {
            let w0 = self.ref_v[k];
            let w1 = self.ref_v[if k + 1 == nref_v { 0 } else { k + 1 }];
            let e = w1 - w0;
            let mut s = e.cross(n_ref);
            if s.length_squared() < 1e-16 {
                continue;
            }
            if s.dot(centroid - w0) > 0.0 {
                s = -s;
            }
            // keep: dot(v - w0, s) <= 0
            self.clip_out.clear();
            let m = self.clip_in.len();
            self.probe_clip_iters += m as u64;
            self.probe_clip_max = self.probe_clip_max.max(m as u64);
            for i in 0..m {
                let (va, fa) = self.clip_in[i];
                let (vb, fb) = self.clip_in[(i + 1) % m];
                let da = (va - w0).dot(s);
                let db = (vb - w0).dot(s);
                if da <= 0.0 {
                    self.clip_out.push((va, fa));
                }
                if da * db < 0.0 {
                    let t = da / (da - db);
                    self.probe_clip_xings += 1;
                    self.clip_out.push((
                        va + (vb - va) * t,
                        feat_intersect(fa, fb, ref_base as usize + k),
                    ));
                }
            }
            core::mem::swap(&mut self.clip_in, &mut self.clip_out);
            if self.clip_in.is_empty() {
                return false;
            }
        }
        true
    }

    /// SAT：双侧分离判定，返回 (分离距离 ≤ skin, 轴 a→b, 来源)。
    ///
    /// 对每根轴同时测两个方向（A 在负侧 / B 在负侧），取较大分离度；
    /// 法线统一取向为 a→b。此前单侧公式的取向错误会造成深度失真（能量泵）。
    pub(crate) fn sat(&mut self, _hint: Vec3) -> Option<(f32, Vec3, AxisSrc)> {
        let na = self.poly_a.face_normal.len();
        let nb = self.poly_b.face_normal.len();
        self.axes.clear();
        // 轴表：盒对专用路径由体轴直生（面 6 轴 + 棱叉积 9 轴，顺序与通用
        // 路径逐条对应——面序 [±X,±Y,±Z]、棱序 [+Y,+Z,+X]×[+Y,+Z,+X]，后者由
        // polytope 测试钉死）；其余形状读多面体。
        let box_axes = match (self.box_axes_a, self.box_axes_b) {
            (Some(aa), Some(ab)) => Some((aa, ab)),
            _ => None,
        };
        if let Some((aa, ab)) = box_axes {
            for f in &BOX_FACES {
                self.axes.push(face_normal_of(&aa, f.0));
            }
            for f in &BOX_FACES {
                self.axes.push(face_normal_of(&ab, f.0));
            }
            let ea = [aa[1], aa[2], aa[0]];
            let eb = [ab[1], ab[2], ab[0]];
            for x in ea {
                for y in eb {
                    let c = x.cross(y);
                    let l2 = c.length_squared();
                    if l2 > 1e-8 {
                        self.axes.push(c * (1.0 / l2.sqrt()));
                    }
                }
            }
        } else {
            for i in 0..na {
                self.axes.push(self.poly_a.face_normal[i]);
            }
            for i in 0..nb {
                self.axes.push(self.poly_b.face_normal[i]);
            }
            for &ea in &self.poly_a.edge_dirs {
                for &eb in &self.poly_b.edge_dirs {
                    let c = ea.cross(eb);
                    let l2 = c.length_squared();
                    if l2 > 1e-8 {
                        self.axes.push(c * (1.0 / l2.sqrt()));
                    }
                }
            }
        }
        // 契约（写清楚，因为原断言把它写反了）：**通用路径的多面体必须已填**
        // （`na/nb` 即面轴条数）。分派侧保证这件事：盒对走 T3 专用路径、**不填**多面体
        // （轴由 (rot, half) 直生）；圆柱/圆锥参与的对走通用分支，进 `sat` 前已
        // `poly_a/poly_b.fill`（世界多面体缓存命中时用的是上一次同 (体, 多面体) 的填充，
        // 长度同源）。⇒ 通用路径下 `na/nb` 恒 > 0；读它们当"面轴数"是对的。
        // ⚠️ 原文是 `debug_assert!(box_axes.is_some() || (na == 6 && nb == 6))`——
        // 第二个析取项与紧随其后的 `match box_axes { None => (na, nb) }` 自相矛盾：
        // 圆柱是 **18 面**（`CYLINDER_SEGMENTS=16` + 两端面），盒×柱是正当组合。
        // 实测（2026-09-22）：debug 档把 `tests/rolling_probe.rs` 打成 panic，
        // 而 release 因 `debug_assert` 被编译掉**掩盖**了它（CI 跑 release ⇒ 全绿）。
        // 现在断言的是真契约——"进了通用路径却没填多面体"才是要抓的 bug（会静默出垃圾接触）。
        debug_assert!(
            box_axes.is_some() || (na > 0 && nb > 0),
            "通用路径的多面体必须已填：na={na}, nb={nb}"
        );
        // 面轴计数：专用路径恒 6+6（体轴直生，未填多面体），通用路径读多面体。
        let (n_face_a, n_face_b) = match box_axes {
            Some(_) => (6usize, 6usize),
            None => (na, nb),
        };
        // ===== 盒对快路径：SIMD 4 轴并行扫描（x86_64 SSE2；非 x86_64 走标量）=====
        // 与下方通用标量循环**逐位等价**（轴内算术/结合序/三条规则全同），
        // `simd::tests::simd_matches_scalar_bitwise` 逐位对照守门。
        // 注：曾试「面轴/棱轴分段扫描（分离时跳过棱轴构建）」——实测**更慢**
        // （窄相峰 26.51 → 28.79，棱轴构建不是瓶颈、分段徒增重入）⇒ 已回退。
        if let Some((aa, ab)) = box_axes {
            let (ha, pa) = self.box_a.expect("盒对快路径必置 box_a");
            let (hb, pb) = self.box_b.expect("盒对快路径必置 box_b");
            let scanned = simd::sat_scan(&self.axes, ha, &aa, pa, hb, &ab, pb, self.skin);
            return scanned.map(|(sep, n, idx)| {
                let src = if idx < n_face_a {
                    AxisSrc::FaceA
                } else if idx < n_face_a + n_face_b {
                    AxisSrc::FaceB
                } else {
                    AxisSrc::Edge
                };
                (sep, n, src)
            });
        }
        // ===== 非盒对（球/圆柱/高度场参与）：通用标量扫描 =====
        let mut best = f32::MIN;
        let mut best_n = Vec3::ZERO;
        let mut best_src = AxisSrc::Edge;
        for (idx, &n0) in self.axes.iter().enumerate() {
            if n0.length_squared() < 0.5 {
                continue;
            }
            let mut min_a = f32::MAX;
            let mut max_a = f32::MIN;
            let mut min_b = f32::MAX;
            let mut max_b = f32::MIN;
            // T3 快路径：盒对用 extents 投影公式（面法线 [0]/[2]/[4] 即体轴，
            // 与逐顶点 min/max 数学等价），轴序/取向/来源分类与通用路径同。
            if let (Some((ha, pa)), Some((hb, pb)), Some((aa, ab))) =
                (self.box_a, self.box_b, box_axes)
            {
                let ra = ha.x * aa[0].dot(n0).abs()
                    + ha.y * aa[1].dot(n0).abs()
                    + ha.z * aa[2].dot(n0).abs();
                let rb = hb.x * ab[0].dot(n0).abs()
                    + hb.y * ab[1].dot(n0).abs()
                    + hb.z * ab[2].dot(n0).abs();
                let ca = pa.dot(n0);
                let cb = pb.dot(n0);
                min_a = ca - ra;
                max_a = ca + ra;
                min_b = cb - rb;
                max_b = cb + rb;
            } else if let (Some((ha, pa)), Some((hb, pb))) = (self.box_a, self.box_b) {
                let ax = &self.poly_a.face_normal;
                let bx = &self.poly_b.face_normal;
                let ra = ha.x * ax[0].dot(n0).abs()
                    + ha.y * ax[2].dot(n0).abs()
                    + ha.z * ax[4].dot(n0).abs();
                let rb = hb.x * bx[0].dot(n0).abs()
                    + hb.y * bx[2].dot(n0).abs()
                    + hb.z * bx[4].dot(n0).abs();
                let ca = pa.dot(n0);
                let cb = pb.dot(n0);
                min_a = ca - ra;
                max_a = ca + ra;
                min_b = cb - rb;
                max_b = cb + rb;
            } else {
                for &v in &self.poly_a.verts {
                    let d = v.dot(n0);
                    if d < min_a {
                        min_a = d;
                    }
                    if d > max_a {
                        max_a = d;
                    }
                }
                for &v in &self.poly_b.verts {
                    let d = v.dot(n0);
                    if d < min_b {
                        min_b = d;
                    }
                    if d > max_b {
                        max_b = d;
                    }
                }
            }
            // 两个分离方向：A 在负侧（n 指向 a→b）或 B 在负侧（翻转）。
            let sep1 = min_b - max_a;
            let sep2 = min_a - max_b;
            let (sep, n) = if sep1 >= sep2 {
                (sep1, n0)
            } else {
                (sep2, -n0)
            };
            if sep > self.skin {
                return None;
            }
            if sep > best {
                best = sep;
                best_n = n;
                best_src = if idx < n_face_a {
                    AxisSrc::FaceA
                } else if idx < n_face_a + n_face_b {
                    AxisSrc::FaceB
                } else {
                    AxisSrc::Edge
                };
            }
        }
        if best == f32::MIN {
            return None;
        }
        Some((best, best_n, best_src))
    }

    /// 参考面裁剪：生成接触点（深度可为小负值）。
    /// 参考面按「ref → incident 方向」与外法线对齐重选（与轴来源无关，稳健）。
    pub(crate) fn clip(&mut self, normal_ab: Vec3, src: AxisSrc) -> bool {
        let ref_is_a = !matches!(src, AxisSrc::FaceB);
        let dir_to_incident = if ref_is_a { normal_ab } else { -normal_ab };
        // 面轴来源：盒对专用路径（体轴直生，免多面体填充）或通用多面体。
        // 两者给出的面法线序列逐位相同（BOX_FACES 顺序 = 多面体面序，
        // ±v 精确），故参考/入射面选择与平局分解不变。
        let box_axes = match (self.box_axes_a, self.box_axes_b) {
            (Some(aa), Some(ab)) => Some((aa, ab)),
            _ => None,
        };
        let (ref_face_idx, n_ref) = self.select_ref_face(ref_is_a, dir_to_incident, box_axes);
        // 参考面世界顶点入 scratch（含全局基准号，供侧平面特征号复用）。
        let ref_base = self.collect_ref_verts(box_axes, ref_is_a, ref_face_idx);
        // 入射面：与 n_ref 最逆平行的面；顶点直接入裁剪多边形（带特征号）。
        self.collect_incident(box_axes, ref_is_a, n_ref);
        // 参考面质心（侧面朝向判定）。
        if !self.clip_by_side_planes(n_ref, ref_base) {
            return false;
        }
        // 主平面过滤：保留 n_ref 方向距离 ≤ skin（+本对充气量）的点（depth = -dist）。
        let p0 = self.ref_v[0];
        self.cand.clear();
        for &(v, feat) in &self.clip_in {
            let d = (v - p0).dot(n_ref);
            if d <= self.skin + self.inflate {
                self.cand.push(ContactPoint {
                    point: v,
                    depth: -d,
                    feature: feat,
                });
            }
        }
        if self.cand.is_empty() {
            return false;
        }
        self.probe_clip_calls += 1;
        self.probe_cand_pts += self.cand.len() as u64;

        // 去重 + 取最深 ≤4 点（确定性排序见 select_contacts）。
        self.select_contacts(self.min_point_sep)
    }
}
