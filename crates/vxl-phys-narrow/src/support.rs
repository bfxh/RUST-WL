//! support：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl DefaultNarrowPhase {
    /// 注册凸体外壳（点云，局部坐标）→ id；配 `Shape::ConvexHull { hull, .. }` 使用。
    pub fn add_hull(&mut self, points: Vec<Vec3>) -> u32 {
        self.hulls.add(points)
    }

    /// 外壳点云（局部坐标；空切片 = id 无效）。
    pub fn hull_points(&self, id: u32) -> &[Vec3] {
        self.hulls
            .get(id)
            .map(|h| h.points.as_slice())
            .unwrap_or(&[])
    }

    /// 注册复合体（子形状 = 形状 + 局部平移/旋转）→ id；配 `Shape::Compound { compound, .. }`。
    pub fn add_compound(&mut self, children: Vec<CompoundChild>) -> u32 {
        self.compounds.add(children)
    }

    /// 复合体子形状表（局部；空切片 = id 无效）。
    pub fn compound_children(&self, id: u32) -> &[CompoundChild] {
        self.compounds.get(id).unwrap_or(&[])
    }

    /// 子形状局部 AABB 并集半长（宽相/惯量近似用；空复合体 = ZERO）。
    pub fn compound_half_extents(&self, id: u32) -> Vec3 {
        self.compounds.half_extents(id)
    }

    /// 子形状表借出到 scratch（递归前必须归还；嵌套复合体已在注册期丢弃 ⇒ 不会重入）。
    pub(crate) fn kids_take(&mut self, id: u32) -> Option<Vec<CompoundChild>> {
        let mut buf = std::mem::take(&mut self.kids_buf);
        buf.clear();
        match self.compounds.get(id) {
            Some(kids) => {
                buf.extend_from_slice(kids);
                Some(buf)
            }
            None => {
                self.kids_buf = buf;
                None
            }
        }
    }

    pub(crate) fn kids_put(&mut self, buf: Vec<CompoundChild>) {
        self.kids_buf = buf;
    }

    /// 外壳点云 → 局部 AABB 半长（门面构 `Shape::ConvexHull` 用）。
    pub fn hull_half_extents(&self, id: u32) -> Vec3 {
        self.hulls.half_extents(id)
    }

    /// **凸体 Voronoi 预断裂**：原壳 ✕ 种子 ⇒ 逐格点云（凸、互斥、并集 = 原体）。
    /// 局部坐标（种子也给局部坐标）。`None` = 外壳 id 无效。
    pub fn fracture_hull(&self, id: u32, seeds: &[Vec3]) -> Option<Vec<Vec<Vec3>>> {
        self.hulls
            .get(id)
            .map(|h| gjk::fracture_voronoi_hull(h, seeds))
    }

    /// 填充「外壳世界点缓存」（side：0 = 对侧 a、1 = 侧 b）。
    /// 返回 true = 该形状是外壳且缓存已就绪（点列在 `self.hull_pts[side]`）。
    pub(crate) fn fill_hull_world(
        &mut self,
        side: usize,
        body: u32,
        shape: &Shape,
        pos: Vec3,
        rot: Quat,
    ) -> bool {
        let Shape::ConvexHull { hull, .. } = *shape else {
            return false;
        };
        let fp = rot_fp(rot);
        if self.cached_hull[side] != (body, fp) {
            let m = Mat3::from_quat(rot);
            let out = &mut self.hull_pts[side];
            out.clear();
            if let Some(h) = self.hulls.get(hull) {
                out.extend(h.points.iter().map(|p| pos + m.mul_vec3(*p)));
            }
            self.cached_hull[side] = (body, fp);
        }
        true
    }

    /// 形状 → 支撑体（不支持的形状返回 None）。
    pub(crate) fn support_of(
        &self,
        shape: &Shape,
        pos: Vec3,
        rot: Quat,
    ) -> Option<gjk::ShapeSupport<'_>> {
        match *shape {
            Shape::ConvexHull { hull, .. } => self.hulls.get(hull).map(|h| {
                gjk::ShapeSupport::Hull(gjk::HullSupport {
                    hull: h,
                    pos,
                    rot: Mat3::from_quat(rot),
                })
            }),
            Shape::Box { half } => Some(gjk::ShapeSupport::Box(gjk::BoxSupport {
                half,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            Shape::Sphere { radius } => Some(gjk::ShapeSupport::Sphere(gjk::SphereSupport {
                radius,
                pos,
            })),
            Shape::Capsule {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Capsule(gjk::CapsuleSupport {
                half_height,
                radius,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            // 圆柱/圆锥：**多面化表示**的支撑（顶点有限 ⇒ EPA 良态）。此前缺这两支 ⇒
            // 「外壳 × 圆柱/锥」这类组合**静默无接触**（见 `TECH-SURVEY.md` A9 ④ 留档）。
            Shape::Cylinder {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Prism(gjk::PrismSupport {
                half_height,
                radius,
                segments: CYLINDER_SEGMENTS,
                cone: false,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            Shape::Cone {
                half_height,
                radius,
            } => Some(gjk::ShapeSupport::Prism(gjk::PrismSupport {
                half_height,
                radius,
                segments: CYLINDER_SEGMENTS,
                cone: true,
                pos,
                rot: Mat3::from_quat(rot),
            })),
            _ => None,
        }
    }

    /// **外壳 × {盒|球|外壳}**：GJK/EPA 求穿透 → 用**外壳近接触面顶点**细化成多点流形。
    ///
    /// - 法线一次求解（EPA，轴对齐退化时退 6 轴 SAT 解析）；
    /// - 流形点 = 外壳点云中落在对方支撑面 `plane ± skin` 带内的顶点，
    ///   逐点深度 `plane − n̂·v`（n̂ = 对方 → 外壳）；取最深 4 点。
    /// - `feature = 顶点序号 + 1`（点云序稳定 ⇒ 跨帧可续接）。
    /// - 外壳 × 高度场：**已支持**——走 `hull_heightfield`（逐顶点采样，与 `poly_heightfield`
    ///   同款），不再走本函数。
    #[allow(clippy::too_many_arguments)] // 与 process_pair 同形（两侧位姿 + 形状 + 出参）
    pub(crate) fn hull_pair(
        &mut self,
        a: u32,
        b: u32,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    ) {
        let a_is_hull = matches!(*sa, Shape::ConvexHull { .. });
        let (side, body_id, hshape, hpos, hrot) = if a_is_hull {
            (0usize, a, sa, pa, ra)
        } else {
            (1usize, b, sb, pb, rb)
        };
        let other_shape = if a_is_hull { sb } else { sa };
        // 世界点缓存（早于借支撑体：填充需要 &mut self）
        if !self.fill_hull_world(side, body_id, hshape, hpos, hrot) {
            return;
        }

        // —— 对方是高度场（L1）：顶点采样，与盒/圆柱版同构 ——
        if let Shape::HeightField(hf_id) = *other_shape {
            let Some(hf) = heightfields.get(hf_id as usize) else {
                return;
            };
            self.cand.clear();
            let n_pts = self.hull_pts[side].len();
            for idx in 0..n_pts {
                let v = self.hull_pts[side][idx];
                if let Some((h, _)) = hf.sample(v.x, v.z) {
                    let depth = h - v.y;
                    if depth > -self.skin {
                        self.cand.push(ContactPoint {
                            point: Vec3::new(v.x, h, v.z),
                            depth,
                            feature: idx as u32,
                        });
                    }
                }
            }
            if self.cand.is_empty() || !self.select_contacts(self.min_point_sep) {
                return;
            }
            let deepest = self.cand[0];
            let n_t = hf
                .sample(deepest.point.x, deepest.point.z)
                .map(|(_, n)| n)
                .unwrap_or(Vec3::Y);
            // 法线约定 a→b：外壳在 a（地形在 b）⇒ −n_t；否则 +n_t
            let normal = if a_is_hull { -n_t } else { n_t };
            out.push(Manifold {
                a,
                b,
                normal,
                points: ContactPoints::from_slice(&self.cand),
            });
            return;
        }

        // —— 对方是盒/球/外壳：GJK/EPA 一次法线 + 外壳近面顶点细化 ——
        let (Some(ua), Some(ub)) = (self.support_of(sa, pa, ra), self.support_of(sb, pb, rb))
        else {
            return; // 对方形状不受理
        };
        let Some((n_p, _depth, _p)) = gjk::epa(&ua, &ub, 32) else {
            return; // 未相交（宽相 fat 边距会给出近邻对）
        };
        let n = if a_is_hull { n_p } else { -n_p }; // 对方 → 外壳
        let other: &dyn gjk::Support = if a_is_hull { &ub } else { &ua };
        let plane = n.dot(other.support(n));
        let mut cand: Vec<(f32, usize, Vec3)> = Vec::new();
        let n_pts = self.hull_pts[side].len();
        for i in 0..n_pts {
            let w = self.hull_pts[side][i];
            let d = plane - n.dot(w);
            if d > -self.skin {
                cand.push((d, i, w));
            }
        }
        if cand.is_empty() {
            return;
        }
        // 最深 4 点（深度降序；并列按顶点序 ⇒ 确定性）
        cand.sort_by(|x, y| {
            y.0.partial_cmp(&x.0)
                .unwrap_or(core::cmp::Ordering::Equal)
                .then(x.1.cmp(&y.1))
        });
        cand.truncate(4);
        let pts: Vec<ContactPoint> = cand
            .iter()
            .map(|&(d, i, w)| ContactPoint {
                point: w,
                depth: d,
                feature: (i as u32) + 1,
            })
            .collect();
        out.push(Manifold {
            a,
            b,
            normal: -n_p, // 流形约定：a → b
            points: ContactPoints::from_slice(&pts),
        });
    }

    /// **设置速度充气视野的预测时长**（0 = 关闭；见 `predict_dt` 字段注）。
    /// 由 `World` 在每次 `collide` 前设置：检测间隔 = 距下一次窄相的时间。
    pub fn set_predict_dt(&mut self, dt: f32) {
        self.predict_dt = if dt > 0.0 { dt } else { 0.0 };
    }

    /// 本对的充气量＝`max(0, 接近速度)·predict_dt`（`predict_dt = 0` ⇒ 恒 0）。
    /// 接近速度取**质心**相对速度在法向的投影（忽略角速度贡献：角项在贴面接触上是一阶小量）。
    pub(crate) fn predict_inflate(
        &self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        n: Vec3,
    ) -> f32 {
        if self.predict_dt <= 0.0 {
            return 0.0;
        }
        let vrel = bodies.linvel[b as usize] - bodies.linvel[a as usize];
        (-vrel.dot(n)).max(0.0) * self.predict_dt
    }

    pub fn new(skin: f32) -> Self {
        Self {
            hulls: HullStore::default(),
            compounds: CompoundStore::default(),
            kids_buf: Vec::new(),
            skin,
            predict_dt: 0.0,
            inflate: 0.0,
            min_point_sep: (skin * 2.0).max(0.01),
            polys: Vec::new(),
            poly_index: HashMap::new(),
            poly_a: WorldPoly::default(),
            poly_b: WorldPoly::default(),
            cached_a: (u32::MAX, u64::MAX),
            cached_b: (u32::MAX, u64::MAX),
            axes: Vec::new(),
            clip_in: Vec::new(),
            clip_out: Vec::new(),
            cand: Vec::new(),
            kept_buf: Vec::new(),
            box_a: None,
            box_b: None,
            box_axes_a: None,
            box_axes_b: None,
            hull_pts: [Vec::new(), Vec::new()],
            cached_hull: [(u32::MAX, u64::MAX), (u32::MAX, u64::MAX)],
            cached_ax_a: (u32::MAX, u64::MAX, [Vec3::ZERO; 3]),
            cached_ax_b: (u32::MAX, u64::MAX, [Vec3::ZERO; 3]),
            ref_v: Vec::new(),
            out_hint: 256,
            probe_clip_calls: 0,
            probe_clip_iters: 0,
            probe_clip_xings: 0,
            probe_cand_pts: 0,
            probe_clip_max: 0,
        }
    }

    pub(crate) fn poly_for(&mut self, shape: &Shape) -> Option<usize> {
        let key = poly_key(shape);
        if key == 0 {
            return None;
        }
        if let Some(&idx) = self.poly_index.get(&key) {
            return Some(idx);
        }
        let poly = match *shape {
            Shape::Box { half } => ConvexPolytope::box_polytope(half),
            Shape::Cylinder {
                half_height,
                radius,
            } => ConvexPolytope::cylinder_polytope(radius, half_height, CYLINDER_SEGMENTS),
            Shape::Cone {
                half_height,
                radius,
            } => ConvexPolytope::cone_polytope(radius, half_height, CYLINDER_SEGMENTS),
            _ => return None,
        };
        self.polys.push(poly);
        let idx = self.polys.len() - 1;
        self.poly_index.insert(key, idx);
        Some(idx)
    }
}
