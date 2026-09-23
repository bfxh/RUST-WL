//! fluid_boundary：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

impl FluidSystem {
    /// 反作用聚合：逐粒 `bforce` → 每体 `(力, 绕**体原点**的力矩)`。
    /// 求和序 = 段序（生成序）× 段内粒子索引序 ⇒ 确定性。
    pub(crate) fn aggregate_reactions(&mut self) {
        self.breact.clear();
        for &(body, origin, start, end) in &self.spans {
            let mut f = Vec3::ZERO;
            let mut tau = Vec3::ZERO;
            for k in start..end {
                let fk = self.bforce[k as usize];
                f += fk;
                tau += (self.pos[k as usize] - origin).cross(fk);
            }
            self.breact.push((body, f, tau));
        }
    }

    /// 边界投影：`contacts_point` 接触带内，只推**真穿透**（sdf < 0，由
    /// 接触点/外法线恢复，与 provider 的 depth 口径解耦）+ 法向速度归零
    /// （非弹性 e=0）。推到 sdf = +skin 驻留线（穿透量 + skin）：静隙由
    /// 壁压自平衡，投影只定下限——且驻留点 sdf > 0 让密度轮的镜像平面
    /// 收集始终覆盖得到（sdf = 0 会被 `sdf > 0` 滤掉 → 壁邻鬼影丢失）。
    /// **投影坍缩消除**：多平面深穿透被逐轴推到各平面交集 = 同一个点
    /// （凹角/棱），同位粒子对互喂 W(0) ⇒ ρ ≈ 2ρ0 ⇒ Tait q⁷ 爆压
    /// （实测角点堆粒子 ρ=2153 起爆喷泉）；且同位对 d = 0 ⇒ 压力梯度
    /// 零方向 ⇒ 永久僵局。解析：低索引驻留、高索引沿自身接触法线和
    /// （流体侧）外移 r_min = 0.2h；索引升序 + 法线序固定 ⇒ 确定。
    /// 预滤余量 = h：必须 ≥ 单子步最大行程（CFL 上限 0.4h）+ 接触带，
    /// 否则快速粒子一步跨过查询带 → 永久脱离所有接触查询（自由落体逃逸）。
    /// 复用统一提供者通道：体素/三角网/喷溅无改动即为边界。
    /// **只投影流体粒子**（索引前缀）：边界粒子在体内、由体运动学带着走，
    /// 既不该被提供者推出，也不该被推出体外。
    pub(crate) fn boundary_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.boundaries.is_empty() {
            return;
        }
        // 本子步被投影粒子：(索引, 接触法线和)。升序登记 ⇒ 消解序确定。
        let mut pushed: Vec<(usize, Vec3)> = Vec::new();
        for i in 0..self.n_fluid {
            let pi = self.pos[i];
            // 投影累计跨所有边界（角部粒子同帧吃地面+墙的多笔推出）。
            let mut p = pi;
            let mut v = self.vel[i];
            let mut n_sum = Vec3::ZERO;
            let mut hit = false;
            for &bid in &self.boundaries {
                // AABB 预滤（外扩 h；None = 无信息 ⇒ 仍查询）。
                if let Some(bb) = providers.bounds(bid) {
                    let m = self.h;
                    if pi.x < bb.min.x - m
                        || pi.x > bb.max.x + m
                        || pi.y < bb.min.y - m
                        || pi.y > bb.max.y + m
                        || pi.z < bb.min.z - m
                        || pi.z > bb.max.z + m
                    {
                        continue;
                    }
                }
                self.contacts.clear();
                // 流体边界口径（内点鲁棒）：体素等截断 SDF 提供者的内部
                // 梯度被格间内面主导可指向固体深处 ⇒ 投影穿壁隧逃（实测
                // 1–2 格厚壁均复现）；解析面提供者默认原样转 contacts_point。
                if providers.contacts_point_boundary(bid, pi, self.skin, &mut self.contacts) {
                    for c in &self.contacts {
                        let sdf = (pi - c.point).dot(c.normal);
                        let pen = -sdf;
                        if pen <= 0.0 {
                            continue;
                        }
                        p += c.normal * (pen + self.skin).min(self.h);
                        n_sum += c.normal;
                        hit = true;
                        let vn = v.dot(c.normal);
                        if vn < 0.0 {
                            v -= c.normal * vn;
                        }
                    }
                    self.pos[i] = p;
                    self.vel[i] = v;
                }
            }
            if hit {
                pushed.push((i, n_sum));
            }
        }
        // 同位坍缩消解：仅在本子步被投影粒子间做最小间距（r_min = 0.2h，
        // 远小于静置间距，常态零触发）。高索引者沿 n_sum 内移——投影把
        // 深穿粒子逐轴推到平面交集点，法线和指向流体侧，一步拉开后
        // 常规压强接管。位移 ≤ r_min，不注入爆发能量。
        let r_min = 0.2 * self.h;
        for a in 0..pushed.len() {
            let (ia, na) = pushed[a];
            let nl = na.length();
            if nl <= 1e-9 {
                continue;
            }
            let dir = na * (1.0 / nl);
            let mut pa = self.pos[ia];
            for &(ib, _) in &pushed[..a] {
                let d = pa - self.pos[ib];
                let r2 = d.length_squared();
                if r2 < r_min * r_min {
                    let r = r2.sqrt();
                    // 同位（r=0）也拉满 r_min；方向 = 自身法线和，不依赖 d。
                    pa += dir * (r_min - r);
                }
            }
            self.pos[ia] = pa;
        }
    }
}
