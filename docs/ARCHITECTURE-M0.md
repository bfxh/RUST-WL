# ARCHITECTURE-M0 —— vxl_phys 竖切架构与验收映射

## 步进管线（每 60Hz tick，`config.substeps` 细分）

```
力场注册表(FieldRegistry) → 速度积分(Integrator, 半隐式欧拉, 限速钳制)
→ 宽相(GridBroadPhase, 空间哈希, 输出有序对) → 窄相(DefaultNarrowPhase)
→ 唤醒传播 → 岛(并查集) → 顺序冲量迭代 ×N → 休眠判定(§4.11)
→ 位置积分(半隐式欧拉 + 四元数角速度积分)
```

## 关键设计决定

1. **圆柱 = 固定 16 边内接棱柱**（碰撞层）。规格书 §2.4 选型 GJK/EPA（M1）；
   M0 用「多面体 SAT + 参考面裁剪」先走通，棱柱化让圆柱进同一凸体管线。
   质量属性仍按解析圆柱（§2.1），两者不一致是已知且记录的近似。
2. **法线约定**：流形 `normal` 一律从 a 指向 b；求解器把 +n 冲量给 b、−n 给 a。
   单测覆盖方向（球在盒上 → normal.y < 0 等）。
3. **确定性**：
   - 全库 `#![forbid(unsafe_code)]`；标量 f32，固定表达式顺序；
   - 宽相只 lookup/insert，从不迭代哈希桶；对输出排序去重；
   - 岛：并查集小索引为根；岛序 = 体索引首现序；约束序 = 流形字典序；
   - warm starting 匹配按距离最近 + 法线对齐（dot > 0.95），匹配本身确定性。
4. **休眠**（§4.11）：岛级判定，全员 速度 < 0.04 / 0.05 rad/s 持续 0.5 s → 入睡
   （速度清零、跳过积分/求解）；唤醒 = 传播（醒体触睡体）或显式 API。
5. **高度场**：静态 marker 体（`Shape::HeightField(id)`）进宽相，窄相对动体做
   球列采样 / 多面体逐顶点采样（M1 换列裁剪连续碰撞）。挖掘走 terrain 账本。
6. **性能预算（M0）**：10k 静态 + 1k 动态目标 60 Hz。当前瓶颈预估：
   窄相 SAT 轴数（圆柱-圆柱 325 轴）与逐步分配（manifold points clone）——
   M1 用轴去重 + 缓冲池解决。

## §3 稳定性验收映射（适用一切规模档）

| 指标 | 实现 | 测试 |
|---|---|---|
| NaN/Inf 计数 = 0 | `World::health().nan_bodies` | `health` 全测试断言 |
| 静默穿透（深度 > skin×4）= 0 | `health().deep_penetrations` | `box_falls_and_rests`、`pyramid_settles` |
| 持续抖动（休眠体重复唤醒 < 1/s/体） | 岛级休眠 + 传播唤醒 | `pyramid_settles_and_sleeps`（600 tick 后 awake = 0） |
| 确定性 | `Recorder` + `state_hash` | `determinism_same_construction_same_hash` + examples/determinism |

## M0 → M1 升级路径（不换接口）

- 顺序冲量 → TGS-Soft（`ImpulseSolver::solve` 内部替换，岛/缓存层不动）；
- 网格宽相 → 增量 AABB 树（`BroadPhase` trait 不变）；
- SAT → GJK/EPA 通用凸体（`NarrowPhase` trait 不变，SAT 保留为盒特化）；
- FNV-1a → xxh3（`StateHash` trait 不变）；
- 串行 → JobSystem（§6 工作窃取；归约已按索引有序，结果 bit 级不变）；
- 关节族（铰链/球窝/滑块/固定/弹簧/马达/齿轮/齿条/绳索，§2.5）→ 岛内新约束类型；
- CCD 扫掠式（§4.12）→ 求解前 toi 阶段。
