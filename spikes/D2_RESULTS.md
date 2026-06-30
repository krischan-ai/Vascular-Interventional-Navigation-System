# D2 结果记录（segment_part 真数据半径管腔）

> 对应 `doc/08` 阶段 D2。脚本：[`d2_segment_part_radius_tube.py`](d2_segment_part_radius_tube.py)。日期：2026-06-29。

## 结论：D2 **物理可行性部分通过**，端到端 60Hz 尚未通过

本轮没有直接拿 `visual.stl` 做 solid SDF，因为该 STL 更像“填满的血管体积表面”，直接碰撞会把导丝当作在实心体内部，方向容易反。采用更稳的 D1 配方：从真实 `Segmentation.seg.nrrd` 做距离变换，沿真实 `segment_part/centerline.json` 采样半径，再生成变半径厚壁管腔 SDF。

已确认：

- NRRD 轴序：`order="F"` 正确；中心线点 `87.35%` 落在 mask 内。`order="C"` 为 0%，不可用。
- 预穿线零推进稳定：`steady=-0.100mm`，0% breach，finite。
- 真实分割半径管腔可稳定推进短导丝段：

```text
rod_length=0.06m, bend=1, push=0.05m/s, frames=60, substeps=30, radius_margin=0
tip_adv=10.2mm  steady=+0.491mm  worst=+0.96mm  breach_frac=35%  finite=True
D2 RADIUS-TUBE GATE: PASS
```

边界：

- `rod_length=0.10m`、`radius_margin=0`：`steady=+0.679mm`，亚毫米级但未过当前 0.5mm 严格门槛。
- `rod_length=0.14m`：`steady≈1.3–1.7mm`，提高 `contact_ke` 到 `1e7` 只小幅改善，主因更像固定长度长杆近端推进导致的压缩屈曲，而不是墙太软。
- 如果沿入口切线直推 root，导丝会被真实入口弯曲推出；必须用中心线/入口路径驱动 proximal 控制点，或在 D3 实现真正“插入新增弧长”的边界条件。

## 性能状态

当前 D2 脚本是 Python spike，每个 substep 都通过 `state.body_q.numpy()` 修改 root，无法 CUDA graph capture：

```text
throughput=5.4 control-fps (163 phys-steps/s)
```

所以 D2 的“真腔碰撞 + 推进稳定”已有可行基线，但“60Hz 端到端”尚未通过。性能问题应在 D3/NewtonEngine 里用 GPU/warp-side root target 更新、CUDA graph capture 或降低 Python 循环开销解决，不能把当前 spike 的 5.4Hz 当作 Newton 求解器上限。

## 关键修正

1. `visual.stl`/填实体 SDF 不宜直接作为内腔碰撞；优先走二值分割距离场 → 变半径厚壁管。
2. 半径采样不应额外扣 `0.5mm` 作为默认；对 segment_part 这段真数据会过紧。保守 margin 可作为 sweep 参数。
3. breach 测量必须用点到中心线**线段**投影并插值半径，不能用最近顶点；最近顶点会把轴向误差误算成径向穿壁。
4. 固定长度杆被近端推进会压缩屈曲；D3 需要实现更真实的插入边界条件，而不是简单移动第一节刚体。

## 下一步

- D3 前先做一个小型 `NewtonEngine` 原型：复用 D2 的半径管腔构建，但把 root target 更新搬到 GPU/可 capture 路径，验证 60Hz。
- 插入边界条件从“移动 root body”升级为“按插入弧长重置/释放近端节点”或增量生成有效导丝长度，减少长杆压缩屈曲。
- 半径来源继续改进：优先利用 Slicer centerline 的半径标注（文档提到 2.1–13.1mm），当前 mask EDT 对 `segment_part` 中线半径偏小（p50≈1.57mm）。
