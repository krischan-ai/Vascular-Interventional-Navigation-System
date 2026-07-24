# D2 gate 结果记录（真实血管腔碰撞）

> 对应 doc/08 §五 D2。脚本：spikes/d2_visual_stl_sdf.py。日期：2026-07-01。

## 结论：D2 碰撞/包容判据 PASS（真实腔壁不穿墙、可 60Hz 化）；推送传导留待阶段 D

### 攻下的核心（方案 P2：真实网格厚壁环管）
- 碰撞墙 = 真实 segment_part/visual.stl（水密，180k tris），沿顶点法向外扩 wall_t 成**厚壁环管实体**，
  是 Newton mesh-SDF 唯一能把导丝**关在腔内**的形态（裸单面会把导丝挤出，见下）。
- 中心线用分割体 Segmentation.seg.nrrd 的 signed field 做梯度上升**自动居中**：
  p50 +1.48mm/97%在腔内 → **+5.45mm/100%在腔内**。
- 静置测试：导丝被真实腔壁稳定接住，**breach −0.41mm（不穿墙）**；bend=200 时弯道 worst 仅 +0.28mm。

### 关键踩坑（对后续/文档的修正）
- **裸 visual.stl 单面 is_solid=True 会把"腔内"当实体**，导丝被挤出（两种绕序都一样，符号来自被包围体积，与绕序无关）。
  必须构造"腔=自由/正、壁=实体/负"的**有界厚壁环管**（内面反绕 + 外面法向外扩）。
- 薄壳会让快推导丝穿隧后被对面正 SDF 弹出 —— 壁要够厚（用 wall_t≈4mm 即可，这里逐步位移仅 0.02mm/substep）。
- 原 d2 变半径管失败根因：分割体距离变换在**偏心**平滑中心线上取值 → 半径被夹到 1.5mm 地板、管过细。居中后消除。

### 仍未达标（= 阶段 D，非 D2 范畴）
- 推送传导衰减：根部推进 34.6mm，尖端仅 3-5mm，**与 bend 刚度无关**（50/200/1000/5000 都一样）。
  根因是"沿弯曲中心线瞬移单个 root 刚体"在弯道把推送吸收成关节转动 —— 即 doc/07 §4.1 的 tip 滞后，
  属阶段 D（sheath/导管-导丝耦合 + 软头硬身 + 推进轮）。
- 吞吐 4.2 fps：瓶颈是每 substep 的 numpy↔GPU 同步 + 每次重算分割体距离变换。需缓存 DT + root 驱动上 GPU。

### 实测（L=0.06m rod, ss=40, wall_t=4mm, sdf_voxel=0.4mm）
| bend | 根部 | 尖端 | steady breach | worst |
|---|---|---|---|---|
| 50   | 34.6mm | 4.7mm | +0.83mm | +1.86mm |
| 200  | 34.6mm | 5.1mm | −0.12mm | +0.28mm |
| 1000 | 34.6mm | 4.0mm | +0.51mm | +1.41mm |
| 5000 | 34.6mm | 3.2mm | −0.61mm | −0.27mm |

## 下一步：D3 集成（把真实网格环管 SDF + 居中搬进 newton_engine.py，替换常半径管）

---

## 追加（2026-07-01）：D2→D3 完整可行配方（aorta_tree，跨 4 条路线 PASS）

发现穿管根源是**驱动**而非墙：静置包容 OK，但瞬移单根/自由尖端在推进时穿墙（=阶段D）。
最小阶段 D 驱动模型验证通过：

- **墙**：沿路线用 routes.json 的真实 VMTK `radius_m` 建**变半径厚壁环管**（aorta_tree 的 visual.stl 太糙且开口，
  不适合直接做 SDF；segment_part 才用真实网格环管）。inner/outer 双环，is_solid，build_sdf。
- **驱动（graded soft-anchor）**：近端"已插入段"贴合居中路线（alpha=1），远端 free_span=6 节做软锚坡道
  （alpha→tip_alpha=0.3），每 substep 物理步进后把 body 位置向居中目标 blend：
  `pos = (1-alpha)*phys + alpha*target`。插入行程封顶到路线末端。
- **参数**：bend=50, wall_t=6mm, sdf_voxel=0.5mm, lumen_band=15mm(需覆盖满局部半径,否则宽腔尖端穿隧),
  substeps=40, push=0.05m/s, free_span=6, tip_alpha=0.3。
- **结果**：endpoint_0/3/6/9 全部 PASS，steady breach −1.4~−11.4mm（负=不穿墙），tip 推进 57–68mm。
- **待办**：吞吐仅 ~4fps（瓶颈=每 substep 的 numpy↔GPU 同步 + python point_at_s）。集成前/中需优化。

脚本：spikes/d2_aorta_tree_tube.py（tube+graded-anchor）、spikes/d2_visual_stl_sdf.py（segment_part 真实网格环管，GT 验证）。
