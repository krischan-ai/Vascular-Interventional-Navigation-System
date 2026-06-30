# D0 gate 结果记录（Newton 细杆可行性）

> 对应 [doc/08 §五 D0](../doc/08-导丝物理迁移技术方案-Warp-XPBD-Newton.md) 与 §三.3 gate。
> 脚本：[`d0_rod_gate.py`](d0_rod_gate.py)。日期：2026-06-29。

## 结论：**D0 GATE PASS → 一路 Newton（方案 B）**

细长柔性导丝（亚毫米半径）在 Newton 1.3.0 上：① 60Hz 实时（4.3× 余量），
② 弯曲刚度可标定（单调、宽响应区），③ 接触稳定、不穿地、长度守恒。
doc/08 §三.3 的两个关口（细弹性杆能否搭、能否撞不穿）中第一个已通过。

## 运行环境

- 服务器：`192.168.1.107`（用户 `ps`），Ubuntu 20.04，**RTX A6000 48G**，driver 550.90.07。
- conda env：`cathsim-newton`（Python 3.10）。`~/anaconda3/envs/cathsim-newton`。
- 关键包：**newton 1.3.0、warp-lang 1.14.0、mujoco-warp 3.8.1、newton-actuators 0.1.0**（`newton[sim]`，清华镜像装）。
- 跑法：`conda activate cathsim-newton && python d0_rod_gate.py`。代码在 `~/cathsim-warp/spikes/`。

## 实测数据（L=0.25m，radius=0.40mm，64 段，substeps=10）

| 测试 | 结果 |
|---|---|
| 吞吐 | **255 control-fps**（= 2552 物理步/s），≥60Hz，finite ✅ |
| 悬臂弯曲 bend=1e-1 → 1e3 | tip 下垂 **249mm → 28mm**，水平伸展 43mm → 245mm，单调，跨度 221mm ✅ |
| 长度守恒（最硬档） | 拉伸 **+0.4%**（杆近似刚性） |
| 自由落+落地 | min_z 0.40mm（不穿地）、settled、finite ✅ |

## 关键 API / 配方（迁移到 `newton_engine.py` 直接复用）

Newton 1.3.0 **自带杆原语**，无需手写 XPBD（doc/08 §三.3 的最大风险消解）：

```python
import newton, warp as wp
b = newton.ModelBuilder(); b.rigid_gap = 0.0
b.default_shape_cfg.density = 1800.0   # 导丝材料密度
pts  = newton.utils.create_straight_cable_points(start, direction, length, num_segments)
quats= newton.utils.create_parallel_transport_cable_quaternions(pts, twist_total=0.0)
rod_bodies, rod_joints = b.add_rod(
    positions=pts, quaternions=quats, radius=4e-4,
    stretch_stiffness=1e5, stretch_damping=0.0,   # ← stretch_damping 必须为 0
    bend_stiffness=BEND, bend_damping=1e-1,        # ← BEND 即 cathsim joint stiffness 标定目标
)
b.color(balance_colors=False)
model = b.finalize(); model.set_gravity((0,0,-9.81))
solver = newton.solvers.SolverVBD(model, iterations=5)   # ← 杆用 VBD，不是 XPBD
s0, s1, ctrl, contacts = model.state(), model.state(), model.control(), model.contacts()
# 步进：每控制帧 N substeps
for _ in range(substeps):
    s0.clear_forces(); model.collide(s0, contacts)
    solver.step(s0, s1, ctrl, contacts, dt); s0, s1 = s1, s0
```

- `add_rod` = 胶囊体链 + cable 关节（1 拉伸 DOF + 1 弯/扭 DOF），即离散 Cosserat 杆。
  `joint_type==CABLE(7)`，`bend_stiffness` 存入 `joint_target_ke`（N·m/rad），正是
  cathsim `stiffness_scale` 的标定目标（doc/08 §1.3）。
- `add_rod_graph(node_positions, edges, ...)`：分支杆，对应血管分支图（D2 之后用得上）。
- `ShapeConfig` 自带 SDF 参数（`sdf_target_voxel_size`/`sdf_narrow_band_range`），
  印证 doc/08 §1.4「SDF 碰撞天然密封」是一等公民（D1/D2 碰撞走这里）。

## 踩坑 / 对 doc/08 的修正

1. **杆求解器是 `SolverVBD`，不是 `SolverXPBD`。** doc/08 §二.1/§三 依 Isaac 技能表写
   「SolverXPBD ← cables/ropes」，但 Newton **自带的 cable 示例全部用 VBD**；实测
   XPBD 不驱动 cable 关节的弯曲 `target_ke`（弯曲刚度扫遍 1e-4~1e-1 下垂完全不变）。
   → 迁移用 VBD。建议把 doc/08 相应表述更正。
2. **`stretch_damping` 必须为 0。** 设成 1.0 会让杆「解体」拉伸 +198%（看似 settle 实则散架）。
   默认 0.0 即刚性（+0.0%）。这是本次最大的伪故障点。
3. **锚定一端需清零 4 个数组**：`body_mass / body_inv_mass / body_inertia(mat33 0) /
   body_inv_inertia(mat33 0)`，只清 mass 会飞掉。
4. 亚毫米半径**不是**问题（doc/08 §七 担心的「细杆支持弱」未出现）：r=0.4mm 长度守恒、稳定。
5. Win11 未装 newton（doc/08 §八担心的轮子问题），全程在 Linux+A6000 上跑，环境与最终部署一致。

## 下一步（D1 碰撞）

杆 vs 一段直/弯管，mesh 或 SDF 碰撞，施加 push，验证不穿墙、被接住、快推不穿隧。
碰撞数据来源见 doc/08 §1.4（`visual.stl` 体素化 / `Segmentation.seg.nrrd` 距离变换）。
