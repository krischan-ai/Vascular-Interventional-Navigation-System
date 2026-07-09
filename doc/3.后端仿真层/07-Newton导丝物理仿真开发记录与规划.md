# Newton 导丝物理仿真开发记录与规划

更新时间：2026-07-01

## 1. 文档目的

本文记录当前 11.3 Newton 后端与 Godot 前端联调状态，说明已完成修复、仍存在的问题、问题归属阶段，以及后续开发规划。

本轮诊断的核心结论是：当前系统已经能够在远端服务器运行 `aorta_tree` 的 Newton 后端，并向 Godot 前端输出导丝刚体序列、路径与目标信息；但“转弯穿管”“红色 tip 滞后或卡住”等现象属于导丝物理真实性阶段的问题，不是单纯前端渲染问题。

## 2. 当前部署状态

### 2.1 后端服务器

| 项目 | 当前状态 |
| --- | --- |
| 后端主机 | `192.168.1.107` |
| 运行目录 | `/home/ps/cathsim-warp` |
| 服务端口 | `9000` |
| WebSocket | `ws://192.168.1.107:9000/ws/session` |
| Python 环境 | `cathsim-newton` |
| 物理后端 | `CATHSIM_PHYSICS_ENGINE=newton_demo` |
| GPU | RTX A6000，Newton/Warp 可用 |

启动脚本位于服务器：

```bash
/home/ps/cathsim-warp/start_newton_demo.sh
```

当前脚本关键配置：

```bash
export CATHSIM_PHYSICS_ENGINE=newton_demo
export CATHSIM_NEWTON_SUBSTEPS=10
export CATHSIM_NEWTON_ROD_LENGTH=0.06
export CATHSIM_NEWTON_PUSH_SPEED=0.03
export CATHSIM_PORT=9000
python -m uvicorn services.main:app --host 0.0.0.0 --port 9000
```

### 2.2 Godot 前端

Godot 当前连接远端后端：

```ini
config/server_url="ws://192.168.1.107:9000/ws/session"
```

默认调试体模为：

```gdscript
@export var phantom: String = "aorta_tree"
```

## 3. 本轮已完成修复

### 3.1 确认后端使用 NewtonEngine

已通过 WebSocket 验证 `aorta_tree` 会话返回：

- `engine = NewtonEngine`
- 初始路径约 139 个 waypoints
- 初始导丝约 20 个 bodies
- 可通过 `control` 消息推进

这说明当前远端不再是错误模型或旧版最小 demo，而是已接入本项目 `services/` 代码后的 Newton 版本。

### 3.2 修复分支切换后 Newton 几何缓存不更新

问题：

`aorta_tree` 运行时切换分支后，`NavigationEngine` 只替换了 `_path` 引用，Newton 后端内部缓存的 `_centerline`、碰撞 SDF、导丝 bodies 仍可能来自旧路径。

修复：

- `services/navigation_engine.py`
  - `set_planned_path()` 优先调用后端的 `set_path()`。
- `services/physics/newton_engine.py`
  - 新增 `NewtonEngine.set_path()`。
  - 切换路径时关闭旧 Newton scene，重采样新路径，并重置插入进度。

效果：

分支切换后 Newton 会按新路线重建内部几何，避免“前端路径切了、物理仍走旧管道”的错配。

### 3.3 修复短分支 route 导致 Newton duplicate points 崩溃

问题：

短分支如 `endpoint_0` 路径长度短于默认导丝长度时，采样末端会产生连续重复点，Newton 创建 cable frame 时抛出：

```text
ValueError: points must not contain duplicate consecutive points
```

修复：

- `_resample()` 增加连续重复点去重。
- `_sample_along()` 将采样长度裁剪到中心线总长。
- 采样结果再次去重，避免短路径末端重复点进入 Newton cable 初始化。

验证：

远端 WebSocket 切换 `endpoint_0` 后成功返回：

- `engine = NewtonEngine`
- `bodies = 11`
- `waypoints = 9`

### 3.4 修复前端分支路径不重绘

问题：

Godot `path_renderer.gd` 原先只按 waypoint 数量判断是否重绘。不同分支可能 waypoint 数量相同，导致路径已经切换但前端仍显示旧路径。

修复：

- 增加路径 signature：`waypoint_count + first waypoint + last waypoint`。
- 数量相同但端点不同也会重绘。
- 输出 `[Path] redraw ...` 日志便于现场确认。

### 3.5 增加导丝 root/tip 双端标记

现象：

用户看到“红色球一端卡住，另一端先往前运动”，但前端只显示红色 tip，难以判断哪一端被主动驱动。

修复：

- 红色球：distal tip，仍代表导丝远端。
- 蓝色球：root/proximal end，代表当前 Newton demo 主动推进端。

当前判断：

Newton demo 现在主要驱动 root/proximal body。转弯或受压时，root 可能先前进，而红色 distal tip 滞后、卡住或回弹。这是当前物理模型的真实暴露，不是前端颜色错误。

### 3.6 临时修复摄像机跟随对象

问题：

前端原先跟随 `batch.tip`。当红色 distal tip 卡住时，即使 root 端继续推进，摄像机也不移动，导致用户看不到实际运动端。

修复：

Godot 在 `engine == "NewtonEngine"` 时，临时使用 `bodies[0]` 作为相机跟随目标；其他引擎仍跟随语义 tip。

定位：

这是调试期的观察修复。正式版本仍应回到“远端 tip / 操作视角 / 双视角”可配置跟随策略。

### 3.7 后端首帧 state_batch 补发

问题：

Godot 连接后有时需要等待控制输入才拿到完整 body/path 渲染数据。

修复：

`session_started` 后，如果启用 `batch_mode`，后端立即发送包含 path 的第一帧 `state_batch`。

效果：

前端连接后可立即显示路径、目标、导丝 body 和入口信息。

## 4. 当前问题归类

### 4.1 红色 tip 卡住，root 先运动

阶段归属：当前可观察、后续物理模型需要解决。

当前原因：

- Newton demo 当前采用 root/proximal body 驱动。
- 缺少真实 sheath/导管约束。
- 导丝受压后会发生压缩、屈曲、远端滞后。

本轮已完成：

- 用蓝色 root 标记暴露主动推进端。
- Newton 模式下摄像机临时跟随 root，便于调试。

后续需要：

- 建立 sheath/导管内约束。
- 将推进从“直接移动 root body”升级为更真实的驱动轮/导管-导丝耦合。
- 增加抗屈曲控制策略和稳定接触参数。

### 4.2 摄像机不随运动端移动

阶段归属：当前已做调试期修复，正式交互仍需规划。

本轮已完成：

- Newton 模式临时跟随 root body。

后续需要：

- 增加相机跟随目标切换：tip / root / 中段 / 自由观察。
- 增加分叉处自动视角重定位。
- 增加 X-ray/内窥镜/第三人称双视角或多视角调试。

### 4.3 转弯时导丝穿出血管壁

阶段归属：后续物理真实性阶段的核心开发。

当前原因：

- 当前 Newton demo 的血管碰撞主要是沿中心线生成的简化管腔，不是真实 `aorta_tree` 血管壁。
- 没有基于真实 lumen 的 SDF/碰撞场。
- 急弯和分叉处曲率变化大，简化管腔无法可靠表示真实壁面。
- 缺少 sheath 约束、摩擦、阻尼、软头/硬身分段刚度。
- root 驱动导致导丝受压屈曲，转弯处更容易越过简化碰撞边界。

本轮已完成：

- 修复路线切换后 Newton 几何不重建的问题。
- 修复短分支采样导致的 Newton 初始化崩溃。
- 前端暴露 root/tip，便于判断穿管发生在推进端、远端还是中段。

本轮未完成：

- 未实现真实血管壁 SDF。
- 未实现 variable radius lumen。
- 未实现 sheath/导管约束。
- 未实现完整接触稳定性调参。

结论：

转弯穿管不是单一 bug，而是“真实血管碰撞 + 导丝约束模型 + 接触稳定性”的阶段性任务。

## 5. 近期可继续修复的内容

这些属于短期工程修复或调试增强，可以继续在当前 Newton demo 上推进。

### 5.1 调参缓解穿管

建议：

- 将 `CATHSIM_NEWTON_SUBSTEPS` 从 10 提高到 30-40。
- 降低 `CATHSIM_NEWTON_PUSH_SPEED`。
- 在急弯分支降低推进增益。
- 加大简化管腔安全半径或壁厚，但要避免导丝被过度挤压。

风险：

这些只能降低穿管概率，不能替代真实碰撞模型。

### 5.2 偏离中心线保护

建议增加安全保护：

- 计算每个 body 到当前 route centerline 的最大偏离。
- 超阈值时进入 `COLLISION_WARN` 或 `OUT_OF_LUMEN_WARN`。
- 严重偏离时暂停推进、回退一步或重置到最近合法姿态。

作用：

在真实 SDF 完成前，防止导丝明显穿出后继续前进。

### 5.3 分支路线调试面板

建议 Godot HUD 增加：

- 当前 engine 名称。
- 当前 route target。
- root/tip 弧长进度。
- 最大 path deviation。
- body 数量。
- substeps/push speed。

作用：

现场调试时可以快速判断是后端模型问题、路径问题还是前端显示问题。

### 5.4 远端部署固化

建议：

- 将远端 Newton 启动脚本参数文档化。
- 增加 `systemd` 或一键重启脚本。
- 增加 `/health` 或 WebSocket smoke test 脚本。
- 明确本地 `project.godot` 是连本地还是连远端。

## 6. 下一阶段正式开发规划

### 阶段 A：Newton demo 稳定化

目标：

让当前 `aorta_tree` Newton demo 可稳定启动、切分支、推进、观察，不崩溃、不明显错配。

任务：

- 完成分支切换后 Newton scene 重建。
- 完成短分支采样鲁棒性。
- 完成 root/tip 可视化。
- 完成相机调试跟随。
- 增加偏离中心线保护。
- 增加远端部署 smoke test。

当前状态：

基本完成，仍需补偏离保护和部署固化。

### 阶段 B：转弯防穿透工程缓解

目标：

在真实 SDF 尚未完成前，尽量减少急弯处穿管。

任务：

- 调高 Newton substeps。
- 降低推进速度和急弯增益。
- 根据局部曲率动态限速。
- 根据 body deviation 做暂停/回退。
- 记录穿管发生时的 route、body index、曲率、速度和接触状态。

验收：

- 主路径推进时不出现立即穿管。
- 急弯处即使失败，也能停止或报警，而不是继续穿出血管。

### 阶段 C：真实血管碰撞建模

目标：

用真实 `aorta_tree` lumen 代替中心线合成管腔。

任务：

- 从 `aorta_tree` 血管表面或半径中心线生成真实 lumen SDF。
- 支持 variable radius。
- 支持分叉区域连续碰撞。
- 评估 Newton/Warp 上 SDF 查询性能。
- 将 SDF 与 Godot visual STL 对齐验证。

验收：

- 导丝在急弯和分叉处受到真实壁面约束。
- 穿管概率显著低于简化中心线管。
- body 到壁面距离可用于风险评估。

### 阶段 D：sheath/导管约束与导丝分段材料

目标：

解决 root 直接驱动造成的压缩、屈曲和远端滞后。

任务：

- 建立导管/sheath 几何约束。
- 区分软头、硬身、不同弯曲刚度和扭转刚度。
- 建立推进轮/导管-导丝耦合模型。
- 引入摩擦、阻尼、接触稳定参数。

验收：

- 推进力能更自然地传递到远端。
- 红色 tip 不再长期卡住而 root 单独前进。
- 转弯时出现合理贴壁、滑动、回弹，而不是直接穿管。

### 阶段 E：性能与 GPU 利用优化

目标：

提高 GPU 利用率和仿真吞吐。

任务：

- 减少 Python 到 CPU 的 `.numpy()` 同步。
- 将更多状态更新留在 GPU。
- 评估 CUDA graph capture。
- 批量化 SDF/contact 查询。
- 将调试输出和渲染数据降采样。

说明：

当前 GPU 已被 Newton/Warp 使用，但模型规模较小，且 Python 同步开销高，因此显卡占用不会很高。这不是“没有使用 GPU”，而是当前仿真规模和数据同步方式限制了 GPU 利用率。

## 7. 当前完成情况汇总

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 远端 Newton 后端启动 | 已完成 | `192.168.1.107:9000` 可用 |
| Godot 连接远端 | 已完成 | `project.godot` 指向远端 WebSocket |
| `aorta_tree` Newton 会话 | 已完成 | 返回 `NewtonEngine` |
| 初始 state_batch | 已完成 | 会话启动后立即发送 |
| 分支切换路径重建 | 已完成 | `NewtonEngine.set_path()` |
| 短分支 duplicate points | 已完成 | 采样裁剪与去重 |
| 路径重绘 | 已完成 | endpoint signature 判断 |
| root/tip 双端显示 | 已完成 | 蓝色 root，红色 tip |
| Newton 相机临时跟随 root | 已完成 | 调试期修复 |
| 转弯穿管根治 | 未完成 | 属真实碰撞/约束模型阶段 |
| sheath/导管约束 | 未完成 | 后续正式物理模型 |
| 真实 lumen SDF | 未完成 | 后续核心任务 |
| GPU 利用优化 | 未完成 | 后续性能阶段 |

## 8. 下一步建议

建议下一轮开发按以下顺序推进：

1. 增加 body 到 centerline 的偏离检测和穿管报警。
2. 将远端 substeps 提高到 30-40，降低 push speed，做一轮急弯稳定性对比。
3. 在 Godot HUD 显示 engine、route、root/tip progress、max deviation。
4. 开始真实 `aorta_tree` lumen SDF 或 variable-radius 管腔建模。
5. 设计 sheath/导管约束，替代当前 root 直接驱动。

当前最重要的工程边界是：

短期可以把 Newton demo 调到“可观察、可定位、失败可控”；但要让转弯不穿管、导丝运动像真实导丝，必须进入真实血管碰撞和导丝约束建模阶段。

## 9. D2/D3 完成记录（真实腔碰撞 + 最小阶段 D 驱动 + 集成上线）

更新时间：2026-07-01。全程在 A6000 服务器（`cathsim-newton` 环境，Newton 1.3.0）验证。对应 `doc/08` 的 D2/D3 阶段。§4.3 的“转弯穿管”本轮已根治并集成上线。

### 9.1 诊断：穿管的真正根源是“驱动”，不是“墙”

对真实 `segment_part` 腔（`Segmentation.seg.nrrd` 做 ground-truth signed field）与 `aorta_tree` 路线逐项实验后确认：

- **静置**时真实腔壁能稳定接住导丝（segment_part 离线验证 breach −0.41mm）。
- **穿管发生在推进过程**：原来“沿弯曲中心线瞬移单根 root 刚体 + 其余自由”的粗糙驱动，会在弯道把推送吸收成关节转动、把自由尖端甩出或隧穿。这正是 §4.1“红色 tip 卡住、root 先动”的同一问题，属**阶段 D（sheath/导管-导丝耦合 + 软头硬身）**，非碰撞墙问题。
- 加硬身管刚度（bend 50→5000）对推送传导**无改善**，进一步印证是驱动模型问题。

### 9.2 攻下的可行配方

- **碰撞墙（真实腔）**
  - `aorta_tree`：其 `visual.stl` 粗糙（7.5k 面）且末端开口，不适合直接做 SDF。改用各路线 `routes.json` 自带的真实 VMTK 内切半径 `radius_m`，沿路线构造**变半径厚壁环管**（内/外双环，`newton.Mesh(is_solid=True).build_sdf`）——水密、真实、已居中。
  - `segment_part`：有干净水密 `visual.stl`（180k 面）+ 分割体，可用**真实网格厚壁环管**并用 signed field 梯度上升把中心线居中。
  - 踩坑：单面网格 `is_solid=True` 会把“腔内”当实体、把导丝挤出（与绕序无关）；必须构造有界厚壁环管（腔=自由/正、壁=实体/负）。`lumen_band` 需覆盖满局部最大半径，否则宽腔（如主动脉根部 12mm）中心处 SDF 未定义，尖端隧穿。
- **驱动（最小阶段 D 模型）**：**graded soft-anchor**。近端“已插入段”贴合居中路线（权重 alpha=1），远端 `free_span=6` 节做软锚坡道（alpha→`tip_alpha=0.3`）；每 substep 物理步进后 `pos = (1-alpha)*phys + alpha*target`，target 为路线上 `base_arc[j] + insert_s` 处的点。插入行程封顶到路线末端。既消除穿管、又保留软头物理。
- **吞吐**：profiling 定位瓶颈是 `solver.step`（VBD，占 79%），**不是 numpy 同步（仅 2%）**。graded anchor 使 VBD 在 **6 substeps × 2 iterations** 即稳定，从 ~4fps 提到 **~58fps**（跨 4 条路线，2× 推速仍不穿管）。

### 9.3 集成（D3，上层零改写）

- `services/physics/base.py`：`PlannedPath` 携带可选 `radii`，新增 `radius_at_arclen`。
- `services/navigation_engine.py`：新增 `_route_radii` 取 `radius_m`，经 `set_planned_path(..., radii=)` 注入（构造默认路线 + `select_route` 换支）。
- `services/physics/newton_engine.py`：`_build_tube_mesh` 改为变半径环管（无半径时回退常半径）；`step()` 改为 insert_s 推进 + 6-substep graded soft-anchor；`set_path` 换路重建；`_raw_pose` 按局部变半径算 wall_distance/contact_force，`arclen` 报告尖端精确弧长。
- 新增环境变量：`CATHSIM_NEWTON_{ITERS,SDF_VOXEL,LUMEN_BAND,MIN_RADIUS,FREE_SPAN,TIP_ALPHA}`；默认即 D3 验证配置。

### 9.4 验证结果

- **引擎直测 / NavigationEngine（aorta_tree）**：endpoint_9/6/0 全部 **contact_force=0（无穿管）**、wall_distance>0、**~46–51 control-fps**、tip 推进到目标、换支重建正常。
- **WebSocket 全链路**：`engine=NewtonEngine`、20 节 body、`path/safety/episode` 协议完整、tip 推进 68.5mm、无穿管。
- **segment_part 常半径回退**：正常渲染、无回归。
- **上线**：远端后端已用该配置重启（`start_newton_demo.sh`：`SUBSTEPS=6`、`PUSH_SPEED=0.05`），经 `run_bg.sh` 脱离会话运行。

### 9.5 仍属阶段 D 的后续

graded anchor 是消除穿管的**偏运动学过渡方案**，未建模真实推送传导（tip 滞后/屈曲、软头硬身分段刚度、推进轮/sheath 动力学）。后续阶段 D 用真实力驱动 + 导管约束替代软锚，才能让导丝“推得像真的”。相关配方与踩坑见记忆 `d2-d3-newton-lumen-recipe` 与 `spikes/D2_RESULTS.md`。
