# 血管介入仿真数字孪生平台 - 总体技术方案

> 版本：v2.0 | 更新日期：2026-07-08  
> 基线来源：`doc/06`-`doc/13` 按时间持续迭代后的当前方案  
> 核心结论：平台已从早期 “CathSim(MuJoCo) + VPP + Godot” 融合原型，演进为 **Newton 物理底座 + ShapeIntent 控制层 + HCI/RL 双模式一体化平台 + 医疗导航工作站前端**。

---

## 一、项目定位

本项目构建面向血管介入导丝/导管导航的高保真数字孪生仿真平台，覆盖：

- **术前规划**：血管中心线、分支图、全局路径规划和目标分支选择。
- **术中导航**：人机交互、点击导航、手动/自动/策略辅助控制和实时风险反馈。
- **物理验证**：Newton GPU 物理导丝、真实腔碰撞、导丝力驱动、扭转/J-tip 转向和抗屈曲。
- **训练评估**：Gymnasium/SB3 强化学习接口、会话录制回放、评分和后续 Isaac Lab 提升。
- **医学可视化**：Godot 医疗工作站 UI、3D 玻璃血管导航视图、DSA/X-ray 风格叠加和安全 HUD。

### 1.1 总体设计原则

| 原则 | 当前方案 |
|---|---|
| 物理保真 | MuJoCo 保留为兼容/历史引擎，主线采用 Newton/Warp GPU 物理；通过 `PhysicsEngine` 协议隔离引擎实现。 |
| 控制解耦 | Human 与 RL 均下达 `ShapeIntent`，由 `ShapeIntentController` 解算为真实 `push/rotate`，物理层保持纯物理。 |
| 共享核心 | HCI 与 RL 共用 `NavigationEngine + ShapeIntentController + PhysicsEngine + PathPlanner`，只在入口和出口分叉。 |
| 数据真实 | 路径、半径、碰撞、风险区必须来自真实中心线/SDF/后端字段；禁止用 mock 红色禁入区表达医学语义。 |
| 渲染与物理解耦 | 视觉使用高质量 GLB/Shader/相机；碰撞使用 Newton/SDF/厚壁环管；两类资产分离。 |
| 可演进 | 当前以 Newton 独立后端最快迭代；未来 RTX 渲染、Isaac Lab、ROS/数字孪生通过 Isaac Sim 提升，而非重写。 |

---

## 二、总体架构

```text
┌──────────────────────────────────────────────────────────────────────┐
│                         Godot 医疗导航客户端                          │
│  HCI 入口：键盘/手柄/点击导航/形变调参                                 │
│  可视化：医疗工作站 UI + 3D 玻璃血管 + DSA/X-ray 叠加 + HUD              │
└───────────────────────────────┬──────────────────────────────────────┘
                                │ WebSocket / REST
                                ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         FastAPI 平台服务层                            │
│  SessionManager / WebSocketHandler / Path API / Health / Replay        │
│  HCI 模式：实时 session                                                │
│  RL 模式：Gym env 进程内复用核心模块                                    │
└───────────────────────────────┬──────────────────────────────────────┘
                                ▼
┌──────────────────────────────────────────────────────────────────────┐
│                           共享核心 Shared Core                         │
│  ShapeIntentController                                                │
│      ShapeIntent(target_direction / target_waypoint / intensity)       │
│      → push / rotate                                                   │
│  NavigationEngine                                                      │
│      progress / deviation / curvature / risk / safety / reward fields  │
│  PathPlanner                                                           │
│      graph A* / route switching / B-spline smooth path                 │
└───────────────────────────────┬──────────────────────────────────────┘
                                ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         PhysicsEngine 抽象层                           │
│  KinematicEngine    ：guided 演示/可达性                               │
│  MuJoCoEngine       ：历史兼容/对照                                    │
│  NewtonEngine       ：当前主线，GPU 物理导丝 + 真实腔碰撞                 │
│  future Warp/Isaac  ：满足同一协议后灰度替换                            │
└───────────────────────────────┬──────────────────────────────────────┘
                                ▼
┌──────────────────────────────────────────────────────────────────────┐
│                           数据与资产管线                               │
│  VPP / 3D Slicer / VMTK → centerline / graph / routes / radius          │
│  STL/GLB 视觉资产、SDF/厚壁环管碰撞资产、训练与回放数据                  │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.1 分层职责

| 层级 | 主要模块 | 职责 |
|---|---|---|
| Godot 客户端 | `main_controller.gd`、`guidewire_renderer.gd`、`path_renderer.gd`、HUD/UI | 实时交互、导丝/血管/路径渲染、点击导航、参数调节、状态显示。 |
| FastAPI 服务 | `services/main.py`、`websocket_handler.py`、`session_manager.py` | WebSocket 会话、REST API、状态流、路径请求、策略加载入口。 |
| 控制层 | `services/shape_intent.py`、`physics_autopilot.py` | 将 Human/RL 的高层意图统一解算为底层 `push/rotate`。 |
| 编排层 | `services/navigation_engine.py` | 统一路径、物理、风险、progress、deviation、safety、reward 派生。 |
| 物理层 | `services/physics/base.py`、`newton_engine.py`、`mujoco_engine.py`、`kinematic_engine.py` | 引擎协议与实现；Newton 为当前高保真主线。 |
| 路径规划 | `services/path_planner.py`、graph/routes 数据 | A* 图搜索、B-spline 平滑、分支目标切换。 |
| 训练层 | `src/cathsim/gym/`、SB3 | Newton Navigation Gym env、PPO/SAC、curriculum、domain randomization。 |
| 资产工具 | `tools/export_godot_assets.py` 等 | 高质量视觉 GLB、preview GLB、SDF/碰撞资产、法线/平滑/LOD。 |

---

## 三、物理引擎基线

### 3.1 当前结论

早期方案以 MuJoCo 为核心，但 `aorta_trunk` 高节数导丝在单核串行 `mj_step` 下存在约 3Hz 控制延迟地板，无法靠堆硬件解决。当前主线已经切换为：

- **Newton 独立后端优先**：底层仍是 Warp/CUDA，可向 Isaac Sim/Isaac Lab 提升。
- **`PhysicsEngine` 协议隔离**：上层不依赖 Newton API，后续可灰度替换。
- **NewtonEngine 已上线验证**：D0-D5 阶段完成，支持 60Hz 级导丝物理、真实腔碰撞、力驱动、J-tip/扭转控制、autopilot 和抗屈曲。

### 3.2 PhysicsEngine 协议

```python
class PhysicsEngine(Protocol):
    def reset(self) -> RawPose: ...
    def step(self, push: float, rotate: float) -> RawPose: ...
    def render_bodies(self) -> list[dict]: ...
    def close(self) -> None: ...

    @property
    def control_timestep(self) -> float: ...
    @property
    def planned_path(self) -> PlannedPath: ...
```

`RawPose` 只暴露跨引擎稳定量：tip 位姿、body 位姿、接触力、壁距、目标、弧长等。`progress/deviation/curvature/risk/safety` 等派生量保留在 `NavigationEngine`，避免引擎实现污染上层算法。

### 3.3 NewtonEngine 当前能力

| 能力 | 状态 | 说明 |
|---|---|---|
| GPU 导丝物理 | 已完成 | Newton 1.3.0，实测满足 60Hz 级控制判据。 |
| 真实腔碰撞 | 已完成 | aorta_tree 使用带半径 routes 构建变半径厚壁环管；segment_part 使用高质量网格/SDF 路线。 |
| 上层零改接入 | 已完成 | `newton_engine.py` 实现 `PhysicsEngine`，`NavigationEngine` 和 WebSocket 协议保持稳定。 |
| 真实力驱动 | 已完成 | 替代 D3 graded soft-anchor 的运动学过渡方案。 |
| 扭转/J-tip 转向 | 已完成 | 与 `PhysicsAutopilot`/ShapeIntent 控制链路联动。 |
| 抗屈曲/sheath 约束 | 已完成并验证 | 通过 slack guard 和 sheath 自动模式减少自由段柱屈曲。 |
| 风险判定分模式 | 已完成 | force physics 模式按穿透量/接触刚度判定，避免正常贴壁误报。 |
| 后续真实导管/sheath | 规划中 | 更真实的推送传导、软头硬身材料和导管-导丝耦合仍是后续物理主线。 |

### 3.4 物理模式语义

| 模式 | 引擎 | 语义 | 用途 |
|---|---|---|---|
| `guided` | `KinematicEngine` | 沿中心线运动学覆盖，低保真。 | 演示、路径可达性、前端调试。 |
| `physics` | `NewtonEngine` | 力驱动 + 真实碰撞，高保真。 | 人机交互、算法验证、训练。 |
| `rl` | `NewtonEngine` | 策略输出 ShapeIntent，高保真。 | 策略推理、AI 辅助驾驶。 |
| `mujoco` | `MuJoCoEngine` | 历史兼容/对照。 | 回归、旧数据复现。 |

---

## 四、控制层：ShapeIntent

### 4.1 设计目标

ShapeIntent 是当前平台的人机交互与强化学习统一控制抽象。它不是物理力场，不直接拉拽导丝各段位置，而是高层意图：

```text
Human / RL Policy
    → ShapeIntent(target_direction | target_waypoint | intensity)
    → ShapeIntentController
    → push / rotate
    → PhysicsEngine.step(push, rotate)
```

这样可以同时满足：

- 手动驾驶、点击导航、自动循线、RL 策略共享同一控制接口。
- 物理引擎只接收真实 2-DOF 控制量，不被策略层污染。
- RL 学到的策略可直接加载到 HCI 模式作为辅助驾驶。

### 4.2 ShapeIntent 数据模型

```python
@dataclass
class ShapeIntent:
    target_direction: np.ndarray | None = None
    target_waypoint: np.ndarray | None = None
    intensity: float = 1.0
```

映射关系：

| 来源 | ShapeIntent |
|---|---|
| 鼠标点击血管/路径 | `target_waypoint` |
| 手柄摇杆 | `target_direction` |
| RL policy | `target_direction + intensity` 或 `waypoint_offset` |
| 默认 autopilot | Controller 内部前视点，不需要外部 intent |

### 4.3 Controller 契约

`ShapeIntentController` 泛化自当前 `PhysicsAutopilot`，核心控制律包括：

- look-ahead 朝向误差。
- J-tip 方位不可观测下的爬山式旋转符号搜索。
- 接触力门控推速。
- stall 扫掠/回拉。
- waypoint/方向目标 override。

`intent=None` 时必须逐帧退化为原有 autopilot 行为，作为回归基线。

---

## 五、HCI/RL 一体化平台

### 5.1 双模式共享核心

```text
HCI 模式：
Godot → WebSocket → SessionManager → NavigationEngine → NewtonEngine → state_batch → Godot

RL 模式：
Gymnasium Env → NavigationEngine → NewtonEngine → obs/reward/done → SB3
```

两种模式共享：

- 同一 `NewtonEngine`。
- 同一 `ShapeIntentController`。
- 同一 `NavigationState`。
- 同一 `PathPlanner` 和中心线/半径/分支图资产。

### 5.2 HCI 模式

| 交互 | 消息/API | 行为 |
|---|---|---|
| 键盘/手柄 | `control(delta_push, delta_rotate)` | 手动直通或更新 latest input。 |
| 点击导航 | `shape_intent(target_waypoint)` | 自动朝目标 waypoint 推进。 |
| ESC/手动介入 | `shape_intent(active=false)` | 退出自动控制。 |
| 分支切换 | `select_route(target)` | 切换 `routes.json` 目标并重建 planned path。 |
| 形变调参 | `engine_params` | 在线调整 sheath、slack、刚度等参数。 |
| 状态流 | `state_batch` | bodies/path/tip/safety/seq/t_phys。 |

### 5.3 RL 模式

近期目标是新增 Newton Navigation Gym env，复用现有 wrappers 和 SB3 管线。

| 项 | 方案 |
|---|---|
| Observation | tip 位姿、方向、progress、deviation、contact_force、wall_distance、curvature、risk、局部 tangent 等。 |
| Action | 推荐 `ShapeIntent`：`desired_direction(3) + intensity(1)`；保留 `direct push/rotate` baseline。 |
| Reward | 以 `Δprogress` 为主项，加对齐奖励、接触/风险惩罚、到达/失败终止。 |
| Curriculum | `endpoint_0 → endpoint_9 → endpoint_3 → 多目标/全树`。 |
| 并行 | 近期多进程；远期提升到 Isaac Lab 千环境。 |

### 5.4 模式显式化

所有 API/HUD/state_batch 必须明确当前模式：

```text
guided | physics | rl
```

避免把低保真 guided 演示误当作真实物理，也避免把正常贴壁误判为穿壁。

---

## 六、路径规划与血管图

### 6.1 segment_part 图规划结论

`doc/06` 的核心结论是：单条 B 样条中心线不能支撑多分支导航和 sim2real 平滑推进，应从 `segment_part.stl` 或分割体中提取完整中心线图：

```text
segment_part.stl / segmentation
    → voxelize + skeletonize
    → junction/endpoints
    → branch clustering
    → per-branch downsample + B-spline smooth
    → graph.json + centerline_branch_*.json
    → NavigationEngine.plan_to_target()
```

### 6.2 路径数据格式

| 资产 | 用途 |
|---|---|
| `centerline.json` | 主路径、渲染、progress/deviation 投影。 |
| `centerlines/*.json` | 分支中心线。 |
| `graph.json` | A* 拓扑图。 |
| `routes.json` | aorta_tree 多目标路径及半径 `radius_m`。 |
| `visual.stl/glb` | 渲染资产。 |
| `Segmentation.seg.nrrd` / SDF | 碰撞/距离场资产。 |

### 6.3 规划服务

`PathPlanner` 负责：

- 起终点映射到图节点。
- A* / Dijkstra 搜索。
- B-spline 平滑。
- 生成 `PlannedPath`，提供弧长、局部 tangent、半径插值。
- 支持分支切换后通知 `NewtonEngine.set_path()` 重建内部几何。

---

## 七、风险与安全体系

### 7.1 状态等级

| 状态 | 语义 |
|---|---|
| `STANDBY` | 会话启动/复位，未开始控制。 |
| `SAFE_NAV` | 正常导航。 |
| `DANGER_WARNING` | 接近阈值或轻度穿透/风险升高，允许受控调整。 |
| `COLLISION_STOP` | 严重穿透/碰撞/越界，禁止继续推进或触发回退。 |

### 7.2 模式化风险判定

| 模式 | 判定依据 |
|---|---|
| guided | 以 wall distance、路径偏差、曲率、速度等启发式为主。 |
| Newton force physics | 以 `contact_force / contact_ke` 推算穿透量，正常贴壁不应全程报 `COLLISION_STOP`。 |
| 后续真实 risk_regions | 必须携带 `source`、空间位置和类型字段；前端只渲染真实来源。 |

### 7.3 真实风险区契约

前端只接受后端真实空间风险字段：

```json
{
  "id": "stenosis_001",
  "level": "warning|danger",
  "kind": "stenosis|collision|no_go|high_curvature",
  "center": [0.0, 0.0, 0.0],
  "radius": 0.008,
  "orientation": [0.0, 0.0, 0.0, 1.0],
  "extent": [0.008, 0.008, 0.008],
  "source": "sdf|annotation|planner"
}
```

缺少 `source` 或空间字段的 placeholder 不得渲染成红色/橙色体积。

---

## 八、前端总体方案

### 8.1 医疗导航工作站 UI

Godot 前端应从调试 HUD 演进为医疗机器人导航工作站：

```text
1920x1080
├─ TopStatusBar      机器人状态 / 导航模式 / 路径进度 / 半径 / 曲率 / 壁距 / 风险 / 急停
├─ MainWorkspace
│  ├─ DSA 实时影像区  灰度医学影像 + 导管/路径/目标/图例/工具栏叠加
│  └─ 右侧导航区
│     ├─ 3D 血管导航  玻璃血管 + 导丝 + 路径 + 相机工具
│     └─ 导航安全数据 2x4 数据卡
└─ BottomControlBar  系统状态 / 机器人连接 / 运动控制 / 日志 / 告警
```

视觉基调：

- 深色医疗工业风。
- 面板色 `#101A26`，背景 `#071019`。
- 强调蓝 `#2F8CFF`，安全绿 `#4EE66B`，警告黄 `#FFD447`，危险红 `#FF4D4F`。
- 禁止默认 Godot 灰色控件、emoji 图标和游戏 HUD 风格。

### 8.2 3D 玻璃血管导航

`doc/12-13` 的当前方向：

| 主题 | 方案 |
|---|---|
| 几何 | 使用 `segment_part` 或 VPP 高质量血管 GLB；新增 `visual_high` 与 `preview` 资产分层。 |
| 材质 | 半透明核心 + Fresnel 边缘高光 + bloom + 距离衰减，不再整体 cyan 发光。 |
| 相机 | 默认 `Clinical Orbit`，围绕导丝 tip 近景斜俯视；全树只作为 overview。 |
| 景深 | tip 附近清晰，远端分支暗化/虚化。 |
| 路径/导丝 | 作为细亮导航 overlay，不压过血管本体；tip 标记小型高对比。 |
| 风险 | 无真实 `risk_regions` 时不显示红色禁入体积。 |

### 8.3 视觉资产策略

| 类型 | 文件建议 | 用途 |
|---|---|---|
| 高质量视觉 mesh | `*_visual_high.glb` | 最终医疗视图验收。 |
| 预览 mesh | `*_preview.glb` | 调试/低配预览。 |
| 碰撞/SDF 资产 | `collision_sdf`、厚壁环管、分割体 | Newton 物理碰撞。 |

`tools/export_godot_assets.py` 后续应支持：

```text
--quality visual_high|preview
--max-faces 300000
--no-decimate
--smooth-normals
--taubin-smooth-iter N
```

---

## 九、API 与通信

### 9.1 REST

| 端点 | 用途 |
|---|---|
| `GET /api/v1/health` | 健康检查。 |
| `GET /api/v1/assets/cases` | 列出可用 case/phantom。 |
| `POST /api/v1/path/plan` | 路径规划。 |
| `POST /api/v1/session/start` | 创建 session。 |
| `POST /api/v1/session/{id}/step` | 单步推进。 |
| `POST /api/v1/session/{id}/reset` | 重置。 |
| `DELETE /api/v1/session/{id}` | 关闭 session。 |

### 9.2 WebSocket `/ws/session`

| 消息 | 方向 | 说明 |
|---|---|---|
| `session_start` | C→S | phantom、target、route、mode、batch_mode。 |
| `session_started` | S→C | session_id、初始 state、routes。 |
| `control` | C→S | `delta_push`、`delta_rotate`。 |
| `shape_intent` | C→S | 点击导航/方向导航/强度。 |
| `select_route` | C→S | 切换分支目标。 |
| `engine_params` | C→S | Newton/sheath/slack/形变参数。 |
| `path_request` | C→S | 起终点规划请求。 |
| `state_batch` | S→C | bodies、path、tip、state、safety、seq、t_phys。 |
| `state_update` | S→C | 轻量状态更新。 |
| `reset` | C→S | 重置会话。 |

---

## 十、部署架构

### 10.1 组件分布

| 组件 | 运行位置 | 说明 |
|---|---|---|
| Godot 客户端 | 本地原生 Windows/Mac/Linux | 显示、输入、交互。 |
| FastAPI + Newton 后端 | Linux + NVIDIA GPU Docker | A6000 服务器为主，本地 WSL2/3060 可开发。 |
| RL 训练 | 同一 Docker 镜像 | Headless Gym/SB3。 |
| 模型/回放/训练日志 | 挂载卷 | 策略权重、TensorBoard、回放轨迹。 |

### 10.2 工作流

1. **远程开发推荐**：VS Code Remote-SSH 到 A6000 服务器，Godot 本地连接远端 WebSocket。
2. **本地容器开发**：WSL2 + Docker Desktop + GPU 直通，运行同一镜像。
3. **生产部署**：`docker compose up` 启动后端；Godot 只切换 `server_url`。

### 10.3 Isaac Sim 演进

当前不直接以完整 Isaac Sim 作为运行时。触发条件是以下任一成为现行目标：

- RTX 拟真渲染/合成数据。
- Isaac Lab 千环境 RL。
- ROS2 桥/真实机器人数字孪生。

届时将 `NewtonEngine` 场景提升到 `isaacsim.physics.newton`，底层 Newton 物理不重写。

---

## 十一、当前实施路线图

### 11.1 物理轨道 D

| 阶段 | 状态 | 内容 |
|---|---|---|
| D0-D3 | 已完成 | Newton gate、真腔碰撞、`NewtonEngine` 接入、上层零改。 |
| D4-D5 | 已完成 | 真实力驱动、扭转/J-tip、autopilot、ShapeIntent 链路。 |
| D4 hardening | 已验证 | 安全阈值分模式、点击导航反馈、抗屈曲/sheath/slack guard。 |
| D6 | 规划中 | 推广到 segment_part 细管与更多分支目标，完善真实风险/半径/碰撞。 |
| D7 | 规划中 | Docker/compose/devcontainer 固化。 |

### 11.2 平台轨道 P/T

| 优先级 | 内容 |
|---|---|
| P0 | segment_part graph 连通性、路线图稳定、模式显式化。 |
| P1 | 第三人称相机可视性、X-ray/DSA 分屏、训练闭环录制回放评分。 |
| P1/T0-T2 | Newton Navigation Gym env、SB3 训练管线、curriculum/domain randomization。 |
| P2/T3-T7 | 策略加载到 HCI、RL 辅助驾驶、多进程训练、Isaac Lab 提升。 |

### 11.3 前端视觉轨道 V

| 阶段 | 内容 |
|---|---|
| V0 | 风险语义收口，禁止 mock 禁入区回归。 |
| V1 | 高质量视觉 GLB 导出，优先加载 `*_visual_high.glb`。 |
| V2 | 医学玻璃血管材质参数化。 |
| V3 | `Clinical Orbit` 默认相机预设。 |
| V4 | 路径与导丝 overlay 细化。 |
| V5 | 景深/远端层次。 |
| V6 | 真实 `risk_regions` 渲染器。 |

---

## 十二、与历史方案的关系

| 维度 | v1.0 旧方案 | v2.0 当前方案 |
|---|---|---|
| 物理主线 | MuJoCo/CathSim | NewtonEngine 为主，MuJoCo 兼容。 |
| 导丝性能判断 | 30Hz WebSocket + MuJoCo step | 60Hz 级 GPU 物理 + PhysicsWorker + 插值可缩短。 |
| 控制方式 | 手动 push/rotate、规划跟踪 | ShapeIntent 统一 Human/RL/Autopilot。 |
| 路径数据 | 单中心线 + VPP 图 | 分支图、routes、半径、PlannedPath、graph A*。 |
| 风险 | wall distance 启发式 | 按模式判定，真实 risk_regions 才渲染空间风险。 |
| 前端 | 基础 3D/HUD/X-ray 设想 | 医疗工作站 UI + 玻璃血管导航 + DSA/3D 双区。 |
| RL | Gymnasium/SB3 设想 | Newton Navigation Gym + ShapeIntent action + HCI 策略复用路线。 |
| 部署 | Docker Compose 计划 | GPU Docker + 远程开发 + Isaac Sim 提升路径。 |

---

## 十三、关键边界

1. `ShapeIntent` 不得实现为分段位置软约束，否则会回退到低保真运动学覆盖。
2. guided 模式只用于演示/可达性，不作为训练保真物理。
3. 红色/橙色风险体积必须来自真实后端 `risk_regions`，不得由固定路径比例或 mock 球体生成。
4. 视觉 mesh 与碰撞/SDF 资产必须分离，不能为了观感破坏物理稳定性。
5. Newton 当前的抗屈曲和 sheath/slack guard 是工程硬化，后续仍需更真实的导管/sheath 与推送传导模型。
6. Isaac Sim 是未来 RTX/RL/ROS 的提升目标，不是当前单根导丝物理可行性验证的起点。

---

## 十四、关联文档

- [06-segment_part_graph规划方案.md](06-segment_part_graph规划方案.md)
- [07-物理引擎抽象与实时性能架构.md](07-物理引擎抽象与实时性能架构.md)
- [07-Newton导丝物理仿真开发记录与规划.md](07-Newton导丝物理仿真开发记录与规划.md)
- [08-导丝物理迁移技术方案-Warp-XPBD-Newton.md](08-导丝物理迁移技术方案-Warp-XPBD-Newton.md)
- [09-人机交互与强化学习架构-ShapeIntent.md](09-人机交互与强化学习架构-ShapeIntent.md)
- [10-介入手术人机交互与强化学习一体化平台设计.md](10-介入手术人机交互与强化学习一体化平台设计.md)
- [11-前端设计方案.md](11-前端设计方案.md)
- [12-三维血管导航渲染改造方案.md](12-三维血管导航渲染改造方案.md)
- [13-三维血管导航渲染开发计划.md](13-三维血管导航渲染开发计划.md)
