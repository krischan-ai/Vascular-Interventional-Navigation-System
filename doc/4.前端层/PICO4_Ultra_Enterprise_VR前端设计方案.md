# PICO 4 Ultra Enterprise VR 前端设计方案

> 项目：CathSim 医疗导管/导丝导航与强化学习仿真平台  
> 目标设备：PICO 4 Ultra Enterprise  
> 应用框架：Godot 4.7.1  
> XR 标准：OpenXR 1.1  
> 交付形态：Android APK  
> 文档版本：v3.0（视觉基准融合重构版）  
> 更新日期：2026-08-03

---

## 1. 方案结论

### 1.1 最终技术路线

当前项目以快速交付和最大化复用现有代码为第一目标，正式路线保持为：

```text
Godot 4.7.1
+ Godot 内置 OpenXR
+ 与 Godot 版本匹配的 OpenXR Vendors Plugin
+ PICO OpenXR Runtime
+ Android Gradle Build
+ 现有 FastAPI / WebSocket 后端
```

最终产物为安装到 PICO 4 Ultra Enterprise 的 Android APK。

PICO 开发者平台中，以 **Native** 作为 OpenXR Runtime、设备要求和 PICO 扩展文档入口，但第一版不使用纯 C/C++ 重写应用。PICO OpenXR Native SDK 仅在后续确实需要标准 OpenXR 和 Godot Vendors Plugin 未暴露的专属能力时，通过 GDExtension、Android Plugin 或 JNI 桥接接入。

### 1.2 第一版视觉与交互基准

本轮用户提供的视觉图设为第一版的**空间构图和视觉语言基准**，重点继承：

- 中央半透明青蓝血管与导丝导航主体；
- 左侧上下双影像面板；
- 右侧常驻导航与安全面板；
- 下方运动控制坞；
- 双手 PICO 控制器与右手 Aim 射线；
- 深黑蓝医疗工业风、细蓝边框和克制发光；
- 安全绿、警告黄、急停红的有限使用。

视觉基准图不是逐像素开发稿。以下语义必须按本方案修正：

1. DSA 为明确标识的占位图，不冒充实时造影流；
2. 腔镜为三维血管模型中的动态第一人称摄像机视角；
3. `CONNECTED` 必须拆分为网络、后端会话、仿真引擎和 XR Session 状态；
4. `SAFE` 必须来自后端 `data.safety`，不得由前端自行推导；
5. Deadman 只是一项使能条件，不能等同于整体控制已获准；
6. 控制模式、控制权来源、数据新鲜度和阻断原因必须可见；
7. 不生成无数据来源的红色危险体积；
8. 第一版不以完整手部追踪和手指动画为交付要求。

### 1.3 第一版核心产品形态

```text
中央：可操作三维血管、导丝、规划路径和目标点
左侧：DSA 占位图 + 动态腔镜第一人称画面
右侧：后端权威安全、导航进度、连接与数据状态
下方：模式、推进、旋转、Deadman、控制门控与急停
次级：日志、连接详情、RL 状态和回放入口
```

这不是把桌面端 11 个面板贴入头显，而是围绕“观察—判断—控制—停止”重组为适合坐姿 VR 的四区工作台。

---

## 2. 产品定位、范围与边界

### 2.1 产品定位

VR 前端定位为坐姿、固定工作区的医疗导丝导航仿真工作站，用于：

- 导丝导航科研仿真；
- 人机交互实验；
- 三维血管导航教学；
- 强化学习策略观察；
- 仿真设备控制流程验证；
- 实验室演示和阶段成果展示。

第一版不表述为真实临床手术控制系统。软件急停不替代真实硬件急停。

### 2.2 第一版必须交付

1. APK 可安装并稳定启动；
2. OpenXR Session 正常建立；
3. 头部 6DoF 和双手柄追踪；
4. 中央三维血管、导丝、规划路径、tip 和目标点；
5. 左侧 DSA 占位图；
6. 左侧动态腔镜 SubViewport；
7. 右侧权威安全与导航面板；
8. 下方运动控制坞；
9. Aim Pose 射线选择空间 UI；
10. Deadman 持续使能下的推进与旋转；
11. 急停请求、后端锁存确认与恢复确认；
12. Session 失焦、追踪丢失、断线或数据过期时立即归零；
13. 真机性能基线；
14. 桌面端现有功能不回归。

### 2.3 第一版不纳入

- 真实 DSA 视频流、配准、窗宽窗位和测量；
- 真实腔镜硬件视频流；
- 手部追踪连续控制；
- 完整手指骨骼与握持动画；
- MR 透视、空间锚点和场景网格；
- 语音控制和多人协作；
- VR 内启动和管理 RL 训练；
- 真实机器人硬件控制；
- 真实扭矩闭环；
- 无来源风险体积；
- 纯 C++ Native 重写。

### 2.4 冻结与允许修改的范围

| 模块 | 第一版状态 | 允许工作 |
|---|---|---|
| DSA 真实数据链路 | 冻结 | 允许做占位图、状态标签和未来接口边界 |
| 腔镜视图 | 开发 | 复用 3D 模型第一人称摄像机 |
| RL 训练指标 | 冻结 | 只读入口或隐藏，不伪造实时指标 |
| 数据回放 | 冻结 | 只读入口或隐藏 |
| 真实机器人连接 | 冻结 | 明确显示“仿真设备”而非真实机器人 |
| 风险体积 | 仅真实数据 | `risk_regions` 为空时不显示 |

---

## 3. OpenXR 架构与安全原则

### 3.1 运行链路

```text
CathSim Godot 应用
    ↓
Godot OpenXR Interface
    ↓
OpenXR Loader / Vendors Plugin
    ↓
PICO OpenXR Runtime
    ↓
PICO 4 Ultra Enterprise
```

职责：

| 层 | 职责 |
|---|---|
| CathSim 应用 | 医疗业务、空间 UI、控制意图、安全门控 |
| Godot OpenXR | XR 节点、Action、Session 与渲染封装 |
| Loader / Vendors Plugin | 连接 Runtime、Android 与厂商适配 |
| PICO Runtime | 帧合成、头显和控制器追踪、输入 |
| PICO 设备 | 显示、传感器、控制器和系统服务 |

### 3.2 Session 状态与控制

| OpenXR 状态 | 显示 | 输入 | 导丝控制 |
|---|---|---|---|
| IDLE | 等待 | 禁用 | 禁止 |
| READY | 初始化 | 禁用 | 禁止 |
| SYNCHRONIZED | 可保持轻量画面 | 不作为控制源 | 禁止 |
| VISIBLE | 可见但无焦点 | Action inactive | 立即归零 |
| FOCUSED | 正常 | 允许同步 Action | 满足全部门控后允许 |
| STOPPING | 停止 XR 循环 | 禁用 | 立即归零 |
| LOSS_PENDING | 准备重建 | 禁用 | 锁定，不恢复旧输入 |
| EXITING | 退出 | 禁用 | 锁定 |

连续控制仅在以下条件全部成立时开放：

```text
xr_session_state == FOCUSED
and xr_actions_active
and required_controller_tracking_valid
and deadman_pressed
and websocket_ready
and backend_session_ready
and navigation_data_fresh
and not emergency_stop_latched
and not local_control_fault
```

任何条件失效时，同一客户端更新周期内执行：

```text
push = 0
rotate = 0
controls_blocked = true
```

### 3.3 Reference Space 与单位

- 主工作台首选 `LOCAL_FLOOR`；
- 不支持时降级为 `LOCAL`；
- 不依赖 `STAGE` 已经配置；
- `VIEW` 只用于短暂提示，不承载主工作台；
- OpenXR/Godot 世界单位为米；
- 后端医学导航数据继续使用毫米；
- 适配层统一执行 `1 mm = 0.001 m`。

Reference Space 变化时：

1. 立即归零；
2. 标记工作台定位失效；
3. 重新定位空间 UI；
4. 要求用户重新按下 Deadman。

---

## 4. 总体软件架构

### 4.1 分层

```text
PICO OS / Android
├── APK、生命周期和企业部署
PICO OpenXR Runtime
├── Session、View、Space、控制器
Godot OpenXR + Vendors Plugin
├── XR 节点和 Android 厂商适配
XR Presentation Layer
├── 空间 UI、指针、摄像机、视觉样式
Shared Frontend Core
├── 状态存储、新鲜度、安全门控、控制路由
WebSocket Client
├── navigation_visual_v3、急停、恢复和重连
FastAPI / SessionManager / Physics
```

### 4.2 双入口与共享核心

```text
DesktopMain.tscn ─┐
                  ├─ NavigationStateStore
XRMain.tscn ──────┤  SafetyStateStore
                  │  DataFreshnessMonitor
                  │  EmergencyStopCoordinator
                  │  ControlRouter
                  └─ WebSocketClient
```

桌面和 VR 共享：

- `navigation_visual_v3` 解析；
- WebSocket 重连；
- 数据新鲜度；
- 后端安全权威；
- 急停锁存和恢复；
- 导丝、路径和血管状态。

VR 独有：

- OpenXR 启动与 Session 桥接；
- Action 输入；
- Aim 射线；
- 空间布局；
- 腔镜 SubViewport；
- VR 性能档位。

### 4.3 推荐节点结构

```text
XRMain : Node3D
├── XRBootstrap
├── XRSessionStateBridge
├── XROrigin3D
│   ├── XRCamera3D
│   ├── LeftController : XRController3D
│   │   ├── LeftControllerVisual
│   │   └── LeftAimPointer
│   └── RightController : XRController3D
│       ├── RightControllerVisual
│       └── RightAimPointer
├── ClinicalWorkspace
│   ├── VesselRoot
│   ├── GuidewireRenderer
│   ├── PlannedPathRenderer
│   ├── TipMarker
│   ├── TargetMarker
│   └── WorkspaceAnchor
├── SpatialPanels
│   ├── ImagingPanel
│   │   ├── DSAPlaceholder
│   │   └── EndoscopeViewportPanel
│   ├── SafetyPanel
│   ├── MotionControlDock
│   └── SecondaryPanelHost
├── EndoscopeViewport
│   ├── EndoscopeWorldRoot
│   └── EndoscopeCamera3D
├── XRInputAdapter
├── XRSafetyGate
├── XRControlRouter
├── NavigationStateStore
├── DataFreshnessMonitor
├── EmergencyStopCoordinator
└── WebSocketClient
```

### 4.4 推荐目录结构

```text
godot_client/
├── scenes/
│   ├── desktop/
│   └── xr/
│       ├── MainXR.tscn
│       ├── XRClinicalWorkspace.tscn
│       ├── XRImagingPanel.tscn
│       ├── XRSafetyPanel.tscn
│       ├── XRMotionControlDock.tscn
│       └── XREndoscopeViewport.tscn
├── scripts/
│   ├── shared/
│   │   ├── navigation_state_store.gd
│   │   ├── safety_state_store.gd
│   │   ├── data_freshness_monitor.gd
│   │   ├── emergency_stop_coordinator.gd
│   │   └── control_router.gd
│   ├── xr/
│   │   ├── xr_bootstrap.gd
│   │   ├── xr_session_state_bridge.gd
│   │   ├── xr_input_adapter.gd
│   │   ├── xr_safety_gate.gd
│   │   ├── xr_ui_pointer.gd
│   │   ├── xr_workspace_manager.gd
│   │   ├── xr_imaging_panel.gd
│   │   ├── xr_endoscope_camera.gd
│   │   └── xr_performance_manager.gd
│   └── rendering/
│       ├── vessel_renderer.gd
│       ├── guidewire_renderer.gd
│       └── path_renderer.gd
└── addons/
    └── godotopenxrvendors/
```

---

## 5. 视觉基准与设计语言

### 5.1 基准图定位

视觉基准图主要约束：

- 空间构图；
- 面板形态；
- 视觉风格；
- 主次关系；
- 操作区位置；
- 血管材质方向。

它不直接决定：

- 数据是否真实；
- 按钮是否已实现；
- 安全状态来源；
- 网络和 Session 语义；
- 具体 OpenXR Action 映射。

### 5.2 统一色彩

```text
环境背景：#030911 / #06101A
面板主体：#0B1521 ～ #101D2B
面板次级区：#0D1A27
边框：#2D5E91，低亮度
主文字：#E2EDF7
次文字：#A8B8C8
弱文字：#73869A
强调蓝：#4FA6FF
路径白蓝：#D7F2FF / #7EC8FF
安全绿：#65E56F
警告黄：#FFD65A
危险红：#FF4B4B
未知灰：#8B98A5
```

原则：

- 绿色只表示后端确认的安全或真实连接就绪；
- 黄色只表示延迟、中风险或预警；
- 红色只表示停止要求、急停和高风险；
- 占位、未知和过期统一使用灰色；
- 不用彩色装饰制造“科幻感”。

### 5.3 面板形态

- 深色磨砂面板；
- 轻微弧形，面向用户；
- 细蓝边框；
- 轻微外发光；
- 圆角 8–14 px 的视觉等效；
- 不使用高成本实时背景模糊；
- 主要信息避免多层卡片嵌套。

### 5.4 字体与图标

- 中文：Noto Sans SC / 思源黑体；
- 数字：等宽字体；
- 主状态大字优先；
- 次级说明可折叠；
- 使用线性图标；
- 不使用 emoji；
- 第一版允许英文技术标签，但正式交付优先简体中文。

### 5.5 真机可读原则

- 关键数字在正常坐姿下不需要靠近面板阅读；
- 普通正文不直接沿用桌面 12 px；
- 关键按钮世界尺寸不少于 0.04–0.055 m；
- 点击碰撞区为视觉区域的 1.15–1.35 倍；
- 低频细节放入展开态。

---

## 6. VR 空间布局

### 6.1 四区布局

```text
                中央三维血管导航

左侧双影像                           右侧导航与安全

                下方运动控制坞
```

次级入口：

- 左下：主菜单；
- 右下：更多；
- 日志、连接详情、RL 和回放在次级层展开。

### 6.2 空间位置建议

以当前头部初始朝向为基准：

| 区域 | 距离 | 横向位置 | 垂直位置 | 建议尺寸 |
|---|---:|---:|---:|---:|
| 中央血管 | 0.95–1.25 m | 0 | 视线下 5°–12° | 占中央视野 45%–55% |
| 左影像面板 | 0.95–1.20 m | 左 25°–35° | 视线附近 | 宽 0.48–0.58 m |
| 右安全面板 | 0.95–1.20 m | 右 25°–35° | 视线附近 | 宽 0.48–0.58 m |
| 下控制坞 | 0.80–1.05 m | 0 | 视线下 22°–32° | 宽 0.68–0.85 m |

### 6.3 与视觉基准图的修正

视觉基准图中的主血管偏大。开发默认值应：

- 将中央模型整体缩小约 15%–25%；
- 避免上端和下端接近视野边缘；
- 保持当前局部主分支清晰；
- 允许用户一键聚焦和复位；
- 不让左右面板与血管产生明显深度冲突。

### 6.4 工作台复位

“重置工作台”只改变本地空间布局：

1. 控制归零；
2. 以当前头部朝向重新定位四区；
3. 不改变后端导丝和血管状态；
4. 完成后要求重新按下 Deadman。

---

## 7. 中央三维血管导航区

### 7.1 默认内容

必须显示：

- 当前局部血管；
- 导丝本体；
- 当前 tip；
- 已走路径；
- 前方规划路径；
- 目标点；
- 必要时的分叉标识。

### 7.2 视觉层级

```text
最高：tip、前方路径、目标点
其次：当前主血管和导丝
再次：已走路径
最低：非目标远端分支
```

建议：

- Tip：青白小型发光环，不用大球；
- 目标点：蓝色双圆环；
- 前方路径：细亮白蓝；
- 已走路径：降低亮度；
- 导丝：银白或浅青；
- 血管：青蓝玻璃、边缘 Fresnel、中心透明。

### 7.3 交互

- Aim 射线可选择血管、路径或目标；
- UI 射线和三维选择射线具有不同命中光标；
- 未命中时缩短或隐藏长射线；
- 模型旋转、缩放和复位只改变视图，不直接控制导丝；
- 点击导航未接入后端时保持禁用。

### 7.4 禁止项

- 无来源红色禁入球；
- 全树同等亮度；
- 整片 cyan 过曝；
- 导丝 tip 巨大化；
- 模型默认快速旋转；
- 用视觉曲率自行推导危险区域。

---

## 8. 左侧双影像面板

### 8.1 总体决策

左侧采用**上下同时显示**，不使用二选一 Tab 作为默认逻辑：

```text
上方：DSA 占位图
下方：动态腔镜第一人称视角
```

两种影像承担互补作用：

- DSA：未来用于全局造影定位；
- 腔镜：当前用于局部内腔、前方分叉和导丝 tip 观察。

### 8.2 默认比例

```text
DSA：约 56%–60%
腔镜：约 40%–44%
```

原因：DSA 未来更依赖细小血管分支和全局结构分辨率；腔镜画面主要观察局部内腔。

### 8.3 三种布局模式

1. **双画面**：默认，上下同时显示；
2. **DSA 主视图**：DSA 占 80%，腔镜作为小画中画；
3. **腔镜主视图**：腔镜占 80%，DSA 作为小画中画。

任何模式下：

- 急停、安全状态和控制阻断提示保持可见；
- 另一画面不完全消失；
- 放大只改变观察布局，不改变数据源状态。

### 8.4 DSA 占位图

第一版 DSA 使用固定演示图或占位图，不接入实时流。

界面必须明确显示：

```text
DSA
占位图
数据源：Demo Image
实时状态：未接入
```

不允许显示：

- `LIVE`；
- 伪实时帧率；
- 伪延迟；
- 未实现的缩放、测量、截图已经可用。

第一版 DSA 工具处理：

| 工具 | 第一版状态 |
|---|---|
| 放大查看 | 可选，仅查看占位图 |
| 平移 | 可选 |
| 复位 | 可选 |
| 测量 | 禁用 |
| 旋转 | 禁用或隐藏 |
| 截图 | 禁用或仅截取界面，不标成 DSA 采集 |
| 窗宽窗位 | 隐藏 |

视觉基准图中的左侧竖向工具栏可以保留造型，但必须根据当前焦点画面和功能状态动态启用。

### 8.5 腔镜动态画面

第一版腔镜不是外部视频，而是 Godot 三维血管场景中的动态摄像机：

```text
Vessel 3D Scene
    ↓
EndoscopeCamera3D
    ↓
SubViewport
    ↓
ViewportTexture
    ↓
左侧腔镜面板
```

界面显示：

```text
腔镜
三维相机视角
实时
```

可选真实状态：

- 摄像机模式：Offset Follow；
- 本地渲染状态；
- 分辨率档位；
- 是否有效定位。

不显示伪造网络延迟，因为画面为本地渲染。

### 8.6 腔镜摄像机位置

相机采用导丝 tip 附近的侧后偏置：

```text
camera_pos = tip_position
           - tangent * back_offset
           + normal * vertical_offset
           + binormal * lateral_offset
```

初始调参范围：

| 参数 | 建议值 |
|---|---:|
| back_offset | 8–15 mm |
| vertical_offset | 3–6 mm |
| lateral_offset | 2–5 mm |
| FOV | 70°–90° |
| near plane | 根据模型尺度尽可能小且稳定 |

目标：

- 导丝可见但不遮挡中央；
- 前方分叉可辨认；
- 相机不穿出血管壁；
- 曲线处不发生剧烈 Roll；
- 路径重规划时平滑过渡。

### 8.7 腔镜画面效果

第一版建议：

- 红棕或暗橙内壁；
- 轻微湿润高光；
- 局部光照；
- 轻微暗角；
- 导丝 tip 可见；
- 分叉清晰；
- 不使用重型 DOF、体积雾和复杂后处理。

### 8.8 焦点与工具栏

点击某个画面后显示焦点边框：

```text
DSA 获得焦点 → 仅启用 DSA 已实现工具
腔镜获得焦点 → 启用亮度、复位、放大等真实功能
```

第一版腔镜工具：

- 放大；
- 复位相机；
- 亮度或曝光档位；
- 返回双画面。

录制和截图未完成时禁用。

---

## 9. 右侧导航与安全面板

### 9.1 面板目标

右侧面板常驻回答四个问题：

1. 导丝走到哪里；
2. 当前是否安全；
3. 数据是否可信；
4. 系统是否允许控制。

### 9.2 常驻字段

```text
安全状态
风险等级
壁距
接触力
推进速度
剩余距离 / 路径进度
数据新鲜度
WebSocket 状态
后端 Session 状态
仿真引擎状态
XR Session 状态
急停锁存状态
```

### 9.3 信息层级

```text
第一层：安全状态、风险等级、急停锁存
第二层：壁距、接触力、速度、剩余距离
第三层：路径进度和接触力趋势
第四层：连接、来源、时间戳和延迟
```

### 9.4 安全来源

图片中的 `SAFE` 必须在实现中变为：

```text
安全
来源：后端 data.safety
状态：Fresh
更新时间：12:33:26.184
```

前端不得根据壁距或接触力自行生成另一套安全结论。

颜色映射：

- 绿色：后端明确安全；
- 黄色：后端中风险或数据延迟；
- 红色：后端高风险、`stop_required` 或急停；
- 灰色：未知、缺失、过期。

### 9.5 连接状态拆分

禁止用单一 `CONNECTED` 代替全部状态。

推荐显示：

```text
网络          已连接
后端会话      已就绪
仿真引擎      Newton / Ready
XR Session    FOCUSED
数据状态      Fresh
控制许可      可重新使能
端到端延迟    18 ms
```

“仿真设备已就绪”不等于真实机器人已连接。

### 9.6 折叠策略

默认展开：

- 安全状态；
- 壁距和接触力；
- 剩余距离。

默认简化：

- 路径趋势；
- 系统状态。

默认折叠：

- 数据来源详情；
- 版本和设备信息；
- 完整事件记录。

---

## 10. 下方运动控制坞

### 10.1 必须包含

- 当前模式；
- 控制权来源；
- 推进输入；
- 旋转输入；
- Deadman 状态；
- 控制门控状态；
- 阻断原因；
- 大尺寸急停按钮。

推荐结构：

```text
当前模式：手动控制
控制来源：PICO 手柄

推进       旋转       Deadman       急停
+0.35      -0.25      已按住         红色大按钮

控制状态：已使能 / 已阻断
原因：— / 数据过期 / XR未聚焦 / 后端急停
```

### 10.2 Deadman 表达

视觉基准图中的 `ENGAGED` 改为更明确的：

```text
持续使能
已按住
左手 Grip
释放立即停止
```

Deadman 只表示“用户持续确认”，整体控制还需要满足 Session、追踪、网络、数据和后端安全条件。

### 10.3 连续控制来源

- 推进和旋转来自 OpenXR Action；
- 面板的 `+ / - / L / R` 只作为预设或辅助，不作为高频连续控制主入口；
- 面板显示的是经过门控后的实际发送值；
- 不显示未经门控的原始摇杆值为“已执行控制”。

### 10.4 急停

急停按钮：

- 视觉尺寸最大；
- 始终可见；
- 不被影像放大模式遮挡；
- 需要明确触发，不因射线掠过误触；
- 点击后客户端立即阻断；
- 等待后端确认后显示“已锁存”。

急停状态：

```text
请求中 → 后端已锁存 → 恢复请求中 → 可重新使能
```

恢复后必须重新按下 Deadman。

---

## 11. OpenXR Action 与手柄交互

### 11.1 Action Set

建议：

```text
medical_control
├── push_axis
├── rotate_axis
├── deadman_enable
├── emergency_stop
└── resume_control

ui_navigation
├── ui_select
├── ui_back
├── reset_workspace
├── toggle_imaging_layout
├── focus_dsa
└── focus_endoscope
```

### 11.2 Aim 与 Grip

- `aim pose`：UI 射线和三维选择；
- `grip pose`：控制器模型或简化手套模型；
- 不使用 Grip 方向代替 Aim 射线；
- 射线命中 UI 时显示圆形光标；
- 射线命中血管时显示表面命中环。

### 11.3 第一版手部表现

视觉基准图显示完整蓝色手套和控制器。第一版实现要求降低为：

```text
必须：PICO 控制器模型 + Aim 射线
可选：简化静态手套或手掌模型
不要求：完整手指骨骼、抓握动画和手部追踪
```

避免为视觉图中的手指姿态延误核心交付。

### 11.4 统一输入快照

每 XR 帧由单一适配器形成：

```text
XRInputSnapshot
├── session_focused
├── actions_active
├── left_tracking_valid
├── right_tracking_valid
├── deadman_pressed
├── push_axis
├── rotate_axis
├── emergency_stop_pressed
├── aim_pose
└── timestamp
```

所有输入经 `XRSafetyGate` 后才能进入 `ControlRouter`。

---

## 12. 三维相机模式

### 12.1 Clinical Tabletop

默认控制和观察模式：

- 世界锁定；
- 中央模型可旋转、缩放、复位；
- 用户本身不移动；
- 右侧安全和下方急停持续可见。

### 12.2 Guidewire Navigation Follow

观察导丝前方路线：

- tip 位于画面下部；
- 规划路径向上延伸；
- 前方弯道和分叉可见；
- 使用平滑切线和最小旋转框架；
- 限制 Roll；
- 非规划分支只在遮挡时淡出。

### 12.3 Endoscope Offset Follow

用于左侧动态腔镜面板：

- 相机位于 tip 侧后方；
- 不固定在导丝轴线上；
- 不直接占满用户头显视野；
- 使用独立 SubViewport；
- 不用头部移动控制导丝。

### 12.4 Tree Overview

- 显示全树拓扑；
- 目标分支高亮；
- 非目标分支暗化；
- tip 和目标点明确；
- 一键返回 Clinical Tabletop。

### 12.5 Experimental Immersive Endoscope

作为后续实验功能，不是第一版主交付：

- 进入时控制默认阻断；
- 保留最小急停和退出提示；
- 需要单独做眩晕和性能验收。

---

## 13. 控制、安全与急停闭环

### 13.1 控制链路

```text
OpenXR Action
    ↓
XRInputSnapshot
    ↓
XRSafetyGate
    ↓
ControlRouter
    ↓
WebSocket control
    ↓
SessionManager
    ↓
物理引擎
```

任何 UI、手柄或快捷入口都不得绕过 `XRSafetyGate`。

### 13.2 Deadman 规则

```text
未按下：push=0, rotate=0
刚按下：从零重新读取摇杆
松开：同帧归零
重新获得焦点：必须重新按下
追踪恢复：必须重新按下
急停恢复：必须重新按下
```

### 13.3 本地故障触发

任一情况触发控制阻断：

- XR Session 非 FOCUSED；
- Action inactive；
- 必需控制器追踪失效；
- Reference Space 变化；
- 应用暂停；
- WebSocket 断开；
- 后端 Session 未 ready；
- 导航数据过期；
- 后端安全要求停止；
- 急停请求中或已锁存；
- 控制发送连续失败。

### 13.4 急停流程

```text
用户触发
→ 客户端立即 controls_blocked=true
→ push=0, rotate=0
→ 发送 emergency_stop
→ 等待 emergency_stop_confirmed
→ 显示“后端已锁存”
```

恢复：

```text
发送 resume
→ 继续阻断
→ 收到 resume_confirmed
→ 进入“可重新使能”
→ 用户重新按下 Deadman
```

禁止自动恢复旧摇杆、旧 ShapeIntent 或旧导航目标。

---

## 14. 数据契约与状态管理

### 14.1 继续使用 `navigation_visual_v3`

核心字段：

```text
schema_version
timestamp_ms
data_status
source
tip_position
tip_direction
tip_quaternion
contact_force
wall_distance
curvature
velocity
path_progress
path_deviation
remaining_distance
path_total_distance
path_travelled_distance
vessel_radius
eta_seconds
risk_regions
safety
control_state
```

原则：

- `data.safety` 是安全唯一权威；
- 缺失值显示 `— / unknown / stale`；
- `risk_regions` 为空时保持空；
- 不用风险分数推算接触力；
- 不用固定数字冒充实时数据；
- DSA 占位状态是本地 UI 状态，不写成实时后端数据。

### 14.2 XR 本地状态

```text
xr_session_state
xr_session_focused
xr_should_render
left_aim_active
right_aim_active
controller_tracking_valid
deadman_pressed
reference_space_type
reference_space_valid
controls_blocked
control_block_reason
imaging_layout_mode
dsa_source_mode
endoscope_camera_valid
last_input_timestamp
last_state_timestamp
```

### 14.3 数据新鲜度

| 状态 | UI | 控制 |
|---|---|---|
| Fresh | 正常 | 按门控开放 |
| Delayed | 黄色提示 | 限速或阻断，按项目策略 |
| Stale | 灰色过期 | 必须阻断 |
| Disconnected | 重连提示 | 必须阻断 |

旧值不得继续以绿色实时状态显示。

### 14.4 影像数据状态

DSA：

```text
dsa_source_mode = placeholder
status = unavailable_for_live
```

腔镜：

```text
source = local_3d_camera
status = live_rendered / invalid_camera / paused
```

二者状态独立，不使用一个总 `CONNECTED` 概括。

---

## 15. 渲染与性能设计

### 15.1 性能优先级

1. 头部追踪稳定；
2. 手柄输入低延迟；
3. 控制门控可靠；
4. 安全状态可读；
5. 血管与导丝清晰；
6. 腔镜画面稳定；
7. 视觉增强。

### 15.2 渲染器

首版优先 Compatibility Renderer 建立真机基线；之后在同设备、同场景对比 Mobile Vulkan。

### 15.3 血管档位

```text
VR High      高质量局部分支
VR Balanced  默认，局部高质量 + 远端 LOD
VR Safe      降低透明、Bloom 和远端分支
Debug        中心线或轻量模型
```

默认 `VR Balanced`。

### 15.4 透明血管优化

- 近端主分支保留 Fresnel；
- 远端降低 Alpha 和 Emission；
- 避免多层透明壳；
- 限制 Bloom；
- 使用 LOD 和视锥剔除；
- 不让血管覆盖大部分双眼视野；
- 必要时以不透明边缘模拟玻璃层次。

### 15.5 DSA 与腔镜开销

DSA：

- 静态 TextureRect；
- 不每帧更新；
- 无视频解码；
- 占位图可使用中等分辨率。

腔镜：

- 只使用一个动态 SubViewport；
- 第一版建议 640×480、768×576 或相近档位；
- 刷新率可低于头显渲染帧率，例如 30–45 Hz；
- 面板放大时再提高分辨率；
- 不启用昂贵独立后处理；
- 腔镜不可见或被折叠时降低刷新率。

这比同时运行两路高分辨率动态视频更适合快速交付。

### 15.6 空间面板优化

- 低频面板不每帧重绘；
- 趋势图限频更新；
- 文本变化才刷新；
- 折叠区暂停更新；
- 避免透明模糊背景；
- 面板纹理使用稳定分辨率。

---

## 16. 舒适性与人体工学

### 16.1 坐姿固定工作区

- 不提供摇杆移动用户；
- 不提供平滑转向；
- 用户操作固定工作台；
- 主信息位于无需大幅转头的范围。

### 16.2 自动相机限制

- 位置平滑；
- 方向响应快于位置；
- 限制 Roll；
- 避免突然反转；
- 路径重规划渐进切换；
- 随时可返回 Clinical Tabletop。

### 16.3 影像面板舒适性

- 双画面保持在左侧，不追随头部；
- 放大模式不把画面贴脸；
- 腔镜保持在空间面板中，避免直接全屏造成眩晕；
- 亮度和对比度不能明显高于环境。

### 16.4 系统弹窗和摘戴

系统菜单、头显摘下或 Runtime 取消焦点时，按 OpenXR Session 状态立即阻断控制，不只依赖 Android Activity 回调。

---

## 17. Android 构建与 PICO 部署

### 17.1 环境

```text
Godot 4.7.1
OpenJDK 17
Android Studio / Android SDK
Godot Android Build Template
ADB
匹配 Godot 版本的 OpenXR Vendors Plugin
PICO 开发者模式
```

### 17.2 导出配置

```text
Platform：Android
Use Gradle Build：Enabled
XR Mode：OpenXR
XR Features：Pico XR Features（若插件提供）
Renderer：Compatibility 首版
Architecture：arm64
Package Name：固定正式命名
```

### 17.3 真机检查

- PICO OS 版本；
- OpenXR Runtime；
- Vendors Plugin 版本；
- ADB；
- APK 签名；
- Session 日志；
- Interaction Profile；
- Reference Space 支持；
- 可用扩展；
- DSA 占位资源；
- 腔镜 SubViewport 是否有效。

### 17.4 企业部署

实验室：ADB、本地签名 APK、局域网后端。  
正式阶段再评估 Kiosk、批量安装、版本管理和远程日志。

---

## 18. 快速交付路线

### 阶段 A：OpenXR 真机冒烟

- 最小 XR 场景；
- 头部 6DoF；
- 双控制器 Aim / Grip；
- Runtime、Session 和 Interaction Profile 日志。

通过：连续启动 10 次无初始化失败。

### 阶段 B：Session 与输入门控

- Session 状态桥接；
- Action Set；
- 统一 `XRInputSnapshot`；
- 非 FOCUSED、追踪丢失和 Space 变化归零。

通过：所有失效场景不残留非零输入。

### 阶段 C：共享核心

- WebSocketClient；
- `navigation_visual_v3`；
- 数据新鲜度；
- 后端 Session ready；
- 重连和旧帧过滤。

通过：VR 与桌面读取同一业务契约。

### 阶段 D：视觉基准四区骨架

- 中央血管；
- 左双影像；
- 右安全；
- 下控制坞；
- 面板位置和弧度；
- 控制器模型和 Aim 射线。

通过：整体构图接近视觉基准，不出现桌面面板堆叠。

### 阶段 E：DSA 占位与动态腔镜

- DSA 静态占位；
- 占位状态标签；
- EndoscopeCamera3D；
- SubViewport；
- 双画面、DSA 主视图和腔镜主视图；
- 腔镜相机偏置和防穿模调参。

通过：DSA 不冒充实时；腔镜可动态观察前方内腔。

### 阶段 F：三维导航

- 导丝、路径、tip 和目标点；
- Clinical Tabletop；
- Guidewire Follow；
- Tree Overview；
- 毫米到米转换。

通过：模型、路径和 tip 无明显漂移。

### 阶段 G：控制与急停

- Deadman；
- 推进与旋转；
- `XRSafetyGate`；
- 急停与恢复；
- 模式和阻断原因显示。

通过：任何异常不自动恢复旧控制。

### 阶段 H：性能与发布

- LOD；
- 透明优化；
- 腔镜刷新率；
- Compatibility / Vulkan 对比；
- 30 分钟热稳定；
- Release APK。

---

## 19. 测试与验收

### 19.1 视觉基准验收

- 四区位置与基准图一致；
- 中央血管缩放适合 VR，不占满垂直视野；
- 左侧上下双画面；
- 右侧安全面板；
- 下方控制坞；
- 深黑蓝医疗风；
- 急停红色优先级明确；
- 不出现游戏 HUD、飞船驾驶舱或装饰性全息过载。

### 19.2 DSA 验收

- 清晰标识“占位图”；
- 不显示 LIVE；
- 不显示伪帧率和伪延迟；
- 未实现工具为禁用或隐藏；
- 后续数据源接口边界明确。

### 19.3 腔镜验收

- 来自 3D 场景动态相机；
- 相机位于 tip 侧后方；
- 导丝不遮挡中央；
- 前方分叉可辨认；
- 无明显穿壁；
- 曲线段无严重 Roll；
- SubViewport 开销在性能预算内；
- 无效相机时显示“无数据”而非静态伪画面。

### 19.4 OpenXR 生命周期

覆盖：

- IDLE → READY → FOCUSED；
- FOCUSED → VISIBLE；
- VISIBLE → FOCUSED；
- STOPPING；
- LOSS_PENDING；
- EXITING；
- 系统菜单；
- 摘戴头显；
- Session 重建。

验收：非 FOCUSED 无非零控制。

### 19.5 输入与控制

- Aim 射线方向正确；
- Grip 控制器模型正确；
- Deadman 松开同帧归零；
- 追踪丢失归零；
- 重新获得焦点后重新按 Deadman；
- 面板显示门控后的实际控制值；
- UI 不能绕过安全门控。

### 19.6 急停

- 正常急停；
- 网络延迟急停；
- 请求中断线；
- 重连后锁存意图；
- 锁存期间拒绝非零控制；
- 恢复后不重放旧值；
- 恢复后重新 Deadman。

### 19.7 数据与安全

- `SAFE` 来源为后端；
- Fresh / Delayed / Stale；
- 缺失值；
- 空 `risk_regions`；
- 旧 Session 帧；
- WebSocket 重连；
- 未知状态不显示绿色；
- 连接状态拆分显示。

### 19.8 性能

至少记录：

- 10 分钟与 30 分钟运行；
- 平均和 95/99 百分位帧时间；
- 丢帧；
- CPU/GPU 瓶颈；
- 温度与降频；
- 内存峰值；
- 腔镜开启/关闭差异；
- 双画面与放大模式差异；
- WebSocket 高峰影响。

---

## 20. 风险与降级

| 风险 | 影响 | 降级 |
|---|---|---|
| 透明血管过绘 | 掉帧 | VR Safe 材质、LOD、缩小模型 |
| 腔镜 SubViewport 开销高 | 卡顿 | 降到 30 Hz、降低分辨率、简化材质 |
| 腔镜穿壁 | 视觉错误 | 增大偏置、SDF/半径约束、回退到最近有效姿态 |
| 面板文字过小 | 无法阅读 | 放大主数字、折叠次级文字 |
| 连接状态混淆 | 错误判断 | 拆分网络、Session、引擎和数据状态 |
| DSA 被误认为实时 | 可信性问题 | 强制 Placeholder 标签 |
| Session 状态处理不全 | 残留控制 | 状态桥接与同帧归零 |
| 手部模型工作量过大 | 延期 | 第一版只显示控制器和射线 |
| Vendors Plugin 不匹配 | 无法运行 | 锁定版本矩阵 |

降级优先级：

```text
LOCAL_FLOOR → LOCAL
VR Balanced → VR Safe
腔镜 45 Hz → 30 Hz
腔镜 768p → 480p
简化手套 → 仅控制器
厂商增强 → 标准 OpenXR
```

---

## 21. 最终工程基线

### 21.1 固定技术栈

```text
应用框架：Godot 4.7.1
XR 标准：OpenXR 1.1
Android 构建：Gradle
PICO 适配：OpenXR Runtime + Vendors Plugin
默认渲染器：Compatibility
输入：PICO 控制器 + OpenXR Action
连续控制：Deadman + 摇杆
通信：WebSocket / navigation_visual_v3
安全权威：后端 data.safety
DSA：静态占位图
腔镜：3D 模型第一人称 SubViewport
交付：Android APK
```

### 21.2 固定视觉基线

```text
中央：青蓝玻璃血管 + 导丝 + 路径
左侧：DSA 占位图 + 动态腔镜
右侧：导航、安全、连接和新鲜度
下方：模式、控制、Deadman 和急停
环境：深黑蓝医疗工业风
交互：右手 Aim 射线优先
```

### 21.3 固定安全原则

1. 只有 FOCUSED 才允许 XR 控制；
2. Action inactive、追踪失效或断线立即归零；
3. Deadman 是必要但非充分条件；
4. 急停先本地阻断，再等待后端锁存；
5. 恢复后不恢复旧输入；
6. 后端安全结论唯一权威；
7. DSA 占位不能描述成实时；
8. 腔镜无效时显示无数据；
9. 空 `risk_regions` 不生成危险体积；
10. Native SDK 只按需桥接。

### 21.4 第一版完成判定

只有同时满足以下条件才算完成：

- APK 可重复安装和启动；
- OpenXR Session 稳定；
- 头显和控制器追踪正常；
- 四区布局达到视觉基准；
- 中央血管、导丝、路径、tip 和目标正确；
- DSA 明确为占位图；
- 腔镜为动态 3D 摄像机视角；
- 空间 UI 可操作；
- Deadman 和急停闭环通过；
- 失焦、追踪丢失和数据过期不残留控制；
- 安全与连接状态语义准确；
- 真机性能达到约定基线；
- 不含伪实时、伪安全或伪风险数据。

---

## 22. 官方资料与版本矩阵

实现依据：

- Khronos OpenXR 1.1 Specification；
- OpenXR API Reference；
- OpenXR Reference Guide；
- Godot XR 与 Android 部署文档；
- Godot OpenXR Vendors Plugin 文档；
- PICO OpenXR Runtime 和设备文档。

仓库应增加：

```text
docs/xr/version-matrix.md
```

记录：

```text
Godot 版本
Vendors Plugin 版本
Android SDK / Build Tools
PICO OS 版本
PICO Runtime 版本
APK 版本
启用的 OpenXR 扩展
Interaction Profile
渲染器
腔镜 SubViewport 分辨率与刷新率
血管性能档位
```

该矩阵用于真机差异排查和团队环境锁定。
