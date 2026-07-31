# PICO 4 Ultra Enterprise VR 前端设计方案

> 项目：CathSim 医疗导管/导丝导航与强化学习仿真平台  
> 目标设备：PICO 4 Ultra Enterprise  
> Android 应用框架：Godot 4.7.1  
> XR 标准：OpenXR  
> 推荐运行形态：Android 原生 APK  
> 文档日期：2026-07-31

---

## 1. 交付决策与方案摘要

### 1.1 Android 应用框架决策

本项目快速交付版本直接采用：

```text
Godot 4.7.1
+ GDScript
+ OpenXR
+ Godot Android Gradle Export
+ PICO 4 Ultra Enterprise
+ 现有 FastAPI / WebSocket 后端
```

这里的 Android 应用开发框架不是 Flutter、React Native 或 Jetpack Compose，而是由 **Godot 同时承担 Android 应用壳、双目 XR 渲染、三维场景、空间 UI、输入和生命周期管理**，最终输出标准签名 APK。

选择 Godot 的首要原因不是它在所有 PICO 项目中都优于 Unity，而是当前 CathSim 已经在 Godot 中实现三维血管、导丝、规划路径、导航视角、WebSocket、状态过期处理和急停闭环。快速交付阶段继续使用 Godot，可以把工作集中在 XR 场景、手柄输入、空间面板和移动端性能适配，而不是迁移既有业务代码。

第一版不使用以下框架作为主体：

- Flutter、React Native、Jetpack Compose：适合二维 Android 页面，不承担双目 VR 三维主渲染；
- Kotlin/Java + Native OpenXR：底层自由度高，但需要自行实现大量 XR、渲染和交互基础设施；
- Unity：PICO 厂商扩展支持更完整，但当前迁移成本高于收益；
- WebXR：适合免安装演示和只读访问，不承担主要连续控制与安全闭环。

### 1.2 正式运行链路

```text
PICO 4 Ultra Enterprise
    ↓ Android 原生 APK
Godot 4.7.1 + OpenXR
    ↓ navigation_visual_v3 / control / safety events
WebSocket / WSS
    ↓
FastAPI SessionManager
    ↓
NavigationEngine / ShapeIntent / PhysicsEngine
    ↓
NewtonEngine / MuJoCoEngine / KinematicEngine
```

WebXR 保留为辅助产品形态：

- 无需安装的产品演示；
- 桌面浏览器或 PICO Browser 中的轻量查看；
- 只读三维模型浏览和教学页面；
- 远程查看导航状态。

WebXR 不负责急停、持续推进、旋转控制和恢复确认。

### 1.3 快速交付范围

第一版必须完成：

1. PICO 头显中安装并启动 Godot APK；
2. OpenXR 会话、头部 6DoF 和双手柄追踪；
3. 三维血管、导丝、规划路径和目标点显示；
4. 手柄射线选择空间 UI；
5. Deadman 持续使能下的推进与旋转；
6. 松开使能键、追踪丢失、应用失焦、断线或数据过期时立即归零；
7. 软件急停请求、后端锁存确认和恢复确认；
8. 导航、安全、接触力、壁距和风险状态显示；
9. VR Balanced 性能档位和真机帧率基线；
10. 现有桌面端功能不回归。

第一版暂不实现：

- 手势直接连续控制导丝；
- MR 透视融合、空间锚点和场景网格；
- 多人协作和语音控制；
- 在 VR 中启动或配置强化学习训练；
- 真实机器人硬件控制；
- 完整复制桌面端 11 面板；
- 大量 PICO 专有接口深度集成。

### 1.4 产品定位与安全边界

VR 前端定位为坐姿、固定工作区的医疗仿真导航工作站，以三维血管、导丝尖端、规划路径和安全状态为核心。手柄作为第一阶段唯一的连续控制输入；手势只用于辅助选择和观察。默认使用世界固定空间工作台，第一人称腔内模式仅作为观察模式。

本方案适用于科研仿真、强化学习训练观察、人机交互实验、教学演示和虚拟设备操作验证。在未完成医疗器械法规、真实设备控制验证和硬件级急停认证前，不应表述为真实临床手术控制系统。VR 软件急停不能替代真实机器人的物理急停装置。

---

## 2. 现有前端基础与继承范围

### 2.1 可直接继承的能力

现有系统已经具备以下可复用基础：

1. Godot 医疗导航工作站的完整模块结构，包括 DSA、腔镜、三维解剖导航、安全数据、运动控制、日志和告警。
2. `navigation_visual_v3` 统一数据契约。
3. 后端权威 `data.safety` 对象。
4. 接触力、壁距、曲率、速度、路径进度等真实或明确缺失的数据绑定。
5. WebSocket 会话创建、自动重连、旧会话帧过滤和数据过期处理。
6. 后端急停锁存、控制拒绝、恢复确认闭环。
7. 键盘、鼠标、按钮和自动导航的统一控制门控。
8. 玻璃血管、规划路径、导丝跟随相机和内窥镜相机的基础实现。

### 2.2 VR 阶段继续冻结的内容

根据当前交接约束，第一阶段 VR 开发继续冻结：

- DSA 实时影像的数据源与工具栏逻辑；
- RL 训练指标的真实绑定；
- 数据采集与回放功能；
- 真实机器人硬件握手；
- 真实扭矩闭环；
- 无来源的风险体积和伪造危险区域。

VR 版可以把冻结模块作为只读空间面板显示，但不得把硬编码数据包装成实时数据，也不得为了 VR 展示而补造缺失字段。

---

## 3. PICO 平台与 SDK 调研结论

### 3.1 PICO 4 Ultra Enterprise 硬件适配性

PICO 4 Ultra Enterprise 具备：

- Qualcomm Snapdragon XR2 Gen 2；
- 12 GB 内存；
- 每眼 2160 × 2160 显示；
- 105° 视场角；
- 90 Hz 级头显运行目标；
- 双 32 MP 彩色透视摄像头；
- Wi-Fi 7；
- 企业设备管理、系统定制和批量部署能力。

这些能力足以支持中等规模的三维血管模型、双目渲染、空间 UI、手柄追踪和局域网 WebSocket 通信。主要性能风险不是模型顶点数量本身，而是半透明血管造成的双目透明过绘、Bloom、实时阴影和多层空间面板同时刷新。

### 3.2 PICO Web 能力

PICO OS 6 提供三类 Web Runtime：

- Web App；
- PICO Browser；
- WebView。

三者均可通过标准 Web API 启动 WebXR 会话。PICO 的 WebXR 路线支持控制器、手势追踪，并在特定运行环境下支持体感追踪器能力。

WebXR 优点：

- 部署快；
- 更新快；
- 跨平台；
- 可用 Three.js、Babylon.js、A-Frame 等生态；
- 适合展示和低摩擦访问。

WebXR 对本项目的限制：

- 现有 Godot 三维、材质、相机和 UI 代码不能直接复用；
- 需要重写 WebGL/WebXR 渲染前端；
- 浏览器运行时、JavaScript 调度和资源管理增加性能不确定性；
- 对关键输入、后台状态和企业设备控制的掌控弱于原生 APK；
- 不适合作为安全关键连续控制客户端。

### 3.3 OpenXR、Godot 与 Android 导出适配性

PICO 设备支持 Khronos Android OpenXR Loader；Godot 4 内置 OpenXR，并可通过 Android Gradle Build、Godot OpenXR Vendors 插件和 Android 导出模板生成独立 APK。第一阶段采用 Compatibility 渲染器建立稳定基线，待真机验证后再评估 Mobile Vulkan。

Godot 在本项目中承担四层职责：

1. **Android 应用层**：安装包、Activity 生命周期、权限、签名和企业部署；
2. **XR 运行层**：OpenXR 会话、双目输出、头显和手柄追踪；
3. **前端表现层**：三维血管、导丝、空间 UI 和舒适性；
4. **客户端业务层**：WebSocket 状态、控制意图、安全门控和数据过期。

Android 平台专有功能采用分级补充策略：

```text
优先：OpenXR 标准能力
    ↓ 标准能力不足
Godot OpenXR Vendors Plugin
    ↓ 仍需 PICO 系统接口
Godot Android Plugin v2（Kotlin / Java）
    ↓ 仅在原生性能或接口必要时
GDExtension / Android NDK
```

因此，少量 PICO 企业接口或 Android 系统能力不足以构成整体迁移 Unity 的理由，可以通过插件边界补充，而不污染三维与业务核心。

### 3.4 Android/XR 框架比较与选型边界

| 方案 | 现有代码复用 | 首版速度 | PICO 专有能力 | 长期成本 | 本项目结论 |
|---|---:|---:|---:|---:|---|
| Godot + OpenXR | 很高 | 最快 | 中，可用插件补充 | 低 | 快速交付与当前正式版主路线 |
| Unity + PICO Unity OpenXR SDK | 低 | 中等偏慢 | 强 | 迁移成本高 | 从零项目或重度 MR 更适合 |
| Kotlin/NDK + Native OpenXR | 很低 | 最慢 | 强 | 很高 | 不适合当前交付周期 |
| WebXR | 低 | 演示快 | 有限 | 需重写 Web 前端 | 只读展示与教学辅助 |
| Flutter / React Native / Compose | 很低 | 二维页面快 | 不适合作为 XR 主体 | 双栈维护 | 不作为 VR 应用框架 |

维持 Godot 路线的条件：

- OpenXR 核心会话、手柄和双目渲染在真机稳定；
- 主要需求仍是三维导航、空间面板和标准控制；
- PICO 专有能力可以通过 Vendors 插件或少量 Android Plugin 解决；
- VR Balanced 场景能够达到可接受帧率。

只有出现以下任一情况，才启动 Unity 技术迁移评估：

- Godot 在 PICO 真机上存在无法规避的 OpenXR 兼容问题；
- 正式需求转为大规模 Passthrough MR、空间锚点、场景理解或复杂手部追踪；
- 需要大量仅由 PICO Unity SDK 稳定提供的企业或交互扩展；
- 团队已有可直接接手的 Unity XR 工程和成熟组件，迁移成本显著下降；
- Godot 性能问题经资产、材质和渲染器优化后仍无法满足验收目标。

迁移评估不应阻塞第一版 Godot APK 交付。

---

## 4. Android XR 软件架构

### 4.1 技术栈与构建链

```text
应用框架          Godot 4.7.1
主要语言          GDScript
XR 标准           OpenXR
PICO 适配         Godot OpenXR Vendors Plugin / Khronos Loader
Android 构建      Godot Gradle Android Build
运行产物          Debug APK / Signed Release APK
渲染器            Compatibility 起步，实机对比 Mobile Vulkan
通信              WebSocket，正式部署使用 WSS
数据契约          navigation_visual_v3
后端               FastAPI + SessionManager
平台扩展          Godot Android Plugin v2（按需）
原生扩展          GDExtension / Android NDK（仅必要时）
```

开发环境至少包含 OpenJDK 17、Android Studio/Android SDK、Godot Android Build Template、ADB 和 PICO 开发者模式。OpenXR 必须在应用启动阶段初始化，不能进入普通 3D 场景后再临时开启。

### 4.2 分层架构

```text
┌──────────────────────────────────────────┐
│ Android / PICO OS                        │
│ APK、生命周期、权限、企业部署            │
├──────────────────────────────────────────┤
│ OpenXR Runtime                           │
│ 双目输出、XROrigin、头显与控制器追踪      │
├──────────────────────────────────────────┤
│ XR Presentation & Input Layer            │
│ 空间 UI、射线交互、Action Map、舒适性     │
├──────────────────────────────────────────┤
│ Shared Frontend Core                     │
│ NavigationViewModel / SafetyStateStore   │
│ DataFreshnessMonitor / ControlRouter     │
├──────────────────────────────────────────┤
│ Shared Rendering                         │
│ Vessel / Guidewire / Path / Risk Visuals │
├──────────────────────────────────────────┤
│ WebSocket Client                         │
├──────────────────────────────────────────┤
│ FastAPI / SessionManager / Physics       │
└──────────────────────────────────────────┘
```

### 4.3 双入口、共享核心

桌面端和 VR 端不共用布局，但必须共用状态、通信、安全和三维渲染核心：

- `DesktopMain.tscn`：保留当前 1920×1080 医疗工作站；
- `MainXR.tscn`：作为 PICO APK 主入口；
- `NavigationViewModel`：统一消费 `navigation_visual_v3`；
- `ControlIntentRouter`：统一控制来源、归零和急停门控；
- `VesselNavigationScene`：桌面和 XR 共用三维血管、导丝与路径；
- UI 不直接解析 WebSocket 原始 JSON；
- XR 输入不直接发送网络消息，必须经过控制路由和安全门控。

### 4.4 推荐节点结构

```text
MainXR : Node3D
├── XRBootstrap
├── XROrigin3D
│   ├── XRCamera3D
│   ├── LeftController : XRController3D
│   │   ├── RayInteractor
│   │   └── ControllerModel
│   └── RightController : XRController3D
│       ├── RayInteractor
│       └── ControllerModel
├── ClinicalWorkspace
│   ├── VesselNavigationScene
│   ├── GuidewireTipIndicator
│   └── TargetIndicator
├── FloatingPanels
│   ├── CriticalSafetyPanel
│   ├── ImagePanel
│   ├── NavigationPanel
│   └── ControlDock
├── SharedServices
│   ├── WebSocketClient
│   ├── NavigationViewModel
│   ├── SafetyStateStore
│   ├── DataFreshnessMonitor
│   └── ControlIntentRouter
└── XRServices
    ├── XRInputAdapter
    ├── XRFocusGuard
    ├── XRComfortController
    └── XRPerformanceManager
```

### 4.5 推荐目录结构

```text
godot_client/
├── scenes/
│   ├── desktop/MainDesktop.tscn
│   ├── xr/MainXR.tscn
│   ├── xr/XRClinicalWorkspace.tscn
│   ├── xr/panels/XRCriticalSafetyPanel.tscn
│   ├── xr/panels/XRImagePanel.tscn
│   ├── xr/panels/XRNavigationPanel.tscn
│   ├── xr/panels/XRControlDock.tscn
│   └── shared/VesselNavigationScene.tscn
├── scripts/
│   ├── shared/navigation_view_model.gd
│   ├── shared/safety_state_store.gd
│   ├── shared/data_freshness_monitor.gd
│   ├── shared/control_intent_router.gd
│   ├── xr/xr_bootstrap.gd
│   ├── xr/xr_input_adapter.gd
│   ├── xr/xr_ui_pointer.gd
│   ├── xr/xr_focus_guard.gd
│   ├── xr/xr_comfort_controller.gd
│   └── xr/xr_performance_manager.gd
├── addons/
│   └── godotopenxrvendors/
├── android/                    # Gradle Build Template
└── openxr_action_map.tres
```

### 4.6 平台扩展边界

第一版业务代码只能依赖 OpenXR 标准动作和 Godot 通用 API。PICO 专有接口必须封装在 `PicoPlatformBridge` 中，并通过 capability check 调用：

```text
PicoPlatformBridge
├── is_feature_available(feature)
├── request_recenter()
├── get_device_info()
├── set_kiosk_related_option()
└── get_runtime_capabilities()
```

平台桥接失败不得影响 WebSocket、安全状态、急停和中性控制帧发送。

---

## 5. VR 空间界面设计

### 5.1 使用场景

默认采用：

- 坐姿；
- 1.5 m × 1.5 m 以上安全区域；
- 不要求用户在房间内移动；
- 控制对象位于身体前方；
- 支持一键重置工作区中心；
- 可在实验室、教室和固定工位使用。

### 5.2 空间布局

建议把原 11 个模块重组为 5 个空间层级。

#### A. 中央三维导航工作台

- 位于用户前方约 1.2–1.8 m；
- 血管模型宽度约 0.8–1.2 m；
- 支持桌面模型和近景跟随两种尺度；
- 当前导丝尖端、规划路线和目标分叉保持最高视觉优先级；
- 用户可抓取工作台边缘旋转、缩放和复位；
- 默认不允许直接抓取导丝改变物理状态。

#### B. 右侧常驻安全面板

常驻显示：

- 后端安全等级；
- 接触力；
- 壁距；
- 速度；
- 剩余距离；
- 数据新鲜度；
- 急停状态；
- 最关键 `reason_code`。

该面板必须始终可见，不随普通面板关闭。

#### C. 左侧影像面板

使用一个可切换空间屏幕承载：

- DSA 只读画面；
- 腔镜画面；
- 必要的图例和数据状态。

第一阶段不同时打开两个大画面，避免双目分辨率和注意力浪费。通过顶部标签切换 DSA/Endoscope。

#### D. 下方运动控制坞

显示：

- 当前模式；
- 推进输入；
- 旋转输入；
- 控制强度；
- Deadman 状态；
- 后端控制确认；
- 控制被拒绝原因。

控制坞只展示控制状态，连续输入主要由手柄完成。

#### E. 次级工具与日志面板

日志、连接详情、模型切换、渲染参数和调试信息放入可召唤的次级面板，不常驻主视野。

### 5.3 空间 UI 尺寸建议

```text
主工作台距离             1.2–1.8 m
主工作台宽度             0.8–1.2 m
侧面面板距离             1.1–1.6 m
面板水平夹角             15°–30° 朝向用户
主要文字视角高度         不小于约 0.5°
正文建议物理高度         8–12 mm（按距离实机校准）
关键数字                 16–25 mm
主要交互按钮             35–50 mm
按钮间最小间距           8–12 mm
```

不建议使用完全头部锁定的整块 HUD。关键告警可以短时进入视野安全区，但主体面板应世界固定或身体固定，避免头部转动时 UI 跟随造成不适。

---

## 6. 三维血管与相机模式

### 6.1 默认模式：Clinical Tabletop

默认打开世界固定的三维血管工作台：

- 用户可以自然移动头部观察；
- 血管不跟随头部移动；
- 用户不会因导丝推进产生被动位移；
- 适合连续控制和安全观察；
- 是 VR 正式版的默认控制模式。

### 6.2 Guidewire Navigation Follow

作为局部导航观察模式：

- 导丝位于视图下部；
- 路线尽量向上；
- 使用规划路径切线和平行移动框架保持方向连续；
- 急弯提前转向；
- 路径重规划使用平滑过渡；
- 用户头部姿态只在小范围叠加，不直接覆盖基准相机姿态。

### 6.3 Endoscope Panel

第一阶段将腔内视角放在左侧浮动屏幕中，而不是让用户整个视野进入血管：

- 保留内窥镜沉浸感；
- 避免持续自动移动造成晕动；
- 保留周边安全信息；
- 更适合边看腔镜边控制。

### 6.4 Experimental Immersive Endoscope

完全沉浸腔内模式仅作为实验功能：

- 相机采用侧后方偏置；
- 不固定在导丝中心轴线上；
- 使用持久侧向向量避免弯道左右跳动；
- 使用 SDF/内壁距离约束防止穿壁；
- 禁用传统屏幕空间 DOF；
- 限制自动相机平移和角速度；
- 进入前显示舒适性提示；
- 任何危险状态立即退出自动前进或切回稳定工作台。

### 6.5 Tree Overview

用于：

- 全局拓扑查看；
- 选择目标分支；
- 模型复位；
- 教学演示。

不作为精细控制时的默认视角。

---

## 7. 输入与交互方案

### 7.1 输入优先级

```text
手柄控制器 > 手势追踪 > 头部凝视
```

原因：

- 导丝推进和旋转需要连续、可控、可回零的输入；
- 手势追踪容易受遮挡和姿态影响；
- 凝视不适合精确连续控制；
- 手势更适合面板选择、模型旋转和教学演示。

### 7.2 OpenXR Action Map 与输入适配

业务代码不得读取 Android KeyCode，也不得硬编码某个 PICO 手柄按钮。所有输入先映射到语义动作：

```text
/ui/select
/ui/back
/ui/scroll
/workspace/grab
/workspace/scale
/view/reset
/view/cycle
/panel/toggle
/control/deadman
/control/push_axis
/control/rotate_axis
/control/emergency_stop
/control/resume_request
```

`XRInputAdapter` 负责把运行时输入配置转换成统一意图：

```text
XRController3D / OpenXR Action
    ↓
XRInputAdapter
    ↓ push, rotate, deadman, select
ControlIntentRouter
    ↓ 安全门控、限幅、序号、归零
WebSocketClient
```

手柄映射建议：

- 左摇杆 Y：推进/回撤；
- 右摇杆 X：旋转；
- 一侧 Grip 或 Trigger：Deadman；
- 独立按键或双键组合：软件急停；
- A/X：UI 选择；
- B/Y：返回或关闭次级面板；
- 摇杆按下：重置观察视角，不改变后端控制状态。

### 7.3 连续控制与失效安全逻辑

只有 `deadman_active=true` 且所有控制条件均有效时，才允许发送非零推进或旋转：

```text
允许非零控制 =
OpenXR 会话可见
AND 应用拥有焦点
AND 指定手柄正在追踪
AND WebSocket 会话 ready
AND 导航数据未过期
AND 后端未急停锁存
AND Deadman 持续按下
```

以下任一情况发生，客户端必须在同一帧将 `push=0`、`rotate=0`，阻断后续非零输入并发送中性控制帧：

- 松开 Deadman；
- 手柄追踪丢失；
- 头显摘下或应用失去焦点；
- OpenXR session 不可见或停止；
- WebSocket 断开或会话未 ready；
- `navigation_visual_v3` 数据过期；
- 收到 `stop_required=true`；
- 本地急停意图或后端急停锁存。

输入参数建议：

- 摇杆死区：0.08–0.15；
- 使用非线性低速优先响应曲线；
- 默认速度档位为“慢”或“中低速”；
- 最大推进、旋转限幅由后端配置确认；
- 恢复后必须重新按下 Deadman，不得恢复旧摇杆值；
- UI 显示“用户意图值”和“后端实际生效值”，不得混为同一数值。

### 7.4 急停

VR 急停必须提供两条入口：

- 手柄上的专用 OpenXR 动作；
- 右侧安全面板中的常驻红色按钮。

触发后：

```text
本地同帧阻断全部控制
    ↓
发送 emergency_stop
    ↓
界面显示“急停请求中”
    ↓
收到 emergency_stop_confirmed
    ↓
显示“后端已锁存”
```

恢复必须：

- 使用独立 `resume_request`；
- 显示明确确认面板；
- 等待后端 `resume_confirmed`；
- 不恢复旧控制值；
- 不重启旧 ShapeIntent；
- 不以摘下或重新戴上头显作为恢复条件。

### 7.5 手势交互

手势追踪建议只用于：

- 指向和点击；
- 抓取并旋转血管工作台；
- 缩放；
- 面板开关；
- 教学标注。

第一阶段不使用手势直接连续控制导丝推进和旋转。

---

## 8. 数据通信与状态管理

### 8.1 继续使用 `navigation_visual_v3`

VR 客户端继续消费：

```text
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
risk_regions
safety
control_state
```

规则保持：

- `null` 不显示为 0；
- `unknown/stale` 不显示为安全；
- 安全结论只来自后端；
- 空 `risk_regions` 不生成风险体积；
- `guided` 明确显示为运动学演示。

### 8.2 建议增加的客户端控制字段

VR 控制消息建议增加：

```json
{
  "type": "control",
  "control_source": "pico_vr",
  "input_sequence": 1024,
  "client_timestamp_ms": 1785480000000,
  "deadman_active": true,
  "push": 0.25,
  "rotate": -0.10
}
```

用途：

- 区分桌面、VR 和自动导航控制来源；
- 识别乱序和重复输入；
- 记录输入延迟；
- 后端可拒绝 `deadman_active=false` 的非零控制；
- 支持审计和回放。

### 8.3 推荐更新频率

```text
头部与手柄姿态             由 OpenXR 本地每帧更新
三维导丝插值               每帧
后端导航状态               30–60 Hz，按实际物理频率配置
连续控制指令               30–60 Hz
安全事件                   到达即处理，不等待 UI 刷新
关键数值 UI                5–10 Hz
日志列表                   2–5 Hz 或事件驱动
连接心跳                   独立低频机制
```

### 8.4 数据过期

建议：

- 状态超过预设时限未更新，路径由白/黄/红切为灰色；
- 安全面板显示“数据已过期”；
- 禁止继续非零控制；
- 断线后立即本地门控；
- 重连后重新建立会话；
- 旧 session ID 数据继续丢弃；
- 急停意图在重连后重新锁存。

---

## 9. 渲染与性能方案

### 9.1 性能目标

PICO 4 Ultra 的目标显示刷新率为 90 Hz，应用帧预算约为：

```text
1000 / 90 ≈ 11.1 ms / frame
```

建议验收目标：

- 正常工作场景稳定接近 90 Hz；
- CPU 与 GPU 帧时间均留出运行时余量；
- 连续 10 分钟无明显热降频导致的持续卡顿；
- 主控制模式不依赖异步空间重投影掩盖长期性能不足；
- 性能不足时优先降低画质，而不是降低输入和安全状态处理频率。

### 9.2 渲染器选择

第一阶段：

```text
Godot Compatibility Renderer
```

原因：

- Godot 官方对 Android XR 的保守推荐；
- 更适合先完成 PICO 实机兼容和稳定性验证；
- 现有项目的半透明材质需要先建立可靠基线。

第二阶段可对比 Mobile Vulkan：

- 只在实机稳定后评估；
- 分别测试透明材质、Bloom、MSAA、shader 编译和热稳定性；
- 不因桌面端效果更好就直接切换。

### 9.3 血管资产档位

建议建立 VR 专用资产：

| 档位 | 目标 | 建议用途 |
|---|---|---|
| VR High | 近景主分支高质量 | 局部导航、截图 |
| VR Balanced | 默认 | 正式运行 |
| VR Lite | 性能降级 | 热降频或复杂病例 |

建议：

- 默认总可见面数控制在实机可承受范围；
- 按 branch ID 拆分 MeshInstance3D 或 surface；
- 近景分支使用高细节；
- 远端分支使用 LOD；
- 隐藏的分支停止不必要更新；
- 高质量视觉 mesh 与碰撞/SDF 资产继续分离。

### 9.4 透明血管优化

半透明血管是主要 GPU 风险。建议：

- 降低重叠透明层数；
- 远端分支降低透明度和 emission；
- 只对主要血管启用玻璃效果；
- 关闭透明物体阴影；
- 控制 Bloom 强度；
- 不启用传统实时 DOF；
- 用距离衰减和材质层次替代景深；
- 按需消隐遮挡路径的非规划分支；
- 必要时将远端分支改为较不透明的简化材质。

### 9.5 灯光与后处理

```text
主光源                 1 个
辅助光                 0–1 个
实时阴影               默认关闭
环境光                 低强度
Bloom                   低或中
DOF                     VR 关闭
SSR                     关闭
体积雾                 谨慎使用或关闭
MSAA                    从 2× 开始实测
动态分辨率             建议支持
固定注视点渲染         运行时支持确认后启用
```

### 9.6 空间面板优化

- 使用 SubViewport 将 2D UI 渲染到 3D 面板；
- 非活动面板降低更新频率；
- 隐藏面板暂停 Viewport 更新；
- DSA/腔镜不同时全分辨率刷新；
- 日志列表不逐帧重排；
- 数值更新与 XR 帧率解耦。

---

## 10. 舒适性设计

### 10.1 默认不进行人工移动

本项目不需要摇杆移动用户在虚拟房间中行走。用户始终位于固定工位，避免额外晕动来源。

### 10.2 被动相机运动限制

- 默认工作台不移动用户视点；
- 跟随相机只在浮动屏幕或局部观察窗口中运行；
- 完全沉浸内窥镜模式限制线速度和角速度；
- 路径急转时提前平滑旋转；
- 路径重规划不得瞬间跳转；
- 危险或数据中断时停止自动相机前进。

### 10.3 重置与佩戴状态

- 提供一键 Recenter；
- 重新佩戴头显后不自动恢复非零控制；
- 追踪丢失时阻断连续控制；
- 手柄丢失时发送中性控制；
- 应用失去焦点或进入系统菜单时阻断控制；
- 从后台恢复时要求重新按下 Deadman。

---

## 11. 企业部署方案

### 11.1 开发阶段

- 开启开发者选项和 USB 调试；
- 配置 Android SDK、OpenJDK 和 Godot Android Export；
- 使用 USB 一键部署 APK；
- 使用 PICO 性能监测工具、ADB 日志和 Godot 日志；
- 后端运行在同一局域网工作站；
- 开发配置使用明确的测试病例和仿真引擎。

### 11.2 实验室部署

推荐：

- 签名 APK；
- 固定后端地址通过配置文件或企业配置包下发；
- 使用 PICO Business Suite / Device Manager 批量安装；
- 配置 Kiosk Mode 或 Quick Launch；
- 限制用户进入无关系统应用；
- 统一 Wi-Fi、时区、亮度和边界策略；
- 记录设备型号、系统版本、应用版本和病例版本。

### 11.3 网络与数据

- 优先使用隔离局域网；
- 正式环境使用 WSS；
- 不默认上传透视摄像头画面；
- 不采集头部、手部和房间数据，除非实验明确需要并取得同意；
- 控制与安全日志带 session ID、设备 ID、时间戳和版本号；
- 患者或病例数据需要脱敏和权限控制。

---

## 12. 快速交付实施路线

实施顺序以“尽早拿到 PICO 真机可运行闭环”为原则，不先重构所有桌面代码，也不先开发完整空间界面。

### 阶段 A：Android OpenXR 冒烟版本

交付：

- 安装 Godot Android Build Template 和 OpenXR Vendors 插件；
- 配置 OpenJDK 17、Android SDK、Gradle 导出和 PICO 开发者模式；
- 新建 `MainXR.tscn`、`XROrigin3D`、`XRCamera3D` 和左右控制器；
- 在 PICO 中安装并启动 Debug APK；
- 显示简单测试模型、XR 状态和帧率；
- 验证应用失焦、退出和重新进入的生命周期事件。

退出条件：真机可以稳定启动 OpenXR 会话并识别头显与手柄。

### 阶段 B：最小业务复用

交付：

- 直接复用现有 WebSocketClient；
- 接收 `navigation_visual_v3`；
- 显示血管、导丝、规划路径和目标点；
- 显示连接、数据新鲜度和后端安全状态；
- 先允许少量适配代码，避免为追求完美架构阻塞首屏运行。

退出条件：PICO 中能看到与桌面端一致的权威导航状态。

### 阶段 C：共享核心重构

交付：

- 从桌面 HUD 中抽离 `NavigationViewModel`；
- 建立 `SafetyStateStore`、`DataFreshnessMonitor` 和 `ControlIntentRouter`；
- 桌面和 VR 同时消费共享状态；
- 建立自动化契约测试，保证桌面端不回归。

退出条件：UI 层不再直接解析原始 WebSocket JSON，两个入口共用安全门控。

### 阶段 D：手柄控制与安全闭环

交付：

- OpenXR Action Map 和 `XRInputAdapter`；
- 手柄射线 UI；
- Deadman 推进/旋转；
- 松开、追踪丢失、失焦、断线和数据过期同帧归零；
- 急停请求、后端锁存确认和恢复确认；
- 控制来源、输入序号和客户端时间戳。

退出条件：所有控制入口均通过前端门控和后端锁存双重验证。

### 阶段 E：最小空间工作台

交付：

- 中央三维血管工作台；
- 右侧常驻安全面板；
- 左侧 DSA/腔镜只读切换面板；
- 下方控制状态坞；
- 工作区复位、抓取和有限缩放；
- 日志、连接和调试面板按需打开。

退出条件：无需复制桌面 11 面板即可完成核心观察和控制任务。

### 阶段 F：观察模式与性能档位

交付：

- Clinical Tabletop；
- Guidewire Navigation Follow；
- Endoscope Panel；
- Tree Overview；
- VR Balanced / VR Lite 资产与材质；
- 分支 LOD、透明过绘优化和动态分辨率；
- 真机连续运行与热稳定性测试。

`Experimental Immersive Endoscope` 不作为快速交付阻塞项。

### 阶段 G：发布与企业部署

交付：

- 签名 Release APK；
- 后端地址和运行参数外部配置；
- Kiosk/Quick Launch 和批量安装说明；
- 设备、系统、应用、病例和协议版本记录；
- 实机功能、安全、性能和舒适性验收报告。

---

## 13. 测试与验收标准

### 13.1 功能验收

- OpenXR 初始化成功；
- 左右手柄追踪正常；
- 头部移动不改变后端仿真状态；
- 三维血管、导丝和路径坐标一致；
- `navigation_visual_v3` 正常解析；
- 桌面端与 VR 端显示同一权威状态；
- 断线、重连和旧 session 过滤正常；
- 冻结面板不被误描述为实时功能。

### 13.2 控制与安全验收

- Deadman 未按下时，不能发送非零控制；
- Deadman 松开后，同帧本地归零；
- 手柄追踪丢失后立即归零；
- 应用失焦后立即归零；
- 急停触发后，本地同帧阻断；
- 后端确认后显示“已锁存”；
- 急停锁存期间所有非零控制被拒绝；
- Resume 不恢复旧输入和旧自动导航；
- `stop_required=true` 不等待颜色动画；
- 数据过期时路径变灰并阻断控制。

### 13.3 性能验收

- 默认病例接近 90 Hz；
- 正常场景无持续性严重掉帧；
- 透明血管不过曝；
- 头部转动无明显抖动；
- 空间面板开启和关闭不产生长时间卡顿；
- 运行一段时间后无明显内存持续增长；
- 高复杂度病例可自动或手动切换 VR Lite 档位。

### 13.4 可读性验收

- 安全等级不只依赖颜色；
- 危险、停止、数据过期有图标和文字；
- 关键数值在正常坐姿距离可读；
- 主控制区不需要频繁转头超过舒适范围；
- DSA/腔镜面板不会遮挡安全面板；
- 工作台缩放后仍能定位导丝尖端。

### 13.5 舒适性验收

- 默认模式连续使用无强制虚拟移动；
- 路径急弯不产生相机瞬间翻滚；
- 第一人称模式退出入口清晰；
- 重置工作区不会改变控制模式；
- 重新佩戴后不自动恢复运动；
- 由多名测试者完成连续体验并记录眩晕、眼疲劳和操作错误。

---

## 14. 主要风险、降级与框架切换条件

| 风险 | 影响 | 当前应对 | 降级/切换条件 |
|---|---|---|---|
| Godot 与 PICO OpenXR 兼容差异 | 无法启动或输入异常 | 先做最小真机冒烟；只依赖 OpenXR 核心；使用 Vendors 插件 | 核心会话或控制器问题无法通过插件、系统版本或配置解决时评估 Unity |
| PICO 专有企业接口缺失 | Kiosk、设备信息等能力不足 | 通过 `PicoPlatformBridge` 和 Godot Android Plugin v2 补充 | 专有接口数量过多、维护成本超过迁移收益时评估 Unity |
| 重度 MR 需求提前进入 | 范围膨胀、厂商依赖增加 | 第一版纯 VR；MR 单独立项 | 正式产品转为 Passthrough、锚点、场景理解主导时评估 Unity + PICO SDK |
| 透明血管双目过绘 | 掉帧和热降频 | LOD、分支拆分、降低透明层、VR 专用材质 | VR Lite 仍无法满足最低帧率时比较 Unity 或 PC 串流路线 |
| 直接复制桌面 11 面板 | 可读性差、视觉拥挤 | 重组为空间工作台、常驻安全面板和按需面板 | 不迁移框架，继续压缩信息架构 |
| 第一人称自动前进 | 晕动 | 默认世界固定工作台，腔镜先使用浮动屏幕 | 完全沉浸模式关闭，不影响主交付 |
| 手势控制不稳定 | 误操作 | 连续控制仅使用手柄 + Deadman | 手势功能降级为只读或关闭 |
| 网络中断 | 控制和状态不确定 | 本地同帧归零、数据过期、后端锁存、session 过滤 | 不允许离线继续非零控制 |
| 软件急停被误认为硬件急停 | 安全边界混乱 | 明确仅为仿真/上层急停；真实设备必须有物理急停 | 不因框架选择改变此边界 |
| WebXR 被当作生产主路线 | 重写、性能和安全风险 | 限定为演示和只读辅助 | 仅当产品转为纯展示时考虑 WebXR 主路线 |

框架切换必须经过单独技术验证，不得仅因某一项 PICO API 暂未接入而立即重写现有 Godot 工程。

---

## 15. 最终工程基线

### 15.1 固定技术选择

```text
Android 应用框架     Godot 4.7.1
脚本                 GDScript
XR                   OpenXR
构建                 Godot Gradle Android Export
目标设备             PICO 4 Ultra Enterprise
主要输入             OpenXR 手柄 + Deadman
界面                 世界固定空间工作台
通信                 现有 FastAPI / WebSocket
安全                 前端同帧门控 + 后端权威锁存
平台扩展             Vendors Plugin / Android Plugin 按需补充
```

### 15.2 第一版交付判定

第一版只有在以下条件同时满足时才算完成：

- PICO 真机可安装、启动和退出；
- OpenXR 头部与双手柄追踪稳定；
- 三维血管、导丝和规划路径与后端状态一致；
- 手柄 Deadman 推进/旋转可用；
- 失焦、丢失追踪、断线和数据过期均立即归零；
- 软件急停完成请求、锁存确认和恢复确认闭环；
- 常驻安全状态清晰可读；
- VR Balanced 达到约定性能基线；
- 现有桌面端通过回归测试。

### 15.3 后续演进原则

- 少量 PICO 特有能力优先通过平台桥接补充，不迁移整个工程；
- 手势、MR、空间锚点和场景理解独立排期，不阻塞核心 VR；
- Unity 只在兼容性、重度 MR、专有接口规模或性能测试给出明确证据后评估；
- WebXR 始终与主控制客户端解耦；
- 任何框架变化都不得削弱后端权威安全状态、急停锁存和数据可信性规则。

### 15.4 明确不纳入当前交付

- 完整 WebXR 重写；
- Flutter、React Native 或 Compose 双栈 UI；
- 全沉浸内窥镜默认控制；
- 未验证的 MR 透视交互；
- 手势连续控制导丝；
- 伪实时 RL、DSA 或风险区域；
- 真实机器人临床控制表述。

---

## 16. 官方调研资料

1. PICO Web 开发资源：  
   https://developer-cn.picoxr.com/resources/?platform=web#sdk
2. PICO WebXR Overview：  
   https://developer.picoxr.com/document/web/webxr/
3. PICO WebXR Performance：  
   https://developer.picoxr.com/document/web/webxr-performance/
4. PICO 4 Ultra Enterprise 产品与规格：  
   https://business.picoxr.com/us/products/pico4-ultra-enterprise
5. PICO OpenXR 1.1 支持说明：  
   https://developer.picoxr.com/blog/openxr-standard-1point1/
6. PICO Native OpenXR：  
   https://developer.picoxr.com/document/native/
7. PICO Unity OpenXR SDK：  
   https://developer.picoxr.com/zh/document/unity-openxr/
8. PICO Unity 项目配置：  
   https://developer.picoxr.com/zh/document/unity/complete-project-settings/
9. Godot Android XR 部署：  
   https://docs.godotengine.org/zh-cn/4.x/tutorials/xr/deploying_to_android.html
10. Godot XR 设置：  
    https://docs.godotengine.org/en/stable/tutorials/xr/setting_up_xr.html
11. Godot Android Plugin：  
    https://docs.godotengine.org/zh-cn/4.x/tutorials/platform/android/android_plugin.html
12. Godot OpenXR Vendors Plugin：  
    https://github.com/GodotVR/godot_openxr_vendors
13. PICO Business Suite：  
    https://business.picoxr.com/global/software/business-suite
