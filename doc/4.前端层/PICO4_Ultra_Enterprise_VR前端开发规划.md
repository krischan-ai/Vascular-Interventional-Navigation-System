# PICO 4 Ultra Enterprise VR 前端开发规划

> 项目：CathSim 医疗导管/导丝导航与强化学习仿真平台
>
> 目标设备：PICO 4 Ultra Enterprise
>
> 设计依据：`PICO4_Ultra_Enterprise_VR前端设计方案.md` v3.0（视觉基准融合重构版）
>
> 技术基线：Godot 4.7.1 + OpenXR 1.1 + Android Gradle Build
>
> 规划版本：v3.2（开发启动核验版）
>
> 更新日期：2026-08-03
>
> 当前状态：M0 进行中；M1 准备中；M2–M8 未开始

---

## 1. 规划目的与交付结论

本规划把设计方案转换为可排期、可验证、可审计的开发工作。目标是在保留桌面客户端的前提下，交付可安装到 PICO 4 Ultra Enterprise 的 Android OpenXR APK，并形成从 XR 输入、前端安全门控、WebSocket、后端锁存到物理引擎的完整仿真控制闭环。

第一版只有同时满足以下结果才算完成：

1. APK 可重复安装、启动、退出和重新进入；
2. OpenXR Session、头部 6DoF 与双手柄追踪稳定；
3. 血管、导丝、规划路径、目标点与后端权威状态一致；
4. Aim Pose 射线可操作空间 UI；
5. Deadman 持续使能下可推进和旋转；
6. 失焦、Action inactive、追踪丢失、Reference Space 变化、断线、数据过期或急停时，同一客户端更新周期归零；
7. 急停完成“本地锁定—后端锁存确认—非零控制拒绝—显式恢复确认—重新按下 Deadman”的闭环；
8. 安全、风险、接触力、壁距、速度、进度和数据新鲜度来源真实、状态可读；
9. 四区空间构图、深色青蓝视觉语言、信息层级和交互反馈达到冻结的真机视觉基准；
10. 左侧默认同时显示 DSA 占位图与动态腔镜，支持双画面、DSA 主视图和腔镜主视图三种布局；
11. 腔镜由独立 `EndoscopeCamera3D → SubViewport → ViewportTexture` 实时渲染，不使用静态图或循环视频伪装；
12. 默认性能档达到 M0 冻结的真机基线，完成 10 分钟迭代稳定性和 30 分钟发布稳定性测试；
13. 签名 Release APK、配置、版本矩阵、部署/回退说明和验收证据齐备；
14. 桌面端既有功能和自动化回归通过；
15. 不含伪实时数据、伪风险区域、伪连接状态或未说明的硬编码业务状态。

本前端仅用于仿真、科研、人机交互和教学，不表述为真实临床控制系统；软件急停不替代真实设备的物理急停。

---

## 2. 固定基线与不可破坏约束

### 2.1 技术基线

```text
应用框架        Godot 4.7.1
XR 标准         OpenXR 1.1
PICO 适配       PICO OpenXR Runtime + 匹配的 Vendors Plugin 稳定版
Android 构建    Gradle Build，arm64
首选渲染器      Compatibility / OpenGL ES
通信协议        现有 WebSocket / navigation_visual_v3
安全权威        后端 data.safety
视觉基准        设计方案 v3.0 参考图的构图、层级与视觉语言
DSA 首版        明示占位图，不接实时数据源
腔镜首版        单动态 SubViewport 本地实时渲染
交付形态        Android APK
```

PICO OpenXR SDK v3.0.0 不作为第一版直接依赖。平台能力依次选择：Godot 标准 API、OpenXR 核心能力、Vendors Plugin、Godot Android Plugin、GDExtension/JNI/PICO Native SDK。引入后两层前必须记录缺失能力、扩展名、最低系统版本、降级行为和桌面端影响。

### 2.2 安全不变量

连续控制只在以下条件全部成立时开放：

```text
session_state == FOCUSED
and xr_actions_active == true
and controller_tracking_valid == true
and deadman_pressed == true
and websocket_ready == true
and navigation_data_fresh == true
and backend_emergency_stop_latched == false
and local_control_fault == false
```

任一条件失效时必须先本地执行：

```text
push = 0
rotate = 0
controls_blocked = true
```

随后再尽力发送中性帧或急停请求，不得等待 UI 动画或后端确认才停止本地输入。重新聚焦、重新追踪、Reference Space 复位或急停恢复后，不得自动恢复旧摇杆值、Deadman、ShapeIntent 或自动导航目标。

### 2.3 状态与数据真实性

- `data.safety` 是风险和停止要求的唯一业务权威来源；
- 前端可做更保守的本地阻断，但不得形成第二套“安全”结论；
- 缺失值显示 `— / unknown / stale`，不得转换为 0；
- 空 `risk_regions` 保持为空，不生成危险体积；
- 失效数据不得继续显示成当前实时值；
- WebSocket 重连后不得恢复旧控制目标；
- 连接状态必须拆分为网络、后端会话、物理引擎、XR Session、数据流和控制门控，不得用单一 `CONNECTED` 概括；
- `SAFE` 只表达后端 `data.safety`，不得由连接正常、Deadman 按下或本地渲染成功推导；
- DSA 必须标注“占位图 / Demo Image / 实时状态未接入”，不得显示伪 `LIVE`、FPS 或延迟；
- RL、采集/回放等冻结模块只能标注真实来源、只读或不可用。

### 2.4 XR 生命周期语义

| Session 状态 | 渲染 | 输入与控制 |
|---|---|---|
| IDLE / READY | 初始化或降载 | 禁止控制 |
| SYNCHRONIZED | 可维持帧循环 | 禁止控制 |
| VISIBLE | 保持合理画面 | 无输入焦点，必须归零并锁定 |
| FOCUSED | 正常渲染 | 仅在全部安全条件成立时允许 |
| STOPPING | 退出 XR 帧循环 | 立即归零并锁定 |
| LOSS_PENDING | 重建 Session 资源 | 锁定，禁止恢复旧输入 |
| EXITING | 结束体验 | 锁定并退出 |

`VISIBLE` 不等于可控制。Activity 生命周期回调不能替代 OpenXR Session 状态判断。

---

## 3. 范围

### 3.1 第一版范围

- Android OpenXR 应用入口、导出和 PICO 真机运行；
- 头部与双手柄追踪、语义 Action、Aim/Grip Pose；
- 桌面与 XR 双入口和共享业务核心；
- 状态快照、数据新鲜度、Session 过滤和重连；
- 三维血管、导丝、规划路径、目标点和统一坐标转换；
- Clinical Tabletop、Guidewire Follow、Endoscope Offset Follow 和 Tree Overview；
- 中央三维工作台、左侧双影像、右侧安全、下方控制坞、左下菜单和右下更多入口；
- DSA 明示占位图及预留数据接口边界；
- 动态腔镜 `SubViewport`、侧后偏置相机、局部照明和导丝可见性；
- 双画面、DSA 主视图和腔镜主视图三种影像布局，非主画面以画中画保留；
- Aim 射线、UI 圆形光标、三维表面环形光标及无命中射线收短/隐藏；
- Deadman 推进/旋转、统一安全门控、急停与恢复闭环；
- VR High、VR Balanced、VR Safe、Debug 四档；
- Debug APK、签名 Release APK、企业部署与回退资料；
- 自动化、桌面回归、PICO 功能/安全/性能/舒适性验收。

### 3.2 第一版不纳入

- 手势连续控制导丝；
- 完整手部骨骼、手指追踪和手势识别；
- MR 透视、空间锚点和场景网格；
- 语音控制和多人协作；
- VR 内启动、配置或管理 RL 训练；
- 真实机器人控制与真实扭矩闭环；
- 完整复制桌面端 11 个窗口；
- Experimental Immersive Endoscope 作为主要控制视角；
- DSA 实时流、真实 DSA 工具链、录制和医学测量；
- 无真实来源的风险体积或伪实时状态；
- WebXR 主控制客户端；
- 纯 C/C++ Native 重写或大规模 PICO 专属扩展。

### 3.3 继续冻结

- DSA 实时影像数据源、采集/解码和真实工具栏逻辑；
- RL 训练面板真实指标绑定；
- 数据采集与回放功能；
- 真实机器人握手；
- 无权威来源的危险区域生成。

第一版仍须交付 DSA 占位面板、状态标签和未来数据接口边界，但不得把占位资产包装成实时能力。冻结模块如需在 VR 中出现，只能作为有明确来源标识的只读或禁用面板，不得扩大业务范围。

---

## 4. 目标架构与命名基线

### 4.1 双入口、共享核心

```text
MainDesktop.tscn ─┐
                  ├─ NavigationStateStore
MainXR.tscn ──────┤  SafetyStateStore
                  │  DataFreshnessMonitor
                  │  EmergencyStopCoordinator
                  │  ControlRouter
                  │  CoordinateAdapter
                  └─ WebSocketClient
```

共享核心负责协议、状态、安全、控制和坐标；桌面与 XR 只拥有各自的展示与输入适配。UI 不解析原始 WebSocket JSON，输入适配器不直接发送控制消息。

### 4.2 XR 专属模块

```text
XRMain
├── XRBootstrap
├── XRSessionStateBridge
├── XROrigin3D
│   ├── XRCamera3D
│   ├── LeftController
│   └── RightController
├── ClinicalWorkspace
├── SpatialPanels
│   ├── ImagingPanel
│   │   ├── DSAPlaceholderView
│   │   └── EndoscopeViewportView
│   ├── SafetyPanel
│   └── MotionControlDock
├── EndoscopeCameraRig
├── XRInputAdapter
├── XRSafetyGate
├── XRControlRouterAdapter
└── XRPerformanceManager
```

规范名称为：`NavigationStateStore`、`SafetyStateStore`、`DataFreshnessMonitor`、`EmergencyStopCoordinator`、`ControlRouter`、`CoordinateAdapter`、`XRSessionStateBridge`、`XRInputAdapter`、`XRInputSnapshot` 和 `XRSafetyGate`。

现有 `NavigationViewModel`、`ControlIntentRouter`、`XRFocusGuard` 等名称若已落地，可在 M0 形成一次性迁移映射；不得长期保留职责重叠的两套实现。

### 4.3 更新与线程模型

| 更新层 | 频率 | 约束 |
|---|---:|---|
| XR 渲染与头手追踪 | 设备帧率 | 只消费当前不可变快照 |
| XR 输入采样 | 每 XR 帧 | 生成单一 `XRInputSnapshot`，限频发送 |
| 后端业务状态 | 20–60 Hz 或现协议频率 | 网络线程只写线程安全状态存储 |
| 低频 UI | 2–10 Hz | 日志、趋势和连接详情限频 |

急停消息优先级高于普通控制；控制发送走单一队列；WebSocket 回调不直接操作 XR 节点。

### 4.4 坐标与单位

```text
BackendSimulationSpace（mm）
    ↓ NavigationTransform
VesselLocalSpace
    ↓ VesselRoot Transform
GodotWorldSpace（m）
    ↓ XROrigin3D
OpenXR Reference Space
```

统一使用 `world_m = medical_mm × 0.001`，由 `CoordinateAdapter` 集中实现。工作区首选 `LOCAL_FLOOR`，不支持时降级为 `LOCAL`；`VIEW` 只用于短暂提示，不承载主工作台和急停。

### 4.5 空间构图、影像链路与本地状态

第一版以设计参考图的空间构图、信息层级和视觉语言为基准，不要求逐像素复刻。中央模型占主视野约 45%–55%，建议距离 0.95–1.25m、低于视线 5°–12°；左右面板建议距离 0.95–1.20m、水平偏转 25°–35°、宽 0.48–0.58m；控制坞建议距离 0.80–1.05m、低于视线 22°–32°、宽 0.68–0.85m。中央模型相对参考图缩小约 15%–25%，避免遮挡左右信息。

```text
环境背景  #030911 / #06101A     面板主体  #0B1521 ～ #101D2B
次级面板  #0D1A27               低亮边框  #2D5E91
主文字    #E2EDF7               次/弱文字 #A8B8C8 / #73869A
强调蓝    #4FA6FF               路径白蓝  #D7F2FF / #7EC8FF
安全绿    #65E56F               警告黄    #FFD65A
危险红    #FF4B4B               未知灰    #8B98A5
```

```text
Vessel 3D
    └─ EndoscopeCamera3D（侧后偏置跟随）
         └─ SubViewport（仅一个动态实例）
              └─ ViewportTexture
                   └─ ImagingPanel / EndoscopeViewportView
```

本地 UI 状态至少包括：`imaging_layout_mode`、`focused_imaging_view`、`dsa_source_mode`、`endoscope_camera_valid`、`endoscope_render_status`、`endoscope_resolution_tier`、控制模式、控制来源、门控状态和阻断原因。它们只能描述本地呈现与控制链路，不得覆盖后端权威安全状态。

---

## 5. 实施策略与总排期

### 5.1 实施顺序

开发顺序严格遵循“基线冻结—平台链路—生命周期安全—共享核心—四区视觉骨架—双影像链路—三维导航—控制闭环—性能发布”。后续阶段可以准备资产或测试夹具，但不得绕过前一阶段门禁提前启用非零控制。

M1–M8 分别对应设计方案快速交付路线 A–H；M0 是在其前增加的仓库、设备、契约与视觉冻结门禁。

### 5.2 里程碑总览

估算假设：1 名 Godot/XR 主开发、1 名后端/测试协作人员、至少 1 台持续可用的开发机和 1 台验收机。人日为规划量级，不替代团队拆分后的任务估算。

| 里程碑 | 目标 | 估算 | 核心门禁 |
|---|---|---:|---|
| M0 | 现状核验、环境、契约和视觉冻结 | 3–4 人日 | 版本、协议、视觉令牌、构图和证据模板冻结 |
| M1 | Android OpenXR 真机冒烟 | 4–6 人日 | 连续 10 次启动，头显与双手柄稳定 |
| M2 | Session、Action 与空间安全基线 | 5–7 人日 | 所有 XR 失效路径保持中性输入 |
| M3 | 共享核心与一致快照 | 6–8 人日 | 桌面/XR 共用状态、安全、控制与坐标 |
| M4 | 四区视觉骨架与空间交互 | 5–7 人日 | 构图、层级、光标与常驻安全入口真机通过 |
| M5 | DSA 占位与动态腔镜 | 6–9 人日 | 双影像三布局、动态视图与真实性边界通过 |
| M6 | 三维导航、选取与观察模式 | 5–7 人日 | 权威导航、命中语义和四种相机模式通过 |
| M7 | 连续控制、急停与状态语义 | 7–10 人日 | 同周期归零、急停闭环和状态来源显示通过 |
| M8 | 性能、构建、部署与发布 | 7–10 人日 | 单动态视口预算、30 分钟稳定性和发布证据通过 |

基准投入为 **48–68 人日**，建议日历排期 **9–12 周**。真机兼容、动态腔镜调校、热稳定性和多人舒适性观察不可用纯编码工时替代。

### 5.3 里程碑状态规则

- `未开始`：前置门禁未满足；
- `进行中`：已有可复现任务和责任人；
- `有条件通过`：非 P0/P1 缺陷已记录、范围和回退明确；
- `通过`：交付物、测试证据和退出条件全部满足；
- `阻断`：存在 P0/P1、真机不可用或关键证据缺失。

---

## 6. 分阶段开发计划

## M0：现状核验、环境、契约与视觉冻结

### 目标

把“设计假设”和参考图转换为当前仓库、设备、后端及真机显示的可验证基线，避免按过期目录、协议或主观视觉判断开发。

### 前置条件

- 可访问当前仓库和既有桌面运行环境；
- 至少一台 PICO 4 Ultra Enterprise 可通过 ADB 连接；
- 前端、后端和测试负责人参与契约确认。

### 开发任务

- 盘点当前 `godot_client` 场景、渲染器、WebSocket、急停和测试，记录“已有/部分/缺失”，不直接相信历史规划；
- 固定 Godot 4.7.1、OpenJDK 17、Android SDK/Build Tools、Build Template、Vendors Plugin、PICO OS 和 Runtime 版本；
- 新建 `docs/xr/version-matrix.md`，记录 APK、扩展和 Interaction Profile；
- 运行桌面端自动化与人工主流程，保存回归基线；
- 冻结 `navigation_visual_v3` 字段、后端安全权威、Session 过滤和时钟语义；
- 与后端确认控制消息是否需要 `control_source`、`input_sequence`、`client_timestamp_ms`、`deadman_active`、`push`、`rotate`；
- 冻结 Fresh/Delayed/Stale 阈值和 Delayed 的限速或阻断策略；
- 冻结控制限幅、死区、发送频率、中性帧和后端控制超时；
- 建立真机日志、功能、安全、性能、热状态和缺陷证据模板；
- 完成旧模块名到规范名的迁移映射；
- 冻结四区构图、面板距离/角度/尺寸、模型缩放、信息优先级和安全/急停不可遮挡规则；
- 冻结深色青蓝色板、8–14px 视觉等效圆角、边框、字号、Noto Sans SC、等宽数字、线性图标和禁用 emoji 规则；
- 建立设计参考图对照页，明确“构图与视觉语言基准，不做逐像素复刻”；
- 冻结 DSA 占位文案、禁用工具状态、腔镜相机偏置/FOV 初值、SubViewport 分辨率和刷新率试验范围。

### 验证与证据

- `tests/test_frontend_contract.py` 和相关协议测试结果；
- Godot headless 导入/脚本检查；
- 桌面 WebSocket、状态流、急停、恢复人工记录；
- ADB 设备、系统、Runtime 和构建工具版本输出；
- 已评审的状态/控制契约与阈值表。
- 真机视觉基准截图、空间尺寸记录和设计差距清单。

### 交付物

- 版本矩阵；
- 当前能力与差距清单；
- 状态/控制契约；
- 桌面回归基线；
- 测试证据模板；
- 命名迁移表。
- 视觉令牌、空间构图参数表、影像真实性边界与参考图对照页。

### 退出门禁

设备和构建环境可复现，桌面基线通过，协议/阈值/时钟/命名、视觉令牌、空间构图和影像真实性边界均有唯一结论。未完成 M0 不进入真机业务开发。

---

## M1：Android OpenXR 真机冒烟

### 目标

尽早证明 Godot 4.7.1、Android Gradle、Vendors Plugin 与 PICO Runtime 的最小链路兼容。

### 开发任务

- 建立 Android Debug 导出预设、Gradle Build Template、arm64、包名和必要权限；
- 固定并纳入匹配版本的 Vendors Plugin；
- 创建最小 `MainXR.tscn`、`XRBootstrap`、`XROrigin3D`、`XRCamera3D` 和左右 `XRController3D`；
- 记录 OpenXR 初始化、扩展、Reference Space、Session 和 Interaction Profile；
- 显示轻量测试模型、头显/双手柄追踪状态、刷新率与帧时间；
- 覆盖冷启动、系统菜单、摘戴头显、暂停、恢复、退出和重入；
- 本阶段业务控制保持禁用。

### 验证矩阵

| 场景 | 预期证据 |
|---|---|
| 连续冷启动 10 次 | 全部进入 OpenXR，无初始化失败 |
| 左右手柄连接/重连 | Aim 与 Grip Pose 持续更新 |
| 系统菜单覆盖 | Session 状态变化可观测，无业务控制 |
| 摘下并戴回头显 | 生命周期正确，输入保持中性 |
| 正常退出/重入 | 无后台控制，Session 可重建 |

### 交付物

Debug APK、最小 XR 场景、Bootstrap 日志、真机冒烟报告、兼容性问题清单。

### 退出门禁

PICO 连续 10 次启动成功，头部和双手柄追踪稳定，Session 事件可观测，恢复后保持中性输入。

---

## M2：Session、Action 与 Reference Space 安全基线

### 目标

在接入非零业务控制前，建立符合 OpenXR 语义的输入和生命周期安全层。

### 开发任务

- 实现 `XRSessionStateBridge`，覆盖 IDLE、READY、SYNCHRONIZED、VISIBLE、FOCUSED、STOPPING、LOSS_PENDING、EXITING；
- 创建完整 Action Set 和 `openxr_action_map.tres`，至少包含 Deadman、推进、旋转、急停、中性化、恢复控制、UI 选择/返回、`toggle_imaging_layout`、`focus_dsa`、`focus_endoscope`、面板切换、工作区复位、Aim/Grip Pose 和 Haptic；
- Action Set 在附加到 Session 前一次性创建，不在运行期动态增补；
- 实现 `XRInputAdapter`，每 XR 帧经 `xrSyncActions` 生成单一不可变 `XRInputSnapshot`；
- Aim Pose 只用于 UI 指向，Grip Pose 只用于手柄握持显示；
- 实现 `XRSafetyGate` 的本地中性化，不接业务 WebSocket 非零发送；
- 处理 Action inactive、单/双手追踪丢失、Session 非 FOCUSED、Reference Space 变化和应用暂停；
- 工作区使用 `LOCAL_FLOOR`，不支持时降级 `LOCAL`；
- Reference Space 变化时归零、重算工作台、检查可见性并要求重新按 Deadman；
- 触觉仅作短促辅助提示，失焦或 Action inactive 时停止。

### 验证与证据

- Session 状态转换表逐项真机录像/日志；
- FOCUSED→VISIBLE→FOCUSED、系统弹窗和摘戴测试；
- Action inactive、追踪丢失和 Reference Space Change 故障注入；
- Aim/Grip 方向和 Interaction Profile 对照；
- 证明所有失效场景输出均为中性，且恢复后不重放旧值。

### 交付物

Session 桥、Action Map、输入快照、安全门控骨架、Reference Space 管理、生命周期安全报告。

### 退出门禁

只有 FOCUSED 可进入“待使能”状态；其他状态、Action inactive 和追踪失效均在同一客户端更新周期归零。不得以 VISIBLE 或画面仍在显示作为控制条件。

---

## M3：共享核心、网络与一致快照

### 目标

让桌面与 XR 读取同一份协议、状态、安全、控制和坐标实现，消除 UI 直接解析网络数据的分叉。

### 开发任务

- 实现/收敛 `NavigationStateStore`、`SafetyStateStore`、`DataFreshnessMonitor`、`EmergencyStopCoordinator`、`ControlRouter` 和 `CoordinateAdapter`；
- 复用 WebSocketClient 与 `navigation_visual_v3`，网络回调只更新线程安全存储；
- 生成 `NavigationSnapshot`、`SafetySnapshot`、`ConnectionSnapshot`、`XRInputSnapshot` 和 `ControlGateSnapshot`；`ConnectionSnapshot` 必须分别保存网络、后端会话、物理引擎、XR Session、数据流和控制门控状态；
- 只在主线程/XR 帧消费同一批次不可变快照；
- 使用单调时钟判断本地 Fresh/Delayed/Stale，保留服务端时间戳用于审计；
- 过滤旧 Session、乱序帧和重连前状态；
- 将后端时间戳、前端接收时间、Freshness 和状态来源随快照保留，供右侧面板与审计日志使用；
- 统一控制发送队列，急停优先于普通控制；
- 将桌面入口迁移到共享核心，保留桌面交互与布局；
- 禁止桌面/XR UI 直接解析原始 JSON，禁止输入适配器直接调用网络发送；
- 为现有业务类提供短期兼容适配，完成后删除重复状态机。

### 自动化与故障注入

- 完整字段、null/unknown/stale、空 `risk_regions`；
- 旧 Session、乱序、断线、慢初始化和重连；
- 快照批次一致性和网络线程约束；
- 坐标/单位转换及往返误差；
- Router 来源、序号、限幅、中性帧和优先级；
- 桌面契约回归和 XR 场景 headless 加载。

### 交付物

共享核心模块、双入口、快照模型、兼容迁移层、自动化测试和数据异常报告。

### 退出门禁

桌面与 XR 共用唯一状态、安全、控制和坐标实现；UI 不解析原始 JSON；同一帧不会组合不同批次数据；桌面功能不回归。

---

## M4：四区视觉骨架与空间交互

### 目标

先在真机建立与参考图一致的空间构图、视觉层级和基本交互语义，再接入复杂影像与连续控制，避免功能完成后返工整体布局。

### 开发任务

- 建立中央三维工作台、左侧影像、右侧安全、下方控制坞、左下菜单和右下更多入口的四区场景骨架；
- 按 M0 参数实现轻微弧面的哑光深色面板、细边框、青蓝主色、安全绿/黄/红/灰和层级化文字；
- 使用 Noto Sans SC 或项目冻结的等价中文字体，数值采用等宽字形，图标使用统一线性资源，不使用 emoji；
- 中央模型占主视野约 45%–55%，相对参考图缩小约 15%–25%，避免遮挡左右面板；
- 固定右侧安全与下方急停的最高可见优先级，折叠或弹出次级面板不得覆盖二者；
- 加载真实手柄模型并显示 Aim ray；静态手套仅为可选装饰，不把手部骨骼、手指追踪列为首版依赖；
- UI 命中显示圆形光标，血管/路径/目标命中显示三维表面环形光标；无有效命中时缩短或隐藏长射线；
- 为中央模型、UI 面板和空白区域设置分离碰撞层，避免 UI 点击误选三维对象；
- 实现主要按钮的悬停、按下、禁用、选中和焦点反馈，碰撞区为视觉区域的 1.15–1.35 倍；
- 菜单和更多入口只承载低频功能；其展开、收起和返回路径均不得改变控制状态。

### 验证与证据

- PICO 真机正视、左右转头和坐姿截图与参考图构图对照；
- 面板距离、偏转角、宽度、模型视野占比和急停可达性测量；
- UI、血管、路径、目标和无命中五类射线反馈录像；
- 中文字体、数值对齐、禁用态、折叠态及高对比状态检查；
- 次级面板展开、工作区复位和 Session 变化期间安全/急停不被遮挡；
- 初始帧时间、透明过绘和面板材质开销记录。

### 交付物

四区空间场景、视觉令牌资源、面板组件库、手柄/Aim 交互、双光标系统、菜单/更多入口和真机构图对照报告。

### 退出门禁

四区构图、信息层级和视觉语言达到 M0 冻结基准；安全状态与急停始终可见可达；不同命中对象反馈明确；无依赖完整手部追踪的首版阻塞项。

---

## M5：DSA 占位与动态腔镜

### 目标

交付符合真实性边界的左侧双影像区：DSA 明示占位，腔镜由本地三维场景实时渲染，默认同时可见且可切换主次布局。

### 开发任务

- 默认上下同时显示 DSA 与腔镜，初始高度比例约为 DSA 56%–60%、腔镜 40%–44%；
- 实现双画面、DSA 主视图约 80% + 腔镜画中画、腔镜主视图约 80% + DSA 画中画三种布局；
- 实现 `toggle_imaging_layout`、`focus_dsa` 和 `focus_endoscope` Action，非主画面不得完全消失；
- 增加 `imaging_layout_mode`、`focused_imaging_view`、`dsa_source_mode`、`endoscope_camera_valid`、`endoscope_render_status` 和 `endoscope_resolution_tier`；
- DSA 固定显示“DSA / 占位图 / Demo Image / 实时状态未接入”，使用静态纹理，不显示 `LIVE`、伪 FPS、伪网络延迟或假采集状态；
- DSA 工具仅按真实实现开放：Zoom/Pan/Reset 可选，Measure 禁用，Rotate 禁用或隐藏，Screenshot 未接能力时禁用，Window/Level 首版隐藏；
- 为未来 DSA Source Adapter 预留接口、错误态和空态，但不接入采集、解码、录制或医学测量；
- 实现 `Vessel 3D → EndoscopeCamera3D → SubViewport → ViewportTexture → ImagingPanel` 的单动态视口链路；
- 腔镜相机采用尖端侧后偏置：后方 8–15mm、上方 3–6mm、侧向 2–5mm，FOV 70°–90°，近裁剪面取兼顾稳定和不穿模的最小值；
- 相机看向前方路径采样点，限制 Roll，重规划时平滑切换；偏置需保证导丝可见但不长期占据画面中心，且相机不落到血管壁外；
- 腔镜视觉采用红棕/橙色腔壁、湿润高光、局部灯光、轻微暗角和可见导丝，不使用重型景深、体积雾或掩盖几何问题的后处理；
- DSA 与腔镜分别获得焦点边框；工具栏跟随焦点切换，腔镜首版仅开放已实现的 Zoom、Reset、Exposure 和返回双画面；
- 腔镜录制和截图未形成真实能力时保持禁用，不用 UI 动画或本地占位文件暗示已完成采集；
- 只显示本地可证实的腔镜渲染状态、分辨率档位和相机定位有效性，不伪造网络延迟。

### 验证与证据

- 三种影像布局逐项录像，证明切换时另一画面始终保留且安全/急停仍可见；
- 文案和状态扫描证明 DSA 无 `LIVE`、伪 FPS、伪延迟及误导性工具；
- 真机推进、旋转、急弯和重规划时腔镜连续性录像；
- 检查相机不出壁、无持续 Roll 翻转、导丝不持续遮挡中央 20%、前方分支可理解；
- 无路径、无导丝、相机无效和 SubViewport 初始化失败的真实降级测试；
- 双画面与两个主视图下记录 CPU/GPU 帧时间、显存和纹理更新时间。

### 交付物

双影像面板、三种布局、DSA 占位及未来接口边界、动态腔镜相机架、单 SubViewport、焦点工具栏、影像状态模型和真实性检查报告。

### 退出门禁

默认双画面与两种主视图真机可用；DSA 被明确识别为非实时占位；腔镜来自当前三维状态的动态渲染且相机约束稳定；无静态图、循环视频或伪状态冒充实时能力。

---

## M6：三维导航、选取与观察模式

### 目标

在四区骨架中显示与桌面一致的权威导航数据，建立清晰的临床桌面构图、对象选取和四种正式观察模式。

### 开发任务

- 接入 VR Balanced 血管、真实导丝、规划路径、目标点和后端权威状态；
- 通过 `CoordinateAdapter` 完成毫米到米和 Backend/Vessel/Godot/OpenXR 多坐标转换；
- 建立中央视觉层级：尖端、前向路线和目标最高，主血管与导丝次之，已走路径降低亮度，远端分支继续降亮；
- 尖端使用小型青白环，目标使用蓝色双环，前向路径使用白蓝高亮，已走路径变暗，导丝保持银蓝实体，血管保持低过绘玻璃质感；
- Aim 可选取血管、规划路径和目标点；选中只改变本地观察状态，不直接修改后端控制；
- 空值、过期、断线、无路径和空 `risk_regions` 按真实语义显示，不生成伪危险体积；
- 实现 Clinical Tabletop、Guidewire Follow、Endoscope Offset Follow 和 Tree Overview；
- Guidewire Follow 使用路径切线和最小旋转框架，限制 Roll，重规划 200–400ms 平滑过渡并支持立即退出；
- Endoscope Offset Follow 复用 M5 的侧后相机逻辑服务影像面板，不作为默认全视野沉浸模式；
- 断线、Stale、无路径或相机定位无效时停止自动相机运动并回退 Clinical Tabletop；
- 工作区复位只修改本地观察变换，复位期间控制归零，完成后要求重新按 Deadman。

### 验证与证据

- 桌面与 PICO 同连一会话的字段、时间戳、坐标和截图对照；
- 血管、导丝、路径、目标点及尖端标记的数值误差报告；
- 血管/路径/目标选择命中、取消、遮挡和 UI 竞争测试；
- 旧 Session、null、Stale、断线、重连、空风险和无路径故障注入；
- 四种观察模式在直段、急弯和重规划下的录像与 Roll/角速度记录；
- 工作区复位、Follow 退出和降级不改变后端状态、不重放旧控制。

### 交付物

权威三维导航工作台、视觉层级材质、对象选取、四种观察模式、坐标对照报告和相机舒适性记录。

### 退出门禁

PICO 与桌面显示同一权威状态；坐标无明显漂移；前向路线、尖端和目标一眼可辨；unknown/stale 不显示为安全；选择与观察不绕过控制路由；自动相机无突跳或明显 Roll 翻转。

---

## M7：连续控制、急停与状态语义

### 目标

实现默认安全、来源明确、可审计的连续控制闭环，并让右侧安全面板和下方控制坞准确表达后端状态、本地门控与实际发送值。

### 开发任务

- 冻结并实现第一版映射：左手 Grip/Deadman 持续使能、左摇杆 Y 推进、右摇杆 X 旋转、右手 Aim/Trigger 选择 UI、独立急停和 `resume_control`；
- 实现摇杆死区、低速曲线、限幅和发送限频；面板 +/- 与 L/R 仅作为预设或辅助调节，不作为连续主控制；
- 所有输入经过 `XRInputSnapshot → XRSafetyGate → ControlRouter → WebSocket`，任何 UI、手柄或快捷入口不得直连网络；
- 接入 FOCUSED、Action active、追踪、Deadman、WebSocket、数据新鲜度、后端急停和本地故障门控；
- 门控失败先本地归零并阻断，再尽力发送一次中性帧；连续发送异常进入本地故障锁定；
- 实现急停请求、本地锁定、后端锁存确认、锁存期间非零拒绝、显式 Resume 请求、后端恢复确认和重新按 Deadman；
- 右侧面板常驻显示：后端安全/风险、壁距、接触力、速度、剩余距离、路径进度、数据新鲜度、时间戳和 Fresh；
- 连接状态拆分显示网络、后端会话、物理引擎、XR Session、数据流和控制门控；禁止用单一 `CONNECTED` 代替；
- `SAFE` 仅显示后端 `data.safety`，并明确来源和更新时间；未知或 Stale 使用灰色且不得显示绿色；
- 下方控制坞显示控制模式、控制来源、推进/旋转、Deadman、门控状态、阻断原因和大尺寸急停；
- 控制坞显示通过门控后的实际发送值，不把原始摇杆值标成已执行；同时保留必要的用户意图调试信息；
- Deadman 文案统一为“持续使能 / 按住 / 左 Grip / 松开即停”，不可暗示它单独足以允许控制；
- 右侧面板按信息层级折叠，安全状态、关键数值、新鲜度和急停锁存不得折叠消失。

### 关键安全矩阵

| 场景 | 必须满足 |
|---|---|
| Deadman 未按/松开 | 不发送非零；松开同周期本地归零 |
| FOCUSED→VISIBLE | 同周期归零并锁定 |
| Action inactive/追踪丢失 | 同周期归零并要求重新使能 |
| Reference Space 变化 | 归零、复位工作区、重新按 Deadman |
| WebSocket 断开/数据 Stale | 立即阻断，不离线继续 |
| `stop_required=true` | 不等待 UI，立即阻断 |
| 急停请求/锁存 | 本地先锁定，前后端拒绝全部非零控制 |
| 急停期间断线/重连 | 保持安全意图，按协议重新确认锁存 |
| Resume 确认 | 仍为中性，重新按 Deadman 才可控制 |
| 控制发送连续异常 | 进入本地故障并锁定 |
| 状态来源缺失 | 显示 unknown/stale，不推导 SAFE 或 CONNECTED |

### 可用性与安全验证

- 坐姿完成连接确认、影像布局、观察模式、Deadman 控制、急停、恢复和复位；
- 用户无需大幅转头即可读取安全状态、阻断原因并触达急停；
- 使用日志关联原始输入、门控快照、实际发送帧和后端确认；
- 逐项验证网络/会话/引擎/XR/数据/控制状态独立变化时的显示；
- 至少 3 名测试者记录误触、状态理解、眼疲劳、眩晕和任务完成情况；
- 重新佩戴、复位、重连和恢复后均不自动恢复控制。

### 交付物

完整 Action Map、统一 SafetyGate/Router、急停协调器、右侧安全面板、运动控制坞、状态来源模型、前后端安全测试、审计日志和舒适性报告。

### 退出门禁

安全矩阵全部通过且无绕过统一门控的入口；安全、连接、控制来源和阻断原因无歧义；急停始终可达；任一残留非零控制、旧输入重放、急停失效或权威状态误导均为 P0。

---

## M8：性能、构建、部署与发布

### 目标

在动态腔镜和四区 UI 完整启用的条件下形成可重复构建、安装、配置、运行、审计和回退的发布包。

### 开发任务

- 建立 VR High、VR Balanced、VR Safe、Debug 资产与渲染档位，默认 VR Balanced；
- 优化血管透明过绘、Fresnel、Emission、远端分支 LOD、面板刷新和线性图标资源；
- 默认关闭重型 DOF、SSR、体积雾、昂贵模糊和透明阴影，限制实时灯光；
- 第一版只保留一个动态腔镜 SubViewport，DSA 使用静态纹理；
- 腔镜初始分辨率在 640×480 或 768×576 中按真机证据冻结，常规刷新 30–45Hz；腔镜放大时可升档，不可见、画中画过小或折叠时降分辨率/降频或暂停；
- 分辨率、刷新率和是否暂停由 `endoscope_resolution_tier` 与可见性策略管理，不在每帧反复分配纹理；
- 先冻结 Compatibility 基线，再在同设备同病例比较 Mobile Vulkan；未经证据不得切换默认渲染器；
- 以设备实际刷新率确定帧预算；90Hz 模式对应约 11.1ms，最终门槛在 M0 记录；
- 记录平均、P95/P99 帧时间、丢帧、CPU/GPU、内存、温度、热降频、网络高峰、透明近景和三种影像布局差异；
- 每次性能迭代运行 10 分钟，发布候选在默认布局、腔镜主视图和复杂病例下完成 30 分钟组合稳定性；
- 创建签名 Release 预设，外部配置后端地址、WSS、病例和运行参数，禁止写死开发机 IP；
- 验证证书、包名、升级、卸载、回退和可选 Kiosk/企业设备管理；
- 归档 APK、符号、配置、版本矩阵、日志、录像、视觉对照、测试报告和已知限制。

### 性能矩阵

| 场景 | 主要证据 |
|---|---|
| 默认病例 + VR Balanced + 双画面 | 目标刷新率、P95/P99、丢帧 |
| DSA 主视图 / 腔镜主视图 | SubViewport 分辨率、刷新率和切换尖峰 |
| 复杂病例 + VR Balanced | 透明过绘、瓶颈与是否触发降级 |
| 复杂病例 + VR Safe | 最低可接受稳定性 |
| 透明血管近景 + 动态腔镜 | GPU 帧时间、过绘、局部灯光 |
| WebSocket 更新高峰 | 主线程尖峰和快照消费 |
| 10/30 分钟连续运行 | 热降频、内存增长和持续掉帧 |
| 快速头动 / Follow / 重规划 | 跟踪稳定、相机连续与舒适性 |

### 最终交付物

- Debug APK 和签名 Release APK；
- 外部配置模板；
- 安装、升级、卸载和回退说明；
- 完整版本矩阵；
- 功能、安全、数据、视觉、双影像、性能/热稳定性、舒适性和桌面回归报告；
- PICO 真机关键状态日志、构图截图、动态腔镜录像和视觉差距闭环；
- 已知问题、限制、真实性边界和降级清单。

### 退出门禁

满足第 10 节最终完成定义；VR Balanced 达到 M0 冻结的真机基线，VR Safe 可在复杂病例稳定降级；单动态 SubViewport 在三种布局下不造成不可接受尖峰；30 分钟运行无持续掉帧、热降频或内存增长；无未关闭 P0/P1。

---
## 7. 持续质量门禁

### 7.1 每次合并请求最低要求

- 相关单元和协议测试通过；
- `tests/test_frontend_contract.py` 通过；
- Godot headless 导入与脚本校验通过；
- 共享核心变更同时验证桌面与 XR；
- 控制/安全变更新增至少一个失败路径测试；
- Android/XR 配置变更至少成功生成 Debug APK；
- 真机相关改动附设备、版本和证据索引；
- 四区构图、视觉令牌或影像布局变更附参考图对照与 PICO 截图；
- DSA 变更通过真实性文案/禁用工具扫描，腔镜变更附动态性、相机约束和 SubViewport 性能证据；
- 不提交密钥、生产证书或个人设备凭据；
- 设计、Action、协议或安全不变量改变时同步更新追踪矩阵。

### 7.2 自动化层级

1. 静态契约：场景、脚本、Action、模块依赖和禁止直连规则；
2. 共享核心单元测试：状态、Freshness、安全、急停、控制和坐标；
3. 后端协议测试：Session、乱序、序号、Deadman、急停、恢复和拒绝；
4. Godot headless：桌面/XR 场景加载、资源导入和脚本解析；
5. Android 构建：Debug/Release、权限、配置、签名和安装；
6. 视觉/资源契约：色板、字体、图标、布局状态、DSA 占位文案和禁用能力；
7. 动态影像：EndoscopeCamera3D、SubViewport、相机约束、重规划连续性和性能档位；
8. PICO 真机：Session、追踪、Action、Reference Space、构图、双影像、安全、性能和舒适性。

### 7.3 发布阻断级别

| 级别 | 定义 | 处理 |
|---|---|---|
| P0 | 非预期非零控制、急停失效、后端权威误导、数据伪造或无法启动 | 必须修复，禁止发布 |
| P1 | 核心任务不可完成、关键状态不可读、持续严重掉帧或桌面主流程回归 | 必须修复或正式裁剪范围 |
| P2 | 次级功能、局部视觉或可接受降级问题 | 记录责任人、版本和回退 |
| P3 | 不影响任务的优化建议 | 纳入后续迭代 |

有条件通过只适用于有明确降级、责任人和截止版本的 P2/P3，不适用于 P0/P1。

---

## 8. 端到端验收矩阵

| 领域 | 必测场景 | 通过标准 | 证据 |
|---|---|---|---|
| OpenXR 生命周期 | 冷启动、VISIBLE/FOCUSED、STOPPING、LOSS_PENDING、EXITING、重建 | 非 FOCUSED 无非零控制 | Runtime/Godot 日志、录像 |
| 输入 | Aim/Grip、Action inactive、摇杆回中、追踪丢失 | 快照一致，旧输入不重放 | 输入日志、故障注入结果 |
| Reference Space | LOCAL_FLOOR、LOCAL 降级、Space Change、复位 | 工作台正确重定位，控制归零 | 状态日志、截图 |
| 数据 | Fresh/Delayed/Stale、null、空风险、乱序、旧 Session、重连 | 真实语义显示，Stale 阻断 | 协议测试、桌面/VR 对照 |
| 控制 | Deadman、限幅、序号、来源、发送异常 | 全入口经统一门控 | 单元/集成/后端拒绝日志 |
| 急停 | 正常、延迟、断线、锁存、恢复 | 本地先锁定，后端确认，恢复不重放 | 状态机日志、录像 |
| 三维导航 | 血管、导丝、路径、目标、重规划 | 坐标一致、无伪风险、无明显漂移 | 数值对照、截图 |
| 视觉与构图 | 四区布局、模型占比、面板角度、字体、状态色 | 达到冻结基准，安全/急停不被遮挡 | 参数记录、参考图对照 |
| DSA | 占位文案、静态纹理、工具禁用、三布局 | 无实时误导，另一影像不消失 | UI 扫描、真机录像 |
| 动态腔镜 | 推进、旋转、急弯、重规划、异常降级 | 当前三维状态实时渲染，不出壁/翻滚/持续遮挡 | 录像、相机日志、帧时间 |
| 空间 UI | 安全面板、影像、控制坞、菜单/更多 | 安全/急停始终可见，中文可读 | 真机任务记录 |
| 舒适性 | Follow、快速头动、复位、重新佩戴 | 无突跳/明显 Roll 翻转，不自动控制 | 3 人记录、录像 |
| 性能 | 默认/复杂病例、三种影像布局、10/30 分钟 | 达到 M0 基线，单动态视口无不可接受降频/增长 | P95/P99、丢帧、温度、内存 |
| 发布 | 安装、升级、卸载、回退、配置 | 可复现且不含开发机硬编码 | APK、哈希、发布清单 |
| 桌面回归 | 主流程、状态、控制、急停、布局 | 既有功能与测试通过 | 自动化和人工报告 |

---

## 9. 设计追踪矩阵

| 设计主题 | 实施里程碑 | 主要验收 |
|---|---|---|
| 技术选择与 Native 边界 | M0、M1、M8 | 版本矩阵、扩展清单、APK |
| Session 生命周期 | M1、M2 | 状态转换与非 FOCUSED 归零 |
| Reference Space、坐标与单位 | M2、M3、M4 | Space Change、坐标误差、复位 |
| 帧循环与快照 | M2、M3、M8 | 快照一致性、网络高峰帧时间 |
| Action、Aim/Grip 与触觉 | M2、M4、M5、M7 | 输入、追踪失效、影像切换和 UI 射线 |
| 双入口与共享核心 | M3 | 模块依赖、桌面/XR 回归 |
| 四区构图与视觉语言 | M0、M4 | 参数量测、参考图对照、状态反馈 |
| DSA 占位与真实性边界 | M0、M5 | 文案/工具扫描、三种布局 |
| 动态腔镜与偏置相机 | M5、M6、M8 | 动态性、相机约束、SubViewport 预算 |
| 三维工作台与观察模式 | M4、M6 | 坐标、选取、构图、舒适性 |
| 控制、Deadman 与急停 | M7 | 关键安全矩阵 |
| 数据真实性与新鲜度 | M3、M5、M6、M7 | 异常数据、来源显示和重连测试 |
| 空间 UI 与人体工学 | M4、M5、M7 | 坐姿任务、可读性、急停可达 |
| 渲染与性能 | M4、M5、M8 | Balanced/Safe、P95/P99、热稳定 |
| Android 与企业部署 | M1、M8 | Debug/Release、安装和回退 |

任何设计条款若没有对应里程碑、测试和证据，不得仅以“已编码”标记完成。

---

## 10. 最终完成定义

发布负责人逐项确认以下清单：

- [ ] 版本矩阵完整且与构建产物一致；
- [ ] APK 可重复安装、启动、退出和重入；
- [ ] OpenXR Session、头部和双手柄追踪通过；
- [ ] Aim Pose UI、Grip Pose 显示和 Action Map 通过；
- [ ] 桌面与 XR 使用同一权威状态和安全核心；
- [ ] 血管、导丝、路径和目标坐标一致；
- [ ] 四区构图、深色青蓝视觉语言、字体、图标和双光标达到冻结基准；
- [ ] DSA 明示占位且无 LIVE、伪 FPS/延迟或未实现工具误导；
- [ ] 动态腔镜由单 SubViewport 实时渲染，相机不出壁、无持续遮挡或明显 Roll；
- [ ] 双画面、DSA 主视图和腔镜主视图均可用，另一画面和安全/急停不消失；
- [ ] Deadman 与全部失效路径同周期归零；
- [ ] 急停请求、锁存、拒绝、恢复确认和重新使能通过；
- [ ] Fresh/Delayed/Stale、空值、乱序、旧 Session 和重连通过；
- [ ] 安全面板和急停始终可读、可达；
- [ ] 网络、后端会话、物理引擎、XR Session、数据流和控制门控状态独立可读；
- [ ] 控制坞显示实际发送值、模式、来源、Deadman、门控状态和阻断原因；
- [ ] VR Balanced 与 VR Safe 性能证据齐备；
- [ ] 10 分钟迭代和 30 分钟发布稳定性通过；
- [ ] 多人舒适性记录完成；
- [ ] 桌面自动化与人工回归通过；
- [ ] Release APK、配置、部署、升级、回退和已知限制齐备；
- [ ] 不含伪实时、伪风险或未说明硬编码状态；
- [ ] 无未关闭 P0/P1 缺陷。

任何一项缺失时，只能报告“阶段完成”或“有条件通过”，不得报告 PICO VR 第一版正式交付完成。

---

## 11. 风险、降级与回退

| 风险 | 预警信号 | 首选应对 | 降级/回退 |
|---|---|---|---|
| Godot/Vendors/PICO 不兼容 | 初始化失败、手柄异常、崩溃 | M1 锁定版本、最小复现 | 标准 OpenXR 路径；核心问题不可解时评估 Unity |
| Session 处理不完整 | 系统菜单后残留控制 | M2 状态桥和故障注入 | 禁用非零控制，保留只读观察 |
| Reference Space 漂移 | 工作台偏移或过近 | Space Change 门控和复位 | LOCAL_FLOOR→LOCAL |
| 网络阻塞 XR | 帧尖峰、追踪抖动 | 线程安全快照、单一发送队列 | 降低 UI/网络可视化刷新 |
| 透明血管过绘 | GPU 超预算、热降频 | LOD、减少透明层、控制 Bloom | VR Balanced→VR Safe |
| DSA 占位被误认实时 | 出现 LIVE、伪延迟或可用假工具 | 固定来源标签、资源/文案契约测试 | 保留静态图并禁用全部未实现工具 |
| 腔镜相机出壁或遮挡 | 黑屏、穿模、导丝占满中央 | 路径约束、偏置收缩、近裁剪与重规划平滑 | 标记相机无效并回退 Clinical Tabletop |
| SubViewport 开销高 | 双画面或放大腔镜掉帧 | 单动态实例、640×480/768×576、30–45Hz 和可见性降频 | VR Safe 降档或暂停非主腔镜，不裁剪安全信息 |
| 空间 UI 不可读 | 误触、频繁转头 | 真机尺寸标定、扩大碰撞区 | 裁剪次级面板，不裁剪安全信息 |
| 共享核心引起桌面回归 | 契约或主流程失败 | 小步迁移、兼容适配层 | 回退当前里程碑，不复制第二套核心 |
| PICO 专属能力缺失 | Kiosk/管理不可用 | 能力检测和后置评估 | 不影响核心导航时禁用增强项 |
| 真机资源不足 | 测试排队、证据缺失 | 固定开发机与验收机 | 无真机不得宣称阶段通过 |

Unity 迁移仅在 Godot/PICO 核心兼容无法规避、正式转向大规模 MR、关键扩展仅有 Unity 稳定支持或 Godot 经优化仍无法达到基线时单独立项评估。

---

## 12. 团队职责与执行节奏

| 角色 | 主要职责 |
|---|---|
| Godot/XR 开发 | OpenXR、Action、场景、空间 UI、Android 导出 |
| 共享核心开发 | 状态、Freshness、安全、控制、坐标和桌面迁移 |
| 后端开发 | 协议、Session、控制校验、急停锁存、WSS |
| 3D/渲染 | 资产档位、透明材质、LOD 和性能 |
| 测试/安全负责人 | 故障注入、真机矩阵、证据和发布阻断 |
| 产品/医学顾问 | 信息优先级、可读性和仿真/临床边界 |

小团队可一人兼任多角色，但控制/急停的最终安全验收不得只由功能实现者单独确认。

建议节奏：

1. 每个里程碑开始前确认前置条件、责任人和证据路径；
2. 每日构建保持桌面 headless/契约测试可运行；
3. 每周至少一次 PICO 真机回归，不把真机测试集中到发布周；
4. 每个里程碑产出可安装 APK、变更说明和测试记录；
5. P0/P1 立即阻断后续阶段，P2/P3 必须登记版本与回退；
6. M0/M1 作为第一轮迭代，建议 1–2 周，只证明环境和 XR 链路，业务控制保持禁用；
7. M2 通过前不得发送非零业务控制，M5 通过前不得宣称影像首版完成，M7 通过前不得对外宣称控制闭环完成；
8. M4–M6 每周保留同视角参考图对照，视觉差距必须登记为参数、缺陷或明确偏差，不能只写“观感可接受”。

---

## 13. 变更控制

以下变化必须先更新设计方案和本规划，再实施：

- 技术栈、Godot/Vendors/OpenXR 主版本变化；
- Session 控制条件、安全权威或急停语义变化；
- `navigation_visual_v3` 或控制协议不兼容变更；
- Reference Space、坐标系或单位变化；
- 四区构图、视觉令牌、影像布局或相机模式变化；
- DSA 来源模式、占位边界或腔镜 SubViewport/相机约束变化；
- 第一版范围新增 MR、手势控制、真实机器人或 Native SDK 直连；
- 性能基线、默认渲染器或设备刷新率变化；
- 发布门禁、P0/P1 定义或验收证据变化。

变更记录至少包含原因、影响里程碑、迁移方案、测试增量、降级/回退和批准人。未进入追踪矩阵的新增能力不属于第一版承诺。

---

## 14. 开发启动执行包

本节把 M0–M1 转成可直接领取的首轮任务。它只改变执行粒度，不改变第 2 节安全不变量、第 3 节范围或 M0–M8 门禁。

### 14.1 2026-08-03 开工判断

当前正式进入 **M0：现状核验、环境、契约与视觉冻结**。M1 允许准备最小场景和构建材料，但在 Android 工具链、Vendors Plugin、PICO Runtime 和真机证据齐备前不得标记通过；M2 以前不得发送 XR 非零业务控制。

已验证基线：

- 实际 Godot 引擎为 `4.7.1.stable.official.a13da4feb`；
- 现有桌面项目使用单一 `res://scenes/main.tscn`、Forward Plus 和 1920×1080 桌面布局；
- `pytest tests/test_frontend_contract.py -q` 为 `3 passed`；
- Godot 4.7.1 headless 导入和脚本解析退出码为 0；
- 桌面端已有 `navigation_visual_v3`、重连、急停/恢复、三维血管/导丝/路径、动态腔镜 SubViewport 和安全 HUD 原型。

当前阻断：

- `godot.cmd`/`godot4.cmd` 仍指向已删除的 4.6.3，环境入口不可复现；
- 仅发现 Java 8 JRE，未发现 JDK 17；
- 未发现 Android SDK、ADB、Gradle、Android Build Template、`export_presets.cfg`；
- `godot_client/addons`、Vendors Plugin、OpenXR Action Map、XR 场景和 XR 脚本均不存在；
- 尚无 PICO 设备、OS、Runtime、Interaction Profile、Reference Space 和连续 10 次启动证据；
- `project.godot` 仍硬编码开发机 WebSocket 地址；
- 当前主控制器 2554 行，同时承担桌面场景、UI、网络编排、控制和渲染，不可整体复制到 XR。

### 14.2 设计图到实施域映射

| 设计图域 | 当前可复用 | 首要缺口 | 实施里程碑 | 首轮验收 |
|---|---|---|---|---|
| 中央半透明血管、导丝和高亮路径 | 现有血管/导丝/路径渲染器、相机模式和模型资产 | 毫米到米适配、XR 空间锚定、VR 材质/LOD、双光标选取 | M0、M4、M6、M8 | 中央占视野 45%–55%，路径/tip/目标层级清楚，无明显漂移 |
| 左侧 DSA + 腔镜 | DSA 面板原型、单动态腔镜 SubViewport、相机滤波和质量档 | 上下双画面三布局、Placeholder 真实性文案、XR 面板、真机开销 | M0、M4、M5、M8 | DSA 无 LIVE/伪延迟；腔镜动态、无明显出壁/Roll，另一画面不消失 |
| 右侧导航与安全 | `data.safety` 展示、Fresh/Stale 降级、关键指标 | 网络/后端/引擎/XR/数据/门控拆分、空间可读性、趋势限频 | M0、M3、M4、M7 | 安全来源和时间戳明确；unknown/stale 不显示绿色 |
| 下方运动控制坞 | 桌面运动控件、急停请求/确认/恢复原型 | XR Action、统一快照、SafetyGate、实际发送值、阻断原因 | M2、M3、M4、M7 | 全入口经统一门控；急停始终可达；所有失效路径同周期归零 |
| 双手柄、右手 Aim 射线 | 无 XR 实现 | Vendors Plugin、Action Map、Aim/Grip、碰撞层和指针反馈 | M1、M2、M4 | 双手柄稳定；UI/三维/无命中反馈区分；不依赖完整手部追踪 |
| 深黑蓝空间环境与玻璃面板 | 桌面 `UiStyle` 与部分渲染材质 | VR 视觉令牌、中文字体、弧形面板、Compatibility 性能基线 | M0、M4、M8 | 真机对照图通过；安全色克制；无高成本背景模糊和装饰性过曝 |

### 14.3 Sprint 0 目标与边界

Sprint 0 计划为 **10 个工作日**，目标是“M0 全部门禁通过，并使 M1 达到可开始真机冒烟的 Ready 状态”。没有 PICO 真机时，可以完成环境、契约、桌面基线、XR 零控制骨架和 Debug APK 构建，但不能把 M1 标记为通过。

本 Sprint 明确不做：

- 不接 XR 非零推进或旋转；
- 不开始高保真四区 UI；
- 不把桌面 CanvasLayer 搬入头显；
- 不重写 WebSocket 或复制第二套状态机；
- 不接真实 DSA、手部追踪、MR、RL 管理或 Native SDK 专属能力；
- 不以桌面截图替代 PICO 真机构图或性能证据。

### 14.4 Sprint 0 可领取任务

| ID | 任务 | 主责 | 依赖 | 产物 | 完成标准 |
|---|---|---|---|---|---|
| S0-01 | 修复并冻结 Godot 4.7.1 命令入口 | Godot/XR | 无 | 环境脚本或团队安装说明 | `godot --version` 稳定返回 a13da4feb，headless 命令不依赖个人临时路径 |
| S0-02 | 建立版本矩阵 | Godot/XR | S0-01、设备信息 | `docs/xr/version-matrix.md` | Godot、JDK、SDK、Build Tools、模板、Vendors、PICO OS/Runtime、扩展和渲染器均有版本/来源/状态 |
| S0-03 | 安装并验证 JDK 17、Android SDK、ADB 和 Build Template | Godot/XR | S0-02 | 环境核验记录 | `java`/`javac`/`adb`/Godot Android export 均可复现；不接受 Java 8 JRE 代替 |
| S0-04 | 当前能力与差距审计 | Godot/XR、共享核心 | 无 | `docs/xr/current-capability-audit.md` | 对现有场景、脚本、协议、急停、腔镜、测试逐项标记“复用/适配/拆分/缺失”并附文件路径 |
| S0-05 | 冻结状态、控制、时钟与 Freshness 契约 | 共享核心、后端、测试 | S0-04 | `docs/xr/control-data-contract.md` | 明确 source、sequence、timestamp、Deadman、限幅、频率、中性帧、Delayed/Stale 策略和后端超时 |
| S0-06 | 保存桌面回归基线 | 测试 | S0-04 | `artifacts/xr/m0/desktop-baseline/` 索引 | 前端契约、Godot headless、真实 WebSocket、急停/恢复和关键截图可复现 |
| S0-07 | 冻结设计图视觉与空间参数 | 3D/渲染、产品、XR | 用户设计图 | `docs/xr/visual-baseline.md` | 五个设计域、色板、字体、面板距离/角度/尺寸、模型占比、急停不可遮挡和允许偏差均有唯一值 |
| S0-08 | 冻结 DSA/腔镜真实性边界 | XR、测试 | S0-07 | `docs/xr/imaging-boundary.md` | DSA 占位文案/禁用工具、腔镜偏置/FOV/分辨率/刷新试验范围和无效态明确 |
| S0-09 | 建立最小 XR 零控制场景 | Godot/XR | S0-01、S0-03 | `godot_client/scenes/xr/MainXR.tscn` 与 Bootstrap 骨架 | 仅头部/双手柄/状态模型；没有业务非零控制发送路径；桌面入口不回归 |
| S0-10 | 接入匹配 Vendors Plugin 与 Action Map 骨架 | Godot/XR | S0-02、S0-09 | `godot_client/addons/`、Action Map | 插件版本锁定；Action 名称和 Interaction Profile 可枚举；控制 Action 默认阻断 |
| S0-11 | 建立 Android Debug 导出 | Godot/XR | S0-03、S0-09、S0-10 | `export_presets.cfg`、Debug APK | arm64/OpenXR/Gradle 配置可重复构建，后端地址不写死为开发机 IP |
| S0-12 | PICO 最小真机冒烟准备与执行 | Godot/XR、测试 | S0-11、PICO 设备 | `artifacts/xr/m1/smoke/` 索引 | 记录安装、启动、Session、Aim/Grip、Reference Space、系统菜单和摘戴；连续 10 次全通过才关闭 M1 |
| S0-13 | Sprint 门禁复核与下一阶段拆分 | 全体 | S0-01～S0-12 | 评审记录、M1/M2 Backlog | M0 无未决唯一结论；未满足项有责任人、截止日和阻断级别 |

### 14.5 建议的 10 日顺序

| 工作日 | 主线 | 当日可验证结果 |
|---|---|---|
| D1 | S0-01、S0-02、S0-04 启动 | Godot 入口可复现，版本矩阵和能力审计有首版 |
| D2 | S0-03、S0-05 | Android/JDK 安装路径明确，状态/控制契约完成第一次评审 |
| D3 | S0-06、S0-07 | 桌面基线归档，设计图五域和空间参数冻结 |
| D4 | S0-08、S0-09 | 影像边界冻结，最小 XR 场景可 headless 加载 |
| D5 | S0-10 | Vendors/Action Map 锁定，全部业务控制保持阻断 |
| D6 | S0-11 | 首个可安装 Debug APK 和构建日志 |
| D7 | S0-12 冷启动/退出 | PICO Session、头显、双手柄与日志首轮证据 |
| D8 | S0-12 生命周期 | 系统菜单、摘戴、暂停、恢复、重入保持中性 |
| D9 | 连续 10 次启动与缺陷修复 | M1 冒烟矩阵完成或形成明确阻断 |
| D10 | S0-13 | M0 门禁评审；M1/M2 下一 Sprint 任务与责任人确定 |

如 D6 前仍没有可用 PICO，D7–D9 改为完成 XR 依赖契约测试、Mock Session/Action 故障注入夹具和真机执行手册；M1 状态保持“准备中”，不得用模拟器替代真机通过。

### 14.6 仓库迁移原则

| 当前对象 | Sprint 0 处理 | 后续目标 |
|---|---|---|
| `scenes/main.tscn` | 保持桌面主入口，不复制 | M3 命名/迁移为桌面入口时保留兼容 |
| `scripts/main_controller.gd` | 只做能力审计和依赖图，不大拆 | M3 逐步抽出状态、控制、坐标和渲染适配 |
| `scripts/websocket_client.gd` | 冻结现有协议行为，补契约测试 | M3 作为共享网络适配，UI 不再解析原始 JSON |
| `scripts/hud_controller.gd` | 保持桌面回归 | M3/M7 共享状态模型，XR 使用独立空间呈现 |
| 腔镜脚本与 shader | 记录可复用 API 和性能参数 | M5 复用算法，XR 使用单独场景/视口宿主 |
| `project.godot` | 不直接把桌面主场景替换为 XR；先冻结双入口启动策略 | M1 由 Android XR 导出进入 `MainXR`，桌面仍可独立运行 |
| 硬编码 WebSocket 地址 | 在 M0 定义配置优先级和 Debug 默认值 | M3/M8 使用外部配置，Release 不含开发机地址 |

共享核心采用“小步抽取 + 兼容适配”策略：先为现有类补行为测试，再抽取一个职责，再让桌面和 XR 同时消费，最后删除重复状态。不得先创建一整套 XR 私有业务核心后再尝试合并。

### 14.7 Sprint 0 完成定义

- [ ] S0-01～S0-13 有责任人、状态、证据路径和阻断级别；
- [ ] Godot 4.7.1、JDK 17、Android SDK/Build Tools、模板和 Vendors Plugin 版本锁定；
- [ ] 当前能力审计、状态/控制契约、桌面基线、视觉基线和影像边界完成评审；
- [ ] 设计图五个域均映射到模块、里程碑、验收和证据；
- [ ] 最小 XR 场景不存在业务非零控制旁路；
- [ ] Android Debug APK 可重复构建，配置中无未说明的开发机硬编码；
- [ ] 有 PICO 时完成 M1 真机矩阵；无 PICO 时明确保持 M1 未通过；
- [ ] 桌面前端契约与 Godot headless 基线不回归；
- [ ] 未关闭 P0/P1 为 0，或当前 Sprint 明确判定阻断并停止进入 M2。

Sprint 0 评审通过后，下一 Sprint 只推进 M1 遗留和 M2 Session/Action/Reference Space 安全基线；四区高保真视觉可准备资产，但不得绕过 M2 提前启用连续控制。

---

## 15. 下一步开发执行计划（2026-08-03 已核验版）

本节基于设计图、设计方案 v3.0、开发规划 v3.1 和当前仓库核验结果，确定“从现在开始”的实际开发顺序。结论是：当前进入 M0/Sprint 0，不直接开发高保真空间 UI，不启用 XR 非零控制，先把环境、契约、能力审计和零控制 XR 骨架做稳。

### 15.1 当前开工结论

| 项目 | 当前核验结果 | 对下一步的影响 |
|---|---|---|
| 桌面入口 | `godot_client/project.godot` 仍以 `res://scenes/main.tscn` 为主场景 | 桌面入口保留；XR 必须新增独立入口，不替换主场景 |
| XR 实现 | `godot_client` 内未发现 XR 场景、Action Map、Vendors Plugin 或 `addons` | M1 不是收尾，而是从最小 `MainXR.tscn` 开始 |
| Godot 版本 | 4.7.1 console 可执行文件存在并返回 `4.7.1.stable.official.a13da4feb` | 可作为 M0 冻结版本 |
| Godot 命令入口 | `godot.cmd` / `godot4.cmd` 仍指向 4.6.3 | S0-01 必须优先修复或固化团队命令 |
| Android 工具链 | PATH 中仅 Java 8；未发现 `javac`、`adb`、`gradle` | S0-03 阻断 Android Debug APK |
| Vendors Plugin | 未发现 `godot_client/addons` | S0-10 阻断真机 OpenXR Action 骨架 |
| 当前回归 | `pytest tests/test_frontend_contract.py -q` 通过，3 passed | 桌面契约可作为 M0 基线 |
| Godot 解析 | 精确 4.7.1 headless editor 退出码 0 | 项目脚本解析可作为 M0 基线；需记录用户级设置写入警告 |
| 设计图语义 | 五域清楚：中央导航、左双影像、右安全、底部控制、双手柄/Aim | Sprint 0 先冻结参数与证据模板，再建空间实现 |

### 15.2 下一步优先级

1. **环境先行**：修复 Godot 4.7.1 命令入口，建立版本矩阵，补齐 JDK 17、Android SDK、ADB、Gradle/Build Template。
2. **能力审计并冻结契约**：对现有桌面三维、腔镜、WebSocket、安全、急停和测试做“复用/适配/拆分/缺失”标记。
3. **冻结视觉与影像边界**：把设计图转成唯一空间参数、色板、字号、面板距离/角度/尺寸、DSA 占位文案和腔镜 SubViewport 预算。
4. **建立零控制 XR 骨架**：新增最小 XR 场景和 Bootstrap，但所有业务推进/旋转保持中性，不连接非零控制发送。
5. **再进入真机冒烟**：只有 Debug APK、Vendors Plugin、Action Map、ADB/PICO 证据齐备后，才执行 M1 连续 10 次启动矩阵。

### 15.3 首 72 小时可执行任务

| 时间窗 | 任务 | 产物 | 完成标准 |
|---|---|---|---|
| 0–4 小时 | S0-01：修复或冻结 Godot 4.7.1 命令入口 | 团队命令说明或脚本修复记录 | `godot --version` 或约定命令稳定返回 4.7.1 a13da4feb |
| 0–4 小时 | S0-04 启动：当前能力与差距审计 | `docs/xr/current-capability-audit.md` 首版 | 每个现有前端关键对象都有复用/适配/拆分/缺失判断和文件路径 |
| 4–8 小时 | S0-02：版本矩阵首版 | `docs/xr/version-matrix.md` | Godot、Java/JDK、Android SDK、ADB、Gradle、模板、Vendors、PICO 项均列出状态 |
| 第 1 天 | S0-05：状态、控制、时钟与 Freshness 契约 | `docs/xr/control-data-contract.md` | Deadman、限幅、频率、中性帧、Delayed/Stale、序号、时间戳和后端超时有唯一口径 |
| 第 1–2 天 | S0-07：设计图视觉与空间参数冻结 | `docs/xr/visual-baseline.md` | 五个设计域、色板、尺寸、距离、偏转角、模型占比和急停不可遮挡规则明确 |
| 第 2 天 | S0-03：Android/JDK/ADB 环境补齐 | 环境核验记录 | JDK 17、`javac`、ADB、Android SDK/Build Tools 可复现 |
| 第 2–3 天 | S0-06：桌面基线归档 | `artifacts/xr/m0/desktop-baseline/` 索引 | 前端契约、Godot headless、关键截图/日志路径齐备 |
| 第 3 天 | S0-08：DSA/腔镜真实性边界 | `docs/xr/imaging-boundary.md` | DSA 占位禁用项、腔镜 FOV/偏置/分辨率/刷新率和无效态明确 |

### 15.4 可以提前准备但不得关闭的任务

- S0-09 可以先创建 `godot_client/scenes/xr/MainXR.tscn` 骨架，但不得产生业务非零控制发送路径；
- S0-10 可以研究 Vendors Plugin 版本和 Action 名称，但未锁定版本矩阵前不得把不确定插件混入主分支；
- S0-11 可以准备 `export_presets.cfg` 草案，但在 JDK 17、Android SDK、Build Template 和 Vendors Plugin 可验证前不得报告 Debug APK 完成；
- S0-12 可以准备真机测试表，但没有 PICO 真机、ADB 安装日志和 Runtime/Interaction Profile 证据时不得标记 M1 通过。

### 15.5 明确暂不进入

- 不把桌面 CanvasLayer 直接贴到 XR；
- 不复制 `main_controller.gd` 形成第二套业务核心；
- 不接 XR 推进/旋转非零控制；
- 不制作伪实时 DSA、伪风险体积、伪 CONNECTED 或伪 SAFE；
- 不以桌面截图、模拟器或 headless 结果替代 PICO 真机视觉/性能/生命周期证据；
- 不在 M2 通过前进入完整连续控制闭环开发。
