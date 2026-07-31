# PICO 4 Ultra Enterprise VR 前端开发规划

> 项目：CathSim 医疗导管/导丝导航与强化学习仿真平台  
> 目标设备：PICO 4 Ultra Enterprise  
> 依据：`PICO4_Ultra_Enterprise_VR前端设计方案(1).md`  
> 技术基线：Godot 4.7.1 + GDScript + OpenXR + Android Gradle Export  
> 计划版本：V1.0  
> 编制日期：2026-07-31

---

## 1. 规划目标

本规划用于把现有 Windows 桌面 Godot 导航客户端演进为可在 PICO 4 Ultra Enterprise 上运行的 Android OpenXR 客户端，并保持桌面端可用。

第一版交付必须形成以下完整闭环：

1. PICO 真机可安装、启动、退出和重新进入；
2. OpenXR 头部 6DoF 与左右手柄追踪稳定；
3. 血管、导丝、规划路径、目标点与后端权威状态一致；
4. 手柄通过 Deadman 控制推进与旋转；
5. 松开 Deadman、追踪丢失、应用失焦、断线、数据过期或急停时同帧归零；
6. 软件急停完成请求、后端锁存、控制拒绝和恢复确认；
7. 空间工作台、安全面板、影像面板和控制坞满足坐姿使用；
8. VR Balanced 在默认病例中接近 90 Hz，并完成连续运行验证；
9. 产出可部署的签名 Release APK、配置说明和验收报告；
10. 桌面客户端回归测试持续通过。

本规划中的 VR 前端仅用于仿真、科研、人机交互和教学，不表述为真实临床设备控制系统；软件急停不替代物理急停。

---

## 2. 当前基线与差距

### 2.1 可继承能力

现有 `godot_client` 已具备：

- Godot 4.7 桌面主场景；
- 血管、导丝、规划路径、目标点和内镜渲染；
- `navigation_visual_v3` 消费逻辑；
- WebSocket 建连、重连、session 过滤与数据过期处理；
- 后端权威安全状态显示；
- 软件急停请求、锁存确认和 Resume 流程；
- 键盘、鼠标、按钮和自动导航的控制门控；
- 桌面前端契约测试。

### 2.2 当前阻塞差距

截至本规划编制时，工程中尚未形成：

- `MainXR.tscn` 与 Android APK 入口；
- OpenXR 初始化、头显和双手柄节点；
- OpenXR Action Map 与 `XRInputAdapter`；
- Android Gradle Build Template、Vendors 插件和导出预设；
- Deadman、失焦、追踪丢失等 XR 安全门控；
- 桌面/VR 共享的 ViewModel、Safety Store 和 Control Router；
- VR 空间工作台与常驻安全面板；
- VR High、Balanced、Lite 渲染档位；
- PICO 真机性能、热稳定性和舒适性报告；
- 签名 Release APK 和企业部署包。

因此开发必须从 Android/OpenXR 真机冒烟开始，不能直接进入空间界面美化。

---

## 3. 范围

### 3.1 第一版范围

- Android OpenXR 应用框架；
- PICO 头显与双手柄追踪；
- 桌面和 VR 双入口；
- 共享导航状态、安全状态和控制路由；
- 三维血管、导丝、路径与目标点；
- 手柄射线 UI；
- Deadman 推进与旋转；
- 软件急停和恢复闭环；
- 数据过期、断线、失焦、追踪丢失的同帧归零；
- Clinical Tabletop、Guidewire Navigation Follow、Endoscope Panel、Tree Overview；
- VR Balanced 和 VR Lite；
- Debug APK、签名 Release APK；
- 真机功能、安全、性能、舒适性和回归测试。

### 3.2 明确不纳入第一版

- 手势连续控制导丝；
- MR 透视、空间锚点和场景网格；
- 多人协作与语音控制；
- VR 内启动或配置 RL 训练；
- 真实机器人控制；
- 完整复制桌面 11 面板；
- 全沉浸内窥镜作为默认控制模式；
- 伪实时 DSA、RL 指标或风险区域；
- WebXR 主控制客户端；
- 大规模 PICO 专有接口集成。

---

## 4. 实施原则

1. **先真机后重构**：先证明 PICO、OpenXR、Android 导出和控制器可用，再开展共享核心重构。
2. **安全链路优先**：所有非零控制必须经过统一 `ControlIntentRouter`；UI 和 XR 输入不得直接发送控制消息。
3. **后端状态权威**：安全等级、急停锁存和控制确认以服务端为准；前端仅做更保守的本地阻断。
4. **共享核心、独立布局**：桌面和 VR 共享数据、状态、安全和三维渲染核心，但不共享顶层 UI 布局。
5. **每阶段可退出**：每个里程碑必须有真机或自动化退出条件，未通过不得把问题推迟到发布阶段。
6. **性能持续预算**：从第一个真机场景开始采集 CPU/GPU 帧时间，不在功能完成后才处理性能。
7. **不伪造数据**：冻结功能只能显示为只读、未知或不可用，不得包装为实时能力。

---

## 5. 目标工程结构

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
├── addons/godotopenxrvendors/
├── android/
├── openxr_action_map.tres
└── export_presets.cfg
```

入口选择应通过构建配置或明确的启动场景完成，禁止在运行时把桌面 UI 强行转换成 VR UI。

---

## 6. 里程碑总览

以下周期以 1 名熟悉 Godot 的主开发、1 名后端/测试协作人员和可持续使用 PICO 真机为估算基础。多人并行时可以压缩日历时间，但不能删除真机观察窗口。

| 里程碑 | 目标 | 估算 | 关键退出条件 |
|---|---|---:|---|
| M0 | 环境、契约与基线冻结 | 2–3 人日 | 桌面回归、设备和构建环境清单完成 |
| M1 | Android OpenXR 真机冒烟 | 4–6 人日 | PICO 稳定进入 OpenXR，会追踪头显和双手柄 |
| M2 | 最小导航业务上屏 | 4–6 人日 | PICO 与桌面显示同一权威导航状态 |
| M3 | 共享核心重构 | 6–8 人日 | 双入口共用状态与安全门控，UI 不解析原始 JSON |
| M4 | 手柄控制与安全闭环 | 7–10 人日 | 所有失效路径同帧归零，急停/恢复闭环通过 |
| M5 | 最小空间工作台 | 7–10 人日 | 坐姿下完成核心观察和控制任务 |
| M6 | 观察模式与性能优化 | 7–10 人日 | VR Balanced 接近 90 Hz，连续 10 分钟稳定 |
| M7 | 发布、部署和验收 | 4–6 人日 | 签名 APK、部署说明和完整验收报告交付 |

建议总投入：**41–59 人日**。建议排期为 **8–10 周**，其中真机兼容和性能测试不得仅按开发编码工时折算。

---

## 7. 分阶段开发计划

## M0：环境、契约与基线冻结

### 目标

建立可重复的桌面与 Android/XR 开发基线，明确不回归范围。

### 开发任务

- 固定 Godot 4.7.1，不使用失效的旧版本启动脚本；
- 安装 OpenJDK 17、Android SDK、ADB 和 Godot Android Build Template；
- 登记 PICO OS 版本、设备序列号、刷新率和开发者模式状态；
- 记录当前桌面端主流程截图和测试结果；
- 将 VR 所需控制字段加入前后端契约设计：
  - `control_source`
  - `input_sequence`
  - `client_timestamp_ms`
  - `deadman_active`
  - `push`
  - `rotate`
- 定义 `DataFreshnessMonitor` 的超时阈值和时钟来源；
- 建立 Android/XR 的日志采集、帧率采集和故障记录模板；
- 为桌面、XR、共享代码定义目录和命名规范。

### 测试

- 运行现有前端契约测试；
- 运行 Godot headless 导入与脚本校验；
- 启动桌面客户端并验证 WebSocket、状态流、急停和 Resume；
- 校验 Android SDK、JDK、ADB 和设备连接。

### 产物

- 环境基线表；
- 设备与软件版本清单；
- 控制消息契约；
- 桌面回归基线；
- 真机测试记录模板。

### 退出条件

- 所有开发机可以使用相同 Godot 版本；
- PICO 可被 ADB 识别；
- 桌面端测试通过；
- 控制字段和数据过期规则经前后端确认。

---

## M1：Android OpenXR 真机冒烟

### 目标

尽早证明 Godot 4.7.1、OpenXR 和 PICO 运行时兼容。

### 开发任务

- 安装并固定 Godot OpenXR Vendors Plugin/Khronos Loader；
- 创建 Android Gradle Build Template；
- 创建 Android Debug 导出预设；
- 配置 OpenXR 启动特性和必要权限；
- 新建 `MainXR.tscn`：
  - `XRBootstrap`
  - `XROrigin3D`
  - `XRCamera3D`
  - 左右 `XRController3D`
- 实现 `xr_bootstrap.gd`：
  - 启动时初始化 OpenXR；
  - 记录 session ready、visible、focused、stopping 状态；
  - 初始化失败时显示明确错误并禁止业务控制；
- 显示简单测试模型、头显/手柄追踪状态、刷新率和帧时间；
- 处理应用失焦、系统菜单、暂停、恢复和退出；
- USB 安装 Debug APK，并保存 ADB/Godot 日志。

### 测试矩阵

| 场景 | 预期 |
|---|---|
| 冷启动 | 稳定进入 OpenXR，不落入普通 3D 窗口 |
| 左右手柄连接 | 两个控制器位置和姿态持续更新 |
| 打开系统菜单 | 应用进入非 focused 状态 |
| 返回应用 | 恢复追踪，但不产生业务控制 |
| 摘下/戴上头显 | 生命周期事件正确，控制状态保持中性 |
| 正常退出 | 无持续后台控制或未释放会话 |

### 产物

- 可安装 Debug APK；
- `MainXR.tscn`；
- OpenXR Bootstrap；
- 真机冒烟报告；
- 已知兼容性问题清单。

### 退出条件

PICO 真机可以连续 10 次启动并稳定识别头显与双手柄；生命周期事件可观测，应用恢复后保持中性输入。

---

## M2：最小导航业务上屏

### 目标

在 PICO 中显示与桌面客户端一致的权威导航状态。

### 开发任务

- 在 XR 入口复用现有 WebSocket 客户端；
- 接收并校验 `navigation_visual_v3`；
- 将血管、导丝、规划路径和目标点加入最小 XR 场景；
- 复用坐标变换、模型选择和导丝尖端姿态；
- 添加最小状态面板：
  - 连接状态
  - session ID
  - 数据新鲜度
  - 后端安全等级
  - 物理引擎/演示模式标识
- 断线、重连后重新建会话；
- 丢弃旧 session 数据；
- 空值显示为未知，不转换为 0；
- 空 `risk_regions` 不生成风险体积。

### 测试

- 桌面和 PICO 同时连接同一测试会话进行字段对照；
- 对比血管、路径、导丝和目标点坐标；
- 注入乱序、旧 session、空字段和数据过期帧；
- 验证断线和重连状态；
- 验证 `guided` 模式标识。

### 产物

- XR 最小业务场景；
- 字段和坐标对照记录；
- WebSocket 异常场景测试结果。

### 退出条件

PICO 和桌面显示同一权威导航状态；断线、过期、旧 session 和空值不会被误显示为正常或安全。

---

## M3：共享核心重构

### 目标

把桌面 HUD 中的状态解析和安全逻辑抽离，建立双入口共享核心。

### 开发任务

- 建立 `NavigationViewModel`：
  - 解析和归一化 `navigation_visual_v3`；
  - 向桌面和 VR 提供稳定字段；
  - 不推导后端未提供的安全结论；
- 建立 `SafetyStateStore`：
  - 保存安全等级、`reason_code`、`stop_required`；
  - 保存急停请求、锁存、恢复状态；
  - 明确 unknown/stale；
- 建立 `DataFreshnessMonitor`：
  - 监控状态时间戳和接收时间；
  - 输出 fresh/stale/unknown；
  - 使用单调时钟做本地超时判断；
- 建立 `ControlIntentRouter`：
  - 统一桌面、VR、自动导航控制来源；
  - 统一限幅、中性帧、序号和时间戳；
  - 执行本地门控；
- 把三维血管、导丝和路径抽为 `VesselNavigationScene`；
- 把桌面入口迁移为 `MainDesktop.tscn`；
- 桌面 UI 和 VR UI 只订阅 ViewModel/Store；
- 禁止 UI 直接解析 WebSocket 原始 JSON；
- 禁止 XR 输入直接调用 WebSocket 发送控制。

### 自动化测试

- ViewModel 字段归一化；
- null/unknown/stale 语义；
- 旧 session 过滤；
- Safety Store 状态转换；
- Freshness 超时；
- Control Router 的来源、序号、限幅与归零；
- 桌面契约回归；
- XR 场景 headless 加载。

### 产物

- 四个共享核心模块；
- 桌面和 XR 双入口；
- 共享三维导航场景；
- 单元和契约测试。

### 退出条件

桌面和 XR 共用状态、安全和控制门控；UI 不再解析原始网络 JSON；现有桌面功能与测试不回归。

---

## M4：手柄控制与安全闭环

### 目标

实现可验证、默认安全、所有失效路径同帧归零的 VR 连续控制。

### 开发任务

- 创建 `openxr_action_map.tres`，定义：
  - `/ui/select`
  - `/ui/back`
  - `/ui/scroll`
  - `/workspace/grab`
  - `/workspace/scale`
  - `/view/reset`
  - `/view/cycle`
  - `/panel/toggle`
  - `/control/deadman`
  - `/control/push_axis`
  - `/control/rotate_axis`
  - `/control/emergency_stop`
  - `/control/resume_request`
- 实现 `XRInputAdapter`，只输出语义输入；
- 实现摇杆死区、非线性低速曲线和最大值限幅；
- Deadman 未按下时强制中性；
- 为控制消息添加来源、序号、客户端时间戳和 Deadman 状态；
- 实现 `XRFocusGuard`，处理：
  - 手柄追踪丢失
  - 头显摘下
  - 应用失焦
  - OpenXR session 不可见或停止
- 将断线、未 ready、数据过期、`stop_required` 和急停锁存接入统一门控；
- 任一门控失败时：
  - 本地同帧将 push/rotate 归零；
  - 阻断后续非零输入；
  - 尽力发送一次中性控制帧；
- 实现两条急停入口：
  - OpenXR 动作
  - 常驻安全面板按钮
- 实现急停状态机：
  - idle
  - requesting
  - latched
  - resume_confirming
  - resumed
- Resume 后清除旧摇杆、Deadman 和自动导航状态；
- 后端增加对应的拒绝和幂等验证。

### 关键安全测试

| 测试 | 必须满足 |
|---|---|
| Deadman 未按 | 非零摇杆不得产生非零网络控制 |
| 松开 Deadman | 同一渲染帧本地归零 |
| 手柄追踪丢失 | 同帧归零并阻断 |
| 打开系统菜单 | 同帧归零并阻断 |
| WebSocket 断开 | 立即阻断，不离线继续 |
| 数据过期 | 路径灰化并阻断 |
| `stop_required=true` | 不等待 UI 动画，立即阻断 |
| 急停触发 | 本地同帧阻断并请求后端锁存 |
| 急停锁存 | 所有非零控制被前后端拒绝 |
| Resume | 不恢复旧输入或旧自动导航 |
| 重连 | 急停意图按协议重新锁存 |

### 产物

- OpenXR Action Map；
- XR 输入与焦点守卫；
- 统一安全控制路由；
- 急停/恢复状态机；
- 安全测试报告。

### 退出条件

上述安全测试全部通过，且不存在绕过 `ControlIntentRouter` 的控制入口。

---

## M5：最小空间工作台

### 目标

在坐姿、固定工位下完成核心观察和控制任务。

### 开发任务

- 建立中央 Clinical Tabletop：
  - 距离用户 1.2–1.8 m；
  - 宽度 0.8–1.2 m；
  - 默认世界固定；
  - 可抓取旋转、有限缩放和复位；
  - 不允许直接抓取导丝改变后端状态；
- 建立右侧常驻安全面板：
  - 安全等级
  - 接触力
  - 壁距
  - 速度
  - 剩余距离
  - 数据新鲜度
  - 急停状态
  - `reason_code`
- 建立左侧影像面板：
  - DSA/腔镜标签切换；
  - 第一版不同时全分辨率刷新；
  - 数据源冻结时明确标注只读或不可用；
- 建立下方控制坞：
  - 当前模式
  - 用户推进/旋转意图
  - 后端生效值
  - Deadman
  - 控制确认
  - 拒绝原因
- 建立按需日志与调试面板；
- 实现左右手柄射线选择；
- 主要交互按钮物理尺寸不小于 35–50 mm；
- 关键状态同时使用颜色、图标和文字；
- 防止影像面板遮挡常驻安全面板。

### 可用性测试

- 坐姿完成：连接确认、选择观察模式、Deadman 控制、急停、恢复、复位；
- 正常视距读取关键数值；
- 不频繁转头超过舒适范围；
- 左右手柄均可完成基本 UI 选择；
- 工作台缩放后仍可定位导丝尖端；
- 次级面板关闭后不影响安全面板。

### 产物

- XR Clinical Workspace；
- 四类核心空间面板；
- 手柄射线 UI；
- 空间尺寸和可读性记录。

### 退出条件

无需复制桌面 11 面板即可完成核心观察和控制任务；安全信息始终可见且不只依赖颜色。

---

## M6：观察模式与性能优化

### 目标

完成正式观察模式和 PICO 移动端性能档位。

### 开发任务

- 实现模式：
  - Clinical Tabletop（默认）
  - Guidewire Navigation Follow
  - Endoscope Panel
  - Tree Overview
- Experimental Immersive Endoscope 保持实验性，不阻塞第一版；
- 建立 VR 资产档位：
  - VR High
  - VR Balanced
  - VR Lite
- 按分支拆分 Mesh/surface；
- 为远端分支启用 LOD 或简化资产；
- 优化透明过绘、Bloom 和 emission；
- 默认关闭透明阴影、DOF、SSR 和体积雾；
- 从 2× MSAA 开始真机实测；
- 非活动面板降频，隐藏面板暂停更新；
- DSA/腔镜不同时全分辨率刷新；
- 支持动态分辨率或手动 VR Lite 降级；
- 记录 CPU、GPU 帧时间、内存和热状态；
- 对路径急弯与重规划进行相机平滑；
- 数据中断时停止自动相机运动。

### 性能测试矩阵

| 场景 | 关注项 |
|---|---|
| 默认病例 + Clinical Tabletop | 接近 90 Hz、11.1 ms 帧预算 |
| 复杂病例 + VR Balanced | 是否需自动降级 |
| 复杂病例 + VR Lite | 最低可接受稳定性 |
| 透明血管近景 | GPU 帧时间和过曝 |
| DSA/腔镜切换 | 卡顿和纹理占用 |
| 连续运行 10 分钟 | 热降频、内存增长、持续掉帧 |
| 头部快速转动 | 抖动、掉帧和舒适性 |

### 舒适性测试

- 默认模式无人工虚拟移动；
- 路径急弯不瞬间翻滚；
- 第一人称观察退出入口清晰；
- Recenter 不改变控制模式；
- 重新佩戴不自动恢复控制；
- 至少 3 名测试者记录眩晕、眼疲劳和操作错误。

### 产物

- 四种正式观察模式；
- VR High/Balanced/Lite 配置；
- 性能采样报告；
- 舒适性测试记录；
- 自动/手动降级策略。

### 退出条件

默认病例 VR Balanced 稳定接近 90 Hz；连续 10 分钟无持续严重掉帧或明显内存增长；高复杂度病例可切换 VR Lite。

---

## M7：发布、部署和最终验收

### 目标

形成可重复安装、配置、运行、审计和回退的企业部署包。

### 开发任务

- 创建 Release 导出预设和签名配置；
- 将后端地址、WSS、病例和运行参数外部配置化；
- 禁止发布包写死开发机 IP；
- 配置生产 WSS 和证书验证；
- 制作 ADB/企业管理平台批量安装说明；
- 评估并记录 Kiosk/Quick Launch 配置；
- 记录设备、PICO OS、应用、病例、协议和后端版本；
- 建立启动、退出、升级和回退流程；
- 执行全量桌面与 VR 回归；
- 归档 APK、符号、配置、测试日志和发布说明。

### 最终交付物

- 签名 Release APK；
- Debug APK；
- 外部配置模板；
- 安装、升级、卸载和回退说明；
- 设备与版本矩阵；
- 功能验收报告；
- 安全验收报告；
- 性能和热稳定性报告；
- 可读性和舒适性报告；
- 桌面端回归报告；
- 已知问题与限制清单。

### 退出条件

设计方案第 15.2 节的全部第一版条件同时满足；任一 P0 安全项失败均不得发布。

---

## 8. 测试策略

### 8.1 自动化层级

1. **静态契约测试**
   - 场景、脚本和 Action Map 必需节点/动作存在；
   - UI 不直接解析 WebSocket JSON；
   - XR 输入不直接调用网络发送；
   - 所有控制入口经过 Router。
2. **共享核心单元测试**
   - ViewModel、Safety Store、Freshness Monitor、Control Router。
3. **后端协议测试**
   - VR 控制字段、序号、Deadman、急停、Resume、session 过滤。
4. **Godot headless 测试**
   - 桌面和 XR 场景可加载；
   - 脚本无解析错误；
   - 资源和 Action Map 可导入。
5. **Android 构建测试**
   - Debug/Release 导出；
   - APK 安装与启动；
   - 配置与权限检查。
6. **PICO 真机测试**
   - 生命周期、追踪、输入、安全、性能、舒适性。

### 8.2 每次合并请求最低门槛

- 相关单元测试通过；
- `tests/test_frontend_contract.py` 通过；
- Godot headless 校验通过；
- 共享核心变更同时验证桌面和 XR；
- 控制或安全变更必须附新增失败路径测试；
- Android/XR 配置变更至少生成一次 Debug APK；
- 不提交密钥、生产证书或个人设备凭据。

### 8.3 发布阻断级别

| 级别 | 定义 | 处理 |
|---|---|---|
| P0 | 可能产生非预期非零控制、急停失效、状态误导或无法启动 | 必须修复，不得发布 |
| P1 | 核心任务不可完成、持续严重掉帧、关键状态不可读 | 必须修复或经正式范围裁剪 |
| P2 | 次级功能或局部视觉问题 | 记录并排期 |
| P3 | 不影响任务的优化建议 | 可后续处理 |

---

## 9. 数据与控制验收清单

### 9.1 状态数据

- [ ] `navigation_visual_v3` 全字段解析符合契约；
- [ ] null 不显示为 0；
- [ ] unknown/stale 不显示为安全；
- [ ] 安全结论只来自后端；
- [ ] 空风险区域不生成可视风险体；
- [ ] `guided` 明确标为运动学演示；
- [ ] 旧 session 数据被过滤；
- [ ] 数据过期后路径灰化；
- [ ] 桌面与 VR 显示同一权威状态。

### 9.2 控制

- [ ] 控制来源为 `pico_vr`；
- [ ] 输入序号单调递增；
- [ ] 客户端时间戳可用于延迟审计；
- [ ] Deadman 未按下时不能发送非零控制；
- [ ] 前端和后端均拒绝非法非零控制；
- [ ] UI 区分用户意图和后端生效值；
- [ ] Resume 后旧输入不会恢复；
- [ ] 自动导航不能绕过同一门控。

### 9.3 失效安全

- [ ] 松开 Deadman 同帧归零；
- [ ] 追踪丢失同帧归零；
- [ ] 应用失焦同帧归零；
- [ ] OpenXR session 不可见时归零；
- [ ] WebSocket 断开立即阻断；
- [ ] 数据过期立即阻断；
- [ ] `stop_required=true` 立即阻断；
- [ ] 急停请求本地同帧阻断；
- [ ] 后端锁存期间所有非零控制被拒绝；
- [ ] 恢复必须经过明确确认。

---

## 10. 风险与应对

| 风险 | 预警信号 | 应对 | 决策点 |
|---|---|---|---|
| Godot/PICO OpenXR 兼容问题 | 无法稳定启动或控制器异常 | M1 最小真机冒烟、固定插件和系统版本 | 核心问题无法解决时单独评估 Unity |
| 透明血管性能不足 | GPU 帧时间超预算、热降频 | 分支 LOD、减少透明层、VR Lite | Lite 仍不达标时比较 PC 串流或框架 |
| 共享核心重构引起桌面回归 | 桌面契约或人工流程失败 | 小步迁移、双入口测试、保留兼容适配层 | M3 未通过不得进入控制开发 |
| XR 生命周期导致残留控制 | 系统菜单或摘下头显后仍有非零值 | Focus Guard、Router 同帧门控、后端超时 | 任一复现视为 P0 |
| 网络不稳定 | 丢帧、乱序、重连 | session 过滤、Freshness、断线归零 | 不允许离线非零控制 |
| 空间 UI 信息拥挤 | 频繁转头、遮挡安全状态 | 五层空间信息架构、常驻安全面板 | 可裁剪次级面板，不裁剪安全状态 |
| PICO 专有能力不足 | Kiosk、设备管理接口缺失 | `PicoPlatformBridge` 能力检测 | 不影响核心时延后专有能力 |
| 真机资源不足 | 开发排队、测试中断 | 固定至少一台开发机和一台验收机 | 无真机不得宣称阶段完成 |

---

## 11. 团队职责建议

| 角色 | 主要职责 |
|---|---|
| Godot/XR 开发 | OpenXR、场景、输入、空间 UI、Android 导出 |
| 前端共享核心开发 | ViewModel、Safety Store、Freshness、Control Router、桌面迁移 |
| 后端开发 | 控制字段、序号/Deadman校验、急停锁存、WSS |
| 3D/渲染 | VR 资产档位、透明材质、LOD、性能优化 |
| 测试/安全负责人 | 失效注入、真机矩阵、验收证据、发布阻断 |
| 产品/医学顾问 | 信息优先级、可读性、仿真/临床表述边界 |

小团队可以一人承担多个角色，但安全验收不应只由功能实现者单独确认。

---

## 12. 版本与分支建议

建议采用以下版本序列：

```text
0.1.0-xr-smoke       M1：OpenXR 真机冒烟
0.2.0-xr-visual      M2：导航业务上屏
0.3.0-shared-core    M3：共享核心
0.4.0-xr-control     M4：控制与安全闭环
0.5.0-xr-workspace   M5：空间工作台
0.6.0-xr-balanced    M6：模式与性能
1.0.0-pico-release   M7：正式交付
```

每个里程碑建立可安装 APK 和对应测试记录，不以仅存在代码或编辑器截图作为完成依据。

---

## 13. 第一轮迭代建议

第一轮只实施 M0 和 M1，建议控制在 1–2 周：

1. 修正并固定 Godot 4.7.1 启动和校验入口；
2. 配置 JDK、Android SDK、ADB 和 Build Template；
3. 引入并固定 OpenXR Vendors 插件；
4. 新建最小 `MainXR.tscn`；
5. 实现 OpenXR 初始化和生命周期日志；
6. 创建 Debug Android 导出预设；
7. 安装到 PICO 并验证头显、左右手柄和应用焦点；
8. 记录冷启动、系统菜单、摘戴、退出和重入结果；
9. 保持所有业务控制为禁用状态；
10. 通过 M1 退出条件后再接入 WebSocket 和真实导航场景。

第一轮不同时开展完整空间 UI、共享核心大重构或材质精调，以免把平台兼容问题与业务问题混在一起。

---

## 14. 最终完成定义

项目只有在以下证据齐备时才标记为第一版完成：

- PICO 真机安装、启动、退出和重入记录；
- OpenXR 头部与双手柄追踪记录；
- 桌面/VR 权威状态一致性记录；
- Deadman 与全部失效路径同帧归零测试；
- 急停请求、锁存、拒绝和恢复闭环测试；
- 常驻安全面板可读性测试；
- VR Balanced 与 VR Lite 性能报告；
- 连续运行和热稳定性报告；
- 多人舒适性记录；
- 桌面端自动化与人工回归报告；
- 签名 APK、部署说明、配置模板和回退方案；
- 无未关闭的 P0/P1 缺陷。

任何单项缺失时，应报告为“阶段完成”或“有条件通过”，不得报告为 PICO VR 第一版正式交付完成。
