# 血管介入导航系统技术设计文档

## 1. 目标与范围

本文档定义血管介入导航系统的完整工程架构、模块职责、数据流、接口规范与安全约束。系统覆盖术前影像建模、血管拓扑提取、路径规划、术中配准、状态估计、机器人闭环控制、安全监督以及高层智能辅助交互，形成面向医疗机器人场景的实时导航系统。

系统设计遵循“可验证、可解释、可接管”的工程原则。自动导航建立在确定性控制、安全监督器和实时闭环之上。所有执行级控制、紧急制动、安全状态切换和硬件驱动均由确定性模块完成，高层智能模块不参与底层控制闭环。

系统能力范围包括：

- 术中 `DSA/X-ray` 主视图导航显示
- 血管三维模型、中心线、规划路径、安全走廊与风险区域可视化
- 导丝/导管位姿、速度、方向、预测轨迹与机器人状态实时显示
- 任意起终点路径规划，支持血流方向、曲率约束、半径约束和导丝运动学约束
- 全局规划器与局部规划器分层协同，支持 `Dijkstra / A* / Hybrid A*`
- `2D/3D` 配准、器械跟踪、时间同步、滤波与多源融合状态估计
- `ROS2` 实时控制层、硬件驱动层、安全监督器、人工接管与独立急停链路
- `VLA` 高层策略辅助与 `LLM` 解释交互，不参与电机级闭环控制

系统能力域定义如下：

```text
Navigation Visualization : 导航可视化系统
Planning & Feasibility   : 路径规划与运动学可行性系统
Robot Closed-loop Control: 机器人闭环辅助导航系统
VLA Policy Layer         : VLA 高层策略辅助系统
Supervised Autonomy      : 受监督自主导航系统
```

系统包含影像导航、路径规划、状态估计、实时控制、安全监督、`RL/VLA/LLM` 高层智能模块。`FastAPI + WebSocket` 承担 UI、数据服务、日志和 AI 调度；`ROS2 + DDS + C++` 承担实时控制、状态发布、硬件接口和安全链路。

## 2. 术中导航界面目标

系统前端界面以术中操作流程为中心。术中模式下，`DSA/X-ray` 为主操作视图，三切面 `Axial / Sagittal / Coronal` 用于规划、配准复核和辅助定位，不作为默认主视图。

### 2.1 术中模式总体布局

系统采用“主操作区 + 辅助导航区 + 顶部控制状态层 + 底部状态栏”的固定布局：

```text
+--------------------------------------------------------------------------------------+
| Top Info Bar: control mode | robot | path | vessel | safety | planner | e-stop      |
+---------------------------------------------------+----------------------------------+
| Main DSA / X-ray View                              | 3D Navigation Assistant         |
| fluoroscopy + tool + path + corridor + prediction | CTA vessel + centerline + path  |
| risk + feasibility + target direction             | bifurcation + risk + tip pose   |
+---------------------------------------------------+----------------------------------+
| Bottom Status Bar: coord | time sync | motion | estimator | controller | alarms      |
+--------------------------------------------------------------------------------------+
```

布局约束如下：

- 主窗口占界面面积 `60%~70%`
- 右侧 3D 辅助窗口占 `20%~30%`
- 顶部信息栏高度 `40~60 px`
- 底部状态栏持续显示控制频率、同步误差、节点健康状态和报警
- 控制模式固定显示 `MANUAL / ASSIST / SUPERVISED AUTO / SAFE HOLD / EMERGENCY STOP`

### 2.2 术中主窗口（DSA / X-ray）

术中导航操作以实时 `DSA/X-ray` 透视图为主依据。该窗口作为默认焦点区域，承担导丝、导管和器械推进操作。

主窗口显示以下叠加信息：

- 导丝/导管当前位置、方向、速度与历史轨迹
- 前瞻 `200~500 ms` 预测轨迹
- 规划路径在 `2D` 透视坐标下的投影
- 安全走廊、中心线目标方向和下一分叉提示
- 分叉点、狭窄段、高曲率段、禁入区和高接触风险区域
- 控制模式、限速、制动、人工接管和安全保持状态
- `VLA` 输出的高层动作建议及其审核结果

路径可行性采用颜色编码：

```text
Green  : 安全可行
Yellow : 曲率较高或接近运动学边界
Orange : 接触风险较高或局部裕量不足
Red    : 运动学不可达、碰撞风险不可接受或禁入
```

系统明确区分安全风险与运动学不可达。安全风险由壁面距离、接触力、预测碰撞和路径偏移触发；运动学不可达由最小弯曲半径、曲率连续性、尖端转向能力和推进/旋转限制触发。

### 2.3 3D 导航辅助窗口（右侧）

右侧窗口用于空间理解与路径解释，不作为主操作界面。其显示内容包括：

- 基于 `CTA/MRA` 重建的血管三维模型
- 中心线骨架、分支拓扑和局部半径
- 全局规划路径、局部规划轨迹和候选路径
- 当前尖端位姿估计及估计置信度
- 短时预测轨迹与安全走廊
- 分叉点、狭窄段、高曲率段、禁入区域和不可达路径段
- 目标点、回退点、最近安全点与重规划起点

右侧窗口以辅助观察视角运行，支持与 `DSA` 选点联动，并优先显示路径、尖端位置、风险区和预测轨迹。

### 2.4 顶部横向信息栏

顶部信息栏采用单行半透明信息条，持续显示控制、路径、血管、安全和通信状态。

显示内容包括：

- 控制模式：`MANUAL / ASSIST / SUPERVISED AUTO / SAFE HOLD / EMERGENCY STOP`
- 机器人状态：连接状态、执行状态、驱动状态、控制器频率
- 路径进度：已行进距离、剩余距离、当前路径段编号、下一分叉方向
- 血管属性：局部半径、曲率、分支等级、局部裕量
- 安全指标：`d_wall`、`d_path`、预测碰撞时间、接触力
- 规划状态：全局路径状态、局部规划状态、运动学可行性等级
- 风险等级：`Green / Yellow / Orange / Red`
- 关键控制按钮：`急停 / 人工接管 / 恢复导航 / 安全保持`

### 2.5 右侧数据与安全面板

右侧数据与安全面板固定显示：

- `d_wall`、`d_path`、`radius`、`curvature`
- 推进速度、旋转速度、加速度和控制限幅状态
- 局部规划器状态、最近一次重规划时间和轨迹可行性
- 状态估计器输出置信度、协方差摘要和时间延迟
- 风险状态与触发原因
- 是否限速、是否制动、是否自动回退、是否进入人工接管
- `VLA` 当前高层策略输出、置信度和安全审核结果
- `LLM` 对路径、报警原因和当前系统状态的解释文本

### 2.6 底部状态栏

底部状态栏显示长期可见的实时运行信息：

- 当前世界坐标、图像坐标、机器人基坐标与尖端坐标
- 当前路径进度和切片信息
- 控制器频率、状态估计频率、安全检测频率
- 图像时间戳、硬件时间戳、ROS2 时间和同步误差
- 当前报警、最近事件编号和事件时间
- 网络连接、图像流状态、ROS2 节点状态和 DDS 状态

### 2.7 规划模式与辅助模式

系统界面区分如下运行模式：

- 术中模式：默认显示 `DSA/X-ray` 主窗口与 3D 辅助窗口
- 规划模式：打开三切面，用于路径复核、目标点设定和配准检查
- 辅助模式：术中临时调出三切面复核空间位置
- 控制模式：在 `MANUAL / ASSIST / SUPERVISED AUTO / SAFE HOLD / EMERGENCY STOP` 之间切换

### 2.8 叠加对象

术中导航界面的标准叠加对象包括：

- 实时 `DSA/X-ray` 图像
- `CTA/MRA` 重建血管模型
- `2D/3D` 配准后的血管轮廓投影
- 中心线、全局规划路径和局部轨迹
- 安全走廊、预测轨迹和最近壁面点
- 机器人当前位置、朝向、速度和控制模式
- 导丝/导管位姿估计、置信区间和不确定性范围
- 目标点、风险点、分叉点、禁入区域和运动学不可达区域

## 3. 总体架构（前后端分离）

系统采用前端导航工作站、Web 后端服务、算法服务、状态估计层、规划层、安全监督层、ROS2 实时控制层和机器人硬件层分离的工业级架构。Web 系统不承担硬实时控制，执行链路由 `ROS2 / DDS / C++` 和硬件驱动完成。

核心数据流如下：

```text
DSA/X-ray
    -> Vessel Segmentation / Tool Tracking
    -> 2D/3D Registration
    -> State Estimator
    -> Global Planner
       (Dijkstra / A* / Hybrid A*)
    -> Local Planner
       (curvature constrained)
    -> Safety Supervisor
    -> MPC / PID Controller
    -> ROS2 Real-time Layer
    -> Robot Hardware
```

`VLA/LLM` 位于高层辅助决策层，仅输出策略意图、解释信息或确认请求，不直接输出电机级控制指令，不绕过状态估计器、安全监督器、局部规划器和底层控制器。

### 3.1 前端职责

前端模块负责：

- 术中导航 UI 渲染与交互
- `DSA` 主窗口、3D 辅助窗口和三切面联动
- 位姿、路径进度、预测轨迹、控制模式和安全预警显示
- 图层开关、透明度、模式切换和人工接管交互
- `VLA` 策略输出、`LLM` 解释结果和决策追溯展示
- 控制频率、通信状态、估计延迟和节点健康状态监视

### 3.2 后端职责

后端模块负责：

- 数据加载：`DICOM / NIfTI / VTK / VTP / STL`
- 血管模型、中心线、拓扑图、端点映射和前端加载包管理
- 全局路径规划、局部轨迹规划、路径平滑和重规划服务
- 中心线属性计算：半径、直径、曲率、流向距离
- 安全风险评估、事件日志和追溯数据管理
- `VLA / LLM` 推理调度与高层语义服务
- WebSocket 实时推送和 HTTP 数据服务
- ROS2 桥接层数据转发

硬件驱动层通过 `hardware_interface_node` 接入瑞鈊跟踪设备 SDK，并纳入 `ROS2` 实时系统。该模块负责设备扫描、物理连接、跟踪启停、原始位姿读取和链路状态监测。系统采用以下接口完成设备接入与运行控制：

- `updateDeviceInfo / getDeviceInfo`：刷新并获取可连接设备主机名与 IP 地址，形成设备发现与自动重连基础
- `connect / disconnect`：建立和释放跟踪设备物理链路
- `startTracking / stopTracking`：控制术中跟踪链路进入运行或停止状态
- `trackingUpdate / getTrackingData`：刷新并获取所有传感器实时跟踪数据
- `getConnectionStatus / getSensorConnected / getNetAdaptorInfo`：获取设备、传感器和网络链路健康状态

瑞鈊设备原始数据进入 `state_estimator_node` 后，统一完成时间戳对齐、坐标转换、滤波融合和置信度计算，输出系统标准状态量：

- `tip_pose`
- `tip_velocity`
- `tip_direction`
- `tracking_confidence`
- `tracking_status`

状态估计器作为独立模块运行，统一处理术中图像、术前模型、编码器、跟踪器、时间戳和多源观测，输出规划与控制可用的统一状态。

### 3.3 VLA 驱动的数据流

`VLA` 被定义为高层策略模块，参与意图理解、动作候选生成和策略解释，不作为闭环控制器。

```text
DSA/X-ray 图像流
    + 3D 血管模型 / 中心线 / 规划路径
    + State Estimator 输出
    + 局部曲率 / 半径 / 风险指标
    + 控制模式 / 安全状态 / 事件历史
    -> 多模态状态编码
    -> VLA 高层策略推理
    -> 动作候选 / 意图 / 解释
    -> Safety Supervisor 审核
    -> Local Planner / Trajectory Planner
    -> MPC / PID Controller
```

`VLA` 输出限定为高层动作候选：

```text
advance_slowly
rotate_clockwise
rotate_counterclockwise
hold
retract
request_manual_override
ask_for_confirmation
replan_required
```

`VLA` 不输出电机电流、PWM、未限幅连续速度或绕过安全模块的推进命令。

## 4. 技术选型

### 4.1 技术方案

系统采用以下技术栈：

- 前端：`React + vtk.js`
- Web 后端：`Python + FastAPI + WebSocket`
- 实时控制：`ROS2 + DDS + C++`
- 硬件通信：`EtherCAT / CAN / 厂商 SDK`
- 算法库：`VTK / NumPy / SciPy / NetworkX / VMTK`
- 状态估计：`EKF / UKF / Particle Filter`
- 控制算法：`PID / MPC / 限幅控制 / 力反馈控制`

`FastAPI + WebSocket` 负责 UI、数据服务、日志和 AI 调度；`ROS2 + DDS + C++` 负责机器人执行、传感器回读、控制周期、安全监督和硬件驱动。

### 4.2 系统组件

系统包含以下工程组件：

- `Cornerstone3D`
- `Three.js`
- `ROS2 Control`
- `Orocos / 实时 Linux`
- `MONAI Deploy`

### 4.3 面向手术机器人血管内导航的系统技术栈

系统技术栈覆盖从术前影像到术中控制的完整闭环：

```text
CT/CTA/DSA 影像
    -> 血管分割与三维建模
    -> 中心线 / 拓扑图提取与修复
    -> 2D/3D 配准与器械跟踪
    -> State Estimator
    -> 全局路径规划与局部曲率受限规划
    -> 导丝 / 导管仿真验证
    -> Safety Supervisor
    -> ROS2 实时控制与硬件执行
```

#### 4.3.1 医学影像与血管建模

系统采用：

- 医学影像格式：`DICOM / NIfTI`
- 影像处理：`SimpleITK / ITK / VTK`
- 血管分割：`MONAI / nnU-Net / 3D Slicer Segment Editor`
- 血管中心线：`VMTK`
- 三维可视化：`3D Slicer / VTK / ParaView`

系统实现 `3D Slicer + VMTK` 建模链路。`3D Slicer` 负责加载、分割检查、配准复核和导航可视化；`VMTK` 负责中心线、分支拓扑和局部半径提取。

#### 4.3.2 路径规划模块

路径规划采用全局规划器与局部规划器分层结构。

全局规划器负责：

- 血管拓扑搜索
- 目标段选择
- 全局最短安全路径求解
- 安全走廊生成
- 多候选路径排序

全局规划算法采用 `Dijkstra / A* / Hybrid A*`。其中 `Dijkstra / A*` 运行于血管拓扑图，`Hybrid A*` 用于包含朝向状态和曲率连续性的候选走廊搜索。

局部规划器负责：

- 曲率可行性验证
- 局部避碰
- 尖端姿态调整
- 高曲率绕行
- 短时轨迹生成
- 控制参考输出

局部规划算法采用 `Hybrid A* + MPC + spline optimization`，平滑方法采用 `B-spline / Bezier / clothoid`。

系统对导丝/导管执行轨迹施加运动学约束：

- 最小弯曲半径约束
- 最大曲率约束
- 曲率变化率约束
- 扭转约束
- 尖端转向约束
- 推进速度与旋转速度约束

统一代价模型定义如下：

```text
edge_cost = w_len  * length
          + w_curv * curvature_penalty
          + w_rad  * radius_penalty
          + w_dir  * direction_penalty
          + w_risk * contact_risk_penalty
          + w_dyn  * kinematic_penalty
```

#### 4.3.3 仿真与数字孪生

系统包含导丝/导管仿真层，用于路径可通过性验证、安全策略验证和闭环联调前测试。

导丝/导管模型包括：

- `Cosserat Rod`
- `Piecewise Constant Curvature`
- `Beam Model`
- `SOFA BeamAdapter`

仿真技术栈包括：

- `SOFA Framework`
- `SOFA collision + FEM`
- `VTK / STL / VTU mesh`
- `Python + SOFA scene`

仿真层输出用于验证：

- 最小弯曲半径是否满足
- 狭窄段和分叉口是否可通过
- 限速、制动、回退和安全保持是否稳定
- 局部策略与确定性安全模块是否一致

#### 4.3.4 实时导航与配准

术中导航系统包含：

- 术前-术中配准：`ICP / CPD / 中心线配准`
- `2D/3D` 配准：`DRR + X-ray / DSA`
- 导丝/导管检测：`U-Net / YOLO / 传统线结构检测`
- 状态估计：`EKF / UKF / Particle Filter`
- 导航平台：`3D Slicer + SlicerIGT`

`State Estimator` 统一处理配准漂移、图像延迟、编码器延迟和器械检测噪声，输出带时间戳、协方差和置信度的统一状态。

#### 4.3.5 机器人控制与通信

控制层采用以下分层结构：

```text
Navigation Planner
        -> Local Planner / Trajectory Planner
        -> Safety Supervisor
        -> MPC / PID Controller
        -> ROS2 Real-time Layer
        -> Motor Driver / Robot Hardware
        -> Sensor Feedback
        -> State Estimator
```

ROS2 图结构包含：

- `image_node`
- `registration_node`
- `state_estimator_node`
- `global_planner_node`
- `local_planner_node`
- `controller_node`
- `safety_supervisor_node`
- `hardware_interface_node`
- `logger_node`

控制周期定义如下：

| 模块 | 频率 |
|---|---|
| 图像刷新 | `15~30 Hz` |
| 状态估计 | `30~60 Hz` |
| 全局重规划 | `1~5 Hz` |
| 局部规划 | `20~100 Hz` |
| 安全检测 | `100 Hz` |
| 控制器 | `500~1000 Hz` |
| VLA 推理 | `1~5 Hz` |
| LLM 解释交互 | 按需触发，非实时 |

#### 4.3.6 VLA 自动导航模块

`VLA` 模块位于高层策略层，负责对术中视觉上下文、当前状态和任务目标进行语义理解，并输出高层动作建议。

模块输入包括：

- 当前 `DSA/X-ray` 图像帧
- 当前尖端位姿、速度和方向
- 全局路径和局部轨迹
- 曲率、半径、风险等级和控制模式
- 事件历史与人工接管状态

模块输出包括：

- 推进、旋转、暂停、回退类高层动作候选
- 重规划请求
- 人工确认请求
- 策略解释和风险描述

`VLA` 输出必须经过安全监督器和局部规划器审核，不直接进入控制器。

#### 4.3.7 AI 模块定位

系统将 AI 模块划分为两类：

- 视觉 AI：负责图像处理、分割、检测、配准辅助
- 大模型模块：负责高层语义理解、策略辅助、解释交互和文档化输出

`LLM` 模块承担以下职责：

- 医疗语义理解
- 术中路径解释
- 手术策略辅助说明
- 系统状态解释
- 报警原因解释
- 自然语言交互

`LLM` 不参与实时控制、不参与急停判断、不替代路径规划算法、不参与安全闭环。

#### 4.3.8 系统执行技术栈

系统执行栈定义如下：

```text
3D Slicer
+ VMTK
+ Python / FastAPI
+ NetworkX
+ SOFA + BeamAdapter
+ ROS2 + DDS + C++
+ MONAI / nnU-Net
+ VLA / LLM Assistant
```

## 5. 数据与坐标规范

### 5.1 输入

系统输入包括：

- `Segment_1.vtk`：血管模型
- `...step3b_with_radius_cleaned.vtp`：中心线与半径
- `...step5_graph_directed.json`：有向拓扑图
- `DSA/X-ray` 术中图像流
- 机器人编码器、驱动和力传感器数据
- 跟踪器输出的器械位姿数据

### 5.2 统一规范

系统统一采用：

- 坐标系：`LPS`
- 长度单位：`mm`
- 角度单位：`rad` 或 `deg`，接口明确声明
- 时间基准：`ROS2 time + hardware timestamp`
- 中心线重采样间距：`0.5~1.0 mm`

系统定义以下坐标系：

- `Robot Base Frame`
- `Tool Frame`
- `Tip Frame`
- `Image Frame`
- `Patient Frame`

系统维护统一 `TF tree`，保证图像、器械、机器人和患者坐标之间的可追溯转换。

系统实现时间同步机制：

- 图像时间戳同步
- 硬件时间戳同步
- ROS2 时钟统一
- 延迟补偿与插值

瑞鈊跟踪设备数据纳入统一坐标规范。系统将 `SensorToolTrackingData` 映射为器械跟踪数据实体，将 `Transformation` 映射为标准位姿对象，并统一转换到 `LPS` 坐标系下。映射结果用于生成以下标准状态：

- `Patient Frame` 下的尖端位置
- `Tip Frame` 下的器械方向
- `Image Frame` 下的显示位姿
- `Robot Base Frame` 下的控制参考姿态

系统采用 `pivotTipCalibration` 完成器械尖端标定，并将标定结果固化为 `Tool Frame -> Tip Frame` 固定变换参数。系统采用 `fixedDirCalibration` 完成器械方向标定，并将结果写入导丝/导管标准方向定义，用于 `Tangent` 计算、路径方向一致性判断、局部规划姿态连续性约束和安全监督中的偏转角计算。

`SensorCalibrationAlert` 映射为系统标定状态对象，用于前端显示标定状态、误差预警和器械可用性判断。

### 5.3 每节点属性

每个中心线节点包含：

- `Radius`
- `Diameter = 2 * Radius`
- `Curvature`
- `FlowDistance`
- `Tangent`
- `BranchId`
- `RiskWeight`
- `KinematicMargin`

### 5.4 索引

系统索引包括：

- `KDTree(point -> centerline_node_id)`
- `PathIndex(point -> path_segment_id)`
- `NearestWallIndex(point -> surface_id)`

## 6. 预处理与离线产物

系统离线阶段生成以下产物：

1. 坐标统一后的血管模型与中心线
2. 重采样中心线及其属性
3. 有向图与边代价
4. 拓扑合法性检查结果
5. 图修复结果 `graph_repair`
6. 终点集合映射 `goal_node_id`
7. 反向最短路 `dist_to_goal[node]`
8. 安全走廊预计算数据
9. 前端加载包 `scene + graph + endpoint map + metadata`

离线处理包含以下固定流程：

```text
影像预处理
    -> 分割模型
    -> 中心线提取
    -> 拓扑验证
    -> 图修复
    -> 属性计算
    -> 有向图生成
    -> 路径代价预计算
    -> 导航场景打包
```

## 7. 路径规划引擎

### 7.1 触发条件

路径规划在以下条件下触发：

- 起点变化
- 终点变化
- 配准更新
- 血流方向约束变化
- 曲率或半径阈值变化
- 局部规划不可行
- 安全监督器触发重规划
- 人工确认重规划

### 7.2 求解算法

路径规划采用双层规划架构。

全局规划器：

- 运行于血管拓扑图
- 使用 `Dijkstra / A* / Hybrid A*`
- 输出候选路径、全局参考路径和安全走廊
- 综合距离、曲率、狭窄、分支方向和风险区域代价

局部规划器：

- 运行于尖端邻域和全局参考走廊
- 使用 `Hybrid A* + MPC + spline optimization`
- 满足曲率、半径、扭转和尖端转向约束
- 输出短时可执行轨迹和控制参考

运动学约束包括：

- 最小弯曲半径
- 最大允许曲率
- 曲率连续性
- 扭转变化限制
- 尖端转向限制
- 推进与旋转速度上限

### 7.3 代价函数（示例）

```text
cost = w_len  * length
     + w_curv * curvature_penalty
     + w_rad  * radius_penalty
     + w_dir  * direction_penalty
     + w_risk * risk_penalty
     + w_dyn  * kinematic_penalty
```

局部轨迹优化目标定义为：

```text
J = w_ref   * tracking_error
  + w_smooth* curvature_change
  + w_ctrl  * control_effort
  + w_safe  * wall_proximity_penalty
  + w_feas  * kinematic_violation_penalty
```

### 7.4 输出

路径规划引擎输出：

- 路径节点序列
- 全局参考路径
- 局部可执行轨迹
- 路径长度
- 总代价
- 顺/逆流比例
- 曲率峰值
- 最小半径裕量
- 可行性等级
- 重规划原因

## 8. 术中联动交互

系统包含以下联动交互：

1. 主视图选点联动 3D 视图与三切面定位
2. 3D 点选反查 `DSA` 投影与切面位置
3. 路径点点击联动属性面板更新
4. 预测轨迹叠加 `predictive overlay`
5. 安全走廊可视化 `safe corridor visualization`
6. 风险点、壁面最近点和偏离方向联动显示
7. 人工接管后冻结自动推进建议并保留解释信息
8. `LLM` 交互面板与当前路径、报警和状态联动

## 9. 实时监控与通信

系统将实时监控与通信分为 UI 层通信和控制层通信。

- WebSocket 仅用于 UI 层实时显示、日志同步和高层交互
- `ROS2 DDS` 用于控制、状态估计、安全监督和硬件链路

瑞鈊跟踪设备的实时通信链路定义如下：

```text
Ruixin Device
    -> updateDeviceInfo / getDeviceInfo
    -> connect
    -> startTracking
    -> trackingUpdate
    -> getTrackingData
    -> hardware_interface_node
    -> state_estimator_node
    -> TF transform normalization
    -> tip_pose / tip_direction / tip_velocity
    -> global_planner_node
    -> local_planner_node
    -> safety_supervisor_node
    -> frontend visualization
```

瑞鈊设备状态监测链路定义如下：

```text
Ruixin Device
    -> getNetAdaptorInfo / getConnectionStatus / getSensorConnected
    -> watchdog_node
    -> safety_supervisor_node
    -> SAFE_HOLD / alarm / control inhibit
    -> frontend status bar / event log
```

### 9.1 WebSocket 推送频率

WebSocket 推送频率定义如下：

- 位姿显示：`10~30 Hz`
- 规划状态：`1~5 Hz`
- 报警事件：事件触发即推送
- `VLA` 结果：`1~5 Hz`
- `LLM` 解释：按需推送
- 设备连接状态：事件触发即推送
- 标定状态：事件触发即推送

### 9.2 消息示例

```json
{
  "type": "robot_pose",
  "timestamp": 1710000000.12,
  "position": [125.3, 88.1, 62.7],
  "direction": [0.21, 0.73, 0.65],
  "path_index": 142,
  "status": "navigating"
}
```

```json
{
  "type": "path_update",
  "global_path_id": "gp_0021",
  "local_traj_id": "lp_1042",
  "feasibility": "yellow",
  "replan_reason": "high_curvature"
}
```

```json
{
  "type": "safety_event",
  "risk_level": "danger",
  "distance_to_wall": 0.82,
  "distance_to_path": 1.24,
  "action": "speed_limit"
}
```

```json
{
  "type": "llm_query",
  "query": "当前路径是否安全？"
}
```

```json
{
  "type": "llm_response",
  "answer": "当前路径存在高曲率段，局部半径裕量偏低，系统已进入限速状态。"
}
```

## 10. 前端显示指标（实时）

### 10.1 顶部关键指标栏

顶部显示：

- 控制模式
- 机器人连接状态
- 当前路径段编号
- 剩余路径长度
- 局部半径
- 曲率
- `d_wall`
- `d_path`
- 风险等级
- 估计延迟
- 控制器频率
- 安全监督状态

### 10.2 主窗口上下文显示

主窗口上下文显示包括：

- 当前器械状态说明
- 下一目标方向
- 路径建议
- 限速或制动原因
- 预测轨迹与可行性颜色编码
- 当前 `VLA` 动作候选及审核结果

### 10.3 右侧数据与安全面板

右侧面板显示：

- 实时运动学指标
- 局部规划状态
- 安全监督器状态
- 最小壁距和路径偏移
- 控制限幅器状态
- 回退、保持、人工接管状态
- 风险触发原因文本

### 10.4 底部状态栏

底部状态栏显示：

- 坐标与时间同步信息
- 图像与硬件时间戳
- ROS2 节点状态
- DDS 通信状态
- 当前事件编号与报警摘要

### 10.5 智能交互与解释模块

前端包含语音/文本交互面板，支持以下内容：

- 当前状态说明
- 路径解释与候选路径比较
- 报警原因解释
- 风险说明
- 手术策略辅助说明

`LLM Assistant Module` 仅提供高层语义支持，不直接参与机器人控制与安全决策。

## 11. 接口设计

### 11.1 HTTP

- `GET /api/case/{id}/volume`
- `GET /api/case/{id}/vessel-model`
- `GET /api/case/{id}/centerline`
- `GET /api/case/{id}/planned-path`
- `POST /api/plan`
- `POST /api/llm/query`
- `GET /api/events/{id}`
- `GET /api/system/status`

### 11.2 WebSocket

- `robot_pose`
- `path_update`
- `risk_alert`
- `safety_event`
- `vla_output`
- `llm_response`

### 11.3 ROS2 接口

ROS2 主题、服务与动作接口定义如下：

- Topic: `/image/current_frame`
- Topic: `/registration/state`
- Topic: `/state_estimator/tip_pose`
- Topic: `/state_estimator/tip_direction`
- Topic: `/state_estimator/tracking_confidence`
- Topic: `/planner/global_path`
- Topic: `/planner/local_trajectory`
- Topic: `/safety/status`
- Topic: `/controller/state`
- Topic: `/hardware/feedback`
- Topic: `/hardware/ruixin_tracking_raw`
- Topic: `/hardware/ruixin_connection_status`
- Topic: `/hardware/ruixin_sensor_status`
- Topic: `/hardware/ruixin_net_status`
- Topic: `/system/watchdog`

- Service: `/planner/replan`
- Service: `/control/set_mode`
- Service: `/safety/safe_hold`
- Service: `/safety/manual_override`
- Service: `/hardware/ruixin_connect`
- Service: `/hardware/ruixin_disconnect`
- Service: `/hardware/ruixin_start_tracking`
- Service: `/hardware/ruixin_stop_tracking`
- Service: `/calibration/pivot_tip`
- Service: `/calibration/fixed_direction`

- Action: `/trajectory/execute_local_plan`
- Action: `/trajectory/retract_safe_distance`

瑞鈊设备接口与节点职责定义如下：

- `hardware_interface_node`：调用 `updateDeviceInfo / getDeviceInfo / connect / disconnect / startTracking / stopTracking / trackingUpdate / getTrackingData`
- `state_estimator_node`：订阅瑞鈊原始跟踪数据并发布标准位姿与置信度状态
- `calibration_manager`：调用 `pivotTipCalibration / fixedDirCalibration`，更新 `TF tree`
- `watchdog_node`：调用 `getNetAdaptorInfo / getConnectionStatus / getSensorConnected`，发布快速健康状态

## 12. 前端模块结构

```text
src/
  components/
    MainDSAView/
      DSAView.tsx
      PredictionOverlay.tsx
      SafetyCorridor.tsx
      TrajectoryPreview.tsx
    Navigation3D/
      View3D.tsx
      VesselRenderer.tsx
      PathRenderer.tsx
    MultiPlanarView/
      AxialView.tsx
      SagittalView.tsx
      CoronalView.tsx
    Panels/
      ControllerStatus.tsx
      SafetyPanel.tsx
      PathInfoPanel.tsx
      LLMChatPanel.tsx
      VLAStatusPanel.tsx
    Layout/
      TopInfoBar.tsx
      BottomStatusBar.tsx
  services/
    api.ts
    websocket.ts
    rosBridge.ts
  vtk/
    volumeLoader.ts
    modelLoader.ts
    pathRenderer.ts
    crosshairController.ts
    cameraSync.ts
  store/
    caseStore.ts
    viewerStore.ts
    robotStore.ts
    safetyStore.ts
    plannerStore.ts
    llmStore.ts
  pages/
    NavigationPage.tsx
```

## 13. 工程交付模块

### 13.1 影像建模与拓扑数据包

交付内容包括：

- 血管模型
- 中心线
- 有向图
- 节点属性
- 终点映射
- 场景打包数据

### 13.2 路径规划与运动学可行性模块

交付内容包括：

- 全局规划器
- 局部规划器
- 曲率约束求解器
- 路径平滑器
- 可行性评估器

### 13.3 ROS2 实时控制模块

交付内容包括：

- 控制节点
- 安全监督节点
- 状态估计节点
- 硬件接口节点
- Watchdog 节点
- 控制参数配置文件

### 13.4 术中导航闭环模块

交付内容包括：

- 术中导航 UI
- 配准与跟踪接口
- 状态显示与日志模块
- 控制模式管理模块

### 13.5 智能局部策略模块

交付内容包括：

- 局部策略评估器
- 高层动作审核接口
- 风险解释接口

### 13.6 高层智能辅助模块

交付内容包括：

- `VLA Assistant`
- `LLM Assistant Module`
- 交互 API
- 解释面板
- 事件追溯文本生成模块

## 14. 质量与验收标准

### 14.1 功能验收

- 连通子图内任意两点路径规划成功率 `>= 99%`
- 局部规划器轨迹连续且满足运动学约束
- 四视图定位误差 `<= 1 voxel`
- 配准状态变更后自动更新路径和安全走廊
- 人工接管、急停和安全保持状态切换正确

### 14.2 性能验收

- 首屏加载 `< 3 s`
- 交互帧率 `>= 30 FPS`
- 单次全局路径计算 `< 200 ms`
- 局部规划周期满足设定频率
- `LLM` 解释不阻塞控制链路

### 14.3 稳定性

- WebSocket 自动重连
- ROS2 节点失联检测
- DDS 通信异常恢复
- 异常请求返回明确错误码
- 日志可追溯

### 14.4 VLA 自动导航验收

- `VLA` 输出必须通过安全审核
- 高层动作候选与当前状态一致
- 审核拒绝时不影响控制闭环
- 输出具备追溯记录

### 14.5 控制与安全验收

- 轨迹跟踪误差满足控制指标
- 超调量受控
- 控制延迟满足实时要求
- `Emergency stop latency < 50 ms`
- 控制器抖动 `jitter` 在阈值内
- 通信丢包和 DDS 切换时系统进入安全态

## 15. 安全控制与碰撞检测机制设计

### 15.1 设计目标

系统安全子系统形成独立闭环，负责：

- 预测碰撞
- 分级预警
- 自动限速
- 紧急制动
- 自动回退
- 安全保持
- 人工接管
- 风险解释

### 15.2 安全子系统架构

```text
传感器 / 位姿输入
    -> 状态估计与时间同步
    -> 位置映射与轨迹预测
    -> 碰撞检测与风险评估
    -> 安全决策引擎
       -> 告警输出
       -> 限速控制
       -> 紧急制动
       -> 回退 / 安全保持 / 人工接管
       -> LLM 风险解释
```

系统包含独立硬件急停链路，不依赖 Web、ROS2 高层节点或 AI 模块。

瑞鈊跟踪设备链路状态纳入安全监督器输入。系统将 `getNetAdaptorInfo` 定义为硬件断连快速检测接口，并将其直接接入 `watchdog_node`。网络链路异常、适配器断连、设备失联或传感器不可用时，安全状态机立即执行以下动作：

- 切换到 `SAFE HOLD`
- 冻结局部规划输出
- 阻断自动推进链路
- 维持限幅器与制动器处于可触发状态
- 推送前端报警与事件日志

### 15.3 输入数据

- 血管表面模型
- 中心线与规划路径
- 尖端位置与方向
- 推进/旋转速度
- 力反馈
- 图像跟踪结果
- 状态估计输出
- 控制模式与当前轨迹

### 15.4 核心风险指标

1. `d_wall`
2. `d_path`
3. `m = r_local - r_tool`
4. `theta = angle(v_tip, t_path)`
5. `d_pred`
6. `F_contact`
7. `curvature_margin`
8. `kinematic_feasibility`

### 15.5 检测实现方法

#### 15.5.1 距离场法

采用 `vtkImplicitPolyDataDistance` 或预计算 `SDF`。

#### 15.5.2 局部截面法

在中心线局部截面评估偏心率、接近度和局部裕量。

#### 15.5.3 前瞻轨迹法

在 `100~300 ms` 窗口内进行短时轨迹预测，提前触发安全动作。

### 15.6 风险分级与阈值

系统定义四级风险：

- `Level 0`: 安全
- `Level 1`: 预警
- `Level 2`: 危险
- `Level 3`: 紧急

阈值定义：

- `T_warn = 2.0 mm`
- `T_danger = 1.0 mm`
- `T_stop = 0.5 mm`

### 15.7 安全控制策略

#### 15.7.1 告警

四窗口同步高亮风险区域，输出文本与声音提示，并记录事件日志。

#### 15.7.2 限速

```text
v_safe = v_max * alpha(d)
```

状态映射：

- 安全：`1.0`
- 预警：`0.5`
- 危险：`0.2`

系统同时启用：

- `velocity limiter`
- `acceleration limiter`
- `rotation limiter`

#### 15.7.3 紧急制动

满足以下任一条件立即制动：

- `d_wall < T_stop`
- 预测碰撞时间过短
- 力阈值超限
- 偏离过大且继续向外推进
- 尖端方向直指壁面
- 关键节点心跳失联
- 跟踪丢失且状态不可恢复

#### 15.7.4 自动回退

系统在允许条件下执行固定安全距离回退，并切换到低速重规划状态。

#### 15.7.5 安全保持

系统在控制链异常、图像中断、状态估计置信度下降或节点失联时进入 `SAFE HOLD`。
系统在 `getNetAdaptorInfo` 返回链路失效、`getConnectionStatus` 返回异常或 `getSensorConnected` 返回关键传感器离线时，同样进入 `SAFE HOLD`。该路径属于一级硬件安全保护链路。

### 15.8 安全状态机（FSM）

```text
NORMAL -> WARNING -> DANGER -> EMERGENCY_STOP -> RECOVERY / MANUAL_OVERRIDE / SAFE_HOLD
```

### 15.9 前端安全可视化

前端同步显示：

- 尖端点
- 最近壁面点
- 风险区域
- 安全走廊
- 限速/制动状态
- 当前安全等级
- 设备连接状态
- 传感器在线状态
- 标定状态与置信度预警

### 15.10 VLA 安全约束

`VLA` 所有输出必须满足以下约束：

- 不绕过安全监督器
- 不绕过局部规划器
- 不直接写入控制器
- 不触发急停判断
- 不更改安全阈值
- 不执行硬件级控制

### 15.11 后端接口

```python
def evaluate_collision_risk(
    tip_position,
    tip_direction,
    tip_velocity,
    vessel_surface,
    centerline_path
):
    return {
        "distance_to_wall": float,
        "distance_to_path": float,
        "risk_level": str,
        "predicted_collision": bool
    }
```

```python
def emergency_stop(reason: str):
    pass

def safe_retract(step_mm: float):
    pass

def safe_hold(reason: str):
    pass
```

### 15.12 参数与配置

```yaml
safety:
  warn_distance_mm: 2.0
  danger_distance_mm: 1.0
  stop_distance_mm: 0.5
  max_force_warn: 0.15
  max_force_stop: 0.30
  retract_distance_mm: 2.0
  prediction_horizon_ms: 200
  danger_speed_scale: 0.2
  warning_speed_scale: 0.5
  watchdog_timeout_ms: 100
  emergency_stop_latency_ms: 50
```

### 15.13 日志与追溯

系统记录：

- 预警、危险、制动和回退时间
- 触发原因
- 位姿与速度
- 路径点编号
- 是否人工接管
- `VLA` 输出与审核结果
- `LLM` 解释文本
- 节点健康状态

### 15.14 测试要求

- 单元测试：距离场、偏移量、分级逻辑、限幅器
- 仿真测试：直段、狭窄段、高曲率、分叉误入、快速逼近壁面
- 联调测试：告警延迟、制动延迟、限速有效性、回退稳定性
- 故障测试：节点失联、图像中断、跟踪丢失、DDS 异常

### 15.15 设计原则

1. 碰撞检测优先于路径跟踪
2. 预测检测优先于事后检测
3. 安全控制链路独立于路径规划
4. 制动链路尽量短
5. 制动事件全量可追溯
6. 人工接管优先级最高
7. 硬件急停独立于软件链路
8. 高层智能模块不参与安全闭环

### 15.16 小结

系统安全链路定义如下：

```text
位姿感知 -> 风险评估 -> 分级告警 -> 自动限速 -> 紧急制动 -> 回退 / 安全保持 / 人工接管 -> LLM 风险解释
```

## 16. 风险与应对

系统运行风险包括：

- 拓扑断裂与错误桥接
- 配准漂移 `registration drift`
- 术中图像质量波动
- 跟踪器丢失与状态估计失稳
- 实时抖动与控制延迟
- 多数据源坐标不一致
- `sim2real mismatch`
- 通信丢包与节点失联

系统通过拓扑验证、图修复、滤波、时间同步、watchdog、安全保持、硬件急停和全量日志追溯进行控制。

## 17. 结论

本系统已经定义为面向血管介入机器人的工业级导航与控制系统。系统采用术前影像建模、拓扑路径规划、`2D/3D` 配准、状态估计、全局与局部双层规划、安全监督、`ROS2` 实时控制和高层智能辅助相结合的架构。

系统执行链路保持确定性和独立性：

```text
DSA/X-ray
    -> Vessel Segmentation
    -> 2D/3D Registration
    -> State Estimator
    -> Global Planner
    -> Local Planner
    -> Safety Supervisor
    -> MPC / PID Controller
    -> ROS2 Real-time Layer
    -> Robot Hardware
```

`VLA/LLM` 永远位于高层辅助决策层。实时控制与安全链路保持确定性、独立性和可追溯性。系统以可验证、可解释、可接管的医疗机器人规范完成工程定义。
