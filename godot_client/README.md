# CathSim VPP — Godot 客户端（阶段八）

Godot 4.7 渲染客户端，通过 WebSocket 连接 `services` 后端，实时显示血管、导丝，
并接收键盘控制。对应 `doc/01-总体技术方案.md` 的 Godot 渲染交互层。

## 目录结构

```
godot_client/
├── project.godot              # Godot 4.7 项目配置（Forward+，后端 URL）
├── scenes/
│   └── main.tscn              # 主场景（仅根节点 + main_controller.gd）
├── scripts/
│   ├── main_controller.gd     # 代码构建场景并装配所有节点/信号
│   ├── websocket_client.gd    # WebSocket 协议客户端
│   ├── guidewire_renderer.gd  # 导丝尖端 + 导丝体（line strip）渲染
│   ├── path_renderer.gd       # 规划路径折线渲染（state_batch.path.waypoints）
│   ├── hud_controller.gd      # 安全状态灯 + 指标读数
│   └── input_handler.gd       # WASD/R 键盘输入 → 控制命令
└── assets/
    └── models/
        └── blood_vessels.glb  # 血管网格（由工具生成，git 忽略）
```

> 设计取舍：场景采用「最小 `.tscn` + 代码构建」，把节点装配放进 GDScript，
> 降低手写 `.tscn`/`InputMap` 序列化出错的风险。键盘用物理按键轮询，无需自定义 InputMap。

## Sprint 0 XR 零控制入口

- 桌面主入口仍为 `res://scenes/main.tscn`。
- XR 准备入口为 `res://scenes/xr/MainXR.tscn`，由 `scripts/xr/xr_bootstrap.gd` 启动。
- 无 OpenXR Runtime 时，该入口只输出 `controls_blocked=true` 的中性快照；不会创建控制发送链路。
- Aim/Grip 节点只在 Runtime 已初始化后创建，避免 headless/无头显环境实例化 XR 跟踪节点。
- 当前尚无 Vendors Plugin、Action Map、Android 导出或 PICO 真机通过结论，详见 `docs/xr/sprint0-status.md`。

## 前置：生成血管/体模 GLB

Godot 只能导入 glTF/GLB，无法直接读 VTK/STL。

**当前默认渲染真实 VPP `case_001` 血管，并使用 Newton 后端**。完整数据应位于
`data/vpp_assets/case_001`，其中包含原始血管、中心线图、半径、25 条目标路线、
MuJoCo 碰撞网格和派生 GLB：

```powershell
conda run -n cathsim-dev python tools/validate_vpp_assets.py data/vpp_assets/case_001
```
首次运行前可执行 `scripts/validate_godot.ps1`，它会自动寻找 Godot、导入 GLB
并检查脚本运行错误。

VPP 真实血管（case_001，约 10MB）：

```powershell
conda run -n cathsim-dev python tools/export_godot_assets.py --case-id case_001 --quality visual_high
```
输出 `godot_client/assets/models/blood_vessels_visual_high.glb`；原生表面版本使用
`--quality visual_native`。

## VPP 真实血管导航（阶段十）

阶段十后端已支持 VPP 入口对齐 + 服务端路径规划。要在 VPP 血管内导航，
在 `main_controller.gd` 的导出变量（或 `main.tscn` Inspector）设置：

| 变量 | VPP 取值示例 | 说明 |
|------|-------------|------|
| `phantom` | `case_001_vpp` | VPP phantom 名（后端按名自动解析 assets 目录） |
| `target` | `endpoints_1` | 目标 endpoint site 名 |
| `case_id` | `case_001` | 规划用 case |
| `start_position` | `[0.173, -268.24, 291.25]` | 入口 endpoint（**LPS 毫米**） |
| `end_position` | `[-975.65, -217.22, 250.32]` | 目标 endpoint（**LPS 毫米**） |

设置 `start_position`/`end_position` 后，客户端在 `session_start` 携带这两点，
后端自动：规划 A*+B-spline 路径 → 转米制 → 在路径起点生成导丝并对齐血管走向 →
通过 `state_batch.path.waypoints` 回传路径供 `path_renderer` 可视化。

渲染 GLB 按 `phantom` 自动选择：`res://assets/models/<phantom>.glb`，
找不到则 `*_vpp` 回退到 `blood_vessels.glb`，否则 `low_tort.glb`。

留空 `start_position`/`end_position` 即为默认 low_tort 会话（导丝在原点附近生成）。

## 运行

1. 启动后端服务（必须使用安装了项目依赖和 Newton/Warp 的 Python）：

   ```powershell
   # Windows 推荐：自动选择 .venv 或 cathsim-dev，并设置可写的 Warp 缓存
   .\start_backend.bat

   # 或直接使用当前开发环境（默认端口 9000）
   conda run -n cathsim-dev python -m services.main
   ```

   > 默认端口为 **9000**，而非 8000：Windows（Hyper-V/winnat）保留了动态 TCP
   > 端口段 7966–8065，绑定 8000 会报 `[Errno 13]` 访问权限错误。可用环境变量
   > `CATHSIM_PORT` 覆盖（用 `python -m services.main` 启动时生效）。
   > 仿真不渲染像素（`use_pixels=False`），无需设置 `MUJOCO_GL`。

2. 双击 `start_godot.bat`，或用 Godot 4.7 打开 `godot_client/`。启动脚本会在
   PATH、桌面和下载目录中查找 Godot，因此无需把桌面目录加入 PATH。

3. 按 F5 运行。客户端会自动连接 `ws://localhost:9000/ws/session`，
   以 `batch_mode=true` 开启会话（获取导丝 body 渲染数据）。

   后端 URL 可在 `project.godot` 的 `[network] config/server_url` 修改。

## 操作

| 按键 | 作用 |
|------|------|
| `W` / `S` | 推进 / 后退（delta_push ±1） |
| `A` / `D` | 左旋 / 右旋（delta_rotate ∓1 / ±1） |
| `R` | 重置当前 episode |

HUD（左上）显示：安全状态灯（STANDBY/SAFE_NAV/DANGER_WARNING/COLLISION_STOP）、
连接状态、episode 步数、速度、壁面距离、曲率、路径进度、风险分。

## 与后端协议对接

| 方向 | 消息 | 客户端处理 |
|------|------|-----------|
| C→S | `session_start`（含 `batch_mode`） | 连接成功后自动发送 |
| C→S | `control` | WASD 输入，~20Hz |
| C→S | `reset` | R 键 |
| C→S | `path_request` | `send_path_request()`（暂未绑定按键，供后续扩展） |
| C→S | `pong` | 收到 `ping` 时立即回复 |
| S→C | `session_started` | 读取 `session_id` 与 `data.state` |
| S→C | `state_batch` | 更新导丝（tip + bodies）、路径折线与 HUD |
| S→C | `state_update` | 非 batch 模式回退路径 |
| S→C | `path_response` | `path_received` 信号（独立 path_request 用） |

字段契约已用后端 `TestClient` 回放校验（见 `doc/05-开发进度记录.md` 阶段八记录）。

## 已知限制

- **坐标对齐**：导丝节点挂在血管 GLB 场景根下，二者共享同一变换，
  以抵消 Godot/trimesh 的 glTF 轴转换。若实测仍有偏移，
  需将导丝节点变换对齐到血管 MeshInstance3D 节点变换。
- **冷启动卡顿**：后端首次 MuJoCo 初始化会同步阻塞事件循环若干秒；
  期间若客户端不及时回 `pong`，连接会 `PONG_TIMEOUT`。Godot 客户端每帧回 `pong`，
  正常情况无碍；但单步阻塞超过 15s 仍可能断连（后端可改为线程池执行 step 以根治）。
- 未做 X-ray 着色器、双视角、手柄映射——留作后续。
- 本客户端无法在当前 CI/无 GUI 环境中自动化验证，仅通过后端契约回放间接验证。
