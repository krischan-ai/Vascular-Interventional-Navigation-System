# CathSim VPP — Godot 客户端（阶段八）

Godot 4.4 渲染客户端，通过 WebSocket 连接 `services` 后端，实时显示血管、导丝，
并接收键盘控制。对应 `doc/01-总体技术方案.md` 的 Godot 渲染交互层。

## 目录结构

```
godot_client/
├── project.godot              # Godot 4.4 项目配置（Forward+，后端 URL）
├── scenes/
│   └── main.tscn              # 主场景（仅根节点 + main_controller.gd）
├── scripts/
│   ├── main_controller.gd     # 代码构建场景并装配所有节点/信号
│   ├── websocket_client.gd    # WebSocket 协议客户端
│   ├── guidewire_renderer.gd  # 导丝尖端 + 导丝体（line strip）渲染
│   ├── hud_controller.gd      # 安全状态灯 + 指标读数
│   └── input_handler.gd       # WASD/R 键盘输入 → 控制命令
└── assets/
    └── models/
        └── blood_vessels.glb  # 血管网格（由工具生成，git 忽略）
```

> 设计取舍：场景采用「最小 `.tscn` + 代码构建」，把节点装配放进 GDScript，
> 降低手写 `.tscn`/`InputMap` 序列化出错的风险。键盘用物理按键轮询，无需自定义 InputMap。

## 前置：生成血管 GLB

Godot 只能导入 glTF/GLB，无法直接读 VTK/STL。运行以下命令从 `visual.stl`
（已是 MuJoCo 米制，与导丝物理坐标同帧）导出 GLB：

```powershell
python tools/export_godot_assets.py
```

输出 `godot_client/assets/models/blood_vessels.glb`（约 10MB）。

## 运行

1. 启动后端服务：

   ```powershell
   $env:MUJOCO_GL="glfw"
   uvicorn services.main:app --host 0.0.0.0 --port 8000
   ```

2. 用 Godot 4.4 打开 `godot_client/`（首次打开会自动导入 GLB 并生成 `.godot/` 缓存）。

3. 按 F5 运行。客户端会自动连接 `ws://localhost:8000/ws/session`，
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
| S→C | `state_batch` | 更新导丝（tip + bodies）与 HUD |
| S→C | `state_update` | 非 batch 模式回退路径 |
| S→C | `path_response` | `path_received` 信号（供后续路径可视化） |

字段契约已用后端 `TestClient` 回放校验（见 `doc/05-开发进度记录.md` 阶段八记录）。

## 已知限制

- **坐标对齐**：导丝节点挂在血管 GLB 场景根下，二者共享同一变换，
  以抵消 Godot/trimesh 的 glTF 轴转换。若实测仍有偏移，
  需将导丝节点变换对齐到血管 MeshInstance3D 节点变换。
- **冷启动卡顿**：后端首次 MuJoCo 初始化会同步阻塞事件循环若干秒；
  期间若客户端不及时回 `pong`，连接会 `PONG_TIMEOUT`。Godot 客户端每帧回 `pong`，
  正常情况无碍；但单步阻塞超过 15s 仍可能断连（后端可改为线程池执行 step 以根治）。
- 本阶段未做 X-ray 着色器、双视角、手柄映射、路径线可视化——留作后续。
- 本客户端无法在当前 CI/无 GUI 环境中自动化验证，仅通过后端契约回放间接验证。
