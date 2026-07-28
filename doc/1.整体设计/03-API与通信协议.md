# API 与通信协议

> 版本：v1.0 | 日期：2026-06-16  
> FastAPI 微服务 + WebSocket 实时通信

---

## 一、WebSocket 通信协议

### 1.1 连接管理

```
端点:  ws://localhost:9000/ws/session
心跳:  服务端每 5s 发送 {"type": "ping"}
       客户端收到后回复 {"type": "pong"}
超时:  45s 未收到 pong 断开连接（为物理引擎冷启动保留余量）
```

### 1.2 消息基类

```json
{
    "type": "message_type",
    "session_id": "uuid",
    "timestamp": 1718534400000,
    "data": {}
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | 消息类型标识 |
| `session_id` | string | 会话 UUID |
| `timestamp` | int | Unix 毫秒时间戳 |
| `data` | object | 消息体 |

### 1.3 客户端 → 服务端消息

#### control — 控制命令

```json
{
    "type": "control",
    "session_id": "uuid",
    "timestamp": 1718534400000,
    "data": {
        "delta_push": 0.5,
        "delta_rotate": 0.1
    }
}
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `delta_push` | float | [-1.0, 1.0] | 推进增量，正=前进，负=后退 |
| `delta_rotate` | float | [-1.0, 1.0] | 旋转增量，正=右旋，负=左旋 |

频率：最高 30Hz（建议 15~20Hz 以避免过载）

#### session_start — 开始会话

```json
{
    "type": "session_start",
    "data": {
        "phantom": "low_tort",
        "target": "bca"
    }
}
```

#### session_stop — 停止会话

```json
{
    "type": "session_stop",
    "data": {}
}
```

#### path_request — 路径规划请求

```json
{
    "type": "path_request",
    "data": {
        "start_position": [-1.49, -268.46, 290.42],
        "end_position": [12.34, -256.78, 301.23],
        "algorithm": "astar",
        "smooth": true,
        "constraints": {
            "max_curvature": 0.1,
            "min_wall_distance": 1.0
        }
    }
}
```

#### reset — 环境重置

```json
{
    "type": "reset",
    "data": {
        "randomize": false
    }
}
```

#### emergency_stop — 急停请求

```json
{
    "type": "emergency_stop",
    "session_id": "uuid",
    "data": {"reason": "operator_hud"}
}
```

服务端收到后锁存当前会话的急停状态、解除 ShapeIntent 自动导航，并拒绝后续所有
非零 `control`。前端在收到 `emergency_stop_confirmed` 前只能显示“急停请求中”，
不能提前显示“已急停”。重复发送具有幂等语义。

#### resume — 解除急停

```json
{
    "type": "resume",
    "session_id": "uuid",
    "data": {}
}
```

`resume` 只解除服务端锁存，不会自动恢复 ShapeIntent 或重放旧控制。前端收到
`resume_confirmed` 后才重新开放键盘、鼠标和运动按钮。

#### control_config — 控制保护配置

```json
{
    "type": "control_config",
    "session_id": "uuid",
    "data": {
        "jtip_assist_enabled": true,
        "torque_limit_enabled": true,
        "withdrawal_protection_enabled": true,
        "auto_stop_push_enabled": true
    }
}
```

服务端以同名 `control_config` 消息返回生效配置。J-tip 仅在物理后端支持实时参数时
可用；其余三项分别对应旋转指令限幅、入口回撤阻止和安全制动后的继续推进阻止。

### 1.4 服务端 → 客户端消息

#### navigation_visual_v3 — 统一导航状态契约

`state_update` 与 `state_batch` 从 v3 起共享同一组顶层状态字段。客户端读取
导航数值时统一使用 `data.<field>`，不再分别从 `path`、`safety`、`episode`
推断。`state_batch` 的嵌套块继续保留，用于渲染和 v2 客户端兼容。

契约规则：

- `schema_version` 固定为 `navigation_visual_v3`。
- 距离、位置、速度使用 SI 单位：m、m/s；曲率使用 m⁻¹；力使用 N。
- 后端当前帧发送 `data_status: "fresh"`；客户端断连或超时后自行转为
  `stale`，尚无来源的字段使用 `null` 或 `unknown`，不得填入假 0。
- 安全展示只使用 `data.safety`；`safety_status` 是保留的状态枚举别名。
- `source` 和 `timestamp_ms` 用于来源审计与过期判断。

两类消息共有的主要字段：

| 字段 | 类型 | 单位/说明 |
|------|------|-----------|
| `schema_version` | string | `navigation_visual_v3` |
| `timestamp_ms` | int | 状态生成时间，Unix ms |
| `data_status` | string | `fresh\|stale\|unknown` |
| `source` | string | 当前为 `navigation_engine` |
| `tip_position` | [float×3] | m，后端仿真坐标系 |
| `tip_direction` | [float×3] | 单位向量 |
| `tip_quaternion` | [float×4] | `[x,y,z,w]` |
| `velocity` | float | m/s |
| `contact_force` | float | N，真实物理/引擎状态值 |
| `wall_distance` | float | m |
| `curvature` | float | m⁻¹ |
| `path_progress` | float | `[0,1]` |
| `path_deviation` | float | m |
| `remaining_distance` | float | m |
| `path_total_distance` | float/null | m，规划路径总弧长 |
| `path_travelled_distance` | float/null | m，已经沿规划路径行进的弧长 |
| `vessel_radius` | float/null | m；无路径半径来源时为 null |
| `eta_seconds` | float/null | 静止且无法估算时为 null |
| `latency_ms` | float/null | 无测量来源时为 null |
| `safety_status` | string | `STANDBY\|SAFE_NAV\|DANGER_WARNING\|COLLISION_STOP` |
| `risk_score` | float | `[0,1]` |
| `risk_regions` | array | 仅允许来源可信的空间风险区域 |
| `episode_length` | int | 当前 episode 步数 |
| `reward` | float | 最近一步 reward |
| `done` | bool | episode 是否终止 |
| `safety` | object | 权威安全展示对象，见下表 |

`data.safety`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | 后端安全状态 |
| `safety_level` | string | `safe\|warning\|danger\|stop\|stale` |
| `risk_score` | float | 聚合风险分数 |
| `reason_codes` | array[string] | 真实风险评估原因码 |
| `source` | string | 当前为 `navigation_engine.risk_assessor` |
| `timestamp_ms` | int | 安全状态生成时间 |
| `data_status` | string | 当前帧为 `fresh` |
| `stop_required` | bool | 是否要求停止 |
| `contact_force` | float | N |
| `wall_distance` | float | m |
| `curvature` | float | m⁻¹ |
| `speed` | float | m/s |
| `risk_regions` | array | 来源可信的空间风险区域 |

#### state_update — 轻量状态更新

```json
{
    "type": "state_update",
    "session_id": "uuid",
    "timestamp": 1718534400033,
    "data": {
        "schema_version": "navigation_visual_v3",
        "timestamp_ms": 1718534400033,
        "data_status": "fresh",
        "source": "navigation_engine",
        "tip_position": [0.0125, -0.0032, 0.0451],
        "tip_direction": [0.02, -0.01, 0.99],
        "tip_quaternion": [0.0, 0.01, 0.0, 0.99],
        "velocity": 0.0003,
        "contact_force": 0.12,
        "wall_distance": 0.0025,
        "curvature": 50.0,
        "safety_status": "SAFE_NAV",
        "path_progress": 0.45,
        "path_deviation": 0.0004,
        "remaining_distance": 0.0837,
        "path_total_distance": 0.156,
        "path_travelled_distance": 0.0723,
        "vessel_radius": 0.0032,
        "eta_seconds": 279.0,
        "latency_ms": null,
        "risk_score": 0.12,
        "risk_regions": [],
        "episode_length": 128,
        "reward": -0.15,
        "done": false,
        "safety": {
            "status": "SAFE_NAV",
            "safety_level": "safe",
            "risk_score": 0.12,
            "reason_codes": [],
            "source": "navigation_engine.risk_assessor",
            "timestamp_ms": 1718534400033,
            "data_status": "fresh",
            "stop_required": false,
            "contact_force": 0.12,
            "wall_distance": 0.0025,
            "curvature": 50.0,
            "speed": 0.0003,
            "risk_regions": []
        }
    }
}
```

#### state_batch — 批量状态（含导丝体渲染数据）

```json
{
    "type": "state_batch",
    "session_id": "uuid",
    "timestamp": 1718534400033,
    "data": {
        "schema_version": "navigation_visual_v3",
        "timestamp_ms": 1718534400033,
        "data_status": "fresh",
        "source": "navigation_engine",
        "contact_force": 0.12,
        "wall_distance": 0.0025,
        "curvature": 50.0,
        "velocity": 0.0003,
        "path_progress": 0.45,
        "path_deviation": 0.0004,
        "remaining_distance": 0.0837,
        "path_total_distance": 0.156,
        "path_travelled_distance": 0.0723,
        "vessel_radius": 0.0032,
        "eta_seconds": 279.0,
        "safety_status": "SAFE_NAV",
        "risk_score": 0.12,
        "tip": {
            "position": [0.0125, -0.0032, 0.0451],
            "direction": [0.02, -0.01, 0.99]
        },
        "bodies": [
            {"pos": [0.0124, -0.0032, 0.0450], "quat": [0, 0, 0, 1]},
            {"pos": [0.0123, -0.0031, 0.0448], "quat": [0, 0, 0, 1]},
            ...
        ],
        "path": {
            "waypoints": [[...], ...],
            "progress": 0.45,
            "deviation": 0.0004,
            "remaining_distance": 0.0837,
            "total_distance": 0.156,
            "travelled_distance": 0.0723,
            "vessel_radius": 0.0032,
            "eta_seconds": 279.0
        },
        "safety": {
            "status": "SAFE_NAV",
            "safety_level": "safe",
            "risk_score": 0.12,
            "reason_codes": [],
            "source": "navigation_engine.risk_assessor",
            "timestamp_ms": 1718534400033,
            "data_status": "fresh",
            "stop_required": false,
            "contact_force": 0.12,
            "wall_distance": 0.0025,
            "curvature": 50.0,
            "speed": 0.0003,
            "risk_regions": []
        },
        "episode": {
            "length": 128,
            "reward": -0.15
        }
    }
}
```

频率：建议每 2~3 帧 control 对应 1 帧 state_batch（含 84 节 body 数据时消息体较大）

v2 → v3 迁移期内，`path/safety/episode` 嵌套字段不会删除；新代码必须优先读取
统一顶层字段和权威 `data.safety`。Godot 客户端包含 legacy 适配器，使旧后端缺失的
字段保持 `null/unknown`，而不是伪造 0。

`session_started.state`、`session_started.initial_batch`、后续 `state_update/state_batch`
都包含 `control_state`，用于同步服务端锁存和保护配置。

#### emergency_stop_confirmed / resume_confirmed

```json
{
    "type": "emergency_stop_confirmed",
    "session_id": "uuid",
    "data": {
        "status": "latched",
        "control_state": {
            "emergency_stop_latched": true,
            "protections": {},
            "last_applied_control": {}
        }
    }
}
```

恢复确认结构相同，`type` 为 `resume_confirmed`、`status` 为 `resumed`，且
`emergency_stop_latched=false`。

#### control_rejected — 控制拒绝

```json
{
    "type": "control_rejected",
    "session_id": "uuid",
    "data": {
        "reason": "EMERGENCY_STOP_LATCHED",
        "message": "Emergency stop is latched; non-zero control was rejected",
        "control_state": {"emergency_stop_latched": true}
    }
}
```

急停锁存期间的非零控制、启用 ShapeIntent、切换分支和重置均返回该消息。

#### path_response — 路径规划响应

```json
{
    "type": "path_response",
    "data": {
        "path_id": "uuid",
        "waypoints": [[x1,y1,z1], [x2,y2,z2], ...],
        "smooth_waypoints": [[x1,y1,z1], ...],
        "length_mm": 1522.8,
        "smooth_length_mm": 1503.5,
        "max_curvature": 0.08,
        "node_count": 1451,
        "compute_time_ms": 45.2
    }
}
```

#### error — 错误响应

```json
{
    "type": "error",
    "data": {
        "code": "COLLISION_STOP",
        "message": "Collision detected at wall_distance=0.3mm. Brake engaged."
    }
}
```

### 1.5 频率控制

| 消息类型 | 方向 | 推荐频率 | 说明 |
|---------|------|---------|------|
| `control` | C→S | 15~30 Hz | 手柄输入（~33-66ms 间隔） |
| `emergency_stop/resume` | C→S→C | 按需 | 后端锁存/解除后返回确认 |
| `control_config` | C→S→C | 按需 | 设置并回显控制保护配置 |
| `control_rejected` | S→C | 事件触发 | 急停锁存等原因拒绝控制 |
| `state_update` | S→C | 15~30 Hz | 基础状态同步 |
| `state_batch` | S→C | 10~15 Hz | 含 render data 的完整状态 |
| `path_request/response` | C→S→C | 按需 | 路径规划通常 < 50ms |
| `reset` | C→S | 按需 | 环境复位 |
| `ping/pong` | 双向 | 每 5s | 心跳保活 |

---

## 二、REST API

### 2.1 路径规划

**GET /api/v1/path/plan**

```bash
curl -X POST http://localhost:9000/api/v1/path/plan \
  -H "Content-Type: application/json" \
  -d '{
    "centerline_id": "vpp_001",
    "start": [-1.49, -268.46, 290.42],
    "end": [12.34, -256.78, 301.23],
    "algorithm": "astar",
    "smooth": true
  }'
```

**响应**：

```json
{
    "path_id": "uuid",
    "waypoints": [[...], ...],
    "length_mm": 1522.8,
    "max_curvature": 0.08,
    "node_count": 1451,
    "compute_time_ms": 45.2
}
```

### 2.2 健康检查

**GET /api/v1/health**

```json
{
    "status": "ok",
    "version": "1.0.0",
    "cathsim_ready": true,
    "uptime_seconds": 3600
}
```

---

## 三、Python 内部接口

### 3.1 导航引擎接口

```python
# cathsim_bridge/navigation_engine.py

class NavigationEngine:
    def __init__(self, phantom: str = "low_tort", target: str = "bca"):
        """初始化 CathSim 环境 + 路径规划器"""

    def step(self, delta_push: float, delta_rotate: float) -> dict:
        """执行一步仿真
        Args:
            delta_push: [-1.0, 1.0] 推进力系数
            delta_rotate: [-1.0, 1.0] 旋转力系数
        Returns:
            state_dict: 完整状态字典
        """

    def reset(self, randomize: bool = False) -> dict:
        """重置环境"""

    def plan_path(self, start: list, end: list, **kwargs) -> dict:
        """调用 A* 路径规划"""

    def get_path_progress(self, position: list) -> float:
        """计算当前位置的路径完成度"""

    def assess_risk(self, state: dict) -> dict:
        """计算风险评分"""
```

### 3.2 路径规划器接口

```python
# cathsim_bridge/path_planner.py

class PathPlanner:
    def __init__(self, graph_path: str):
        """加载 VPP 图结构"""

    def plan(self, start: tuple, end: tuple,
             algorithm: str = "astar",
             constraints: dict = None) -> PathResult:
        """路径规划"""

    def smooth(self, waypoints: list,
               max_curvature: float = 0.1) -> list:
        """B-spline 平滑"""

    def find_nearest_node(self, position: tuple) -> int:
        """KDTree 查找最近节点"""
```

### 3.3 风险评估接口

```python
# cathsim_bridge/risk_assessor.py

class RiskAssessor:
    def __init__(self):
        self.weights = {
            "wall_distance": 0.4,
            "curvature": 0.3,
            "velocity": 0.2,
            "deviation": 0.1
        }
        self.thresholds = {
            "wall_distance": {"safe": 1.0, "warning": 0.5, "critical": 0.3},
            "curvature": {"safe": 0.10, "warning": 0.15, "critical": 0.20},
            "velocity": {"safe": 5.0, "warning": 8.0, "critical": 10.0},
            "deviation": {"safe": 1.0, "warning": 2.0, "critical": 3.0}
        }

    def assess(self, state: dict, planned_path: list = None) -> dict:
        """综合风险评估"""
```

---

## 四、错误码

| 错误码 | HTTP WS | 说明 |
|--------|---------|------|
| `INVALID_CONTROL` | 400 | 控制命令参数越界 |
| `EMERGENCY_STOP_LATCHED` | WS | 急停锁存期间拒绝非零控制或自动导航 |
| `COLLISION_STOP` | 409 | 触壁制动 |
| `PATH_NOT_FOUND` | 404 | 路径规划无解 |
| `SESSION_EXPIRED` | 440 | 会话超时 |
| `CATHSIM_ERROR` | 500 | 物理引擎错误 |
| `RATE_LIMIT` | 429 | 频率超限 |
