# API 与通信协议

> 版本：v1.0 | 日期：2026-06-16  
> FastAPI 微服务 + WebSocket 实时通信

---

## 一、WebSocket 通信协议

### 1.1 连接管理

```
端点:  ws://localhost:8000/ws/session
心跳:  服务端每 5s 发送 {"type": "ping"}
       客户端需在 5s 内回复 {"type": "pong"}
超时:  15s 无响应断开连接
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

### 1.4 服务端 → 客户端消息

#### state_update — 状态更新（主循环）

```json
{
    "type": "state_update",
    "session_id": "uuid",
    "timestamp": 1718534400033,
    "data": {
        "tip_position": [12.5, -3.2, 45.1],
        "tip_direction": [0.02, -0.01, 0.99],
        "tip_quaternion": [0.0, 0.01, 0.0, 0.99],
        "velocity": 0.3,
        "wall_distance": 2.5,
        "curvature": 0.05,
        "safety_status": "SAFE_NAV",
        "path_progress": 0.45,
        "contact_force": 0.0,
        "episode_length": 128
    }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `tip_position` | [float×3] | 导丝尖端位置 (LPS) |
| `tip_direction` | [float×3] | 尖端方向单位向量 |
| `tip_quaternion` | [float×4] | 尖端朝向四元数 |
| `velocity` | float | 瞬时速度 (mm/s) |
| `wall_distance` | float | 距血管壁最小距离 (mm) |
| `curvature` | float | 尖端局部曲率 (mm⁻¹) |
| `safety_status` | string | 安全状态枚举 |
| `path_progress` | float | 沿规划路径完成度 [0, 1] |
| `contact_force` | float | 接触力 (N) |
| `episode_length` | int | 当前 episodes 步数 |

#### state_batch — 批量状态（含导丝体渲染数据）

```json
{
    "type": "state_batch",
    "session_id": "uuid",
    "timestamp": 1718534400033,
    "data": {
        "tip": {
            "position": [12.5, -3.2, 45.1],
            "direction": [0.02, -0.01, 0.99]
        },
        "bodies": [
            {"pos": [12.4, -3.2, 45.0], "quat": [0, 0, 0, 1]},
            {"pos": [12.3, -3.1, 44.8], "quat": [0, 0, 0, 1]},
            ...
        ],
        "path": {
            "waypoints": [[...], ...],
            "progress": 0.45
        },
        "safety": {
            "status": "SAFE_NAV",
            "wall_distance": 2.5,
            "curvature": 0.05,
            "speed": 0.3,
            "risk_score": 0.12
        },
        "episode": {
            "length": 128,
            "reward": -0.15
        }
    }
}
```

频率：建议每 2~3 帧 control 对应 1 帧 state_batch（含 84 节 body 数据时消息体较大）

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
curl -X POST http://localhost:8000/api/v1/path/plan \
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
| `COLLISION_STOP` | 409 | 触壁制动 |
| `PATH_NOT_FOUND` | 404 | 路径规划无解 |
| `SESSION_EXPIRED` | 440 | 会话超时 |
| `CATHSIM_ERROR` | 500 | 物理引擎错误 |
| `RATE_LIMIT` | 429 | 频率超限 |
