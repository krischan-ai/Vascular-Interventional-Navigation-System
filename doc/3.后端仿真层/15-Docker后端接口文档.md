# Docker 后端接口文档

> 接口实现版本：0.1.0  
> 文档日期：2026-07-27  
> 服务实现：FastAPI + WebSocket  
> 默认宿主机端口：19000  
> 容器内部端口：9000

## 1. 文档范围

本文档描述 CathSim Docker 后端对前端、测试程序和其他服务开放的 HTTP
REST API 与 WebSocket 实时通信接口。

完整服务端镜像和纯后端仿真镜像使用同一套 FastAPI 接口：

| 镜像 | 用途 |
|---|---|
| `swr.cn-east-3.myhuaweicloud.com/siat/cathsim-server:v1.0` | 后端、仿真及强化学习环境 |
| `swr.cn-east-3.myhuaweicloud.com/siat/cathsim-simulation:v1.0` | 纯后端仿真，不包含强化学习框架 |

强化学习镜像增加训练依赖，但不会额外开放 HTTP 端口。两种镜像的 REST 和
WebSocket 协议保持一致。

## 2. 服务地址与端口

当前部署将宿主机的 `19000` 端口映射到容器内部的 `9000`：

```text
<服务器IP>:19000 -> cathsim-server:9000
```

接口地址如下：

| 类型 | 地址 |
|---|---|
| REST 基础地址 | `http://<服务器IP>:19000` |
| WebSocket 地址 | `ws://<服务器IP>:19000/ws/session` |
| Swagger UI | `http://<服务器IP>:19000/docs` |
| ReDoc | `http://<服务器IP>:19000/redoc` |
| OpenAPI JSON | `http://<服务器IP>:19000/openapi.json` |

如果通过 HTTPS 反向代理发布服务，应分别使用：

```text
https://<服务器IP或域名>/api/v1/...
wss://<服务器IP或域名>/ws/session
```

如果修改了 Compose 中的 `CATHSIM_HOST_PORT`，应将本文档示例中的 `19000`
替换为实际宿主机端口。容器内部监听端口仍为 `9000`。

## 3. 通用约定

### 3.1 数据格式

- REST 请求体和响应体使用 `application/json`。
- WebSocket 消息使用 JSON 文本帧。
- 时间戳使用 Unix 毫秒时间戳。
- 当前接口没有身份鉴权。
- 请求模型校验失败时，REST 返回 HTTP `422`。
- 当前后端未配置 CORS 中间件。浏览器前端跨域访问时，应通过同源反向代理，
  或在服务端增加明确的 CORS 白名单。

### 3.2 坐标与单位

接口中存在两类坐标，调用方不可混用：

| 场景 | 坐标系与单位 |
|---|---|
| 路径规划请求和响应 | LPS，毫米 |
| 仿真状态、导丝位置、显式 `planned_path` | 仿真坐标，米 |
| 速度 | 米/秒 |
| 接触力 | 牛顿 |
| 曲率 | 米的负一次方 |

`POST /api/v1/path/plan` 和 WebSocket `path_request` 接收 LPS 毫米坐标。
服务端为仿真会话规划路径时，会将规划结果从毫米转换为米。

## 4. REST API 总览

| 方法 | 路径 | 功能 |
|---|---|---|
| `GET` | `/api/v1/health` | 健康检查 |
| `GET` | `/api/v1/assets/cases` | 获取 VPP 病例列表 |
| `GET` | `/api/v1/assets/cases/{case_id}` | 获取病例 manifest |
| `POST` | `/api/v1/path/plan` | 执行 A* 路径规划 |
| `POST` | `/api/v1/session/start` | 创建仿真会话 |
| `GET` | `/api/v1/session` | 列出活动会话 |
| `GET` | `/api/v1/session/{session_id}` | 获取会话元数据 |
| `POST` | `/api/v1/session/{session_id}/step` | 执行一步仿真 |
| `POST` | `/api/v1/session/{session_id}/reset` | 重置会话 |
| `DELETE` | `/api/v1/session/{session_id}` | 关闭并释放会话 |

## 5. REST API 详细说明

### 5.1 健康检查

```http
GET /api/v1/health
```

调用示例：

```bash
curl http://<服务器IP>:19000/api/v1/health
```

响应示例：

```json
{
  "status": "ok",
  "version": "0.1.0",
  "vpp_ready": true,
  "cases": ["case_001"]
}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `status` | string | 服务状态，正常时为 `ok` |
| `version` | string | API 实现版本 |
| `vpp_ready` | boolean | 是否发现可用 VPP 病例 |
| `cases` | string[] | 可用病例 ID |

### 5.2 获取病例列表

```http
GET /api/v1/assets/cases
```

调用示例：

```bash
curl http://<服务器IP>:19000/api/v1/assets/cases
```

响应示例：

```json
[
  {
    "case_id": "case_001",
    "coordinate_system": "LPS",
    "unit": "mm",
    "graph_nodes": 1000,
    "graph_edges": 999,
    "endpoints": 24
  }
]
```

病例数量及图统计值由实际部署的数据决定。

### 5.3 获取病例 Manifest

```http
GET /api/v1/assets/cases/{case_id}
```

调用示例：

```bash
curl http://<服务器IP>:19000/api/v1/assets/cases/case_001
```

响应结构：

```json
{
  "case_id": "case_001",
  "manifest": {}
}
```

病例不存在时返回 HTTP `404`。

### 5.4 路径规划

```http
POST /api/v1/path/plan
Content-Type: application/json
```

请求示例：

```bash
curl -X POST http://<服务器IP>:19000/api/v1/path/plan \
  -H "Content-Type: application/json" \
  -d '{
    "case_id": "case_001",
    "start": [-1.49, -268.46, 290.42],
    "end": [12.34, -256.78, 301.23],
    "algorithm": "astar",
    "smooth": true,
    "smooth_factor": 0.5
  }'
```

请求字段：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `case_id` | string | 否 | `case_001` | 病例 ID |
| `start` | float[3] | 是 | - | LPS 起点，单位毫米 |
| `end` | float[3] | 是 | - | LPS 终点，单位毫米 |
| `algorithm` | string | 否 | `astar` | 当前只支持 `astar` |
| `smooth` | boolean | 否 | `false` | 是否执行 B 样条平滑 |
| `smooth_factor` | float | 否 | `0.5` | 平滑强度，仅在 `smooth=true` 时生效 |

响应示例：

```json
{
  "path_id": "6adbd030-0f5d-4eb1-a2a9-a50ae9b17921",
  "case_id": "case_001",
  "coordinate_system": "LPS",
  "unit": "mm",
  "waypoints": [[-1.49, -268.46, 290.42]],
  "smooth_waypoints": [[-1.49, -268.46, 290.42]],
  "length_mm": 1522.8,
  "smooth_length_mm": 1503.5,
  "max_curvature": 0.08,
  "node_count": 1451,
  "compute_time_ms": 45.2
}
```

病例、图节点或路径不存在时返回 HTTP `404`。

### 5.5 创建仿真会话

```http
POST /api/v1/session/start
Content-Type: application/json
```

请求示例：

```bash
curl -X POST http://<服务器IP>:19000/api/v1/session/start \
  -H "Content-Type: application/json" \
  -d '{
    "phantom": "low_tort",
    "target": "bca",
    "use_pixels": false
  }'
```

请求字段：

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `phantom` | string | `low_tort` | 仿真血管模型 |
| `target` | string | `bca` | 导航目标 |
| `use_pixels` | boolean | `false` | 是否生成像素观测 |

响应示例：

```json
{
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "phantom": "low_tort",
  "target": "bca",
  "state": {
    "tip_position": [0.0, 0.0, 0.0],
    "tip_direction": [0.0, 0.0, 1.0],
    "tip_quaternion": [0.0, 0.0, 0.0, 1.0],
    "velocity": 0.0,
    "contact_force": 0.0,
    "wall_distance": 0.0,
    "curvature": 0.0,
    "episode_length": 0,
    "target_position": [0.0, 0.0, 0.0],
    "path_progress": 0.0,
    "path_deviation": 0.0,
    "remaining_distance": 0.0,
    "vessel_radius": null,
    "eta_seconds": null,
    "latency_ms": null,
    "fidelity_mode": "physics",
    "risk_score": 0.0,
    "risk_regions": [],
    "flow_guidance": {},
    "reward": 0.0,
    "done": false,
    "safety_status": "STANDBY"
  }
}
```

后端初始化失败时返回 HTTP `503`。

### 5.6 列出活动会话

```http
GET /api/v1/session
```

调用示例：

```bash
curl http://<服务器IP>:19000/api/v1/session
```

响应示例：

```json
[
  {
    "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
    "phantom": "low_tort",
    "target": "bca",
    "created_at": "2026-07-27T10:00:00+00:00",
    "last_active": "2026-07-27T10:00:05+00:00",
    "episode_count": 1,
    "total_steps": 128
  }
]
```

### 5.7 获取会话信息

```http
GET /api/v1/session/{session_id}
```

调用示例：

```bash
curl http://<服务器IP>:19000/api/v1/session/<session_id>
```

会话不存在时返回 HTTP `404`。

### 5.8 执行一步仿真

```http
POST /api/v1/session/{session_id}/step
Content-Type: application/json
```

调用示例：

```bash
curl -X POST http://<服务器IP>:19000/api/v1/session/<session_id>/step \
  -H "Content-Type: application/json" \
  -d '{
    "delta_push": 0.5,
    "delta_rotate": 0.1,
    "microcatheter_advance": 0.0
  }'
```

请求字段：

| 字段 | 类型 | 必填 | 范围 | 说明 |
|---|---|---|---|---|
| `delta_push` | float | 是 | `[-1, 1]` | 正值前进，负值后退 |
| `delta_rotate` | float | 是 | `[-1, 1]` | 正值顺时针，负值逆时针 |
| `microcatheter_advance` | float | 否 | `[-1, 1]` | 支撑器械推进量，默认 `0` |

响应包含：

```json
{
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "state": {},
  "step_count": 129
}
```

会话不存在时返回 HTTP `404`；仿真步执行失败时返回 HTTP `400`。

### 5.9 重置会话

```http
POST /api/v1/session/{session_id}/reset
```

调用示例：

```bash
curl -X POST http://<服务器IP>:19000/api/v1/session/<session_id>/reset
```

响应包含重置后的 `state` 和递增后的 `episode_count`。会话不存在时返回
HTTP `404`。

### 5.10 关闭会话

```http
DELETE /api/v1/session/{session_id}
```

调用示例：

```bash
curl -X DELETE http://<服务器IP>:19000/api/v1/session/<session_id>
```

成功响应：

```json
{
  "status": "closed",
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71"
}
```

## 6. 仿真状态字段

REST 的 `state` 与 WebSocket 的 `state_update.data` 使用同一核心状态结构：

| 字段 | 类型 | 说明 |
|---|---|---|
| `tip_position` | float[3] | 导丝尖端位置，仿真坐标，米 |
| `tip_direction` | float[3] | 尖端方向单位向量 |
| `tip_quaternion` | float[4] | 四元数 `[x,y,z,w]` |
| `velocity` | float | 尖端速度，米/秒 |
| `contact_force` | float | 接触力，牛顿 |
| `wall_distance` | float | 到血管壁的最小距离，米 |
| `curvature` | float | 局部曲率，米的负一次方 |
| `episode_length` | integer | 当前 episode 步数 |
| `target_position` | float[3] | 目标位置，仿真坐标 |
| `path_progress` | float | 路径进度，范围 `[0,1]` |
| `path_deviation` | float | 相对路径偏差，米 |
| `remaining_distance` | float | 剩余路径弧长，米 |
| `vessel_radius` | float/null | 当前局部管腔半径，米 |
| `eta_seconds` | float/null | 预计到达时间，秒 |
| `latency_ms` | float/null | 最近测得的通信延迟，毫秒 |
| `fidelity_mode` | string | `guided`、`physics` 或 `rl` |
| `risk_score` | float | 综合风险分数，范围 `[0,1]` |
| `risk_regions` | object[] | 空间风险区域 |
| `flow_guidance` | object | 大曲率流程指导数据 |
| `reward` | float | 最近一步奖励 |
| `done` | boolean | 当前 episode 是否结束 |
| `safety_status` | string | 安全状态 |

`safety_status` 的合法值为：

```text
STANDBY
SAFE_NAV
DANGER_WARNING
COLLISION_STOP
```

## 7. WebSocket 协议

### 7.1 建立连接

```text
ws://<服务器IP>:19000/ws/session
```

浏览器示例：

```javascript
const socket = new WebSocket("ws://<服务器IP>:19000/ws/session");

socket.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log(message);
};
```

### 7.2 消息基类

客户端和服务端消息均使用以下外层结构：

```json
{
  "type": "message_type",
  "session_id": "uuid-or-null",
  "timestamp": 1785123456789,
  "data": {}
}
```

客户端发送消息时，`session_id` 和 `timestamp` 可以省略。服务端响应会填充
时间戳；会话创建后会填充 `session_id`。

### 7.3 心跳与频率限制

- 服务端每 `5` 秒发送一次 `ping`。
- 客户端收到 `ping` 后应立即发送 `pong`。
- 连续 `45` 秒未收到 `pong`，服务端可关闭连接。
- `control` 消息最高处理频率约为 `30 Hz`。
- 超过频率的控制帧会被丢弃，不返回错误。

心跳响应：

```json
{
  "type": "pong",
  "data": {}
}
```

### 7.4 客户端消息类型

| `type` | 是否需要活动会话 | 功能 |
|---|---:|---|
| `session_start` | 否 | 创建并绑定会话 |
| `session_stop` | 是 | 关闭当前连接绑定的会话 |
| `control` | 是 | 推进、旋转及微导管控制 |
| `path_request` | 否 | 单独请求路径规划 |
| `select_route` | 是 | 切换多分支血管的目标路线 |
| `device_config` | 是 | 配置导丝尖端形态 |
| `engine_params` | 是 | 在线调整物理引擎参数 |
| `shape_intent` | 是 | 启用或更新 ShapeIntent 自动控制 |
| `reset` | 是 | 重置当前会话 |
| `pong` | 否 | 响应服务端心跳 |

### 7.5 `session_start`

基础示例：

```json
{
  "type": "session_start",
  "data": {
    "phantom": "aorta_tree",
    "target": "endpoint_0",
    "use_pixels": false,
    "batch_mode": true,
    "n_bodies": 80,
    "physics_engine": "newton",
    "route_target": "endpoint_0"
  }
}
```

完整参数：

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `phantom` | string | `low_tort` | 血管模型 |
| `target` | string | `bca` | 导航目标 |
| `use_pixels` | boolean | `false` | 是否生成像素观测 |
| `batch_mode` | boolean | `false` | 是否返回包含导丝体渲染数据的 `state_batch` |
| `n_bodies` | integer | `80` | 导丝离散体数量 |
| `n_substeps` | integer/null | `null` | 物理子步数 |
| `case_id` | string | `case_001` | 自动规划使用的病例 |
| `start_position` | float[3]/null | `null` | LPS 起点，毫米 |
| `end_position` | float[3]/null | `null` | LPS 终点，毫米 |
| `smooth` | boolean | `true` | 自动规划时是否平滑 |
| `smooth_factor` | float | `0.5` | 路径平滑强度 |
| `planned_path` | float[][3]/null | `null` | 显式仿真路径，单位米 |
| `guided` | boolean | `false` | 是否请求引导模式 |
| `physics_engine` | string | `auto` | 物理引擎选择 |
| `route_target` | string/null | `null` | 多分支血管初始路线 ID |

`physics_engine` 支持：

```text
auto
guided
mujoco
physics
newton
newton_demo
```

服务端成功创建会话后返回 `session_started`：

```json
{
  "type": "session_started",
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "timestamp": 1785123456789,
  "data": {
    "phantom": "aorta_tree",
    "target": "endpoint_0",
    "guided": false,
    "physics_engine": "newton",
    "engine": "NewtonPhysicsEngine",
    "fidelity_mode": "physics",
    "state": {},
    "initial_batch": {},
    "routes": []
  }
}
```

`initial_batch` 仅在 `batch_mode=true` 时返回对象，否则为 `null`。

### 7.6 `control`

```json
{
  "type": "control",
  "data": {
    "delta_push": 0.5,
    "delta_rotate": 0.1,
    "microcatheter_advance": 0.0
  }
}
```

三个控制值范围均为 `[-1,1]`。普通模式返回 `state_update`，批量模式返回
`state_batch`。

### 7.7 `path_request`

该消息不要求先创建会话：

```json
{
  "type": "path_request",
  "data": {
    "case_id": "case_001",
    "start_position": [-1.49, -268.46, 290.42],
    "end_position": [12.34, -256.78, 301.23],
    "algorithm": "astar",
    "smooth": true,
    "smooth_factor": 0.5
  }
}
```

成功后返回 `path_response`，其 `data` 与 REST 路径规划结果结构一致。

### 7.8 `select_route`

用于 `aorta_tree` 等多分支血管运行时切换目标：

```json
{
  "type": "select_route",
  "data": {
    "target": "endpoint_24"
  }
}
```

切换路线后服务端重置会话，并在后续批量状态中重新发送路径。

### 7.9 `device_config`

```json
{
  "type": "device_config",
  "data": {
    "guidewire": {
      "tip_shape": "j_tip",
      "tip_curve_angle_deg": 45.0,
      "tip_length_mm": 20.0,
      "soft_tip_length_mm": 40.0,
      "tip_stiffness": 0.25
    }
  }
}
```

`tip_shape` 支持：

```text
straight
angled
j_tip
j
c_shape
hook
```

服务端会将语义配置映射到当前物理引擎，并通过同类型 `device_config` 消息返回
实际生效值。

### 7.10 `engine_params`

该接口使用扁平键值对象在线调整当前物理引擎：

```json
{
  "type": "engine_params",
  "data": {
    "bend": 0.5,
    "tip_bend": 0.7,
    "soft_tip": 0.4,
    "push_speed": 0.05,
    "rotate_speed": 1.0,
    "jtip_deg": 45.0
  }
}
```

具体支持参数由当前物理引擎决定。服务端返回：

```json
{
  "type": "engine_params",
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "data": {
    "effective": {}
  }
}
```

`effective` 为物理引擎裁剪和校验后的实际参数。

### 7.11 `shape_intent`

```json
{
  "type": "shape_intent",
  "data": {
    "active": true,
    "target_waypoint": [0.01, 0.02, 0.03],
    "target_direction": null,
    "intensity": 0.8
  }
}
```

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `active` | boolean | `true` | 启用或关闭 ShapeIntent |
| `target_waypoint` | float[3]/null | `null` | 目标点，仿真坐标 |
| `target_direction` | float[3]/null | `null` | 目标方向 |
| `intensity` | float | `1.0` | 控制强度，范围 `[0,1]` |

### 7.12 `reset` 与 `session_stop`

重置：

```json
{
  "type": "reset",
  "data": {
    "randomize": false
  }
}
```

关闭：

```json
{
  "type": "session_stop",
  "data": {}
}
```

连接断开时，服务端也会清理该连接绑定的会话。

## 8. WebSocket 服务端消息

### 8.1 `state_update`

非批量模式下，每次有效 `control` 后返回：

```json
{
  "type": "state_update",
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "timestamp": 1785123456789,
  "data": {
    "tip_position": [0.0, 0.0, 0.0],
    "tip_direction": [0.0, 0.0, 1.0],
    "velocity": 0.0,
    "contact_force": 0.0,
    "path_progress": 0.1,
    "reward": 0.0,
    "done": false,
    "safety_status": "SAFE_NAV"
  }
}
```

`data` 的完整字段见“仿真状态字段”。

### 8.2 `state_batch`

当 `session_start.data.batch_mode=true` 时返回适合三维渲染的批量状态：

```json
{
  "type": "state_batch",
  "session_id": "b6765825-67bb-451b-83d8-b7fcaaf49e71",
  "timestamp": 1785123456789,
  "data": {
    "schema_version": "navigation_visual_v2",
    "timestamp_ms": 1785123456789,
    "engine": "NewtonPhysicsEngine",
    "fidelity_mode": "physics",
    "diagnostics": {},
    "guidewire": {},
    "support": {},
    "procedure": {},
    "risk": {},
    "tip": {
      "position": [0.0, 0.0, 0.0],
      "direction": [0.0, 0.0, 1.0],
      "quaternion": [0.0, 0.0, 0.0, 1.0]
    },
    "bodies": [],
    "path": {
      "waypoints": [],
      "progress": 0.1,
      "deviation": 0.0,
      "remaining_distance": 0.1,
      "vessel_radius": 0.002,
      "eta_seconds": null
    },
    "entry": {},
    "target": [0.0, 0.0, 0.0],
    "safety": {},
    "flow_guidance": {},
    "training_score": {},
    "episode": {
      "length": 1,
      "reward": 0.0,
      "done": false
    }
  }
}
```

规划路径可能包含大量点，因此只在首个 `state_batch`、切换路线或重置后发送。
后续批量状态中的 `path.waypoints` 通常为空数组，客户端应保留之前收到的路径。

### 8.3 `error`

错误消息结构：

```json
{
  "type": "error",
  "session_id": "uuid-or-null",
  "timestamp": 1785123456789,
  "data": {
    "code": "INVALID_PARAMS",
    "message": "错误详情"
  }
}
```

常见错误码：

| 错误码 | 说明 |
|---|---|
| `INVALID_MESSAGE` | 外层消息格式不合法 |
| `UNKNOWN_TYPE` | 不支持的消息类型 |
| `INVALID_PARAMS` | `data` 参数校验失败 |
| `INVALID_CONTROL` | 控制参数不合法 |
| `SESSION_EXISTS` | 当前连接已有活动会话 |
| `NO_SESSION` | 当前连接没有活动会话 |
| `SESSION_EXPIRED` | 会话已被清理 |
| `SESSION_ERROR` | 会话初始化失败 |
| `PATH_NOT_FOUND` | 病例或路径不存在 |
| `STEP_ERROR` | 仿真步执行失败 |
| `PARAM_ERROR` | 物理参数设置失败 |
| `DEVICE_CONFIG_ERROR` | 器械配置失败 |
| `INTENT_ERROR` | ShapeIntent 设置失败 |

## 9. 推荐调用流程

实时三维客户端推荐使用以下顺序：

```text
连接 ws://<服务器IP>:19000/ws/session
    -> 接收 ping 并持续回复 pong
    -> 发送 session_start（batch_mode=true）
    -> 接收 session_started 和 initial_batch
    -> 以不高于 30 Hz 的频率发送 control
    -> 接收 state_batch 并更新渲染
    -> 按需发送 select_route/device_config/shape_intent
    -> 发送 session_stop
    -> 关闭 WebSocket
```

只进行离线路径规划时，不需要创建会话，直接调用 REST
`POST /api/v1/path/plan` 或发送 WebSocket `path_request`。

## 10. 部署与连通性检查

### 10.1 Docker 状态

```bash
docker ps --filter name=cathsim-server
docker logs --tail 100 cathsim-server
```

### 10.2 服务健康

```bash
curl http://<服务器IP>:19000/api/v1/health
```

### 10.3 OpenAPI

```bash
curl http://<服务器IP>:19000/openapi.json
```

### 10.4 外部访问要求

如果其他计算机无法访问，应依次检查：

1. Docker 是否映射 `0.0.0.0:19000->9000/tcp`。
2. 服务器防火墙或云安全组是否放行 TCP `19000`。
3. 服务前方是否存在反向代理、WAF 或端口转发。
4. 浏览器前端是否因跨域或混合内容被阻止。
5. HTTPS 页面是否将 WebSocket 地址相应改为 `wss://`。

## 11. 实现位置

接口的实际行为以代码和运行时 OpenAPI 为准：

- REST 路由：`services/main.py`
- REST 数据模型：`services/schemas.py`
- WebSocket 协议：`services/websocket_handler.py`
- Docker 端口映射：`server/compose.simulation.deploy.yaml`

