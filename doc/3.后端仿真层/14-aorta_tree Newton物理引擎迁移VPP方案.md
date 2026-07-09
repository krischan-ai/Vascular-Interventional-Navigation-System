# aorta_tree Newton 物理引擎迁移 VPP 方案

## 1. aorta_tree 现有能力验收

`aorta_tree` 的 Newton 物理链路已经形成可迁移基线：

- 后端：`services.physics.newton_engine.NewtonEngine`
- 入口：`physics_engine=newton_demo` 或环境变量 `CATHSIM_PHYSICS_ENGINE=newton_demo`
- 路径资产：`routes.json`
- 管腔建模：每条 route 携带 `radius_m`，进入 `PlannedPath.radii`
- 碰撞建模：沿中心线构建变半径厚壁环管 SDF
- 导丝驱动：force drive、sheath 约束、J-tip、软头/硬身参数
- 交互协议：`control`、`select_route`、`shape_intent`、`engine_params`
- 安全语义：force 模式按 `contact_force/contact_ke` 判断穿壁，避免把正常贴壁误判为碰撞

验收结论：`aorta_tree` 的可迁移核心不是低多边形 `visual.stl`，而是 `routes.json radius_m -> PlannedPath.radii -> NewtonEngine variable-radius SDF` 这条真实腔物理链路。

## 2. VPP 当前迁移状态

已迁移：

- VPP 会话可显式选择 `physics_engine`
- Godot VPP 默认入口切到 `newton_demo`
- VPP 路径规划结果开始携带 `node_radii.json` 半径
- WebSocket 边界将 VPP 坐标和半径从毫米转换为米
- `planned_radii` 已穿透 `WebSocketHandler -> SessionManager -> NavigationEngine -> PlannedPath`
- Newton 后端无需 VPP 特判即可读取 `PlannedPath.radii`
- VPP `targets.json + graph.json + node_radii.json` 已可生成 `derived/routes.json`
- `NavigationEngine(case_001_vpp)` 已可加载 VPP routes，并复用 `available_routes/select_route`

当前 VPP 与 aorta_tree 的共同链路：

```text
VPP graph + node_radii.json
  -> PathPlanner.plan()
  -> PathResult.waypoints / radii
  -> WebSocket _resolve_session_path()
  -> NavigationEngine(planned_path, planned_radii)
  -> PlannedPath.radii
  -> NewtonEngine._centerline_radii()
  -> variable-radius thick-wall SDF
```

## 3. 缺失内容盘点

### P0 必须补齐

- VPP 多分支目标清单：已完成第一版，`tools/build_vpp_routes.py` 生成 `derived/routes.json`。
- VPP 分支切换：已完成后端复用，`case_001_vpp` 可加载 routes 并使用 `select_route(target)`。
- VPP Newton 启动验收：需要确认 `session_started.engine == NewtonEngine`，首帧 batch 正常返回 bodies/path/radii-derived radius。
- VPP Newton 推进验收：需要跑短路径 smoke，检查 `path_progress` 增加、`contact_force` 合理、无明显穿壁。

### P1 迁移后开发

- VPP 路径半径平滑策略：当前 `smooth_radii` 按弧长插值，后续需检查窄分支是否需要半径下限/局部收缩。
- VPP entry pose 校准：真实物理下入口方向、sheath 起点、初始插入长度需要按 VPP 几何重新标定。
- VPP 导丝参数标定：`rod_length`、`free_len`、`max_slack`、`jtip_deg`、`bend/stretch/contact_ke` 需要针对长路径和细分支调参。
- VPP 真实内壁验证：需要把 Newton 合成厚壁环管与 VPP native mesh/中心线半径做几何一致性审计。
- Godot HUD 后端状态显示：把 `engine/fidelity_mode/physics_engine` 显示到调试面板，避免误看 guided/MuJoCo。

### P2 后续增强

- 从 VPP `targets.json` 自动生成可选分支菜单。
- 保存 VPP 规划路线为 `routes.json` 缓存，支持重复实验可复现。
- 将 VPP Newton smoke 纳入 CI 的可选慢测试。
- 比较 VPP MuJoCo hull 与 Newton SDF 的接触差异，决定保留/废弃 MuJoCo hull 训练路径。

## 4. 迁移验收标准

阶段 1：数据通路

- `PathPlanner` 能从 `node_radii.json` 读出每个 waypoint 半径。
- `smooth_waypoints` 与 `smooth_radii` 数量一致。
- WebSocket 解析出的 `planned_radii` 单位为米，长度等于 `planned_path`。

阶段 2：Newton 后端

- VPP `session_start(physics_engine=newton_demo)` 成功。
- `session_started.engine == "NewtonEngine"`。
- `state.fidelity_mode == "physics"`。
- 首帧/首步 `state_batch.bodies` 非空。
- `path.vessel_radius` 来自 VPP 半径数据，而不是 Newton 默认常数半径。

阶段 3：交互功能

- 手动推进/回拉/旋转有效。
- 点击导航 `shape_intent` 可进入 waypoint 模式。
- 急停/人工接管/恢复导航可工作。
- 分支切换在 VPP route target 生成后可工作。

## 5. 下一步开发顺序

1. 在装有 `newton` 包的环境跑 VPP Newton 短路径 smoke，验证启动和推进。
2. 把后端模式显示到 Godot HUD。
3. 针对 VPP 调参 Newton force drive。
4. 增加 VPP native mesh 与半径管腔的一致性审计。
5. 将 VPP routes 缓存纳入资产生成/校验流程。
