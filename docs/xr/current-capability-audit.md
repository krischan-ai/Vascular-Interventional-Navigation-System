# XR 当前能力与差距审计

> 审计日期：2026-08-03；判定词仅使用：复用、适配、拆分、缺失。

| 对象 | 证据 | 判定 | 动作 |
|---|---|---|---|
| 桌面入口 | `scenes/main.tscn`、`project.godot` | 复用 | 保持桌面主场景 |
| 主控制器 | `scripts/main_controller.gd` | 拆分 | M3 逐项抽状态、控制、坐标与渲染适配 |
| WebSocket | `scripts/websocket_client.gd` | 适配 | 复用重连/session/急停；原始 JSON 不进入 XR UI |
| 数据契约 | `navigation_visual_v3` | 复用 | 保留 source/timestamp/null/unknown/stale |
| 后端安全 | `data.safety`、`control_state` | 复用 | 保持安全与急停锁存权威 |
| 桌面 HUD | `scripts/hud_controller.gd` | 拆分 | 保持回归，不把 CanvasLayer 贴入 XR |
| 血管/导丝/路径 | 现有 renderer | 适配 | M6 增加 mm→m 和 XR 锚定 |
| 腔镜 | 单动态 SubViewport | 适配 | XR 独立宿主并按性能档限频 |
| DSA | 桌面占位界面 | 适配 | 强制 Placeholder 文案与工具禁用 |
| 急停 | 请求、确认、恢复、锁存拒绝 | 复用 | XR 接入前建立统一 SafetyGate |
| Freshness | 桌面 Fresh/Stale | 适配 | 新增 Delayed；Stale 必须阻断 |
| XR 开关 | `project.godot` | 已适配 | OpenXR/shaders 已启用 |
| XR 入口 | `scenes/xr/MainXR.tscn` | 已适配 | 独立入口，桌面入口不变 |
| XR Bootstrap | `scripts/xr/xr_bootstrap.gd` | 已适配 | 只报告 Runtime/Aim/Grip；控制固定中性 |
| 后端 Debug 地址 | `project.godot`、`start_godot.bat` | 已适配 | 默认回环地址，可由 CLI/环境变量覆盖 |
| Action Map | 仓库无 `.tres` | 缺失 | S0-10 锁定插件后建立 |
| Vendors Plugin | 仓库无 `addons` | 缺失 | 官方 release 锁版 |
| Android 导出 | 无 preset 和 4.7.1 模板 | 缺失 | S0-03/11 阻断 |
| PICO 真机 | 无 ADB/Runtime 证据 | 缺失 | S0-12 阻断 |

## 依赖边界

- XR Bootstrap 不持有 WebSocketClient，不调用 `send_control()`。
- 网络层只更新共享状态；空间 UI 只消费一致快照。
- 输入先形成 `XRInputSnapshot`，再经 SafetyGate 和 ControlRouter。
- 任一门控失效同周期归零；Reference Space 变化后重新 Deadman。
- DSA 占位状态不写入后端实时数据；腔镜状态与网络状态分离。

## 阻断结论

- M0 可继续完成文档、契约、桌面基线与零控制场景。
- 工具链和插件未齐前，不声明 S0-03/10/11 完成。
- 无 PICO 设备证据时，M1 保持“准备中”。
