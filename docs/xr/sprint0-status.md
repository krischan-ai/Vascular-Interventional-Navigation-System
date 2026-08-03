# Sprint 0 状态与证据

> 更新：2026-08-03。状态只表示当前仓库与本机证据，不替代评审或 PICO 真机门禁。

| ID | 状态 | 证据 | 阻断/下一步 |
|---|---|---|---|
| S0-01 | 完成 | `scripts/godot.ps1`、`validate_godot.ps1` | 无 |
| S0-02 | 部分完成 | `version-matrix.md` | 待 PICO OS/Runtime、Vendors、SDK 实值 |
| S0-03 | 阻断 | `environment-check.md` | 缺 JDK17、SDK、ADB、Gradle、模板 |
| S0-04 | 完成 | `current-capability-audit.md` | M3 按边界拆分 |
| S0-05 | 完成 | `control-data-contract.md` | 后端目标字段/150ms 看门狗待实现，非零控制禁用 |
| S0-06 | 部分完成 | `artifacts/xr/m0/desktop-baseline/README.md` | 缺真实 Newton Session 与最新截图 |
| S0-07 | 完成 | `visual-baseline.md` | 待真机视觉校准 |
| S0-08 | 完成 | `imaging-boundary.md` | 待真机 SubViewport 性能试验 |
| S0-09 | 完成 | `MainXR.tscn`、`xr_bootstrap.gd` | headless 零控制快照通过 |
| S0-10 | 阻断 | `version-matrix.md` | Vendors 未锁版，Action Map 未建 |
| S0-11 | 阻断 | `environment-check.md` | Android 工具链/插件未齐 |
| S0-12 | 阻断 | 无真机证据 | 需要 PICO、ADB、Runtime |
| S0-13 | 进行中 | 本表 | M0 阻断项尚未关闭 |

## 已通过

- `.\scripts\godot.ps1` → Godot 4.7.1 精确版本。
- 前端与 XR 契约 → 6 passed。
- WebSocket 协议回归 → 44 passed。
- Godot 导入/脚本/资源检查 → passed。
- MainXR headless 无 Runtime → exit 0，`controls_blocked=true`，push/rotate 为 0。

## 明确未通过

- 本机 `.venv` 可启动健康后端，但缺 `newton`，默认 `newton_demo` Session 返回 `SESSION_ERROR`。
- 没有 Android Debug APK、Vendors Plugin、Action Map 或 PICO 真机证据。
- M1 继续为“准备中”，M2 非零控制不得启用。
