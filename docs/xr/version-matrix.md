# CathSim XR 版本矩阵

> 基线日期：2026-08-03  
> 状态：M0 初版。`阻断` 项未满足前，不得声明 Android APK 或 PICO 真机通过。

| 项目 | 冻结值/目标 | 来源 | 当前状态 |
|---|---|---|---|
| Godot | `4.7.1.stable.official.a13da4feb` | `scripts/godot.ps1` 实测 | 已锁定 |
| 桌面渲染器 | Forward Plus | `project.godot` | 已验证 |
| XR 发布渲染器 | Mobile | Godot 官方一体机建议 | 待 Android 验证 |
| OpenXR | 1.1；Godot Core OpenXR | 设计方案、Godot 4.7 | 骨架已启用 |
| OpenXR shaders | enabled | `project.godot` | 已启用 |
| OpenJDK | 17 | 开发规划 | 阻断；仅有 Java 8 JRE |
| Android SDK / Build Tools | 待 S0-03 锁定 | 本机核验 | 阻断；未安装 |
| ADB / Gradle | 与 SDK/模板匹配 | 本机核验 | 阻断；命令缺失 |
| Godot Android Build Template | `4.7.1.stable` | Godot 模板目录 | 阻断；模板缺失 |
| Godot OpenXR Vendors Plugin | 与 Godot 4.7/PICO Runtime 匹配的稳定版 | GodotVR 官方 release | 阻断；未选版、未安装 |
| PICO OS / Runtime | 待设备实测 | ADB、Runtime 日志 | 阻断；无设备证据 |
| Action Map | `medical_control`、`ui_navigation` | 设计方案 | S0-10 待建 |
| Reference Space | `LOCAL_FLOOR`，降级 `LOCAL` | 设计方案 | S0-10/12 待验证 |
| Interaction Profile / 扩展 | 运行时枚举，不预填 | 真机日志 | 阻断；无证据 |
| APK | arm64 Debug，版本待定 | Android export | S0-11 阻断 |
| 腔镜 | 单动态 SubViewport；FOV 78°–85° 档位 | 当前代码 | 桌面已验证 |
| 腔镜刷新/分辨率 | 60/45/30 Hz、1024/768/640 长边试验 | M0 影像基线 | 真机待冻结 |

## 可复现命令

```powershell
.\scripts\godot.ps1
.\scripts\validate_godot.ps1
```

可用 `CATHSIM_GODOT_EXE` 或 `-GodotExe` 指向团队安装，但版本不匹配会立即失败。

## 变更规则

Godot、Vendors Plugin、JDK、SDK、Build Tools、PICO Runtime、渲染器或 OpenXR 扩展变化时，必须更新本表并重跑桌面基线、Android 构建及 M1 真机矩阵。
