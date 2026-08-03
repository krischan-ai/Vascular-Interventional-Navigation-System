# XR 开发环境核验

> 核验日期：2026-08-03

| 工具 | 结果 | 判定 |
|---|---|---|
| Godot | `4.7.1.stable.official.a13da4feb` | 通过 |
| 仓库入口 | `.\scripts\godot.ps1` | 通过 |
| Java | Oracle JRE `1.8.0_441` | 不合格 |
| javac / JDK | 缺失 | 阻断 |
| Android SDK / Build Tools | 默认目录缺失 | 阻断 |
| adb / Gradle | 命令缺失 | 阻断 |
| Godot 4.7.1 Android 模板 | 缺失 | 阻断 |
| Vendors Plugin | 未安装、未锁版 | 阻断 |
| PICO 设备/Runtime | 无 ADB 证据 | 阻断 |

验证命令：

```powershell
.\scripts\godot.ps1
.\scripts\validate_godot.ps1
pytest tests\test_frontend_contract.py tests\test_xr_sprint0_contract.py -q
```

Godot 校验日志在当前受限 Windows 环境中包含根证书读取和用户级 editor settings 写入失败；脚本/资源解析仍通过。该环境限制不等于 Android/PICO 通过，发布环境必须重新验证。

S0-03、S0-10、S0-11、S0-12 当前均不得关闭。
