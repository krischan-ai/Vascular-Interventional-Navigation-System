# Progress Log

## 2026-07-28 MuJoCo 分割与全量测试修复
- **Status:** in_progress
- 已恢复会话、读取规划文件并复核 Git 状态；目标源码和 8 个旧测试无用户未提交修改。
- 已确认修复范围：分割背景兼容层、现代化 8 个旧测试、完整收集和全量 pytest。

## 2026-07-27 本地前后端展示
- **Status:** in_progress
- 已运行仓库内 session-catchup，读取三份规划文件并复核 `git diff --stat`；现有未提交腔镜实现保持不变。
- 当前 9000 端口空闲，配置为 localhost；下一步启动本轮专用后端日志和实际 Godot GUI。
- 后端启动成功并保持运行：PID 40684，健康接口通过；Warp 缓存使用工作区 `.tmp/warp`，已有本地 WebSocket 连接。
- Godot GUI PID 34888 启动成功；session `e4bb3db7-ae7c-4c45-b4c1-b6c2762c8b6c` 已收到 NewtonEngine 首个 state_batch（2152 路径点），窗口 `CathSim VPP Client (DEBUG)` 已置前。
- 最终健康与日志复核通过：前后端进程均响应，运行错误扫描为 CLEAN。前后端保持运行，交由用户本地操作。
- **Status:** complete

## 2026-07-27 当前工作树本地测试
- **Status:** in_progress
- 已确认当前分支 `guidewire-device-procedure-design` 比远端领先 1 个提交，工作树包含尚未提交的腔镜渲染、前端契约测试、文档和规划文件改动；本轮测试保留并覆盖当前工作树，不执行 reset/checkout。
- 已按 `planning-with-files` 运行会话恢复。全局脚本路径不存在，记录为一次环境路径错误；改用仓库内 `.codex/skills/planning-with-files/scripts/session-catchup.py` 后成功恢复上下文。
- 下一步：确认 README/测试入口与运行环境，然后执行自动化、Godot、后端和 WebSocket 实测。
- 环境复核完成：Python/Godot/依赖均可调用；默认 WebSocket 地址是 localhost；本地后端当前未运行。发现两个未知 Godot 进程，因无法安全确认命令行而保持不动。
- 前端定向回归：`python -m pytest tests/test_frontend_contract.py -q` → `3 passed in 0.05s`。
- 全量收集：`python -m pytest --collect-only -q` → 收集 261 项，但 8 个旧测试模块发生收集错误并中断；已记录逐项原因，下一轮隔离这些收集级失败后执行其余测试。
- 静态差异检查：`git diff --check` 通过，仅有 LF→CRLF 提示。
- 可执行测试第一轮（隔离 8 个 collection error 模块）：`241 passed, 1 skipped, 4 failed, 15 errors in 137.82s`。15 个 setup error 已归因为沙箱不可写的 pytest 默认 Temp；4 个断言期失败均集中在旧 dm_control segmentation 渲染。
- 工作区 basetemp 复测：`19 passed, 4 failed in 22.04s`；此前 15 个权限型 setup error 全部通过，4 个 dm_control 失败稳定复现。可执行集合合并结果：`256 passed, 1 skipped, 4 failed`。
- Godot 第一次校验未形成有效结果：显式传入非 console EXE 后 PowerShell 未取得退出码，脚本以空 `$LASTEXITCODE` 误报导入失败；下一次切换 console EXE。
- Godot console 在沙箱内完成项目启动，但被证书库/AppData 权限错误拦截；获准在正常权限复跑后 `Godot validation passed`，真实 GLB 与 enhanced/balanced 渲染脚本加载正常。
- 本地后端启动成功，健康接口为 ok/vpp_ready=true/case_001，Warp 识别 RTX 4060。
- 第一次 WebSocket 探针失败：旧脚本落入 MuJoCo 并使用无效 `endpoint_9` target；已记录服务端栈，下一轮按当前协议显式测试 Newton。
- 当前协议 WebSocket/Newton 复测通过：`session_started` 确认为 NewtonEngine/newton_demo/physics；初始 20 bodies、139 path points；控制后收到 state_batch（progress≈0.1061、SAFE_NAV）；session_stop 正常。
- 最终健康检查通过；成功会话未出现 PONG_TIMEOUT/SESSION_ERROR。本轮创建的后端 PID 38812 已关闭，9000 端口恢复空闲。
- **Status:** complete。产品代码未改；测试证据和遗留问题已写入规划文件。

## Session: 2026-07-24

### Phase 1: 需求与现状发现
- **Status:** complete
- **Started:** 2026-07-24
- Actions taken:
  - 显式启用并完整读取项目级 `planning-with-files` v3.8.1。
  - 运行 `session-catchup.py`；未报告需要恢复的既有计划上下文。
  - 读取三份官方模板并确认项目根目录不存在旧计划文件。
  - 将用户参考图的面板结构、色彩、控件和真实实时渲染要求写入 `findings.md`。
  - 建立五阶段开发计划。
  - 搜索腔镜、SubViewport、Camera3D、lumen 与 LIVE 相关代码和文档。
  - 审读 `main_controller.gd` 的窗格构建、血管复制/材质、相机同步，以及 `camera_rig.gd`、`endoscope_fallback.gd` 和前端契约测试。
  - 确认当前未完成实现已经使用真实血管 GLB 与导丝尖端相机，但仍存在静态 fallback 叠加、独立纵深效果和器械尖端/控件呈现缺口。
  - 追踪 `_guidewire_front_pose_from_batch()` → `_feed_rig()` → `CameraRig.update_tip()` → `_sync_scope_endoscope_camera()`，确认腔镜相机使用真实导丝最前端与末端切线。
  - 核对前端交接文档 8.4，确认无数据状态、材质/雾化/暗角/分叉，以及相机/REC/时间轴/亮度/全屏真实状态均为明确待办。
  - 检查最新本地连接截图 `.tmp/local_runtime/control-connected.png`，确认腔镜窗格实际显示程序化 fallback，而不是可辨认的真实血管腔内几何。
  - 记录当前已连接且有导丝进度时仍静态显示 LIVE 的伪实时问题，以及参考图和现有控件的视觉差距。
  - 核对 GLB 源文件、Godot `.godot/imported` 产物和运行日志；确认真实血管加载成功且后端已提供有效 tip/state_batch，因此问题集中在腔镜渲染可见性链路。
  - 运行改造前前端契约测试，2 项通过。
- Files created/modified:
  - `task_plan.md`（创建）
  - `findings.md`（创建）
  - `progress.md`（创建）

### Phase 2: 技术方案与验收设计
- **Status:** complete
- Actions taken:
  - 选择独立 `World3D` 的真实腔镜渲染方案，隔离主视图环境、雾和渲染层。
  - 设计有效连接 + 血管模型 + 真实 tip 三条件门控，禁用伪 LIVE。
  - 设计相机姿态插值、暖色内壁/雾/头灯、暗角和相机罩尖端视觉。
  - 设计 REC 计时、亮度调节、当前帧截图和中央工作区全屏行为。
  - 定义前端契约、Godot headless 和实际连接运动截图三层验收标准。
- Files created/modified:
  - `task_plan.md`（阶段状态与技术决策）
  - `findings.md`（实现方案）
  - `progress.md`（发现阶段结果与测试基线）

### Phase 3: 实现
- **Status:** complete
- Actions taken:
  - 准备修改腔镜专用脚本、主控制器、图标绘制和前端契约测试。
  - 将 `EndoscopeFallback` 从程序化拟真隧道改为明确的等待/无数据状态。
  - 新增 `EndoscopeOverlay`，只绘制暗角、镜头边缘和虚拟镜头罩，不生成血管解剖内容。
  - 扩展 `PaneToolIcon` 的 refresh/record/brightness/capture/fullscreen 线绘图标。
  - 将腔镜 SubViewport 改为独立 World3D，加入专用暖色环境、指数雾、ACES、glow 和镜头头灯。
  - 重构腔镜标题徽标与底栏结构，预接 REC、亮度、截图和全屏信号。
  - 将真实血管副本移入腔镜私有 World3D，并使用双面暖色噪声材质呈现实际内壁几何。
  - 接入连接 ready、血管实例和真实 tip 三条件门控；断连/换模型会清空 LIVE 与相机平滑状态。
  - 为腔镜相机加入逐帧 Transform3D 插值，仍以真实导丝前端相机为唯一目标姿态。
  - 实现 REC 单调计时、亮度同时控制材质与头灯、当前 SubViewport 帧保存以及中央区域扩大/恢复。
  - 增补腔镜前端契约测试，覆盖独立 World3D、真实血管父级、导丝相机位姿、LIVE 门控、非伪造 fallback、动态控制与图标。
- Files created/modified:
  - `task_plan.md`（进入 Phase 3）
  - `findings.md`（记录可执行验收入口）
  - `progress.md`（设计阶段完成）
  - `godot_client/scripts/ui/endoscope_fallback.gd`（移除伪实时隧道）
  - `godot_client/scripts/ui/endoscope_overlay.gd`（新增光学叠加层）
  - `godot_client/scripts/ui/pane_tool_icon.gd`（新增腔镜图标）
  - `godot_client/scripts/main_controller.gd`（独立腔镜世界与面板结构）
  - `tests/test_frontend_contract.py`（腔镜实时渲染契约）

### Phase 4: 测试与可视化验证
- **Status:** complete
- Actions taken:
  - 运行改造后的前端契约测试，3 项全部通过。
  - 首次 Godot headless 校验发现并定位不兼容的灯光掩码属性；私有 World3D 本身已隔离灯光，移除该冗余赋值。
  - 再次运行 Godot 4.7.1 资源导入与 headless 启动，项目脚本加载和血管 GLB 实例化成功，校验脚本通过。
  - 找到 `cathsim-dev` Conda 环境，并用其前台保持本地 uvicorn 进程；健康检查返回 `status=ok`、`vpp_ready=true`、`case_001`。
  - 启动实际 Godot 客户端并连接 NewtonEngine；运行日志确认首个真实 `state_batch`、2152 个路径点与持续状态流。
  - 保存首张实际画面 `.tmp/godot/scope-live-before.png`：实时 SubViewport 已出现，但血管内壁近乎纯红，视觉验收未通过。
  - 用 `trimesh` 确认真实 GLB 有 312507 顶点和法线但无 UV；据此改为世界空间三平面组织纹理，并重新平衡雾、环境光、头灯和自发光。
  - 按 Godot 4.7 `Light3D.light_cull_mask` 属性把镜头头灯明确限制到腔镜专用血管层，并使用独立日志规避一次崩溃进程造成的默认日志锁。
  - 通过目标窗口 `WM_KEYDOWN/UP` 发送真实 W 推进；Godot 日志确认首个 `push=0.35` control 帧。
  - 实际运行中路径进度 5% → 7% → 14%，剩余距离 106.5 cm → 104.1 cm → 97.0 cm，腔镜画面随 tip 位姿发生连续变化。
  - 保存运动前、运动后和深入推进三张实机截图到 `.tmp/godot/`，完成参考图的暖色组织、LIVE、器械罩与底栏控件检查。
- Files created/modified:
  - `progress.md`（记录测试结果）

### Phase 5: 交付
- **Status:** complete
- Actions taken:
  - 运行最终前端契约测试、Godot 独立日志 headless 校验和 `git diff --check`，均通过。
  - 检查 `git status` 和相关文件 diff；确认工作区原有后端、HUD、文档和测试修改均保留，未执行 reset/checkout。
  - 确认所有截图、日志和临时依赖均位于被 `.gitignore` 排除的 `.tmp/`；最终三张截图 SHA-256 不同，且界面状态和日志共同证明运动变化。
  - 关闭本次 uvicorn 与正常 Godot GUI/console 进程；一次 Godot 属性探测崩溃留下 PID 46856 的无命令行残留项，系统拒绝 Stop-Process/CIM 终止，重启 Windows 可清理。
- Files created/modified:
  - 无。

## Test Results
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| planning-with-files 会话恢复检查 | `python .codex/skills/planning-with-files/scripts/session-catchup.py <project>` | 无旧计划时安静退出 | 退出码 0，无恢复报告 | ✓ |
| 改造前前端契约基线 | `python -m pytest tests/test_frontend_contract.py -q` | 现有测试全绿 | `2 passed in 0.06s` | ✓ |
| 腔镜实时渲染前端契约 | `python -m pytest tests/test_frontend_contract.py -q` | 独立世界、真实位姿、状态门控与控件契约全绿 | `3 passed in 0.06s` | ✓ |
| Godot 资源导入与 headless 校验（首次） | `scripts/validate_godot.ps1` | 无脚本错误并正常退出 | 资源导入成功；`OmniLight3D.cull_mask` 属性赋值触发 `SCRIPT ERROR` | ✗ |
| Godot 资源导入与 headless 校验（修复后） | `scripts/validate_godot.ps1` | 无脚本错误并正常退出 | Godot 4.7.1 加载脚本与真实血管 GLB 成功；`Godot validation passed` | ✓ |
| 本地真实后端启动（首次） | `python -m uvicorn services.main:app --host 127.0.0.1 --port 9000` | 30 秒内健康检查成功 | 端口未就绪，重定向日志暂未给出诊断 | ✗ |
| 工作区临时运行依赖安装 | `python -m pip install --target .tmp/pydeps "uvicorn[standard]"` | 不修改全局环境并提供 ASGI/WebSocket 服务依赖 | 沙箱内网络被拒；获准联网后成功安装到 `.tmp/pydeps` | ✓ |
| 本地真实后端启动（第二次） | 设置 `PYTHONPATH=.tmp/pydeps` 后隐藏启动 uvicorn | 45 秒内健康检查成功 | 子进程仍立即退出且重定向日志为空，需前台运行捕获启动行为 | ✗ |
| 项目 Conda 后端健康检查 | `cathsim-dev/python -m uvicorn ...` + `GET /api/v1/health` | 本地后端与 VPP 资产就绪 | `{"status":"ok","version":"0.1.0","vpp_ready":true,"cases":["case_001"]}` | ✓ |
| 实际 Godot/后端连接 | 启动嵌套 `godot_client/project.godot` 并读取运行日志 | 收到真实 Newton 状态流 | `session_started`、`engine=NewtonEngine`、`waypoints=2152`、首个 `state_batch` | ✓ |
| 首次实际腔镜视觉验收 | `.tmp/godot/scope-live-before.png` | 内壁纹理、形状和纵深可读 | 真实视口已显示，但画面近乎纯红，仅有暗色远端孔洞 | ✗ |
| 血管网格纹理通道检查 | `trimesh.load(...blood_vessels_visual_native.glb...)` | 确认材质可用的顶点通道 | 312507 顶点、625124 面、有法线、无 UV | ✓ |
| 调光后第二次实际腔镜视觉验收 | `.tmp/godot/scope-live-textured.png` | 纹理与局部光照层次清晰 | 真实管腔轮廓和环形表面已出现，但整体过暗；专用 18 层未收到默认 1 层头灯 | ✗ |
| Godot 灯光属性探测（首次） | 同时运行 GUI 时执行临时 headless 属性脚本 | 输出 Light3D 的层掩码属性 | `user://logs` 并发写入失败后 Godot 4.7.1 崩溃，无属性输出 | ✗ |
| 独立日志 headless 灯光修复校验 | Godot `--headless --log-file .tmp/godot/headless-scope-light.log --quit-after 2` | 新属性和材质可被 Godot 4.7.1 加载 | 正常启动、加载真实 GLB、退出码 0 | ✓ |
| 最终 GUI 重连等待（首次） | Godot 独立日志运行后等待 45 秒 | 收到 `state_batch` | 后端保持进程已到工具时限而结束，客户端记录 WebSocket closed | ✗ |
| 最终 GUI 重连等待（第二次） | 重启后端后轮询独立日志 | 识别首个真实状态批次 | 日志已出现 `[Main] first state_batch`，但轮询只匹配 `[WS] first 'state_batch'` 而误报超时 | ⚠ |
| GUI 物理按键运动注入（首次） | `keybd_event` 保持 W/A 后检查 Godot 日志 | 产生真实 control 帧 | 窗口截图成功，但日志无 control；通用键事件未进入 Godot 物理键轮询 | ✗ |
| GUI 物理按键运动注入（第二次） | `PostMessage(WM_KEYDOWN/UP)` | 产生真实 control 帧 | PowerShell 将 `Int32` 转换为 `UIntPtr` 失败，消息未发送 | ✗ |
| GUI 物理按键运动注入（修复后） | 目标窗口 `WM_KEYDOWN/UP`，W 保持 4 秒及继续推进 12 秒 | 后端收到真实推进，画面与状态变化 | 日志 `push=0.35`；进度 5% → 7% → 14%，腔镜视角连续变化 | ✓ |
| 最终前端契约回归 | `python -m pytest tests/test_frontend_contract.py -q` | 全绿 | `3 passed in 0.01s` | ✓ |
| 最终 Godot 4.7.1 headless | `Godot --headless --path godot_client --log-file ... --quit-after 2` | 无脚本错误、真实 GLB 加载 | 退出码 0；`MeshInstance3D count=1` | ✓ |
| 最终 diff 空白检查 | `git diff --check` | 无 whitespace error | 退出码 0；仅报告既有 Windows LF→CRLF 提示 | ✓ |

## Error Log
| Timestamp | Error | Attempt | Resolution |
|-----------|-------|---------|------------|
| 2026-07-24 | `apply_patch` 未找到视觉更新的复合补丁锚点 | 1 | 读取当前文件后改用按文件顺序排列的小锚点补丁 |
| 2026-07-24 | 阶段同步补丁把 `Test Results` 锚点放入 `findings.md` 更新块 | 2 | 拆分为三个独立文件补丁，第三次成功 |
| 2026-07-24 | 阶段同步补丁同时跨文件匹配时未找到 `progress.md` 锚点 | 3 | 先单独更新 `task_plan.md`，再按实际 UTF-8 内容更新 `progress.md` |
| 2026-07-24 | Godot 4.7.1 中 `OmniLight3D` 不接受 `cull_mask` 属性 | 1 | 私有 World3D 已天然隔离，移除冗余掩码后第二次 headless 校验通过 |
| 2026-07-24 | 隐藏启动本地 uvicorn 后 30 秒内健康检查未就绪 | 1 | 检查子进程退出状态与 stdout/stderr，再按实际启动错误处理 |
| 2026-07-24 | 沙箱网络拒绝下载缺失的 uvicorn | 1 | 获得联网许可后仅安装到工作区 `.tmp/pydeps`，不改全局 Python 环境 |
| 2026-07-24 | 带临时依赖的第二个隐藏 uvicorn 进程仍立即退出且日志为空 | 2 | 改为有限时长前台启动，直接捕获进程输出与退出行为 |
| 2026-07-24 | Anaconda `base` 前台启动报告 `ModuleNotFoundError: fastapi` | 3 | 找到并改用项目专用 `cathsim-dev` Conda 环境，健康检查通过 |
| 2026-07-24 | 首张真实腔镜截图为近乎纯红平面，缺少内壁层次 | 1 | 确认 GLB 无 UV；启用三平面纹理并降低雾/环境光、提高局部头灯对比，待重测 |
| 2026-07-24 | 调光后真实管腔可见但整体过暗 | 2 | 确认默认头灯只照明第 1 层，而血管位于专用第 18 层；探测 Godot 4.7 实际灯光掩码属性 |
| 2026-07-24 | GUI 与临时 headless 属性脚本并发写 `user://logs` 导致 Godot 崩溃 | 1 | 先关闭本次 GUI 验证进程，再单独运行只读属性探测脚本 |
| 2026-07-24 | 最终 GUI 启动时本地后端保持进程已到 10 分钟工具时限 | 1 | 重新以前台保持单元启动 `cathsim-dev` uvicorn，利用客户端自动重连继续验证 |
| 2026-07-24 | 重连轮询的正则过窄，日志已有 `[Main] first state_batch` 仍判超时 | 2 | 以实际日志中的 Main 状态批次作为连接成功证据，继续截图和运动验证 |
| 2026-07-24 | `keybd_event` 未进入 Godot 的 `Input.is_physical_key_pressed` 状态 | 1 | 改用目标窗口的 `WM_KEYDOWN/UP` 并携带物理扫描码 |
| 2026-07-24 | `PostMessage` 的 `[UIntPtr]0x57` PowerShell 转换失败 | 2 | 使用显式 `UIntPtr` 构造函数后重试同一目标窗口消息 |
| 2026-07-24 | Godot 属性探测崩溃残留 PID 46856，Stop-Process/CIM 返回拒绝访问 | 1 | 正常 Godot/后端已全部关闭；记录该无命令行残留项，Windows 重启后清理 |

## 5-Question Reboot Check
| Question | Answer |
|----------|--------|
| Where am I? | Phase 5：交付 |
| Where am I going? | 核对计划文件、已知环境限制和测试证据后向用户交付 |
| What's the goal? | 让腔镜板块真实显示随导丝尖端运动的实时血管腔内视图，并达到参考图视觉效果 |
| What have I learned? | 见 `findings.md` |
| What have I done? | 已实现并实机验证独立实时腔镜渲染、真实状态门控、底栏交互和随导丝推进变化，最终测试全绿 |

---
*每完成阶段、测试或出现错误后立即更新。*

## Session: 2026-07-24 — 用户人工测试运行

### Phase 6: 启动测试环境
- **Status:** complete
- Actions taken:
  - 重新完整读取 `planning-with-files` Skill、任务计划和最近进度。
  - 运行 `session-catchup.py`，发现上一轮 6 条未同步上下文；重新核对 `git diff --stat`、三份计划文件和当前进程状态。
  - 确认 TCP 9000 空闲；正常 Godot/uvicorn 均未运行，只有上一轮已记录的 PID 46856 残留项。
  - 使用 `cathsim-dev` Python 和工作区 `.tmp/warp` 缓存启动 uvicorn，保持前台工具单元运行。
  - `/api/v1/health` 返回 `status=ok`、`vpp_ready=true`、`case_001`。
  - 使用独立日志 `.tmp/godot/user-manual-test.log` 打开实际 Godot 4.7.1 客户端，并把窗口切至前台。
  - 确认 WebSocket 已连接，session `3c4d9dd0-3568-4612-b819-0d5b4d3b1d51` 已建立，收到 NewtonEngine `state_batch` 和 2152 个路径点。
  - 后端与 Godot 工具单元均保持运行，交由用户进行人工操作测试。
- Test Results:
  - `GET http://127.0.0.1:9000/api/v1/health`：通过；后端与 VPP 资产就绪。
  - Godot 运行日志：通过；真实血管 GLB 加载、WebSocket 连接、`session_started`、NewtonEngine `state_batch` 均成功。
- Errors:
  - `Get-CimInstance Win32_Process` 在非提升权限下返回拒绝访问；已改用 `Get-Process` 和端口监听检查。

### Phase 7: 血管内壁渲染优化方案
- **Status:** complete
- Actions taken:
  - 按用户要求只进入方案阶段，不修改 GDScript 渲染实现。
  - 复核腔镜相机、同轴头灯、ACES/glow/雾、三平面噪声材质、亮度控制和光学叠加层。
  - 对照 14% 路径位置实机截图，确认当前优势是真实几何与运动一致，主要短板是微表面、湿润高光、非对称照明、深度分层和运动稳定性。
  - 核对 Godot 4.7 官方材质、spatial shader、Forward+ 后处理和专用 Viewport 抗锯齿能力；确定“原生材质通道 → 小型 shader → 可选后处理”的渐进路线。
  - 完成 `doc/4.前端层/腔镜血管内壁渲染优化方案_2026-07-24.md`，覆盖真实性边界、候选路线、阶段 0～4、参数起点、性能预算、测试矩阵和回退策略。
  - 推荐默认采用多尺度三平面微表面、粗糙度/法线/低强度 clearcoat、偏轴辅光、半径自适应光照与雾、FOV 78～80°、2× MSAA；TAA 与高级后处理保持关闭或可选。
- Test Results:
  - 方案审查：通过；内容明确不改变真实 GLB 拓扑，不生成虚假分叉，并区分“视觉近似”与诊断级医学成像。
  - 实施门禁：通过；本阶段未修改任何 GDScript、场景、材质或运行配置，等待用户确认后再进入基线采集和代码实现。
- Files created/modified:
  - `doc/4.前端层/腔镜血管内壁渲染优化方案_2026-07-24.md`
  - `task_plan.md`
  - `findings.md`
  - `progress.md`

## Session: 2026-07-24 — 实施血管内壁渲染优化阶段 0～3

### Stage 0: 可复现基线
- **Status:** complete
- Actions taken:
  - 重新完整读取 `planning-with-files`、批准的优化方案以及三份持久计划文件。
  - 运行 `session-catchup.py`；检测到上一轮方案交付和本轮批准指令，已依据实际 Git 状态恢复。
  - 确认工作区原有修改和方案文档均保留；未执行 reset/checkout。
  - 建立阶段 0～3 的执行清单，阶段 4 高级后处理不在本轮范围内。
  - 复核 `main_controller.gd` 的基线材质、环境、灯光、相机同步和 `state_batch.path.vessel_radius` 数据入口。
  - 确认当前本地后端健康、Godot/Newton 会话在线，运行日志含真实状态批次和连续路径导航记录，可继续用于增强前后 A/B。
  - 通过 Godot 4.7.1 运行时属性反射确认原生法线噪声、粗糙度、clearcoat、三平面锐度和 mipmap 的准确属性名。
  - 复核 `camera_rig.gd` 和现有契约测试，确定优化只作用于私有腔镜相机，并同步升级测试锚点。
  - 在不改变渲染参数的前提下加入 300 帧一次性 `[ScopePerf]` 采样，准备记录旧渲染平均 FPS、平均帧时和 P95 帧时。
  - 关闭旧客户端并通过已验证的 Godot console 前台命令重启；后端保持运行，客户端重新进入真实 LIVE。
- Test Results:
  - 基线前端契约：`cathsim-dev/python -m pytest tests/test_frontend_contract.py -q` → `3 passed in 0.10s`。
  - 基线运行日志：未发现 `SCRIPT ERROR`/`ERROR:`；后端健康检查通过，Newton 状态流在线。
  - 基线视觉证据：保留 `scope-final-before-motion.png`、`scope-final-after-motion.png`、`scope-final-advanced.png`，三张 SHA-256 不同，可用于入口/推进/深入位置 A/B。
  - 基线性能采样脚本校验：`scripts/validate_godot.ps1` → `Godot validation passed`，真实 GLB 加载成功。
  - 300 帧性能基线：`[ScopePerf] profile=baseline quality=legacy frames=300 avg_fps=698.5 avg_ms=1.43 p95_ms=1.39`。
- Errors:
  - 首次调用 `C:\Users\Xu Bingxuan\.conda\envs\cathsim-dev\python.exe` 时未使用 PowerShell `&`，路径在空格处被截断；将使用调用运算符重跑。
  - 修复后使用 `& '完整路径'` 重跑成功，未再采用失败写法。
  - Godot 校验结束时因沙箱权限无法保存用户级 `editor_settings-4.7.tres`；项目校验已在该错误前明确通过，未修改或申请写入用户配置目录。
  - 使用 `Start-Process` 数组参数重启基线客户端后，45 秒内未生成指定日志；下一步检查进程/参数，不继续相同轮询。
  - 检查确认 `Start-Process` 新进程已退出且没有目标日志；改用与此前实机验证相同的 Godot console 前台保持方式后采样成功。

### Stage 1: 低风险材质升级
- **Status:** complete
- Actions taken:
  - 按 baseline/enhanced profile 和 performance/balanced/high 质量档设计独立材质工厂。
  - 新增 `scripts/rendering/endoscope_material_factory.gd`，生成无 UV 的世界三平面宏观颜色、粗糙度和微法线纹理。
  - 为 enhanced profile 加入 mipmap、各向异性过滤、低强度 clearcoat 和更低自发光；baseline 完整保留旧材质参数。
  - 主控制器改为通过 profile/quality 构建材质，并让亮度滑杆调用统一材质增益。
  - 升级前端契约测试，使其检查独立材质工厂与关键通道，而不是要求噪声实现留在主控制器。
- Test Results:
  - `cathsim-dev/python -m pytest tests/test_frontend_contract.py -q` → `3 passed in 0.12s`。
  - `scripts/validate_godot.ps1` → `Godot validation passed`；Godot 注册材质全局类并加载真实血管 GLB。
- Errors:
  - 阶段完成时跨三文件的复合进度补丁锚点匹配失败；读取磁盘尾部后拆分为三个小补丁完成同步。
- Files created/modified:
  - `godot_client/scripts/rendering/endoscope_material_factory.gd`
  - `godot_client/scripts/main_controller.gd`
  - `tests/test_frontend_contract.py`

### Stage 2: 照明与深度
- **Status:** complete
- Actions taken:
  - 设计相机同轴主灯、3 mm 偏轴补光以及局部半径低通/安全默认值。
  - 新增 `scripts/rendering/endoscope_lighting_rig.gd`，在私有相机下创建主灯和 3.5 mm 偏轴补光，保持固定质量档能量比例。
  - 从每个 `state_batch.path.vessel_radius` 保存有效半径，并以低通后的半径驱动灯光范围、衰减和能量。
  - 保存私有 `Environment` 引用，使增强模式动态调整 far 与指数雾；baseline 保持旧光学参数。
  - 亮度滑杆改为统一传给灯光组件，不再单独破坏主灯/补光比例。
  - 扩展契约测试，覆盖双灯、半径低通、range 公式与动态雾入口。
- Test Results:
  - `cathsim-dev/python -m pytest tests/test_frontend_contract.py -q` → `3 passed in 0.10s`。
  - `scripts/validate_godot.ps1` → `Godot validation passed`；Godot 注册照明全局类并加载真实血管 GLB。
- Files created/modified:
  - `godot_client/scripts/rendering/endoscope_lighting_rig.gd`
  - `godot_client/scripts/main_controller.gd`
  - `tests/test_frontend_contract.py`

### Stage 3: 相机稳定与质量档
- **Status:** complete
- Actions taken:
  - 设计位置临界阻尼、旋转独立平滑、投影 up 向量 roll 限幅以及 profile/quality 视口配置。
  - 用 Godot 4.7.1 运行时反射确认 `SubViewport.msaa_3d`、`use_taa` 以及 2×/4× MSAA 枚举可用。
  - 新增 `scripts/rendering/endoscope_camera_filter.gd`，实现位置临界阻尼、旋转独立响应和光轴 roll 速率限制。
  - 将断连、模型 teardown、手动重置统一接入滤镜 reset，防止跨会话沿旧姿态插值。
  - 增加三档私有视口配置：performance 无 MSAA、balanced 2×、high 4×；三档均关闭 TAA，并分别设置 FOV/glow。
  - 扩展契约测试，覆盖临界阻尼、roll 限制、2×/4× MSAA 与 TAA 关闭。
- Test Results:
  - `cathsim-dev/python -m pytest tests/test_frontend_contract.py -q` → `3 passed in 0.10s`。
  - `scripts/validate_godot.ps1` → `Godot validation passed`；Godot 注册相机滤镜类并加载真实血管 GLB。
- Errors:
  - 临时 Viewport 属性探针创建节点后未释放，退出时报告 RID/ObjectDB 泄漏；该探针不属于项目运行代码，已定位为临时节点生命周期问题。
- Files created/modified:
  - `godot_client/scripts/rendering/endoscope_camera_filter.gd`
  - `godot_client/scripts/main_controller.gd`
  - `tests/test_frontend_contract.py`

### Final validation: 真实运行、视觉与性能
- **Status:** complete
- Actions taken:
  - 准备以 enhanced/balanced、localhost 和真实 NewtonEngine 重新启动实际客户端。
  - 实际 Godot Forward+ 使用 RTX 4060 Laptop GPU 启动，重新建立 session `a1a85a5d-548f-482c-a0ee-7826bb1318ca` 并收到 NewtonEngine 首个真实 `state_batch`。
  - 经批准在交互权限下启动非 console 客户端，并把 `GODOT_USER_HOME` 定向到工作区 `.tmp/godot-user`；新 session `dcfd51c5-6f7e-49b9-baf2-eb397d39c26f` 正常进入 LIVE。
  - 因当前工具会话无法枚举 Godot 顶层窗口，选择复用 SubViewport 原生截图路径，增加显式命令行验证开关，而不捕获可能包含其他内容的整个桌面。
- Test Results:
  - enhanced/balanced 300 帧：`avg_fps=681.4`、`avg_ms=1.47`、`p95_ms=1.39`；相同启动方式下较 baseline 平均帧时增加 0.04 ms（约 2.8%），远低于 16.7 ms 预算。
  - 运行日志未出现 `SCRIPT ERROR`、解析错误或无效属性；真实 GLB、WebSocket、session 和 Newton 状态流均正常。
  - 工作区用户目录复测的 300 帧结果：`avg_fps=716.7`、`avg_ms=1.40`、`p95_ms=1.67`；未再出现 shader cache 保存错误。
  - 自动 SubViewport 截图功能正常：保存 `scope_validation_enhanced_balanced.png`，日志无脚本错误。
  - 首次增强视觉验收：失败；画面大面积暗红，真实腔口和褶皱只集中在中央小区域，需调光/材质回归后重测。
  - 增加命令行 profile/quality 覆盖，准备在完全相同的自动 SubViewport 截图流程下生成 baseline 对照。
  - baseline 自动 SubViewport 对照已生成：画面全域可见真实壁面纹理，确认 enhanced 初版暗化是材质/光程调参问题而非相机进入不同位置。
  - 第一轮提高灯光 range/energy、far、自发光并降低 normal/clearcoat 后重新截图；画面仍大面积暗红，视觉验收继续失败，转入法线通道隔离诊断。
  - 临时关闭 enhanced normal map 后截图仍过暗，排除法线通道为主因；转而修正三平面颜色的空间尺度和亮度覆盖。
  - 恢复更细的颜色周期和 scale=70 后，增强画面全域壁面纹理与纵深恢复，第二轮视觉调优通过基础可读性门槛；准备重新启用低强度 normal 做最终对照。
  - 重新启用 balanced 微法线后截图视觉保持稳定，未复现暗化；300 帧为 `avg_ms=1.54`、`p95_ms=1.52`，下一步量化亮度并做真实运动对照。
  - 直方图确认 enhanced 中位亮度仅 15.31/255、p95 19.56 且无过曝；决定使用私有环境 tonemap exposure 提亮，而不是继续用自发光压平表面。
  - balanced exposure=2.0 后中位亮度提高到 26.01、暗像素降至 0.07%、仍无过曝；视觉可读性通过，最后微调红橙色平衡。
  - 自动真实运动模式完成：60 个 `push=0.28, rot=0.18` 控制帧，前后截图 98.74% 像素不同且哈希不同；相机/纹理随 Newton 状态连续变化。
  - 差分统计发现绿色通道近乎为 0，画面仍偏纯红；需完成 RGB 通道诊断后再冻结最终色彩参数。
- Errors:
  - GPU 启动时出现一次 `_save_to_cache` 的 `Condition "f.is_null()"`，但后续渲染和性能采样正常；将用工作区 Godot 用户目录复测，区分沙箱缓存写权限与项目 shader 错误。
  - 首次窗口截图仅匹配非 console Godot 进程名，未找到 GUI；将先枚举实际窗口句柄再捕获。
  - 枚举 PID 53152/53040 的所有顶层窗口仍为空，确认 console 前台工具单元可渲染但没有交互桌面窗口；改用参数整体引用的非 console `Start-Process`。
  - 修正参数引用并设置工作区 `GODOT_USER_HOME` 后，沙箱内非 console 进程仍立即退出且没有日志；判定为 GUI 交互桌面权限边界，下一次改为申请提升权限启动。
  - baseline console 自动截图反复返回保存错误码 12：`user://` 仍指向不可写 AppData；终止该进程并改为显式工作区输出参数。
  - Environment 临时属性探针误对 RefCounted 调用 `free()`，导致探针脚本超时；属性已取得，已移除错误释放，不涉及项目运行脚本。
  - 把 NoiseTexture 配置顺序改为 ramp→noise 后，首次 enhanced RGB 捕获 35 秒内未完成；先查日志，不重复等待相同条件。
  - 日志和健康检查确认根因是原 uvicorn 保持进程达到 1 小时时限，非材质或脚本错误；将重启后端并复用当前客户端自动重连。
  - 截图格式诊断确认 viewport=RGB8、albedo=RGBA8，转换 PNG 不是绿色通道丢失原因；继续采样实际 albedo 像素。
  - albedo 像素确认含 RGB；临时 unshaded 输出也恢复橙色，定位为真实网格外向法线在腔内背面没有进入正常光照，而不是纹理、PNG 或色调映射问题。
  - 原生 `CULL_FRONT` 隔离仍为纯红，正式触发方案中的小型 spatial shader 路线；baseline StandardMaterial 保持不变。
  - 新增 `endoscope_wall.gdshader`，显式内向法线、三平面颜色/粗糙度/微法线和湿润高光；首次 custom light 编译发现 `CLEARCOAT` 作用域限制。
  - 记录该错误的首次跨文件补丁锚点不匹配，已读取实际文件尾部并改用小补丁。
  - 修复策略：light 函数直接读取 `clearcoat_strength` uniform，不读取 fragment 输出。
  - 引擎 custom light 仍未恢复 RGB，改为 shader 内基于真实内向法线、相机位置和偏轴方向计算光照；截图首次恢复橙红色，meanRGB≈`(219.4, 52.9, 23.7)`。
  - 手动光首版高光偏亮并有红通道裁剪，继续降低 exposure/光照系数后再冻结。
  - 最终 balanced exposure=0.62 并降低光照系数后，meanRGB≈`(187.1, 32.1, 15.0)`、红通道 0% 裁剪；亮橙高光和暗红阴影同时保留。
  - 最终静态 300 帧性能：`avg_ms=1.83`、`p95_ms=1.56`，满足性能预算。
  - 最终真实运动回归完成：60 个 `push=0.28, rot=0.18` 控制帧，前后截图 99.95% 像素不同，mean abs RGB≈`(32.1, 28.8, 9.1)`。
  - 运动期间性能：`avg_ms=2.27`、`p95_ms=4.17`；运动后仍保持橙红内壁、褶皱、高光和纵深。
  - 最终契约首次回归为 `1 failed, 2 passed`：测试仍匹配修改前的 `render_mode cull_front` 连续字符串；实现中的 `unshaded, cull_front` 合法，更新契约为独立语义断言后重跑。
  - 第二次契约回归仍为 `1 failed, 2 passed`：遗留 `CLEARCOAT` 断言未随手动湿润高光实现更新；Godot 校验同时通过，测试改查 `clearcoat_strength` 与 `wet_highlight` 后重跑。
  - 修复契约后 `tests/test_frontend_contract.py` → `3 passed in 0.10s`。
  - performance/balanced/high 三档及 baseline 回退 profile 均完成 Godot headless 启动：退出码 0、profile 日志匹配、0 个 shader/script/parse/invalid 错误。
  - `git diff --check` 通过，仅有 Windows LF→CRLF 提示；未执行 reset/checkout，用户既有未提交修改保持不变。
  - 最终实际客户端 PID 23664 已启动并保持运行；profile=`enhanced`、quality=`balanced`，session `636d51ef-4bc5-4db8-8ce7-bc686c849dbc` 收到 NewtonEngine 首个 `state_batch`。
- Files created/modified:
  - `godot_client/scripts/rendering/endoscope_material_factory.gd`
  - `godot_client/scripts/rendering/endoscope_lighting_rig.gd`
  - `godot_client/scripts/rendering/endoscope_camera_filter.gd`
  - `godot_client/scripts/rendering/endoscope_wall.gdshader`
  - `godot_client/scripts/main_controller.gd`
  - `tests/test_frontend_contract.py`
  - `doc/4.前端层/腔镜血管内壁渲染优化方案_2026-07-24.md`
  - `task_plan.md`、`findings.md`、`progress.md`

## 2026-07-24 中断恢复校验
- 按 `planning-with-files` 重新读取 `task_plan.md`、`findings.md` 与 `progress.md`；阶段 0～3 及交付阶段均已完成，没有遗留代码实施项。
- 当前 Godot 客户端 PID `23664` 仍在响应，运行配置为 `enhanced/balanced`。
- 后端 `/api/v1/health` 返回 `status=ok`、`vpp_ready=true`、`case_001`。
- 最终运行日志仍包含 `session_started`、`engine=NewtonEngine` 与首个真实 `state_batch`；人工测试环境保持可用。
- 恢复后重跑 `tests/test_frontend_contract.py -q`：`3 passed in 0.05s`。
- `git diff --check` 通过，仅报告既有的 Windows LF→CRLF 提示。
- 测试后再次确认客户端 PID `23664` 正常响应，后端健康状态仍为 `ok`；当前可继续人工测试。
- Errors:
  - 首次追加恢复记录时使用了文件中不存在的末尾锚点，`apply_patch` 校验失败；读取实际文件尾部后改用现有文件清单作为锚点完成追加。

## 2026-07-24 再次中断恢复
- **Status:** complete
- Actions taken:
  - 完整读取 `planning-with-files` 技能以及三份持久化任务文件。
  - 运行 `session-catchup.py`，确认未同步上下文只涉及此前已完成的恢复与交付消息。
  - 运行 `git diff --stat` 并复核工作区；阶段 0～3 的实现仍在，未执行 reset/checkout。
  - 后端 `/api/v1/health` 返回 `status=ok`、`vpp_ready=true`、`case_001`。
  - 发现此前人工测试客户端 PID `23664` 已退出；本轮从重新启动实际客户端继续。
  - 重新启动实际 Godot GUI，PID `50972`，profile=`enhanced`、quality=`balanced`。
  - 新 session `40185414-efcd-4b8f-a1d4-b6b806d73f50` 已收到 `engine=NewtonEngine` 的首个真实 `state_batch`。
- Test Results:
  - `pytest tests/test_frontend_contract.py -q` → `3 passed in 0.04s`。
  - 新运行日志扫描 → 未发现 shader、script、parse 或 invalid 错误。
  - 恢复后端健康复核 → `status=ok`、`vpp_ready=true`。
  - `git diff --check` → 通过，仅有 Windows LF→CRLF 提示。
- Errors:
  - 首次同步完成状态时 `findings.md` 锚点与实际末尾不一致，复合补丁未应用；读取三个文件实际尾部后拆分并使用现有段落锚点。

## 2026-07-28 MuJoCo 分割兼容与旧测试迁移
- **Status:** complete
- Actions taken:
  - 在 `src/cathsim/dm/observables.py` 增加受限兼容解码：白色/非法分割 ID 映射为背景，其他类型的 `IndexError` 不被吞掉。
  - 迁移 `test_data.py`、`test_dm_env.py`、`test_guidewire.py`、`test_gym_env.py`、`test_phantom.py`、`test_sb3_train.py`、`test_scene.py`、`test_utils.py` 到当前包路径、Gymnasium 与 RL API。
  - 更新 `test_task.py` 使用 `get_camera_matrix()` 的返回值，并释放测试相机场景。
  - 修复 `Scene.add_light()` 显式传入 `castshadow` 时的重复关键字问题。
  - 在 pytest 配置加入仓库根目录，确保顶层 `services/` 与 `tools/` 可从直接 pytest 入口收集。
- Test Results:
  - 8 个迁移文件：`37 tests collected`，0 collection error。
  - 完整测试收集：`298 tests collected in 11.30s`，0 collection error。
  - 迁移文件 + 原 `test_task.py`：`44 passed`。
  - 收窄异常兼容路径后，分割/Task/Scene：`12 passed in 21.49s`。
  - 最终全量：`297 passed, 1 skipped, 6 warnings in 130.92s`。
- Errors:
  - 第一轮针对性测试发现 7 个额外旧契约断言；按当前实现分别更新，并修复 `add_light` 的真实兼容缺陷。
  - 直接 `pytest.exe` 首次完整收集有 15 个顶层包导入错误；配置 `pythonpath = ["."]` 后完整收集通过。
- Scope protection:
  - 未改动 RL 训练算法、数据采集/回放业务、DSA 面板或 Godot 前端；现有用户前端/腔镜未提交变更保持原样。
