# Task Plan: 实时血管内腔镜视图

## Goal
完善 Godot 前端“腔镜实时视图”板块，使其以导丝尖端的实时位姿驱动血管腔内相机，真实呈现随导丝运动变化的血管内视图，并达到参考图所示的医疗监控面板视觉效果。

## Next Step
修复已完成；当前 9000 后端与 Godot 客户端已切换到新实现，可由用户继续推进导丝进行人工观察。

## Current Phase
Phase 17 complete

## Phases

### Phase 1: 需求与现状发现
- [x] 记录用户目标和参考图视觉要求
- [x] 检查当前 Git 工作区，区分既有修改与本任务改动
- [x] 定位腔镜面板、三维血管、导丝和实时状态数据流
- [x] 将代码与资产发现记录到 findings.md
- **Status:** complete

### Phase 2: 技术方案与验收设计
- [x] 确定真实腔内相机的坐标、朝向、近远裁剪和运动平滑方案
- [x] 确定血管内表面材质、照明和分叉可见性的实现方式
- [x] 确定参考图面板标题、LIVE、底栏控件与视口布局
- [x] 定义自动测试与实际 Godot 渲染验收项
- **Status:** complete

### Phase 3: 实现
- [x] 实现导丝尖端驱动的实时腔内 Camera3D/SubViewport 渲染
- [x] 实现血管腔内材质、灯光和必要的导丝尖端可视化
- [x] 完善腔镜面板 UI 与底栏交互/状态展示
- [x] 增补或调整前端契约测试
- **Status:** complete

### Phase 4: 测试与可视化验证
- [x] 运行针对性测试并记录命令、结果和失败原因
- [x] 运行 Godot headless 导入/脚本校验
- [x] 启动实际项目，验证导丝运动时腔镜画面与位姿同步
- [x] 对照参考图检查布局、可读性和实时状态
- **Status:** complete

### Phase 5: 交付
- [x] 复核 Git diff，确保未覆盖用户既有修改
- [x] 核对任务清单、测试证据和已知限制
- [x] 更新三份计划文件并向用户交付结果
- **Status:** complete

### Phase 6: 用户人工测试运行
- [x] 启动本地 uvicorn/Newton 后端并确认 `/api/v1/health`
- [x] 打开嵌套 `godot_client/project.godot` 实际窗口
- [x] 确认 `session_started`、`state_batch` 与腔镜 LIVE
- [x] 保持前后端运行并向用户交付测试入口
- **Status:** complete

### Phase 7: 血管内壁渲染优化方案
- [x] 复核当前材质、灯光、环境、相机与光学叠加参数
- [x] 对照实机截图识别视觉短板
- [x] 核对 Godot 4.7 可用的材质与后处理能力
- [x] 撰写分阶段实施、验收、性能预算和回退方案
- [x] 仅交付方案，等待用户确认后再改代码
- **Status:** complete

### Phase 8: 按批准方案实施阶段 0～3
- [x] 阶段 0：建立可复现的代码、视觉、状态流和性能基线
- [x] 阶段 1：实现多尺度三平面颜色/法线/粗糙度与低强度湿润层
- [x] 阶段 2：实现偏轴补光、局部半径自适应以及 glow/雾深度调整
- [x] 阶段 3：实现位置/旋转分离阻尼、roll 限制、2× MSAA 和三档质量配置
- [x] 运行前端契约、Godot 4.7.1、真实 Newton 运动与视觉/性能回归
- [x] 更新方案实施状态、三份计划文件并交付
- **Status:** complete

### Phase 9: 中断后恢复人工测试现场
- [x] 重新读取三份持久化文件并运行 session catchup
- [x] 确认阶段 0～3 没有遗留实施项
- [x] 确认本地 Newton 后端健康且 `vpp_ready=true`
- [x] 重新启动 enhanced/balanced 实际 Godot 客户端
- [x] 复核新会话的 `session_started`、`state_batch` 与运行日志
- [x] 同步恢复结果并交付人工测试入口
- **Status:** complete

## Key Questions
1. 当前“腔镜实时视图”是静态占位图、程序绘制效果，还是已有独立 SubViewport？
2. `state_batch` 是否直接提供导丝尖端姿态；若只有点列，怎样以末端切线和稳定上方向构造相机姿态？
3. 现有血管模型是否包含可从内部渲染的表面，还是需要双面材质/反转剔除或程序化腔内代理几何？
4. 如何在不伪造医学成像的前提下实现参考图的红橙色腔内光照与实时运动效果？
5. 当前用户未提交的前端改动与本任务可能重叠在哪里，怎样最小化冲突？

## Decisions Made
| Decision | Rationale |
|----------|-----------|
| 使用项目根目录的三份计划文件 | 用户明确要求，且 planning-with-files 规定复杂任务先持久化计划 |
| 参考图只作为视觉和布局目标，不作为静态背景图 | 用户要求真实随导丝运动变化的实时血管内腔镜视图 |
| 在确定实现前先追踪 Godot → WebSocket `state_batch` → 导丝显示的数据流 | 必须让相机使用真实前端状态，避免与导丝模型脱节 |
| 保留现有未提交修改并对重叠文件逐项审查 | 当前工作区已有用户开发内容，不能覆盖或回退 |
| 腔镜 SubViewport 改为 `own_world_3d=true` | 专用环境、雾、灯光和血管副本不再受主 3D 相机模式影响，也消除透明 fallback 叠加的不确定性 |
| 保留真实 tip/末端切线作为相机来源，并仅对专用相机做插值平滑 | 保证画面与导丝状态一致，同时降低物理解算细小抖动造成的眩晕 |
| 无有效连接、tip 或血管模型时隐藏实时视口并显示明确状态 | 禁止无数据时继续显示拟真 fallback 和伪 LIVE |
| 复用并扩展 `PaneToolIcon` 的线绘图标 | 与当前医疗工业风一致，避免 emoji/平台字体差异 |
| 按用户批准范围只实施优化方案阶段 0～3 | SSAO、SSIL、景深和更重的自定义后处理属于阶段 4，本轮不默认启用 |
| 保留 baseline/enhanced 与 quality preset 回退入口 | 视觉调优具有设备差异，必须能够单独回退材质、灯光和质量设置 |
| enhanced 升级为小型 spatial shader 并显式翻转内向法线 | 实机 A/B 已证明 StandardMaterial 在真实 GLB 背面无法获得正常 RGB 光照，`CULL_FRONT` 也无效；方案明确允许原生材质不足时采用小型 shader |

## Errors Encountered
| Error | Attempt | Resolution |
|-------|---------|------------|
| 更新视觉发现时 `apply_patch` 复合锚点匹配失败 | 1 | 读取磁盘现状，改为按段落顺序的小锚点补丁，避免重复原失败操作 |
| 阶段同步复合补丁把 Test Results 锚点放错文件 | 2 | 第三次改为逐文件小补丁并分别验证，不再使用跨三文件复合更新 |
| Godot 4.7.1 报告 `OmniLight3D.cull_mask` 为无效属性 | 1 | 私有 World3D 已天然隔离灯光；移除不兼容掩码后第二次校验通过 |
| 本地 uvicorn 隐藏进程 30 秒内未通过健康检查 | 1 | 检查退出状态与重定向日志，确定是启动耗时、依赖还是进程参数问题 |
| Godot 属性探测崩溃后残留进程 46856 无法通过 Stop-Process/CIM 结束 | 1 | 正常 GUI 与后端进程均已关闭；残留项无命令行/父进程，记录为环境限制，重启 Windows 可清理 |
| 人工测试启动前使用 CIM 枚举进程被当前权限拒绝 | 1 | 改用 `Get-Process` 与端口监听检查；确认 9000 空闲，只有既知 PID 46856 残留项 |
| 阶段 0 基线 pytest 使用含空格的 Conda 解释器路径时未加调用运算符 | 1 | PowerShell 将路径截断为 `C:\Users\Xu`；改用 `& '完整路径'` 后重跑，不重复原命令 |
| Godot 校验结束时无法写用户级 `editor_settings-4.7.tres` | 1 | 项目脚本、GLB 与 headless 启动均已通过；该路径不在工作区写权限内，记录为非项目错误，不提升权限修改用户配置 |
| 基线性能客户端重启后 45 秒未生成指定日志 | 1 | 不重复盲等；检查新进程退出状态和实际日志位置，再改用已验证的前台 console 启动参数 |
| 阶段 1 完成后的三文件复合进度补丁锚点匹配失败 | 1 | 补丁把 `progress.md` 段落误置于 `findings.md` 更新上下文；拆为逐文件小补丁并按磁盘内容更新 |
| Viewport 属性探针退出时报告 RID/ObjectDB 泄漏 | 1 | 探针创建了未释放的临时 SubViewport；属性名与枚举已成功取得，给临时节点调用 `free()` 后不把该警告带入项目实现 |
| enhanced 实机启动时 Godot shader cache 报 `Condition "f.is_null()"` | 1 | 发生在连接前且渲染/状态流/性能采样继续正常；按用户目录写权限问题处理，最终重启时把 `GODOT_USER_HOME` 指向工作区 `.tmp` 验证 |
| enhanced 窗口截图首次按非 console 进程名查找失败 | 1 | 当前前台由 console 可执行文件保持，未找到指定进程名；枚举实际 Godot 进程及窗口句柄后按非零句柄捕获，不重复硬编码进程名 |
| 沙箱内非 console `Start-Process` 启动后立即退出且无日志 | 1 | 参数引用已修正但 GUI 仍不能进入交互桌面；按 GUI 应用权限边界改为请求提升权限启动，不继续沙箱内重复启动 |
| enhanced 初次 SubViewport 视觉截图过暗、有效管腔只占中央小区域 | 1 | 运行/性能虽通过但视觉验收失败；对比 baseline 并分离检查微法线、灯光 range/energy、自发光与半径适配，不以“测试全绿”误判完成 |
| 沙箱 console 的 baseline 自动截图无法写 `user://`，错误码 12 | 1 | console 会话仍把 `user://` 解析到不可写 AppData；新增显式 `--scope-validation-output=<workspace path>`，直接写工作区，不继续重试 user 目录 |
| Environment 属性探针调用 `free()` 导致 RefCounted 脚本错误并超时 | 1 | `Environment` 由引用计数管理，移除显式 free；所需属性已成功输出，不把临时探针错误归因于项目代码 |
| 调整 NoiseTexture 初始化顺序后 35 秒未生成 enhanced RGB 验证图 | 1 | 日志显示 WebSocket closed 且健康检查失败；原后端恰到 1 小时工具时限，重启同一 `cathsim-dev` 后端后利用客户端自动重连继续 |
| 自定义 `light()` 首次编译报 `Unknown identifier: CLEARCOAT` | 1 | `CLEARCOAT` 是 fragment 输出，不能在 light 函数读取；改用已声明的 `clearcoat_strength` uniform 后重新编译 |
| 记录 shader 错误的跨文件补丁锚点匹配失败 | 1 | `progress.md` 的错误行位置与假设不一致；读取实际尾部后拆成逐文件补丁 |
| 最终契约仍断言旧的连续文本 `render_mode cull_front` | 1 | shader 已合法加入 `unshaded, cull_front`；把契约改为分别断言 `render_mode unshaded` 和 `cull_front`，验证语义而非旧排列 |
| 第二次契约仍断言已替换的引擎输出 `CLEARCOAT` | 1 | 最终 unshaded shader 用 `clearcoat_strength` 计算 `wet_highlight`；契约改查这两个实际湿润高光语义 |

## Notes
- 参考图要求：深色蓝黑面板、左上步骤编号 2 与“腔镜实时视图”、右上绿色 LIVE、中央红橙色血管腔内画面、底部录制/计时/亮度/滑杆/截图/全屏控件。
- “真实”在本项目中定义为：画面由当前血管几何和导丝尖端实时位姿渲染，随推进与旋转变化；不得仅替换为静态图片或循环视频。
- 每完成一个阶段立即更新本文件，并在 progress.md 记录测试命令与结果。
- 自动验收：`tests/test_frontend_contract.py` 必须断言独立 World3D、真实血管父级、状态门控、动态计时/亮度/截图/全屏以及无拟真 fallback。
- Godot 验收：运行 `scripts/validate_godot.ps1` 完成资产导入和 headless 启动，日志不得含 `SCRIPT ERROR`/`ERROR`。
- 视觉验收：使用当前嵌套工程实际运行，连接后截图必须显示真实非规则血管内壁；推进/旋转前后两帧应有可观察变化，断连或未收到 tip 时必须显示等待状态而非 LIVE。

### Phase 10: 2026-07-27 当前工作树本地回归测试
- [x] 复核测试入口、解释器、Godot 与当前进程状态
- [x] 运行 Python 自动化测试与静态差异检查
- [x] 运行 Godot 资源导入和 headless 项目校验
- [x] 启动本地后端并验证健康接口、WebSocket、Newton 状态流
- [x] 汇总问题、修复（如有）并完成复测
- **Status:** complete
- 测试完成；遗留的旧测试收集/渲染兼容问题已记录，本轮未获授权修改旧功能代码。

### Errors for Phase 10
| Error | Attempt | Resolution |
|-------|---------|------------|
| 规划文件首次复合补丁锚点匹配失败 | 1 | 读取 UTF-8 实际文件头尾，改用现有精确锚点拆分更新，不重复原补丁 |
| 技能说明中的全局 session-catchup 路径不存在 | 1 | 改用仓库内 `.codex/skills/planning-with-files/scripts/session-catchup.py`，恢复成功 |
| 全量 pytest 收集被 8 个旧 CathSim/MuJoCo 测试模块中断 | 1 | 保留原始失败证据；将这 8 个收集级失败模块显式隔离，继续运行其余 261 个已成功收集的测试，不把“无法收集”和当前功能回归混为一谈 |
| 可执行测试中 15 项因系统 pytest 临时目录 WinError 5 而 setup error | 1 | 第二轮显式使用工作区内全新 `--basetemp=.tmp/pytest-local-20260727`，避免不可写的用户 Temp 目录 |
| 4 个旧 `test_task.py` 相机用例因 dm_control segmentation id 16777215 越界失败 | 1 | 核对 MuJoCo/dm_control 实装版本与锁定版本；与环境权限错误分开诊断 |
| Godot 第一次校验显式使用非 console EXE，`$LASTEXITCODE` 为空并误报导入失败 | 1 | 检查同目录 console 版；改用可同步等待并返回退出码的 Godot console EXE，不重复原启动方式 |
| Godot console 校验在沙箱内因 Windows 根证书库与 AppData 设置不可访问而被错误日志拦截 | 2 | 按权限边界在沙箱外复跑同一 headless 导入/校验，退出码 0 且项目日志通过 |
| 旧 `spikes/ws_probe.py` 未指定 Newton 且把 `aorta_tree` target 写为 `endpoint_9`，会话启动落入 MuJoCo 后失败 | 1 | 不改旧探针；读取当前 `SessionStartData`/后端解析契约，下一轮显式使用 `physics_engine=newton_demo` 和受支持字段 |

### Phase 11: 2026-07-27 启动本地前后端并展示
- [x] 恢复会话并复核工作树、9000 端口、Godot 进程和 localhost 配置
- [x] 启动并保持本地 uvicorn/Newton 后端
- [x] 验证健康接口与 GPU/Warp 初始化
- [x] 使用嵌套 `godot_client/project.godot` 启动可交互 Godot 窗口
- [x] 验证 WebSocket、session_started、NewtonEngine 与首个 state_batch
- [x] 保持前后端运行并交付本地操作入口
- **Status:** complete

### Errors for Phase 11
| Error | Attempt | Resolution |
|-------|---------|------------|
| `rg` 搜索把以 `--` 开头的模式和 `-g` 位置解析为选项/路径 | 1 | 使用 `rg -n -- <pattern>` 获取所需启动参数证据，不重复错误参数排列 |
| `Get-Process.MainWindowHandle` 对 Godot 返回 0 | 1 | 使用 Win32 `EnumWindows` 按 PID 34888 找到 `CathSim VPP Client (DEBUG)` 窗口，并成功 `SetForegroundWindow` |

### Phase 12: 修复 MuJoCo 分割兼容与旧测试收集
- [x] 恢复上下文并确认目标源码/测试文件没有用户未提交修改
- [x] 为 dm_control 的 0xFFFFFF 背景分割值实现兼容修复并补单元测试
- [x] 将 8 个旧测试文件迁移到当前包路径、Gymnasium 和 RL API
- [x] 让完整测试集无 collection error
- [x] 运行分割、MuJoCo、RL 针对性回归
- [x] 使用工作区 basetemp 运行全量 pytest 并解决剩余失败
- [x] 复核 diff、记录兼容性边界并交付
- **Status:** complete

### Errors for Phase 12
| Error | Attempt | Resolution |
|-------|---------|------------|
| 递归 `rg` 搜索多个符号在当前 Windows 工作树超时 | 1 | 改用 `Get-ChildItem -Recurse -Filter *.py | Select-String` 定位符号，不重复原扫描 |
| 第一轮 44 项针对性测试中 7 项仍使用旧契约 | 1 | Phantom 断言改用分层默认配置；Scene 的已删除 `regenerate` 测试改为相机集合；相机矩阵比较使用方法返回值；同时修复 `add_light` 重复传入 `castshadow` 的真实缺陷 |
| Conda 的 `pytest.exe` 未把仓库根目录加入 `sys.path`，15 个服务/工具测试收集失败 | 1 | 在 pytest 项目配置中显式加入 `pythonpath = ["."]`，统一直接入口与 `python -m pytest` 的导入行为 |

### Phase 13: 偏离中心线始终为 0.0 mm 的只读诊断
- [x] 复核当前分支、工作树、既有规划和前端交接边界
- [x] 定位 HUD 指标的数据字段、格式化逻辑与刷新入口
- [x] 追踪 WebSocket `state_batch` 到物理引擎的真实数据来源
- [x] 用现有测试/运行日志或最小只读探针验证数值是否真实变化
- [x] 区分设计语义、当前实现和异常根因并给出结论
- **Status:** complete

### Errors for Phase 13
| Error | Attempt | Resolution |
|-------|---------|------------|
| 递归枚举交接文档时进入旧 `.tmp/pytest-local-20260727` 被拒绝访问 | 1 | 已取得目标交接文档路径；后续搜索显式限定源码/文档目录，不再扫描该临时目录 |
| 第一次并行进程/日志查询被一个非零 PowerShell 子命令中断 | 1 | 拆为限定目录且显式容错的只读查询，成功取得健康、PID 和日志证据 |
| 首次最小探针混入最近顶点的沿程采样误差，得到 500 mm | 1 | 把 tip 放到既有路径顶点并仅增加 0.5 mm 横向偏移；重跑得到 reported=0.000 mm、independent=0.500 mm |

### Phase 14: 修复 Newton 真实偏离中心线计算
- [x] 获得用户实施确认并恢复上一轮诊断上下文
- [x] 确认共享路径几何 API 与现有测试契约
- [x] 解耦 `raw.arclen` 进度与真实 tip 横向偏差
- [x] 使用最近折线段投影避免最近顶点采样误差
- [x] 补充 Kinematic、Newton-style arclen 和 off-path 回归测试
- [x] 运行相关 pytest 与完整回归
- [x] 通过实际 Newton/WebSocket 状态验证非零动态偏差
- [x] 复核差异、更新三份记录并交付
- **Status:** complete

### Errors for Phase 14
| Error | Attempt | Resolution |
|-------|---------|------------|
| 组合式停止 9000 旧后端并启动新后端的 PowerShell 命令在执行前被本地策略拦截 | 1 | 旧进程未受影响；不绕过策略，改在 9001 启动当前工作区代码做独立 Newton/WebSocket 验证 |

### Phase 15: 切换修复后端并启动 Godot 展示
- [x] 再次确认 9000/9001 端口进程身份和健康状态
- [x] 按用户明确要求停止旧 9000 与临时 9001 验证服务
- [x] 从当前工作区修复代码启动新 9000 后端
- [x] 验证 health=`ok`、vpp_ready=true、case_001
- [x] 发现无 Godot 进程后从正确内层项目启动可交互窗口
- [x] 验证 RTX 4060、WebSocket、session_started、NewtonEngine、首个 state_batch
- [x] 保持新后端与 Godot 窗口运行供用户操作
- **Status:** complete

### Errors for Phase 15
| Error | Attempt | Resolution |
|-------|---------|------------|
| 首次端口预检的 PowerShell foreach 后直接接管道导致 Empty pipe element | 1 | 命令未执行任何进程操作；改为先收集 `$rows` 再格式化输出，第二次成功 |

### Phase 16: 接触力始终为 0 的只读诊断
- [x] 恢复现有工作区上下文并记录既有未提交修改
- [x] 定位 Godot HUD 接触力字段、单位换算与刷新入口
- [x] 追踪 WebSocket/NavigationEngine 到物理引擎的接触力来源
- [x] 用针对性测试或最小只读探针验证根因
- [x] 区分显示问题、数据传输问题与物理解算问题并给出修复建议
- **Status:** complete

### Errors for Phase 16
| Error | Attempt | Resolution |
|-------|---------|------------|
| `rg` 同时传入不存在的 `tests/test_newton*` 路径模式后返回退出码 1 | 1 | 已保留有效匹配输出；改用 `Get-ChildItem -Filter '*newton*.py'` 取得实际测试文件名，后续使用精确路径 |
| 首次聚合健康/进程/递归日志查询因部分 PowerShell 枚举项非零退出 | 1 | 健康与监听信息已取得；改为限定 `.tmp/runtime`、`.tmp/godot` 且 JSON 输出，成功确认进程与日志 |
| 首轮并行验证使用了错误的 pytest 类节点名，pytest 未收集到用例并使聚合调用提前报错 | 1 | 未执行任何测试修改；先用 `--collect-only` 获取精确 node id，并确认并行的隔离探针是否仍在运行/是否已建会话，再分别复跑 |
| 隔离 WebSocket 探针把 `session_started.data` 的会话标识误写为 `session_id`，触发 KeyError | 1 | `finally` 已收到 `session_stop=confirmed`，无遗留会话；读取实际协议键名后修正探针，不重复错误字段 |
| VBD 直接探针首次把 `PlannedPath` 作为 NewtonEngine 位置参数传入，构造器只接受关键字参数 | 1 | 引擎尚未初始化且无 GPU/会话资源遗留；按签名改为 `NewtonEngine(path=...)` 后重跑 |

### Phase 17: 接入 Newton 真实接触力并分离穿壁安全量
- [x] 恢复 Phase 16 诊断、既有偏离中心线修复和当前运行现场
- [x] 扩展 RawPose/NavigationState，显式传递 `wall_penetration`
- [x] 在 Newton 8-substep VBD 循环中采集并稳定聚合尖端真实接触反力
- [x] 将 NavigationEngine/RiskAssessor 安全判断改为独立 penetration
- [x] 更新 WebSocket mechanics/safety 字段与兼容契约
- [x] 增加单元与协议回归测试
  - [x] 增加并通过 GPU Newton 回归测试
  - [x] 运行全量 pytest、隔离真实 WebSocket 与 Godot 验证
  - [x] 复核差异并交付，不提交、不上传
  - **Status:** complete

### Errors for Phase 17
| Error | Attempt | Resolution |
|-------|---------|------------|
| 参数/文档 `rg` 搜索取得有效结果但因部分路径/匹配返回退出码 1 | 1 | 已保留有效阈值证据；后续限定源码和已知文件，不重复宽泛路径组合 |

### Phase 18: 3D 解剖导航左键平移中心模式
- [x] 用户批准实施并完成交接文档、工作树和现有相机输入链路复核
- [x] 将选择/旋转布尔状态升级为选择/旋转/平移三态工具模式
- [x] 新增平移工具按钮、线框图标与明确提示文字
- [x] 复用现有中键平移数学，并让切回旋转后保留用户平移中心
- [x] 更新前端设计说明与静态契约测试
- [x] 运行前端 pytest、Godot 4.7.1 解析/启动和差异检查
- [x] 启动真实 Newton/FastAPI/Godot，完成按钮截图与平移后旋转行为探针验收
- [x] 复核冻结范围和既有未提交改动，交付但不提交、不上传
- **Status:** complete

### Errors for Phase 18
| Error | Attempt | Resolution |
|-------|---------|------------|
| 方案阶段首次按默认用户目录调用 `session-catchup.py`，实际技能安装在仓库 `.codex/skills` | 1 | 改用仓库内实际脚本路径；本实施阶段恢复检查已直接成功 |
| 方案阶段两次把 PowerShell 不展开的 `*.gd` 传给 `rg`，并假设了错误的 `ui_style.gd` 路径 | 1 | 后续统一使用 `rg --files` 或精确文件路径；未对代码和工作树造成影响 |
| Godot 首轮解析无法从未声明类型的方向数组推断 `pan` 图标局部变量 `tip` | 1 | 将 directions 声明为 `Array[Vector2]`，循环变量和 `tip` 显式声明为 `Vector2` 后重新解析 |
| GUI 自动拖拽首轮使用 `SetCursorPos`，按钮切换成功但 3D 区域前后采样 0% 像素变化 | 1 | Godot 原始鼠标流没有得到有效 relative；改用相对 `mouse_event(MOVE)`，并增加独立 Godot 行为探针验证 pivot 语义 |
| 相对 `mouse_event(MOVE)` 执行期间 Codex/VS Code 重新获得前台，输入落到编辑器命令面板 | 1 | 未修改任何文件；停止桌面级拖拽自动化，改用隔离 Godot 行为探针直接执行相同 GDScript 交互方法 |
| 行为探针首轮在 `SceneTree._initialize()` 中过早访问 Camera3D，断言通过但夹具报告节点未进入树 | 1 | 将探针主体 `call_deferred` 到场景树就绪后，并在退出前显式释放 controller/holder，随后无错误复跑 |
| 最终截图哈希汇总把 PowerShell `foreach` 代码块直接接到管道，触发 Empty pipe element | 1 | 命令未执行任何操作；先收集 `$rows` 再传给 `Format-Table`，不重复错误语法 |
| 最终四项只读汇总的并行聚合在 10 秒时限内未返回分项结果 | 1 | 拆成精确单项查询，并只为 Git 状态检查提高时限；不重复整组调用 |
| 尝试对仍由运行中 Godot 持有的 stdout/stderr 日志执行 `Get-FileHash` 被文件锁拒绝 | 1 | 截图哈希已取得；运行日志已通过内容扫描验证，不停止交互客户端仅为获取非必要日志哈希 |

### Phase 19: 腔镜规划路径与真实导丝可视化优化方案（只读分析）
- [x] 恢复既有规划、核对交接文档与当前工作树
- [x] 追踪规划路径从后端协议到 Godot 渲染器的数据链路
- [x] 追踪腔镜私有 SubViewport 的可见层、相机和材质链路
- [x] 识别当前腔镜中“箭头”与真实导丝几何的来源和限制
- [x] 给出分阶段实现、视觉规范、性能与验收方案
- [x] 复核冻结范围并交付方案，不修改产品代码
- **Status:** complete

### Errors for Phase 19
| Error | Attempt | Resolution |
|-------|---------|------------|
| 首次并行执行 session-catchup/status/文件枚举时，其中一项非零导致聚合调用未返回完整分项 | 1 | 分拆为精确只读命令；session-catchup 后续确认 exit=0，status 和交接文档均已读取 |
