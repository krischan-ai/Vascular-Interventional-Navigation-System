# 前后端连接排查计划

## Goal
定位 Godot 前端无法连接 Python 后端的根因；在用户授权范围内修复，并用健康检查与 WebSocket 会话验证。

## Phases
- [ ] 1. 核对启动脚本、地址、端口和协议配置（in progress）
- [ ] 2. 检查实际进程、端口监听与后端启动错误
- [ ] 3. 复现前端连接并定位失败层
- [ ] 4. 实施最小修复并回归验证

## Constraints
- 保留工作区既有修改，不执行 reset/checkout。
- 不覆盖根目录已有任务规划。

## Errors
- 全局 session-catchup.py 不存在；改为人工核对现有规划与 git 状态。
