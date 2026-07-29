# Task Plan: 服务器 CI/CD 工作流设计

## Goal
只读审计 192.168.1.107 上 `/home/ps/cathsim-warp` 后端环境，撰写可实施、安全且可回滚的 CI/CD 设计文档。

## Phases
- [x] 建立服务器连接并采集运行环境
- [x] 审计后端目录、启动进程、依赖与部署脚本
- [x] 设计 CI、制品、部署、健康检查与回滚流程
- [x] 撰写并校验设计文档

## Constraints
- 不修改服务器文件、进程或服务。
- 不在仓库或文档中记录明文密码。
- 保留现有工作区改动。

## Errors
- 本地辅助脚本首次缺少 PYTHONPATH；修正后完成检查。`r`n- root 读取 ps 所有的 Git 仓库触发 ownership 保护；改用 ps 身份只读检查。
