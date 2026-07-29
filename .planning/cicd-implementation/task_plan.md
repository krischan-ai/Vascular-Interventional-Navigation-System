# Task Plan: 上游仓库 CI/CD 实施

## Goal
在 krischan-ai/cathsim-centerline 上游仓库搭建可信 Collaborator 提交后自动测试、构建、
候选验证、生产部署、备份和失败回滚的 CI/CD。

## Phases
- [x] 核对远端分支、Docker 文件和 GitHub/Runner 状态
- [x] 在独立 worktree 实现 CI/CD 文件
- [x] 本地静态检查与脚本测试
- [x] 创建提交并推送上游实施分支
- [ ] 配置 GitHub 与服务器运行条件
- [x] 验证远端工作流可见性并交付

## Constraints
- 不污染当前脏工作区。
- 不提交或输出明文密码。
- 未验证前不替换当前生产 9000 服务。

## Errors
- GitHub CI 三轮发现环境依赖缺口，第四轮通过。
- 服务器 Runner、sudoers 与 root 部署入口安装被安全审批拒绝，等待用户明确批准。
