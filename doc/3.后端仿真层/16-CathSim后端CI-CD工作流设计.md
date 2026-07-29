# CathSim 后端 CI/CD 工作流设计

> 文档状态：设计稿  
> 目标服务器：`192.168.1.107`  
> 后端工作目录：`/home/ps/cathsim-warp`  
> 生产接口：`http://192.168.1.107:9000`、`ws://192.168.1.107:9000/ws/session`

## 1. 目标

建立从代码提交到远端生产发布的可审计流水线，满足：

1. 后端相关代码提交后自动执行静态检查、单元测试和协议测试。
2. 只部署通过 CI 的同一份不可变 Docker 镜像。
3. 部署前不覆盖生产目录，先在候选端口完成 GPU、HTTP 和 WebSocket 验证。
4. 发布失败自动恢复上一镜像，避免手工拷贝造成半更新状态。
5. 部署身份、运行身份和管理员身份分离，不在仓库保存密码。
6. 每次发布可追溯到 Git commit SHA，并保留部署记录和日志。

## 2. 服务器现状审计

### 2.1 基础环境

| 项目 | 实测结果 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS，Linux 5.15 |
| CPU | 76 个逻辑处理器 |
| 内存 | 30 GiB，可用约 17 GiB |
| 磁盘 | 938 GiB，已使用约 67% |
| GPU | NVIDIA RTX A6000，48 GiB |
| NVIDIA 驱动 | 550.90.07 |
| Conda Python | `/home/ps/anaconda3/envs/cathsim-newton/bin/python`，Python 3.10.20 |
| Docker Compose | 2.35.1 |
| 后端目录 | `/home/ps/cathsim-warp` |

### 2.2 当前运行状态

当前存在两条后端运行链路：

1. 生产端口 `9000`
   - root 用户直接运行 Conda 环境中的 Uvicorn；
   - 工作目录为 `/home/ps/cathsim-warp`；
   - PID 1 的子进程，不受 systemd 管理；
   - `/api/v1/health` 返回正常。
2. 候选端口 `19000`
   - Docker 容器名：`cathsim-server`；
   - 镜像：`cathsim-simulation:local`；
   - 容器内端口 9000 映射到宿主机 19000；
   - 使用 NVIDIA/CUDA、MuJoCo EGL 和 Newton/Warp；
   - `restart=unless-stopped`，Docker health 状态为 healthy；
   - `/api/v1/health` 返回正常。

这说明服务器已经具备容器化生产基础。CI/CD 应统一到 Docker 路线，不应继续让
`tools/deploy_backend.py` 解压覆盖运行目录并扫描/终止 Uvicorn 进程。

### 2.3 代码与配置风险

- 服务器 Git 分支为 `feature/backend-rl-training-framework`，存在未提交和未跟踪文件。
- 当前服务器提交与客户端开发分支不一致，不能把服务器工作目录当成唯一可信构建源。
- `9000` 与 `19000` 运行的是两套独立实例，版本可能不同。
- 当前镜像标签 `local` 不可追溯到 commit SHA。
- 生产 9000 进程仍携带历史部署环境变量，重启和回滚依赖人工操作。
- 未发现 CathSim systemd 服务；裸进程异常退出后缺少标准服务管理。
- SSH 当前允许密码认证；自动化若继续使用个人密码和 `su root`，凭据泄露面过大。

## 3. 推荐总体架构

```text
开发者 push / Pull Request
          │
          ▼
GitHub Actions CI（公共 Runner）
  ├─ Ruff / compile
  ├─ pytest 单元与协议测试
  ├─ 构建 simulation Docker 镜像
  ├─ 镜像内 smoke test
  └─ 推送 GHCR：sha-<commit>
          │
          ▼
GitHub Environment: production
  ├─ 分支保护
  ├─ 可选人工审批
  └─ 并发锁
          │
          ▼
服务器内网 Self-hosted Runner（ps 用户）
  ├─ 拉取指定 SHA 镜像
  ├─ 启动候选实例 :19000
  ├─ HTTP / WebSocket / GPU smoke
  ├─ 切换生产实例 :9000
  ├─ 发布后验证
  └─ 失败切回 previous 镜像
```

### 3.1 为什么使用 Self-hosted Runner

`192.168.1.107` 是内网地址，GitHub 公共 Runner 通常无法访问。将 GitHub Actions
Self-hosted Runner 安装在服务器上，可以：

- 不暴露 SSH 端口到公网；
- 不在流水线中传递 SSH 密码和 root 密码；
- 直接使用本机 Docker Engine；
- 保留 GitHub 的审批、日志、并发控制和审计能力。

Runner 只负责受控部署，不用于执行未经审批的外部 PR。外部 PR 只能在 GitHub 公共
Runner 执行无密钥 CI。

## 4. 分支与触发规则

建议：

- Pull Request：只运行 CI，不部署。
- 合并到 `guidewire-device-procedure-design`：自动部署测试/候选环境。
- 创建 `backend-v*` tag：部署生产环境。
- `workflow_dispatch`：允许授权人员手工重部署或回滚指定 SHA。

后端触发路径：

```yaml
paths:
  - "services/**"
  - "src/cathsim/**"
  - "data/vpp_assets/**"
  - "server/**"
  - "tests/**"
  - "pyproject.toml"
```

前端、文档或普通模型导入文件变动不应触发后端生产发布。

## 5. CI 阶段设计

### 5.1 快速检查

每个 PR 必须执行：

```bash
python -m compileall services src/cathsim
ruff check services src/cathsim tests
pytest tests/test_websocket.py \
       tests/test_navigation_engine.py \
       tests/test_frontend_contract.py -q
```

### 5.2 完整回归

合并和发布标签执行：

```bash
pytest -q
```

MuJoCo/Newton/GPU 测试应使用 pytest marker 分层：

- `unit`：无 GPU，公共 Runner 执行；
- `simulation`：需要 MuJoCo/Newton；
- `gpu`：需要 NVIDIA GPU，在服务器候选容器执行；
- `slow`：夜间或发布标签执行。

### 5.3 镜像构建

以服务器已有的 `server/Dockerfile.simulation` 为基础构建：

```bash
docker build \
  -f server/Dockerfile.simulation \
  --label org.opencontainers.image.revision="$GITHUB_SHA" \
  --label org.opencontainers.image.source="$GITHUB_SERVER_URL/$GITHUB_REPOSITORY" \
  -t ghcr.io/<org>/cathsim-simulation:sha-$GITHUB_SHA \
  .
```

镜像同时可以标记：

- `sha-<40位提交>`：不可变部署标识；
- `branch-<分支>`：便于候选环境引用；
- `backend-vX.Y.Z`：生产发布版本；
- `latest`：只作为展示标签，不作为部署依据。

禁止只推送 `latest` 或 `local` 后直接部署。

## 6. CD 阶段设计

### 6.1 候选部署

1. 登录 GHCR。
2. 拉取精确的 `sha-$GITHUB_SHA` 镜像。
3. 用候选 compose 启动 `cathsim-candidate`：
   - 宿主端口：19000；
   - GPU：`device_ids: ["0"]`；
   - 独立容器名和网络；
   - 环境变量来自 `/etc/cathsim/backend.env`；
   - 不挂载服务器源码。
4. 等待 Docker health 变为 healthy。
5. 执行 HTTP、WebSocket 和 GPU smoke test。

### 6.2 发布前验证

至少验证：

```bash
curl --fail http://127.0.0.1:19000/api/v1/health
docker exec cathsim-candidate python server/verify_simulation.py
```

还应建立 WebSocket smoke：

1. 连接 `/ws/session`；
2. 发送 `session_start`；
3. 断言收到 `session_started`；
4. 断言 `engine` 为预期 Newton/MuJoCo 实现；
5. 发送最小 control；
6. 断言收到合法 `state_batch`；
7. 主动关闭会话。

### 6.3 切换生产

推荐初期采用短暂停机切换：

1. 记录当前生产镜像 digest 到 `previous.env`。
2. 停止旧的裸 Uvicorn 9000 进程。
3. 使用候选验证通过的同一镜像启动生产容器：
   - 容器名：`cathsim-production`；
   - 端口：`9000:9000`；
   - `restart: unless-stopped`。
4. 等待 health。
5. 从宿主机和局域网分别验证 HTTP 与 WebSocket。
6. 成功后停止候选容器，保留上一镜像。

后续可引入 Nginx/OpenResty，将生产入口固定在 9000，蓝绿容器使用 19001/19002，
通过 upstream 切换实现更短中断。

## 7. 自动回滚

下列任一条件满足即回滚：

- 容器在 90 秒内未变为 healthy；
- `/api/v1/health` 非 200 或 `status != ok`；
- `vpp_ready != true`；
- WebSocket 无法创建会话或未返回首个状态包；
- 容器异常退出；
- 镜像 revision 与目标 SHA 不一致。

回滚步骤：

```text
停止失败的新生产容器
→ 用 previous.env 中的镜像 digest 恢复生产容器
→ 等待 health
→ 再次执行 HTTP/WebSocket smoke
→ GitHub Job 标记失败并保留诊断日志
```

至少保留最近 5 个已验证镜像。镜像清理不得删除当前和 previous digest。

## 8. GitHub Actions 工作流建议

文件：`.github/workflows/backend-ci-cd.yml`

```yaml
name: Backend CI/CD

on:
  pull_request:
    paths:
      - "services/**"
      - "src/cathsim/**"
      - "data/vpp_assets/**"
      - "server/**"
      - "tests/**"
      - "pyproject.toml"
  push:
    branches: [guidewire-device-procedure-design]
    tags: ["backend-v*"]
  workflow_dispatch:
    inputs:
      image_ref:
        description: "Optional sha-* image to redeploy"
        required: false

permissions:
  contents: read
  packages: write

concurrency:
  group: cathsim-production
  cancel-in-progress: false

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.10"
          cache: pip
      - run: python -m pip install -e ".[dev]"
      - run: python -m compileall services src/cathsim
      - run: ruff check services src/cathsim tests
      - run: pytest -q

  build:
    needs: test
    if: github.event_name != 'pull_request'
    runs-on: ubuntu-latest
    outputs:
      image: ${{ steps.meta.outputs.image }}
    steps:
      - uses: actions/checkout@v4
      - uses: docker/setup-buildx-action@v3
      - uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - id: meta
        run: echo "image=ghcr.io/${GITHUB_REPOSITORY,,}/simulation:sha-$GITHUB_SHA" >> "$GITHUB_OUTPUT"
      - uses: docker/build-push-action@v6
        with:
          context: .
          file: server/Dockerfile.simulation
          push: true
          tags: ${{ steps.meta.outputs.image }}
          labels: |
            org.opencontainers.image.revision=${{ github.sha }}

  deploy:
    needs: build
    runs-on: [self-hosted, linux, x64, cathsim-production]
    environment: production
    timeout-minutes: 15
    steps:
      - uses: actions/checkout@v4
      - name: Deploy immutable image
        env:
          CATHSIM_IMAGE: ${{ needs.build.outputs.image }}
          EXPECTED_SHA: ${{ github.sha }}
        run: sudo --non-interactive /usr/local/sbin/cathsim-deploy
```

工作流中不直接拼接 `sudo`、Docker 和回滚命令。复杂且高权限的部署逻辑固化为服务器
root 所有、普通用户不可写的 `/usr/local/sbin/cathsim-deploy`，并经过代码审查。

## 9. 服务器端程序和目录

建议目录：

```text
/etc/cathsim/
├── backend.env              # root:root 0600
├── production.env           # 当前镜像引用
└── previous.env             # 上一镜像引用

/opt/cathsim/
├── compose/
│   ├── production.yaml
│   └── candidate.yaml
├── logs/
└── releases/

/usr/local/sbin/
└── cathsim-deploy           # root:root 0755
```

`backend.env` 保存物理参数，不进入 Git：

```dotenv
CATHSIM_PHYSICS_ENGINE=newton
CATHSIM_PORT=9000
CATHSIM_VPP_DATA_ROOT=/app/data/vpp_assets
CATHSIM_NEWTON_SUBSTEPS=8
CATHSIM_NEWTON_ITERS=4
CATHSIM_NEWTON_PUSH_SPEED=0.05
```

## 10. 身份与密钥设计

### 10.1 禁止项

- 不在 workflow、仓库、文档或 `.env` 示例中保存明文 SSH/root 密码。
- 不让 CI 执行任意 `su root` 命令。
- 不使用 `AutoAddPolicy` 自动接受未知 SSH 主机指纹。
- 不允许来自 fork 的 PR 使用 Self-hosted Runner 或生产 secrets。
- 不在服务器有未提交改动的工作树内构建生产镜像。

### 10.2 推荐权限

创建独立用户 `cathsim-runner`，安装 Self-hosted Runner。其 sudoers 仅允许：

```sudoers
cathsim-runner ALL=(root) NOPASSWD: /usr/local/sbin/cathsim-deploy
```

`cathsim-deploy` 必须校验镜像格式，只接受：

```text
ghcr.io/<固定组织>/<固定仓库>/simulation@sha256:<digest>
```

不能接受任意 shell 参数、任意 compose 文件或任意镜像仓库。

当前提供过的登录密码已经出现在协作消息中，应视为已暴露：完成 Runner 和 SSH key
配置后应轮换 `ps` 与 root 密码，并逐步关闭 SSH 密码认证。

## 11. GitHub 配置

### 11.1 Environment

创建 `production` Environment：

- 限制允许部署的分支或 tag；
- 配置 Required reviewers；
- 设置部署 URL 为健康检查地址；
- 可选设置等待时间；
- 禁止管理员绕过审批。

### 11.2 Secrets

采用服务器自托管 Runner 后，通常不需要 SSH 密码。可能需要：

| Secret | 用途 |
|---|---|
| `GHCR_READ_TOKEN` | 私有 GHCR 镜像拉取；若 Runner 使用 GitHub job token 可省略 |
| `DEPLOY_NOTIFY_WEBHOOK` | 可选部署通知 |

运行参数优先存放服务器 `/etc/cathsim/backend.env`，不要把生产参数全部暴露给 GitHub
工作流。

## 12. 可观测性与审计

每次发布记录：

- Git commit SHA；
- 镜像 tag 和 digest；
- GitHub run ID、触发人和审批人；
- 部署开始/结束时间；
- previous/new 镜像；
- HTTP、WebSocket、GPU smoke 结果；
- 回滚原因。

建议 Docker 日志设置轮转：

```yaml
logging:
  driver: json-file
  options:
    max-size: "50m"
    max-file: "5"
```

候选或生产失败时，workflow 上传：

```bash
docker inspect <container>
docker logs --tail 500 <container>
nvidia-smi
curl -v http://127.0.0.1:<port>/api/v1/health
```

日志必须过滤 token、密码、Authorization header 和完整环境变量。

## 13. 实施阶段

### 阶段 A：基线固化

1. 确认 9000 与 19000 的业务差异和目标镜像版本。
2. 给镜像写入 revision label。
3. 建立 HTTP/WebSocket/GPU smoke。
4. 将现有 Docker compose 文件整理并提交到主开发分支。

### 阶段 B：CI

1. 增加 Ruff 和 pytest 分层。
2. 创建 `backend-ci-cd.yml`，先只运行测试和构建。
3. 推送 SHA 镜像到 GHCR。
4. 配置分支保护，要求 CI 通过才能合并。

### 阶段 C：候选 CD

1. 安装 `cathsim-runner`。
2. 只允许部署到 19000。
3. 连续完成至少 5 次自动候选部署。
4. 验证并发锁、失败日志和镜像清理。

### 阶段 D：生产 CD

1. 安装受限 sudo 部署脚本。
2. 将 9000 从 root/Conda 裸进程迁移为生产容器。
3. 启用 GitHub Environment 人工审批。
4. 演练健康检查失败自动回滚。
5. 轮换密码并关闭不必要的密码认证。

### 阶段 E：蓝绿和监控

1. 引入 OpenResty/Nginx upstream 切换。
2. 接入 Prometheus/Grafana 或现有监控。
3. 增加部署成功率、延迟、会话错误率和 GPU 指标。

## 14. 验收标准

- PR 未通过测试时无法合并和部署。
- 部署镜像可以从运行容器反查到唯一 commit SHA。
- 候选验证不影响 9000 的现有会话。
- 同一时间最多一个生产部署。
- 健康检查失败能在 2 分钟内恢复上一版本。
- 服务器源码目录即使存在未提交文件，也不会进入生产镜像。
- GitHub 日志和仓库中不存在 SSH/root 明文密码。
- 服务器重启后生产容器能自动恢复。
- HTTP、WebSocket 和 GPU 仿真链路均有自动验收。

## 15. 结论

当前服务器已经验证了 simulation Docker 镜像、NVIDIA GPU、容器健康检查和 19000
候选端口，最稳妥的路线不是继续强化“SSH + 密码 + su root + 覆盖目录”，而是：

> 公共 Runner 完成 CI 和不可变镜像构建，内网 Self-hosted Runner 仅部署固定 digest，
> 先验证 19000 候选实例，再切换 9000 生产实例，失败自动恢复 previous digest。

该方案最大限度复用服务器现有 Docker 成果，同时解决版本不可追溯、双运行链路、
明文密码、高权限部署和缺少自动回滚的问题。
