# CathSim 服务端

本目录定义项目的独立服务端交付边界。源代码仍保持单一副本，避免移动
`services`、`src/cathsim` 后破坏现有绝对导入；交付时由清单生成一个不含
Godot 前端、站点文档、虚拟环境、训练产物和历史备份的服务端包。

## 服务端组成

| 目录 | 职责 |
|---|---|
| `services/` | FastAPI、WebSocket、会话、路径规划、风险、ShapeIntent |
| `services/physics/` | PhysicsEngine 抽象及 Kinematic/MuJoCo/Newton 实现 |
| `src/cathsim/dm/` | CathSim/MuJoCo 仿真核心与运行资产 |
| `src/cathsim/gym/` | Gymnasium 环境，包括 Newton NavigationGym |
| `src/cathsim/rl/` | PPO/SAC 训练、评估、配置与数据工具 |
| `reinforcement_learning/` | 独立训练配置、依赖和操作脚本 |
| `data/` | VPP 图、半径、路线和仿真验收数据 |
| `tools/` | 服务端资产、物理和部署工具；Godot 资产导出工具不进入交付包 |

客户端 `godot_client/`、Jekyll 站点 `docs/`、完整设计档 `doc/` 和本地运行
产物不属于服务端交付物。

## 构建独立交付包

在仓库根目录执行：

```bash
python tools/build_server_bundle.py \
  --output dist/cathsim-server \
  --zip
```

构建器严格读取 [`manifest.json`](manifest.json)，遇到缺失的必需路径或已
存在的输出目录会直接失败。生成结果包括：

- `dist/cathsim-server/`：可直接复制到 Linux/GPU 主机的服务端目录；
- `dist/cathsim-server.zip`：同内容压缩包；
- `SERVER_BUILD.json`：文件数、总大小及逐文件 SHA-256；
- `server/verify_server.py`：独立包结构、导入和健康检查。

## 本地验收

推荐使用强化学习环境的 Python 3.10：

```bash
reinforcement_learning/.venv310/bin/python server/verify_server.py --root .

MPLCONFIGDIR=/tmp/cathsim-mpl \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
reinforcement_learning/.venv310/bin/python -m pytest -q \
  tests/test_physics_engine.py \
  tests/test_shape_intent.py \
  tests/test_navigation_intent.py \
  tests/test_risk_assessor.py \
  tests/test_physics_autopilot.py \
  tests/test_navigation_gym_env.py \
  tests/test_navigation_train.py \
  tests/test_navigation_evaluate.py \
  tests/test_session_recording.py \
  tests/test_vpp_path_planner.py \
  tests/test_route_quality_report.py \
  tests/test_large_curvature_audit.py \
  tests/test_newton_antibuckle.py
```

独立包默认不携带虚拟环境。可先运行
`bash reinforcement_learning/scripts/setup.sh`，或复用服务器已有 Python：

```bash
CATHSIM_RL_PYTHON=/path/to/python \
  bash reinforcement_learning/scripts/smoke_test.sh
```

## 启动与训练

```bash
export CATHSIM_PHYSICS_ENGINE=newton
reinforcement_learning/.venv310/bin/python -m uvicorn \
  services.main:app --host 0.0.0.0 --port 9000

bash reinforcement_learning/scripts/train_ppo.sh \
  stage0_endpoint_0 100000 0
```

容器构建入口：

```bash
cp server/.env.example server/.env
docker build -f server/Dockerfile -t cathsim-server .
docker compose -f server/compose.yaml up
```

已有本地镜像时，使用运行版 Compose 启动，不会重新构建或拉取镜像：

```bash
docker compose -f server/compose.runtime.yaml up -d
```

运行版默认将宿主机 `19000` 映射到容器 `9000`，避免与宿主机已有的
`9000` 后端冲突。需要调整时可设置 `CATHSIM_HOST_PORT`：

```bash
CATHSIM_HOST_PORT=19002 \
  docker compose -f server/compose.runtime.yaml up -d
```

## 生产环境部署

生产环境使用 [`compose.production.yaml`](compose.production.yaml)，默认从华为云
SWR 拉取固定版本 `cathsim-server:v1.0`，不会在生产服务器上执行镜像构建。
部署机需要提前安装兼容的 NVIDIA 驱动、NVIDIA Container Toolkit、Docker
Engine 和 Docker Compose v2。

首次部署先登录镜像仓库。密码或访问令牌只通过交互式标准输入提供，不要写入
Compose 或 Git：

```bash
docker login swr.cn-east-3.myhuaweicloud.com
```

生产 Compose 已固定使用 VPP 资产目录 `/app/data/vpp_assets`、Newton
物理引擎以及宿主机端口 `19000`，不依赖 `.env` 文件。登录仓库后只需一条
命令，Compose 会自动拉取镜像并启动容器：

```bash
docker compose \
  -f server/compose.production.yaml \
  up -d
```

如需在启动前检查展开后的配置，可单独执行
`docker compose -f server/compose.production.yaml config`。

默认对外地址为 `http://<服务器IP>:19000`，健康检查和接口文档地址为：

```text
http://<服务器IP>:19000/api/v1/health
http://<服务器IP>:19000/docs
ws://<服务器IP>:19000/ws/session
```

生产 Compose 使用只读根文件系统、非 root 镜像用户、Linux capability
清理、健康检查、日志轮转和命名卷。缓存、训练日志、模型及运行记录不会随
容器重建丢失。查看状态和日志：

```bash
docker compose \
  -f server/compose.production.yaml \
  ps

docker compose \
  -f server/compose.production.yaml \
  logs --tail 100 cathsim-server
```

需要升级或回滚时，直接修改 `compose.production.yaml` 中的固定镜像版本，
再执行 `pull` 和 `up -d`。`latest` 适合联调，不建议作为生产环境的长期
固定版本。

## 纯后端仿真镜像

不需要强化学习训练时，可构建仅包含 FastAPI、MuJoCo、Newton/Warp 的镜像：

```bash
docker build -f server/Dockerfile.simulation \
  -t cathsim-simulation:local .
```

该镜像不安装 Torch、Gymnasium、Stable-Baselines3 或 TensorBoard，也不包含
`src/cathsim/rl`、`src/cathsim/gym` 和 `reinforcement_learning`。运行版
Compose 默认使用宿主机 `19001`，与 `9000` 旧后端和 `19000` 完整服务端
错开：

```bash
docker compose -f server/compose.simulation.yaml up -d
```

替换正式 `19000` 服务、同时保留原完整镜像 Compose 文件时，使用独立部署
Compose：

```bash
docker compose -f server/compose.runtime.yaml down
docker compose -f server/compose.simulation.deploy.yaml up -d
```

`compose.simulation.deploy.yaml` 沿用容器名 `cathsim-server` 和宿主机端口
`19000`，但只使用 `cathsim-simulation:local` 镜像。

默认镜像基于 CUDA 12.1、Python 3.10，并同时包含 API、Newton/Warp 仿真和
PPO/SAC 训练依赖。Newton/Warp 真物理运行需要 NVIDIA GPU、兼容驱动和
NVIDIA Container Toolkit。

构建默认使用华为云 Ubuntu/PyPI 镜像；需要切换时设置
`CATHSIM_APT_MIRROR`、`CATHSIM_PIP_INDEX_URL`。

网络受限时可仅对单次构建配置临时代理，同时让国内镜像源保持直连：

```bash
docker build --network=host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  --build-arg NO_PROXY=localhost,127.0.0.1,repo.huaweicloud.com,mirrors.huaweicloud.com,developer.download.nvidia.com,developer.download.nvidia.cn \
  -f server/Dockerfile -t cathsim-server:local .
```

这些代理参数只在本次构建中生效，不会写入运行容器配置。

独立启动训练任务：

```bash
docker compose --env-file server/.env \
  -f server/compose.yaml --profile training run --rm cathsim-train
```

训练日志、模型和运行记录分别持久化到
`reinforcement_learning/logs`、`models`、`runs`。容器默认使用 UID/GID
`1000:1000`；共享主机目录权限不一致时，在 `server/.env` 中调整。
