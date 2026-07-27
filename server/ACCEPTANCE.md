# 服务端验收记录

验收日期：2026-07-27

## 结论

项目已具备可独立交付的服务端主链路：

```text
FastAPI/WebSocket
  -> NavigationEngine / PathPlanner / RiskAssessor / ShapeIntent
  -> PhysicsEngine (Kinematic / MuJoCo / Newton)
  -> Gymnasium NavigationGym
  -> SB3 PPO/SAC train/evaluate
```

服务端核心测试集通过 119 项，覆盖物理引擎契约、ShapeIntent、导航控制、
风险评估、Newton 防屈曲参数、VPP 路径规划、会话录制、Gym 环境和 PPO/SAC
训练/评估入口。Python 3.10 环境可导入 FastAPI、Uvicorn、Gymnasium、
Stable-Baselines3、Newton 1.3.0 和 Warp 1.14.0。

独立交付包采用 `server/manifest.json` 白名单生成，不包含 Godot 客户端、
Jekyll 文档站、本地虚拟环境、训练日志/模型、历史备份或异常命名的临时文件。

## 未通过或未覆盖项

- 全仓 `pytest --collect-only` 不是全绿：初次检查有 12 个收集错误；增加
  `testpaths = ["tests"]`、排除历史备份后剩余 11 个正式测试收集错误。
- 一批上游 CathSim 旧测试仍引用已删除的 `cathsim.guidewire`、
  `cathsim.phantom`、`cathsim.env`、`cathsim.utils` 和 `make_gym_env` 接口。
- `tests/test_dm_env.py` 在无显示环境下触发 MuJoCo `gladLoadGL error`，需要
  EGL/OSMesa 或带显示的专用环境。
- 当前 FastAPI/Starlette 的 `TestClient` 需要额外的 `httpx2` 测试依赖，
  因此 REST/WebSocket TestClient 用例未计入本轮 119 项核心通过结果。
- 本轮没有重新执行长时 Newton GPU 大曲率实机审计；已有文档记录的 A6000
  结果不等同于本次重新验证。

因此验收等级为：**服务端核心链路通过，可拆分交付；全仓历史兼容与图形/
TestClient 测试仍需专项治理。**

## Docker 配置更新

2026-07-27 更新为 CUDA 12.1 + Python 3.10 服务镜像，覆盖 FastAPI、
Newton/Warp、MuJoCo headless 和 PPO/SAC 训练依赖；容器使用非 root 用户，
提供 HTTP 健康检查、GPU Compose 配置及独立 `training` profile。

以下静态检查通过：

```bash
docker compose --env-file server/.env.example \
  -f server/compose.yaml config --quiet
docker compose --env-file server/.env.example \
  -f server/compose.yaml --profile training config --quiet
python -m pip check
```

真实镜像构建与容器运行验收也已通过：

- 镜像：`cathsim-server:local`
- 镜像 ID：`sha256:ce7d75669f81caf6016633026b4c9b28723ee4bf53671c57ad31d2abbdbe2e9d`
- 未压缩大小：`9,703,904,680` 字节
- 运行用户：`cathsim`（非 root）
- 容器内 `pip check`：`No broken requirements found.`
- 关键版本：FastAPI 0.139.2、Newton 1.3.0、Warp 1.14.0、
  Torch 2.4.1+cu121
- GPU 容器：识别 NVIDIA RTX A6000 48 GiB；Torch CUDA 可用；
  Warp 成功初始化 `cuda:0`
- API 容器：Docker `healthy`；`/api/v1/health` 返回
  `status=ok`、`vpp_ready=true`、`cases=["case_001"]`

构建时仅为 Docker Hub 配置临时代理；Ubuntu APT、PyPI 和 NVIDIA 国内源
通过 `NO_PROXY` 直连。临时验收容器已在检查完成后停止并自动移除。

## 运行部署

宿主机已有 Python 后端监听 `0.0.0.0:9000`。完整镜像运行配置仍保留在
`server/compose.runtime.yaml`，当前正式服务已切换为新增的
`server/compose.simulation.deploy.yaml`：

- 端口：宿主机 `19000` -> 容器 `9000`
- 容器：`cathsim-server`
- 镜像：`cathsim-simulation:local`
- 状态：`Up (healthy)`
- 健康接口：`http://127.0.0.1:19000/api/v1/health`
- 重启策略：`unless-stopped`
- GPU：Warp 成功初始化 RTX A6000 `cuda:0`
- 镜像内确认不存在 Torch、Gymnasium、Stable-Baselines3 和 TensorBoard

如需回退完整服务端，先关闭 `compose.simulation.deploy.yaml`，再启动原
`compose.runtime.yaml`；两个 Compose 均只使用本地镜像，不会自动拉取。

## 纯后端仿真镜像

为不需要 PPO/SAC 训练的部署新增 `server/Dockerfile.simulation` 与
`server/compose.simulation.yaml`。MuJoCo 通用工具已与 Gym/SB3 工具解耦，
原有完整训练环境仍保留，纯仿真镜像则只复制后端、仿真核心和数据。

- 镜像：`cathsim-simulation:local`
- 镜像 ID：`sha256:a21bdc2ff262a0314be5b1127cb8d53d2bf304c528a9683891d82f89c656677d`
- 未压缩大小：`3,500,603,865` 字节，较完整镜像减少约 64%
- 运行用户：`cathsim`（非 root）
- 已排除框架：Torch、Gymnasium、Stable-Baselines3、TensorBoard
- 已排除源码：`src/cathsim/rl`、`src/cathsim/gym`、
  `reinforcement_learning`
- 容器内 `pip check`：无依赖冲突
- GPU：Warp 1.14.0 成功初始化 RTX A6000 `cuda:0`
- MuJoCo：真实执行 `reset()` 和一个 `step()`，导管尖端位置发生有效变化
- API：临时容器在宿主机 `19001` 返回 `status=ok` 且 Docker 状态为
  `healthy`；验收后已自动移除
- 回归测试：物理引擎、自动控制、VPP 路径与 Newton 防屈曲共 76 项通过

运行版 Compose 默认预留宿主机 `19001`，不会与现有 `9000` 后端及
`19000` 正式服务冲突；正式 `19000` 服务当前使用新增的
`compose.simulation.deploy.yaml`。
