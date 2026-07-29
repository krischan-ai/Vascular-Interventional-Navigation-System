# CathSim Reinforcement Learning Workspace

Independent training workspace for `/home/ps/cathsim-warp`, based on the reinforcement-learning design document.

- Python: `.venv310` (Python 3.10.20)
- Default physics: Newton/Warp on CUDA (`CATHSIM_PHYSICS_ENGINE=newton`)
- Action: high-level ShapeIntent
- Baseline: PPO with SB3; outputs under `runs/`
- Safety: existing backend Safety Shield remains active

## Documentation

- [Training design (Chinese)](../doc/5.训练层/11-强化学习训练设计方案.md)
- [Beginner tutorial: environment checks to the first PPO experiment (Chinese)](../doc/5.训练层/12-强化学习训练教程-从环境检查到首次PPO实验.md)
- [HCI data collection and high-quality RL dataset tutorial (Chinese)](../doc/5.训练层/13-人机交互与强化学习一体化平台高质量数据集制作教程.md)

The YAML files under `configs/` are currently design snapshots. The active
training entrypoint does not load them automatically; verify effective values in
each run's `run_config.json`.

## Setup

```bash
cd /home/ps/cathsim-warp/reinforcement_learning
bash scripts/setup.sh
bash scripts/smoke_test.sh
```

If downloads are slow, use the local proxy only for that command:

```bash
HTTP_PROXY=http://127.0.0.1:7890 \
HTTPS_PROXY=http://127.0.0.1:7890 \
CATHSIM_RL_PIP_INDEX_URL=https://pypi.org/simple \
  bash scripts/setup.sh
```

## Train PPO on Newton/Warp

```bash
bash scripts/train_ppo.sh stage0_endpoint_0 100000 0
```

Override `CATHSIM_PHYSICS_ENGINE=mujoco` only for the MuJoCo comparison baseline.
