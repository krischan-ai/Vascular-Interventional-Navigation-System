# CathSim Reinforcement Learning Workspace

Independent training workspace for `/home/ps/cathsim-warp`, based on the reinforcement-learning design document.

- Python: `.venv310` (Python 3.10.20)
- Default physics: Newton/Warp on CUDA (`CATHSIM_PHYSICS_ENGINE=newton`)
- Action: high-level ShapeIntent
- Baseline: PPO with SB3; outputs under `runs/`
- Safety: existing backend Safety Shield remains active

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
