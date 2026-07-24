#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ps/cathsim-warp
RL_ROOT="$ROOT/reinforcement_learning"
RUN_NAME="${1:-stage0_endpoint_0}"
TIMESTEPS="${2:-100000}"
SEED="${3:-0}"
cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/src"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export CATHSIM_PHYSICS_ENGINE="${CATHSIM_PHYSICS_ENGINE:-newton}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$RL_ROOT/.venv310/bin/python" "$RL_ROOT/scripts/train_navigation.py" --run-name "$RUN_NAME" --output-dir "$RL_ROOT/runs" --phantom aorta_tree --route-target endpoint_0 --action-mode shape_intent --total-timesteps "$TIMESTEPS" --seed "$SEED" --eval-episodes 5
