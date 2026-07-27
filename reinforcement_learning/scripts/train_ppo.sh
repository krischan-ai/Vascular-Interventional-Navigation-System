#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RL_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$RL_ROOT/.." && pwd)"
RUN_NAME="${1:-stage0_endpoint_0}"
TIMESTEPS="${2:-100000}"
SEED="${3:-0}"
PYTHON="${CATHSIM_RL_PYTHON:-$RL_ROOT/.venv310/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  echo "RL Python not found: $PYTHON" >&2
  echo "Run scripts/setup.sh or set CATHSIM_RL_PYTHON." >&2
  exit 1
fi
cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/src"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export CATHSIM_PHYSICS_ENGINE="${CATHSIM_PHYSICS_ENGINE:-newton}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$PYTHON" "$RL_ROOT/scripts/train_navigation.py" --run-name "$RUN_NAME" --output-dir "$RL_ROOT/runs" --phantom aorta_tree --route-target endpoint_0 --action-mode shape_intent --total-timesteps "$TIMESTEPS" --seed "$SEED" --eval-episodes 5
