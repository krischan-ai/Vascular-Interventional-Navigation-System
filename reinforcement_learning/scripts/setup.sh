#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ps/cathsim-warp
RL_ROOT="$ROOT/reinforcement_learning"
INDEX_URL="${CATHSIM_RL_PIP_INDEX_URL:-https://mirrors.huaweicloud.com/repository/pypi/simple}"
PIP_ARGS=(--index-url "$INDEX_URL" --timeout 120 --retries 10)
UV=/home/ps/.local/bin/uv
PYTHON=/home/ps/.local/share/uv/python/cpython-3.10-linux-x86_64-gnu/bin/python3.10
if [[ ! -x "$RL_ROOT/.venv310/bin/python" ]]; then
  "$UV" venv --python "$PYTHON" --seed "$RL_ROOT/.venv310"
fi
"$RL_ROOT/.venv310/bin/python" -m pip install "${PIP_ARGS[@]}" --upgrade "pip<25" setuptools wheel
"$RL_ROOT/.venv310/bin/pip" install "${PIP_ARGS[@]}" -r "$RL_ROOT/requirements-train.txt"
"$RL_ROOT/.venv310/bin/pip" install "${PIP_ARGS[@]}" -r "$RL_ROOT/requirements-backend.txt"
"$RL_ROOT/.venv310/bin/pip" install --no-build-isolation --no-deps -e "$ROOT"
echo "Training environment ready: $RL_ROOT/.venv310"
