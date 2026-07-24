#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ps/cathsim-warp
RL_ROOT="$ROOT/reinforcement_learning"
cd "$ROOT"
PYTHONPATH="$ROOT:$ROOT/src" "$RL_ROOT/.venv310/bin/python" "$RL_ROOT/scripts/smoke_test.py"
