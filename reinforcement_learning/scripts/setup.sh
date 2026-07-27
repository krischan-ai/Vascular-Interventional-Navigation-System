#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RL_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$RL_ROOT/.." && pwd)"
INDEX_URL="${CATHSIM_RL_PIP_INDEX_URL:-https://mirrors.huaweicloud.com/repository/pypi/simple}"
PIP_ARGS=(--index-url "$INDEX_URL" --timeout 120 --retries 10)
UV="${CATHSIM_UV_BIN:-$(command -v uv || true)}"
PYTHON="${CATHSIM_PYTHON_BIN:-$(command -v python3.10 || true)}"
if [[ -z "$PYTHON" ]]; then
  echo "Python 3.10 is required. Set CATHSIM_PYTHON_BIN to its executable." >&2
  exit 1
fi
if [[ ! -x "$RL_ROOT/.venv310/bin/python" ]]; then
  if [[ -n "$UV" ]]; then
    "$UV" venv --python "$PYTHON" --seed "$RL_ROOT/.venv310"
  else
    "$PYTHON" -m venv "$RL_ROOT/.venv310"
  fi
fi
"$RL_ROOT/.venv310/bin/python" -m pip install "${PIP_ARGS[@]}" --upgrade "pip<25" setuptools wheel
"$RL_ROOT/.venv310/bin/pip" install "${PIP_ARGS[@]}" -r "$RL_ROOT/requirements-train.txt"
"$RL_ROOT/.venv310/bin/pip" install "${PIP_ARGS[@]}" -r "$RL_ROOT/requirements-backend.txt"
"$RL_ROOT/.venv310/bin/pip" install --no-build-isolation --no-deps -e "$ROOT"
echo "Training environment ready: $RL_ROOT/.venv310"
