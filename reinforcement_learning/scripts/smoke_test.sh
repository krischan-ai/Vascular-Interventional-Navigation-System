#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RL_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$RL_ROOT/.." && pwd)"
PYTHON="${CATHSIM_RL_PYTHON:-$RL_ROOT/.venv310/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  echo "RL Python not found: $PYTHON" >&2
  echo "Run scripts/setup.sh or set CATHSIM_RL_PYTHON." >&2
  exit 1
fi
cd "$ROOT"
PYTHONPATH="$ROOT:$ROOT/src" "$PYTHON" "$RL_ROOT/scripts/smoke_test.py"
