#!/usr/bin/env bash
set -e
source ~/anaconda3/etc/profile.d/conda.sh
conda activate cathsim-newton
cd ~/cathsim-warp
export CATHSIM_PHYSICS_ENGINE=newton_demo
# D3 validated config (doc/08): variable-radius tube wall + graded soft-anchor,
# 6 substeps x 2 VBD iters ~= 60fps, contained on aorta_tree (no 穿管).
export CATHSIM_NEWTON_SUBSTEPS=6
export CATHSIM_NEWTON_PUSH_SPEED=0.05
export CATHSIM_PORT=9000
exec python -m uvicorn services.main:app --host 0.0.0.0 --port 9000
