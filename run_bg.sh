#!/usr/bin/env bash
# Detached runner: survives SSH/session close. Logs to spikes/logs/<name>.log
# Usage: ./run_bg.sh <name> <command...>
set -euo pipefail
name="$1"; shift
mkdir -p ~/cathsim-warp/spikes/logs
log=~/cathsim-warp/spikes/logs/"$name".log
source ~/anaconda3/etc/profile.d/conda.sh
conda activate cathsim-newton
cd ~/cathsim-warp
setsid bash -c "$*" >"$log" 2>&1 < /dev/null &
echo $! > ~/cathsim-warp/spikes/logs/"$name".pid
echo "started $name pid=$(cat ~/cathsim-warp/spikes/logs/$name.pid) log=$log"
