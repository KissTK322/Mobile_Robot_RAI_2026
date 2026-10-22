#!/bin/bash
set -e
TOOLS=$(cd "$(dirname "$0")" && pwd)
# Wait for the owned sequential batch; never overlap ROS simulation sets.
while pgrep -f '[r]un_matrix.sh' >/dev/null; do sleep 2; done
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture rearm --name rearm-final > "$TOOLS/rearm-final.log" 2>&1
