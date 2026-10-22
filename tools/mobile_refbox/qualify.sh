#!/bin/bash
set -e
TOOLS=$(cd "$(dirname "$0")" && pwd)
while pgrep -f '[a]fter_matrix.sh' >/dev/null; do sleep 2; done
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture rearm --name qualified-rearm > "$TOOLS/qualified-rearm.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture game --name qualified-2 > "$TOOLS/qualified-2.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 3 --seed 11 --fixture game --name qualified-3 > "$TOOLS/qualified-3.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 4 --seed 11 --fixture game --name qualified-4 > "$TOOLS/qualified-4.log" 2>&1
