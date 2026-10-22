#!/bin/bash
set -e
TOOLS=$(cd "$(dirname "$0")" && pwd)
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture stop --name stop-final > "$TOOLS/stop-final.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture lease-loss --name lease-final > "$TOOLS/lease-final.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture mqtt-loss --name mqtt-final > "$TOOLS/mqtt-final.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 2 --seed 11 --fixture game --name game-2 > "$TOOLS/game-2.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 3 --seed 11 --fixture game --name game-3 > "$TOOLS/game-3.log" 2>&1
bash "$TOOLS/run_sim_check.sh" --boxes 4 --seed 11 --fixture game --name game-4 > "$TOOLS/game-4.log" 2>&1
