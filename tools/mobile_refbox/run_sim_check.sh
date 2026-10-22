#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/ired/ired_work/sim_ws/install/setup.bash
source /home/ired/ired_work/sim_tools/mobile_refbox_ws/install/setup.bash
export ROS_DOMAIN_ID=77 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 REFBOX_BRIDGE_ADDRESS=127.0.0.1 ROBOT_IP=ired-practice
TASK_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
export PYTHONPATH="$TASK_ROOT/src/ired_web:$TASK_ROOT/src/ired_challenge:$PYTHONPATH"
exec python3 "$TASK_ROOT/tools/mobile_refbox/run_sim_check.py" "$@"
