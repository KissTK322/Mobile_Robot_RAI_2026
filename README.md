# iRED robot · Group 5 (AOW LOEY) · KMITL Mobile Robots 2026

ROS 2 Jazzy workspace of our omni robot for the Industrial Logistics Challenge
(Navigation: visit 12 RefBox zones; Exploration: find, identify and report 2 MPS stations).
Race result (16 Nov 2026): Navigation 11/11 (12/12 zones in 124 s), Exploration 11/11 (2/2 stations in 57 s).

The code that ran in the race is branch **`proposal/field-tune-1030`**.

## Hardware

| Part | Connection |
|------|------------|
| Raspberry Pi 5 (4 GB), Ubuntu 24.04 | main computer, Wi-Fi to the team laptop |
| iRED controller board | USB (CP2102N) · Modbus RTU 115200 · `/dev/ttyMicro`; runs the wheel PID and reads the IMU |
| 4 × DC motor + encoder, omni wheels Ø 130 mm at ±45°, ±135° | driven by the iRED board |
| RPLIDAR A1 | USB (CP2102) 115200 · `/dev/ttyLiDAR` |
| Logitech C270 | USB webcam, ArUco markers (DICT_ARUCO_ORIGINAL, calibrated 640×480) |
| 2 × Makita 12 V 4.0 Ah in series (24 V) | motors; DC-DC buck (12–80 V in, 800 W 25 A) gives 5 V to the Pi |

## Packages (`src/`)

| Package | What it does |
|---------|--------------|
| `ired_bringup` | robot bringup: Modbus serial node, forward/inverse kinematics, IMU and odometry nodes, EKF, URDF, RPLIDAR, ArUco camera node |
| `ired_navigation` | maps, SLAM (Cartographer) and Nav2 / AMCL parameter sets (omni, soft stop, fast) |
| `ired_challenge` | the two challenge programs: `navigation_node` (12 zones from the RefBox, 5 s stay) and `exploration_node` (LiDAR box finder + camera confirmation, report zone and rotation) |
| `ired_web` | web app on the robot (port 8080): launch modes, drive, map, game / practice RefBox, calibration tests, camera, Record and 3D Replay. See [docs/WEB_GUIDE.md](docs/WEB_GUIDE.md) |
| `ired_msgs` | robot messages (MotorData, IMU) |
| `rplidar_ros` | LiDAR driver |

The RefBox link is `ired_refbox/mqtt_refbox` (separate repository) talking MQTT to the team's RefBox Bridge.

## Build and run

```bash
cd ~/ired_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=105
```

Day to day everything is started from the web app (`http://<robot>:8080`, PIN protected):
**Prepare** buttons on the Game page start Robot, Navigation, Camera and the challenge in the right order.
By hand:

```bash
ros2 launch ired_bringup bringup.launch.py                                   # robot
ros2 launch ired_navigation navigation.launch.xml map_name:=<map>.yaml param_name:=navigation_softstop_quick.yaml
ros2 run ired_challenge exploration_challenge --ros-args -p search_mode:=lidar
```

## How we tested

Bench tests on the web Test page (LiDAR as the ruler), a PC simulation with a fake RefBox, a practice mode
with a local RefBox on the robot, and real RefBox games recorded with rosbag and replayed in 3D.
Error numbers over 13 recorded games: odometry alone drifts about 4.5 % of the distance driven, AMCL keeps
the stop within about 5 cm of each zone centre.
