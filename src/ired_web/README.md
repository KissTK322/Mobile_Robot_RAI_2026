# ired_web

Phone / tablet / PC web app for the iRED robot. Open it on any device on the
same Wi-Fi as the robot:

    http://robotg5.local:8080        (Android: use the robot IP, e.g. http://172.16.0.15:8080)

On iPad / iPhone: Share → *Add to Home Screen* to open it like an app.

## Pages

| Page   | What it does |
|--------|--------------|
| Status | CPU, temperature, fan, memory, Wi-Fi, power flags; live rate of the main topics; running nodes |
| Launch | One button per mode: **Robot** (bringup), **Camera + ArUco**, **Mapping**, **Navigation**, with state and live log. *Stop all* stops everything cleanly |
| Drive  | Two touch pads (move, turn) with speed limits. Robot moves only while a pad is held. Keyboard: W/S, A/D, Q/E |
| Test   | Calibration tests with the LiDAR as the ruler: **turn in place** (IMU / wheel / EKF yaw error), **drive straight** (distance error, side and heading drift), **wheel step test** (PID tracking per wheel, wheels lifted), set or restore wheel PID gains. Results are kept and can be downloaded as CSV |
| Camera | Live camera with ArUco overlay and the detected markers |
| STOP   | Always in the top bar. Latches the robot at zero speed until *RELEASE*. Space bar on a PC |

The hardware emergency button on the robot is still the real E-STOP; the app's
STOP only commands zero velocity.

## Tests

Each test is a small script in `ired_web/tests/` started by the manager as
`ros2 run ired_web test_*` only while it runs (one at a time), so it costs
nothing when idle. They need Robot mode running, refuse to start while STOP is
latched, and pressing STOP aborts them. Results go to
`~/.ros/ired_web/results.jsonl` (CSV download on the Test page).

They also run from a terminal, e.g. `ros2 run ired_web test_rotate 1 -0.5`
(one clockwise turn) or `ros2 run ired_web test_straight 1.0 0.15`.

## How it runs

- `ired_web/manager.py`: HTTP server on port 8080 (serves `web/` and a small
  JSON API), starts/stops modes in their own process group, reads Pi health,
  publishes zero `/cmd_vel` while STOP is latched.
- `launch/web.launch.py`: rosbridge (9090), rosapi, web_video_server (8081),
  foxglove_bridge (8765). Started automatically by the manager.
- `config/web.yaml`: **PIN**, ports and the mode list (commands, requires,
  conflicts). Add a new mode here, no code change needed.
- `systemd/ired-web.service`: starts the manager at boot.

## Install / update

```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server ros-jazzy-foxglove-bridge
cd ~/ired_ws && colcon build --symlink-install --packages-select ired_web
sudo cp ~/ired_ws/src/ired_web/systemd/ired-web.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now ired-web
```

Useful:

```bash
sudo systemctl restart ired-web       # after changing web.yaml or manager.py
journalctl -u ired-web -f             # manager log
ls ~/.ros/ired_web/                   # one log file per mode
```

## Security

Every API call needs the PIN (5 wrong tries = 30 s lock). The rosbridge
(9090) and Foxglove (8765) ports have no password, so only use the robot on
a network you trust.

## Foxglove

Foxglove Studio (desktop app on the PC): *Open connection* → Foxglove
WebSocket → `ws://robotg5.local:8765`.
