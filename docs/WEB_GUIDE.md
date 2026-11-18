# Web app guide (ired_web)

The robot runs a web app at boot (systemd service `ired-web`). Open it from any phone or laptop on the
same Wi-Fi:

    http://robotg5.local:8080        (or http://<robot IP>:8080)

Enter the team PIN once; the browser remembers it. The red **STOP** button in the top bar is always
there: it holds the robot at zero speed until you press RELEASE (the hardware E-STOP is still the real one).

## Pages

| Tab | Use it for |
|-----|-----------|
| **Ready** | one checklist before a game: STOP released, Robot mode, wheel board, LiDAR, Navigation, start pose on C_Z51, challenge + refbox, Pi temperature and CPU |
| **Status** | Pi health (CPU, temperature, Wi-Fi, power), live rate of the main topics, running nodes |
| **Launch** | start / stop each mode: Robot, Camera + ArUco, Mapping, Navigation profiles, challenges, Record; live log of each |
| **Drive** | two touch pads (move, turn); the robot moves only while a pad is held |
| **Map** | live map and robot pose, save / select / clean maps, mark zone centres |
| **Game** | RefBox game: choose **Field refbox** or **Practice**, set the refbox (Bridge) address, **Prepare** buttons, practice layouts and scoring |
| **Test** | calibration tests: turn in place, drive straight, wheel PID step; results kept and downloadable as CSV |
| **Camera** | live camera with ArUco overlay and the marker list |
| **Replay** | 3D replay of any recorded game (see below) |

## Game with the real RefBox

1. Game tab: source **Field refbox**, refbox address = IP of the PC running the RefBox Bridge, Save.
2. Put the robot on C_Z51 facing C_Z41.
3. Press **Prepare: Navigation** (or **Prepare: Exploration**). It starts Robot, Navigation, the camera
   (exploration) and the challenge, waits for each one, and sets the start pose.
4. Launch tab: press **Record** so the game is saved for Replay.
5. The referee presses Setup, then Production. The robot leaves START by itself.

## Practice game (no RefBox server)

1. Launch: start **Robot** and a **Navigation** profile first.
2. Game tab: source **Practice** → **Use selected source** (starts a local MQTT broker on the robot).
3. Choose the mode, **Generate field** or **Load** a saved layout, place the boxes as shown.
4. **Prepare robot for this practice game**, press Record, tick the confirm box, **Start game**.
   The page scores every report like the course table (+3.5 zone, ±2 rotation, −2 wrong zone).

## Replay

Every game recorded with **Record** is a rosbag in `~/bags` on the robot. Open the **Replay** tab, pick a
bag and press **Convert** (a few seconds; refused while a challenge runs), then **Open**. You see the field
in 3D with the robot path, LiDAR points, camera marker sightings and the challenge events; play at
1–8×, scrub the time, switch Tilt / Top / Follow views. `replay.html?bag=<name>` opens one bag directly.

## Why the camera stream has little lag

- The image does not go through rosbridge (JSON + base64 is slow). `web_video_server` on the robot
  encodes the ROS image to **MJPEG** and the browser shows it in a plain `<img>`; no JavaScript decoding.
- It is scaled to 640×360 at JPEG quality 60 before sending, and the camera publishes at 15 Hz.
- The stream only runs while the Camera tab is open; ArUco results come separately as small JSON.

## For maintainers

- Server: `src/ired_web/ired_web/manager.py` (HTTP + JSON API, port 8080); pages in `src/ired_web/web/`.
- Modes are defined in `src/ired_web/config/web.yaml` (command, requires, conflicts); no code change needed.
- `sudo systemctl restart ired-web` after an update; logs: `journalctl -u ired-web -f`, `~/.ros/ired_web/`.
