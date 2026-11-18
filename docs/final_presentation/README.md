# Final presentation material

| File | Content |
|------|---------|
| `slide_connect.png` | component connection diagram (power, USB / serial, Wi-Fi) |
| `slide_software.png` | how the ROS 2 software uses the hardware (driving loop, game loop, web app) |
| `slide_evaluate.png` | test levels, metrics and results before tuning / after tuning / race |
| `slide_errors.png` | error analysis of the race bags: odometry drift, zone-centre error |
| `slide_bench.png` | bench tests and the same error numbers over 13 recorded real games |
| `slide_performance.png` | Pi 5 CPU / temperature with each part running, camera and recording techniques (`pi_load.jsonl`) |
| `race_*_route.png` | robot path of the two race games (16 Nov 2026) |
| `race_*_replay.mp4` | 2× top-down replay of the race games, made from the rosbags (no camera images were recorded) |
| `race_eval.json`, `games_eval.json` | numbers behind the slides |

Regenerate (on the team PC, bags in `~/ired_work/robot_runs/bags`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ired_ws/install/setup.bash
python3 tools/analysis/race_eval.py && python3 tools/analysis/games_eval.py   # numbers
python3 tools/analysis/race_plots.py                                          # route figures
python3 tools/analysis/race_data.py && python3 tools/analysis/race_video.py   # replay videos
python3 docs/final_presentation/make_slides.py                                # slide images
```

The scripts use the team PC's absolute paths (`/home/ired/ired_work/...`); change `BAGS` / `OUT` to run elsewhere.
