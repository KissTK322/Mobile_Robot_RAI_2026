# ired_challenge

Programs for the Industrial Logistics Challenge.

## Navigation challenge

```bash
ros2 launch ired_bringup bringup.launch.py
ros2 launch ired_navigation navigation.launch.xml
ros2 run ired_refbox mqtt_refbox
ros2 launch ired_challenge navigation_challenge.launch.py
```

What it does (`ired_challenge/navigation_task.py`):

1. Waits until the refbox says the game is running (`/refbox/game_play`).
2. Drives through the red zone **C_Z32** first (required by the rules).
3. Visits every zone in `/refbox/navigation_routes` (`remaining` list), stays
   5.5 s inside each and waits until the refbox drops it from `remaining`.
4. A failed drive is retried twice (costmaps cleared); after that the zone is
   moved to the end and tried again later. If the refbox does not count a
   zone, the robot drives to the zone again.
5. If an MPS station stands in a target zone, the robot stops beside it,
   inside the zone, on the side it comes from.
6. If the game stops, the robot stops and continues later from where it was.

The robot keeps its heading while driving (omni base, no turning needed).
State is published as JSON on `/challenge/navigation/state`.

Settings: `config/navigation_challenge.yaml` (zone size, map origin, red zone,
dwell time, retries, visit order).

## Things to check on the field

- The map origin must be the outer corner of C_Z11 (or set `origin_x/y`).
- Whether the zones must be visited in the refbox order (`order: given`) or
  any order (`order: nearest`, about half the driving).
- That the refbox `remaining` list starts with all zones and shrinks as
  zones are counted (assumed from `ired_refbox`).

## Tests (no robot, no ROS needed)

```bash
cd src/ired_challenge && PYTHONPATH=. python3 -m pytest -q test
```

They simulate a whole challenge with a fake Nav2 and a fake refbox: all 12
zones, red zone first, retries, giving up and coming back, game pause, MPS in
a zone, refbox not counting a zone.
