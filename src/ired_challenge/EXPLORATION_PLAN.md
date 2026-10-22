# Exploration challenge plan

Goal: find the 2 MPS stations (20 x 20 x 30 cm boxes, ArUco 7 x 7 cm on the
input and output sides), report each one's **zone** (+3.5) and **rotation**
(+2) to the refbox within 10 min, after passing the red zone C_Z32. A wrong
position costs -2, so it is better to report late than wrong.

## Main problem: the LiDAR does not see the boxes

The LiDAR scan plane is about 33-35 cm above the floor (3D model), the boxes
are 30 cm tall. So:

- Nav2 cannot avoid the boxes on its own, the robot can drive into them.
- "Compare the scan with the empty-field map" does not work.

Until the LiDAR is lowered (e.g. to plate L2, scan plane ~25 cm), the boxes
must come from the camera.

## Strategy

1. **Red zone**: drive through C_Z32 (same as the navigation challenge).
2. **Look-around points**: drive to a few points that together see the whole
   field, and at each one turn slowly 360 deg (about 0.4 rad/s so the camera
   image stays sharp). Candidate points: C_Z22, C_Z42, C_Z44, C_Z24, C_Z33.
   The C270 detects 7 cm markers reliably up to about 1.5-2 m; this is to be
   measured (see "Field checks").
3. **Every ArUco sighting** goes into `MachineEstimator`
   (`ired_challenge/exploration.py`): marker -> box centre (10 cm behind the
   marker) -> zone, input-side direction -> refbox rotation. Sightings
   farther than 2 m are ignored.
4. **Boxes become obstacles**: as soon as a box is estimated, publish its
   20 x 20 cm footprint (plus margin) into the Nav2 costmaps, so the robot
   drives around it although the LiDAR cannot see it.
5. **Get closer if unsure**: when a machine has fewer than 3 agreeing
   sightings, drive to a point 0.8-1.0 m in front of the seen marker and look
   again.
6. **Report** a machine once it is `ready` (>= 3 sightings within 15 cm),
   through `/refbox/machine_report`; then continue with the next one. Stop
   when both are reported or at ~9 min report the best estimate (score
   floor is 0, and an unreported machine scores 0 anyway).

## Field checks before trusting it

- **Rotation convention**: in the navigation challenge the refbox sends
  `machine_info` with each MPS zone and rotation. Place a box, read its
  rotation from the refbox, and compare with `refbox_rotation()` from the
  robot's sighting. Adjust `zero_deg` / `sign` if they differ.
- **Marker ids**: the slides list CS1 101/102 and BS 121/122 (cyan). Fill
  in the other machines in `DEFAULT_MARKERS`.
- **Detection range and accuracy**: markers at 0.5 / 1 / 1.5 / 2 m and
  0 / 30 / 60 deg, compare the reported box centre with a tape measure. This
  sets `max_range` and the look-around points.
- **Report message**: what x / y mean in `MachineReport` (metres or zone
  numbers) and whether `yaw` must be the refbox rotation.

## Already written and tested (no ROS)

- `field.py`: zone math (shared with the navigation challenge).
- `exploration.py`: marker -> map, box centre, input direction, refbox
  rotation snapped to 45 deg, `MachineEstimator` with median / agreement /
  readiness.
- `test/test_exploration.py`: simulated noisy sightings from 4 robot poses
  give the right zone and rotation; unknown ids, far markers and missing
  poses are ignored.

## Still to write (after the field checks)

- ROS node: subscribe `/aruco/detections` (JSON), robot pose from TF, drive
  the look-around points with Nav2, spin, feed the estimator, publish box
  obstacles, call `/refbox/machine_report`.
- Costmap layer for the estimated boxes.
