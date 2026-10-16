#!/usr/bin/env python3
"""
aruco_ros2_node.py
--------------------
ArUco marker detection as a real ROS2 (Jazzy) node, so it plugs into the
rest of the iRED nav stack instead of running as a standalone script.

No custom message package needed - built with only core ROS2 interfaces
(sensor_msgs, std_msgs) that are already installed as part of
ros-jazzy-*, and constructs the Image message by hand instead of using
cv_bridge, so there's nothing extra to apt-install.

Camera is calibrated now (camera_calibration.npz from
camera_calib_capture.py + camera_calib_compute.py), so this publishes REAL
3D pose per marker via cv2.solvePnP - not just 2D pixel position anymore:
    - distance_m               : straight-line distance from camera lens to marker, meters
    - position_base_link_m     : [x, y, z] of the marker in the robot's base_link
                                  frame (REP-103: x=forward, y=left, z=up), IF the
                                  base_link -> aruco_camera_link TF is available
    - marker_yaw_deg           : which way the marker's face is pointing, as an angle
                                  in the base_link XY-plane (0 deg = facing straight
                                  back at the robot). Rough/approximate - sanity check
                                  it empirically (see note below) before trusting it
                                  for anything like docking alignment.

This still ALSO tags the image and detections with the REAL camera frame
(aruco_camera_link) from your URDF, and looks up the live
base_link -> aruco_camera_link transform published by robot_state_publisher
(using the origin xyz/rpy you measured and filled into ired.urdf.xacro).

NEW: every detected marker is also broadcast as a live TF frame
(aruco_marker_<id>, child of aruco_camera_link) - open RViz, add a "TF"
display, and you'll see a frame pop into existence right where the marker
is, moving live as the robot/camera moves, and disappearing ~10s after the
marker leaves view. This is on top of (not instead of) the existing
/aruco/detections JSON topic.

IMPORTANT assumption baked into position_base_link_m / marker_yaw_deg:
solvePnP returns pose in OpenCV's OPTICAL camera convention (x=right,
y=down, z=out-of-lens/forward), but your URDF's aruco_camera_link is
assumed to follow the normal ROS/REP-103 mechanical convention
(x=forward, y=left, z=up) - i.e. you measured the joint's rpy as if
"forward" for the camera link means "the direction the lens points".
That's the standard way people wire up a URDF camera link, and is very
likely what you did - but if position_base_link_m looks rotated 90 deg
from what you expect, this is the first thing to double check.

Sanity check before trusting any of this: point the camera straight at a
marker held flat, dead ahead. distance_m should roughly match a tape
measure. position_base_link_m's x should be roughly
"how far forward" the marker is, y should be ~0 if the marker is
straight ahead (positive if it's to the robot's LEFT), z should be close
to the marker's real height above the ground minus nothing weird.

Topics published:
    /aruco/image_raw       (sensor_msgs/Image)   - annotated camera feed,
                                                     header.frame_id = camera_frame_id
    /aruco/detections       (std_msgs/String)      - JSON object, e.g.
        {
          "camera_frame": "aruco_camera_link",
          "base_frame": "base_link",
          "camera_offset_from_base_m": [0.0, 0.0, 0.335],
          "markers": [
            {
              "id": 12, "cx": 320.5, "cy": 180.2, "angle_deg": 87.3,
              "distance_m": 0.842,
              "position_base_link_m": [0.81, -0.05, 0.12],
              "marker_yaw_deg": 178.4
            },
            ...
          ]
        }
    (position_base_link_m / marker_yaw_deg are null if calibration failed to
    load, or if the base_link -> aruco_camera_link TF isn't available yet;
    distance_m/position/yaw are all omitted entirely if calibration wasn't
    loaded at all - you still get id/cx/cy/angle_deg either way)

Parameters (set with --ros-args -p name:=value):
    camera_index      (int,    default 0)
    frame_width        (int,    default 1280)
    frame_height        (int,    default 720)
    aruco_dict          (string, default "DICT_ARUCO_ORIGINAL")
    publish_rate        (double, default 15.0)   # Hz
    camera_frame_id     (string, default "aruco_camera_link")  # must match your URDF link name
    base_frame_id       (string, default "base_link")
    calibration_file     (string, default "~/ired_ws/camera_calib/camera_calibration.npz")
    marker_size_m       (double, default 0.07)   # real marker edge length, meters -
                                                     CONFIRM this against your actual
                                                     competition markers, it directly
                                                     scales distance_m/position

Run this ALONGSIDE bringup.launch.py on the Pi (needs robot_state_publisher
already running and publishing /tf, /tf_static - so start bringup first):
    source ~/ros_environment.bash
    ros2 launch ired_bringup bringup.launch.py   # terminal 1
    python3 aruco_ros2_node.py --ros-args -p camera_index:=0   # terminal 2

Quick check without any viewer, just read the terminal:
    ros2 topic echo /aruco/detections --full-length
    ros2 run tf2_ros tf2_echo base_link aruco_camera_link   # confirm the TF chain independently

To actually see the annotated video from your PC/VM (needs the Pi and PC
on the same ROS_DOMAIN_ID and reachable over the network - skip this for
now if you haven't set that up):
    ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/aruco/image_raw
"""

import json
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class ArucoDetectorCompat:
    """Wraps both the old (<=4.6) and new (>=4.7) cv2.aruco APIs."""

    def __init__(self, dict_name: str):
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f"Unknown ArUco dictionary name: {dict_name}")
        dict_id = getattr(cv2.aruco, dict_name)

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
            self._new_api = True
        else:
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

    def detect(self, gray):
        if self._new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.params
            )
        return corners, ids


def marker_angle_deg(corner):
    pts = corner.reshape(4, 2)
    dx = pts[1][0] - pts[0][0]
    dy = pts[1][1] - pts[0][1]
    return math.degrees(math.atan2(dy, dx))


# Fixed hardware-convention rotation: converts a direction/vector expressed
# in OpenCV's OPTICAL camera frame (x=right, y=down, z=forward/out-of-lens)
# into the ROS/REP-103 MECHANICAL convention (x=forward, y=left, z=up) that
# your URDF's aruco_camera_link is assumed to use. This is a constant - it
# has nothing to do with how the camera is mounted/tilted on the robot
# (that part is handled separately by the live base_link->aruco_camera_link
# TF, which already encodes the real xyz/rpy you measured).
OPTICAL_TO_MECH = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def quaternion_to_matrix(x, y, z, w):
    """Standard quaternion -> 3x3 rotation matrix, no external deps."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


def matrix_to_quaternion(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion, no external deps."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("aruco_dict", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("camera_frame_id", "aruco_camera_link")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter(
            "calibration_file",
            os.path.expanduser("~/ired_ws/camera_calib/camera_calibration.npz"),
        )
        self.declare_parameter("marker_size_m", 0.07)

        index = self.get_parameter("camera_index").value
        width = self.get_parameter("frame_width").value
        height = self.get_parameter("frame_height").value
        dict_name = self.get_parameter("aruco_dict").value
        rate = self.get_parameter("publish_rate").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        calib_path = os.path.expanduser(self.get_parameter("calibration_file").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)

        # --- Camera calibration (needed for real 3D pose via solvePnP) ---
        self.have_calib = False
        self.camera_matrix = None
        self.dist_coeffs = None
        try:
            data = np.load(calib_path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            self.have_calib = True
            self.get_logger().info(
                f"Loaded camera calibration from {calib_path} "
                f"(marker_size_m={self.marker_size_m})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not load camera calibration from {calib_path} ({e}). "
                f"Detections will still publish id/cx/cy/angle_deg, but "
                f"distance_m/position_base_link_m/marker_yaw_deg will be omitted."
            )

        # Object points for solvePnP: one ArUco marker's 4 corners in its
        # OWN coordinate frame, centered at the marker's center, z=0 plane.
        # Order (top-left, top-right, bottom-right, bottom-left) matches
        # what cv2.aruco returns for detected corners.
        half = self.marker_size_m / 2.0
        self.marker_obj_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        self.solvepnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

        # TF2: lets us ask "where is aruco_camera_link relative to base_link
        # right now?" using the transform robot_state_publisher is already
        # broadcasting from your URDF's aruco_camera_joint origin.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warned = False

        # Broadcasts a live TF frame per detected marker (aruco_marker_<id>,
        # child of camera_frame_id) so RViz's TF display shows exactly where
        # each marker is right now, composed automatically through the
        # existing base_link -> aruco_camera_link transform.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.detector = ArucoDetectorCompat(dict_name)

        self.image_pub = self.create_publisher(Image, "/aruco/image_raw", 10)
        self.detections_pub = self.create_publisher(String, "/aruco/detections", 10)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f"aruco_detector started: camera_index={index} dict={dict_name} "
            f"rate={rate}Hz -> /aruco/image_raw, /aruco/detections"
        )

    def estimate_marker_pose(self, corner):
        """Returns (distance_m, tvec_optical, rvec) for one marker's corners,
        using solvePnP + this node's loaded calibration. tvec_optical is in
        OpenCV's optical camera frame (x=right, y=down, z=forward)."""
        img_points = corner.reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            self.marker_obj_points, img_points,
            self.camera_matrix, self.dist_coeffs,
            flags=self.solvepnp_flag,
        )
        if not ok:
            return None
        tvec = tvec.reshape(3)
        distance_m = float(np.linalg.norm(tvec))
        return distance_m, tvec, rvec

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Frame grab failed", throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detector.detect(gray)
        now = self.get_clock().now().to_msg()

        # Look up the live base_link -> aruco_camera_link transform once per
        # frame (shared by every marker seen this frame), i.e. the origin
        # xyz you measured and filled into aruco_camera_joint in
        # ired.urdf.xacro, as robot_state_publisher is actually broadcasting
        # it right now.
        camera_offset = None
        R_bc = None
        t_bc = None
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.camera_frame_id, rclpy.time.Time()
            )
            tr = t.transform.translation
            rot = t.transform.rotation
            camera_offset = [round(tr.x, 4), round(tr.y, 4), round(tr.z, 4)]
            R_bc = quaternion_to_matrix(rot.x, rot.y, rot.z, rot.w)
            t_bc = np.array([tr.x, tr.y, tr.z])
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"Can't look up {self.base_frame_id} -> {self.camera_frame_id} yet "
                    f"({e}). Is bringup.launch.py (robot_state_publisher) running? "
                    f"Detections will still publish, just without camera_offset_from_base_m "
                    f"/ position_base_link_m."
                )
                self._tf_warned = True

        detections = []
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, marker_id in zip(corners, ids.flatten()):
                pts = c.reshape(4, 2)
                cx, cy = pts.mean(axis=0)
                angle = marker_angle_deg(c)
                label = f"id={marker_id} {angle:5.0f}deg"

                det = {
                    "id": int(marker_id),
                    "cx": round(float(cx), 1),
                    "cy": round(float(cy), 1),
                    "angle_deg": round(float(angle), 1),
                }

                if self.have_calib:
                    pose = self.estimate_marker_pose(c)
                    if pose is not None:
                        distance_m, tvec_opt, rvec = pose
                        det["distance_m"] = round(distance_m, 3)
                        label += f" {distance_m:4.2f}m"

                        # camera-optical -> camera-mechanical (fixed conversion,
                        # see OPTICAL_TO_MECH). This is the marker's pose
                        # relative to aruco_camera_link's OWN axes - doesn't
                        # need base_link/TF at all yet.
                        pos_mech = OPTICAL_TO_MECH @ tvec_opt
                        R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                        R_marker_to_mech = OPTICAL_TO_MECH @ R_marker_to_cam

                        # Broadcast it as a live TF frame so RViz (Displays >
                        # TF, enable "Show Names"/"Show Axes") shows exactly
                        # where this marker is right now, relative to the
                        # camera - and, transitively, to base_link/map since
                        # aruco_camera_link is already in the tree. Frame
                        # disappears from RViz ~10s after the marker is no
                        # longer seen (TF cache timeout) - that's expected.
                        tf_msg = TransformStamped()
                        tf_msg.header.stamp = now
                        tf_msg.header.frame_id = self.camera_frame_id
                        tf_msg.child_frame_id = f"aruco_marker_{int(marker_id)}"
                        tf_msg.transform.translation.x = float(pos_mech[0])
                        tf_msg.transform.translation.y = float(pos_mech[1])
                        tf_msg.transform.translation.z = float(pos_mech[2])
                        qx, qy, qz, qw = matrix_to_quaternion(R_marker_to_mech)
                        tf_msg.transform.rotation.x = qx
                        tf_msg.transform.rotation.y = qy
                        tf_msg.transform.rotation.z = qz
                        tf_msg.transform.rotation.w = qw
                        self.tf_broadcaster.sendTransform(tf_msg)

                        det["position_base_link_m"] = None
                        det["marker_yaw_deg"] = None
                        if R_bc is not None:
                            pos_base = R_bc @ pos_mech + t_bc
                            det["position_base_link_m"] = [
                                round(float(v), 3) for v in pos_base
                            ]

                            face_normal_mech = R_marker_to_mech[:, 2]  # marker's local +z, in mech frame
                            face_normal_base = R_bc @ face_normal_mech
                            yaw = math.degrees(math.atan2(
                                face_normal_base[1], face_normal_base[0]
                            ))
                            det["marker_yaw_deg"] = round(float(yaw), 1)

                cv2.putText(
                    frame, label,
                    (int(cx) - 40, int(cy) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                )
                detections.append(det)

        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.camera_frame_id
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()
        self.image_pub.publish(img_msg)

        det_msg = String()
        det_msg.data = json.dumps({
            "camera_frame": self.camera_frame_id,
            "base_frame": self.base_frame_id,
            "camera_offset_from_base_m": camera_offset,
            "markers": detections,
        })
        self.detections_pub.publish(det_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
