import argparse
import math
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler

import cv2
import numpy as np

camera = None  # set in __main__


# ---------------------------------------------------------------------------
# ArUco detection wrapper - handles both the old (<=4.6) and new (>=4.7)
# OpenCV cv2.aruco APIs, since the version on your Pi's apt package may
# differ from what you develop with elsewhere.
# ---------------------------------------------------------------------------
class ArucoDetectorCompat:
    def __init__(self, dict_name: str):
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f"Unknown ArUco dictionary name: {dict_name}")
        dict_id = getattr(cv2.aruco, dict_name)

        if hasattr(cv2.aruco, "ArucoDetector"):
            # OpenCV >= 4.7 API
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
            self._new_api = True
        else:
            # OpenCV <= 4.6 API
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
    """Approximate in-image rotation: angle of the top edge (corner0->corner1)."""
    pts = corner.reshape(4, 2)
    dx = pts[1][0] - pts[0][0]
    dy = pts[1][1] - pts[0][1]
    return math.degrees(math.atan2(dy, dx))


class Camera:
    def __init__(self, index=0, width=640, height=480, dict_name="DICT_4X4_50"):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.detector = ArucoDetectorCompat(dict_name)
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.last_print = 0.0
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = self.detector.detect(gray)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                lines = []
                for c, marker_id in zip(corners, ids.flatten()):
                    pts = c.reshape(4, 2)
                    cx, cy = pts.mean(axis=0)
                    angle = marker_angle_deg(c)
                    cv2.putText(
                        frame, f"id={marker_id} {angle:5.0f}deg",
                        (int(cx) - 40, int(cy) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                    )
                    lines.append(f"id={marker_id} center=({cx:.0f},{cy:.0f}) angle={angle:.0f}deg")

                now = time.time()
                if now - self.last_print > 1.0:  # throttle console spam to 1/sec
                    print("[DETECT] " + " | ".join(lines))
                    self.last_print = now

            with self.lock:
                self.frame = frame

    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            frame = self.frame.copy()
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes() if ok else None

    def stop(self):
        self.running = False
        self.thread.join(timeout=1)
        self.cap.release()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><body style='background:#111'>"
                b"<h3 style='color:white;font-family:sans-serif'>Pi ArUco Detection</h3>"
                b"<img src='/stream.mjpg' /></body></html>"
            )
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    jpg = camera.get_jpeg()
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live ArUco marker detection from Pi webcam")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--dict", type=str, default="DICT_4X4_50",
                         help="ArUco dictionary name, e.g. DICT_4X4_50, DICT_5X5_100")
    args = parser.parse_args()

    camera = Camera(args.index, dict_name=args.dict)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    print(f"[INFO] Using dictionary {args.dict}")
    print(f"[INFO] Streaming at http://<pi-ip>:{args.port}/  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        server.server_close()
