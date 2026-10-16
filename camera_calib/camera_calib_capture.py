import argparse
import os
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler
from urllib.parse import urlparse

import cv2

camera = None
CHESSBOARD = (9, 6)  # internal corners (cols, rows)
OUT_DIR = "calib_images"


class Camera:
    def __init__(self, index=0, width=1280, height=720):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.lock = threading.Lock()
        self.raw_frame = None     # unmodified, for saving
        self.display_frame = None  # with overlay, for streaming
        self.corners_found = False
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

        os.makedirs(OUT_DIR, exist_ok=True)
        self.count = len([f for f in os.listdir(OUT_DIR) if f.endswith(".jpg")])

    def _update(self):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, CHESSBOARD, flags=flags)

            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, CHESSBOARD, corners, found)
            status = "FOUND - good to capture" if found else "not detected"
            color = (0, 255, 0) if found else (0, 0, 255)
            cv2.putText(disp, f"Chessboard: {status}  |  captured: {self.count}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            with self.lock:
                self.raw_frame = frame
                self.display_frame = disp
                self.corners_found = found

    def get_jpeg(self):
        with self.lock:
            if self.display_frame is None:
                return None
            frame = self.display_frame.copy()
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        return buf.tobytes() if ok else None

    def capture(self):
        with self.lock:
            if self.raw_frame is None:
                return None, self.count
            frame = self.raw_frame.copy()
            self.count += 1
            n = self.count
        path = os.path.join(OUT_DIR, f"img_{n:03d}.jpg")
        cv2.imwrite(path, frame)
        return path, n

    def stop(self):
        self.running = False
        self.thread.join(timeout=1)
        self.cap.release()


PAGE = """<html><body style='background:#111;color:white;font-family:sans-serif;text-align:center'>
<h3>Camera Calibration Capture</h3>
<img src='/stream.mjpg' style='max-width:95%%'/><br/><br/>
<button onclick="capture()" style='font-size:1.3em;padding:10px 30px'>Capture</button>
<p id='status'>captured: %d</p>
<script>
function capture(){
  fetch('/capture').then(r=>r.text()).then(t=>{
    document.getElementById('status').innerText = t;
  });
}
</script>
</body></html>"""


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write((PAGE % camera.count).encode())
        elif path == "/capture":
            filepath, n = camera.capture()
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            msg = f"captured: {n} -> {filepath}" if filepath else "no frame yet"
            self.wfile.write(msg.encode())
            print(f"[CAPTURE] {msg}")
        elif path == "/stream.mjpg":
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
    parser = argparse.ArgumentParser(description="Chessboard capture for camera calibration")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    camera = Camera(args.index)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    print(f"[INFO] {camera.count} image(s) already in ./{OUT_DIR}/")
    print(f"[INFO] Open http://<pi-ip>:{args.port}/  and click Capture ~15-20 times")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        server.server_close()
