import argparse
import glob
import os

import cv2
import numpy as np

CHESSBOARD = (9, 6)  # internal corners (cols, rows) - must match capture script


def main():
    parser = argparse.ArgumentParser(description="Compute camera calibration from chessboard images")
    parser.add_argument("--dir", type=str, default="calib_images")
    parser.add_argument("--square-size", type=float, default=25.0,
                         help="REAL measured square size in mm (ruler, not nominal)")
    args = parser.parse_args()

    images = sorted(glob.glob(os.path.join(args.dir, "*.jpg")))
    if len(images) < 8:
        print(f"[WARN] Only found {len(images)} images in {args.dir}/ - "
              f"calibration needs at least ~10-15 good ones for a reliable result.")
    if not images:
        print(f"[FAIL] No .jpg images found in {args.dir}/. "
              f"Run camera_calib_capture.py first.")
        return

    # 3D object points for one chessboard, in the board's own coordinate
    # frame (z=0 plane), scaled by the real square size.
    objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []  # 3D points, real world
    imgpoints = []  # 2D points, image plane
    used = 0
    image_size = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"[SKIP] Could not read {fname}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]  # (width, height)

        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD)
        if not found:
            print(f"[SKIP] Chessboard not found in {fname}")
            continue

        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        used += 1
        print(f"[OK]   {fname}")

    if used < 5:
        print(f"[FAIL] Only {used} usable images - need more. Capture more with varied "
              f"angles/distances and try again.")
        return

    print(f"[INFO] Calibrating with {used} images...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    print(f"[RESULT] Reprojection error: {ret:.4f} px "
          f"({'good' if ret < 0.5 else 'usable' if ret < 1.0 else 'HIGH - recapture recommended'})")
    print(f"[RESULT] Camera matrix:\n{camera_matrix}")
    print(f"[RESULT] Distortion coefficients:\n{dist_coeffs}")

    # Save as OpenCV YAML (FileStorage format - loadable from C++ or Python)
    fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("reprojection_error", ret)
    fs.release()
    print("[SAVED] camera_calibration.yaml")

    np.savez("camera_calibration.npz",
              camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
              image_width=image_size[0], image_height=image_size[1],
              reprojection_error=ret)
    print("[SAVED] camera_calibration.npz")


if __name__ == "__main__":
    main()
