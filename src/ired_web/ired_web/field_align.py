"""Align a saved map to the 5 x 5 m field: find the four field walls in the map and
turn the map so that its frame is the field frame (outer corner of C_Z11 at (0, 0),
x along C_Z11 -> C_Z51, y along C_Z11 -> C_Z15). Zone C_Zab is then centred at
(a - 0.5, b - 0.5), AMCL's start pose (4.5, 0.5, pi) is C_Z51, and the web/Nav2
need no other change.

How: the wall cells give the field direction (rotation that makes the occupied cells
line up in x and y), the rough transform (`hint`, e.g. Cartographer's start pose at
C_Z51 facing -x) picks which wall is which, each wall is then fitted with a straight
line, and the field is centred between opposite walls. The fit reports how long each
wall is seen, how far its cells stray from a straight line (bent or crooked walls)
and the measured field size, so a bad fit is visible before the map is used.

Pure numpy + PIL so it runs on the PC too:
    python3 -m ired_web.field_align IN.yaml OUT_NAME [--hint X Y YAW_DEG]"""
import math
import os

import numpy as np
import yaml
from PIL import Image

FIELD = 5.0
# Cartographer's map frame is where mapping started: the robot at C_Z51 facing -x
HINT_CARTOGRAPHER = (4.5, 0.5, math.pi)


def load_map(yaml_path):
    """-> (img uint8 rows top-down, resolution, (origin x, y, yaw), meta)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), meta['image'])).convert('L'))
    o = meta['origin']
    return img, float(meta['resolution']), (float(o[0]), float(o[1]), float(o[2]) if len(o) > 2 else 0.0), meta


def classes(img, meta):
    """occupied / free masks like map_server (trinary, negate 0)."""
    p = (255.0 - img) / 255.0
    if meta.get('negate', 0):
        p = img / 255.0
    return p > meta.get('occupied_thresh', 0.65), p < meta.get('free_thresh', 0.25)


def cell_centres(mask, res, origin):
    r, c = np.nonzero(mask)
    h = mask.shape[0]
    x = origin[0] + (c + 0.5) * res
    y = origin[1] + (h - 1 - r + 0.5) * res
    return np.stack([x, y], 1)


def rot(p, a):
    c, s = math.cos(a), math.sin(a)
    return np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]], 1)


def peakiness(p, a, bin_m=0.02):
    q = rot(p, a)
    s = 0.0
    for k in (0, 1):
        v = q[:, k]
        h, _ = np.histogram(v, bins=max(1, int((v.max() - v.min()) / bin_m) + 1))
        s += float((h.astype(np.float64) ** 2).sum())
    return s


def wall_direction(p):
    """Rotation in (-45, 45] deg that lines the occupied cells up with x and y."""
    best = max(np.radians(np.arange(-45.0, 45.0, 0.25)), key=lambda a: peakiness(p, a))
    fine = best + np.radians(np.arange(-0.3, 0.31, 0.02))
    return float(max(fine, key=lambda a: peakiness(p, a)))


def find_wall(v, w, guess, span, search=0.6, band=0.08):
    """Wall at v ~ guess: v = coordinate across the wall, w = along it.
    Only cells with w inside `span` count. Returns (position, cells used) or None."""
    m = (np.abs(v - guess) < search) & (w > span[0]) & (w < span[1])
    if m.sum() < 10:
        return None
    vv = v[m]
    h, e = np.histogram(vv, bins=int(2 * search / 0.02))
    k = int(np.argmax(h))
    peak = 0.5 * (e[k] + e[k + 1])
    sel = m & (np.abs(v - peak) < band)
    return peak, sel


def fit_line(v, w):
    """v = a + b w (least squares) -> (a, b, residuals)."""
    A = np.stack([np.ones_like(w), w], 1)
    (a, b), *_ = np.linalg.lstsq(A, v, rcond=None)
    return a, b, v - (a + b * w)


def align(img, res, origin, meta, hint=HINT_CARTOGRAPHER):
    """Find map -> field transform. Returns dict with yaw, t (field = R(yaw) map + t),
    per-wall report, measured size, or raises ValueError."""
    if abs(origin[2]) > 1e-6:
        raise ValueError('map origin has a yaw; only yaw 0 maps are supported')
    occ, _ = classes(img, meta)
    p = cell_centres(occ, res, origin)
    if len(p) < 200:
        raise ValueError('too few occupied cells to find the walls')
    th = wall_direction(p)
    # among th + k*90 deg take the one closest to the hint's rotation
    hx, hy, hyaw = hint
    cands = [th + k * math.pi / 2 for k in range(4)]
    yaw = min(cands, key=lambda a: abs(math.atan2(math.sin(a - hyaw), math.cos(a - hyaw))))

    report = {}
    for it in range(2):                       # second pass after the small angle correction
        q = rot(p, yaw) + np.array([hx, hy])  # roughly field coordinates
        x, y = q[:, 0], q[:, 1]
        span_x, span_y = (-0.3, FIELD + 0.3), (-0.3, FIELD + 0.3)
        walls = {
            'x0': find_wall(x, y, 0.0, span_y),
            'x5': find_wall(x, y, FIELD, span_y),
            'y0': find_wall(y, x, 0.0, span_x),
            'y5': find_wall(y, x, FIELD, span_x),
        }
        missing = [k for k, v in walls.items() if v is None]
        if missing:
            raise ValueError(f'wall(s) not found: {", ".join(missing)} (map not near the hint?)')
        fits, slopes = {}, []
        for k, (_, sel) in walls.items():
            v, w = (x, y) if k[0] == 'x' else (y, x)
            a, b, r = fit_line(v[sel], w[sel])
            fits[k] = (a, b, r, w[sel])
            slopes.append(b if k[0] == 'x' else -b)
        corr = math.atan(float(np.median(slopes)))   # remaining rotation of the walls
        if it == 0:
            yaw -= corr
    # wall positions at mid-field
    def at_mid(k):
        a, b, _, _ = fits[k]
        return a + b * FIELD / 2
    x0, x5, y0, y5 = at_mid('x0'), at_mid('x5'), at_mid('y0'), at_mid('y5')
    tx = hx + FIELD / 2 - (x0 + x5) / 2
    ty = hy + FIELD / 2 - (y0 + y5) / 2
    for k, (a, b, r, w) in fits.items():
        report[k] = {'cells': int(len(r)), 'seen_m': round(float(w.max() - w.min()), 2) if len(w) else 0.0,
                     'rms_cm': round(float(np.sqrt((r ** 2).mean())) * 100, 1),
                     'max_dev_cm': round(float(np.abs(r).max()) * 100, 1),
                     'angle_deg': round(math.degrees(math.atan(b)), 2)}
    names = {'x0': 'wall x=0 (C_Z11-C_Z15 side)', 'x5': 'wall x=5 (C_Z51-C_Z55 side)',
             'y0': 'wall y=0 (C_Z11-C_Z51 side)', 'y5': 'wall y=5 (C_Z15-C_Z55 side)'}
    return {'yaw': yaw, 't': (tx, ty), 'size': (round(x5 - x0, 3), round(y5 - y0, 3)),
            'walls': {names[k]: v for k, v in report.items()}}


def resample(img, res, origin, yaw, t, margin=0.5):
    """New image in the field frame (yaw 0), nearest neighbour, unknown outside."""
    h, w = img.shape
    corners = np.array([[origin[0], origin[1]], [origin[0] + w * res, origin[1]],
                        [origin[0], origin[1] + h * res], [origin[0] + w * res, origin[1] + h * res]])
    fc = rot(corners, yaw) + np.array(t)
    lo = np.floor((np.minimum(fc.min(0), [-margin, -margin])) / res) * res
    hi = np.ceil((np.maximum(fc.max(0), [FIELD + margin, FIELD + margin])) / res) * res
    W, H = int(round((hi[0] - lo[0]) / res)), int(round((hi[1] - lo[1]) / res))
    cc, rr = np.meshgrid(np.arange(W), np.arange(H))
    fx = lo[0] + (cc + 0.5) * res
    fy = lo[1] + (H - 1 - rr + 0.5) * res
    pm = rot(np.stack([fx.ravel() - t[0], fy.ravel() - t[1]], 1), -yaw)
    c = np.floor((pm[:, 0] - origin[0]) / res).astype(int)
    r = h - 1 - np.floor((pm[:, 1] - origin[1]) / res).astype(int)
    ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
    out = np.full(H * W, 205, np.uint8)
    out[ok] = img[r[ok], c[ok]]
    return out.reshape(H, W), (float(lo[0]), float(lo[1]), 0.0)


def save_map(img, res, origin, folder, name, meta):
    Image.fromarray(img, mode='L').save(os.path.join(folder, name + '.pgm'))
    out = {'image': name + '.pgm', 'mode': meta.get('mode', 'trinary'), 'resolution': res,
           'origin': [round(origin[0], 4), round(origin[1], 4), 0.0], 'negate': meta.get('negate', 0),
           'occupied_thresh': meta.get('occupied_thresh', 0.65), 'free_thresh': meta.get('free_thresh', 0.25)}
    with open(os.path.join(folder, name + '.yaml'), 'w') as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False)


def align_file(yaml_path, out_folder, out_name, hint=HINT_CARTOGRAPHER):
    img, res, origin, meta = load_map(yaml_path)
    fit = align(img, res, origin, meta, hint)
    new, new_origin = resample(img, res, origin, fit['yaw'], fit['t'])
    save_map(new, res, new_origin, out_folder, out_name, meta)
    fit['yaw_deg'] = round(math.degrees(fit['yaw']), 2)
    fit['t'] = [round(v, 3) for v in fit['t']]
    del fit['yaw']
    return fit


if __name__ == '__main__':
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml')
    ap.add_argument('out_name')
    ap.add_argument('--hint', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--out-dir')
    a = ap.parse_args()
    hint = (a.hint[0], a.hint[1], math.radians(a.hint[2])) if a.hint else HINT_CARTOGRAPHER
    print(json.dumps(align_file(a.yaml, a.out_dir or os.path.dirname(a.yaml), a.out_name, hint), indent=1))
