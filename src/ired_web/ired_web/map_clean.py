"""Clean a saved map automatically (a new version; the original is not touched).

What it does, in this order:
1. Outside the field: the LiDAR sees through gaps and over low walls, so a SLAM map has
   rays and blobs far outside the 5 x 5 m field. The field is found with field_align
   (the same wall fit the Align button uses, with the measured field size); free cells more
   than `free_margin` outside the fitted wall lines become unknown, wall cells are kept out to
   `wall_reach` when their wall group comes within `margin` of the line (bent walls), and
   everything else outside becomes unknown. If the walls cannot be fitted, the largest free area
   (grown by `margin`) is kept instead.
2. Specks: small groups of occupied cells inside the field that do not touch the field
   walls (people walking by, noise) become free.
3. Holes: small unknown or occupied islands completely surrounded by free space inside
   the field become free.
Field walls and fences (large wall groups) are never changed.

Works on the trinary map_server image (0 occupied, 254 free, 205 unknown); pure
numpy/scipy so it runs on the PC too:
    python3 -m ired_web.map_clean IN.yaml OUT_NAME [--start c_z51|aligned]"""
import math
import os

import numpy as np
from scipy import ndimage

from ired_web import field_align

FREE, UNKNOWN, OCC = 254, 205, 0
EIGHT = np.ones((3, 3), bool)


def cell_xy(shape, res, origin):
    """Map-frame x, y of every cell centre (rows top-down like the image)."""
    h, w = shape
    c, r = np.meshgrid(np.arange(w), np.arange(h))
    return origin[0] + (c + 0.5) * res, origin[1] + (h - 1 - r + 0.5) * res


def field_mask(shape, res, origin, fit, margin):
    """Cells inside the field square grown by margin, from a field_align fit
    (field = R(yaw) map + t)."""
    x, y = cell_xy(shape, res, origin)
    c, s = math.cos(fit['yaw']), math.sin(fit['yaw'])
    fx = c * x - s * y + fit['t'][0]
    fy = s * x + c * y + fit['t'][1]
    size = field_align.FIELD
    return (fx > -margin) & (fx < size + margin) & (fy > -margin) & (fy < size + margin), fx, fy


def clean(img, res, origin, meta, hint=field_align.HINT_CARTOGRAPHER, margin=0.20, free_margin=0.05,
          wall_reach=0.50,
          speck_cells=6, hole_m2=0.25, wall_band=0.25):
    """Return (clean image, report dict). img: uint8 rows top-down."""
    occ, _ = field_align.classes(img, meta)
    # map_saver writes unknown as 205, which is under free_thresh 0.25 by value; treat only
    # near-white cells as free so unknown stays unknown
    free = (img >= 230) & ~occ if not meta.get('negate', 0) else (img <= 25) & ~occ
    out = np.full(img.shape, UNKNOWN, np.uint8)
    out[free] = FREE
    out[occ] = OCC
    report = {'cells': int(img.size)}

    # 1. outside the field
    try:
        fit = field_align.align(img, res, origin, meta, hint)
        _, fx, fy = field_mask(img.shape, res, origin, fit, margin)
        # distance (m) of each cell from the fitted wall lines (positive inside): the field
        # is centred on (2.5, 2.5) with the measured size, not exactly 5 x 5 m
        w = fit['walls_at']        # fitted wall lines in the field frame (START-fence anchored)
        edge = np.minimum.reduce([fx - w['x0'], w['x5'] - fx, fy - w['y0'], w['y5'] - fy])
        inside = edge > -wall_reach
        report['field'] = f'walls fitted ({fit["size"][0]:.2f} x {fit["size"][1]:.2f} m)'
    except ValueError as e:
        lab, n = ndimage.label(free, EIGHT)
        if n == 0:
            raise ValueError('no free space in the map')
        biggest = 1 + int(np.argmax(ndimage.sum(free, lab, range(1, n + 1))))
        inside = ndimage.binary_dilation(lab == biggest, EIGHT, iterations=max(1, int(wall_reach / res)))
        edge = ndimage.distance_transform_edt(inside) * res - wall_reach
        report['field'] = f'largest free area (walls not fitted: {e})'
    # free space cannot lie beyond the wall line: free cells more than free_margin outside
    # the field go too (wall cells are kept up to margin, the wall band is ~10-15 cm)
    # bent walls stick out of the fitted lines (up to ~10-15 cm seen): wall cells out to
    # wall_reach are kept when their wall group reaches within `margin` of the line
    occ_near = (out == OCC) & inside
    lab, n = ndimage.label(occ_near, EIGHT)
    keep_wall = np.zeros_like(occ_near)
    if n:
        reach = ndimage.maximum(edge, lab, range(1, n + 1))       # closest-to-field cell of each group
        ok = np.zeros(n + 1, bool)
        ok[1:] = np.asarray(reach) > -margin
        keep_wall = ok[lab] & occ_near
    beyond = ~inside | ((out == FREE) & (edge < -free_margin)) | ((out == OCC) & ~keep_wall)
    report['outside_cleared'] = int((beyond & (out != UNKNOWN)).sum())
    out[beyond] = UNKNOWN

    # 2. specks: small occupied groups well inside the field
    occ_now = out == OCC
    lab, n = ndimage.label(occ_now, EIGHT)
    specks = 0
    if n:
        sizes = ndimage.sum(occ_now, lab, range(1, n + 1))
        min_edge = ndimage.minimum(edge, lab, range(1, n + 1))
        for i, (sz, me) in enumerate(zip(sizes, min_edge), start=1):
            if sz < speck_cells and me > wall_band:
                out[lab == i] = FREE
                specks += int(sz)
    report['speck_cells_removed'] = specks

    # 3. holes: small non-free islands surrounded by free space, inside the field
    not_free = (out != FREE) & inside
    lab, n = ndimage.label(not_free, EIGHT)
    filled = 0
    max_cells = hole_m2 / (res * res)
    if n:
        sizes = ndimage.sum(not_free, lab, range(1, n + 1))
        for i, sz in enumerate(sizes, start=1):
            if sz > max_cells:
                continue
            m = lab == i
            ring = ndimage.binary_dilation(m, EIGHT) & ~m
            # only when every neighbour is free (not touching walls, unknown or the border)
            if ring.any() and (out[ring] == FREE).all() and edge[m].min() > wall_band:
                out[m] = FREE
                filled += int(sz)
    report['hole_cells_filled'] = filled
    return out, report


def clean_file(yaml_path, out_folder, out_name, hint=field_align.HINT_CARTOGRAPHER, **kw):
    img, res, origin, meta = field_align.load_map(yaml_path)
    if abs(origin[2]) > 1e-6:
        raise ValueError('map origin has a yaw; only yaw 0 maps are supported')
    out, report = clean(img, res, origin, meta, hint, **kw)
    field_align.save_map(out, res, origin, out_folder, out_name, meta)
    return report


if __name__ == '__main__':
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml')
    ap.add_argument('out_name')
    ap.add_argument('--start', choices=['c_z51', 'aligned'], default='c_z51')
    ap.add_argument('--out-dir')
    a = ap.parse_args()
    hint = field_align.HINT_CARTOGRAPHER if a.start == 'c_z51' else (0.0, 0.0, 0.0)
    print(json.dumps(clean_file(a.yaml, a.out_dir or os.path.dirname(a.yaml), a.out_name, hint), indent=1))
