"""Map page of the web app: live SLAM map, robot pose, save / list / choose maps.

Only active while the Map page is open (subscriptions are made and dropped by
MapWatch.watch). /tf and /odometry/filtered are taken raw and unpacked at most
10 times a second, so a fast TF stream (Cartographer publishes map->odom at up
to 200 Hz) costs almost nothing in Python.

Maps are written here in map_saver's format (trinary PGM + YAML) straight from
the /map message the page already has, so saving does not depend on matching
Cartographer's /map QoS. Files go to the source maps folder (kept, versioned)
and are linked into the installed share folder, where navigation.launch.xml
looks for them, so a new map can be used without rebuilding."""
import io
import math
import os
import re
import threading
import time

import numpy as np
import yaml
from PIL import Image, ImageDraw
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

from ired_web import field_align

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_to_pgm_array(msg):
    """OccupancyGrid -> uint8 image rows top-down, map_saver trinary values:
    254 free (< 25 %), 0 occupied (> 65 %), 205 unknown or in between."""
    a = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    img = np.full(a.shape, 205, dtype=np.uint8)
    img[(a >= 0) & (a < 25)] = 254
    img[a > 65] = 0
    return img[::-1]                      # ROS row 0 is the bottom of the map


def grid_to_png(msg):
    """Same picture as the saved map, PNG bytes for the browser."""
    buf = io.BytesIO()
    Image.fromarray(grid_to_pgm_array(msg), mode='L').save(buf, format='PNG', optimize=False)
    return buf.getvalue()


def write_map(msg, folder, name):
    """Write <name>.pgm and <name>.yaml like `map_saver_cli -f folder/name`."""
    os.makedirs(folder, exist_ok=True)
    pgm = os.path.join(folder, name + '.pgm')
    Image.fromarray(grid_to_pgm_array(msg), mode='L').save(pgm)
    o = msg.info.origin
    meta = {
        'image': name + '.pgm',
        'mode': 'trinary',
        'resolution': float(msg.info.resolution),
        'origin': [float(o.position.x), float(o.position.y), float(yaw_of(o.orientation))],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(os.path.join(folder, name + '.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    return pgm


class MapWatch:
    """Latest /map and the robot pose in the map frame, while someone looks."""

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.subs = []
        self.map = None
        self.version = 0
        self.png = None
        self.png_version = -1
        self.map_odom = None              # (x, y, yaw) of odom in map
        self.odom_base = None             # (x, y, yaw) of the robot in odom
        self.pose_time = 0.0
        self.t_tf = 0.0
        self.t_odom = 0.0

    def watch(self, on):
        if on and not self.subs:
            n = self.node
            self.subs = [
                # Cartographer publishes /map volatile every second, Nav2's map_server once,
                # latched (transient local): one subscription for each kind
                n.create_subscription(OccupancyGrid, '/map', self.on_map,
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)),
                n.create_subscription(OccupancyGrid, '/map', self.on_map,
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                n.create_subscription(TFMessage, '/tf', self.on_tf, 100, raw=True),
                n.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10, raw=True),
            ]
        elif not on and self.subs:
            for s in self.subs:
                self.node.destroy_subscription(s)
            self.subs = []
            with self.lock:
                self.png = None
                self.png_version = -1

    def on_map(self, msg):
        with self.lock:
            if self.map is not None and msg.header.stamp == self.map.header.stamp and msg.info == self.map.info:
                return                    # the same map from the second subscription
            self.map = msg
            self.version += 1

    def on_tf(self, raw):
        now = time.time()
        if now - self.t_tf < 0.1:
            return
        msg = deserialize_message(raw, TFMessage)
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                tr = t.transform
                self.map_odom = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                self.t_tf = now
                self.pose_time = now

    def on_odom(self, raw):
        now = time.time()
        if now - self.t_odom < 0.1:
            return
        self.t_odom = now
        m = deserialize_message(raw, Odometry)
        p = m.pose.pose
        self.odom_base = (p.position.x, p.position.y, yaw_of(p.orientation))

    def pose(self):
        """Robot (x, y, yaw) in the map frame, or None if not known recently."""
        if self.map_odom is None or self.odom_base is None or time.time() - self.pose_time > 5.0:
            return None
        mx, my, myaw = self.map_odom
        ox, oy, oyaw = self.odom_base
        c, s = math.cos(myaw), math.sin(myaw)
        return (mx + c * ox - s * oy, my + s * ox + c * oy, math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw)))

    def info(self):
        with self.lock:
            m = self.map
        if m is None:
            return {'have': False, 'pose': self.pose()}
        o = m.info.origin.position
        return {'have': True, 'version': self.version, 'width': m.info.width, 'height': m.info.height,
                'resolution': m.info.resolution, 'origin': [o.x, o.y], 'pose': self.pose()}

    def png_bytes(self):
        with self.lock:
            m, v = self.map, self.version
            if m is None:
                return None
            if v == self.png_version and self.png:
                return self.png
        png = grid_to_png(m)              # outside the lock: about 20 ms for a 6 x 6 m map
        with self.lock:
            self.png, self.png_version = png, v
        return png


class MapStore:
    """Saved maps on disk and the one Navigation uses."""

    def __init__(self, src_dir, share_dir, state_dir):
        self.src_dir = os.path.expanduser(src_dir)
        self.share_dir = share_dir
        self.active_path = os.path.join(state_dir, 'active_map.txt')

    def active(self):
        try:
            with open(self.active_path) as f:
                name = f.read().strip()
            if name:
                return name
        except OSError:
            pass
        return 'map'

    def list(self):
        out = []
        try:
            files = sorted(os.listdir(self.src_dir))
        except OSError:
            files = []
        for fn in files:
            if fn.endswith('.yaml'):
                name = fn[:-5]
                out.append({'name': name, 'time': os.path.getmtime(os.path.join(self.src_dir, fn)),
                            'active': name == self.active()})
        out.sort(key=lambda m: m['time'], reverse=True)
        return out

    def link_into_share(self, name):
        """navigation.launch.xml loads $(find-pkg-share ired_navigation)/maps/<name>.yaml;
        with --symlink-install that folder holds links made at build time only."""
        if not self.share_dir or os.path.realpath(self.share_dir) == os.path.realpath(self.src_dir):
            return
        os.makedirs(self.share_dir, exist_ok=True)
        for ext in ('.yaml', '.pgm'):
            dst = os.path.join(self.share_dir, name + ext)
            if os.path.lexists(dst):
                os.remove(dst)
            os.symlink(os.path.join(self.src_dir, name + ext), dst)

    def save(self, msg, name):
        if not NAME_RE.match(name or ''):
            raise ValueError('Name: letters, digits, - and _ only (max 40)')
        if msg is None:
            raise ValueError('No map yet: start Mapping and drive a little first')
        if os.path.exists(os.path.join(self.src_dir, name + '.yaml')):
            raise ValueError(f'A map called {name} already exists')
        write_map(msg, self.src_dir, name)
        self.link_into_share(name)
        return {'name': name, 'width_m': round(msg.info.width * msg.info.resolution, 2),
                'height_m': round(msg.info.height * msg.info.resolution, 2)}

    def unique_name(self, base):
        name, k = base[:40], 2
        while os.path.exists(os.path.join(self.src_dir, name + '.yaml')):
            name = f'{base[:36]}-{k}'
            k += 1
        return name

    def align(self, name, start='auto'):
        """Write <name>-aligned: the map turned so its frame is the field frame (field_align).
        start 'c_z51': the map was made starting at C_Z51 facing -x (raw Cartographer map),
        'aligned': it is already roughly field-aligned, 'auto': 'aligned' if the name says
        field, else 'c_z51'. The original is not touched."""
        if not any(m['name'] == name for m in self.list()):
            raise ValueError(f'No map called {name}')
        hints = {'c_z51': field_align.HINT_CARTOGRAPHER, 'aligned': (0.0, 0.0, 0.0)}
        if start == 'auto':                   # guess from the name, the other one if that fails
            first = 'aligned' if re.search(r'field|aligned', name, re.I) else 'c_z51'
            order = [first, 'c_z51' if first == 'aligned' else 'aligned']
        elif start in hints:
            order = [start]
        else:
            raise ValueError('start must be auto, c_z51 or aligned')
        base = re.sub(r'-(field|aligned)(-\d+)?$', '', name, flags=re.I) + '-aligned'
        out = self.unique_name(base)
        for i, start in enumerate(order):
            try:
                fit = field_align.align_file(os.path.join(self.src_dir, name + '.yaml'), self.src_dir, out,
                                             hints[start])
                break
            except ValueError:
                if i == len(order) - 1:
                    raise
        bad = [k for k, w in fit['walls'].items() if w['seen_m'] < 2.5]
        if bad or not all(4.7 < v < 5.4 for v in fit['size']):
            for ext in ('.yaml', '.pgm'):
                os.remove(os.path.join(self.src_dir, out + ext))
            raise ValueError(f'Field walls not found cleanly (size {fit["size"]}, short walls: {bad}). '
                             'Map the whole field, or try the other start option.')
        self.link_into_share(out)
        fit.update({'name': out, 'from': name, 'start': start})
        return fit

    def preview_png(self, name, scale=3):
        """Saved map as PNG with the 5 x 5 m zone grid drawn where the map frame puts it."""
        if not any(m['name'] == name for m in self.list()):
            raise ValueError(f'No map called {name}')
        img, res, origin, _ = field_align.load_map(os.path.join(self.src_dir, name + '.yaml'))
        h, w = img.shape
        im = Image.fromarray(img, mode='L').convert('RGB').resize((w * scale, h * scale), Image.NEAREST)
        d = ImageDraw.Draw(im)

        def px(x, y):
            return ((x - origin[0]) / res * scale, (h - (y - origin[1]) / res) * scale)
        for k in range(6):
            col, wd = ((8, 126, 156), 2) if k in (0, 5) else ((8, 126, 156), 1)
            d.line([px(k, 0), px(k, 5)], fill=col, width=wd)
            d.line([px(0, k), px(5, k)], fill=col, width=wd)
        for a in range(1, 6):
            for b in range(1, 6):
                x, y = px(a - 0.5, b - 0.5)
                d.line([(x - 4, y), (x + 4, y)], fill=(198, 40, 40))
                d.line([(x, y - 4), (x, y + 4)], fill=(198, 40, 40))
        # show the field with 1 m around it, not the whole SLAM area
        (x0, y1), (x1, y0) = px(-1, 6), px(6, -1)
        box = (max(0, int(x0)), max(0, int(y1)), min(im.width, int(x1)), min(im.height, int(y0)))
        if box[2] - box[0] > 20 and box[3] - box[1] > 20:
            im = im.crop(box)
        buf = io.BytesIO()
        im.save(buf, format='PNG')
        return buf.getvalue()

    def select(self, name):
        if not any(m['name'] == name for m in self.list()):
            raise ValueError(f'No map called {name}')
        self.link_into_share(name)
        os.makedirs(os.path.dirname(self.active_path), exist_ok=True)
        with open(self.active_path, 'w') as f:
            f.write(name + '\n')
