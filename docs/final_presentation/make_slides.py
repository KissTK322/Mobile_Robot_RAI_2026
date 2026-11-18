"""Slide images (1920x1080, black like the Canva template) for the final presentation.
python3 make_slides.py  ->  slide_connect.png, slide_software.png, slide_evaluate.png"""
import os
from PIL import Image, ImageDraw, ImageFont

OUT = os.path.dirname(os.path.abspath(__file__))
W, H = 1920, 1080
BG, FG, MUTE, BOX, EDGE = (10, 10, 10), (255, 255, 255), (150, 150, 150), (34, 34, 34), (70, 58, 36)
POWER, DATA, WIFI = (242, 179, 61), (80, 200, 255), (160, 140, 255)


def font(size, bold=False):
    return ImageFont.truetype(f'/usr/share/fonts/truetype/noto/NotoSans-{"Bold" if bold else "Regular"}.ttf', size)


def slide(title, sub=None):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    d.text((90, 70), title, fill=FG, font=font(64, True))
    if sub:
        d.text((92, 158), sub, fill=MUTE, font=font(26))
    return im, d


def box(d, x, y, w, h, head, lines=(), accent=None):
    d.rounded_rectangle([x, y, x + w, y + h], 10, fill=BOX, outline=accent or EDGE, width=3 if accent else 2)
    d.text((x + w / 2, y + 30), head, fill=FG, font=font(28, True), anchor='mt')
    for i, l in enumerate(lines):
        d.text((x + w / 2, y + 74 + i * 34), l, fill=(205, 205, 205), font=font(23), anchor='mt')
    return (x, y, w, h)


def link(d, pts, col, label=None, at=None, width=5, anchor='mm'):
    d.line(pts, fill=col, width=width, joint='curve')
    if label:
        x, y = at or ((pts[0][0] + pts[-1][0]) / 2, (pts[0][1] + pts[-1][1]) / 2)
        f = font(21, True)
        l, t, r, b = d.textbbox((x, y), label, font=f, anchor=anchor)
        d.rectangle([l - 8, t - 5, r + 8, b + 5], fill=BG)
        d.text((x, y), label, fill=col, font=f, anchor=anchor)


def legend(d, y=1048):
    x = 90
    for col, name in ((POWER, 'Power'), (DATA, 'USB / serial data'), (WIFI, 'Wi-Fi')):
        d.line([(x, y), (x + 44, y)], fill=col, width=6)
        d.text((x + 56, y), name, fill=MUTE, font=font(22), anchor='lm')
        x += 300
    d.text((W - 90, y), 'Group 5 · AOW LOEY', fill=MUTE, font=font(22), anchor='rm')


def connect():
    im, d = slide('HOW DOES OUR ROBOT CONNECT?', 'Component connection diagram · checked on the robot (lsusb, udev, ired_bringup)')
    bat = box(d, 90, 240, 470, 150, 'Battery 24 V', ['2 × Makita 12 V 4.0 Ah', 'in series'], POWER)
    dc = box(d, 90, 520, 470, 150, 'DC-DC buck 24 V to 5 V', ['12–80 V in · 800 W 25 A', 'Pi 5 + LiDAR + camera (USB)'])
    motors = box(d, 90, 800, 470, 150, '4 × DC motor + encoder', ['omni wheels at ±45°, ±135°', 'Ø 130 mm'])
    pi = box(d, 740, 430, 460, 230, 'Raspberry Pi 5 (4 GB)', ['Ubuntu 24.04 · ROS 2 Jazzy', 'Nav2 · AMCL · EKF', 'web app :8080'], FG)
    ctl = box(d, 740, 800, 460, 150, 'iRED Controller', ['wheel PID (4 motors)', 'IMU (gyro / accel)'])
    lidar = box(d, 1380, 240, 450, 150, 'RPLIDAR A1', ['360° laser · ~7 Hz', '0.30 m above floor'])
    cam = box(d, 1380, 520, 450, 150, 'Logitech C270', ['USB webcam 640×480', 'ArUco 7 cm markers'])
    pc = box(d, 1380, 800, 450, 150, 'Team laptop', ['web control · refbox Bridge', 'MQTT to RefBox server'])
    # power
    link(d, [(325, 390), (325, 520)], POWER, '24 V', (325, 455))
    link(d, [(90, 315), (50, 315), (50, 990), (830, 990), (830, 950)], POWER, '24 V motor power', (440, 990))
    link(d, [(560, 595), (740, 595)], POWER, '5 V', (650, 595))
    link(d, [(740, 910), (560, 910)], POWER, 'motor drive', (650, 910))
    link(d, [(560, 850), (740, 850)], DATA, 'encoders', (650, 850))
    # data
    link(d, [(1200, 470), (1290, 470), (1290, 315), (1380, 315)], DATA, 'USB · CP2102 UART 115200', (1290, 410), anchor='mm')
    link(d, [(1200, 595), (1380, 595)], DATA, 'USB (UVC)', (1290, 595))
    link(d, [(970, 660), (970, 800)], DATA, 'USB · CP2102N · Modbus RTU 115200', (970, 730))
    link(d, [(1200, 630), (1290, 630), (1290, 875), (1380, 875)], WIFI, 'Wi-Fi', (1290, 760))
    legend(d)
    im.save(f'{OUT}/slide_connect.png')


def flow(d, y, items, col):
    x = 90
    w = (W - 180 - 30 * (len(items) - 1)) / len(items)
    for i, (head, lines) in enumerate(items):
        box(d, x, y, w, 170, head, lines)
        if i:
            link(d, [(x - 30, y + 85), (x, y + 85)], col, width=6)
            d.polygon([(x, y + 85), (x - 12, y + 77), (x - 12, y + 93)], fill=col)
        x += w + 30


def software():
    im, d = slide('HOW THE SOFTWARE USES THE HARDWARE', 'ROS 2 nodes on the Pi 5 · arrows = topics / messages')
    d.text((90, 225), 'DRIVING LOOP', fill=POWER, font=font(26, True))
    flow(d, 265, [('iRED board', ['encoders + IMU', 'wheel PID']),
                  ('modbus_serial', ['/ired/motor/data', '/ired/imu/raw']),
                  ('Odometry', ['forward kinematics', 'IMU + wheel odom']),
                  ('EKF + AMCL', ['/odometry/filtered', 'LiDAR + map = pose']),
                  ('Nav2', ['planner + DWB', 'collision monitor']),
                  ('Inverse kin.', ['/cmd_vel to 4 wheel', 'RPM over Modbus'])], POWER)
    d.text((90, 500), 'GAME LOOP', fill=DATA, font=font(26, True))
    flow(d, 540, [('RefBox server', ['zones / phase', 'scores reports']),
                  ('Bridge (our PC)', ['RefBox <-> MQTT', 'auto reset on SETUP']),
                  ('mqtt_refbox', ['robot pose 5 Hz', 'tasks to ROS']),
                  ('Challenge node', ['navigation: 12 zones', 'exploration: 2 boxes']),
                  ('Box finder', ['LiDAR clusters', 'C270 + ArUco']),
                  ('Nav2 goals', ['drive, dwell 5 s', 'look, report'])], DATA)
    d.text((90, 775), 'WEB APP (phone / laptop)', fill=WIFI, font=font(26, True))
    flow(d, 815, [('ired_web', ['Launch profiles', 'PIN, STOP button']),
                  ('rosbridge :9090', ['status, map, markers', 'small JSON']),
                  ('web_video_server', ['camera MJPEG :8081', '640×360, q60']),
                  ('Record + Replay', ['rosbag of each game', '3D replay page'])], WIFI)
    im.save(f'{OUT}/slide_software.png')


def table(d, x, y, cols, rows, widths, head_col=FG):
    f, fb = font(24), font(24, True)
    for j, c in enumerate(cols):
        d.text((x + sum(widths[:j]) + 16, y), c, fill=MUTE, font=font(22, True))
    y += 44
    for i, r in enumerate(rows):
        d.rectangle([x, y, x + sum(widths), y + 96], fill=BOX if i % 2 == 0 else (24, 24, 24))
        for j, c in enumerate(r):
            for k, line in enumerate(c.split('\n')):
                d.text((x + sum(widths[:j]) + 16, y + 14 + k * 34), line,
                       fill=(POWER if j == len(r) - 1 else FG) if k == 0 else (190, 190, 190), font=fb if (j == 0 or k == 0) else f)
        y += 100
    return y


def evaluate():
    im, d = slide('HOW DO WE EVALUATE OUR ROBOT?', 'Same metrics at every level, from bench test to race')
    steps = [('1  Bench tests', 'web Test tab: turn in place, drive straight,\nwheel PID step · LiDAR is the ruler'),
             ('2  PC simulation', 'fake RefBox + many random layouts\nbefore code goes on the robot'),
             ('3  Practice mode', 'local RefBox on the robot, layouts we\nchoose (corners, diagonal boxes)'),
             ('4  Real RefBox field test', 'seniors\' server, rosbag of every game,\nreplayed in 3D to find causes')]
    y = 230
    for head, body in steps:
        d.rounded_rectangle([90, y, 790, y + 150], 10, fill=BOX, outline=EDGE, width=2)
        d.text((120, y + 22), head, fill=FG, font=font(30, True))
        for k, l in enumerate(body.split('\n')):
            d.text((120, y + 70 + k * 32), l, fill=(200, 200, 200), font=font(23))
        y += 170
    d.text((880, 225), 'METRICS', fill=MUTE, font=font(22, True))
    mets = ['zones reached / 12  ·  stations correct (zone + rotation)', 'time to finish  ·  retries  ·  wrong reports (-2)',
            'localisation: AMCL jump < 3 cm  ·  ArUco distance error']
    for k, m in enumerate(mets):
        d.text((880, 262 + k * 38), '•  ' + m, fill=FG, font=font(24))
    d.text((880, 400), 'RESULTS', fill=MUTE, font=font(22, True))
    table(d, 880, 440, ['', 'Before tuning (28 Oct)', 'After (12–15 Nov)', 'Race (16 Nov)'],
          [['Navigation', '12/12 zones\n171 s', '12/12 zones\n132 s', '11/11 points\n12/12 zones · 124 s'],
           ['Exploration', '2/2 stations\n219–269 s (sweep)', '2/2 stations\n73–122 s · practice 11/11', '11/11 points\n2/2 correct · 57 s'],
           ['ArUco', '10.4 % too far\nat 0.5–1.5 m', 'fixed: marker size\ncalibrated (6.34 cm)', 'IDs 121, 122, 102\nboth boxes named']],
          [175, 245, 290, 240])
    d.text((880, 790), 'What changed the numbers', fill=FG, font=font(26, True))
    for k, l in enumerate(['LiDAR finds the boxes first, camera only confirms: Exploration 4× faster',
                           'Leave a zone right after 5.5 s, faster accel: Navigation -47 s',
                           'Bridge auto-reset on SETUP: no more -2 from old reports']):
        d.text((880, 835 + k * 38), '•  ' + l, fill=(205, 205, 205), font=font(23))
    im.save(f'{OUT}/slide_evaluate.png')


if __name__ == '__main__':
    connect(); software(); evaluate()
    print('ok')


def chart_png(fn, size):
    import io
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'Noto Sans', 'font.size': 15, 'text.color': 'white', 'axes.labelcolor': '#cccccc',
                         'xtick.color': '#aaaaaa', 'ytick.color': '#aaaaaa', 'axes.edgecolor': '#555555'})
    fig, ax = plt.subplots(figsize=(size[0] / 100, size[1] / 100), dpi=100, facecolor='#222222')
    ax.set_facecolor('#222222')
    for k in ('top', 'right'):
        ax.spines[k].set_visible(False)
    fn(ax)
    fig.tight_layout()
    buf = io.BytesIO(); fig.savefig(buf, format='png', facecolor='#222222'); plt.close(fig)
    buf.seek(0)
    return Image.open(buf)


def errors():
    import json
    e = json.load(open(f'{OUT}/race_eval.json'))
    n, x = e['navigation'], e['exploration']
    im, d = slide('HOW ACCURATE IS IT?', 'Error analysis from the race rosbags (16 Nov) · LiDAR is the reference')

    def odo(ax):
        for g, col, name in ((n, '#f2b33d', 'Navigation'), (x, '#50c8ff', 'Exploration')):
            s = g['odometry']['series_t_dist_errcm']
            ax.plot([p[1] for p in s], [p[2] for p in s], color=col, lw=3, label=name)
        ax.set_xlabel('distance driven (m)'); ax.set_ylabel('error of odometry alone (cm)')
        ax.legend(frameon=False); ax.grid(color='#333333')

    def zones(ax):
        z = n['zone_centre']['per_zone_cm']
        ax.bar([k.replace('C_', '') for k in z], list(z.values()), color='#f2b33d')
        ax.axhline(n['zone_centre']['mean_cm'], color='white', ls='--', lw=1.5)
        ax.set_ylim(0, 10); ax.text(len(z) - .5, 9.4, f"- - -  mean {n['zone_centre']['mean_cm']} cm", ha='right', color='white')
        ax.set_ylabel('distance to zone centre (cm)'); ax.tick_params(axis='x', labelsize=12); ax.grid(axis='y', color='#333333')

    d.text((90, 220), 'ODOMETRY ALONE DRIFTS (wheels + IMU, EKF)', fill=MUTE, font=font(22, True))
    im.paste(chart_png(odo, (860, 420)), (90, 255))
    d.text((990, 220), 'WHERE THE ROBOT STOPPED IN EACH 1 × 1 m ZONE (5 s stay)', fill=MUTE, font=font(22, True))
    im.paste(chart_png(zones, (840, 420)), (990, 255))
    o = n['odometry']
    tiles = [('Odometry alone', f"{o['odom_only_error_pct']} % of distance", f"{o['odom_only_end_error_cm']:.0f} cm after {o['distance_m']:.0f} m, heading {o['odom_yaw_error_deg']}°"),
             ('Heading (IMU)', f"{x['odometry']['odom_yaw_error_deg'] / x['odometry']['turned_deg'] * 100:.0f} % of turning",
              f"{x['odometry']['odom_yaw_error_deg']}° after {x['odometry']['turned_deg']}° turned"),
             ('AMCL correction', f"{o['jump_median_cm']} cm median", f"95 %: {o['jump_p95_cm']} cm · max {o['jump_max_cm']} cm"),
             ('Zone centre', f"{n['zone_centre']['mean_cm']} cm mean", f"max {n['zone_centre']['max_cm']} cm · AMCL pose"),
             ('Wheel speed PID', f"{n['wheels']['all_median_pct']} % median", 'error vs set-point at 10 Hz')]
    w = (W - 180 - 4 * 24) / 5
    for i, (h, big, small) in enumerate(tiles):
        x0 = 90 + i * (w + 24)
        d.rounded_rectangle([x0, 710, x0 + w, 900], 10, fill=BOX, outline=EDGE, width=2)
        d.text((x0 + 22, 728), h, fill=MUTE, font=font(22, True))
        d.text((x0 + 22, 768), big, fill=POWER, font=font(32, True))
        d.text((x0 + 22, 822), small, fill=(200, 200, 200), font=font(19))
    d.text((90, 935), 'Odometry alone would leave the robot ~1 m off after one game, mostly from heading drift. AMCL (LiDAR + map) removes it',
           fill=FG, font=font(25))
    d.text((90, 975), 'in small steps, so every stop lands within 9 cm of the zone centre.', fill=FG, font=font(25))
    im.save(f'{OUT}/slide_errors.png')


if __name__ == '__main__':
    errors()


def bench():
    im, d = slide('BENCH TESTS AND REPEATABILITY', 'Tests we ran on the robot (web Test tab, tape measure) and the same error numbers over all recorded games')
    import json
    games = json.load(open(f'{OUT}/games_eval.json'))
    rows = []
    for g in games:
        o = g['odometry']
        if o['distance_m'] < 5:
            continue
        b = g['bag'][4:12]
        rows.append([f"{b[6:8]}/{b[4:6]} " + {'navigation': 'nav', 'exploration': 'exp'}.get(g['mode'], ''), f"{o['distance_m']:.0f} m", f"{o['odom_only_error_pct']} %",
                     f"{abs(o['odom_yaw_error_deg']) / max(o['turned_deg'], 1) * 100:.1f} %", f"{o['jump_p95_cm']} cm", f"{g['wheels']['all_median_pct']} %"])
    d.text((90, 225), f'ALL RECORDED REAL GAMES ({len(rows)})', fill=MUTE, font=font(22, True))
    cols, ws = ['game', 'driven', 'odom alone', 'heading', 'AMCL step 95 %', 'wheel PID'], [150, 110, 150, 130, 190, 140]
    x0, y = 90, 262
    for j, c in enumerate(cols):
        d.text((x0 + sum(ws[:j]), y), c, fill=MUTE, font=font(20, True))
    y += 36
    for i, r in enumerate(rows):
        d.rectangle([x0 - 10, y - 4, x0 + sum(ws), y + 34], fill=BOX if i % 2 == 0 else BG)
        for j, c in enumerate(r):
            d.text((x0 + sum(ws[:j]), y), c, fill=FG if j else (200, 200, 200), font=font(21, j == 2))
        y += 40
    import statistics as st
    med = lambda k: st.median(float(r[k].split()[0]) for r in rows)
    d.text((x0, y + 10), f'median: odometry alone {med(2):.1f} % of distance · heading {med(3):.1f} % of turning · AMCL 95 % step {med(4):.1f} cm',
           fill=POWER, font=font(21, True))
    tests = [('Drive straight 1 m', 'wheels vs LiDAR: +3.1 % / -4.0 %', 'side drift 1.5–2.5 cm · yaw drift ~1.8°'),
             ('Wheel PID step (lifted)', 'kp 6 · ki 35 · kd 0.05 on all 4', 'overshoot up to 33 % · in game 11–14 % median'),
             ('Camera calibration', '72 chessboard images', 'reprojection 0.25 px (held-out 0.18 px)'),
             ('ArUco distance', '50/100/150 cm read 54/111/165 cm', 'fixed marker size: 100 cm reads 100.3 cm'),
             ('Localisation (tape)', 'robot put on C_Z33 by hand', 'AMCL 7.2 cm from the measured spot')]
    y = 225
    for h, a, b in tests:
        d.rounded_rectangle([1100, y, 1830, y + 140], 10, fill=BOX, outline=EDGE, width=2)
        d.text((1125, y + 18), h, fill=FG, font=font(26, True))
        d.text((1125, y + 62), a, fill=POWER, font=font(22, True))
        d.text((1125, y + 98), b, fill=(200, 200, 200), font=font(21))
        y += 160
    im.save(f'{OUT}/slide_bench.png')


if __name__ == '__main__':
    bench()


def performance():
    import json
    rows = [json.loads(l) for l in open(f'{OUT}/pi_load.jsonl')]
    names = {'idle': 'web app\nonly', 'robot': '+ Robot\n(drivers, EKF)', 'robot+nav': '+ Nav2\nAMCL', 'robot+nav+camera': '+ camera\nArUco',
             'full-stack': '+ challenge\n(LiDAR boxes)', 'stream-curl': '+ live\ncamera view'}
    rows = [r for r in rows if r['label'] in names]
    im, d = slide('HOW HARD DOES THE PI 5 WORK?', 'Measured on the robot: all game programs running, robot standing still, 3 extra fans')

    def chart(ax):
        x = range(len(rows))
        ax.bar(x, [r['cpu_total_pct'] for r in rows], color='#f2b33d', width=.6)
        for i, r in enumerate(rows):
            ax.text(i, r['cpu_total_pct'] + 1.5, f"{r['cpu_total_pct']:.0f} %", ha='center', color='white', fontsize=15, weight='bold')
        ax.set_xticks(list(x)); ax.set_xticklabels([names[r['label']] for r in rows], fontsize=12)
        ax.set_ylim(0, 100); ax.set_ylabel('CPU, all 4 cores (%)'); ax.grid(axis='y', color='#333333')
        t = ax.twinx()
        t.plot(list(x), [r['temp_c'] for r in rows], color='#50c8ff', lw=3, marker='o')
        t.set_ylim(20, 85); t.set_ylabel('temperature (°C)', color='#50c8ff'); t.tick_params(colors='#50c8ff')
        t.axhline(80, color='#e5484d', ls='--', lw=1.5); t.text(len(rows) - .5, 81, 'Pi 5 throttles at 80 °C', color='#e5484d', ha='right', fontsize=12)
        for k in ('top',):
            t.spines[k].set_visible(False)
    d.text((90, 220), 'CPU AND TEMPERATURE AS WE ADD EACH PART', fill=MUTE, font=font(22, True))
    im.paste(chart_png(chart, (930, 600)), (90, 255))
    full = next(r for r in rows if r['label'] == 'stream-curl')
    d.text((90, 880), f"Everything on: {full['cpu_total_pct']:.0f} % CPU, {full['temp_c']:.0f} °C, {full['mem_pct']:.0f} % of 4 GB RAM, "
                      f"no throttling ({full['throttled'].split('=')[1]}), 2.4 GHz", fill=POWER, font=font(24, True))
    d.text((90, 920), 'Biggest users: Nav2 + AMCL ~70 % of one core, ArUco ~35 %, LiDAR box finder ~25 %.', fill=(200, 200, 200), font=font(22))
    tips = [('Camera', ['C270 at 640×480 (calibrated size)', 'ArUco detection ~9 frames/s']),
            ('Live view to the web', ['MJPEG 640×360, JPEG q60: ~21 KB/frame', '~10 fps, ~1.7 Mbit/s, only while tab open']),
            ('LiDAR first', ['LiDAR finds boxes, camera only confirms', 'no camera sweep of the whole field']),
            ('Light recording', ['no images in rosbag: ~30 MB per game', 'replay converted in 3 s at low priority']),
            ('Cooling and power', ['3 fans blow hot air out: 42 °C max', 'USB current limit raised for the camera'])]
    y = 225
    for h, (a, b) in tips:
        d.rounded_rectangle([1080, y, 1830, y + 136], 10, fill=BOX, outline=EDGE, width=2)
        d.text((1105, y + 16), h, fill=FG, font=font(26, True))
        d.text((1105, y + 58), a, fill=(215, 215, 215), font=font(22))
        d.text((1105, y + 92), b, fill=(215, 215, 215), font=font(22))
        y += 152
    im.save(f'{OUT}/slide_performance.png')


if __name__ == '__main__':
    performance()


def aruco():
    im, d = slide('HOW THE CAMERA READS THE MARKERS', 'Live frame from the robot (Logitech C270, 640×480) with the ArUco overlay drawn by our node')
    cam = Image.open(f'{OUT}/aruco_camera_view.jpg').convert('RGB').resize((960, 720), Image.LANCZOS)
    d.rounded_rectangle([86, 226, 1050, 950], 8, outline=EDGE, width=4)
    im.paste(cam, (90, 230))
    d.text((90, 975), 'Marker 121 (BS output side) read at 1.27 m, 2° · marker 122 (BS input side) on a far box at 3.57 m', fill=POWER, font=font(24, True))
    steps = [('1  Detect', 'grey image, OpenCV ArUco', 'DICT_ARUCO_ORIGINAL, ~9 frames/s'),
             ('2  Measure', 'solvePnP with our calibration', 'fx 999.5 px, marker 6.34 cm'),
             ('3  Name the box', 'ID pair: BS 121/122, CS1 101/102', 'even ID = input side, odd = output'),
             ('4  Rotation', 'which face we see + its yaw', 'gives the 0/45/.../315° to report'),
             ('5  Confirm', 'LiDAR gives the box position', 'report only when views agree')]
    y = 226
    for h, a, b in steps:
        d.rounded_rectangle([1110, y, 1830, y + 132], 10, fill=BOX, outline=EDGE, width=2)
        d.text((1135, y + 16), h, fill=FG, font=font(26, True))
        d.text((1135, y + 56), a, fill=(215, 215, 215), font=font(22))
        d.text((1135, y + 90), b, fill=(170, 170, 170), font=font(21))
        y += 146
    im.save(f'{OUT}/slide_aruco.png')


if __name__ == '__main__':
    aruco()
