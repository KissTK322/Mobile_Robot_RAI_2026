"""Top-down replay videos of the race games (race.json from race_data.py) -> docs/final_presentation/*.mp4
python3 race_video.py"""
import bisect, json, math, re
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = '/home/ired/ired_work/docs/final_presentation/'
W, H, FPS, SPEED = 1280, 720, 20, 2.0
F = lambda s, b=False: ImageFont.truetype(f'/usr/share/fonts/truetype/noto/NotoSans-{"Bold" if b else "Regular"}.ttf', s)
BG, FG, GRID, MUTE, ACC, RED, LID, BOX = (10, 10, 10), (255, 255, 255), (58, 58, 58), (140, 140, 140), (242, 179, 61), (229, 72, 77), (80, 200, 255), (154, 160, 166)
S, OX, OY = 128, 40, 680      # px per metre, field origin (bottom-left) in the frame


def px(x, y):
    return OX + x * S, OY - y * S


def english(text):
    rules = [(r'เริ่มเกม.*', 'Game start: leave START via the red zone C_Z32'),
             (r'ถึงช่อง (\S+) \((\d+)/12\)', r'Reached \1  (\2/12)'),
             (r'ครบ 12 ช่อง ใน (\d+) วินาที ไม่มี retry', r'All 12 zones in \1 s, no retries'),
             (r'LiDAR เห็นกล่องใน (\S+) กล้องอ่านได้ว่าเป็น (\S+)', r'LiDAR finds a box in \1 (\2)'),
             (r'หยุดหันกล้องดู marker', 'Stop and point the camera at the marker'),
             (r'รายงาน (\S+): (\S+) มุม (\d+)', r'Report \1: \2, \3°'),
             (r'รายงานครบ 2 กล่อง ใน (\d+) วินาที', r'Both stations reported in \1 s')]
    for p, r in rules:
        if re.fullmatch(p, text):
            return re.sub(p, r, text)
    return text


def rect(d, x, y, rot, w, h, fill, outline):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    pts = [px(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2))]
    d.polygon(pts, fill=fill, outline=outline, width=2)


def centre(z):
    return int(z[-2]) - .5, int(z[-1]) - .5


def background(g, nav):
    im = Image.new('RGB', (W, H), BG); d = ImageDraw.Draw(im)
    d.rectangle([*px(2, 1), *px(5, 0)], fill=(29, 29, 29))
    d.rectangle([*px(2, 2), *px(3, 1)], fill=(64, 26, 28))
    for k in range(6):
        d.line([px(k, 0), px(k, 5)], fill=GRID, width=1); d.line([px(0, k), px(5, k)], fill=GRID, width=1)
    for i in range(1, 6):
        for j in range(1, 6):
            d.text(px(i - .95, j - .05), f'Z{i}{j}', fill=MUTE, font=F(13))
    d.line([px(2, 0), px(2, 1), px(5, 1)], fill=FG, width=3)
    d.text(px(3.5, .15), 'START', fill=FG, font=F(18, True), anchor='mm')
    for b in g['boxes'] if nav else []:      # exploration boxes appear when the robot finds them
        x, y = centre(b['zone'])
        rect(d, x, y, b['rot'], .7, .5, BOX if nav else ACC, FG)
        d.text(px(x, y), b['name'].replace('C-', ''), fill=BG, font=F(15, True), anchor='mm')
    X = 730
    d.text((X, 40), 'GROUP 5 · AOW LOEY', fill=MUTE, font=F(18, True))
    d.text((X, 66), g['title'].replace('แข่งจริง: ', '').upper() + ' · RACE', fill=FG, font=F(40, True))
    d.text((X, 120), '16 Nov 2026 · replay from the robot rosbag · 2× speed', fill=MUTE, font=F(17))
    return im


def render(g, name, nav):
    bg = background(g, nav)
    tt = [p[0] for p in g['track']]; lt = [l[0] for l in g['lidar']]; st = [s[0] for s in g['seen']]
    end = g['end'] + 3
    vw = cv2.VideoWriter(OUT + name, cv2.VideoWriter_fourcc(*'avc1'), FPS, (W, H))
    t, X = -2.0, 730
    while t <= end:
        im = bg.copy(); d = ImageDraw.Draw(im)
        i = max(0, bisect.bisect_right(lt, t) - 1)
        if g['lidar'] and abs(lt[i] - t) < 1.0:
            p = np.array(g['lidar'][i][1]).reshape(-1, 2) / 100
            for x, y in p:
                a, b = px(x, y); d.ellipse([a - 2, b - 2, a + 2, b + 2], fill=LID)
        evs = [e for e in g['ev'] if e[0] <= t]
        if nav:
            n = 0
            for e in evs:
                m = re.match(r'ถึงช่อง (\S+) \((\d+)/12\)', e[2])
                if m:
                    n += 1; x, y = centre(m[1]); a, b = px(x, y)
                    d.ellipse([a - 17, b - 17, a + 17, b + 17], fill=ACC); d.text((a, b), str(n), fill=BG, font=F(16, True), anchor='mm')
        if not nav:
            for b in g['boxes']:
                found = any(b['name'] in e[2] and e[1] == 'lidar' for e in evs)
                done = any(e[2].startswith('รายงาน ' + b['name'] + ':') for e in evs)
                if found or done:
                    x, y = centre(b['zone'])
                    rect(d, x, y, b['rot'], .7, .5, ACC if done else None, FG if done else ACC)
                    d.text(px(x, y), b['name'].replace('C-', ''), fill=BG if done else ACC, font=F(15, True), anchor='mm')
        k = bisect.bisect_right(tt, t)
        trail = [px(p[1], p[2]) for p in g['track'][:max(k, 1)]]
        if len(trail) > 1:
            d.line(trail, fill=ACC, width=4, joint='curve')
        _, x, y, yaw = g['track'][max(0, k - 1)]
        a, b = px(x, y); r = .2 * S
        d.ellipse([a - r, b - r, a + r, b + r], fill=(40, 40, 40), outline=FG, width=3)
        d.line([(a, b), (a + math.cos(yaw) * r * 1.4, b - math.sin(yaw) * r * 1.4)], fill=FG, width=4)
        j = bisect.bisect_right(st, t) - 1
        if j >= 0 and t - st[j] < .6:
            for s in (-.45, .45):
                d.line([(a, b), (a + math.cos(yaw + s) * 150, b - math.sin(yaw + s) * 150)], fill=(255, 120, 80), width=2)
            d.text((X, 600), 'Camera sees marker ' + ', '.join(map(str, g['seen'][j][1])), fill=(255, 120, 80), font=F(20, True))
        gt = max(0.0, min(t, g['end']))
        d.text((X, 170), f'{gt:6.1f} s', fill=FG, font=F(64, True))
        if nav:
            score = f'{sum(1 for e in evs if e[2].startswith("ถึงช่อง"))}/12 zones'
        else:
            score = f'{sum(1 for e in evs if e[2].startswith("รายงาน C"))}/2 stations reported'
        d.text((X, 255), score, fill=ACC, font=F(30, True))
        for n, e in enumerate(evs[-7:]):
            col = FG if n == len(evs[-7:]) - 1 else MUTE
            d.text((X, 315 + n * 38), f'{e[0]:5.1f}s  {english(e[2])}', fill=col, font=F(19, n == len(evs[-7:]) - 1))
        if t >= g['end']:
            d.text((X, 650), 'DONE · ' + g['result'].replace('ช่อง', 'zones').replace('วินาที', 's').replace('รายงาน', 'reported').replace('กล่อง', 'stations'), fill=ACC, font=F(24, True))
        vw.write(cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR))
        t += SPEED / FPS
    for _ in range(FPS * 2):
        vw.write(cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR))
    vw.release()
    print(name, 'done')


games = {g['id']: g for g in json.load(open('race.json'))}
render(games['race_nav'], 'race_navigation_replay.mp4', True)
render(games['race_exp'], 'race_exploration_replay.mp4', False)
