"""race_eval numbers for every real game bag that has a finished challenge -> games_eval.json
source /opt/ros/jazzy/setup.bash; source ~/ired_ws/install/setup.bash; python3 games_eval.py"""
import glob, json, os
from race_eval import BAGS, evaluate

out = []
for path in sorted(glob.glob(BAGS + 'nav-2026*')):
    bag = os.path.basename(path)
    for mode in ('navigation', 'exploration', None):
        try:
            g = evaluate(mode or 'navigation', bag, strict=mode is not None)
        except Exception as e:
            continue
        mode = mode or 'recording'
        g['mode'] = mode
        out.append(g)
        o = g['odometry']
        print(bag, mode, g['game_s'], 's', o['distance_m'], 'm', 'odom', o['odom_only_error_pct'], '%', 'yaw', o['odom_yaw_error_deg'], '/', o['turned_deg'],
              'jump95', o['jump_p95_cm'], 'zone', g.get('zone_centre', {}).get('mean_cm'), 'wheel', g['wheels']['all_median_pct'], flush=True)
        break
json.dump(out, open('games_eval.json', 'w'), indent=1)
