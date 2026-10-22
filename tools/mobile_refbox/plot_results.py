#!/usr/bin/env python3
"""Plot recorded PC qualification results; never launches ROS or MQTT."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent / 'results'
fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), layout='constrained')
games = [json.loads((root/f'qualified-{n}'/'result.json').read_text()) for n in (2, 3, 4)]
axes[0].bar(['2 boxes', '3 boxes', '4 boxes'], [d['game']['elapsed_s'] for d in games], color='#38abc8')
for i, data in enumerate(games):
    axes[0].text(i, data['game']['elapsed_s']+3, f"{data['game']['elapsed_s']:.1f}s\n12/12; contacts {data['contacts']}", ha='center')
axes[0].set(title='PC simulation: seed 11, soft-stop 0.18', ylabel='Game elapsed time (s)', ylim=(0, 230))
for name, label in [('stop-final','Phone Stop'), ('mqtt-final','MQTT disconnect'), ('lease-final','Web worker freeze')]:
    data = json.loads((root/name/'result.json').read_text())
    trace = json.loads((root/name/'commands.json').read_text())
    points = [(t-data['fault_at'], v) for t,v in trace if -1 <= t-data['fault_at'] <= 5]
    x,y = zip(*points)
    # collision_monitor stops publishing after its zero tail: display that held zero.
    axes[1].step(list(x)+[5], list(y)+[y[-1]], where='post', label=f"{label}: zero at {data['first_zero_s']:.2f}s")
axes[1].axvline(0, color='black', linestyle='--', linewidth=1)
axes[1].set(title='Final command speed after simulated stop/fault', xlabel='Seconds relative to trigger', ylabel='Command speed (m/s)', xlim=(-1, 5))
axes[1].legend(fontsize=8)
fig.suptitle('Mobile refbox qualification — PC only; hardware untested')
fig.savefig(root/'qualification.png', dpi=160)
