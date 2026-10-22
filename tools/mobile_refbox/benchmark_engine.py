#!/usr/bin/env python3
"""PC microbenchmark of game tick + pose + JSON status; no ROS or MQTT."""
import json
import math
from pathlib import Path
import statistics
import tempfile
import time
from ired_web.practice import PracticeGame

out = Path(__file__).resolve().parent / 'results' / 'engine-cpu.json'
clock = [0.0]
with tempfile.TemporaryDirectory(prefix='mobile-refbox-bench-') as folder:
    game = PracticeGame(folder, lambda topic, payload: None, clock=lambda: clock[0])
    config = game.prepare({'n': 4, 'seed': 11, 'limit_s': 1800, 'countdown_s': 3})
    game.on_pose(4.5, .5, math.pi)
    game.start(config['layout_id'], confirmed=True)
    samples = []
    for index in range(3000):
        clock[0] += .2
        start = time.process_time_ns()
        game.on_pose(4.5, .5, math.pi)
        game.tick()
        json.dumps(game.snapshot())
        samples.append((time.process_time_ns()-start)/1e6)
result = {'cycles': len(samples), 'cpu_ms_median': statistics.median(samples),
          'cpu_ms_p95': sorted(samples)[int(.95*len(samples))], 'cpu_ms_max': max(samples),
          'estimated_one_core_percent_at_5Hz': statistics.mean(samples)*5/10,
          'scope': 'PC game engine + pose + JSON status only; no ROS/MQTT/network or Pi measurement'}
out.write_text(json.dumps(result, indent=2))
print(json.dumps(result))
