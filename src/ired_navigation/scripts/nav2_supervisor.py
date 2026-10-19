#!/usr/bin/env python3
"""Start Nav2 and start it again if it does not come up.

About 1 in 20 starts (PC sim) Nav2 never becomes active: a reply inside the component
container is lost and a server stays unconfigured. Asking the lifecycle managers again
does not always help, a fresh launch does. This runs

    ros2 launch ired_navigation navigation.launch.xml <arguments given here>

in its own process group and asks the servers for their lifecycle state (2 s reply
timeout, so a lost reply cannot hang it). If they are not all active within
--ready-timeout seconds (default 60) the whole group is stopped and launched again, at
most --restarts times (default 2). Once active it only waits; Ctrl-C / SIGTERM stops
Nav2 too.

usage: ros2 run ired_navigation nav2_supervisor.py [--ready-timeout S] [--restarts N] [launch args...]
       e.g. ros2 run ired_navigation nav2_supervisor.py map_name:=field.yaml"""
import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState

SERVERS = ('map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator')


def log(msg):
    print(f'[nav2_supervisor] {msg}', flush=True)


def all_active(node, clients):
    """Names of the servers that are not active (a missing reply counts as not active)."""
    waiting = []
    for name, cli in clients.items():
        if not cli.service_is_ready():
            waiting.append(name)
            continue
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        if not (fut.done() and fut.result() is not None and fut.result().current_state.label == 'active'):
            waiting.append(name)
    return waiting


def stop(proc, grace=6.0):
    # under the web manager's 10 s SIGINT grace, so Nav2 is never left behind
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--ready-timeout', type=float, default=60.0)
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--test-stuck-first', action='store_true',
                    help='test only: first launch with autostart:=false, so it never becomes active')
    opts, launch_args = ap.parse_known_args()

    rclpy.init()
    node, proc = None, None
    quit_now = []
    signal.signal(signal.SIGTERM, lambda *a: quit_now.append(1))
    signal.signal(signal.SIGINT, lambda *a: quit_now.append(1))
    try:
        for attempt in range(opts.restarts + 1):
            extra = ['autostart:=false'] if opts.test_stuck_first and attempt == 0 else []
            cmd = ['ros2', 'launch', 'ired_navigation', 'navigation.launch.xml'] + launch_args + extra
            log(f'start {attempt + 1} of {opts.restarts + 1}: {" ".join(cmd)}')
            proc = subprocess.Popen(cmd, start_new_session=True)
            # a fresh node per start: clients of the last one stayed matched to the
            # killed servers and never got an answer from the new ones (PC sim)
            if node is not None:
                node.destroy_node()
            node = rclpy.create_node(f'nav2_supervisor_{attempt}')
            clients = {n: node.create_client(GetState, f'{n}/get_state') for n in SERVERS}
            t0 = time.monotonic()
            waiting = list(SERVERS)
            while not quit_now and proc.poll() is None and time.monotonic() - t0 < opts.ready_timeout:
                waiting = all_active(node, clients)
                if not waiting:
                    break
                time.sleep(1.0)
            if quit_now:
                break
            if not waiting and proc.poll() is None:
                log(f'Nav2 active after {time.monotonic() - t0:.0f} s')
                while not quit_now and proc.poll() is None:
                    time.sleep(0.5)
                break
            why = 'launch exited' if proc.poll() is not None else f'not active after {opts.ready_timeout:.0f} s: {", ".join(waiting)}'
            log(f'Nav2 {why}, stopping it')
            stop(proc)
            proc = None
        else:
            log('Nav2 did not come up, giving up')
            return 1
    finally:
        if proc is not None:
            stop(proc)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
