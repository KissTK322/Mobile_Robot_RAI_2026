#!/usr/bin/env python3
"""Sequential real ROS/Nav2 PC checks of the mobile refbox and opt-in lease.
Run with ROS + sim_ws sourced and this branch's Python packages first in PYTHONPATH.
Uses domain77, an already running LOOPBACK MQTT broker, and owns every child process.
No SSH, no robot/network broker, no installed/source sim workspace modifications.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import yaml

import psutil
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ired_web.practice import PracticeGame, ACTIVE, generate_layout
from ired_web.practice_runtime import PracticeMQTT

ROOT=Path(__file__).resolve().parents[2]
WORK=Path('/home/ired/ired_work')


class Observer(Node):
    def __init__(self):
        super().__init__('mobile_refbox_test_observer')
        self.nav={};self.nav_at=self.scan_at=0.;self.true=None;self.true_trace=[];self.cmd=[];self.states=[]
        self.initial_pose=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.true_yaw=math.pi
        self.heartbeat=self.create_publisher(String,'/practice/heartbeat',10)
        self.create_subscription(String,'/challenge/navigation/state',self.on_nav,10)
        self.create_subscription(LaserScan,'/scan_filtered',lambda m:setattr(self,'scan_at',time.monotonic()),qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/sim/true_pose',self.on_true,10)
        self.create_subscription(TwistStamped,'/cmd_vel',self.on_cmd,30)
        self.cancel_clients=[self.create_client(CancelGoal,p) for p in ('/navigate_to_pose/_action/cancel_goal','/follow_path/_action/cancel_goal')]
    def on_true(self,m):
        self.true=(m.pose.position.x,m.pose.position.y)
        q=m.pose.orientation
        self.true_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.true_trace.append((time.monotonic(),*self.true))
    def seed_current_pose(self):
        # Simulate the user's Map start-pose action after preparing a NEW round.
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y=self.true
        msg.pose.pose.orientation.z=math.sin(self.true_yaw/2);msg.pose.pose.orientation.w=math.cos(self.true_yaw/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.01;msg.pose.covariance[35]=.02
        self.initial_pose.publish(msg)
    def on_nav(self,m):
        self.nav=json.loads(m.data);self.nav_at=time.monotonic()
        self.states.append([self.nav_at,self.nav.get('state'),self.nav.get('practice_watchdog_ok'),self.nav.get('game_on')])
    def on_cmd(self,m):
        self.cmd.append((time.monotonic(),math.hypot(m.twist.linear.x,m.twist.linear.y)))
    def cancel(self,reason):
        for c in self.cancel_clients:
            if c.service_is_ready():c.call_async(CancelGoal.Request())
    def ready(self):
        now=time.monotonic();issues=[]
        if now-self.nav_at>3:issues.append('Navigation state stale')
        if now-self.scan_at>2:issues.append('LiDAR stale')
        if self.nav.get('practice_watchdog_ok') is False:issues.append('Practice lease tripped')
        return issues


def main():
    p=argparse.ArgumentParser();p.add_argument('--boxes',type=int,default=2);p.add_argument('--seed',type=int,default=11)
    p.add_argument('--fixture',choices=['game','stop','lease-loss','mqtt-loss','rearm'],default='game');p.add_argument('--name',required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='77':raise SystemExit('Refusing: ROS_DOMAIN_ID must be77')
    out=ROOT/'tools/mobile_refbox/results'/args.name;out.mkdir(parents=True,exist_ok=False)
    config=generate_layout({'n':args.boxes,'seed':args.seed,'limit_s':600,'countdown_s':3})
    physical=out/'physical-layout.json';physical.write_text(json.dumps(config))
    os.environ['GAME_JSON']=str(physical)
    params=yaml.safe_load((ROOT/'src/ired_navigation/param/navigation_softstop.yaml').read_text())
    params['map_server']['ros__parameters']['yaml_filename']=str(WORK/'sim_ws/src/ired_navigation/maps/field.yaml')
    for key in ('default_nav_to_pose_bt_xml','default_nav_through_poses_bt_xml'):
        params['bt_navigator']['ros__parameters'][key]=params['bt_navigator']['ros__parameters'][key].replace('$(find-pkg-share nav2_bt_navigator)','/opt/ros/jazzy/share/nav2_bt_navigator')
    param_path=out/'params.yaml';param_path.write_text(yaml.safe_dump(params))
    children=[];handles=[]
    def launch(name,command):
        log=(out/(name+'.log')).open('w');handles.append(log)
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=out,start_new_session=True)
        children.append(proc);return proc
    rclpy.init();o=Observer();ex=SingleThreadedExecutor();ex.add_node(o)
    spin=threading.Thread(target=ex.spin,daemon=True);spin.start()
    link=None;record={'boxes':args.boxes,'seed':args.seed,'fixture':args.fixture}
    cpu=psutil.Process();cpu.cpu_percent()
    try:
        launch('robot',['python3',str(WORK/'sim_tools/sim_robot.py')])
        launch('nav2',['ros2','launch','nav2_bringup','bringup_launch.py',
                      'map:='+str(WORK/'sim_ws/src/ired_navigation/maps/field.yaml'),
                      'params_file:='+str(param_path),
                      'use_composition:=False'])
        holder={}
        def publish(t,p):
            if 'link' in holder:holder['link'].publish(t,p)
        game=PracticeGame(out/'refbox',publish,lambda:o.ready()+([] if link and link.connected else ['MQTT offline']),o.cancel)
        game.prepare(config)
        link=PracticeMQTT(game);holder['link']=link
        game.tick()
        env_robot=os.environ.get('ROBOT_IP')
        assert env_robot=='ired-practice', 'Set ROBOT_IP=ired-practice for isolated MQTT namespace'
        launch('bridge',['ros2','run','ired_refbox','mqtt_refbox'])
        launch('mps',['ros2','run','ired_challenge','mps_obstacles','--ros-args','--params-file',str(ROOT/'src/ired_challenge/config/navigation_challenge.yaml')])
        launch('challenge',['ros2','run','ired_challenge','navigation_challenge','--ros-args',
                            '--params-file',str(ROOT/'src/ired_challenge/config/navigation_challenge.yaml'),'-p','practice_watchdog:=true'])
        end=time.monotonic()+90
        while time.monotonic()<end:
            game.tick()
            if not game.ready() and game.pose_at is not None and game.clock()-game.pose_at<2:break
            if any(c.poll() is not None for c in children):raise RuntimeError('A simulation process exited during startup')
            time.sleep(.2)
        else:raise RuntimeError('Simulation not ready: '+str(game.ready()))
        # Preparation stays non-moving for several bridge cycles.
        wait=time.monotonic()+3
        while time.monotonic()<wait:game.tick();time.sleep(.2)
        assert not any(v>.005 for _,v in o.cmd),'Unexpected motion before explicit Start'
        game.start(game.layout_id,True)
        start=time.monotonic();fault_at=None;baseline_pose=None
        while time.monotonic()-start<650:
            now=time.monotonic()
            if args.fixture=='game' or fault_at is None:
                if game.state in ACTIVE:o.heartbeat.publish(String(data=game.run_id))
                game.tick()
            speed=o.cmd[-1][1] if o.cmd else 0
            if args.fixture!='game' and fault_at is None and game.state=='running' and speed>.08:
                fault_at=now;baseline_pose=o.true
                if args.fixture in ('stop','rearm'):game.stop('Simulated phone Stop game')
                elif args.fixture=='mqtt-loss':link.close()
                # lease-loss stops both heartbeat and web refbox ticks, leaving cached MQTT RUNNING.
            if fault_at is not None and now-fault_at>7:
                after=[(t,v) for t,v in o.cmd if t>=fault_at]
                tail=[v for t,v in after if t>=fault_at+5]
                # collision_monitor stops publishing after its zero tail; retain the last command.
                if o.cmd and o.cmd[-1][0]>=fault_at:tail.append(o.cmd[-1][1])
                poses=[(x,y) for t,x,y in o.true_trace if t>=fault_at+5]
                stationary=max((math.dist(p,poses[0]) for p in poses),default=999)<.005
                first_zero=next((t-fault_at for t,v in after if v<.005),None)
                record.update(fault_at=fault_at,first_zero_s=first_zero,
                              tail_max_speed=max(tail,default=None),stationary_tail=stationary,pose_before=baseline_pose,pose_after=o.true,
                              lease_tripped=o.nav.get('practice_watchdog_ok') is False)
                assert tail and max(tail)<.005 and stationary, 'Did not stop after fault/Stop'
                if args.fixture=='lease-loss':
                    assert record['lease_tripped'],'Practice lease did not trip'
                    # Recovered heartbeat must NOT restart motion with a cached RUNNING.
                    for _ in range(15):o.heartbeat.publish(String(data=game.run_id));time.sleep(.2)
                    assert o.cmd[-1][1]<.005,'Lease silently resumed'
                if args.fixture=='rearm':
                    old_run=game.run_id
                    # Restart exactly the same bridge+challenge group that Manager rearms.
                    old=children[2:]
                    for proc in old:
                        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                    for proc in old:
                        try:proc.wait(timeout=3)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    game.prepare(config);game.pose_at=None;o.nav={};o.nav_at=0
                    launch('bridge-round2',['ros2','run','ired_refbox','mqtt_refbox'])
                    launch('mps-round2',['ros2','run','ired_challenge','mps_obstacles','--ros-args','--params-file',str(ROOT/'src/ired_challenge/config/navigation_challenge.yaml')])
                    launch('challenge-round2',['ros2','run','ired_challenge','navigation_challenge','--ros-args','--params-file',str(ROOT/'src/ired_challenge/config/navigation_challenge.yaml'),'-p','practice_watchdog:=true'])
                    deadline=time.monotonic()+30
                    next_seed=time.monotonic()+2
                    while time.monotonic()<deadline:
                        game.tick()
                        if time.monotonic()>=next_seed:
                            o.seed_current_pose();next_seed=time.monotonic()+1
                        if not game.start_problems():break
                        time.sleep(.2)
                    assert not game.start_problems(),'Second challenge did not rearm: '+str(game.start_problems())
                    game.start(game.layout_id,True)
                    deadline=time.monotonic()+15
                    while time.monotonic()<deadline:
                        o.heartbeat.publish(String(data=game.run_id));game.tick()
                        if game.state=='running' and o.nav.get('game_on') and o.nav.get('target')=='C_Z32' and o.cmd[-1][1]>.03:break
                        time.sleep(.2)
                    assert game.run_id!=old_run and o.nav.get('practice_watchdog_ok') and o.nav.get('target')=='C_Z32' and o.cmd[-1][1]>.03,'Second game did not start safely'
                    record['second_round_started']=True
                    record['second_run_id']=game.run_id
                    game.stop('Second round verified')
                break
            if args.fixture=='game' and game.state not in ACTIVE:
                break
            if int(now-start)%15==0 and now-getattr(o,'reported',0)>10:
                print(json.dumps({'elapsed':round(now-start),'state':game.state,'counted':len(game.done_at),'target':o.nav.get('target')}),flush=True);o.reported=now
            time.sleep(.2)
        if args.fixture=='game' and game.state=='finished':
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:game.tick();time.sleep(.2)
            record['final_stop_verified']=not o.nav.get('game_on') and o.cmd and o.cmd[-1][1]<.005
            assert record['final_stop_verified'],'Finished game did not stop navigation'
        record.update(game=game.snapshot(),history=game.history(),last_nav=o.nav,
                      harness_cpu_one_core_percent=cpu.cpu_percent(),duration_s=time.monotonic()-start)
        if args.fixture=='game':assert game.state=='finished' and len(game.done_at)==12,game.message
        print('RESULT '+json.dumps(record),flush=True)
    except Exception as e:
        record['error']=repr(e);raise
    finally:
        record['contacts']=sum('contact at' in line for line in (out/'robot.log').read_text().splitlines()) if (out/'robot.log').exists() else None
        (out/'result.json').write_text(json.dumps(record,indent=2))
        (out/'commands.json').write_text(json.dumps(o.cmd))
        (out/'states.json').write_text(json.dumps(o.states))
        (out/'true_pose.json').write_text(json.dumps(o.true_trace))
        if link:link.close()
        for c in reversed(children):
            if c.poll() is None:
                try:os.killpg(c.pid,signal.SIGINT)
                except ProcessLookupError:pass
        end=time.monotonic()+8
        while any(c.poll() is None for c in children) and time.monotonic()<end:time.sleep(.1)
        for c in children:
            if c.poll() is None:
                try:os.killpg(c.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            c.wait()
        for h in handles:h.close()
        ex.shutdown(timeout_sec=2);spin.join(timeout=2);o.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
