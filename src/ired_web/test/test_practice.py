"""Practice refbox protocol, immutable fields, continuous dwell and fail-closed lifecycle."""
import json
import math

import pytest

from ired_web.practice import PracticeGame, generate_layout, validate_layout, near
from ired_challenge.practice_lease import PracticeLease
from ired_challenge.exploration import ORANGE_ZONES, allowed_rotations


class Clock:
    def __init__(self): self.t = 100.
    def __call__(self): return self.t
    def advance(self, dt): self.t += dt


@pytest.fixture
def rig(tmp_path):
    clock = Clock(); sent = []; halted = []; problems = []
    game = PracticeGame(tmp_path, lambda t,p: sent.append((t,p)), lambda: problems,
                        halted.append, clock, clock)
    game.prepare({'n': 2, 'seed': 11, 'limit_s': 60, 'countdown_s': 3})
    return game, clock, sent, halted, problems


def begin(rig):
    game, clock, sent, halted, problems = rig
    game.on_pose(4.5,.5,math.pi)
    game.start(game.layout_id, True)
    for _ in range(4):
        clock.advance(1)
        game.on_pose(4.5,.5,math.pi)
        game.tick()
    assert game.state == 'running'
    return game


@pytest.mark.parametrize('n', [2,3,4])
def test_generator_rules_many_seeds(n):
    for seed in range(200):
        cfg=generate_layout({'n':n,'seed':seed})
        assert cfg == generate_layout({'n':n,'seed':seed})
        assert len(cfg['machines']) == n and len(set(cfg['route'])) == 12
        for m in cfg['machines']:
            assert m['zone'] not in ORANGE_ZONES
            assert m['rotation'] in allowed_rotations(m['zone'],m['name'])
        assert not any(near(a['zone'],b['zone']) for i,a in enumerate(cfg['machines']) for b in cfg['machines'][i+1:])


def test_prepare_never_autostarts(rig):
    g,c,s,h,_=rig
    for _ in range(60): c.advance(1);g.tick()
    assert g.state=='prepared'
    assert all(p!='RUNNING' for t,p in s if t=='game_state')
    assert not h


@pytest.mark.parametrize('field,value', [('n',5),('n',True),('seed',-1),('seed','x'),('limit_s',0),('countdown_s',2)])
def test_reject_settings(field,value):
    with pytest.raises(ValueError): generate_layout({field:value})


def test_layout_count_and_placement_validation(rig):
    g,*_=rig
    cfg=g.snapshot()['config'];cfg['n']=3
    with pytest.raises(ValueError,match='count'): validate_layout(cfg)
    cfg=g.snapshot()['config'];cfg['machines'][0]['zone']='C_Z13'
    with pytest.raises(ValueError): validate_layout(cfg)
    cfg=g.snapshot()['config'];cfg['machines'][0]['zone']='C_Zxx'
    with pytest.raises(ValueError): validate_layout(cfg)
    cfg=g.snapshot()['config'];cfg['route'][0]=cfg['route'][1]
    with pytest.raises(ValueError): validate_layout(cfg)


def test_start_gates_and_stale_requests(rig):
    g,c,_,_,problems=rig
    with pytest.raises(ValueError): g.start('old-id',True)
    with pytest.raises(ValueError): g.start(g.layout_id,False)
    with pytest.raises(ValueError): g.start(g.layout_id,True)
    g.on_pose(3.5,.5,math.pi)
    with pytest.raises(ValueError): g.start(g.layout_id,True)
    g.on_pose(4.5,.5,0)
    with pytest.raises(ValueError): g.start(g.layout_id,True)
    g.on_pose(4.5,.5,math.pi);problems.append('LiDAR stale')
    with pytest.raises(ValueError,match='LiDAR'): g.start(g.layout_id,True)
    problems.clear();c.advance(3)
    with pytest.raises(ValueError,match='fresh'): g.start(g.layout_id,True)


def test_countdown_explicit_and_protocol_order(rig):
    g,c,s,_,_=rig
    g.on_pose(4.5,.5,-math.pi);g.start(g.layout_id,True)
    c.advance(1);g.on_pose(4.5,.5,-math.pi);g.tick()
    assert g.state=='countdown' and g.snapshot()['countdown_left']==2
    assert not any(t=='game_state' and p=='RUNNING' for t,p in s)
    c.advance(2);g.on_pose(4.5,.5,-math.pi);g.tick()
    assert g.state=='running'
    assert [t for t,p in s[-5:]]==['teamColor','phase','private/machine_info','private/navigation_routes','game_state']
    assert json.loads(s[-2][1])['routes'][0]['route']==g.config['route']
    with pytest.raises(ValueError): g.start(g.layout_id,True)
    with pytest.raises(ValueError): g.prepare({'n':4})


def test_continuous_dwell_resets_on_exit_or_missing_pose(rig):
    g=begin(rig);c=rig[1];zone=g.remaining[0];x,y=int(zone[3])-.5,int(zone[4])-.5
    for _ in range(5): g.on_pose(x,y,0);c.advance(1)
    assert zone in g.remaining
    g.on_pose(-1,-1,0)
    for _ in range(6): g.on_pose(x,y,0);c.advance(1)
    assert zone not in g.remaining and len(g.done_at)==1
    another=g.remaining[0];x,y=int(another[3])-.5,int(another[4])-.5
    g.on_pose(x,y,0);c.advance(4);g.on_pose(x,y,0)
    assert another in g.remaining
    assert g.inside[another]==c()


@pytest.mark.parametrize('failure', ['pose','readiness','countdown-position'])
def test_failure_stops_and_never_auto_resumes(rig,failure):
    g,c,s,h,problems=rig
    if failure=='countdown-position':
        g.on_pose(4.5,.5,math.pi);g.start(g.layout_id,True);g.on_pose(3,.5,math.pi)
    else:
        begin(rig)
        if failure=='pose': c.advance(3)
        else: problems.append('MQTT disconnected')
    g.tick()
    assert g.state=='error' and len(h)==1 and s[-1]==('game_state','PAUSED')
    problems.clear();g.on_pose(4.5,.5,math.pi);c.advance(1);g.tick()
    assert g.state=='error' and len(h)==1


def test_stop_and_finish_persist_and_freeze_time(rig):
    g=begin(rig);c=rig[1]
    g.stop();elapsed=g.snapshot()['elapsed_s'];c.advance(20)
    assert g.snapshot()['elapsed_s']==elapsed
    assert g.history()[0]['state']=='stopped'
    g.prepare({'n':3,'seed':10,'limit_s':60,'countdown_s':3});begin(rig)
    g.remaining.clear();g.tick()
    assert g.state=='finished' and g.history()[0]['reason']=='All targets counted'


def test_time_limit(rig):
    g=begin(rig);c=rig[1]
    c.advance(61);g.on_pose(4.5,.5,math.pi);g.tick()
    assert g.state=='finished' and g.message=='Time limit reached'


def test_restore_layout_and_restart_never_resumes(rig,tmp_path):
    g=begin(rig);c=rig[1]
    saved=g.layout_id;cfg=g.snapshot()['config']
    replacement=PracticeGame(tmp_path,lambda *a:None,clock=c,wall=c)
    assert replacement.state=='idle'
    assert replacement.history()[0]['state']=='interrupted'
    out=replacement.prepare({'layout_id':saved})
    assert out['config']==cfg and out['layout_id']!=saved
    with pytest.raises(ValueError): replacement.prepare({'layout_id':saved,'n':4})
    with pytest.raises(ValueError): replacement.prepare({'layout_id':'../outside'})


def test_bad_pose_does_not_score_or_refresh(rig):
    g=begin(rig);before=g.pose_at
    g.on_pose(float('nan'),2,0);g.on_pose(1,1,'bad')
    assert g.pose_at==before


def test_layout_response_is_a_copy(rig):
    g,*_=rig;config=g.snapshot()['config'];config['machines'].clear()
    assert len(g.config['machines'])==2


def test_practice_lease_is_opt_in_and_latched():
    c=Clock();lease=PracticeLease(True,clock=c);token='a'*32
    assert lease.allowed(False) is False
    lease.heartbeat(token);assert lease.allowed(True)
    c.advance(2.01);assert not lease.allowed(True)
    lease.heartbeat(token);assert not lease.allowed(True)
    assert PracticeLease(False,clock=c).allowed(True)
    lease=PracticeLease(True,clock=c);lease.heartbeat('invalid');assert not lease.allowed(True)
    lease=PracticeLease(True,clock=c);lease.heartbeat(token);lease.heartbeat('b'*32);assert not lease.allowed(True)


def test_storage_failure_cannot_block_stop(rig,monkeypatch):
    g=begin(rig)
    def fail(*args): raise OSError('Disk full')
    monkeypatch.setattr(g,'_record',fail)
    with pytest.raises(OSError): g.stop()
    assert rig[3] and rig[2][-1]==('game_state','PAUSED')


def test_corrupt_history_does_not_prevent_web_start(tmp_path):
    (tmp_path/'run-bad.json').write_text('corrupt')
    (tmp_path/'layout-bad.json').write_text('[]')
    game=PracticeGame(tmp_path,lambda *a:None)
    assert game.history()==[] and game.saved()==[]


def test_bridge_compass_yaw_and_retained_pose():
    from types import SimpleNamespace
    from ired_web.practice_runtime import PracticeMQTT
    received=[];link=PracticeMQTT.__new__(PracticeMQTT)
    link.game=SimpleNamespace(on_pose=lambda *p:received.append(p))
    for yaw in (90,270,0):
        link.on_message(None,None,SimpleNamespace(topic='ired-practice/pose',retain=False,payload=json.dumps({'x':4.5,'y':.5,'yaw':yaw}).encode()))
    assert received[0][2]==pytest.approx(-math.pi)
    assert received[1][2]==pytest.approx(0)
    assert received[2][2]==pytest.approx(-math.pi/2)
    link.on_message(None,None,SimpleNamespace(topic='ired-practice/pose',retain=True,payload=b'{"x":4.5,"y":0.5,"yaw":90}'))
    assert len(received)==3


def test_one_hz_bridge_jitter_does_not_reset_continuous_dwell(rig):
    g=begin(rig);c=rig[1];zone=g.remaining[0];x,y=int(zone[3])-.5,int(zone[4])-.5
    for _ in range(7):g.on_pose(x,y,0);c.advance(1.03)
    assert zone not in g.remaining


def test_expired_practice_lease_cancels_even_without_tf():
    from types import SimpleNamespace
    from ired_challenge.navigation_node import NavigationChallengeNode
    clock=Clock();lease=PracticeLease(True,clock=clock);lease.heartbeat('a'*32);clock.advance(3)
    cancelled=[]
    fake=SimpleNamespace(practice_lease=lease,game_on=True,pose=lambda:None,
                         task=SimpleNamespace(nav=SimpleNamespace(cancel=lambda:cancelled.append(True))))
    NavigationChallengeNode.step(fake)
    assert cancelled and lease.tripped


# ---------------------------------------------------------------- exploration
from ired_web.practice import score_report


@pytest.fixture
def exp(tmp_path):
    clock = Clock(); sent = []; halted = []
    game = PracticeGame(tmp_path, lambda t,p: sent.append((t,p)), lambda: [], halted.append, clock, clock)
    game.prepare({'mode': 'exploration', 'n': 2, 'seed': 5, 'limit_s': 600, 'countdown_s': 3})
    return game, clock, sent, halted, []


def centre(m):
    return int(m['zone'][3])-.5, int(m['zone'][4])-.5


def test_score_report_points():
    truth = {'zone': 'C_Z23', 'rotation': 90}
    assert score_report(truth, 1.5, 2.5, 92)['points'] == 5.5
    assert score_report(truth, 1.5, 2.5, 180)['points'] == 1.5
    assert score_report(truth, 2.5, 2.5, 90)['points'] == -2
    assert score_report(None, 1.5, 2.5, 90)['points'] == -2
    assert score_report({'zone': 'C_Z23', 'rotation': 0}, 1.2, 2.9, 358)['rotation_ok']


def test_exploration_layout_has_no_route_and_only_bs_cs1():
    cfg = generate_layout({'mode': 'exploration', 'n': 2, 'seed': 3})
    assert cfg['route'] == [] and sorted(m['name'] for m in cfg['machines']) == ['C-BS', 'C-CS1']
    with pytest.raises(ValueError):
        validate_layout(dict(cfg, n=3))
    with pytest.raises(ValueError):
        validate_layout(dict(cfg, mode='race'))


def test_exploration_publishes_no_ground_truth(exp):
    g,c,s,h,_ = exp
    begin(exp)
    running = [p for t,p in s if t in ('private/machine_info','private/navigation_routes')]
    assert running and all(json.loads(p) == {} for p in running)


def test_exploration_first_report_counts_and_finishes(exp):
    g,c,s,h,_ = exp
    begin(exp)
    bs, cs = sorted(g.config['machines'], key=lambda m: m['name'])
    x, y = centre(bs)
    assert g.on_report('C-BS', x, y, bs['rotation'])['points'] == 5.5
    assert g.on_report('C-BS', 0.5, 0.5, 0) is None          # only the first report counts
    assert g.score() == 5.5 and g.state == 'running'
    g.on_report('C-CS1', 4.5, 4.5, (cs['rotation']+90) % 360)
    g.tick()
    assert g.state == 'finished' and len(g.reports) == 2


def test_exploration_negative_total_is_zero_and_report_before_start_ignored(exp):
    g,c,s,h,_ = exp
    assert g.on_report('C-BS', 1.5, 1.5, 0) is None
    begin(exp)
    bs = next(m for m in g.config['machines'] if m['name'] == 'C-BS')
    wrong = next(z for z in ('C_Z11', 'C_Z55') if z != bs['zone'])
    g.on_report('C-BS', int(wrong[3])-.5, int(wrong[4])-.5, 0)
    g.on_report('C-XX', 0.5, 0.5, 0)
    assert sum(r['points'] for r in g.reports.values()) == -4 and g.score() == 0


def test_bridge_dispatches_machine_report():
    from types import SimpleNamespace
    from ired_web.practice_runtime import PracticeMQTT
    got=[];link=PracticeMQTT.__new__(PracticeMQTT)
    link.game=SimpleNamespace(on_report=lambda *a:got.append(a), on_pose=lambda *a:got.append(('pose',)+a))
    link.on_message(None,None,SimpleNamespace(topic='ired-practice/machine_report',retain=False,
                    payload=json.dumps({'machine':'C-BS','x':1.5,'y':2.5,'yaw':90}).encode()))
    assert got == [('C-BS',1.5,2.5,90)]
