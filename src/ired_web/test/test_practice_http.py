"""Actual authenticated manager handler + engine, with inert robot/process adapters."""
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import pytest
import yaml

from ired_web.manager import Manager, make_handler


class Client:
    def wait_for_service(self, **kw): return False


class Node:
    estop = False
    mapwatch = SimpleNamespace(watch=lambda on: None)
    def create_client(self, *a): return Client()
    def create_publisher(self, *a): return SimpleNamespace(publish=lambda msg: None)
    def create_subscription(self, *a, **kw): return object()
    def destroy_subscription(self, sub): pass
    def watch(self, topics): pass


@pytest.fixture
def server(tmp_path):
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/web.yaml').read_text())
    # Synthetic test authentication; never read or send the actual configured PIN.
    cfg.update(pin='test-only',log_dir=str(tmp_path),practice_broker_config=str(root/'config/practice-mosquitto.conf'),
               game_refbox_address='127.0.0.1',game_robot_topic='test-robot',practice_watchdog='false')
    manager=Manager(cfg,Node())
    http=ThreadingHTTPServer(('127.0.0.1',0),make_handler(manager))
    thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
    def request(action,body=None,auth=True):
        headers={'Content-Type':'application/json'}
        if auth: headers['X-Pin']='test-only'
        req=Request(f'http://127.0.0.1:{http.server_port}/api/practice/{action}',
                    data=json.dumps(body).encode() if body is not None else None,headers=headers)
        try:
            with urlopen(req,timeout=5) as response: return response.status,json.load(response)
        except HTTPError as e: return e.code,json.load(e)
    yield manager,request
    http.shutdown();thread.join();http.server_close();manager.practice.close()


def test_auth_and_routes(server):
    m,req=server
    assert req('status',auth=False)[0]==401
    assert req('stop',{},auth=False)[0]==401
    assert req('status')[0]==200
    assert req('unknown',{})[0]==404
    assert req('prepare',[])[0]==400
    assert req('source',{'source':'arbitrary-host'})[0]==400


def test_prepare_levels_reload_and_no_autostart(server):
    m,req=server
    saved=None
    for n in (2,3,4):
        code,d=req('prepare',{'n':n,'seed':11})
        assert code==200 and d['state']=='prepared' and len(d['config']['machines'])==n
        saved=d['layout_id']
        assert m.profiles['robot'].proc is None and m.profiles['challenge_nav'].proc is None
    assert req('start',{'layout_id':saved,'confirmed':True})[0]==400
    assert req('prepare',{'layout_id':saved,'n':2})[0]==400
    code,d=req('prepare',{'layout_id':saved})
    assert code==200 and d['config']['n']==4
    assert req('prepare',{'layout_id':'../../web.yaml'})[0]==400


def test_start_stop_race_is_fail_closed(server):
    m,req=server
    d=req('prepare',{'n':2,'seed':1})[1]
    m.practice.source='practice';m.practice.game.ready=lambda:[]
    m.practice.game.on_pose(4.5,.5,3.14159265)
    assert req('start',{'layout_id':d['layout_id'],'confirmed':True})[0]==200
    assert req('start',{'layout_id':d['layout_id'],'confirmed':True})[0]==400
    assert req('prepare',{'n':3})[0]==400
    assert req('stop',{})[1]['state']=='stopped'
    assert req('start',{'layout_id':d['layout_id'],'confirmed':True})[0]==400
    assert req('status')[1]['history'][0]['state']=='stopped'


def test_practice_connect_never_auto_starts_robot(server):
    m,req=server
    assert req('source',{'source':'practice'})[0]==200
    end=time.monotonic()+3
    while m.practice.busy and time.monotonic()<end: time.sleep(.01)
    assert 'Start Robot' in m.practice.error
    assert m.profiles['robot'].proc is None
    assert m.profiles['navigation_softstop'].proc is None


def test_profile_format_uses_selected_source(server):
    m,_=server
    for local in (False,True):
        m.practice.source='practice' if local else 'field'
        spec=m.profiles['challenge_nav'].spec
        command=spec['command'].format(**dict(m.cfg,**m.profile_fields()))
        assert ('REFBOX_BRIDGE_ADDRESS=127.0.0.1' in command)==local
        assert f'practice_watchdog:={str(local).lower()}' in command
        assert ('ROBOT_IP=ired-practice' in command)==local
