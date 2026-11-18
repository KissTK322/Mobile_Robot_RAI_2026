"""Replay page: bag listing, conversion in a child process, and the HTTP routes."""
import json
import os
import shutil
import threading
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import pytest
import yaml

from ired_web.manager import Manager, make_handler
from ired_web.replay import ReplayStore
from test_practice_http import Node

RACE = Path('~/ired_work/robot_runs/bags/nav-20261116-100745').expanduser()


def fake_bag(root, name):
    d = root / name
    d.mkdir(parents=True)
    (d / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    return d


def test_list_and_names(tmp_path):
    s = ReplayStore(tmp_path / 'bags', tmp_path / 'cache')
    assert s.list() == []
    fake_bag(tmp_path / 'bags', 'nav-1')
    (tmp_path / 'bags' / 'not-a-bag').mkdir()
    assert [b['name'] for b in s.list()] == ['nav-1']
    assert s.list()[0]['state'] == 'new'
    for bad in ('../etc', '.hidden', 'a/b', '', None, 'missing'):
        with pytest.raises(ValueError):
            s.convert(bad)
    with pytest.raises(ValueError):
        s.game('nav-1')


def test_failed_conversion_reports_error(tmp_path):
    s = ReplayStore(tmp_path / 'bags', tmp_path / 'cache')
    fake_bag(tmp_path / 'bags', 'nav-1')            # metadata without data: the reader fails
    s.convert('nav-1')
    for _ in range(300):
        if s.list()[0]['state'] != 'converting':
            break
        time.sleep(0.1)
    b = s.list()[0]
    assert b['state'] == 'error' and b['error']


@pytest.mark.skipif(not RACE.is_dir(), reason='race bag only on the team PC')
def test_converts_race_exploration(tmp_path):
    shutil.copytree(RACE, tmp_path / 'bags' / RACE.name)
    s = ReplayStore(tmp_path / 'bags', tmp_path / 'cache')
    s.convert(RACE.name)
    for _ in range(1200):
        if s.list()[0]['state'] != 'converting':
            break
        time.sleep(0.1)
    assert s.list()[0]['state'] == 'ready', s.list()[0]['error']
    g = json.loads(s.game(RACE.name))
    assert g['title'] == 'Exploration' and abs(g['end'] - 56.6) < 0.5
    assert {b['name']: b['zone'] for b in g['boxes']} == {'C-BS': 'C_Z22', 'C-CS1': 'C_Z53'}
    assert g['track'] and g['lidar'] and g['seen']
    assert [e for e in g['ev'] if e[1] == 'report'][-1][2].startswith('Report C-CS1')


@pytest.fixture
def server(tmp_path):
    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root / 'config/web.yaml').read_text())
    # Synthetic test authentication; never read or send the actual configured PIN.
    cfg.update(pin='test-only', log_dir=str(tmp_path), bags_dir=str(tmp_path / 'bags'),
               practice_broker_config=str(root / 'config/practice-mosquitto.conf'),
               game_refbox_address='127.0.0.1', game_robot_topic='test-robot', practice_watchdog='false')
    manager = Manager(cfg, Node())
    http = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(manager))
    thread = threading.Thread(target=http.serve_forever, daemon=True); thread.start()

    def request(path, body=None, auth=True):
        headers = {'Content-Type': 'application/json'}
        if auth:
            headers['X-Pin'] = 'test-only'
        req = Request(f'http://127.0.0.1:{http.server_port}/api/replay/{path}',
                      data=json.dumps(body).encode() if body is not None else None, headers=headers)
        try:
            with urlopen(req, timeout=5) as r:
                return r.status, r.read()
        except HTTPError as e:
            return e.code, e.read()
    yield manager, request
    http.shutdown(); thread.join(); http.server_close(); manager.practice.close()


def test_http_routes(server, tmp_path):
    m, req = server
    assert req('list', auth=False)[0] == 401
    assert json.loads(req('list')[1]) == {'bags': []}
    fake_bag(tmp_path / 'bags', 'nav-1')
    assert json.loads(req('list')[1])['bags'][0]['name'] == 'nav-1'
    assert req('game?name=nav-1')[0] == 404
    assert req('convert', {'name': '../x'})[0] == 400
    assert req('unknown')[0] == 404
    # never convert while a challenge runs
    m.profiles['challenge_exp_lidar'].info = lambda: {'state': 'running'}
    code, body = req('convert', {'name': 'nav-1'})
    assert code == 400 and b'Stop the challenge' in body
    os.makedirs(tmp_path / 'replay', exist_ok=True)
    (tmp_path / 'replay' / 'nav-1.json').write_text('{"id":"nav-1"}')
    assert req('game?name=nav-1') == (200, b'{"id":"nav-1"}')
