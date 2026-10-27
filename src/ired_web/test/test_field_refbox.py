"""Field refbox broker address: web.yaml default, override from the Game page, connection test."""
import socket

import pytest

from ired_web.manager import Manager


class P:
    def __init__(self, state):
        self.state = state

    def info(self):
        return {'state': self.state}


def fake(tmp_path, default='172.16.1.4', challenge='stopped'):
    m = Manager.__new__(Manager)
    m.cfg = {'refbox_address': default, 'log_dir': str(tmp_path)}
    m.profiles = {'challenge_nav': P(challenge), 'challenge_exp': P('stopped')}
    return m


def test_default_then_override_then_back(tmp_path):
    m = fake(tmp_path)
    assert m.field_refbox() == '172.16.1.4'
    info = m.set_field_refbox(' 172.16.1.50 ')
    assert info['address'] == '172.16.1.50' and info['default'] == '172.16.1.4'
    assert fake(tmp_path).field_refbox() == '172.16.1.50'        # survives a web restart
    assert m.set_field_refbox('')['address'] == '172.16.1.4'


@pytest.mark.parametrize('bad', ['172.16.1.4; rm -rf ~', '$(reboot)', 'a b', '-x'])
def test_rejects_anything_but_a_host(tmp_path, bad):
    with pytest.raises(ValueError):
        fake(tmp_path).set_field_refbox(bad)


def test_cannot_change_while_challenge_runs(tmp_path):
    with pytest.raises(ValueError):
        fake(tmp_path, challenge='running').set_field_refbox('172.16.1.50')


def test_connection_test(tmp_path):
    srv = socket.socket(); srv.bind(('127.0.0.1', 0)); srv.listen(1)
    port = srv.getsockname()[1]
    m = fake(tmp_path)
    m.set_field_refbox('127.0.0.1')
    real = socket.create_connection
    socket.create_connection = lambda addr, timeout: real((addr[0], port), timeout)   # broker port 1883 -> test port
    try:
        assert m.field_refbox_info(test=True)['broker_ok'] is True
        srv.close()
        out = m.field_refbox_info(test=True)
        assert out['broker_ok'] is False and out['broker_error']
    finally:
        socket.create_connection = real
