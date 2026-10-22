"""On-demand loopback MQTT transport and web manager orchestration for practice games."""
import json
import math
import threading
import time

from ired_web.practice import ACTIVE, PracticeGame


class PracticeMQTT:
    def __init__(self, game, host='127.0.0.1', port=1883, topic='ired-practice'):
        import paho.mqtt.client as mqtt
        self.game, self.topic = game, topic
        self.connected = False
        self.client = mqtt.Client(client_id='ired-web-practice', clean_session=True)
        self.client.will_set(f'{topic}/game_state', 'PAUSED', qos=1, retain=False)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.connect(host, int(port), keepalive=5)
        self.client.loop_start()
        end = time.monotonic()+3
        while not self.connected and time.monotonic()<end:
            time.sleep(.02)
        if not self.connected:
            self.close()
            raise ValueError('Practice MQTT connection timed out')

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            # A broker reconnect never resumes an old game.
            client.publish(f'{self.topic}/game_state', 'PAUSED', qos=1)
            client.subscribe(f'{self.topic}/pose', qos=0)
            self.connected = True

    def on_disconnect(self, *args):
        self.connected = False
        if self.game.state in ACTIVE:
            self.game.stop('Practice MQTT disconnected; prepare a new round', error=True)

    def on_message(self, client, userdata, message):
        # Retained pose is not live robot feedback.
        if message.retain or len(message.payload)>2048:
            return
        try:
            p = json.loads(message.payload)
            # Existing ired_refbox bridge publishes compass degrees: 270 - ROS yaw.
            yaw = math.radians((270.0-float(p['yaw'])) % 360.0)
            yaw = (yaw+math.pi) % (2*math.pi)-math.pi
            self.game.on_pose(p['x'], p['y'], yaw)
        except (ValueError, TypeError, KeyError):
            return

    def publish(self, topic, payload):
        if self.connected:
            self.client.publish(f'{self.topic}/{topic}', payload, qos=1, retain=False)

    def close(self):
        if self.connected:
            msg = self.client.publish(f'{self.topic}/game_state', 'PAUSED', qos=1)
            try:
                msg.wait_for_publish(timeout=1)
            except RuntimeError:
                pass
        self.connected = False
        self.client.disconnect()
        self.client.loop_stop()


class PracticeController:
    def __init__(self, folder, manager):
        self.manager = manager
        self.source = 'field'
        self.busy = False
        self.error = ''
        self.link = None
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.game = PracticeGame(folder, self.publish, self.ready, self.halt)
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

    def publish(self, topic, payload):
        if self.source == 'practice' and self.link:
            self.link.publish(topic, payload)

    def ready(self):
        reasons = []
        if self.source != 'practice':
            reasons.append('Choose Practice and connect first')
        if self.busy:
            reasons.append('Preparing the practice bridge')
        if not self.link or not self.link.connected:
            reasons.append('Practice broker not connected')
        reasons.extend(self.manager.practice_readiness())
        return reasons

    def halt(self, reason):
        # Never block the game lock/MQTT callback while cancelling a ROS action.
        self.manager.practice_stopping = True
        threading.Thread(target=self.manager.halt_practice, args=(reason,), daemon=True).start()

    def loop(self):
        while not self.quit.wait(.2):
            if self.source == 'practice' and not self.busy:
                try:
                    if self.game.state in ACTIVE:
                        self.manager.send_practice_heartbeat()
                    self.game.tick()
                except Exception as e:
                    self.error = f'Practice loop failed: {e}'
                    # Halt even if persistence or transport itself failed.
                    self.game.state = 'error'
                    self.game.message = self.error
                    self.halt(self.error)
                    self.quit.set()

    def background(self, action):
        if self.busy:
            raise ValueError('Wait for the current connection operation')
        self.busy, self.error = True, ''
        def run():
            try:
                action()
            except Exception as e:
                self.error = str(e)
                self.manager.profiles['challenge_nav'].stop()
                if self.link:
                    self.link.close()
                    self.link = None
            finally:
                self.busy = False
        threading.Thread(target=run, daemon=True).start()

    def select(self, source):
        with self.lock:
            if source not in ('practice', 'field'):
                raise ValueError('Source must be practice or field')
            if self.game.state in ACTIVE:
                raise ValueError('Stop the game before changing refbox source')
            if self.quit.is_set():
                raise ValueError('Practice worker failed; restart web manager before reconnecting')
            def connect():
                # Stop the previous subscriber before switching its broker address.
                self.manager.profiles['challenge_nav'].stop()
                if self.link:
                    self.link.close()
                    self.link = None
                self.source = source
                self.manager.clear_practice_feedback()
                if source == 'practice':
                    if self.manager.node.estop:
                        raise ValueError('Release STOP before connecting practice')
                    running = self.manager.profiles['robot'].info()['state'] == 'running'
                    nav = any(self.manager.profiles[n].info()['state'] == 'running' for n in ('navigation', 'navigation_omni', 'navigation_softstop'))
                    if not running or not nav:
                        raise ValueError('Start Robot and a Navigation profile on Launch first')
                    broker = self.manager.profiles['practice_broker']
                    broker.start()
                    end = time.monotonic()+3
                    while True:
                        try:
                            if broker.proc.poll() is not None:
                                raise ValueError('Local practice broker did not start; check its Launch log')
                            self.link = PracticeMQTT(self.game)
                            if broker.proc.poll() is not None:
                                self.link.close()
                                self.link = None
                                raise ValueError('Practice broker exited; refusing to use another broker')
                            break
                        except OSError:
                            if time.monotonic()>=end:
                                raise ValueError('Cannot start local MQTT broker; check mosquitto installation') from None
                            time.sleep(.1)
                    self.game.pose_at = None
                    self.game.tick()  # send only WAIT_START/PAUSED, never RUNNING
                    self.manager.start('challenge_nav')
                else:
                    self.manager.profiles['practice_broker'].stop()
                    # Field mode uses the original configured remote broker and manual Launch.
            self.background(connect)
        return self.status()

    def prepare(self, body):
        with self.lock:
            if self.busy or self.manager.practice_stopping:
                raise ValueError('Wait for connection setup or navigation stop')
            result = self.game.prepare(body)
            # Every new round needs a fresh NavigationTask: DONE/red-zone/visited are per game.
            if self.source == 'practice' and self.link and self.link.connected:
                def rearm():
                    self.manager.profiles['challenge_nav'].stop()
                    self.game.pose_at = None
                    self.manager.clear_practice_feedback()
                    self.manager.start('challenge_nav')
                self.background(rearm)
            return result

    def start(self, body):
        with self.lock:
            self.game.start(body.get('layout_id'), body.get('confirmed', False))
            return self.status()

    def stop(self):
        with self.lock:
            if self.source != 'practice':
                raise ValueError('Use the field refbox controls in Field mode')
            self.game.stop()
            return self.status()

    def status(self):
        from ired_challenge.exploration import allowed_rotations
        from ired_web.practice import KINDS, ZONES
        rules = {name: {z: allowed_rotations(z, name) for z in ZONES if allowed_rotations(z, name)}
                 for name in KINDS[4]}
        return dict(self.game.snapshot(), source=self.source, busy=self.busy,
                    error=self.error, ready_reasons=self.game.start_problems() if self.game.state == 'prepared' else self.ready(),
                    broker_connected=bool(self.link and self.link.connected),
                    layouts=self.game.saved(), history=self.game.history(), placement_rules=rules)

    def close(self):
        self.quit.set()
        self.worker.join(timeout=2)
        if self.game.state in ACTIVE:
            self.game.stop('Web manager shutting down', error=True)
        if self.link:
            self.link.close()
            self.link = None
