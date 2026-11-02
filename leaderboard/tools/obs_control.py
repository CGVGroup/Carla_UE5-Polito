#!/usr/bin/env python3
"""
obs_control.py: starts / stops the OBS recording through OBS's built-in remote control (obs-websocket,
OBS 28+), so that run_leaderboard_xai.ps1 records the headset view by itself.

Setup once: in OBS, Tools > WebSocket Server Settings: enable the server (port 4455); set a password or
not (if set, put it in the OBS_PASSWORD environment variable); then:
    pip install obs-websocket-py        (Python 3.8; or obsws-python on Python 3.9+)

Usage:
    python tools/obs_control.py start
    python tools/obs_control.py stop      # prints the recorded file
Exit code 0 on success, 1 when OBS can't be controlled (not running, websocket off, package missing): the
recording then has to be started / stopped by hand.
"""
import os
import sys
import time

HOST = os.environ.get('OBS_HOST', 'localhost')
PORT = int(os.environ.get('OBS_PORT', 4455))
PASSWORD = os.environ.get('OBS_PASSWORD', '')


class _ObsWebsocketPy:
    """obs-websocket-py (works on Python 3.8)"""

    def __init__(self):
        from obswebsocket import obsws, requests
        self._requests = requests
        self._ws = obsws(HOST, PORT, PASSWORD)
        self._ws.connect()

    def recording(self):
        return bool(self._ws.call(self._requests.GetRecordStatus()).datain.get('outputActive'))

    def start(self):
        self._ws.call(self._requests.StartRecord())

    def stop(self):
        return self._ws.call(self._requests.StopRecord()).datain.get('outputPath', '')

    def close(self):
        self._ws.disconnect()


class _ObswsPython:
    """obsws-python (Python 3.9+)"""

    def __init__(self):
        import obsws_python
        self._client = obsws_python.ReqClient(host=HOST, port=PORT, password=PASSWORD, timeout=3)

    def recording(self):
        return bool(self._client.get_record_status().output_active)

    def start(self):
        self._client.start_record()

    def stop(self):
        return getattr(self._client.stop_record(), 'output_path', '')

    def close(self):
        pass


def connect():
    """A connected OBS client, or (None, reason)"""
    reasons = []
    for client_class in (_ObsWebsocketPy, _ObswsPython):
        try:
            return client_class(), None
        except ImportError:
            reasons.append('{} not installed'.format(client_class.__doc__.split(' (')[0]))
        except Exception as e:  # noqa: BLE001 - OBS not running, websocket disabled, wrong password
            return None, 'OBS not reachable on {}:{} ({}: {})'.format(HOST, PORT, type(e).__name__, e)
    return None, ', '.join(reasons) + ' (pip install obs-websocket-py)'


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('start', 'stop'):
        print(__doc__)
        return 2
    client, reason = connect()
    if client is None:
        print('OBS remote control unavailable: {}. Start / stop the recording by hand'.format(reason))
        return 1
    try:
        recording = client.recording()
        if sys.argv[1] == 'start':
            if recording:
                # Still recording (a run interrupted before its stop, or started by hand): a new file for this
                # run, otherwise the route ends up minutes into an older recording
                print('OBS was already recording: stopped ({}), started again for this run'.format(client.stop()))
                for _ in range(50):
                    time.sleep(0.1)
                    if not client.recording():
                        break
            client.start()
            print('OBS recording started')
        else:
            if not recording:
                print('OBS was not recording')
            else:
                print('OBS recording stopped: {}'.format(client.stop()))
        return 0
    except Exception as e:  # noqa: BLE001
        print('OBS request failed ({}): start / stop the recording by hand'.format(e))
        return 1
    finally:
        try:
            client.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
