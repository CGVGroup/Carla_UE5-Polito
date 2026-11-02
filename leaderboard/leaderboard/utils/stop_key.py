"""
Operator's stop key: Ctrl+Shift+Q, at any time of the run, whichever window has the focus (CARLA's
included), ends the run cleanly: the current route stops as if it had ended (the session logger writes its
summary, the CARLA recording is closed), the next routes are not started, and run_leaderboard_xai.ps1 goes
on with its post-run steps (OBS stopped, recording moved, commands.txt written).

Unlike Ctrl+C in the terminal, which also stops the PowerShell script before those steps.
Windows only (the key is read with GetAsyncKeyState); elsewhere the watcher does nothing.
"""

import sys
import threading
import time

STOP_KEY_NAME = 'Ctrl+Shift+Q'

_VK_SHIFT = 0x10
_VK_CONTROL = 0x11
_VK_Q = 0x51


class StopKey(object):
    """Calls on_stop() once, from its own thread, when the stop key is pressed"""

    def __init__(self, on_stop, poll_period=0.1):
        self._on_stop = on_stop
        self._poll_period = poll_period
        self.pressed = False
        self._thread = None
        try:
            import ctypes
            self._get_key = ctypes.windll.user32.GetAsyncKeyState if sys.platform == 'win32' else None
        except (ImportError, AttributeError):
            self._get_key = None

    @property
    def available(self):
        return self._get_key is not None

    def start(self):
        if not self.available or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._watch, name='StopKey', daemon=True)
        self._thread.start()

    def _down(self, key):
        return bool(self._get_key(key) & 0x8000)

    def _watch(self):
        while not self.pressed:
            if self._down(_VK_CONTROL) and self._down(_VK_SHIFT) and self._down(_VK_Q):
                self.pressed = True
                try:
                    self._on_stop()
                except Exception as e:  # never let the watcher take the run down
                    print('Stop key: {}'.format(e))
                return
            time.sleep(self._poll_period)
