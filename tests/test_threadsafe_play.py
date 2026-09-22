"""
Concurrent play() test.

Two threads share one Dorna client:
  * IO thread loops output(config=[[pin, val, 0.05]]) — shaker-style
  * Motion thread loops jmove(joint=[...])

Under the pre-2.1.6 single-slot design, each play() overwrote a shared
tracking slot, and the shaker's play() would return an empty union
whenever a motion's play() started/finished in between. This test asserts
that every play() returns a union with the matching id and a terminal
stat, and that the IO settle actually blocks the calling thread for the
requested time — the exact regressions the fix targets.

Runs standalone: python tests/test_threadsafe_play.py
"""

import json
import os
import queue
import sys
import threading
import time

# make the local checkout importable without an install
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from dorna2.dorna import Dorna
from dorna2.ws import WS


class FakeController:
    """Consumes JSON frames handed to write() and emits replies the way
    the real read_loop would — but injects them straight into _tracks
    (skipping the socket). A motion takes ~motion_delay seconds; an
    output/other takes ~io_delay seconds. Each frame produces one
    stat=0 followed by a stat=2 reply for its id."""

    _MOTION_CMDS = {"jmove", "lmove", "cmove", "smove", "cjmove", "clmove", "rmove"}

    def __init__(self, dorna, motion_delay=0.02, io_delay=0.0):
        self.dorna = dorna
        self.motion_delay = motion_delay
        self.io_delay = io_delay
        self._q = queue.Queue()
        self._alive = True
        # a worker per pending frame keeps IO and motion latencies
        # independent — otherwise a motion queued ahead of an IO would
        # serialise them and hide the exact race we want to exercise.
        self._workers = []
        threading.Thread(target=self._dispatch, daemon=True).start()

    def stop(self):
        self._alive = False
        self._q.put(None)

    def submit(self, frame_json):
        self._q.put(frame_json)

    def _post_reply(self, msg):
        with self.dorna._tracks_lock:
            entry = self.dorna._tracks.get(msg.get("id"))
        if entry is None:
            return
        entry["msgs"].append(msg)
        stat = msg.get("stat", 0)
        if stat < 0 or stat >= 2:
            entry["event"].set()

    def _handle_one(self, frame_json):
        try:
            cmd = json.loads(frame_json)
        except Exception:
            return
        cid = cmd.get("id")
        if cid is None:
            return
        base = dict(cmd)
        base["stat"] = 0
        self._post_reply(base)
        delay = self.motion_delay if cmd.get("cmd") in self._MOTION_CMDS else self.io_delay
        if delay:
            time.sleep(delay)
        done = dict(base)
        done["stat"] = 2
        self._post_reply(done)

    def _dispatch(self):
        while self._alive:
            frame = self._q.get()
            if frame is None:
                return
            t = threading.Thread(target=self._handle_one, args=(frame,), daemon=True)
            t.start()


def _make_offline_dorna():
    """Build a Dorna without touching the network. WS.__init__ is
    called directly, and we swap write() to route into the fake
    controller instead of the (unopened) asyncio writer."""
    d = Dorna.__new__(Dorna)
    WS.__init__(d)
    d.config = {
        "cmd_init": [],
        "limit": {"dorna_ta": []},
        "speed": {"very_quick": {
            "jmove": {"vel": 100, "accel": 500, "jerk": 2000},
            "lmove": {"vel": 100, "accel": 500, "jerk": 2000},
        }},
    }
    d.logger = None
    d.model = "dorna_ta"
    return d


def test_concurrent_play_no_slot_collision():
    d = _make_offline_dorna()
    ctrl = FakeController(d, motion_delay=0.02, io_delay=0.001)
    d.write = lambda msg="", mode="cmd": ctrl.submit(msg)

    errors = []
    motion_results = []
    io_results = []

    N_MOTIONS = 40
    N_TOGGLES = 40

    def motion_loop():
        for i in range(N_MOTIONS):
            rtn = d.play(cmd="jmove", j0=float(i), j1=0.0)
            try:
                cid = rtn["cmd"]["id"]
                stat = rtn["union"]["stat"]
                if rtn["union"].get("id") != cid:
                    errors.append(("motion id mismatch", rtn))
                elif stat < 0 or stat >= 2:
                    motion_results.append((cid, stat))
                else:
                    errors.append(("motion non-terminal stat", rtn))
            except Exception as e:
                errors.append(("motion exc", e, rtn))

    def io_loop():
        for i in range(N_TOGGLES):
            rtn = d.output(config=[[5, i % 2, 0.005]])
            try:
                cid = rtn["cmd"]["id"]
                stat = rtn["union"]["stat"]
                if rtn["union"].get("id") != cid:
                    errors.append(("io id mismatch", rtn))
                elif stat < 0 or stat >= 2:
                    io_results.append((cid, stat))
                else:
                    errors.append(("io non-terminal stat", rtn))
            except Exception as e:
                errors.append(("io exc", e, rtn))

    threads = [threading.Thread(target=motion_loop),
               threading.Thread(target=io_loop)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)

    ctrl.stop()

    assert not errors, "unexpected errors: %r" % (errors,)
    assert len(motion_results) == N_MOTIONS, (len(motion_results), N_MOTIONS)
    assert len(io_results) == N_TOGGLES, (len(io_results), N_TOGGLES)


def test_output_config_settle_uses_python_sleep():
    """The Python-side settle must block the calling thread for
    ~settle seconds, regardless of what the controller queue is
    doing. Slow the fake controller's reply to zero so any waiting
    time is purely from time.sleep(c[2])."""
    d = _make_offline_dorna()
    ctrl = FakeController(d, motion_delay=0.0, io_delay=0.0)
    d.write = lambda msg="", mode="cmd": ctrl.submit(msg)

    t0 = time.time()
    d.output(config=[[5, 1, 0.10], [5, 0, 0.10]])
    elapsed = time.time() - t0

    ctrl.stop()

    assert 0.18 <= elapsed <= 0.6, "expected ~0.20 s settle, got %.3f s" % elapsed


def test_get_last_alarm_and_callback():
    d = _make_offline_dorna()
    received = []
    d.register_alarm_callback(lambda a: received.append(a))

    # Simulate an alarm broadcast by invoking the same code path the
    # read loop uses — the callback list + _last_alarm are the public
    # observations we care about.
    with d._alarm_lock:
        d._last_alarm = {"time": time.time(), "msg": {"cmd": "alarm", "alarm": 1, "err0": 3}}
        callbacks = list(d._alarm_callbacks)
    for fn in callbacks:
        fn(d._last_alarm)

    assert d.get_last_alarm()["msg"]["err0"] == 3
    assert received and received[0]["msg"]["alarm"] == 1

    d.clear_last_alarm()
    assert d.get_last_alarm() is None


if __name__ == "__main__":
    test_concurrent_play_no_slot_collision()
    print("test_concurrent_play_no_slot_collision: ok")
    test_output_config_settle_uses_python_sleep()
    print("test_output_config_settle_uses_python_sleep: ok")
    test_get_last_alarm_and_callback()
    print("test_get_last_alarm_and_callback: ok")
