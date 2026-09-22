"""
Real-hardware acceptance test for the play() thread-safety fix.

This runs against an actual dorna2 controller — NOT a mock. It is the
exact failure the 2026-09-21 incident traced to:

    Thread A: output(config=[[5, 1, 0.5], [5, 0, 0.5]]) — shaker
    Thread B: jmove(...)  — 2-3 s motions

Before 2.1.6, the two threads shared a single tracking slot; whichever
play() finished second would return an empty union (stat missing) and
get_all_output() would return a non-list. The three journals in
core/robot_log.jsonl.old on the bna project show this.

After the fix, three invariants must hold on the wire:

  (1) Every motion play() blocks for at least its motion time.
  (2) Every output row returns stat 2 (accepted).
  (3) get_all_output() polled from either thread never returns a
      non-list.

Usage:
    python tests/acceptance_two_threads_wire.py --host 192.168.1.100 \\
        --duration 60

Exit code is 0 on pass, 1 on fail; a summary is printed on stderr.
"""

import argparse
import sys
import threading
import time
import traceback

from dorna2 import Dorna


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=443)
    ap.add_argument("--duration", type=float, default=60.0,
                    help="how long to run the two threads, in seconds")
    ap.add_argument("--pin", type=int, default=5,
                    help="output pin used by the shaker thread")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="seconds between toggles")
    ap.add_argument("--motion-joint", type=int, default=0,
                    help="joint index the motion thread wiggles")
    ap.add_argument("--motion-amp", type=float, default=5.0,
                    help="degrees the motion thread wiggles")
    ap.add_argument("--motion-vel", type=float, default=25.0)
    ap.add_argument("--motion-accel", type=float, default=500.0)
    ap.add_argument("--motion-jerk", type=float, default=2500.0)
    ap.add_argument("--min-motion-time", type=float, default=0.3,
                    help="a motion play() must block at least this long")
    args = ap.parse_args()

    robot = Dorna()
    if not robot.connect(host=args.host, port=args.port):
        print("connect failed", file=sys.stderr)
        return 2

    # observations
    io_ok = 0
    io_bad = []            # (i, rtn) for rows that did not return stat 2
    motion_ok = 0
    motion_bad = []        # (i, elapsed, rtn)
    outputs_polled_ok = 0
    outputs_polled_bad = []  # (i, value)
    thread_excs = []
    stop = threading.Event()
    t_start = time.time()

    def io_loop():
        i = 0
        try:
            while not stop.is_set():
                # each config sends two rows with a Python-side settle;
                # each row is a play() that blocks on its own ack
                rtn = robot.output(config=[
                    [args.pin, 1, args.settle],
                    [args.pin, 0, args.settle],
                ])
                nonlocal io_ok
                for row_rtn in (rtn,):
                    if row_rtn is None:
                        io_bad.append((i, None))
                    else:
                        try:
                            if row_rtn["union"]["stat"] == 2:
                                io_ok += 1
                            else:
                                io_bad.append((i, row_rtn))
                        except Exception:
                            io_bad.append((i, row_rtn))
                i += 1
        except Exception:
            thread_excs.append(("io", traceback.format_exc()))

    def motion_loop():
        i = 0
        try:
            base = robot.get_joint(index=args.motion_joint)
            while not stop.is_set():
                target = base + (args.motion_amp if (i % 2 == 0) else -args.motion_amp)
                t0 = time.time()
                rtn = robot.jmove(
                    joint=[target if k == args.motion_joint else robot.get_joint(index=k) for k in range(6)],
                    vel=args.motion_vel, accel=args.motion_accel, jerk=args.motion_jerk,
                )
                elapsed = time.time() - t0
                nonlocal motion_ok
                if isinstance(rtn, int) and rtn >= 2 and elapsed >= args.min_motion_time:
                    motion_ok += 1
                else:
                    motion_bad.append((i, elapsed, rtn))
                i += 1
        except Exception:
            thread_excs.append(("motion", traceback.format_exc()))

    def poll_loop():
        i = 0
        try:
            while not stop.is_set():
                vals = robot.get_all_output()
                nonlocal outputs_polled_ok
                if isinstance(vals, list):
                    outputs_polled_ok += 1
                else:
                    outputs_polled_bad.append((i, vals))
                time.sleep(0.05)
                i += 1
        except Exception:
            thread_excs.append(("poll", traceback.format_exc()))

    threads = [
        threading.Thread(target=io_loop, name="io"),
        threading.Thread(target=motion_loop, name="motion"),
        threading.Thread(target=poll_loop, name="poll"),
    ]
    for t in threads:
        t.start()

    try:
        while time.time() - t_start < args.duration and not thread_excs:
            time.sleep(0.5)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=10)
        try:
            robot.halt()
        except Exception:
            pass
        try:
            robot.close()
        except Exception:
            pass

    # verdict
    print("---- summary ----", file=sys.stderr)
    print("duration:                %.1f s" % (time.time() - t_start), file=sys.stderr)
    print("io rows stat 2:          %d" % io_ok, file=sys.stderr)
    print("io rows bad:             %d" % len(io_bad), file=sys.stderr)
    print("motions ok (>= %.2fs):   %d" % (args.min_motion_time, motion_ok), file=sys.stderr)
    print("motions bad:             %d" % len(motion_bad), file=sys.stderr)
    print("get_all_output list:     %d" % outputs_polled_ok, file=sys.stderr)
    print("get_all_output non-list: %d" % len(outputs_polled_bad), file=sys.stderr)
    print("thread exceptions:       %d" % len(thread_excs), file=sys.stderr)

    if thread_excs:
        for name, tb in thread_excs:
            print("---- exception in %s ----" % name, file=sys.stderr)
            print(tb, file=sys.stderr)

    if io_bad or motion_bad or outputs_polled_bad or thread_excs:
        print("FAIL", file=sys.stderr)
        return 1
    print("PASS", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
