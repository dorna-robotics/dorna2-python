# Changelog

## 2.1.7

Follow-up to 2.1.6: remove every remaining shared-state path to a
command's stat. There is now **one** way to read a stat — the value
returned by `play()` (or the `set_*` / `_motion` wrappers around it).
No hidden per-thread stash, no best-effort shim, no cross-caller state.

### Removed
- `_track_cmd_stat()` — every `set_*` wrapper now returns
  `self._stat_cmd(cmd, **kwargs)` directly.
- `track_cmd()` — external consumers of the old shim should use
  `play()`'s return value.
- `WS._track` (single-slot shim), `WS._local` (threading.local rtn
  stash). Both existed only to feed the two methods above.

### Added
- `_stat_cmd(cmd, **kwargs)` — sends a command via `play()` and
  returns `rtn["union"]["stat"]` (or `False` if the reply had no
  stat). Every `set_*` method now uses it.
- `play()`'s `_tracks` registration now also purges any entries whose
  `Event` fired but whose owner never popped them (e.g. a caller using
  `timeout=0` fire-and-forget). Never touches an entry with a live
  waiter. The 256 cap now truly means "live waiters"; hitting it is
  the signal that something is piling up.
- `tests/acceptance_two_threads_wire.py` — real-hardware acceptance
  test that runs against an actual controller. Two threads
  (shaker-style `output(config=...)` + jmove wiggle) plus a
  `get_all_output()` poller, asserting on the wire that every motion
  play() blocks for at least its motion time, every output row
  returns stat 2, and `get_all_output()` never returns a non-list —
  the exact three invariants the 2026-09-21 incident violated.

### Preserved (called out for reviewers)
- `set_freq` / `set_duty` keep the pre-existing (buggy) wire shape:
  `freq` was sent as a bare `"freq"` key rather than `"freq{index}"`,
  and `set_duty` sends `"duty": index` (a copy-paste). The refactor
  does not silently repair these — they are noted in the source so a
  proper fix can be made in scope of its own change.

### Compatibility
- `play()`'s return shape is unchanged: `{"msgs":[...], "cmd":{...},
  "union":{...}}`. Anything that consumes it (including `_motion`,
  `_key_val_cmd`, and now `_stat_cmd`) still sees the same structure.
- `set_*` return types are unchanged. Only the path is different.

## 2.1.6

### Thread-safety
- `play()` now tracks each command by its own id (`_tracks` dict, per-id
  `threading.Event`) instead of a shared single slot. Concurrent `play()`
  calls from different threads no longer overwrite each other's tracking
  slot, and each call returns a union containing only the replies for
  its own id.
- `_track_cmd_stat()` reads from a `threading.local` populated by
  `play()`, so `set_output()` / `set_joint()` / other setters return the
  stat of their own call rather than whichever play happened to complete
  most recently.
- `track_cmd()` and `last_cmd()` remain functional but are now
  documented as "most recently completed play" — best-effort, not
  meaningful across threads. External consumers should prefer the value
  returned by `play()` directly.
- `wait()` is serialised by a lock (fixes the shared `_ptrn` slot) and
  the `self.ptrn` typo that broke the `timeout<0` path.
- `_tracks` has a soft cap of 256 live entries. Exceeding it logs a
  one-time warning per over-cap episode; no live waiter is ever evicted.

### Priority of IO rows — behavior change
- `output(config=[[pin, val, settle], ...])` now sends each row with
  `"queue": 1` (high-priority) and sleeps in Python for `settle`
  seconds, so device IO does not wait behind motions in the normal
  queue and the timing follows the hardware rather than the controller
  backlog. This changes timing relative to queued motions — an output
  toggle sent during a motion now fires immediately instead of after
  the motion completes.
- The same change is applied to the `pick_n_place` pick/place output
  sections for consistency (they were not on any known caller's path
  but should not diverge).

### Homing
- `home_with_stop()` guards against `get_pid()` returning `False` under
  alarm (previously raised `TypeError: 'bool' object is not
  subscriptable` on `pid_init[0]`). It now logs and returns `False`, so
  a launch with the alarm still set fails as "not homed" instead of
  aborting the run.

### New — alarm surface
- `get_last_alarm()` returns the last `{"cmd":"alarm", ...}` broadcast
  as `{"time": ts, "msg": {...}}`, or `None` if cleared.
- `clear_last_alarm()` clears it.
- `register_alarm_callback(fn)` / `deregister_alarm_callback(fn)` —
  `fn(alarm_dict)` fires from the read loop when an alarm arrives. The
  callback must not block (runs on the asyncio thread); a raising
  callback is dropped, never the loop.

### Tests
- Added `tests/test_threadsafe_play.py` — two threads on one client
  (one loops `output(config=...)` with 50 ms settles, the other issues
  jmoves) — asserts every `play()` returns a union with its own id and
  a terminal stat. This exact pattern was the failure mode fixed above.
