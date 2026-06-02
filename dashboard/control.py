"""Control state for browser-driven follower operation.

Pure state + policy. No serial I/O — the main teleop loop drives all
actual servo writes. next_follower_targets() decides what targets to
write this iteration; consume_force_release() signals that a watchdog
event requires the main loop to blip torque and re-seat last_written
from current physical positions.
"""

import json
import threading
import time
from pathlib import Path

HEARTBEAT_TIMEOUT_S = 1.5
MAX_DELTA_TICKS = 100  # per joint, per next_follower_targets() call

_CALIBRATION_PATH = Path(__file__).resolve().parent.parent / "kinematics_calibration.json"

_lock = threading.Lock()
_mode = "physical"
_sim_targets = None
_last_heartbeat = None
_last_written = None
_force_release_pending = False
_joint_limits = None


def init(initial_targets):
    """Seed last_written from current follower positions and cache joint
    limits from kinematics_calibration.json. Safe to call multiple times."""
    global _last_written, _joint_limits, _mode, _sim_targets
    global _last_heartbeat, _force_release_pending
    with _lock:
        _last_written = list(initial_targets)
        _mode = "physical"
        _sim_targets = None
        _last_heartbeat = None
        _force_release_pending = False
        try:
            cal = json.loads(_CALIBRATION_PATH.read_text())
            _joint_limits = [(j["tick_min"], j["tick_max"])
                             for j in cal["joints"]]
        except Exception as e:
            print(f"[control] calibration load failed ({e}); using wide defaults")
            _joint_limits = [(0, 4095)] * 6


def engage():
    """Switch to sim mode. Returns (ok, message)."""
    global _mode, _sim_targets, _last_heartbeat
    with _lock:
        if _mode == "sim":
            return False, "already engaged"
        _mode = "sim"
        _sim_targets = list(_last_written)
        _last_heartbeat = time.time()
        return True, "engaged"


def release():
    """Switch back to physical. Always succeeds. Does not clear the
    force-release flag — if the watchdog set it, the main loop still
    needs to act."""
    global _mode, _sim_targets, _last_heartbeat
    with _lock:
        _mode = "physical"
        _sim_targets = None
        _last_heartbeat = None


def set_target(joints):
    """Update sim_targets. Clips each value to its joint's tick limits."""
    global _sim_targets
    with _lock:
        if _mode != "sim":
            return False, "not engaged"
        if len(joints) != 6:
            return False, "expected 6 joint values"
        clipped = []
        for i, t in enumerate(joints):
            lo, hi = _joint_limits[i]
            clipped.append(max(lo, min(hi, int(t))))
        _sim_targets = clipped
        return True, "ok"


def heartbeat():
    """Stamp last-seen-alive time."""
    global _last_heartbeat
    with _lock:
        _last_heartbeat = time.time()


def next_follower_targets(physical_targets, now):
    """Main-loop entry point. Returns the target list to write."""
    global _mode, _force_release_pending, _last_written
    with _lock:
        # 1. Heartbeat watchdog
        if (_mode == "sim"
                and _last_heartbeat is not None
                and now - _last_heartbeat > HEARTBEAT_TIMEOUT_S):
            _mode = "physical"
            _force_release_pending = True
            # Fall through into physical branch.

        # 2. Physical mode — pass through.
        if _mode == "physical":
            _last_written = list(physical_targets)
            return list(physical_targets)

        # 3. Sim mode — velocity-clamp against last_written.
        clamped = []
        for i in range(6):
            desired = _sim_targets[i]
            last = _last_written[i]
            delta = desired - last
            if delta >  MAX_DELTA_TICKS: delta =  MAX_DELTA_TICKS
            if delta < -MAX_DELTA_TICKS: delta = -MAX_DELTA_TICKS
            clamped.append(last + delta)
        _last_written = clamped
        return list(clamped)


def consume_force_release():
    """Atomically read-and-clear the force-release flag."""
    global _force_release_pending
    with _lock:
        flag = _force_release_pending
        _force_release_pending = False
        return flag


def snapshot():
    """Debug snapshot of internal state."""
    with _lock:
        return {
            "mode": _mode,
            "sim_targets": list(_sim_targets) if _sim_targets else None,
            "last_written": list(_last_written) if _last_written else None,
            "last_heartbeat": _last_heartbeat,
            "force_release_pending": _force_release_pending,
        }
