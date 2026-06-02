"""Shared, lock-guarded telemetry state consumed by the dashboard HTTP thread.

All public functions are thread-safe. The lock is held only for dict/deque
operations — never across I/O. The main (teleop) thread calls update();
the HTTP thread calls snapshot().
"""

import threading
import time
from collections import deque

HISTORY_LEN = 60  # ~60 seconds at 1 Hz UI poll

_lock = threading.Lock()
_state = {}    # motor_id -> dict of current values
_history = {}  # motor_id -> {"temp": deque, "load": deque}
_roles = {}    # motor_id -> "leader" | "follower"
_order = []    # stable display order: leader IDs first, then follower IDs


def init(leader_ids, follower_ids):
    """Seed state with one entry per motor. Safe to call multiple times."""
    global _state, _history, _roles, _order
    with _lock:
        _state = {}
        _history = {}
        _roles = {}
        _order = list(leader_ids) + list(follower_ids)
        for mid in leader_ids:
            _state[mid] = _empty_entry()
            _history[mid] = {"temp": deque(maxlen=HISTORY_LEN),
                             "load": deque(maxlen=HISTORY_LEN)}
            _roles[mid] = "leader"
        for mid in follower_ids:
            _state[mid] = _empty_entry()
            _history[mid] = {"temp": deque(maxlen=HISTORY_LEN),
                             "load": deque(maxlen=HISTORY_LEN)}
            _roles[mid] = "follower"


def update(motor_id, *, pos=None, goal=None, temp=None, volt=None,
           load=None, speed=None):
    """Merge supplied fields into the motor's state entry.

    Only non-None fields are written. temp/load are also appended to their
    ring buffers. Unknown motor_ids are silently ignored."""
    with _lock:
        entry = _state.get(motor_id)
        if entry is None:
            return
        if pos is not None:
            entry["pos"] = pos
        if goal is not None:
            entry["goal"] = goal
        if temp is not None:
            entry["temp"] = temp
            _history[motor_id]["temp"].append(temp)
        if volt is not None:
            entry["volt"] = volt
        if load is not None:
            entry["load"] = load
            _history[motor_id]["load"].append(load)
        if speed is not None:
            entry["speed"] = speed
        entry["updated_at"] = time.time()


def snapshot():
    """Return a JSON-serializable dict describing all motors right now."""
    with _lock:
        now = time.time()
        motors = []
        for mid in _order:
            entry = _state[mid]
            motors.append({
                "id": mid,
                "role": _roles[mid],
                "pos": entry["pos"],
                "goal": entry["goal"],
                "temp": entry["temp"],
                "volt": entry["volt"],
                "load": entry["load"],
                "speed": entry["speed"],
                "updated_at": entry["updated_at"],
                "temp_history": list(_history[mid]["temp"]),
                "load_history": list(_history[mid]["load"]),
            })
        return {"now": now, "history_len": HISTORY_LEN, "motors": motors}


def _empty_entry():
    return {"pos": None, "goal": None, "temp": None, "volt": None,
            "load": None, "speed": None, "updated_at": None}
