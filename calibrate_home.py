"""One-time per-rig calibration: record the current follower servo
positions as the URDF "home pose" tick_zero values.

Usage:
  1. Stop any running follower_sees_follower_does.py.
  2. With the follower powered on, manually move the arm into the URDF
     home pose (all URDF joint angles = 0 -- see README for a description).
  3. Run:  python calibrate_home.py
  4. Review the diff printed; the file is updated in place.
  5. Restart the main script and verify the ghost arm matches the real
     pose; flip tick_sign by hand if a joint is mirrored.
"""

import ast
import json
import sys
import time
from pathlib import Path

import serial

_ROOT = Path(__file__).resolve().parent
_MAIN = _ROOT / "follower_sees_follower_does.py"
_CAL = _ROOT / "kinematics_calibration.json"


def _ast_to_value(node):
    """Recursively convert an AST literal subtree to a Python value.
    Accepts Constants, Lists, and Tuples only. Raises on anything else."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_ast_to_value(e) for e in node.elts]
    raise ValueError(f"unsupported AST node: {type(node).__name__}")


def _load_config():
    """Extract FOLLOWER_PORT, BAUDRATE, FOLLOWER_IDS from the main script
    by AST-walking it. This avoids running the module's top-level I/O."""
    tree = ast.parse(_MAIN.read_text())
    wanted = {"FOLLOWER_PORT", "BAUDRATE", "FOLLOWER_IDS"}
    vals = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            t = node.targets[0]
            if isinstance(t, ast.Name) and t.id in wanted:
                vals[t.id] = _ast_to_value(node.value)
    missing = wanted - vals.keys()
    if missing:
        raise SystemExit(f"Could not find {missing} in {_MAIN}")
    return vals["FOLLOWER_PORT"], vals["BAUDRATE"], vals["FOLLOWER_IDS"]


def _calculate_checksum(packet):
    return (~sum(packet[2:])) & 0xFF


def _read_position(ser, servo_id):
    ser.reset_input_buffer()
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, 56, 2]
    packet.append(_calculate_checksum(packet))
    ser.write(bytearray(packet))
    time.sleep(0.01)
    r = ser.read(8)
    if len(r) == 8 and r[0] == 0xFF:
        return ((r[6] << 8) | r[5]) % 4096
    return None


def main():
    port, baud, follower_ids = _load_config()
    print(f"Opening {port} @ {baud}, IDs {follower_ids}")
    ser = serial.Serial(port, baud, timeout=0.05)
    positions = []
    for sid in follower_ids:
        p = None
        for _ in range(5):
            p = _read_position(ser, sid)
            if p is not None:
                break
        if p is None:
            print(f"  ID {sid}: FAILED to read (retry or check wiring)")
            sys.exit(1)
        print(f"  ID {sid}: {p}")
        positions.append(p)
    ser.close()

    cal = json.loads(_CAL.read_text())
    assert len(cal["joints"]) == len(positions), (
        f"calibration has {len(cal['joints'])} joints, got {len(positions)} positions")
    print("\nUpdating kinematics_calibration.json -- diffs:")
    for j, new_zero in zip(cal["joints"], positions):
        old = j.get("tick_zero")
        j["tick_zero"] = new_zero
        delta = new_zero - old
        print(f"  {j['name']:<14} (id {j['id']}): {old} -> {new_zero}  (delta {delta:+d})")

    _CAL.write_text(json.dumps(cal, indent=2) + "\n")
    print("\nDone. Restart follower_sees_follower_does.py.")


if __name__ == "__main__":
    main()
