# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A minimal teleoperation setup for the SO-101 robot arm: a hand-moved **leader** arm drives a **follower** arm in real time over USB serial. Only three Python scripts, no build system, no tests, no package metadata — just `pyserial` against the Waveshare/ST3215 servo bus protocol.

## Running

Single runtime dependency: `pyserial`. Scripts are run directly with `python <script>.py`.

- `python follower_sees_follower_does.py` — main teleop loop. Both arms must be powered and connected. Prompts you to physically move both arms to the same starting pose, then ENTER to engage sync.
- `python scan_motors.py` — pings IDs 0–20 on `SERIAL_PORT` and reports which respond. Use to verify wiring or discover IDs. Only one bus at a time (one port).
- `python set/set_motor_id.py <new_id>` — rewrites a single motor's ID (and burns the lock). Only one motor may be on the bus during this; otherwise the broadcast-style write hits all of them. README's example path (`python set_motor_id.py`) omits the `set/` directory — the script actually lives there.

## Per-machine configuration (edit before running)

Every script has hardcoded constants at the top that must match the local rig — there is no config file or CLI flag for these:

- **COM ports**: `LEADER_PORT`/`FOLLOWER_PORT` in the main script, `SERIAL_PORT` in the others. Change to whatever Windows assigned (e.g. `COM6`, `COM8`).
- **Motor IDs**: `LEADER_IDS` and `FOLLOWER_IDS` in `follower_sees_follower_does.py` are ordered to match the joint order from the [SO-101 assembly guide](https://huggingface.co/docs/lerobot/so101) (shoulder → gripper). The current values reflect this lab's wiring: leader = 7–12, follower = 1–4,6,5 (motors 5 and 6 are physically swapped). Reorder the list — don't reflash IDs — to adapt to a different rig.
- **`DIRECTIONS`**: per-joint sign (`+1`/`-1`). Flip an entry if a follower joint mirrors the leader instead of tracking it.
- `BAUDRATE` is `1000000` everywhere; don't change unless the servos were reflashed.

## Protocol & architecture notes

The scripts speak the ST3215/Waveshare serial protocol by hand — no SDK. Three things to know before editing the bus code:

1. **Packet shape**: `[0xFF, 0xFF, ID, length, instruction, ...params, checksum]` where `checksum = (~sum(packet[2:])) & 0xFF`. Instructions used here: `0x01` PING, `0x02` READ, `0x03` WRITE, `0x83` SYNC WRITE. Key register addresses: `40` torque enable, `42` goal position, `56` present position, `48`/`55` EEPROM lock, `5` ID.
2. **Why the main loop is structured the way it is**: reads are per-motor (one round-trip each, `timeout=0.01`, single-attempt — dropped frames are tolerated to keep loop rate up); writes use **one** sync-write packet (`0xFE` broadcast, instruction `0x83`) to move all six follower motors atomically. Don't replace sync-write with per-motor writes — the loop will visibly lag.
3. **Delta-following, not absolute mirroring**: the loop tracks each leader's `present_position` *change* between iterations and adds it (with wrap-around at 4096 ticks/rev) to the follower's accumulated target. That's why both arms must be moved to the same starting pose before ENTER — the follower's starting absolute position is the anchor, and only deltas are propagated. Targets can grow beyond `[0, 4095]` (multi-turn); the writer remaps negatives via `pos += 65536`.

`Ctrl+C` is the intended exit — the handler disables follower torque and closes both ports. There is no graceful shutdown signal otherwise.
