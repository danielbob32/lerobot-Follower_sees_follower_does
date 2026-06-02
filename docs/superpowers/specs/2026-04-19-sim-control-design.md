# Browser Sim & End-Effector Control — Design

**Date:** 2026-04-19
**Scope:** Extend the existing telemetry dashboard with a 3D simulation of the SO-101 and browser-driven control of the follower via an end-effector gizmo. After the normal physical sync, the operator clicks "Engage" in the browser to drive the follower by dragging the end-effector in 3D; clicking "Release" returns control to the physical leader.

## Goals

- Live 3D view of the arm that always mirrors the real follower (ghost, from telemetry).
- Mouse-optimized end-effector drag → IK → follower motion when control is engaged.
- 100% geometric fidelity to the actual SO-101 (URDF-driven, no hand-authored dimensions).
- Multiple dead-man safety layers: heartbeat watchdog, velocity clamp, URDF joint limits, "Release" button, two-stage Ctrl+C.
- Zero new Python dependencies. All browser assets vendored (offline-capable).

## Non-goals (YAGNI)

- Saved poses, motion recording, trajectory playback.
- Multi-viewport (top/side/front) or measurement overlay.
- Keyboard shortcuts.
- Joint-slider mode (direct per-joint drag) — wrist pitch/roll/gripper already bypass IK via side panel.
- Multi-user control arbitration beyond "first engaged wins, 409 for others".
- Mobile/touch layout.
- Any test suite.

## Architecture

**Browser side (new)**
- 3D panel added to `dashboard/static/index.html`, rendered with vendored three.js.
- Kinematic model and meshes sourced from the SO-101 URDF vendored into
  `dashboard/static/models/so101/`. URDF parsed in-browser by vendored `urdf-loader`.
- New `dashboard/static/sim.js`: scene setup, URDF load, ghost-vs-target arm, CCD IK, gizmo, camera, orientation/gripper side panel, engage/release button, heartbeat pump, API client.
- Per-rig tick calibration in `kinematics_calibration.json` at the repo root. `tick_zero` per joint + `tick_sign` (±1) + constant `tick_per_rad = 4096/(2π)`.

**Server side (extended)**
- New `dashboard/control.py`: control state + velocity clamp + heartbeat watchdog + single public function `next_follower_targets(physical_targets, now)`.
- Four new POST endpoints in `dashboard/server.py`: `/api/engage`, `/api/release`, `/api/target`, `/api/heartbeat`.

**Main loop (`follower_sees_follower_does.py`)**
- One-line change to routing: the sync-write target list flows through `control.next_follower_targets(...)`.
- Ctrl+C handler upgraded to two-stage signal handler (freeze on first, exit on second).

## File layout (deltas on top of the existing dashboard)

```
kinematics_calibration.json                   NEW  per-rig tick_zero / tick_sign
dashboard/
  control.py                                  NEW  mode/targets/heartbeat state
  server.py                                   MOD  add POST routes
  static/
    index.html                                MOD  add 3D panel + side panel
    sim.js                                    NEW  three.js scene, IK, gizmo, API
    vendor/
      three.min.js                            NEW  vendored ~600 KB
      OrbitControls.js                        NEW  vendored
      STLLoader.js                            NEW  vendored
      urdf-loader.js                          NEW  vendored
    models/so101/
      so101.urdf                              NEW  vendored from lerobot
      meshes/*.stl                            NEW  vendored from lerobot
follower_sees_follower_does.py                MOD  signal handler + routing line
```

No new Python dependencies. All browser assets vendored; zero network at runtime.

## Kinematics & IK

**Model source.** The canonical SO-101 URDF and STL meshes are copied from the
`huggingface/lerobot` repository (Apache 2.0). The URDF is the single source of geometric truth — link origin transforms, joint axes, joint limits, mesh references. No hand-authored dimensions anywhere. Python never parses the URDF; only the browser does.

**Tick calibration.** URDF speaks radians; servos speak ticks. `kinematics_calibration.json` carries everything Python needs about each joint:

```json
{
  "tick_per_rad": 651.9,
  "joints": [
    {"id": 1, "tick_zero": 2048, "tick_sign":  1, "tick_min": 512, "tick_max": 3584},
    ...
  ]
}
```

- `tick_zero` — tick value when the URDF joint angle = 0.
- `tick_sign` — ±1, captures servo mounting orientation (supersedes the current `DIRECTIONS` array).
- `tick_min` / `tick_max` — the URDF's joint limits converted to ticks once, at file-generation time. Server uses these for its belt-and-braces clip on incoming `/api/target` values, keeping Python URDF-free.
- `tick_per_rad` — constant 4096/(2π) ≈ 651.9, kept in the file for transparency.

A committed default matches a rig assembled per the SO-101 assembly guide. The implementation plan includes a `calibrate_home.py` script that overwrites `tick_zero` entries by recording current servo positions when the arm is in the URDF home pose.

**IK algorithm.** Cyclic Coordinate Descent (CCD). Iterative, per-joint, wrist-to-base sweep. ~50 iterations per animation frame; converges for small inter-frame deltas. Joint limits enforced per iteration by clip-and-reproject against the URDF limits. Residual distance threshold defines "unreachable" (target handle turns red; arm goes to best-effort pose). Zero external math dependencies — three.js `Vector3` and `Quaternion` suffice.

**Rendering.**
- **Ghost arm** (semi-transparent): driven by live telemetry (`/api/state.motors[*].pos`), updated at 1 Hz with the existing dashboard poll. Represents where the follower actually is.
- **Target arm** (solid): driven by IK from the gizmo handle. Hidden until first `Engage`; on engage, initialized from the ghost pose so there's no visual jump.
- Both share the same URDF-loaded scene graph (one model instance per arm).

## Control path

**Endpoints (all POST, loopback-only, no auth):**

| Route | Body | Semantics |
|-------|------|-----------|
| `/api/engage` | — | Mode: `physical` → `sim`. Seed `sim_targets` from current follower positions. Returns 409 if already `sim`. |
| `/api/release` | — | Mode: `sim` → `physical`. Blip follower torque off then on at current positions. Always 200. |
| `/api/target` | `{"joints": [t1,...,t6]}` | Set `sim_targets`. 400 if mode != `sim` or array length != 6. |
| `/api/heartbeat` | — | Stamp `last_heartbeat = time.time()`. Browser calls every 500 ms while engaged. |

**Control module (`dashboard/control.py`).** Lock-guarded state:

```python
mode: "physical" | "sim"     # default "physical"
sim_targets: list[int] | None
last_heartbeat: float | None
last_written: list[int]       # last targets actually written to servos
_force_release_pending: bool  # set by watchdog; drained by consume_force_release()
```

**Public functions used by the main loop:**

```python
def next_follower_targets(physical_targets: list[int], now: float) -> list[int]:
    # 1. If mode == "sim" and now - last_heartbeat > 1.5 s:
    #      mode -> "physical"; set _force_release_pending = True; fall through.
    # 2. If mode == "physical": return physical_targets unchanged.
    # 3. If mode == "sim":
    #      for each joint i:
    #          desired = sim_targets[i]
    #          delta = desired - last_written[i]
    #          clamp delta to [-MAX_DELTA, +MAX_DELTA] where MAX_DELTA = 100 ticks
    #          clamped[i] = last_written[i] + delta
    #      last_written = clamped
    #      return clamped

def consume_force_release() -> bool:
    # Atomically read-and-clear _force_release_pending.
```

The control module does NOT perform serial I/O — it only maintains state and returns target values. `consume_force_release()` is the one-shot signal that tells the main loop "you need to blip torque and re-seat last_written at the current positions". This keeps all serial calls on the main thread, consistent with the existing discipline.

**Main loop change (one call site in `follower_sees_follower_does.py`):**

```python
# was: sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)
final_targets = control.next_follower_targets(follower_targets, time.time())
if control.consume_force_release():
    for sid in FOLLOWER_IDS: write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    # Re-read current positions before re-engaging torque to avoid a jump
    for i in range(6):
        fid = FOLLOWER_IDS[i]
        fp = read_position_robust(f_ser, fid)
        if fp is not None: final_targets[i] = fp
        write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)
sync_write_positions(f_ser, FOLLOWER_IDS, final_targets)
for i in range(6):
    dash_state.update(FOLLOWER_IDS[i], goal=int(final_targets[i]))
```

## Safety layers

All four active simultaneously when mode is `sim`:

1. **Heartbeat watchdog (1.5 s).** No `/api/heartbeat` for 1.5 s → `next_follower_targets` flags force-release; main loop blips torque, mode → `physical`.
2. **Velocity clamp (MAX_DELTA = 100 ticks/iteration).** Applied in `control.py`. At the natural ~500 Hz loop, that's 50,000 ticks/s theoretical ceiling; in practice IK updates at ~60 Hz, giving ~6,000 ticks/s (~1.5 rev/s) effective.
3. **Joint limits.** URDF `<limit>` values clipped in-browser before each `/api/target`; server re-clips as belt-and-braces using the same URDF values loaded at startup.
4. **Release button.** Always visible, always responsive. One click → mode → `physical`, torque blipped.

**Two-stage Ctrl+C** (replaces current `except KeyboardInterrupt`):

- Installed as a `signal.signal(SIGINT, handler)` at startup. Module-level `_interrupt_count` and `_interrupt_ts`.
- **First Ctrl+C:** if mode is `sim`, force release. Freeze targets at current values. Keep follower torque ON. Print `Motion frozen. Press Ctrl+C again to exit.` Increment counter, stamp timestamp.
- **Second Ctrl+C within 5 s:** full shutdown — disable follower torque, close ports, `sys.exit(0)`.
- **No second Ctrl+C within 5 s:** reset counter. Loop continues in physical mode. User can re-engage.

The arm cannot drop unexpectedly from a web-commanded pose; a panicked Ctrl+C always leaves it holding position first.

## UI layout

Page content width caps at ~940 px. New 3D panel sits above the existing tables; tables are unchanged.

```
┌────────────────────────────────────────────────────┐
│ ● SO-101 Telemetry          14:32:05  sim control │
├────────────────────────────────────────────────────┤
│                                   │                │
│   3D viewport (~560×400)          │  Wrist pitch   │
│   • ghost arm (live telemetry)    │   ─────●────   │
│   • target arm (IK from gizmo)    │  Wrist roll    │
│   • end-effector handle           │   ──●───────   │
│   • orbit camera                  │  Gripper       │
│                                   │   ─────────●   │
│                                   │                │
│                                   │  [  Engage  ]  │
├────────────────────────────────────────────────────┤
│  Leader table  (unchanged)                         │
├────────────────────────────────────────────────────┤
│  Follower table  (unchanged)                       │
└────────────────────────────────────────────────────┘
```

**Interaction (mouse-first):**
- Left-click-drag the end-effector handle → moves target in camera plane; IK runs continuously.
- Left-click-drag empty space → orbit camera.
- Right-click-drag → pan camera.
- Scroll → zoom.
- Side panel sliders drive wrist pitch (motor 10), wrist roll (motor 11), gripper (motor 12) directly — never touched by IK.
- Engage / Release button: green "Engage" when idle, red "Release Control" when engaged. Disabled while URDF is loading.

**Visual states:**
- Handle color: `--accent` normally, `--err` when IK residual above threshold.
- Ghost arm: `--dim` at ~30% opacity.
- Target arm: `--fg` at full opacity.
- Status text top-right: dim `physical leader` / bright `sim control — heartbeat OK` / flashing red `sim control released (heartbeat lost)` for 2 s on forced release.

## Error handling & edge cases

| Scenario | Behavior |
|----------|----------|
| URDF or STL fails to load | Viewport shows `Failed to load SO-101 model. Check dashboard/static/models/so101/.`; Engage disabled; tables still work. |
| Second client tries to engage | `/api/engage` returns 409; UI shows `Another client has control`. No forced takeover in v1. |
| Tab closed mid-control | Heartbeat watchdog triggers within 1.5 s → force release. |
| Laptop sleep | Same as tab closed. On wake, UI shows `heartbeat was lost`; user clicks Engage to resume. |
| Server process dies | Browser sees connection refused; UI shows `server offline`. Arm relaxes via existing shutdown path. |
| Network glitch < 1.5 s | Pending POSTs retry; targets may jitter for one frame. No mode change. |
| IK target outside reach | CCD residual stays high; handle red; arm holds best-effort pose. |
| IK singularity | CCD handles naturally (no special case). |
| User yanks gizmo fast | Velocity clamp produces a visible ~200 ms "chase". Working as intended. |
| Calibration file missing | `control.py` logs warning, uses committed defaults. Ghost will be visibly offset in tables pos if wrong; user runs calibration. |

## Calibration procedure

Committed as a standalone `calibrate_home.py` script so the user can align ticks to URDF angles for their specific rig. The main teleop script must be stopped while this runs (single-master bus).

1. Stop any running `follower_sees_follower_does.py`.
2. Manually move the follower arm into the URDF home pose (defined as: shoulder_pan = 0, shoulder_lift = 0, elbow_flex = 0, wrist_pitch = 0, wrist_roll = 0, gripper half-open — visually this is the arm pointing "straight forward" with wrist level). The README section added in the implementation plan describes this pose in words and with an ASCII diagram.
3. Run `python calibrate_home.py`. It opens the follower port, reads present positions, writes them as `tick_zero` values into `kinematics_calibration.json`, and prints a before/after diff.
4. Restart `python follower_sees_follower_does.py`; open the browser.
5. Verify: the ghost arm should match the real arm's pose. If a joint is mirrored (moves opposite to the real arm), flip that joint's `tick_sign` by hand in the JSON.

One-time per rig. Defaults ship with the repo and are good enough for visual correctness before the rig-specific calibration step.

## Validation (no test suite)

Manual checks listed per implementation task. Categories:

- **Python-only**: `python -c` smoke tests for `control.py` logic (velocity clamp, heartbeat timeout) without touching serial.
- **Browser-only**: open the page with a mock JSON server (or the live server with the rig idle) and verify scene loads, ghost follows live pose, gizmo drag works.
- **Full rig**: engage sim mode, drag gizmo, verify follower tracks with ~200 ms lag, verify release returns to physical, verify Ctrl+C freezes then exits.

## Risks & open questions

- **URDF accuracy.** If lerobot's URDF disagrees with your physical rig (e.g. shoulder mount angle differs), the ghost will be visibly offset from reality. Mitigation: calibration procedure can adjust `tick_zero`; systematic offsets are captured there. Geometry mismatches (wrong link length) require editing the vendored URDF directly — unlikely but possible.
- **CCD oscillation near base singularity.** If the user drags the handle very close to the arm's base, CCD can oscillate instead of converging. Mitigation: residual threshold + red handle visual. User reads it as "can't go there" and backs off.
- **One operator, one browser.** No session/auth; anyone who can hit `127.0.0.1:8080` can engage. Since server binds loopback, attack surface is "someone has an SSH tunnel in", which is out of scope.

## Out of scope for this spec (possible follow-ups)

- Saved poses ("home", "rest", user-named).
- Motion recording and replay.
- Per-joint direct-drag mode (alternative to IK).
- Gamepad / spacemouse input.
- Web sockets for lower-latency target push (current design: HTTP POST @ ~60 Hz is fine for a 100-tick/iter clamp).
- Collision checking.
