# Servo Telemetry Dashboard — Design

**Date:** 2026-04-19
**Scope:** Add a read-only HTML dashboard that displays live servo parameters
(position, goal, temperature, voltage, load) for all 12 motors (leader + follower)
while `follower_sees_follower_does.py` teleop is running.

## Goals

- Live visibility into per-motor temperature, voltage, and load during teleop.
- 60 s rolling trend (sparkline) for temperature and load to catch drift and spikes.
- Zero impact on teleop loop responsiveness.
- Zero new runtime dependencies. Stay consistent with the project's
  "three scripts, one dependency (`pyserial`)" ethos.

## Non-goals (YAGNI)

- Controls of any kind (no torque toggles, no goal writes, no calibration).
  `Ctrl+C` in the terminal remains the emergency stop.
- Persisted history (no disk logging, no CSV export, no database).
- Authentication, TLS, or remote access. Binds to `127.0.0.1` only.
- WebSockets / push. 1 Hz polling is sufficient for 12 rows.
- A test suite. The project is deliberately test-free; manual validation only.
- Changes to `scan_motors.py` or `set/set_motor_id.py`.

## Architecture

One process, one event source, two threads:

- **Main thread — teleop loop.** Owns both serial ports. Runs the existing
  delta-following logic. On each iteration, one motor (round-robin cursor)
  gets an 8-byte telemetry read instead of the usual 2-byte position read;
  results are written to a shared `STATE` dict under a lock.
- **HTTP daemon thread.** Runs `http.server.ThreadingHTTPServer` bound to
  `127.0.0.1:8080`. Serves `GET /` (static HTML) and `GET /api/state` (JSON
  snapshot). Reads `STATE` under the same lock; never touches serial.

The HTTP thread is a daemon, so `Ctrl+C` tears it down without explicit
shutdown plumbing. The existing `KeyboardInterrupt` handler is unchanged.

## File layout

```
follower_sees_follower_does.py    (modified: telemetry tick + dashboard startup)
dashboard/
  __init__.py                     (empty)
  state.py                        (STATE dict, lock, ring buffers, public API)
  server.py                       (ThreadingHTTPServer + request handler)
  static/
    index.html                    (table + sparkline JS, one file)
```

Rationale for a package vs. inlining: the main script is currently ~180 lines
and reads end-to-end. Adding ~200 lines of HTTP + state code would dilute the
teleop logic. The package is imported and started with three lines.

### Module responsibilities

**`dashboard/state.py`**
- `init(leader_ids: list[int], follower_ids: list[int]) -> None`
  — Initialize `STATE` with one entry per motor; pre-allocate ring buffers.
- `update(motor_id: int, *, pos: int|None = None, goal: int|None = None,
  temp: int|None = None, volt: float|None = None, load: int|None = None) -> None`
  — Acquire lock, update supplied fields, append `temp` and `load` to the
  motor's 60-slot ring buffer if present, set `updated_at` to `time.time()`.
- `snapshot() -> dict` — Acquire lock, return a deep-copyable dict ready for
  `json.dumps`.
- Ring buffer: plain `collections.deque(maxlen=60)` per motor per tracked field.
- Lock: a single module-level `threading.Lock`. Held for dict ops only;
  never held across serial I/O.

**`dashboard/server.py`**
- `start(host: str = "127.0.0.1", port: int = 8080) -> None`
  — Construct `ThreadingHTTPServer`, spin it up in a daemon thread,
  return immediately. Wraps construction in `try/except OSError` so a
  port-in-use condition logs a warning but does not crash teleop.
- Request handler routes:
  - `GET /` → serve `dashboard/static/index.html` with `Content-Type: text/html`.
  - `GET /api/state` → serve `json.dumps(state.snapshot())` with
    `Content-Type: application/json` and `Cache-Control: no-store`.
  - Anything else → 404.
- Silences the default `BaseHTTPRequestHandler` access log (redirects to
  `logging.NullHandler`) so it doesn't spam stdout and crowd the teleop
  status output.

**`dashboard/static/index.html`**
- Single file, no external assets, no build step.
- Vanilla JS `fetch('/api/state')` every 1000 ms.
- Two tables: "Leader (IDs 7–12)" and "Follower (IDs 1–6, in wired order)".
- Columns per row: `ID, Pos, Goal, Temp, Volt, Load, Temp trend, Load trend`.
  Leader rows show `—` for Goal (no goal on a hand-moved arm).
- Canvas-rendered sparklines, 80×20 px, 60 samples, auto-scaled per cell.
- Header: title, connection indicator (green/red dot), last-update timestamp.

## Data flow & bus scheduling

**ST3215 register layout used here (contiguous):**

| Addr | Field    | Bytes |
|------|----------|-------|
| 56   | Position | 2     |
| 58   | Speed    | 2     |
| 60   | Load     | 2     |
| 62   | Voltage  | 1     |
| 63   | Temp     | 1     |

One 8-byte read at address 56 yields all telemetry fields for a motor.
Speed is captured but not displayed in this iteration; keeping it in `STATE`
costs nothing and leaves room for a future sparkline.

**Scheduling model — "telemetry tick substitution":**

- A module-level cursor indexes into `LEADER_IDS + FOLLOWER_IDS` (12 entries).
- On each teleop loop iteration, for the motor at `cursor % 12`, perform an
  8-byte telemetry read *instead of* the usual 2-byte position read.
  Extract position normally (delta-following logic is unaffected — it only
  needs `position`), extract temp/volt/load and call `state.update(...)`.
  Advance cursor.
- All other motors that iteration get the usual 2-byte read.
- At a natural loop rate of ~500 Hz, full 12-motor telemetry refresh completes
  in ~24 ms. Browser polls at 1 Hz, so data is always fresh.
- Goal position: for followers, the main loop already holds
  `follower_targets[i]`, so `state.update(fid, goal=follower_targets[i])`
  is called any time the target changes. For leaders, goal is never set.

**Dropped frames:** if the 8-byte read returns `None` (same single-attempt
policy as today), `state.update()` is simply not called for that tick. The UI
retains the previous values. No retry, no teleop stutter.

**Lock discipline:** `state.update()` and `state.snapshot()` are the only
lock holders. Both complete in microseconds. Serial I/O happens outside the
lock.

## UI details

**Thresholds (coloring):**
- Temp: <50 °C neutral, 50–64 °C amber, ≥65 °C red.
- Voltage: <10.5 V or >13.5 V amber.
- Load: ≥70 % amber, ≥90 % red.

**Connection indicator:** green if the last successful `/api/state` fetch was
<2 s ago, red otherwise.

**Polling failure:** do not clear the table on fetch error; flip the indicator
red and retain last values. Telemetry gaps should not look like crashes.

**Dark theme.** ~700 px content width. No external fonts, no CDN, no JS
libraries.

## Error handling & lifecycle

| Condition | Behavior |
|-----------|----------|
| Serial port open fails | Existing behavior — script exits. Dashboard thread is started only after both ports open, so no orphan thread. |
| Port 8080 in use | `server.start()` catches `OSError`, prints a warning, returns. Teleop continues without a dashboard. |
| Telemetry read drops a frame | `state.update()` skipped for that tick. UI shows stale value; indicator stays green until no updates arrive for 2 s. |
| No browser open | Server just sits idle. No cost. |
| Browser disconnects mid-request | Handled by `http.server` default behavior. |
| `Ctrl+C` | HTTP thread is daemon → dies with process. Existing `KeyboardInterrupt` handler disables follower torque and closes ports. Unchanged. |

## Testing / validation

Manual, per project convention:

1. Run `python follower_sees_follower_does.py`.
2. Open `http://localhost:8080` in a browser.
3. Wiggle leader motors — position values change, follower goal tracks.
4. Leave idle for 60 s — temperature sparkline populates; voltage steady.
5. Ctrl+C — browser tab shows red indicator within 2 s.

README gets a short section documenting (2) and the port.

## Risks & mitigations

- **Risk:** 8-byte read takes longer than 2-byte, potentially noticeable at loop rate.
  **Mitigation:** only one motor per iteration does the telemetry read, so the
  per-iteration overhead increase is one extra 6-byte payload at 1 Mbps
  (~60 µs). Negligible vs. the existing 2 ms `time.sleep`.
- **Risk:** HTTP handler holds the lock across `json.dumps`.
  **Mitigation:** `snapshot()` copies data out under the lock and returns a
  plain dict; `json.dumps` runs lock-free.
- **Risk:** Ring buffer growth.
  **Mitigation:** `deque(maxlen=60)` is fixed-size.

## Out of scope for this spec (possible follow-ups)

- Speed sparkline (data is already captured).
- Configurable thresholds via query string or small settings page.
- CSV export of the ring buffer.
- Authentication if binding to non-loopback.
