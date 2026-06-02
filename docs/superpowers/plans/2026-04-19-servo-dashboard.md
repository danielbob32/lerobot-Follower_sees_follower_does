# Servo Telemetry Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a read-only HTML dashboard to `follower_sees_follower_does.py` that shows live position, goal, temperature, voltage, and load for all 12 servos, with 60 s sparklines for temperature and load.

**Architecture:** In-process `http.server` daemon thread serves a static HTML page and a JSON snapshot endpoint. The main teleop loop interleaves one 8-byte telemetry read per iteration (round-robin across the 12 motors) into a shared, lock-guarded state dict. Zero new Python dependencies; no build step; no tests (project convention — validation is manual on the live rig).

**Tech Stack:** Python 3 stdlib (`http.server`, `threading`, `collections.deque`, `json`, `pathlib`), `pyserial` (already required), vanilla HTML/CSS/JS with Canvas sparklines.

**Spec:** `docs/superpowers/specs/2026-04-19-servo-dashboard-design.md`

**Project context worth repeating for engineers new to this repo:**
- `CLAUDE.md` is the authoritative orientation — read it before Task 1.
- Wire protocol is ST3215/Waveshare: `[0xFF, 0xFF, ID, len, instr, ...params, chk]` where `chk = (~sum(packet[2:])) & 0xFF`.
- Registers relevant here: `56-57` pos, `58-59` speed, `60-61` load, `62` voltage (0.1 V units), `63` temperature (°C), `42-43` goal, `40` torque enable.
- Bus is single-master, single-process per port. All serial I/O MUST stay on the main thread. The HTTP thread never touches `serial.Serial` instances.
- There is intentionally no test suite. Every task ends with a manual smoke check against a live rig (or a targeted `python -c` invocation where possible).

---

## File Structure

Files created:
- `dashboard/__init__.py` — empty package marker
- `dashboard/state.py` — shared STATE, lock, ring buffers, public API (`init`, `update`, `snapshot`)
- `dashboard/server.py` — `ThreadingHTTPServer` wrapper + request handler
- `dashboard/static/index.html` — single-file dashboard UI

Files modified:
- `follower_sees_follower_does.py` — add `read_telemetry()`, wire dashboard startup, interleave telemetry cursor into main loop, push follower goals into state on sync-write
- `README.md` — short "Dashboard" section

No other files touched. `scan_motors.py` and `set/set_motor_id.py` are unchanged.

---

## Task 1: Dashboard state module

**Files:**
- Create: `dashboard/__init__.py`
- Create: `dashboard/state.py`

- [ ] **Step 1: Create the empty package marker**

Create `dashboard/__init__.py` with an empty file (zero bytes — do not add a docstring or anything else).

- [ ] **Step 2: Write `dashboard/state.py`**

Create `dashboard/state.py` with exactly this content:

```python
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
    ring buffers. Unknown motor_ids are silently ignored (defensive — the
    main loop should never hit this path)."""
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
```

- [ ] **Step 3: Smoke-test the module from the command line**

Run:

```bash
python -c "from dashboard import state; state.init([7,8],[1,2]); state.update(1, pos=2048, temp=38, volt=11.9, load=12); state.update(1, temp=39); import json; print(json.dumps(state.snapshot(), indent=2))"
```

Expected output includes:
- `"now":` a float timestamp
- `"history_len": 60`
- four motors in order `[7, 8, 1, 2]`, each with `"role"` set
- motor `1` has `"pos": 2048, "temp": 39, "volt": 11.9, "load": 12`, `"temp_history": [38, 39], "load_history": [12]`
- motor `2` has all numeric fields `null`, histories `[]`

If the output matches, the module works.

- [ ] **Step 4: Commit**

```bash
git add dashboard/__init__.py dashboard/state.py
git commit -m "$(cat <<'EOF'
Add dashboard state module with lock-guarded ring buffers

Shared thread-safe state consumed by the dashboard HTTP thread. Exposes
init/update/snapshot. temp and load get 60-sample deques for sparklines.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: HTTP server module + stub HTML

**Files:**
- Create: `dashboard/server.py`
- Create: `dashboard/static/index.html` (stub — replaced in Task 3)

- [ ] **Step 1: Write `dashboard/server.py`**

Create `dashboard/server.py` with exactly this content:

```python
"""Tiny stdlib HTTP server that exposes the telemetry state as JSON and
serves a single static HTML file. Runs in a daemon thread so Ctrl+C in the
main script tears it down without explicit shutdown."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from dashboard import state

_STATIC_DIR = Path(__file__).parent / "static"


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        # Silence default stdout access logs so they don't crowd teleop status.
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file("index.html", "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._serve_json(state.snapshot())
        else:
            self.send_error(404)

    def _serve_file(self, name, content_type):
        path = _STATIC_DIR / name
        try:
            data = path.read_bytes()
        except FileNotFoundError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _serve_json(self, obj):
        data = json.dumps(obj).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def start(host="127.0.0.1", port=8080):
    """Spin up the server in a daemon thread. Prints a warning and returns
    (without raising) if the port is already in use — the dashboard is a
    nice-to-have and must not take down teleop."""
    try:
        srv = ThreadingHTTPServer((host, port), _Handler)
    except OSError as e:
        print(f"[dashboard] port {port} unavailable ({e}); dashboard disabled.")
        return
    t = threading.Thread(target=srv.serve_forever, daemon=True,
                         name="dashboard-http")
    t.start()
    print(f"[dashboard] serving on http://{host}:{port}")
```

- [ ] **Step 2: Create a stub `dashboard/static/index.html`**

Create `dashboard/static/index.html` with just this content (it is replaced in Task 3):

```html
<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>SO-101 Telemetry</title></head>
<body><p>Dashboard stub. See <a href="/api/state">/api/state</a>.</p></body>
</html>
```

- [ ] **Step 3: Smoke-test the server end-to-end in one shell command**

Run this from the repo root — it starts the server, seeds some state, queries both endpoints, and exits:

```bash
python -c "
from dashboard import state, server
import time, urllib.request
state.init([7],[1])
state.update(1, pos=2048, temp=38, volt=11.9, load=12)
server.start(port=8099)
time.sleep(0.2)
print('--- GET / ---')
print(urllib.request.urlopen('http://127.0.0.1:8099/').read().decode()[:80])
print('--- GET /api/state ---')
print(urllib.request.urlopen('http://127.0.0.1:8099/api/state').read().decode()[:200])
"
```

Expected output:
- Line `[dashboard] serving on http://127.0.0.1:8099`
- `--- GET / ---` followed by the opening of the stub HTML (`<!DOCTYPE html>...`)
- `--- GET /api/state ---` followed by JSON starting `{"now": ..., "history_len": 60, "motors": [{"id": 7,`

If both endpoints return content, the server works.

- [ ] **Step 4: Commit**

```bash
git add dashboard/server.py dashboard/static/index.html
git commit -m "$(cat <<'EOF'
Add dashboard HTTP server with stub page

ThreadingHTTPServer bound to 127.0.0.1:8080 in a daemon thread. Serves
GET / (static HTML) and GET /api/state (snapshot JSON). Port-in-use is
non-fatal: teleop must continue even if the dashboard fails to start.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Full dashboard HTML/CSS/JS

**Files:**
- Modify: `dashboard/static/index.html` (replace stub with the full UI)

**XSS note:** the JS builds rows with `createElement` + `textContent` — no `innerHTML`, no template-string HTML interpolation. Even though the state values are numeric today, this keeps the code safe by construction if a string field is ever added.

- [ ] **Step 1: Replace `dashboard/static/index.html` with the full UI**

Overwrite `dashboard/static/index.html` with exactly this content:

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>SO-101 Telemetry</title>
<style>
:root {
  --bg: #0f1115;
  --panel: #161a21;
  --border: #262b35;
  --fg: #d8dde5;
  --dim: #7b8496;
  --ok: #58d068;
  --warn: #f0b429;
  --err: #e5484d;
  --accent: #4c8df6;
}
* { box-sizing: border-box; }
body {
  background: var(--bg);
  color: var(--fg);
  font: 13px/1.4 ui-monospace, "SF Mono", Menlo, Consolas, monospace;
  margin: 0;
  padding: 24px;
}
.wrap { max-width: 760px; margin: 0 auto; }
header { display: flex; align-items: center; gap: 12px; margin-bottom: 20px; }
h1 { font-size: 15px; font-weight: 600; margin: 0; letter-spacing: 0.3px; }
.dot {
  width: 8px; height: 8px; border-radius: 50%;
  background: var(--dim); box-shadow: 0 0 6px currentColor;
}
.dot.ok { background: var(--ok); color: var(--ok); }
.dot.err { background: var(--err); color: var(--err); }
.ts { color: var(--dim); margin-left: auto; font-variant-numeric: tabular-nums; }
section { margin-bottom: 24px; }
h2 {
  font-size: 11px; font-weight: 600; text-transform: uppercase;
  letter-spacing: 1px; color: var(--dim); margin: 0 0 8px;
}
table {
  width: 100%; border-collapse: collapse; background: var(--panel);
  border: 1px solid var(--border); border-radius: 6px; overflow: hidden;
}
th, td {
  padding: 8px 10px; text-align: right; border-bottom: 1px solid var(--border);
  font-variant-numeric: tabular-nums;
}
th {
  background: #1a1e26; color: var(--dim); font-weight: 500;
  font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px;
}
td.id, th.id { text-align: left; }
tr:last-child td { border-bottom: none; }
.warn { color: var(--warn); }
.err { color: var(--err); }
canvas { display: block; margin-left: auto; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <span id="dot" class="dot"></span>
    <h1>SO-101 Telemetry</h1>
    <span id="ts" class="ts">&mdash;</span>
  </header>

  <section>
    <h2>Leader (IDs 7&ndash;12)</h2>
    <table>
      <thead><tr>
        <th class="id">ID</th><th>Pos</th><th>Goal</th>
        <th>Temp</th><th>Volt</th><th>Load</th>
        <th>Temp 60s</th><th>Load 60s</th>
      </tr></thead>
      <tbody id="leader"></tbody>
    </table>
  </section>

  <section>
    <h2>Follower (IDs 1&ndash;6, wired order)</h2>
    <table>
      <thead><tr>
        <th class="id">ID</th><th>Pos</th><th>Goal</th>
        <th>Temp</th><th>Volt</th><th>Load</th>
        <th>Temp 60s</th><th>Load 60s</th>
      </tr></thead>
      <tbody id="follower"></tbody>
    </table>
  </section>
</div>

<script>
const POLL_MS = 1000;
const STALE_MS = 2000;
let lastOk = 0;

function tempClass(t) {
  if (t === null || t === undefined) return "";
  if (t >= 65) return "err";
  if (t >= 50) return "warn";
  return "";
}
function voltClass(v) {
  if (v === null || v === undefined) return "";
  if (v < 10.5 || v > 13.5) return "warn";
  return "";
}
function loadClass(l) {
  if (l === null || l === undefined) return "";
  if (l >= 90) return "err";
  if (l >= 70) return "warn";
  return "";
}
function num(v, suffix, digits) {
  if (v === null || v === undefined) return "\u2014";
  const s = digits ? v.toFixed(digits) : String(Math.round(v));
  return suffix ? s + suffix : s;
}

function sparkline(data, color) {
  const dpr = window.devicePixelRatio || 1;
  const w = 80, h = 20;
  const c = document.createElement("canvas");
  c.width = w * dpr; c.height = h * dpr;
  c.style.width = w + "px"; c.style.height = h + "px";
  const ctx = c.getContext("2d");
  ctx.scale(dpr, dpr);
  if (!data || data.length < 2) return c;
  const min = Math.min.apply(null, data);
  const max = Math.max.apply(null, data);
  const range = max - min || 1;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.beginPath();
  data.forEach(function (v, i) {
    const x = (i / (data.length - 1)) * (w - 2) + 1;
    const y = h - 1 - ((v - min) / range) * (h - 2);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  return c;
}

function cell(text, cls) {
  const td = document.createElement("td");
  td.textContent = text;
  if (cls) td.className = cls;
  return td;
}

function row(m) {
  const tr = document.createElement("tr");

  const idCell = document.createElement("td");
  idCell.className = "id";
  idCell.textContent = String(m.id);
  tr.appendChild(idCell);

  tr.appendChild(cell(num(m.pos)));
  tr.appendChild(cell(m.role === "leader" ? "\u2014" : num(m.goal)));
  tr.appendChild(cell(num(m.temp, "\u00b0C"), tempClass(m.temp)));
  tr.appendChild(cell(num(m.volt, "V", 1), voltClass(m.volt)));
  tr.appendChild(cell(num(m.load, "%"), loadClass(m.load)));

  const tempSparkTd = document.createElement("td");
  tempSparkTd.appendChild(sparkline(m.temp_history, "#f0b429"));
  tr.appendChild(tempSparkTd);

  const loadSparkTd = document.createElement("td");
  loadSparkTd.appendChild(sparkline(m.load_history, "#4c8df6"));
  tr.appendChild(loadSparkTd);

  return tr;
}

async function tick() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    if (!r.ok) throw new Error("status " + r.status);
    const data = await r.json();
    lastOk = Date.now();
    const leader = document.getElementById("leader");
    const follower = document.getElementById("follower");
    const leaderRows = data.motors.filter(function (m) { return m.role === "leader"; }).map(row);
    const followerRows = data.motors.filter(function (m) { return m.role === "follower"; }).map(row);
    leader.replaceChildren.apply(leader, leaderRows);
    follower.replaceChildren.apply(follower, followerRows);
    const d = new Date(data.now * 1000);
    document.getElementById("ts").textContent = d.toTimeString().slice(0, 8);
  } catch (e) {
    /* keep previous values on screen — see STALE_MS check below */
  }
  const fresh = Date.now() - lastOk < STALE_MS;
  document.getElementById("dot").className = "dot " + (fresh ? "ok" : "err");
}

tick();
setInterval(tick, POLL_MS);
</script>
</body>
</html>
```

- [ ] **Step 2: Smoke-test the UI against synthetic state**

Run this in a shell — it starts the server, seeds realistic-looking state for all 12 motors (including warn/err values to exercise the color classes), and waits:

```bash
python -c "
from dashboard import state, server
import time, random
state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
for mid in [7,8,9,10,11,12]:
    state.update(mid, pos=2048+random.randint(-100,100), temp=42+mid%3, volt=11.9, load=5+mid%10)
state.update(1, pos=2050, goal=2050, temp=55, volt=11.7, load=72)  # warn: temp + load
state.update(2, pos=1000, goal=1000, temp=68, volt=11.9, load=94)  # err: temp + load
state.update(3, pos=1500, goal=1500, temp=30, volt=10.2, load=20)  # warn: volt
state.update(4, pos=3000, goal=3000, temp=35, volt=11.9, load=10)
state.update(6, pos=2500, goal=2500, temp=40, volt=12.0, load=15)
state.update(5, pos=2200, goal=2200, temp=38, volt=11.8, load=8)
# Populate 30 samples of history to get visible sparklines
for i in range(30):
    for mid in [1,2,3,4,6,5,7,8,9,10,11,12]:
        state.update(mid, temp=35+i%10, load=i%40)
server.start(port=8099)
print('Open http://127.0.0.1:8099 — Ctrl+C to stop')
time.sleep(600)
" &
SERVER_PID=$!
echo "server pid: $SERVER_PID — open http://127.0.0.1:8099 in your browser"
```

Verify in a browser at `http://127.0.0.1:8099`:
- Header shows green dot and a timestamp that updates every second.
- Leader table has 6 rows, IDs 7–12, Goal column shows `—`.
- Follower table has 6 rows in order `1, 2, 3, 4, 6, 5`, Goal shows integers.
- Motor 1 row: Temp and Load cells are amber.
- Motor 2 row: Temp and Load cells are red.
- Motor 3 row: Volt cell is amber.
- Temp and Load sparklines are visible and not flat (because of the 30-sample loop).

Stop the background server when satisfied:

```bash
kill $SERVER_PID 2>/dev/null
```

- [ ] **Step 3: Commit**

```bash
git add dashboard/static/index.html
git commit -m "$(cat <<'EOF'
Implement dashboard UI with sparklines and threshold coloring

Dark-theme single-file HTML. Polls /api/state every 1 s, renders two
tables (leader/follower), color-codes temp/volt/load against safety
thresholds, and draws 60-sample inline Canvas sparklines for temp and
load. Retains last values on fetch failure; flips connection indicator
red after 2 s of staleness. Rows built via createElement/textContent —
no innerHTML, safe against future string fields.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Add `read_telemetry()` helper to the main script

**Files:**
- Modify: `follower_sees_follower_does.py` (add one function, no loop changes yet)

- [ ] **Step 1: Insert `read_telemetry()` alongside `read_position_robust()`**

Open `follower_sees_follower_does.py`. Find `read_position_robust` (currently at lines 32–46). Immediately after its closing line (the line containing `return None`, followed by a blank line), insert this function — keep `read_position_robust` itself unchanged:

```python
def read_telemetry(ser, servo_id):
    """Read 8 bytes starting at ADDR_PRESENT_POSITION: pos(2), speed(2),
    load(2), voltage(1), temperature(1). Returns a dict of normalized
    values or None on any read failure. Single attempt — same discipline
    as read_position_robust."""
    ser.reset_input_buffer()
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, ADDR_PRESENT_POSITION, 8]
    packet.append(calculate_checksum(packet))
    ser.write(bytearray(packet))
    try:
        # Response: header(2)+id(1)+len(1)+err(1)+data(8)+chk(1) = 14 bytes
        r = ser.read(14)
        if len(r) != 14 or r[0] != 0xFF or r[1] != 0xFF:
            return None
        pos = ((r[6] << 8) | r[5]) % 4096
        speed_raw = (r[8] << 8) | r[7]
        load_raw = (r[10] << 8) | r[9]
        volt_raw = r[11]
        temp_raw = r[12]
        # ST3215 load is 10 bits + direction in bit 10; we ignore direction
        # and rescale magnitude 0-1023 to 0-100 (approximate percent).
        load_pct = int((load_raw & 0x3FF) * 100 / 1023)
        return {
            "pos": pos,
            "speed": speed_raw,
            "load": load_pct,
            "volt": volt_raw / 10.0,
            "temp": temp_raw,
        }
    except Exception:
        return None
```

- [ ] **Step 2: Smoke-test the helper against a real servo (requires the rig)**

With the follower arm powered and connected, run this from the repo root. Replace `COM7` with whatever `FOLLOWER_PORT` is in your copy of `follower_sees_follower_does.py`, and the ID `1` with an ID that actually responds (check `scan_motors.py` output first if unsure):

```bash
python -c "
import serial, follower_sees_follower_does as m
ser = serial.Serial('COM7', 1000000, timeout=0.01)
print(m.read_telemetry(ser, 1))
ser.close()
"
```

Expected output: a dict like `{'pos': 2048, 'speed': 0, 'load': 2, 'volt': 11.9, 'temp': 35}` (exact numbers depend on the arm).

- `pos` should be in `[0, 4095]`.
- `temp` should be a sane room-temperature integer (20–50).
- `volt` should be near the rig's supply voltage (~11–12 V for a 12 V PSU).
- `load` should be small when the arm is idle.

If the call returns `None`, try again (single-attempt reads drop frames occasionally). If it consistently returns `None`, pause and investigate — something is wrong with the port or the ID.

**If the rig is not available**, skip this step and mark it done; the helper is exercised end-to-end in Task 5.

- [ ] **Step 3: Commit**

```bash
git add follower_sees_follower_does.py
git commit -m "$(cat <<'EOF'
Add read_telemetry helper for 8-byte block reads

Reads pos/speed/load/voltage/temperature in one round-trip at addr 56.
Single-attempt, returns None on any failure — same dropped-frame
discipline as read_position_robust. Not wired into the main loop yet.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Wire dashboard startup + telemetry cursor into the main loop

**Files:**
- Modify: `follower_sees_follower_does.py`

This task changes the main loop. Work carefully — the delta-following logic must remain exactly as it is, only the *source* of `curr_l` changes on some iterations.

- [ ] **Step 1: Add the dashboard import near the top of the file**

Open `follower_sees_follower_does.py`. Right below the existing `import time` line (line 2), insert:

```python

from dashboard import state as dash_state
from dashboard import server as dash_server
```

(Blank line above the imports preserves the existing spacing before `# --- CONFIGURATION ---`.)

- [ ] **Step 2: Initialize dashboard state and start the server after both ports open**

Find this block (it begins around line 80 after the `try:` line):

```python
    print("Opening High-Speed Ports...")
    l_ser = serial.Serial(LEADER_PORT, BAUDRATE, timeout=0.01)  # Ultra low timeout
    f_ser = serial.Serial(FOLLOWER_PORT, BAUDRATE, timeout=0.01)

    print("\n--- HIGH SPEED SYNC TELEOP ---")
```

Replace it with:

```python
    print("Opening High-Speed Ports...")
    l_ser = serial.Serial(LEADER_PORT, BAUDRATE, timeout=0.01)  # Ultra low timeout
    f_ser = serial.Serial(FOLLOWER_PORT, BAUDRATE, timeout=0.01)

    dash_state.init(LEADER_IDS, FOLLOWER_IDS)
    dash_server.start(host="127.0.0.1", port=8080)

    print("\n--- HIGH SPEED SYNC TELEOP ---")
```

- [ ] **Step 3: Seed state with initial positions during the lock phase**

Find the existing "Locking..." block:

```python
    print("Locking...")
    # Initial lock must be individual to read positions safely
    for i in range(6):
        lid = LEADER_IDS[i]
        fid = FOLLOWER_IDS[i]

        lp = read_position_robust(l_ser, lid)
        while lp is None: lp = read_position_robust(l_ser, lid)
        prev_leader_pos[i] = lp

        fp = read_position_robust(f_ser, fid)
        while fp is None: fp = read_position_robust(f_ser, fid)
        follower_targets[i] = fp

        write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)  # Lock
```

Replace it with (only two lines added — the two `dash_state.update(...)` calls):

```python
    print("Locking...")
    # Initial lock must be individual to read positions safely
    for i in range(6):
        lid = LEADER_IDS[i]
        fid = FOLLOWER_IDS[i]

        lp = read_position_robust(l_ser, lid)
        while lp is None: lp = read_position_robust(l_ser, lid)
        prev_leader_pos[i] = lp
        dash_state.update(lid, pos=lp)

        fp = read_position_robust(f_ser, fid)
        while fp is None: fp = read_position_robust(f_ser, fid)
        follower_targets[i] = fp
        dash_state.update(fid, pos=fp, goal=fp)

        write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)  # Lock
```

- [ ] **Step 4: Replace the main loop body with the interleaved-telemetry version**

Find the main loop. It starts with `while True:` (around line 121) and currently looks like this:

```python
    while True:
        # Loop Variables
        update_needed = False

        # 1. READ PHASE (Loop through Leader)
        # We still read individually (unless you want to implement Bulk Read later)
        # But we do it fast.
        for i in range(6):
            lid = LEADER_IDS[i]
            curr_l = read_position_robust(l_ser, lid)

            if curr_l is not None:
                # Calculate Delta
                delta = curr_l - prev_leader_pos[i]

                # Wrap-Around Logic
                if delta > 2048:  delta -= 4096
                if delta < -2048: delta += 4096

                if delta != 0:
                    # Update Target
                    prev_leader_pos[i] = curr_l
                    follower_targets[i] += (delta * DIRECTIONS[i])
                    update_needed = True

        # 2. WRITE PHASE (One Packet for All)
        if update_needed:
            sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)
```

Replace that section (up to and including the `if update_needed:` block with its `sync_write_positions(...)` call) with:

```python
    telemetry_cursor = 0

    while True:
        # Loop Variables
        update_needed = False

        # Telemetry cursor picks one motor per iteration to receive the
        # 8-byte block read instead of (for leaders) the 2-byte position
        # read, or (for followers, which have no baseline read) one extra
        # 8-byte read.
        tele_idx = telemetry_cursor % 12
        tele_is_leader = tele_idx < 6
        tele_arm_idx = tele_idx if tele_is_leader else tele_idx - 6

        # 1. READ PHASE (Leaders)
        for i in range(6):
            lid = LEADER_IDS[i]

            if tele_is_leader and tele_arm_idx == i:
                tele = read_telemetry(l_ser, lid)
                if tele is not None:
                    curr_l = tele["pos"]
                    dash_state.update(lid,
                                      pos=curr_l,
                                      temp=tele["temp"],
                                      volt=tele["volt"],
                                      load=tele["load"],
                                      speed=tele["speed"])
                else:
                    curr_l = None
            else:
                curr_l = read_position_robust(l_ser, lid)
                if curr_l is not None:
                    dash_state.update(lid, pos=curr_l)

            if curr_l is not None:
                # Calculate Delta
                delta = curr_l - prev_leader_pos[i]

                # Wrap-Around Logic
                if delta > 2048:  delta -= 4096
                if delta < -2048: delta += 4096

                if delta != 0:
                    # Update Target
                    prev_leader_pos[i] = curr_l
                    follower_targets[i] += (delta * DIRECTIONS[i])
                    update_needed = True

        # 1b. Follower telemetry (only when the cursor lands on one)
        if not tele_is_leader:
            fid = FOLLOWER_IDS[tele_arm_idx]
            tele = read_telemetry(f_ser, fid)
            if tele is not None:
                dash_state.update(fid,
                                  pos=tele["pos"],
                                  temp=tele["temp"],
                                  volt=tele["volt"],
                                  load=tele["load"],
                                  speed=tele["speed"])

        telemetry_cursor += 1

        # 2. WRITE PHASE (One Packet for All)
        if update_needed:
            sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)
            for i in range(6):
                dash_state.update(FOLLOWER_IDS[i], goal=int(follower_targets[i]))
```

Leave the status-print block (the `if time.time() - last_print_time >= 5.0:` section) and the `time.sleep(0.002)` at the end of the loop unchanged.

- [ ] **Step 5: Manually validate on the live rig**

With both arms powered and connected (this is the full-system test):

```bash
python follower_sees_follower_does.py
```

Expected terminal output:
- `Opening High-Speed Ports...`
- `[dashboard] serving on http://127.0.0.1:8080`
- `--- HIGH SPEED SYNC TELEOP ---`
- `Move to Sync Position. Press ENTER.`

Press ENTER once the arms are aligned. Verify:

1. **Teleop still works** — move the leader arm; the follower follows. The 5-second status lines still print in the terminal.
2. **Dashboard works** — open `http://127.0.0.1:8080` in a browser:
   - Both tables populate within a second or two (12 motors × ~24 ms → first full refresh < 1 s).
   - Green dot; timestamp ticks every second.
   - Pos values in the leader table change live as you wiggle the leader arm.
   - Goal values in the follower table track the leader.
   - Temperature readings are in the 25–45 °C range (normal).
   - Voltage readings are near supply voltage.
   - After ~60 s idle, the temperature and load sparklines are populated.
3. **No bus contention** — teleop does not visibly lag; the follower tracks in real time.
4. **Ctrl+C** — follower relaxes (torque disabled), ports close, process exits. The browser tab's indicator turns red within 2 s.

If any of these fail, stop and debug before committing.

- [ ] **Step 6: Commit**

```bash
git add follower_sees_follower_does.py
git commit -m "$(cat <<'EOF'
Wire telemetry dashboard into teleop loop

Round-robin cursor picks one motor per iteration for an 8-byte block
read; the leader path substitutes the 2-byte position read, the
follower path adds a dedicated read. All serial I/O stays on the main
thread; dash_state.update() is called under a lock. Dashboard server
starts after both ports open and shuts down as a daemon thread on
Ctrl+C. Goals for followers are pushed to state after each sync-write.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Document the dashboard in the README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Read the current README to pick the right insertion point**

Run:

```bash
cat README.md
```

Note which heading documents how to run the main script. The new section goes directly after that one.

- [ ] **Step 2: Add a "Telemetry dashboard" section**

Append (or insert, depending on what you found in Step 1) this section. If the README already uses `##` headings at the top level for its other sections, match that level; otherwise match whatever level the "run the main script" section uses.

```markdown
## Telemetry dashboard

While `follower_sees_follower_does.py` is running, a read-only dashboard is
served at `http://127.0.0.1:8080`. It shows per-motor position, goal,
temperature, voltage, and load for all 12 servos, with 60-second trend
sparklines for temperature and load.

The dashboard uses only the Python standard library and has no build step.
If port 8080 is already in use, the script logs a warning and continues
without a dashboard — teleop is never blocked by dashboard startup.

Thresholds:

- Temperature: amber ≥ 50 °C, red ≥ 65 °C
- Voltage: amber outside 10.5–13.5 V
- Load: amber ≥ 70%, red ≥ 90%

The dashboard is read-only by design. Use `Ctrl+C` in the terminal to stop
the robot — there is no stop button in the UI.
```

- [ ] **Step 3: Re-read the README and confirm the section landed cleanly**

```bash
cat README.md
```

Check that the new section sits where you intended and that heading levels are consistent with neighbors.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
Document the telemetry dashboard in the README

Covers access URL, what it shows, threshold values, and the
read-only-by-design posture.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final check

After Task 6, run `git log --oneline` from the repo root and confirm the last six commits (in addition to the two design-spec commits) are present in order:

1. Add dashboard state module with lock-guarded ring buffers
2. Add dashboard HTTP server with stub page
3. Implement dashboard UI with sparklines and threshold coloring
4. Add read_telemetry helper for 8-byte block reads
5. Wire telemetry dashboard into teleop loop
6. Document the telemetry dashboard in the README

Then do one more end-to-end run (`python follower_sees_follower_does.py`, wiggle arms, browse, Ctrl+C) to confirm the README instructions match reality.
