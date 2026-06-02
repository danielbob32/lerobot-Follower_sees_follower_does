# Browser Sim & End-Effector Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Commit steps are included for conventional hygiene; this project's owner typically skips commits during rapid iteration, so skipping them is acceptable as long as all tasks reach their validation step successfully.

**Goal:** Extend the existing telemetry dashboard with a 3D visualization of the SO-101 (ghost arm from live telemetry, URDF-driven) and, on top of that, a browser-driven control mode where the operator drags an end-effector handle to move the follower via IK.

**Architecture:** All new browser assets are vendored (three.js, urdf-loader, STLLoader, OrbitControls, SO-101 URDF + meshes) — zero runtime network. Python stays URDF-free; all kinematics live in the browser. A new `dashboard/control.py` module owns mode/sim-target/heartbeat state behind a lock; the main teleop loop calls one function per iteration to decide whether to follow the physical leader or the sim. Multiple dead-man safety layers: heartbeat watchdog, per-joint velocity clamp, URDF-derived joint limits in tick space, explicit Release button, two-stage Ctrl+C.

**Tech Stack:** Python 3 stdlib (`http.server`, `threading`, `signal`, `json`, `ast`), `pyserial`, three.js (vendored), urdf-loader (vendored), SO-101 URDF + STL meshes from `huggingface/lerobot`.

**Spec:** `docs/superpowers/specs/2026-04-19-sim-control-design.md`

**Phases:**
- **Phase 1 (Tasks 1–3, 7):** Vendor assets; render read-only ghost arm in browser from live telemetry. Shippable on its own — operator gets a 3D monitor for free.
- **Phase 2 (Tasks 4–6, 8–12):** Gizmo + IK + control path + safety + calibration + README. Phase 1 is a prerequisite.

**Project context worth repeating for engineers new to this repo:**
- `CLAUDE.md` is the authoritative orientation — read it before starting.
- The bus is single-master, one process per port. All serial I/O must stay on the main thread. `dashboard/control.py` is a pure state module; it does not touch `serial.Serial`.
- There is intentionally no test suite. Every task ends with a manual smoke check (Python `-c` snippet, browser observation, or full-rig run). TDD is adapted to this by writing runnable validation snippets inline.
- Windows + bash shell environment. Forward slashes in paths. No sudo.

---

## File Structure

Files created in this plan:
- `dashboard/static/vendor/three.min.js`
- `dashboard/static/vendor/OrbitControls.js`
- `dashboard/static/vendor/STLLoader.js`
- `dashboard/static/vendor/URDFLoader.js`
- `dashboard/static/models/so101/so101.urdf`
- `dashboard/static/models/so101/meshes/*.stl` (multiple files referenced by the URDF)
- `dashboard/static/sim.js`
- `dashboard/control.py`
- `kinematics_calibration.json` (repo root)
- `calibrate_home.py` (repo root)

Files modified in this plan:
- `dashboard/server.py` — add POST routes
- `dashboard/static/index.html` — add 3D panel, side panel, engage/release button, script tags
- `follower_sees_follower_does.py` — route targets through `control.next_follower_targets`, replace `except KeyboardInterrupt` with a two-stage `SIGINT` handler
- `README.md` — sim control section + calibration procedure

---

## Task 1: Vendor three.js and helper libraries

**Files:**
- Create: `dashboard/static/vendor/three.min.js`
- Create: `dashboard/static/vendor/OrbitControls.js`
- Create: `dashboard/static/vendor/STLLoader.js`
- Create: `dashboard/static/vendor/URDFLoader.js`

**Why this task exists:** the dashboard must remain offline-capable. We pin specific versions by vendoring them.

- [ ] **Step 1: Decide on version pinning**

Use **three.js r152** (mid-2023 LTS-flavor release; stable API for `OrbitControls`, `STLLoader`, and the external `urdf-loader` library). This matches the expectations of `urdf-loader` at version 0.12.x.

- [ ] **Step 2: Download the four files**

From the repo root, run these commands (each produces ~100–600 KB of minified JS):

```bash
mkdir -p dashboard/static/vendor
curl -L -o dashboard/static/vendor/three.min.js \
  https://unpkg.com/three@0.152.2/build/three.min.js
curl -L -o dashboard/static/vendor/OrbitControls.js \
  https://unpkg.com/three@0.152.2/examples/js/controls/OrbitControls.js
curl -L -o dashboard/static/vendor/STLLoader.js \
  https://unpkg.com/three@0.152.2/examples/js/loaders/STLLoader.js
curl -L -o dashboard/static/vendor/URDFLoader.js \
  https://unpkg.com/urdf-loader@0.12.1/umd/URDFLoader.js
```

If any URL 404s, search npm or jsdelivr for the same package+version; the filename pattern is stable across mirrors.

- [ ] **Step 3: Smoke-test the files load correctly**

Create a temporary `dashboard/static/_vendor_check.html` with this content:

```html
<!DOCTYPE html>
<html><head><meta charset="utf-8"></head><body>
<script src="vendor/three.min.js"></script>
<script src="vendor/OrbitControls.js"></script>
<script src="vendor/STLLoader.js"></script>
<script src="vendor/URDFLoader.js"></script>
<script>
document.body.textContent =
  "THREE: " + (typeof THREE) +
  ", OrbitControls: " + (typeof THREE.OrbitControls) +
  ", STLLoader: " + (typeof THREE.STLLoader) +
  ", URDFLoader: " + (typeof URDFLoader);
</script>
</body></html>
```

Then from the repo root:

```bash
python -m http.server 9999 --directory dashboard/static
```

Open `http://localhost:9999/_vendor_check.html` in a browser. Expected on-page text:
`THREE: object, OrbitControls: function, STLLoader: function, URDFLoader: function`

If any shows `undefined`, re-download the file — it was an HTML 404 page saved as .js.

Kill the server (Ctrl+C). Delete `dashboard/static/_vendor_check.html`.

- [ ] **Step 4: Verify file sizes are sane**

Run:

```bash
ls -la dashboard/static/vendor/
```

Expected: `three.min.js` ~600 KB, `OrbitControls.js` ~25 KB, `STLLoader.js` ~10 KB, `URDFLoader.js` ~40–80 KB. Anything under 1 KB means a failed download — re-fetch.

- [ ] **Step 5: Commit**

```bash
git add dashboard/static/vendor/
git commit -m "$(cat <<'EOF'
Vendor three.js r152 + OrbitControls + STLLoader + URDFLoader

Pinned versions: three@0.152.2, urdf-loader@0.12.1. Committed so the
dashboard remains offline-capable (no CDN at runtime).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Vendor SO-101 URDF and meshes

**Files:**
- Create: `dashboard/static/models/so101/so101.urdf`
- Create: `dashboard/static/models/so101/meshes/*.stl`

**Why this task exists:** "100% aligned to robot models" requires the canonical URDF, not a hand-authored kinematic chain.

- [ ] **Step 1: Locate the SO-101 URDF and meshes**

The canonical source is the `huggingface/lerobot` repo on GitHub. Paths have moved across lerobot versions; search in order:

1. `https://github.com/huggingface/lerobot/tree/main/src/lerobot/common/robot_devices/robots/configs/so101`
2. `https://github.com/huggingface/lerobot/tree/main/lerobot/common/assets/so101`
3. `https://github.com/TheRobotStudio/SO-ARM100` — SO-101 is a variant of SO-ARM100; the STL meshes and URDF may live here under an `urdf/` directory.

What to find:
- One `.urdf` file describing 6 joints named (approximately) `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.
- A set of `.stl` mesh files referenced from the URDF's `<mesh filename="..."/>` tags (typically 6–8 files — one per link).

- [ ] **Step 2: Download the URDF and all meshes**

Put them into the project:

```bash
mkdir -p dashboard/static/models/so101/meshes
# Download the URDF (replace URL with the one you found)
curl -L -o dashboard/static/models/so101/so101.urdf \
  "<URDF_RAW_URL_YOU_FOUND>"
# Download each STL the URDF references, preserving filenames
# Example — repeat for each mesh filename:
curl -L -o dashboard/static/models/so101/meshes/base_link.stl \
  "<MESH_RAW_URL>"
```

Look at `dashboard/static/models/so101/so101.urdf` after download to enumerate the exact mesh filenames you need; each `<mesh filename="..."/>` element names one.

- [ ] **Step 3: Normalize mesh paths in the URDF**

Open `dashboard/static/models/so101/so101.urdf`. Mesh references often use either `package://so101/meshes/foo.stl` or `../meshes/foo.stl`. `URDFLoader.js` can handle both but needs hints. We'll standardize to plain relative paths.

Find every `<mesh filename="..."/>` and rewrite to:
```xml
<mesh filename="meshes/<filename>.stl"/>
```

So if the original was `<mesh filename="package://so101/meshes/base_link.stl"/>`, it becomes `<mesh filename="meshes/base_link.stl"/>`.

- [ ] **Step 4: Verify all referenced meshes exist**

Run this from the repo root:

```bash
python -c "
import re, pathlib
urdf = pathlib.Path('dashboard/static/models/so101/so101.urdf').read_text()
refs = re.findall(r'<mesh filename=\"([^\"]+)\"/>', urdf)
base = pathlib.Path('dashboard/static/models/so101')
missing = [r for r in refs if not (base / r).exists()]
print(f'{len(refs)} mesh refs; {len(missing)} missing')
for m in missing: print('  MISSING:', m)
"
```

Expected: `<N> mesh refs; 0 missing`. If any are missing, download them.

- [ ] **Step 5: Commit**

```bash
git add dashboard/static/models/so101/
git commit -m "$(cat <<'EOF'
Vendor SO-101 URDF and STL meshes from lerobot

Canonical robot description for the 3D sim. Mesh paths normalized to
repo-relative form so URDFLoader can resolve them without package://
prefixes.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Seed `kinematics_calibration.json`

**Files:**
- Create: `kinematics_calibration.json`

**Why this task exists:** URDF speaks radians; servos speak ticks. This file owns the two-way conversion, plus joint limits in tick space (so the Python server can enforce limits without parsing URDF).

- [ ] **Step 1: Verify URDF joint names**

Open `dashboard/static/models/so101/so101.urdf` and note the six `<joint name="..."/>` values. The template below uses `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`. If the URDF uses different names (common variants: `joint_1`, `joint_2`, ...; or `wrist_pitch` instead of `wrist_flex`), update the `name` fields in the JSON below to match the URDF exactly — the browser looks up joints by name.

- [ ] **Step 2: Create the file at the repo root**

Write `kinematics_calibration.json` with this content (update `name` values if Step 1 showed different URDF names). The `tick_zero` values assume each servo is mounted per the SO-101 assembly guide and centered at a 2048-tick midpoint. The `tick_min`/`tick_max` reflect a safe working range.

```json
{
  "tick_per_rad": 651.8986469044033,
  "joints": [
    {"name": "shoulder_pan",  "id": 1, "tick_zero": 2048, "tick_sign":  1, "tick_min":  512, "tick_max": 3584},
    {"name": "shoulder_lift", "id": 2, "tick_zero": 2048, "tick_sign":  1, "tick_min":  512, "tick_max": 3584},
    {"name": "elbow_flex",    "id": 3, "tick_zero": 2048, "tick_sign":  1, "tick_min":  512, "tick_max": 3584},
    {"name": "wrist_flex",    "id": 4, "tick_zero": 2048, "tick_sign":  1, "tick_min":  512, "tick_max": 3584},
    {"name": "wrist_roll",    "id": 6, "tick_zero": 2048, "tick_sign":  1, "tick_min":    0, "tick_max": 4095},
    {"name": "gripper",       "id": 5, "tick_zero": 2048, "tick_sign":  1, "tick_min": 1500, "tick_max": 2700}
  ]
}
```

Note the ID order matches the current `FOLLOWER_IDS = [1, 2, 3, 4, 6, 5]` — motors 5 and 6 are physically swapped in this lab rig.

- [ ] **Step 3: Smoke-test the file parses and has the expected shape**

```bash
python -c "
import json
d = json.load(open('kinematics_calibration.json'))
assert d['tick_per_rad'] > 651 and d['tick_per_rad'] < 652
assert len(d['joints']) == 6
ids = [j['id'] for j in d['joints']]
assert ids == [1, 2, 3, 4, 6, 5], f'order mismatch: {ids}'
for j in d['joints']:
    assert j['tick_sign'] in (-1, 1)
    assert 0 <= j['tick_min'] < j['tick_max'] <= 4095
    assert j['tick_min'] <= j['tick_zero'] <= j['tick_max']
print('OK')
"
```

Expected output: `OK`.

- [ ] **Step 4: Commit**

```bash
git add kinematics_calibration.json
git commit -m "$(cat <<'EOF'
Seed kinematics_calibration.json with default tick mapping

Maps URDF joint angles to servo ticks per motor (tick_zero, tick_sign,
tick_min, tick_max). Defaults assume SO-101 assembly guide mounting;
per-rig values will be written by calibrate_home.py.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7 (Phase 1 completion — out of numerical order): Browser ghost arm

This task depends only on Tasks 1–3 and uses the existing `/api/state` endpoint. It can be validated without any of the Phase 2 backend work. Done at this point, the project has a working 3D ghost view.

**Files:**
- Create: `dashboard/static/sim.js`
- Modify: `dashboard/static/index.html`
- Modify: `dashboard/server.py` (new routes for sim.js, vendor, models, calibration.json)

- [ ] **Step 1: Add the 3D panel container and script tags to `index.html`**

Open `dashboard/static/index.html`. Find the `<div class="wrap">` opening. Directly after the `<header>` element and BEFORE the first `<section>` (the leader table), insert:

```html
  <section class="sim-panel">
    <div id="viewport"></div>
    <div id="sim-status" class="sim-status">loading model…</div>
  </section>
```

Then at the END of the `<body>` (just before `</body>`), insert these five script tags BEFORE the existing inline `<script>` that contains `tick()`:

```html
  <script src="/vendor/three.min.js"></script>
  <script src="/vendor/OrbitControls.js"></script>
  <script src="/vendor/STLLoader.js"></script>
  <script src="/vendor/URDFLoader.js"></script>
  <script src="/sim.js"></script>
```

Add these CSS rules inside the existing `<style>` block (grouped near the other `section` rules is tidier):

```css
.sim-panel {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 0;
  overflow: hidden;
  position: relative;
  margin-bottom: 24px;
}
#viewport {
  width: 100%;
  height: 400px;
  display: block;
}
.sim-status {
  position: absolute;
  top: 8px;
  left: 10px;
  color: var(--dim);
  font-size: 11px;
  pointer-events: none;
}
.sim-status.err { color: var(--err); }
.wrap { max-width: 940px; }
```

(Note the `.wrap` max-width update from 760 to 940.)

- [ ] **Step 2: Update `dashboard/server.py` to serve the new paths**

The existing handler only routes `/` and `/api/state`. Sim.js and the vendor/models directories need to be served too.

Open `dashboard/server.py`. Replace the `do_GET` method of `_Handler` with:

```python
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file("index.html", "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._serve_json(state.snapshot())
        elif self.path == "/sim.js":
            self._serve_file("sim.js", "application/javascript; charset=utf-8")
        elif self.path == "/calibration.json":
            self._serve_calibration()
        elif self.path.startswith("/vendor/"):
            self._serve_under("vendor", self.path[len("/vendor/"):],
                              "application/javascript; charset=utf-8")
        elif self.path.startswith("/models/"):
            self._serve_model(self.path[len("/models/"):])
        else:
            self.send_error(404)
```

Add these three helpers to the `_Handler` class (directly under `_serve_json`):

```python
    def _serve_under(self, subdir, relpath, content_type):
        # Path-traversal guard: resolve and confirm still inside the subdir.
        base = (_STATIC_DIR / subdir).resolve()
        target = (base / relpath).resolve()
        if not str(target).startswith(str(base)):
            self.send_error(403)
            return
        try:
            data = target.read_bytes()
        except FileNotFoundError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_model(self, relpath):
        ct = "application/octet-stream"
        if relpath.endswith(".urdf"):
            ct = "application/xml; charset=utf-8"
        elif relpath.endswith(".stl"):
            ct = "model/stl"
        self._serve_under("models", relpath, ct)

    def _serve_calibration(self):
        # Lives at the repo root (one level above dashboard/).
        path = Path(__file__).resolve().parent.parent / "kinematics_calibration.json"
        try:
            data = path.read_bytes()
        except FileNotFoundError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)
```

- [ ] **Step 3: Emit a `so101:state` custom event from the existing poll**

In `dashboard/static/index.html`'s existing inline `<script>`, find the `async function tick()` function. Inside the `try` block, right after `lastOk = Date.now();` and BEFORE the `const leader = document.getElementById...` line, add:

```javascript
    window.dispatchEvent(new CustomEvent('so101:state', { detail: data }));
```

This preserves the existing table rendering and just notifies `sim.js` whenever fresh state arrives.

- [ ] **Step 4: Create `dashboard/static/sim.js` — Phase 1 content (ghost only, no gizmo)**

Create the file with exactly this content:

```javascript
// sim.js — SO-101 3D viewport. Phase 1: ghost arm only, driven by /api/state.
(function () {
  const VIEW_EL = document.getElementById('viewport');
  const STATUS_EL = document.getElementById('sim-status');

  function setStatus(text, isErr) {
    STATUS_EL.textContent = text;
    STATUS_EL.classList.toggle('err', !!isErr);
  }

  // --- three.js scene ----------------------------------------------------
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1115);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
  camera.position.set(0.4, 0.3, 0.5);
  camera.up.set(0, 0, 1); // Z-up matches URDF convention

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x0f1115, 1);
  VIEW_EL.appendChild(renderer.domElement);

  function sizeToContainer() {
    const w = VIEW_EL.clientWidth;
    const h = VIEW_EL.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  sizeToContainer();
  window.addEventListener('resize', sizeToContainer);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.15);
  controls.update();

  // Lights
  const hemi = new THREE.HemisphereLight(0xffffff, 0x222233, 0.9);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(0.5, 0.5, 1);
  scene.add(dir);

  // Ground plane grid
  const grid = new THREE.GridHelper(1, 10, 0x262b35, 0x161a21);
  grid.rotation.x = Math.PI / 2; // align with Z-up world
  scene.add(grid);

  // --- URDF load ---------------------------------------------------------
  const loader = new URDFLoader();
  loader.loadMeshCb = function (path, manager, onLoad) {
    const stl = new THREE.STLLoader(manager);
    stl.load(path, function (geom) {
      const mat = new THREE.MeshStandardMaterial({
        color: 0x7b8496,
        transparent: true,
        opacity: 0.35,
        metalness: 0.1,
        roughness: 0.9,
      });
      const mesh = new THREE.Mesh(geom, mat);
      onLoad(mesh);
    });
  };

  let ghostRobot = null;
  let calibration = null;

  loader.load('/models/so101/so101.urdf', function (robot) {
    ghostRobot = robot;
    scene.add(robot);
    setStatus('ghost live');
    window.SIM = { ghostRobot };
  }, null, function (err) {
    console.error('URDF load failed', err);
    setStatus('Failed to load SO-101 model. Check dashboard/static/models/so101/.', true);
  });

  // --- Calibration fetch + telemetry -> ghost pose ----------------------
  fetch('/calibration.json').then(function (r) { return r.json(); })
                            .then(function (c) { calibration = c; });

  function ticksToRad(tick, j) {
    return ((tick - j.tick_zero) / calibration.tick_per_rad) * j.tick_sign;
  }

  function applyFollowerPose(motorsById) {
    if (!ghostRobot || !calibration) return;
    calibration.joints.forEach(function (j) {
      const motor = motorsById[j.id];
      if (!motor || motor.pos === null || motor.pos === undefined) return;
      const rad = ticksToRad(motor.pos, j);
      const urdfJoint = ghostRobot.joints[j.name];
      if (urdfJoint) urdfJoint.setJointValue(rad);
    });
  }

  window.addEventListener('so101:state', function (ev) {
    const data = ev.detail;
    const byId = {};
    data.motors.filter(function (m) { return m.role === 'follower'; })
               .forEach(function (m) { byId[m.id] = m; });
    applyFollowerPose(byId);
  });

  // --- Render loop -------------------------------------------------------
  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
})();
```

- [ ] **Step 5: Validate — no rig required**

From the repo root:

```bash
python -c "
from dashboard import state, server
import time
state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
for mid in [1,2,3,4,6,5,7,8,9,10,11,12]:
    state.update(mid, pos=2048, temp=30, volt=11.9, load=5)
server.start(port=8099)
print('Open http://127.0.0.1:8099 — Ctrl+C to stop')
time.sleep(1800)
"
```

Open `http://127.0.0.1:8099`. Verify:

1. The 3D panel renders a dark scene with a grid.
2. Status text reads `ghost live` (not `loading model…` indefinitely, not `Failed to load...`).
3. The SO-101 ghost arm is visible (semi-transparent gray).
4. Right-click-drag orbits the camera; scroll zooms.
5. The leader and follower tables below still render as before.

Kill the server, re-run with a bent joint to validate pose mapping:

```bash
python -c "
from dashboard import state, server
import time
state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
for mid in [1,2,3,4,6,5,7,8,9,10,11,12]: state.update(mid, pos=2048)
# Bend shoulder_lift by ~30 degrees
state.update(2, pos=2048 + int(30 * 3.14159 / 180 * 651.9))
server.start(port=8099)
time.sleep(1800)
"
```

Reload the browser. The shoulder_lift joint should be visibly bent relative to the zero pose.

- [ ] **Step 6: Full-rig validation**

Run `python follower_sees_follower_does.py`. Press ENTER at the sync prompt. Open `http://127.0.0.1:8080` in a browser.

1. Ghost arm renders in approximately the same pose as the real follower. Approximately — default `tick_zero` values are generic; the calibration script in Task 11 tightens this.
2. Wiggle the leader arm; real follower follows; ghost in browser also tracks (with ~1 s lag — the poll rate).
3. The leader and follower tables update as before.

If the ghost is wildly misaligned (joints moving backwards), note it but don't fix now — Task 11 handles per-rig calibration.

- [ ] **Step 7: Commit**

```bash
git add dashboard/static/sim.js dashboard/static/index.html dashboard/server.py
git commit -m "$(cat <<'EOF'
Add 3D ghost arm rendered from live telemetry (Phase 1)

three.js viewport above the existing tables. Loads the vendored SO-101
URDF and meshes, renders a semi-transparent ghost arm whose joint angles
track follower telemetry at 1 Hz. Server gains routes for sim.js, vendor
files, model files, and calibration.json.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

**End of Phase 1.** The dashboard now has a 3D view. The arm visual is read-only. Phase 2 adds control.

---

## Task 4: `dashboard/control.py` — control state module

**Files:**
- Create: `dashboard/control.py`

- [ ] **Step 1: Write the module**

Create `dashboard/control.py` with exactly this content:

```python
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
    """Switch to sim mode. Returns (ok, message). Returns (False, ...) if
    already engaged."""
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
    """Update sim_targets. Clips each value to its joint's tick limits.
    Returns (ok, message)."""
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
```

- [ ] **Step 2: Smoke-test the module's core logic without the rig**

```bash
python -c "
from dashboard import control
import time

control.init([2000, 2100, 1900, 2048, 2048, 2000])

# Physical pass-through
assert control.next_follower_targets([2001, 2100, 1900, 2048, 2048, 2000], time.time()) == [2001, 2100, 1900, 2048, 2048, 2000]
assert not control.consume_force_release()

# Engage
ok, _ = control.engage()
assert ok
ok, _ = control.engage()
assert not ok  # already engaged

# Target + velocity clamp
ok, _ = control.set_target([2500, 2100, 1900, 2048, 2048, 2000])
assert ok
now = time.time()
control.heartbeat()
out = control.next_follower_targets([9999]*6, now)
assert out[0] == 2101, f'expected 2101 got {out[0]}'  # 2001 + 100 clamp
assert out[1] == 2100

# Watchdog force-release
out = control.next_follower_targets([9999]*6, now + 10)
assert control.consume_force_release()
assert out == [9999]*6

print('OK')
"
```

Expected: `OK`.

Note: on the first `next_follower_targets` call in sim mode, `last_written[0]` was set to 2001 by the physical pass-through (not 2000 from `init`). The clamp math is 2001 + 100 = 2101. If this differs in your output, inspect control.snapshot() to find where the drift came from.

- [ ] **Step 3: Commit**

```bash
git add dashboard/control.py
git commit -m "$(cat <<'EOF'
Add control.py — sim-mode state, velocity clamp, heartbeat watchdog

Lock-guarded module owning (mode, sim_targets, last_heartbeat,
last_written). No serial I/O — the main loop acts on
consume_force_release() to blip torque. MAX_DELTA_TICKS=100 per iter,
HEARTBEAT_TIMEOUT_S=1.5.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Extend `dashboard/server.py` with POST endpoints

**Files:**
- Modify: `dashboard/server.py`

- [ ] **Step 1: Add POST routing and the four handler methods**

Open `dashboard/server.py`. At the top, alongside `from dashboard import state`, add:

```python
from dashboard import control
```

Add a `do_POST` method to `_Handler` (placed right after `do_GET`):

```python
    def do_POST(self):
        if self.path == "/api/engage":
            self._handle_engage()
        elif self.path == "/api/release":
            self._handle_release()
        elif self.path == "/api/target":
            self._handle_target()
        elif self.path == "/api/heartbeat":
            self._handle_heartbeat()
        else:
            self.send_error(404)

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return None

    def _reply_json(self, status, obj):
        data = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _handle_engage(self):
        ok, msg = control.engage()
        self._reply_json(200 if ok else 409, {"ok": ok, "message": msg})

    def _handle_release(self):
        control.release()
        self._reply_json(200, {"ok": True, "message": "released"})

    def _handle_target(self):
        body = self._read_json()
        if body is None or "joints" not in body:
            self._reply_json(400, {"ok": False, "message": "bad body"})
            return
        ok, msg = control.set_target(body["joints"])
        self._reply_json(200 if ok else 400, {"ok": ok, "message": msg})

    def _handle_heartbeat(self):
        control.heartbeat()
        self._reply_json(200, {"ok": True})
```

- [ ] **Step 2: Smoke-test all four endpoints via urllib**

```bash
python -c "
from dashboard import state, server, control
import time, urllib.request, urllib.error, json

state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
control.init([2048]*6)
server.start(port=8099)
time.sleep(0.2)

def post(path, body=None):
    data = json.dumps(body).encode() if body is not None else b''
    req = urllib.request.Request(f'http://127.0.0.1:8099{path}', data=data, method='POST',
                                 headers={'Content-Type': 'application/json'})
    try:
        return urllib.request.urlopen(req).read().decode(), 200
    except urllib.error.HTTPError as e:
        return e.read().decode(), e.code

print('engage 1:', post('/api/engage'))
print('engage 2:', post('/api/engage'))
print('target ok:', post('/api/target', {'joints':[2100,2048,2048,2048,2048,2048]}))
print('target bad:', post('/api/target', {'joints':[1,2,3]}))
print('heartbeat:', post('/api/heartbeat'))
print('release:', post('/api/release'))
print('target after release:', post('/api/target', {'joints':[2100]*6}))
print('snapshot:', control.snapshot())
"
```

Expected output: `engage 1` returns 200/ok:true; `engage 2` returns 409; `target ok` returns 200; `target bad` returns 400; `heartbeat` returns 200; `release` returns 200; `target after release` returns 400. `snapshot` shows `mode: physical`.

- [ ] **Step 3: Commit**

```bash
git add dashboard/server.py
git commit -m "$(cat <<'EOF'
Add POST endpoints for sim control (engage/release/target/heartbeat)

All routes return JSON. 409 on re-engage, 400 on bad target body or
not-engaged target, 200 otherwise. Reads control module state; does
not touch serial.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Wire `control.py` + two-stage SIGINT into the main script

**Files:**
- Modify: `follower_sees_follower_does.py`

- [ ] **Step 1: Add imports**

At the top of the file, alongside the existing dashboard imports, add:

```python
from dashboard import control as dash_control
import signal
import sys
```

- [ ] **Step 2: Install the two-stage SIGINT handler (above the `try:` block)**

Directly above the `try:` line that starts with `print("Opening High-Speed Ports...")`, insert:

```python
_interrupt_count = 0
_interrupt_ts = 0.0


def _sigint_handler(signum, frame):
    """Two-stage Ctrl+C. First press: if sim-engaged, release and freeze
    at current position; keep torque on. Second press within 5 s: full
    shutdown."""
    global _interrupt_count, _interrupt_ts
    now = time.time()
    if now - _interrupt_ts > 5.0:
        _interrupt_count = 0
    _interrupt_count += 1
    _interrupt_ts = now
    if _interrupt_count == 1:
        dash_control.release()
        print("\nMotion frozen. Press Ctrl+C again within 5 s to exit.")
        return
    # Second press — full shutdown.
    print("\nStopping...")
    try:
        for sid in FOLLOWER_IDS:
            write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass
    try: l_ser.close()
    except Exception: pass
    try: f_ser.close()
    except Exception: pass
    sys.exit(0)


signal.signal(signal.SIGINT, _sigint_handler)
```

- [ ] **Step 3: Replace the `except KeyboardInterrupt` block**

Scroll to the bottom of the file. Find:

```python
except KeyboardInterrupt:
    print("\nStopping...")
    for sid in FOLLOWER_IDS: write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    l_ser.close()
    f_ser.close()
```

Replace with:

```python
except Exception as e:
    print(f"\nUnexpected error: {e}")
    try:
        for sid in FOLLOWER_IDS: write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass
    try: l_ser.close()
    except Exception: pass
    try: f_ser.close()
    except Exception: pass
    raise
```

SIGINT is now handled by the signal handler (which calls `sys.exit`), not by an `except KeyboardInterrupt`. This `except Exception` catches other unexpected errors and still cleans up.

- [ ] **Step 4: Initialize control after state.init**

Find:

```python
    dash_state.init(LEADER_IDS, FOLLOWER_IDS)
    dash_server.start(host="127.0.0.1", port=8080)
```

Replace with:

```python
    dash_state.init(LEADER_IDS, FOLLOWER_IDS)
    dash_control.init([2048] * 6)  # re-seated after the lock phase
    dash_server.start(host="127.0.0.1", port=8080)
```

Then find the "Locking..." for-loop. After that for-loop finishes (where `sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)` sits outside the for-loop), add:

```python
    # Now that follower_targets reflect real positions, re-seat control.
    dash_control.init(list(follower_targets))
```

- [ ] **Step 5: Route the sync-write through `control.next_follower_targets`**

In the main `while True:` loop, find:

```python
        # 2. WRITE PHASE (One Packet for All)
        if update_needed:
            sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)
            for i in range(6):
                dash_state.update(FOLLOWER_IDS[i], goal=int(follower_targets[i]))
```

Replace with:

```python
        # 2. WRITE PHASE — route through control so sim mode can override
        # and safety clamps apply.
        final_targets = dash_control.next_follower_targets(
            list(follower_targets), time.time())
        if dash_control.consume_force_release():
            print("[control] heartbeat lost — released to physical")
            for sid in FOLLOWER_IDS:
                write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
            for i in range(6):
                fid = FOLLOWER_IDS[i]
                fp = read_position_robust(f_ser, fid)
                if fp is not None:
                    final_targets[i] = fp
                    follower_targets[i] = fp
                write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)
            dash_control.init(list(final_targets))
        # In sim mode, final_targets may differ from follower_targets every
        # iteration even without leader motion, so always write.
        sync_write_positions(f_ser, FOLLOWER_IDS, final_targets)
        for i in range(6):
            dash_state.update(FOLLOWER_IDS[i], goal=int(final_targets[i]))
```

- [ ] **Step 6: Syntax check**

```bash
python -c "import ast; ast.parse(open('follower_sees_follower_does.py').read()); print('syntax OK')"
```

Expected: `syntax OK`.

- [ ] **Step 7: Live-rig validation**

```bash
python follower_sees_follower_does.py
```

- Ports open; `[dashboard] serving on http://127.0.0.1:8080`; sync prompt.
- Press ENTER. Teleop runs as before.
- Press Ctrl+C once → `Motion frozen. Press Ctrl+C again within 5 s to exit.`
- Wait 6 s → the counter resets silently; press Ctrl+C → same "frozen" message again.
- Press Ctrl+C twice in quick succession → `Stopping...` and clean exit with follower torque off.

Re-run. In another terminal: `curl -X POST http://127.0.0.1:8080/api/engage`. Expect 200 with `ok:true`. Within 1.5 s the teleop terminal should print `[control] heartbeat lost — released to physical` (because no heartbeats follow). Leader control resumes.

- [ ] **Step 8: Commit**

```bash
git add follower_sees_follower_does.py
git commit -m "$(cat <<'EOF'
Wire control module into teleop loop + two-stage Ctrl+C handler

Sync-writes now flow through control.next_follower_targets(), which
picks physical_targets or velocity-clamped sim_targets. Watchdog
force-release blips torque and re-seats from current positions.
SIGINT handler replaces the old except block: first press freezes
and releases sim mode, second within 5 s exits cleanly.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Browser — end-effector handle + CCD IK + target arm

**Files:**
- Modify: `dashboard/static/sim.js`

Builds on Task 7's sim.js. Adds a second URDF instance (target arm, solid), a draggable sphere at the end-effector, and an inline CCD IK solver. No POSTing yet — this step validates the math in isolation.

- [ ] **Step 1: Extend `sim.js` with target arm, handle, and IK**

Open `dashboard/static/sim.js`. Replace the entire `loader.load('/models/so101/so101.urdf', function (robot) { ... })` block with:

```javascript
  function tintRobot(robot, opts) {
    robot.traverse(function (n) {
      if (n.isMesh) {
        n.material = new THREE.MeshStandardMaterial({
          color: opts.color,
          transparent: opts.transparent,
          opacity: opts.opacity,
          metalness: 0.1,
          roughness: 0.9,
        });
      }
    });
  }

  let ghostRobot = null;
  let targetRobot = null;
  let endEffectorHandle = null;
  const JOINT_ORDER = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"];

  loader.load('/models/so101/so101.urdf', function (robot) {
    ghostRobot = robot;
    tintRobot(ghostRobot, { color: 0x7b8496, transparent: true, opacity: 0.35 });
    scene.add(ghostRobot);

    loader.load('/models/so101/so101.urdf', function (robot2) {
      targetRobot = robot2;
      tintRobot(targetRobot, { color: 0xd8dde5, transparent: false, opacity: 1.0 });
      targetRobot.visible = false;
      scene.add(targetRobot);

      endEffectorHandle = new THREE.Mesh(
        new THREE.SphereGeometry(0.015, 16, 12),
        new THREE.MeshStandardMaterial({
          color: 0x4c8df6, emissive: 0x4c8df6, emissiveIntensity: 0.2
        })
      );
      endEffectorHandle.visible = false;
      scene.add(endEffectorHandle);
      window.SIM = { ghostRobot, targetRobot, endEffectorHandle };
      setStatus('ghost live');
    });
  }, null, function (err) {
    console.error('URDF load failed', err);
    setStatus('Failed to load SO-101 model. Check dashboard/static/models/so101/.', true);
  });
```

- [ ] **Step 2: Add the end-effector-world helper (above the loader.load)**

Insert near the top of the IIFE, before the `loader.load(...)` call:

```javascript
  function endEffectorWorld(robot) {
    const candidates = ["gripper_link", "wrist_roll_link", "end_effector_link"];
    for (const name of candidates) {
      if (robot.links && robot.links[name]) {
        const v = new THREE.Vector3();
        robot.links[name].getWorldPosition(v);
        return v;
      }
    }
    const names = Object.keys(robot.links || {});
    const last = robot.links[names[names.length - 1]];
    const v = new THREE.Vector3();
    if (last) last.getWorldPosition(v);
    return v;
  }
```

If none of the link names match your vendored URDF, open the URDF and pick the outermost link (usually named after the gripper or end-effector), and add it to the `candidates` array.

- [ ] **Step 3: Add the CCD IK solver**

Append inside the IIFE, below the loader callback:

```javascript
  function ccdSolve(robot, targetPos, iterations) {
    if (!robot) return 0;
    const ee = new THREE.Vector3();
    const jointPos = new THREE.Vector3();
    const axisWorld = new THREE.Vector3();
    const toEe = new THREE.Vector3();
    const toTarget = new THREE.Vector3();

    for (let iter = 0; iter < iterations; iter++) {
      for (let ji = JOINT_ORDER.length - 1; ji >= 0; ji--) {
        const name = JOINT_ORDER[ji];
        const joint = robot.joints[name];
        if (!joint) continue;

        ee.copy(endEffectorWorld(robot));
        joint.getWorldPosition(jointPos);
        toEe.copy(ee).sub(jointPos);
        toTarget.copy(targetPos).sub(jointPos);
        if (toEe.lengthSq() < 1e-10 || toTarget.lengthSq() < 1e-10) continue;
        toEe.normalize(); toTarget.normalize();

        axisWorld.copy(joint.axis).transformDirection(joint.matrixWorld);

        const eeDot = toEe.dot(axisWorld);
        const tgDot = toTarget.dot(axisWorld);
        const eeProj = toEe.clone().addScaledVector(axisWorld, -eeDot).normalize();
        const tgProj = toTarget.clone().addScaledVector(axisWorld, -tgDot).normalize();

        let angle = Math.acos(Math.min(1, Math.max(-1, eeProj.dot(tgProj))));
        const cross = new THREE.Vector3().crossVectors(eeProj, tgProj);
        if (cross.dot(axisWorld) < 0) angle = -angle;
        if (!isFinite(angle) || Math.abs(angle) < 1e-5) continue;

        const newVal = joint.angle + angle;
        const lo = joint.limit && typeof joint.limit.lower === 'number' ? joint.limit.lower : -Math.PI;
        const hi = joint.limit && typeof joint.limit.upper === 'number' ? joint.limit.upper :  Math.PI;
        joint.setJointValue(Math.max(lo, Math.min(hi, newVal)));
      }
    }

    ee.copy(endEffectorWorld(robot));
    return ee.distanceTo(targetPos);
  }
```

- [ ] **Step 4: Add click-drag raycaster for the end-effector handle**

Append inside the IIFE:

```javascript
  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  let dragging = false;
  const dragPlane = new THREE.Plane();
  const dragPoint = new THREE.Vector3();
  let ikResidual = 0;

  function setMouseNDC(ev) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndc.x =  ((ev.clientX - rect.left) / rect.width)  * 2 - 1;
    ndc.y = -((ev.clientY - rect.top)  / rect.height) * 2 + 1;
  }

  renderer.domElement.addEventListener('pointerdown', function (ev) {
    if (!endEffectorHandle || !endEffectorHandle.visible) return;
    setMouseNDC(ev);
    raycaster.setFromCamera(ndc, camera);
    const hits = raycaster.intersectObject(endEffectorHandle, false);
    if (hits.length > 0) {
      dragging = true;
      controls.enabled = false;
      dragPlane.setFromNormalAndCoplanarPoint(
        camera.getWorldDirection(new THREE.Vector3()).negate(),
        endEffectorHandle.position.clone());
      renderer.domElement.setPointerCapture(ev.pointerId);
      ev.preventDefault();
    }
  });

  renderer.domElement.addEventListener('pointermove', function (ev) {
    if (!dragging) return;
    setMouseNDC(ev);
    raycaster.setFromCamera(ndc, camera);
    if (raycaster.ray.intersectPlane(dragPlane, dragPoint)) {
      ikResidual = ccdSolve(targetRobot, dragPoint, 30);
      const ee = endEffectorWorld(targetRobot);
      endEffectorHandle.position.copy(ee);
      const color = (ikResidual > 0.02) ? 0xe5484d : 0x4c8df6;
      endEffectorHandle.material.color.setHex(color);
      endEffectorHandle.material.emissive.setHex(color);
    }
  });

  renderer.domElement.addEventListener('pointerup', function (ev) {
    if (!dragging) return;
    dragging = false;
    controls.enabled = true;
    try { renderer.domElement.releasePointerCapture(ev.pointerId); } catch (e) {}
  });
```

- [ ] **Step 5: Keep handle glued to the target arm each frame**

In the `animate()` function, right before `renderer.render(...)`, add:

```javascript
    if (endEffectorHandle && endEffectorHandle.visible && targetRobot) {
      endEffectorHandle.position.copy(endEffectorWorld(targetRobot));
    }
```

- [ ] **Step 6: Developer-flag visibility for this task's standalone validation**

Add this one line at the end of the inner `loader.load` callback (right after `setStatus('ghost live');`), to be **removed in Task 9**:

```javascript
      targetRobot.visible = true; endEffectorHandle.visible = true;
```

- [ ] **Step 7: Validate — browser only, no rig needed**

```bash
python -c "
from dashboard import state, server, control
import time
state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
control.init([2048]*6)
for mid in [1,2,3,4,6,5,7,8,9,10,11,12]: state.update(mid, pos=2048)
server.start(port=8099)
time.sleep(1800)
"
```

Open `http://127.0.0.1:8099`. Verify:

1. Ghost arm renders (semi-transparent gray).
2. Target arm renders on top (solid, opaque, same pose).
3. Blue sphere visible at the end-effector.
4. Click-drag the sphere: target arm's joints track the cursor; sphere stays glued to the end-effector.
5. Drag far: sphere turns red (unreachable); arm extends to its limit.
6. Drag back within reach: sphere returns to blue.
7. Click-drag empty space: orbits camera (arm does not move).

If the handle never appears, check the browser console for URDFLoader errors or an end-effector link-name mismatch (update the `candidates` array in Step 2).

- [ ] **Step 8: Commit**

```bash
git add dashboard/static/sim.js
git commit -m "$(cat <<'EOF'
Add target arm, end-effector handle, CCD IK (browser-only)

Click-drag the blue sphere to move the target arm via IK. Residual-
distance threshold drives red/blue coloring. No POSTing yet — backend
wiring lands in the next task.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Engage/Release button + heartbeat + target POSTing

**Files:**
- Modify: `dashboard/static/sim.js`
- Modify: `dashboard/static/index.html` — add button + side panel

- [ ] **Step 1: Rework the `.sim-panel` section in `index.html`**

Find:

```html
  <section class="sim-panel">
    <div id="viewport"></div>
    <div id="sim-status" class="sim-status">loading model…</div>
  </section>
```

Replace with:

```html
  <section class="sim-panel">
    <div class="sim-layout">
      <div class="sim-viewport-wrap">
        <div id="viewport"></div>
        <div id="sim-status" class="sim-status">loading model…</div>
      </div>
      <aside class="sim-side">
        <label>Wrist pitch <input id="slider-wrist-pitch" type="range" min="0" max="4095" value="2048"></label>
        <label>Wrist roll  <input id="slider-wrist-roll"  type="range" min="0" max="4095" value="2048"></label>
        <label>Gripper     <input id="slider-gripper"     type="range" min="1500" max="2700" value="2048"></label>
        <button id="engage-btn" class="engage" disabled>Engage</button>
      </aside>
    </div>
  </section>
```

Add these CSS rules in the existing `<style>`:

```css
.sim-layout { display: flex; gap: 12px; }
.sim-viewport-wrap { flex: 1; position: relative; }
.sim-side {
  width: 180px; padding: 12px; display: flex; flex-direction: column;
  gap: 12px; background: #1a1e26; border-left: 1px solid var(--border);
}
.sim-side label { color: var(--dim); font-size: 11px; text-transform: uppercase; }
.sim-side input[type="range"] { width: 100%; }
.engage {
  padding: 10px; border-radius: 4px; border: none; color: white;
  font: inherit; font-weight: 600; letter-spacing: 0.3px; cursor: pointer;
  background: #3a8f3a;
}
.engage.active { background: #c24141; }
.engage:disabled { background: #444; cursor: not-allowed; }
```

- [ ] **Step 2: Remove the temporary visibility hack**

Open `dashboard/static/sim.js`. Remove the line added in Task 8 Step 6:

```javascript
      targetRobot.visible = true; endEffectorHandle.visible = true;
```

Then, in the SAME inner `loader.load` callback (right after `setStatus('ghost live');` and BEFORE the closing `});`), add:

```javascript
      document.getElementById('engage-btn').disabled = false;
```

And update the OUTER error callback to also disable the button (belt-and-braces):

```javascript
  }, null, function (err) {
    console.error('URDF load failed', err);
    setStatus('Failed to load SO-101 model. Check dashboard/static/models/so101/.', true);
    document.getElementById('engage-btn').disabled = true;
  });
```

- [ ] **Step 3: Track last-follower positions by joint name**

Replace the existing `window.addEventListener('so101:state', ...)` block in `sim.js` with:

```javascript
  let lastFollowerByName = {};
  window.addEventListener('so101:state', function (ev) {
    const data = ev.detail;
    const byId = {};
    data.motors.filter(function (m) { return m.role === 'follower'; })
               .forEach(function (m) { byId[m.id] = m; });
    applyFollowerPose(byId);
    if (calibration) {
      lastFollowerByName = {};
      calibration.joints.forEach(function (j) {
        const m = byId[j.id];
        if (m && typeof m.pos === 'number') lastFollowerByName[j.name] = m.pos;
      });
    }
  });
```

- [ ] **Step 4: Append API client, heartbeat, target POST, engage handler**

Append inside the IIFE, below the drag handlers and above `animate()`:

```javascript
  let engaged = false;
  let heartbeatTimer = null;
  let postInFlight = false;
  let pendingTargetTicks = null;
  const engageBtn = document.getElementById('engage-btn');

  async function post(path, body) {
    const r = await fetch(path, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : null,
    });
    const text = await r.text();
    let data = {};
    try { data = JSON.parse(text); } catch (e) {}
    return { ok: r.ok, status: r.status, data };
  }

  function radToTick(rad, j) {
    return Math.round(j.tick_zero + j.tick_sign * rad * calibration.tick_per_rad);
  }

  function currentSimTargetTicks() {
    if (!targetRobot || !calibration) return null;
    const bySlider = {
      wrist_flex: +document.getElementById('slider-wrist-pitch').value,
      wrist_roll: +document.getElementById('slider-wrist-roll').value,
      gripper:    +document.getElementById('slider-gripper').value,
    };
    return calibration.joints.map(function (j) {
      if (j.name in bySlider) return bySlider[j.name];
      const urdfJoint = targetRobot.joints[j.name];
      return radToTick(urdfJoint ? urdfJoint.angle : 0, j);
    });
  }

  async function pushTarget() {
    if (!engaged) return;
    const ticks = currentSimTargetTicks();
    if (!ticks) return;
    if (postInFlight) { pendingTargetTicks = ticks; return; }
    postInFlight = true;
    try {
      await post('/api/target', { joints: ticks });
    } catch (e) {
      setStatus('control offline — retrying', true);
    } finally {
      postInFlight = false;
      if (pendingTargetTicks) {
        const t = pendingTargetTicks; pendingTargetTicks = null;
        post('/api/target', { joints: t }).catch(function () {});
      }
    }
  }

  async function setEngaged(on) {
    const res = await post(on ? '/api/engage' : '/api/release', null);
    if (on && res.status === 409) {
      setStatus('another client has control', true);
      return;
    }
    if (!res.ok) {
      setStatus((on ? 'engage' : 'release') + ' failed', true);
      return;
    }
    engaged = on;
    engageBtn.textContent = on ? 'Release Control' : 'Engage';
    engageBtn.classList.toggle('active', on);
    targetRobot.visible = on;
    endEffectorHandle.visible = on;
    if (on) {
      // Seed target arm from current ghost pose so there's no jump.
      if (ghostRobot && calibration) {
        calibration.joints.forEach(function (j) {
          const urdfJoint = targetRobot.joints[j.name];
          const ghostJoint = ghostRobot.joints[j.name];
          if (urdfJoint && ghostJoint) urdfJoint.setJointValue(ghostJoint.angle);
        });
      }
      // Sync sliders to current follower positions.
      if (lastFollowerByName.wrist_flex !== undefined)
        document.getElementById('slider-wrist-pitch').value = lastFollowerByName.wrist_flex;
      if (lastFollowerByName.wrist_roll !== undefined)
        document.getElementById('slider-wrist-roll').value  = lastFollowerByName.wrist_roll;
      if (lastFollowerByName.gripper !== undefined)
        document.getElementById('slider-gripper').value     = lastFollowerByName.gripper;
      setStatus('sim control — heartbeat OK');
      heartbeatTimer = setInterval(function () {
        post('/api/heartbeat', null).catch(function () {});
      }, 500);
      pushTarget();
    } else {
      setStatus('ghost live');
      if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
    }
  }

  engageBtn.addEventListener('click', function () { setEngaged(!engaged); });

  ['slider-wrist-pitch', 'slider-wrist-roll', 'slider-gripper'].forEach(function (id) {
    document.getElementById(id).addEventListener('input', pushTarget);
  });
```

- [ ] **Step 5: Push target on each IK solve**

Find the `pointermove` handler. Inside the `if (raycaster.ray.intersectPlane(...))` block, after the coloring lines, add:

```javascript
      pushTarget();
```

- [ ] **Step 6: Validate — no rig needed**

```bash
python -c "
from dashboard import state, server, control
import time
state.init([7,8,9,10,11,12],[1,2,3,4,6,5])
control.init([2048]*6)
for mid in [1,2,3,4,6,5,7,8,9,10,11,12]: state.update(mid, pos=2048)
server.start(port=8099)
time.sleep(1800)
"
```

Open `http://127.0.0.1:8099`. Verify:

1. Ghost arm renders; target arm + handle hidden; status says `ghost live`.
2. Engage button is green and enabled.
3. Click Engage → button turns red, status says `sim control — heartbeat OK`, target arm and handle appear.
4. Drag the handle. In another terminal:
   ```bash
   python -c "from dashboard import control; print(control.snapshot())"
   ```
   Expect `mode: 'sim'`, `sim_targets` populated, recent `last_heartbeat`.
5. Release → button turns green, target/handle hidden, status `ghost live`.

- [ ] **Step 7: Full-rig validation**

```bash
python follower_sees_follower_does.py
```

Sync arms, press ENTER, open browser. Verify:

1. Ghost arm mirrors live follower.
2. Leader motion drives follower + ghost.
3. Engage → target arm appears at current ghost pose. Physical leader is now ignored.
4. Drag handle → real follower tracks with ~200 ms lag. Ghost updates from telemetry.
5. Slider wrist pitch/roll/gripper → corresponding motors respond immediately.
6. Release → physical leader drives again.
7. Engage, close browser tab → within 1.5 s, terminal prints `[control] heartbeat lost — released to physical`. Arm briefly blips torque at its current position.
8. Ctrl+C once while engaged → `Motion frozen.` Control released. Ctrl+C again → clean shutdown.

- [ ] **Step 8: Commit**

```bash
git add dashboard/static/sim.js dashboard/static/index.html
git commit -m "$(cat <<'EOF'
Wire sim control path end-to-end — Engage/Release, heartbeat, POST targets

Engage button toggles /api/engage and /api/release. Heartbeat pumps
every 500 ms while engaged. Drag/slider events push targets to
/api/target (coalesced via postInFlight). Button color and side-panel
values reflect live state.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: IK scope correction — wrist and gripper bypass IK

**Files:**
- Modify: `dashboard/static/sim.js`

Task 9 shipped with `JOINT_ORDER` including `wrist_flex` and `wrist_roll`, which means IK rotates those joints while the sliders also set them — visually inconsistent. Fix: restrict IK to the three arm joints; drive wrist/gripper visuals from sliders.

- [ ] **Step 1: Narrow `JOINT_ORDER`**

In `sim.js`, replace:

```javascript
  const JOINT_ORDER = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"];
```

With:

```javascript
  const JOINT_ORDER = ["shoulder_pan","shoulder_lift","elbow_flex"];
```

- [ ] **Step 2: Add slider-to-visual sync helper**

Append inside the IIFE (placement: below `currentSimTargetTicks`, above `pushTarget`):

```javascript
  function syncTargetRobotFromSliders() {
    if (!targetRobot || !calibration) return;
    function tickToRad(tick, j) {
      return ((tick - j.tick_zero) / calibration.tick_per_rad) * j.tick_sign;
    }
    [['wrist_flex','slider-wrist-pitch'],
     ['wrist_roll','slider-wrist-roll'],
     ['gripper',   'slider-gripper']].forEach(function (pair) {
      const name = pair[0], sliderId = pair[1];
      const j = calibration.joints.find(function (x) { return x.name === name; });
      if (!j) return;
      const tick = +document.getElementById(sliderId).value;
      const rad = tickToRad(tick, j);
      const urdfJoint = targetRobot.joints[name];
      if (urdfJoint) urdfJoint.setJointValue(rad);
    });
  }
```

- [ ] **Step 3: Call it from `currentSimTargetTicks` and `animate`**

At the top of `currentSimTargetTicks` (right after the `if (!targetRobot || !calibration) return null;` guard), add:

```javascript
    syncTargetRobotFromSliders();
```

In `animate()`, right after `controls.update();`, add:

```javascript
    if (engaged) syncTargetRobotFromSliders();
```

- [ ] **Step 4: Full-rig validation**

Re-run the full-rig test from Task 9 Step 7. Specifically verify:

1. Wrist-pitch slider rotates **only** the wrist pitch joint on the real follower — no shoulder/elbow motion.
2. Dragging the handle rotates **only** shoulder_pan, shoulder_lift, elbow_flex — wrist joints are frozen at the slider's value.
3. Gripper slider opens/closes the gripper without moving anything else.

- [ ] **Step 5: Commit**

```bash
git add dashboard/static/sim.js
git commit -m "$(cat <<'EOF'
Restrict IK to shoulder+elbow; wrist/gripper are slider-direct

Previously JOINT_ORDER included wrist joints, causing visual/control
conflicts when sliders set ticks that disagreed with IK rad values.
IK now solves 3 DOFs (position-only); wrist pitch/roll and gripper
are driven from sliders; target arm's wrist joints are visually
synced from slider values for consistency.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 11: `calibrate_home.py` — per-rig tick_zero recorder

**Files:**
- Create: `calibrate_home.py`

- [ ] **Step 1: Write the script**

Create `calibrate_home.py` at the repo root with exactly this content. Uses `ast.literal_eval` to read config from the main script safely (no `exec`).

```python
"""One-time per-rig calibration: record the current follower servo
positions as the URDF "home pose" tick_zero values.

Usage:
  1. Stop any running follower_sees_follower_does.py.
  2. With the follower powered on, manually move the arm into the URDF
     home pose (all URDF joint angles = 0 — see README for a description).
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


def _load_config():
    """Extract FOLLOWER_PORT, BAUDRATE, FOLLOWER_IDS from the main script
    by AST-walking it. Avoids exec() and avoids importing the module
    (which would run its top-level I/O)."""
    tree = ast.parse(_MAIN.read_text())
    wanted = {"FOLLOWER_PORT", "BAUDRATE", "FOLLOWER_IDS"}
    vals = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            t = node.targets[0]
            if isinstance(t, ast.Name) and t.id in wanted:
                vals[t.id] = ast.literal_eval(node.value)
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
            if p is not None: break
        if p is None:
            print(f"  ID {sid}: FAILED to read (retry or check wiring)")
            sys.exit(1)
        print(f"  ID {sid}: {p}")
        positions.append(p)
    ser.close()

    cal = json.loads(_CAL.read_text())
    assert len(cal["joints"]) == len(positions), (
        f"calibration has {len(cal['joints'])} joints, got {len(positions)} positions")
    print("\nUpdating kinematics_calibration.json — diffs:")
    for j, new_zero in zip(cal["joints"], positions):
        old = j.get("tick_zero")
        j["tick_zero"] = new_zero
        delta = new_zero - old
        print(f"  {j['name']:<14} (id {j['id']}): {old} -> {new_zero}  (Δ {delta:+d})")

    _CAL.write_text(json.dumps(cal, indent=2) + "\n")
    print("\nDone. Restart follower_sees_follower_does.py.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Syntax check**

```bash
python -c "import ast; ast.parse(open('calibrate_home.py').read()); print('syntax OK')"
```

Expected: `syntax OK`.

- [ ] **Step 3: Live-rig validation**

Stop the main script. Physically pose the follower into your chosen URDF home pose (arm pointing forward along world X, all joints "zero" visually, gripper half-open). Then:

```bash
python calibrate_home.py
```

Expected: reads 6 positions, prints a diff, writes the new `tick_zero` values, prints `Done.` Verify:

```bash
python -c "import json; print(json.dumps(json.load(open('kinematics_calibration.json')), indent=2))"
```

Restart the main script; open the browser. The ghost arm should now match the real pose closely. If a joint is mirrored, edit `kinematics_calibration.json` and flip that joint's `tick_sign` from `1` to `-1`.

- [ ] **Step 4: Commit**

```bash
git add calibrate_home.py
git commit -m "$(cat <<'EOF'
Add calibrate_home.py — record servo positions as URDF home tick_zero

Uses ast.literal_eval to read FOLLOWER_PORT/BAUDRATE/FOLLOWER_IDS from
the main script without executing it. Updates tick_zero values in
kinematics_calibration.json with a diff printout. One-time per rig.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 12: README updates

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Read current README**

```bash
cat README.md
```

Note the section heading style (the existing "Telemetry dashboard" section uses `###`).

- [ ] **Step 2: Replace the existing Telemetry dashboard section**

Open `README.md`. Find the `### Telemetry dashboard` section added in the previous feature. Replace the entire section (from that heading through to the next `###` heading) with:

```markdown
### Telemetry dashboard & 3D sim

While `follower_sees_follower_does.py` is running, a dashboard is served at `http://127.0.0.1:8080`. It shows:

- A 3D view of the SO-101 (ghost arm, semi-transparent) that mirrors the live follower pose.
- Per-motor tables: position, goal, temperature, voltage, load — with 60-second trend sparklines for temperature and load.

Thresholds (color-coded):
- Temperature: amber ≥ 50 °C, red ≥ 65 °C
- Voltage: amber outside 10.5–13.5 V
- Load: amber ≥ 70%, red ≥ 90%

#### Browser control mode

Instead of the physical leader arm, you can drive the follower by dragging the end-effector in the 3D view.

1. Start teleop normally — sync the two arms as usual and press ENTER.
2. In the browser, click **Engage**.
3. Click-drag the blue sphere at the end-effector to move the arm. IK solves in the browser; the follower tracks with ~200 ms of lag (intentional safety clamp).
4. Use the side sliders for wrist pitch, wrist roll, and gripper — these bypass IK.
5. Click **Release Control** to hand control back to the physical leader.

Safety layers active during browser control:
- **Heartbeat watchdog** — if the browser stops pinging for 1.5 s (tab closed, laptop sleep, network drop), the follower returns to physical-leader control automatically, with a torque blip to prevent drift.
- **Per-joint velocity clamp** — each sync-write advances a joint by at most 100 ticks (~9°).
- **URDF joint limits** — IK output is clipped to the servo's safe working range.
- **Release button** — always visible; one click returns control.
- **Two-stage Ctrl+C** — press once to freeze motion and release control (arm holds position). Press again within 5 seconds for full shutdown.

#### Per-rig calibration

The vendored SO-101 URDF defines joint angles in radians; servos speak ticks. `kinematics_calibration.json` owns the per-joint `tick_zero` (the tick value that corresponds to the URDF's home pose), `tick_sign`, and `tick_min`/`tick_max`.

One-time calibration:

1. Stop `follower_sees_follower_does.py`.
2. Manually move the follower arm into the URDF home pose: shoulder straight, elbow straight, wrist level, arm pointing forward along the world X axis, gripper half-open.
3. Run `python calibrate_home.py`. It records the current positions as `tick_zero`.
4. Restart the main script. The ghost arm should match the real arm very closely. If a joint moves the wrong way (ghost rotates left when real moves right), edit `kinematics_calibration.json` and flip that joint's `tick_sign` between `+1` and `-1`.

Defaults shipped with the repo assume a rig assembled per the [SO-101 assembly guide](https://huggingface.co/docs/lerobot/so101); calibration is strictly a refinement.
```

- [ ] **Step 3: Verify the README reads cleanly**

```bash
cat README.md
```

Check heading-level consistency with neighboring sections; no Markdown artifacts pasted.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
Document sim control, safety, and per-rig calibration in README

Expanded the Telemetry dashboard section to cover the 3D ghost, the
browser control flow (Engage/drag/Release), the five safety layers,
and the one-time calibration procedure with calibrate_home.py.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Final check

Run `git log --oneline` and confirm 12 new commits (one per task). Then the full end-to-end demo:

1. `python calibrate_home.py` once on your rig.
2. `python follower_sees_follower_does.py`, sync, ENTER.
3. Open browser — ghost tracks live.
4. Engage → drag handle → follower tracks (~200 ms lag).
5. Slider the wrist pitch → isolated motion.
6. Close browser mid-drag → terminal prints `heartbeat lost`; arm blips and resumes physical.
7. Ctrl+C once → `Motion frozen`. Ctrl+C again → clean shutdown.

If all seven steps pass, the plan is complete.
