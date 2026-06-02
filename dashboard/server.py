"""Tiny stdlib HTTP server that exposes the telemetry state as JSON and
serves a single static HTML file. Runs in a daemon thread so Ctrl+C in the
main script tears it down without explicit shutdown."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from dashboard import state
from dashboard import control

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

    def _serve_under(self, subdir, relpath, content_type):
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

    def do_POST(self):
        if self.path == "/api/engage":
            self._handle_engage()
        elif self.path == "/api/release":
            self._handle_release()
        elif self.path == "/api/target":
            self._handle_target()
        elif self.path == "/api/heartbeat":
            self._handle_heartbeat()
        elif self.path == "/api/save_offsets":
            self._handle_save_offsets()
        elif self.path == "/api/capture_home":
            self._handle_capture_home()
        elif self.path == "/api/flip_sign":
            self._handle_flip_sign()
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

    def _handle_save_offsets(self):
        """Write angle_offset_rad values into kinematics_calibration.json."""
        body = self._read_json()
        if body is None or "offsets" not in body:
            self._reply_json(400, {"ok": False, "message": "bad body"})
            return
        path = Path(__file__).resolve().parent.parent / "kinematics_calibration.json"
        try:
            cal = json.loads(path.read_text())
        except Exception as e:
            self._reply_json(500, {"ok": False, "message": f"read failed: {e}"})
            return
        offsets_by_name = {o["name"]: float(o["angle_offset_rad"]) for o in body["offsets"]}
        for j in cal["joints"]:
            if j["name"] in offsets_by_name:
                j["angle_offset_rad"] = offsets_by_name[j["name"]]
        try:
            path.write_text(json.dumps(cal, indent=2) + "\n")
        except Exception as e:
            self._reply_json(500, {"ok": False, "message": f"write failed: {e}"})
            return
        self._reply_json(200, {"ok": True, "message": "saved"})

    def _handle_capture_home(self):
        """Record current follower tick values (from live telemetry) as
        tick_zero for every joint. User positions the real arm to match
        the ghost's URDF-zero pose and clicks the button."""
        path = Path(__file__).resolve().parent.parent / "kinematics_calibration.json"
        try:
            cal = json.loads(path.read_text())
        except Exception as e:
            self._reply_json(500, {"ok": False, "message": f"read failed: {e}"})
            return
        snap = state.snapshot()
        by_id = {m["id"]: m.get("pos") for m in snap.get("motors", [])}
        updated = []
        for j in cal["joints"]:
            pos = by_id.get(j["id"])
            if pos is None:
                self._reply_json(409, {"ok": False,
                    "message": f"no live position for joint {j['name']} (id {j['id']})"})
                return
            j["tick_zero"] = int(pos)
            updated.append({"name": j["name"], "tick_zero": int(pos)})
        try:
            path.write_text(json.dumps(cal, indent=2) + "\n")
        except Exception as e:
            self._reply_json(500, {"ok": False, "message": f"write failed: {e}"})
            return
        self._reply_json(200, {"ok": True, "message": "captured", "joints": updated})

    def _handle_flip_sign(self):
        """Flip tick_sign between +1 and -1 for a single joint."""
        body = self._read_json()
        if body is None or "name" not in body:
            self._reply_json(400, {"ok": False, "message": "bad body"})
            return
        target_name = body["name"]
        path = Path(__file__).resolve().parent.parent / "kinematics_calibration.json"
        try:
            cal = json.loads(path.read_text())
        except Exception as e:
            self._reply_json(500, {"ok": False, "message": f"read failed: {e}"})
            return
        for j in cal["joints"]:
            if j["name"] == target_name:
                j["tick_sign"] = -1 if int(j.get("tick_sign", 1)) > 0 else 1
                new_sign = j["tick_sign"]
                try:
                    path.write_text(json.dumps(cal, indent=2) + "\n")
                except Exception as e:
                    self._reply_json(500, {"ok": False, "message": f"write failed: {e}"})
                    return
                self._reply_json(200, {"ok": True, "tick_sign": new_sign})
                return
        self._reply_json(404, {"ok": False, "message": f"joint {target_name} not found"})


_server = None


def start(host="127.0.0.1", port=8080):
    """Spin up the server in a daemon thread. Prints a warning and returns
    (without raising) if the port is already in use — the dashboard is a
    nice-to-have and must not take down teleop."""
    global _server
    try:
        _server = ThreadingHTTPServer((host, port), _Handler)
    except OSError as e:
        print(f"[dashboard] port {port} unavailable ({e}); dashboard disabled.")
        _server = None
        return
    t = threading.Thread(target=_server.serve_forever, daemon=True,
                         name="dashboard-http")
    t.start()
    print(f"[dashboard] serving on http://{host}:{port}")


def stop():
    """Shut down the HTTP server and close the listening socket.
    Call from the main script's shutdown path so the port is released
    promptly — otherwise Windows can hold it until the next process start
    fails with WSAEACCES."""
    global _server
    if _server is None:
        return
    try:
        _server.shutdown()
    except Exception:
        pass
    try:
        _server.server_close()
    except Exception:
        pass
    _server = None
