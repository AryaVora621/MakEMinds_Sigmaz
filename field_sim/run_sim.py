"""BIOBUZZ field simulator: starts a local web server and opens the 3D viewer in your browser (WebGL, hardware accelerated).

    .venv\\Scripts\\python.exe run_sim.py                # serve on http://127.0.0.1:8780 and open the browser
    .venv\\Scripts\\python.exe run_sim.py --port 8780 --no-browser
    .venv\\Scripts\\python.exe run_sim.py --export       # rebuild web/data from data/field_model first

LIVE LINK (sync with Pedro Pathing / the robot): the viewer polls GET /api/state. Anything that POSTs JSON to
/api/state drives the robot in the viewer, for example from the robot code (see TeamCode SimBridge) or curl:

    curl -X POST http://127.0.0.1:8780/api/state -d "{\\"pose\\":{\\"x\\":72,\\"y\\":30,\\"h\\":1.57},\\"turretDeg\\":10,\\"hoodDeg\\":40,\\"rpm\\":3000}"

State fields (all optional except pose): pose{x,y,h} in Pedro inches / radians, turretDeg, hoodDeg, rpm,
alliance ("red"|"blue"), tip{red,blue} ("audience_up"|"far_up"), path [[x,y],...] (drawn as a green line).
The server stamps every POST with the current time; the viewer shows LINKED while updates are < 1 s old.
POST /api/replay with a JSON list [{t,pose,...}, ...] to play a recorded run back in real time.
"""
import argparse
import http.server
import json
import os
import pathlib
import subprocess
import sys
import threading
import time
import webbrowser

import sim_link

ROOT = pathlib.Path(__file__).resolve().parent
WEB = ROOT / "web"

STATE = {"t": 0.0}
LOCK = threading.Lock()
TOKEN = ""          # shared secret for robot POSTs (--token or SIM_TOKEN env); "" = none (loopback only)
INPUTS = sim_link.InputStore()
ACCEPT_INPUT = False  # --accept-input: let the sim send simulated pose/balls back to the robot


def replay(frames):
    t0 = time.time()
    base = frames[0].get("t", 0.0)
    for fr in frames:
        wait = (fr.get("t", 0.0) - base) - (time.time() - t0)
        if wait > 0:
            time.sleep(wait)
        merge({k: v for k, v in fr.items() if k != "t"})


def merge(update):
    with LOCK:
        STATE.update(update)
        STATE["t"] = time.time()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=str(WEB), **k)

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        # No CORS: the robot is not a browser, and a web page must not be able to drive the sim cross-origin.
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/api/state"):
            with LOCK:
                return self._json(dict(STATE))
        if self.path.startswith("/api/pedro/status"):
            try:
                import pedro_bridge
                return self._json({"available": bool(pedro_bridge.available()), "follow": bool(pedro_bridge.follow_available())})
            except Exception as exc:
                return self._json({"available": False, "error": str(exc)})
        return super().do_GET()

    def do_POST(self):
        pedro = self.path.startswith("/api/pedro/")
        robot_link = self.path.startswith(("/api/state", "/api/replay", "/api/input"))
        if robot_link and not sim_link.check_token(self.headers.get("X-Sim-Token"), TOKEN):
            return self._json({"error": "bad or missing X-Sim-Token"}, 401)
        try:
            limit = sim_link.MAX_REPLAY_BODY if self.path.startswith("/api/replay") else (1024 * 1024 if pedro else sim_link.MAX_BODY)
            data = sim_link.parse_json(sim_link.read_body(self.rfile, self.headers.get("Content-Length"), limit))
            if self.path.startswith("/api/state"):
                merge(sim_link.validate_state(data))
                reply = {"ok": True}
                reply.update(sim_link.input_reply(INPUTS, self.headers.get("X-Sim-Accept-Input"), ACCEPT_INPUT))
                return self._json(reply)
            if self.path.startswith("/api/input"):
                if not ACCEPT_INPUT:
                    return self._json({"error": "reverse channel disabled (start with --accept-input)"}, 403)
                INPUTS.set(data)
                return self._json({"ok": True})
            if self.path.startswith("/api/replay"):
                frames = sim_link.validate_replay(data)
                threading.Thread(target=replay, args=(frames,), daemon=True).start()
                return self._json({"ok": True, "frames": len(frames)})
        except sim_link.LinkError as exc:
            return self._json({"error": exc.msg}, exc.status)
        if self.path.startswith("/api/pedro/sample"):
            # Samples a route with the REAL Pedro Pathing 2.1.0 core library (tools_java/PedroSampler.java).
            try:
                import pedro_bridge
                if not pedro_bridge.available():
                    return self._json({"error": "Pedro core jar or JDK not found (see tools_java/README.md)"}, 503)
                return self._json(pedro_bridge.sample_route(data))
            except Exception as exc:  # report to the page instead of crashing the server
                return self._json({"error": str(exc)}, 500)
        if self.path.startswith("/api/pedro/follow") or self.path.startswith("/api/pedro/live/"):
            # Real Pedro 2.1.0 Follower on a simulated mecanum robot (tools_java/PedroFollowSim.java).
            #   POST /api/pedro/follow            {start, segments, ...}     -> {ticks:[{t,x,y,h,vx,vy,w,p,seg,phase}], events, info}
            #   POST /api/pedro/live/start        {start, segments, ...}     -> {id, tick, info}
            #   POST /api/pedro/live/step         {id, n}                    -> {ticks, events, done}
            #   POST /api/pedro/live/stop         {id}
            try:
                import pedro_bridge
                if not pedro_bridge.follow_available():
                    return self._json({"error": "Pedro core jar or JDK not found (see tools_java/README.md)"}, 503)
                path = self.path.split("?")[0]
                if path.endswith("/live/start"):
                    res = pedro_bridge.live_start(data)
                elif path.endswith("/live/step"):
                    res = pedro_bridge.live_step(str(data.get("id", "")), int(data.get("n", 10)))
                elif path.endswith("/live/stop"):
                    res = pedro_bridge.live_stop(str(data.get("id", "")))
                else:
                    res = pedro_bridge.follow(data)
                return self._json(res, 400 if res.get("error") else 200)
            except Exception as exc:
                return self._json({"error": str(exc)}, 500)
        return self._json({"error": "unknown endpoint"}, 404)

    def log_message(self, *a):
        pass

    def end_headers(self):
        if self.path.split("?")[0] in ("/", "") or self.path.split("?")[0].endswith((".js", ".json", ".html", ".bin")):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to let the robot / another PC post state")
    ap.add_argument("--token", default=os.environ.get("SIM_TOKEN", ""), help="shared secret the robot sends as X-Sim-Token (required with a non-loopback --host)")
    ap.add_argument("--accept-input", action="store_true", help="enable the reverse channel (POST /api/input -> robot, only if the robot opts in)")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--export", action="store_true", help="rebuild web/data from data/field_model before starting")
    args = ap.parse_args()
    if args.export or not (WEB / "data" / "scene.json").exists():
        subprocess.check_call([sys.executable, "-I", str(ROOT / "tools" / "export_web.py")])
    global TOKEN, ACCEPT_INPUT
    TOKEN, ACCEPT_INPUT = args.token, args.accept_input
    if not sim_link.is_loopback_host(args.host) and not TOKEN:
        sys.exit("Refusing to listen on %s without --token (or SIM_TOKEN): anyone on the network could drive the sim." % args.host)
    srv = http.server.ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://127.0.0.1:{args.port}/"
    print("BIOBUZZ field sim on", url, "  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
