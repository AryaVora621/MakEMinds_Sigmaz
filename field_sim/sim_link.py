"""Pure helpers for the robot <-> sim link (token check, body limits, strict validation). No server code here.

Used by run_sim.py:
    check_token(header_value, expected)         -> bool   (constant time; expected "" = no auth required)
    is_loopback_host(host)                      -> bool   (warn/require --token when binding to non-loopback)
    read_body(rfile, content_length_header, max_bytes=MAX_BODY) -> bytes   (raises LinkError(status, msg))
    parse_json(raw)                             -> object (raises LinkError(400))
    validate_state(data)                        -> dict   (clean, clamped; unknown keys dropped; raises LinkError(400))
    validate_replay(data)                       -> list   (<= MAX_REPLAY_FRAMES validated frames)
    InputStore                                  -> .set(dict) / .get(max_age_s) / .clear()  reverse channel store
    validate_input(data)                        -> dict   (sim -> robot: {"pose":{x,y,h},"balls":n})
    input_reply(store, accept_header, enabled)  -> dict   (extra keys to merge into the POST /api/state reply)
"""
import hmac
import math
import time

MAX_BODY = 64 * 1024
MAX_REPLAY_BODY = 4 * 1024 * 1024
MAX_REPLAY_FRAMES = 20000
MAX_PATH_POINTS = 400
FIELD_MIN, FIELD_MAX = -20.0, 164.0
MAX_RPM = 8000.0


class LinkError(Exception):
    def __init__(self, status, msg):
        super().__init__(msg)
        self.status = status
        self.msg = msg


def is_loopback_host(host):
    return host in ("127.0.0.1", "localhost", "::1")


def check_token(header_value, expected):
    if not expected:
        return True
    if not header_value:
        return False
    return hmac.compare_digest(str(header_value).encode(), str(expected).encode())


def read_body(rfile, content_length_header, max_bytes=MAX_BODY):
    try:
        n = int(content_length_header or 0)
    except (TypeError, ValueError):
        raise LinkError(400, "bad content-length")
    if n < 0:
        raise LinkError(400, "bad content-length")
    if n > max_bytes:
        raise LinkError(413, "body too large")
    return rfile.read(n) if n else b""


def parse_json(raw):
    import json
    try:
        return json.loads(raw or b"{}")
    except (ValueError, UnicodeDecodeError, RecursionError):
        raise LinkError(400, "bad json")


def _num(v, lo, hi, default=None):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        return default
    return max(lo, min(hi, float(v)))


def _bool(v):
    return v if isinstance(v, bool) else None


def _label(v):
    if not isinstance(v, str):
        return None
    return "".join(c for c in v if c.isalnum() or c in "_ -")[:24]


def _pose(p):
    if not isinstance(p, dict):
        return None
    x, y = _num(p.get("x"), FIELD_MIN, FIELD_MAX), _num(p.get("y"), FIELD_MIN, FIELD_MAX)
    h = _num(p.get("h"), -7.0, 7.0)
    if x is None or y is None or h is None:
        return None
    return {"x": x, "y": y, "h": h}


def validate_path(pts):
    if not isinstance(pts, list):
        raise LinkError(400, "path must be a list")
    out = []
    for q in pts[:MAX_PATH_POINTS]:
        if isinstance(q, (list, tuple)) and len(q) >= 2:
            x, y = _num(q[0], FIELD_MIN, FIELD_MAX), _num(q[1], FIELD_MIN, FIELD_MAX)
            h = _num(q[2], -7.0, 7.0, 0.0) if len(q) > 2 else 0.0
            if x is not None and y is not None:
                out.append([x, y, h])
    return out


def validate_state(data):
    """Whitelist + clamp. Unknown keys are dropped, wrong types skipped. Raises LinkError(400) if not an object."""
    if not isinstance(data, dict):
        raise LinkError(400, "state must be a JSON object")
    out = {}
    pose = _pose(data.get("pose"))
    if pose:
        out["pose"] = pose
    for key, lo, hi in (("turretDeg", -360, 360), ("hoodDeg", -90, 90), ("rpm", 0, MAX_RPM)):
        v = _num(data.get(key), lo, hi)
        if v is not None:
            out[key] = v
    for key in ("intakeOn", "feeding"):
        b = _bool(data.get(key))
        if b is not None:
            out[key] = b
    if data.get("alliance") in ("red", "blue"):
        out["alliance"] = data["alliance"]
    tip = data.get("tip")
    if isinstance(tip, dict):
        t = {k: v for k, v in tip.items() if k in ("red", "blue") and v in ("far_up", "audience_up")}
        if t:
            out["tip"] = t
    balls = _num(data.get("balls"), 0, 9)
    if balls is not None:
        out["balls"] = int(round(balls))
    fl = data.get("flower")
    if isinstance(fl, dict):
        f = {}
        s = _label(fl.get("state"))
        if s is not None:
            f["state"] = s
        d = _bool(fl.get("deployed"))
        if d is not None:
            f["deployed"] = d
        if f:
            out["flower"] = f
    g = data.get("gate")
    if isinstance(g, dict) and _bool(g.get("open")) is not None:
        out["gate"] = {"open": g["open"]}
    fo = data.get("follower")
    if isinstance(fo, dict):
        f = {k: fo[k] for k in ("busy", "holding", "pathActive") if _bool(fo.get(k)) is not None}
        if f:
            out["follower"] = f
    if "path" in data:
        out["path"] = validate_path(data["path"])
    return out


def validate_replay(data):
    if not isinstance(data, list) or not data:
        raise LinkError(400, "replay must be a non-empty list")
    if len(data) > MAX_REPLAY_FRAMES:
        raise LinkError(413, "too many frames")
    frames = []
    for fr in data:
        if not isinstance(fr, dict):
            continue
        t = _num(fr.get("t"), 0.0, 1e6)
        if t is None:
            continue
        clean = validate_state(fr)
        clean["t"] = t
        frames.append(clean)
    if not frames:
        raise LinkError(400, "no valid frames")
    return frames


def validate_input(data):
    """Sim -> robot simulated input. Only pose and balls are accepted."""
    if not isinstance(data, dict):
        raise LinkError(400, "input must be a JSON object")
    out = {}
    pose = _pose(data.get("pose"))
    if pose:
        out["pose"] = pose
    b = _num(data.get("balls"), 0, 9)
    if b is not None:
        out["balls"] = int(round(b))
    return out


class InputStore:
    """Holds the latest simulated input (reverse channel) with a timestamp."""

    def __init__(self):
        self._data = {}
        self._t = 0.0

    def set(self, data):
        self._data = validate_input(data)
        self._t = time.time()

    def clear(self):
        self._data = {}
        self._t = 0.0

    def get(self, max_age_s=1.0):
        if not self._data or time.time() - self._t > max_age_s:
            return {}
        return dict(self._data)


def input_reply(store, accept_header, enabled):
    """Reply extras for a robot POST: only if the server enabled the reverse channel AND the robot asked for it."""
    if not enabled or str(accept_header or "") != "1":
        return {}
    inp = store.get()
    return {"input": inp} if inp else {}
