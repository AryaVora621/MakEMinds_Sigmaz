"""Bridge to the REAL Pedro Pathing core jar (via tools_java/PedroSampler.java).

sample_route(route) shells out to java; the compiled class dir is cached in
tools_java/out and rebuilt when PedroSampler.java is newer. Standard library only.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import zipfile
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_JAVA_DIR = _HERE / "tools_java"
_SRC = _JAVA_DIR / "PedroSampler.java"
_OUT = _JAVA_DIR / "out"
_CLASS = _OUT / "PedroSampler.class"

_cache: dict = {}


def _find_jar() -> str | None:
    env = os.environ.get("PEDRO_CORE_JAR")
    if env and os.path.isfile(env):
        return env
    root = Path.home() / ".gradle" / "caches" / "modules-2" / "files-2.1" / "com.pedropathing" / "core"
    hits = [p for p in glob.glob(str(root / "**" / "core-*.jar"), recursive=True)
            if "sources" not in p and "javadoc" not in p]
    return sorted(hits)[-1] if hits else None


def _find_tool(name: str) -> str | None:
    exe = name + (".exe" if os.name == "nt" else "")
    cands = []
    for var in ("JAVA_HOME",):
        if os.environ.get(var):
            cands.append(Path(os.environ[var]) / "bin" / exe)
    cands += [Path(r"C:\Program Files\Android\Android Studio\jbr\bin") / exe,
              Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "Android Studio" / "jbr" / "bin" / exe]
    for c in cands:
        if c.is_file():
            return str(c)
    return shutil.which(name)


def _tools():
    if "t" not in _cache:
        _cache["t"] = (_find_tool("java"), _find_tool("javac"), _find_jar())
    return _cache["t"]


def available() -> bool:
    java, javac, jar = _tools()
    return bool(java and javac and jar and _SRC.is_file())


def _ensure_built() -> str | None:
    """Return an error string, or None when the class is up to date."""
    java, javac, jar = _tools()
    if _CLASS.is_file() and _CLASS.stat().st_mtime >= _SRC.stat().st_mtime:
        return None
    _OUT.mkdir(parents=True, exist_ok=True)
    r = subprocess.run([javac, "-encoding", "UTF-8", "-cp", jar, "-d", str(_OUT), str(_SRC)],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        return "javac failed: " + (r.stderr or r.stdout)
    return None


def sample_route(route: dict, timeout: float = 20) -> dict:
    """Sample `route` with real Pedro classes. Returns parsed JSON or {'error': ...}."""
    if not available():
        return {"error": "JDK or Pedro core jar not found (set JAVA_HOME / PEDRO_CORE_JAR)"}
    java, _javac, jar = _tools()
    try:
        err = _ensure_built()
        if err:
            return {"error": err}
        cp = os.pathsep.join([str(_OUT), jar])
        r = subprocess.run([java, "-cp", cp, "PedroSampler"], input=json.dumps(route),
                           capture_output=True, text=True, timeout=timeout, encoding="utf-8")
        if not r.stdout.strip():
            return {"error": "no output from PedroSampler: " + r.stderr[-500:]}
        return json.loads(r.stdout)
    except subprocess.TimeoutExpired:
        return {"error": "PedroSampler timed out"}
    except Exception as e:  # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}"}


# ---------------------------------------------------------------------------------------------------------------
# PedroFollowSim: the REAL Pedro 2.1.0 Follower driving a simulated localizer + mecanum drivetrain.
# The Follower/PIDFs read System.nanoTime()/currentTimeMillis(); to run faster than real time with an exact dt we
# compile patched copies of the 6 core classes that read the clock (taken from the official sources jar) so they
# read tools_java/sim/.../SimClock instead. All control math is Pedro's own. If the sources jar is missing we fall
# back to the unpatched jar and run in real time (-Dpedro.realtime=true, slow but still the real algorithm).
# ---------------------------------------------------------------------------------------------------------------
_FOLLOW_OUT = _OUT / "follow"
_FOLLOW_SRC = _JAVA_DIR / "PedroFollowSim.java"
_CLOCK_SRC = _JAVA_DIR / "sim" / "com" / "pedropathing" / "util" / "SimClock.java"
_PATCH_FILES = [
    "com/pedropathing/control/PIDFController.java",
    "com/pedropathing/control/FilteredPIDFController.java",
    "com/pedropathing/localization/PoseTracker.java",
    "com/pedropathing/paths/callbacks/TemporalCallback.java",
    "com/pedropathing/follower/Follower.java",
    "com/pedropathing/util/Timer.java",
]
_follow_lock = threading.Lock()


def _find_sources_jar(jar):
    if not jar:
        return None
    root = Path(jar).parent.parent  # .../core/<ver>/<hash>/core-x.jar -> .../core/<ver>
    hits = glob.glob(str(root / "**" / "core-*-sources.jar"), recursive=True)
    return hits[0] if hits else None


def follow_realtime_clock() -> bool:
    """True when the build could not patch the clock (no sources jar), so runs are real time."""
    return not (_FOLLOW_OUT / "PATCHED").is_file()


def follow_available() -> bool:
    java, javac, jar = _tools()
    return bool(java and javac and jar and _FOLLOW_SRC.is_file() and _SRC.is_file() and _CLOCK_SRC.is_file())


def build_follow(force: bool = False):
    """Compile PedroFollowSim (+ patched clock classes). Returns an error string, or None when up to date/built."""
    java, javac, jar = _tools()
    srcs = [_FOLLOW_SRC, _CLOCK_SRC, _SRC, Path(__file__)]
    marker = _FOLLOW_OUT / "PedroFollowSim.class"
    newest = max(p.stat().st_mtime for p in srcs if p.is_file())
    if not force and marker.is_file() and marker.stat().st_mtime >= newest:
        return None
    if _FOLLOW_OUT.exists():
        shutil.rmtree(_FOLLOW_OUT, ignore_errors=True)
    _FOLLOW_OUT.mkdir(parents=True, exist_ok=True)
    patched_dir = _FOLLOW_OUT / "_patched_src"
    base_files = [str(_FOLLOW_SRC), str(_CLOCK_SRC), str(_SRC)]
    files = list(base_files)
    patched = False
    sj = _find_sources_jar(jar)
    if sj:
        try:
            with zipfile.ZipFile(sj) as z:
                for name in _PATCH_FILES:
                    text = z.read(name).decode("utf-8")
                    text = text.replace("System.nanoTime()", "com.pedropathing.util.SimClock.nanoTime()")
                    text = text.replace("System.currentTimeMillis()", "com.pedropathing.util.SimClock.currentTimeMillis()")
                    dest = patched_dir / name
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_text(text, encoding="utf-8")
                    files.append(str(dest))
            patched = True
        except Exception:  # noqa: BLE001  fall back to the unpatched jar
            patched = False
            files = list(base_files)
    r = subprocess.run([javac, "-encoding", "UTF-8", "-nowarn", "--release", "11", "-cp", jar, "-d", str(_FOLLOW_OUT), *files],
                       capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        return "javac failed: " + (r.stderr or r.stdout)[-2000:]
    shutil.rmtree(patched_dir, ignore_errors=True)
    if patched:
        (_FOLLOW_OUT / "PATCHED").write_text("virtual clock\n")
    return None


def _follow_cmd(extra=()):
    java, _javac, jar = _tools()
    cmd = [java]
    if follow_realtime_clock():
        cmd.append("-Dpedro.realtime=true")
    cmd += ["-cp", os.pathsep.join([str(_FOLLOW_OUT), jar]), "PedroFollowSim", *extra]
    return cmd


def follow(req: dict, timeout: float = 60) -> dict:
    """Batch: run the real Follower over `req` (see tools_java/PedroFollowSim.java). Returns parsed JSON or {'error'}."""
    if not follow_available():
        return {"error": "JDK or Pedro core jar not found (set JAVA_HOME / PEDRO_CORE_JAR)"}
    try:
        with _follow_lock:
            err = build_follow()
        if err:
            return {"error": err}
        if follow_realtime_clock():
            timeout = max(timeout, float(req.get("maxTime", 60)) + 30)
        r = subprocess.run(_follow_cmd(), input=json.dumps(req), capture_output=True, text=True, timeout=timeout, encoding="utf-8")
        if not r.stdout.strip():
            return {"error": "no output from PedroFollowSim: " + r.stderr[-500:]}
        return json.loads(r.stdout)
    except subprocess.TimeoutExpired:
        return {"error": "PedroFollowSim timed out"}
    except Exception as e:  # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}"}


# ---- live stepping: one long-lived java process per session, JSON lines ----
_sessions: dict = {}
_sess_lock = threading.Lock()
_MAX_SESSIONS = 4
_SESSION_IDLE_S = 300


def _kill(sid):
    s = _sessions.pop(sid, None)
    if s:
        try:
            s["proc"].kill()
        except Exception:  # noqa: BLE001
            pass


def _reap(now):
    for sid in [k for k, v in _sessions.items() if now - v["last"] > _SESSION_IDLE_S or v["proc"].poll() is not None]:
        _kill(sid)


def _rpc(s: dict, msg: dict) -> dict:
    with s["lock"]:
        s["proc"].stdin.write(json.dumps(msg) + "\n")
        s["proc"].stdin.flush()
        line = s["proc"].stdout.readline()
    s["last"] = time.time()
    if not line:
        return {"error": "PedroFollowSim live process ended"}
    return json.loads(line)


def live_start(req: dict) -> dict:
    """Start a live session (returns {'id','ok','info','tick'}); drive it with live_step(id, n)."""
    if not follow_available():
        return {"error": "JDK or Pedro core jar not found (set JAVA_HOME / PEDRO_CORE_JAR)"}
    with _follow_lock:
        err = build_follow()
    if err:
        return {"error": err}
    with _sess_lock:
        _reap(time.time())
        while len(_sessions) >= _MAX_SESSIONS:
            _kill(min(_sessions, key=lambda k: _sessions[k]["last"]))
        proc = subprocess.Popen(_follow_cmd(["--live"]), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, encoding="utf-8", bufsize=1)
        sid = "s%d" % int(time.time() * 1000)
        s = {"proc": proc, "lock": threading.Lock(), "last": time.time()}
        _sessions[sid] = s
    res = _rpc(s, {**req, "cmd": "init"})
    if res.get("error"):
        with _sess_lock:
            _kill(sid)
        return res
    res["id"] = sid
    return res


def live_step(sid: str, n: int = 10) -> dict:
    with _sess_lock:
        s = _sessions.get(sid)
    if not s:
        return {"error": "unknown or expired session"}
    return _rpc(s, {"cmd": "step", "n": int(n)})


def live_stop(sid: str) -> dict:
    with _sess_lock:
        _kill(sid)
    return {"ok": True}


if __name__ == "__main__":
    print(json.dumps(sample_route(json.load(sys.stdin)))[:2000])
