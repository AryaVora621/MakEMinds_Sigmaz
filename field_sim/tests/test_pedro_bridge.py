import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pedro_bridge  # noqa: E402

pytestmark = pytest.mark.skipif(not pedro_bridge.available(),
                                reason="no JDK (javac/java) or Pedro core jar found in the Gradle cache")


def bez(pts, t):
    n = len(pts) - 1
    x = y = 0.0
    for i, (px, py) in enumerate(pts):
        b = math.comb(n, i) * (1 - t) ** (n - i) * t ** i
        x += b * px
        y += b * py
    return x, y


def test_line_length_50():
    r = pedro_bridge.sample_route({"start": {"x": 0, "y": 0, "h": 0}, "step": 1.0, "segments": [
        {"type": "line", "points": [{"x": 0, "y": 0}, {"x": 30, "y": 40}], "heading": {"mode": "constant", "start": 10}}]})
    assert "error" not in r, r
    seg = r["segments"][0]
    assert seg["length"] == pytest.approx(50, abs=0.01)
    assert r["total"] == pytest.approx(50, abs=0.01)
    assert len(seg["samples"]) == 51
    assert all(s["h"] == pytest.approx(10, abs=1e-4) for s in seg["samples"])


CURVE = [(0, 0), (30, 0), (60, 40), (60, 80)]


def curve_route(heading):
    return {"start": {"x": 0, "y": 0, "h": 0}, "step": 0.5, "segments": [
        {"type": "curve", "points": [{"x": x, "y": y} for x, y in CURVE], "heading": heading}]}


def test_curve_endpoints_and_agreement():
    r = pedro_bridge.sample_route(curve_route({"mode": "linear", "start": 0, "end": 90}))
    assert "error" not in r, r
    sm = r["segments"][0]["samples"]
    assert (sm[0]["x"], sm[0]["y"]) == pytest.approx(CURVE[0], abs=1e-4)
    assert (sm[-1]["x"], sm[-1]["y"]) == pytest.approx(CURVE[-1], abs=1e-4)
    assert sm[-1]["h"] == pytest.approx(90, abs=1e-3)
    # every sample lies on the analytic Bezier at its own t
    for s in sm:
        ex, ey = bez(CURVE, s["t"])
        assert math.hypot(s["x"] - ex, s["y"] - ey) < 0.05
    # arc-length spacing: cumulative s matches chord sums (within 0.05)
    length = r["segments"][0]["length"]
    ref = [bez(CURVE, i / 20000) for i in range(20001)]
    ref_len = sum(math.dist(ref[i], ref[i + 1]) for i in range(20000))
    assert length == pytest.approx(ref_len, abs=0.05)
    cum = 0.0
    for a, b in zip(sm, sm[1:]):
        cum += math.dist((a["x"], a["y"]), (b["x"], b["y"]))
        assert cum == pytest.approx(b["s"], abs=0.05)


def test_tangent_heading_follows_derivative():
    r = pedro_bridge.sample_route(curve_route({"mode": "tangent"}))
    assert "error" not in r, r
    for s in r["segments"][0]["samples"][1:-1]:
        t, e = s["t"], 1e-5
        x0, y0 = bez(CURVE, t - e)
        x1, y1 = bez(CURVE, t + e)
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        diff = (s["h"] - ang + 180) % 360 - 180
        assert abs(diff) < 0.1


def test_tangent_reversed_is_180_off():
    f = pedro_bridge.sample_route(curve_route({"mode": "tangent"}))["segments"][0]["samples"]
    b = pedro_bridge.sample_route(curve_route({"mode": "tangent", "reversed": True}))["segments"][0]["samples"]
    for x, y in zip(f, b):
        assert abs((x["h"] - y["h"] + 180) % 360 - 180) == pytest.approx(180, abs=0.1)


def test_error_reported():
    r = pedro_bridge.sample_route({"start": {}, "segments": [{"type": "curve", "points": []}]})
    assert "error" in r
