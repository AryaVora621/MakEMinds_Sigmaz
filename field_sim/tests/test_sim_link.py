import io
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sim_link as sl  # noqa: E402


def test_token():
    assert sl.check_token(None, "")
    assert not sl.check_token(None, "s")
    assert not sl.check_token("x", "s")
    assert sl.check_token("s", "s")


def test_loopback():
    assert sl.is_loopback_host("127.0.0.1") and not sl.is_loopback_host("0.0.0.0")


def test_read_body_limits():
    assert sl.read_body(io.BytesIO(b"abc"), "3") == b"abc"
    with pytest.raises(sl.LinkError) as e:
        sl.read_body(io.BytesIO(b""), str(sl.MAX_BODY + 1))
    assert e.value.status == 413
    with pytest.raises(sl.LinkError):
        sl.read_body(io.BytesIO(b""), "-1")
    with pytest.raises(sl.LinkError):
        sl.read_body(io.BytesIO(b""), "abc")


def test_parse_json_bad():
    with pytest.raises(sl.LinkError):
        sl.parse_json(b"{nope")


def test_validate_state_clamps_and_drops():
    out = sl.validate_state({"pose": {"x": 1e9, "y": 5, "h": 1}, "rpm": float("nan"), "turretDeg": 999,
                             "evil": 1, "balls": 99, "intakeOn": "yes", "feeding": True,
                             "flower": {"state": "DEPLOY<script>", "deployed": True},
                             "alliance": "green", "gate": {"open": False}})
    assert out["pose"]["x"] == sl.FIELD_MAX
    assert "rpm" not in out and "evil" not in out and "intakeOn" not in out and "alliance" not in out
    assert out["turretDeg"] == 360 and out["balls"] == 9 and out["feeding"] is True
    assert out["flower"]["state"] == "DEPLOYscript"
    assert out["gate"] == {"open": False}


def test_validate_state_rejects_non_object_and_bad_pose():
    with pytest.raises(sl.LinkError):
        sl.validate_state([1])
    assert "pose" not in sl.validate_state({"pose": {"x": "a", "y": 1, "h": 0}})
    assert "pose" not in sl.validate_state({"pose": {"x": True, "y": 1, "h": 0}})


def test_path_capped():
    out = sl.validate_state({"path": [[1, 2, 0]] * 1000 + [["a"]]})
    assert len(out["path"]) == sl.MAX_PATH_POINTS


def test_replay():
    fr = sl.validate_replay([{"t": 0, "pose": {"x": 1, "y": 2, "h": 0}}, "junk", {"pose": {}}])
    assert len(fr) == 1
    with pytest.raises(sl.LinkError):
        sl.validate_replay([])
    with pytest.raises(sl.LinkError):
        sl.validate_replay([{"t": 0}] * (sl.MAX_REPLAY_FRAMES + 1))


def test_input_store_and_reply():
    st = sl.InputStore()
    assert sl.input_reply(st, "1", True) == {}
    st.set({"pose": {"x": 10, "y": 20, "h": 1}, "balls": 2, "junk": 1})
    assert sl.input_reply(st, "1", True) == {"input": {"pose": {"x": 10.0, "y": 20.0, "h": 1.0}, "balls": 2}}
    assert sl.input_reply(st, None, True) == {}
    assert sl.input_reply(st, "1", False) == {}
    assert st.get(max_age_s=-1) == {}
