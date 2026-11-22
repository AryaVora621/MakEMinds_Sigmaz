"""CLI smoke tests for field_sim/run_sim_software.py.

Each test launches run_sim_software.py in a subprocess with --offscreen and checks that
a valid, non-trivial PNG is written and its path is reported on stdout.

Run with:  .venv\\Scripts\\python.exe -m pytest tests\\test_cli.py -q
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RUN_SIM = ROOT / "run_sim_software.py"
VENV_PY = ROOT / ".venv" / "Scripts" / "python.exe"
PY = str(VENV_PY) if VENV_PY.exists() else sys.executable

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
MIN_PNG_BYTES = 4 * 1024  # flat-colour renders (e.g. --only tile) compress to ~6 kB
TIMEOUT_S = 180

pytestmark = pytest.mark.slow

if not RUN_SIM.exists():
    pytest.skip(f"run_sim_software.py not found at {RUN_SIM}", allow_module_level=True)


# Each entry: (id, extra CLI args). --offscreen and --out are added by the test.
RENDER_CASES = [
    ("iso", ["--camera", "iso"]),
    ("top-tile", ["--camera", "top", "--only", "tile"]),
    ("tips-labels-tags", ["--tip-red", "far", "--tip-blue", "audience", "--labels", "--show-tags"]),
    ("robot-box-pose", ["--robot-box", "--pose", "72,24,90", "--frame", "ftc"]),
    ("hide-aim-axes", ["--hide", "perimeter_wall", "--show-aim-points", "--alliance-axes"]),
    ("side-x", ["--camera", "side-x"]),
    ("audience", ["--camera", "audience"]),
    ("low-lod-window", ["--lod", "low", "--window", "800x600"]),
]


def _run(args: list[str]) -> subprocess.CompletedProcess:
    """Run run_sim_software.py with the venv interpreter from ROOT; never raises on nonzero exit."""
    return subprocess.run(
        [PY, str(RUN_SIM), *args],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=TIMEOUT_S,
        env=os.environ.copy(),
    )


def _diag(proc: subprocess.CompletedProcess) -> str:
    return (
        f"returncode={proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}"
    )


@pytest.mark.parametrize("name,extra", RENDER_CASES, ids=[c[0] for c in RENDER_CASES])
def test_render_produces_valid_png(name, extra, tmp_path):
    out = tmp_path / f"{name}.png"
    proc = _run([*extra, "--offscreen", "--out", str(out)])

    assert proc.returncode == 0, "run_sim_software.py exited nonzero\n" + _diag(proc)
    assert out.is_file(), "PNG was not written to --out path\n" + _diag(proc)

    data = out.read_bytes()
    assert data[:8] == PNG_SIGNATURE, (
        f"bad PNG signature: {data[:8]!r}\n" + _diag(proc)
    )
    assert len(data) > MIN_PNG_BYTES, (
        f"PNG too small ({len(data)} bytes <= {MIN_PNG_BYTES})\n" + _diag(proc)
    )

    reported = str(out) in proc.stdout or out.resolve().as_posix() in proc.stdout
    assert reported, "stdout does not contain the PNG path\n" + _diag(proc)


def test_unknown_flag_exits_nonzero(tmp_path):
    proc = _run(["--offscreen", "--out", str(tmp_path / "x.png"), "--no-such-flag-xyz"])
    assert proc.returncode != 0, "unknown flag was accepted\n" + _diag(proc)
