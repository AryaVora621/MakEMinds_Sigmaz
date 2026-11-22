"""Produce the standard QA renders via run_sim_software.py (offscreen).

Usage: python tests/make_renders.py [--model-dir DIR] [--out-dir DIR]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

RENDERS = {
    "iso": ["--camera", "iso"],
    "top": ["--camera", "top"],
    "side_x": ["--camera", "side-x"],
    "audience": ["--camera", "audience"],
    "tip_red_far_blue_audience": ["--camera", "iso", "--tip-red", "far", "--tip-blue", "audience"],
    "tip_red_audience_blue_far": ["--camera", "iso", "--tip-red", "audience", "--tip-blue", "far"],
    "labels_iso": ["--camera", "iso", "--labels", "--show-tags"],
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", default=None)
    ap.add_argument("--out-dir", default=str(ROOT / "docs" / "renders"))
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    py = ROOT / ".venv" / "Scripts" / "python.exe"
    exe = str(py) if py.exists() else sys.executable
    bad = 0
    for name, flags in RENDERS.items():
        png = out / f"{name}.png"
        cmd = [exe, str(ROOT / "run_sim_software.py"), "--offscreen", "--out", str(png), *flags]
        if a.model_dir:
            cmd += ["--model-dir", a.model_dir]
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        ok = r.returncode == 0 and png.exists() and png.stat().st_size > 0
        print(("OK   " if ok else "FAIL ") + name, png)
        if not ok:
            bad += 1
            print(r.stdout[-500:], r.stderr[-1500:])
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
