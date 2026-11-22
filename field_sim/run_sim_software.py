"""Field simulator CLI.  See --help.  Software (numpy/Pillow) renderer; interactive via Tk."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from sim.field import Field                      # noqa: E402
from sim.frames import ftc_to_pedro               # noqa: E402,F401
from sim.loader import CATEGORIES, load_model     # noqa: E402
from sim.render import SoftPlotter                # noqa: E402
from sim.scene import CAMERAS, SceneOpts, apply_camera, build_scene   # noqa: E402


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="BIOBUZZ field 3D simulator (software renderer)")
    ap.add_argument("--offscreen", action="store_true", help="render one PNG and exit")
    ap.add_argument("--out", default=None, help="output PNG (offscreen)")
    ap.add_argument("--camera", default="iso", choices=CAMERAS)
    ap.add_argument("--tip-red", choices=["audience", "far"], default=None)
    ap.add_argument("--tip-blue", choices=["audience", "far"], default=None)
    ap.add_argument("--hide", default="", help="comma list of categories to hide")
    ap.add_argument("--only", default="", help="comma list: show only these categories")
    ap.add_argument("--alliance-axes", action="store_true", help="draw coordinate axes of --frame")
    ap.add_argument("--labels", action="store_true", help="AprilTag IDs and zone names")
    ap.add_argument("--frame", choices=["pedro", "ftc"], default="pedro")
    ap.add_argument("--show-aim-points", action="store_true")
    ap.add_argument("--show-tags", action="store_true", help="tag squares + normal arrows")
    ap.add_argument("--robot-box", action="store_true")
    ap.add_argument("--pose", default=None, help="x,y,heading_deg in Pedro (default 72,24,90 with --robot-box)")
    ap.add_argument("--model-dir", default=None)
    ap.add_argument("--lod", choices=["full", "low"], default="full")
    ap.add_argument("--window", default="1280x800", help="WxH")
    ap.add_argument("--grid", action="store_true", help="draw 24 in grid")
    ap.add_argument("--no-zones", action="store_true")
    ap.add_argument("--ssaa", type=int, default=2, help="supersampling factor for final renders")
    ap.add_argument("--max-tris", type=int, default=400000)
    ap.add_argument("--auto-close", type=float, default=None, help="interactive: close after N seconds (test hook)")
    return ap.parse_args(argv)


def build(args):
    mdir = Path(args.model_dir) if args.model_dir else None
    field = Field(load_model(mdir))
    if args.tip_red:
        field.set_tip("red", args.tip_red)
    if args.tip_blue:
        field.set_tip("blue", args.tip_blue)
    w, h = (int(v) for v in args.window.lower().split("x"))
    plotter = SoftPlotter((w, h))
    plotter.max_tris = args.max_tris
    pose = None
    if args.pose:
        pose = tuple(float(v) for v in args.pose.split(","))
        if len(pose) != 3:
            raise SystemExit("--pose needs x,y,heading_deg")
    elif args.robot_box:
        pose = (72.0, 24.0, 90.0)
    opts = SceneOpts(lod=args.lod,
                     hide=tuple(x for x in args.hide.split(",") if x),
                     only=tuple(x for x in args.only.split(",") if x),
                     alliance_axes=args.alliance_axes, labels=args.labels, frame=args.frame,
                     show_aim_points=args.show_aim_points, show_tags=args.show_tags,
                     robot_box=args.robot_box or args.pose is not None and args.robot_box,
                     pose=pose, grid=args.grid, zones=not args.no_zones)
    handles = build_scene(plotter, field, opts)
    apply_camera(plotter, args.camera)
    return field, plotter, handles


def main(argv=None):
    args = parse_args(argv)
    field, plotter, handles = build(args)
    if args.offscreen:
        out = Path(args.out) if args.out else ROOT / "docs" / "renders" / f"offscreen_{time.strftime('%Y%m%d_%H%M%S')}.png"
        t0 = time.time()
        handles.screenshot(out) if args.ssaa == 2 else plotter.screenshot(out, ssaa=args.ssaa)
        if not out.exists() or out.stat().st_size == 0:
            print("ERROR: PNG not written", file=sys.stderr)
            return 1
        print(f"{out.resolve()}  ({time.time() - t0:.1f}s)")
        return 0
    from sim.viewer import run_interactive
    return run_interactive(args, field, plotter, handles, ROOT)


if __name__ == "__main__":
    sys.exit(main())
