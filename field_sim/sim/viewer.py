"""Interactive viewer: matplotlib (TkAgg) window showing frames from sim.render.SoftPlotter.

Controls (also printed at start):
  left-drag orbit | right-drag or middle-drag pan | wheel zoom
  1-9,0 toggle first 10 categories present | h toggle remaining categories | l toggle LOD low/full
  t toggle BOTH hive tips (animated ~1 s) | j red hive only | k blue hive only
  r reset camera + tips | v iso | b top | n side-x | m side-y
  p print Pedro + FTC coords of the point under the mouse | s screenshot | g grid | q/Esc close
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from .scene import CAMERAS, LOW_LOD_HIDDEN, apply_camera
from . import hive


def run_interactive(args, field, plotter, handles, root: Path) -> int:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    # free the keys for ourselves
    for k in list(matplotlib.rcParams):
        if k.startswith("keymap."):
            matplotlib.rcParams[k] = []

    W, H = plotter.window_size
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.canvas.manager.set_window_title("BIOBUZZ field simulator")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    im = ax.imshow(np.zeros((H, W, 3), np.uint8), aspect="auto", interpolation="nearest")
    state = dict(btn=None, last=None, mouse=None, hq=None, shift=False, lod_low=(args.lod == "low"),
                 others_visible=True, grid=args.grid, cam_name=args.camera, dirty=True)

    keymap = {}
    for i, cat in enumerate(handles.categories_present[:10]):
        keymap[str((i + 1) % 10)] = cat
    extra = handles.categories_present[10:]
    print("Category keys:")
    for k in sorted(keymap, key=lambda s: (s == "0", s)):
        print(f"  [{k}] {keymap[k]}")
    if extra:
        print(f"  [h] toggles {len(extra)} remaining categories: {', '.join(extra)}")
    print(__doc__)

    def draw(fast=False):
        size = (W // 2, H // 2) if fast else (W, H)
        img = plotter.render(size, ssaa=1 if fast else 2)
        im.set_data(img)
        im.set_extent((0, size[0], size[1], 0))
        ax.set_xlim(0, size[0])
        ax.set_ylim(size[1], 0)
        state["size"] = size
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    def schedule_hq():
        if state["hq"] is not None:
            state["hq"].stop()
        t = fig.canvas.new_timer(interval=350)
        t.single_shot = True
        t.add_callback(lambda: draw(False))
        t.start()
        state["hq"] = t

    plotter.on_render = lambda: draw(True)

    def pointer_px(event):
        # event.x,y are in figure pixels (origin bottom-left)
        w, h = fig.canvas.get_width_height()
        sx, sy = state["size"]
        return event.x / w * sx, (h - event.y) / h * sy

    def on_press(ev):
        state["btn"] = ev.button
        state["last"] = (ev.x, ev.y)

    def on_release(ev):
        state["btn"] = None
        schedule_hq()

    def on_move(ev):
        if ev.x is not None:
            state["mouse"] = (ev.x, ev.y)
        if state["btn"] is None or state["last"] is None or ev.x is None:
            return
        dx, dy = ev.x - state["last"][0], ev.y - state["last"][1]
        state["last"] = (ev.x, ev.y)
        cam = plotter.camera
        if state["btn"] == 1 and not state["shift"]:
            cam.orbit(-dx * 0.4, dy * 0.4)
        else:
            dist = np.linalg.norm(cam.position - cam.focal)
            scale = dist * 2 * np.tan(np.radians(cam.fov_deg) / 2) / fig.canvas.get_width_height()[1]
            cam.pan(-dx * scale, -dy * scale)
        draw(True)

    def on_scroll(ev):
        plotter.camera.zoom(0.88 if ev.step > 0 else 1 / 0.88)
        draw(True)
        schedule_hq()

    def set_cam(name):
        state["cam_name"] = name
        apply_camera(plotter, name)
        draw(False)

    def apply_lod():
        for c in LOW_LOD_HIDDEN:
            handles.set_category_visible(c, not state["lod_low"])

    def toggle_pick():
        if state["mouse"] is None:
            print("move the mouse over the field, then press p")
            return
        w, h = fig.canvas.get_width_height()
        mx, my = state["mouse"]
        sx, sy = state["size"]
        pt = plotter.unproject(mx / w * sx, (h - my) / h * sy)
        if pt is None:
            print("PICK: no surface under cursor")
        else:
            handles.on_pick(pt)

    def on_key(ev):
        k = ev.key
        if k is None:
            return
        state["shift"] = False
        if k in ("q", "escape"):
            plt.close(fig)
        elif k in keymap:
            on = handles.toggle_category(keymap[k])
            print(f"{keymap[k]}: {'shown' if on else 'hidden'}")
            draw(False)
        elif k == "h" and extra:
            state["others_visible"] = not state["others_visible"]
            for c in extra:
                handles.set_category_visible(c, state["others_visible"])
            draw(False)
        elif k == "l":
            state["lod_low"] = not state["lod_low"]
            apply_lod()
            print("LOD", "low" if state["lod_low"] else "full")
            draw(False)
        elif k in ("t", "T"):
            tg = handles.toggle_tip(("red", "blue"))
            print("tip:", tg)
            schedule_hq()
        elif k == "j":
            print("tip:", handles.toggle_tip(("red",)))
            schedule_hq()
        elif k == "k":
            print("tip:", handles.toggle_tip(("blue",)))
            schedule_hq()
        elif k == "r":
            handles.reset_tips()
            set_cam("iso")
            print("reset camera + tips")
        elif k in ("v", "b", "n", "m"):
            set_cam({"v": "iso", "b": "top", "n": "side-x", "m": "side-y"}[k])
        elif k == "p":
            toggle_pick()
        elif k == "s":
            out = root / "docs" / "renders" / f"screenshot_{time.strftime('%Y%m%d_%H%M%S')}.png"
            handles.screenshot(out)
            print("saved", out)
        elif k == "g":
            state["grid"] = not state["grid"]
            from . import overlays
            from .scene import draw_polylines
            if state["grid"]:
                draw_polylines(plotter, overlays.grid_polylines(), "grid")
            else:
                plotter.overlays.pop("grid", None)
            draw(False)

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_move)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)

    if args.auto_close:
        t = fig.canvas.new_timer(interval=int(args.auto_close * 1000))
        t.single_shot = True
        t.add_callback(lambda: plt.close(fig))
        t.start()
        state["auto"] = t

    draw(False)
    plt.show()
    print("viewer closed")
    return 0
