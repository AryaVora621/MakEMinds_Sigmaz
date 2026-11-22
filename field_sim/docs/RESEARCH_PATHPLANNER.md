# Research: PathPlanner and AdvantageScope UX, and how the sim maps it to Pedro

## Sources (read for this revision, 2026-11)

- PathPlanner docs, Project Browser: https://pathplanner.dev/gui-project-browser.html
- PathPlanner docs, Editing Paths and Autos: https://pathplanner.dev/gui-editing-paths-and-autos.html
- PathPlanner docs, Build an Auto (PathPlannerLib side, `AutoBuilder.buildAutoChooser()`): https://pathplanner.dev/pplib-build-an-auto.html
- PathPlanner repository README (real-time preview, modular autos, automatic saving): https://github.com/mjansen4857/pathplanner
- AdvantageScope docs, 3D Field tab (timeline, camera modes, objects panel): https://docs.advantagescope.org/tab-reference/3d-field/
- Pedro API names were verified earlier against `core-2.1.0-sources.jar` (see `autoexport.js`).

What the pages state explicitly is marked (doc). Layout details that the text pages do not spell out (exact pixel layout, colours) come from working knowledge of the app
and are marked (recalled); treat those as design inspiration, not a spec.

## PathPlanner GUI

**Project browser (doc).** The main screen. Split in two: **Paths on the left, Autos on the right**, resizable by dragging the divider. Each file is a **card**
(default view shows a field preview, compact mode is a list). Click a card to open the editor, a back button returns. Click the name to rename; the card's menu has
**Duplicate** and **Delete**. Two icon buttons at the top right of each section create a **folder** and a **new path/auto**. Folders are one level deep; drag cards onto a
folder or onto the "root folder" card. Each section has independent sorting. A floating button (bottom right) manages **named commands** and **linked waypoints** across
the project (deleting a still-used named command leaves a warning on the affected cards). Files save automatically (README).

**Path editor (doc).** The field is the main view. *Double-click the field to add a waypoint* (appended), drag waypoints to move them. A side tree of collapsible sections:
- **Waypoints**: per waypoint a lock (no dragging), delete, X/Y of the anchor, heading, previous/next control length, buttons to add a rotation target or insert a waypoint after it.
  A *link* option shares a waypoint position with same-named waypoints in other paths.
- **Global constraints**: max velocity/acceleration, max angular velocity/acceleration, nominal voltage, "use default", unlimited.
- **Ideal starting state** and **Goal end state**: velocity and (holonomic) rotation, also draggable on the field.
- **Rotation targets** (holonomic): a rotation plus a position slider along the path.
- **Event markers**: a command group, optional *zoned* marker with start/end sliders; unzoned markers fire once.
- **Constraint zones**: own constraints plus start/end sliders; earlier zones win on overlap.
- **Point towards zones**, **Path optimizer** (genetic algorithm, Optimize / Discard / Accept), **Reversed** checkbox (non-holonomic).
- Editor settings (doc): snap to guidelines, hide other paths on hover (auto editor), show trajectory states, show robot details, show grid (0.5 m).
- (recalled) The side panel is on the right of the field, the preview is a **slider under the field** with a play button and the **estimated time**; the first waypoint is drawn
  green, the last red, the rest light; control points are small circles joined to their anchor by a thin line; a dark Material theme with one accent colour.

**Auto editor (doc).** An auto is a **sequential command group** containing path-following commands (any path of the project, reusable), **named commands**, **wait**
commands and nested groups; the tree "works the same as the command tree for event markers": add, remove, reorder. Group rules follow WPILib (a *deadline* group
uses its first command as the deadline; sequential, parallel, race are the others). Named commands must be confirmed with Enter when typed. Autos are loaded by name
(`new PathPlannerAuto("name")`) or all at once into a `SendableChooser` (`AutoBuilder.buildAutoChooser()`).

## AdvantageScope (viewer)

- **Timeline (doc):** click selects a time, right-click deselects, scroll to zoom, shift-drag selects a range, drag/scroll to pan; **green = autonomous, blue = teleop**;
  the selected time syncs across tabs; Ctrl+\ zooms to the enabled period.
- **3D field (doc):** objects panel (drag poses in, eye icon hides, X removes), per-object colour; camera modes Orbit Field, Orbit Robot, Driver Station, Fixed Camera.
- (recalled) transport buttons, playback speed, a robot ghost with a trail; compact dark UI, high contrast.

## What the sim copies (and where)

| PathPlanner / AdvantageScope idea | Here |
|---|---|
| Project browser with Paths and Autos, + button, rename/duplicate/delete, autosave | Auto tab left sidebar: **Autos** and **Paths** lists (rows with hover actions), `+` creates, duplicate/delete on hover, name edited at the top of the editor. Autos live in localStorage (`autos.js` library) and autosave on every edit. Paths live in a shared project library (`pathing.js` `createPathLibrary`, key `sim.paths.v1`) used by the path editor and the auto planner |
| Collapsible editor sections (Waypoints, Global constraints, Goal end state, Rotation, Event markers) | Path tab right panel (`pathui.js`): toolbar, **Paths** list, **Waypoints** (rows + selected point X/Y), **Global constraints** (preview speed/accel), **Start and goal end state**, **Path settings and rotation** (type, heading mode, start/end degrees, reversed, colour), **Event markers**, **Export** |
| Field: green start, red end, light middle anchors, small control points with thin handle lines, rotation handle | `pathui.js` `draw`: blue path with dark casing, green/red/white anchors, control points of the selected path joined by a dashed line, amber rotation handle at each waypoint (drag to rotate, 15 deg snap) |
| Auto editor: sequential command tree with named / wait / path / parallel / race / deadline, drag to reorder | `autoui.js`: **Auto sequence** tree (coloured icon badge per command kind, summary line, hover actions up/down/delete, drag handle, collapse), Inspector section for the selected command (indent, outdent, wrap in group, ungroup), command palette in the sidebar (click or drag) |
| Event markers and "estimated time" | Pedro `addParametricCallback`; markers on the path in the field (amber diamonds) and in the inspector; total shown in the timeline header with a 30 s autonomous limit marker and red warning |
| Preview slider with play button | Path tab: sticky transport at the bottom of the panel (first/back/play/forward/last/stop, speed, loop, scrubber with event ticks, time from the real Pedro follower). Auto tab: **bottom timeline** with transport, speed, time readout, ruler, one lane per parallel command, draggable playhead, `Preview` (start paused) and `Run in sim` |
| AdvantageScope timeline (cursor, auto period colouring, click to jump) | Auto timeline ruler with seconds, dashed red 30 s auto-period line, click or drag anywhere to scrub the runner, bars highlight while running |
| Warnings on cards | Validation section (empty groups, missing paths, bad conditions) with click-to-select, plus red total over 30 s |
| Mirror alliance | `Mirror` on routes and autos (x -> 144 - x, heading pi - h); Blue / Red segmented control |
| Export | Pedro 2.1.0 Java, two styles (state machine / Ivy), compile-checked structure tests |

Not copied: card previews of each file, folders, linked waypoints, constraint zones and rotation targets along the path (Pedro paths have a per-path heading interpolation only), the path optimizer.
Folders are skipped because the project is small; names are unique keys in both libraries.

## Design rules used for the UI

1. One accent colour (blue, from `ui.css --accent`), neutral dark surfaces, 8 px grid, 32 px controls (24 px for in-row icon buttons), visible focus rings.
2. Every icon button has a tooltip (`title`) and `aria-label`; icons are inline SVG (`stroke: currentColor`) so they follow the text colour.
3. Row actions appear on hover as an overlay, so rows never change height (no layout jitter); two-line command rows are fixed at 40 px.
4. Direct manipulation first (drag on the field), numeric fields second (inspector), no modal dialogs; edits use `change` events so typing is never interrupted.
5. Playback never blocks editing; edits re-plan the trajectory after a short debounce; the timeline fits the panel width (pixels per second adapt) so it needs no horizontal scrolling.

## Pedro-specific constraints that shape the design

- Pedro paths are Bezier chains with per-path heading interpolation (constant, linear, tangent) and optional parametric callbacks;
  there is no holonomic rotation-target-along-path and no constraint zones, so waypoint heading means "heading at the end of this path".
- Motion is produced by the Follower (PIDF/vector correction), not by a precomputed trajectory, so the honest time estimate comes from running the
  real follower (`POST /api/pedro/follow`); the local trapezoid model is only a fallback and is labelled as such in the UI.
- Export targets Pedro 2.1.0: `PathBuilder.addPath`, `set*HeadingInterpolation`, `setReversed`, `addParametricCallback(double, Runnable)`.
