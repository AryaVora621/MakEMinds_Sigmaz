# Overnight work rules (read first, every worker)

Project: `C:\Users\Jindal\StudioProjects\MakEMinds_Sigmaz\field_sim` (FTC BIOBUZZ 2026-27 web/three.js field simulator + Pedro Pathing bridge) and `..\TeamCode` (robot Java).
Nobody is available to answer questions. Make the best decision, write it down in your report, and keep going.

## Hard rules
- NEVER run git add/commit/stash/push/checkout/reset. Read-only git (status, diff, log) only. The user forbids commits.
- Edit ONLY the files you own (listed in your brief). If you need a change elsewhere, describe it in your report; do not edit.
- Do not kill processes you did not start. A test server is on port 8792; do not touch it. Start your own on a free port (8800+) if needed and stop only that one.
- Do not build the TeleOp auto-park feature.
- Do not invent robot constants (Pedro gains, measured values). Mark TODO instead.
- Do not fabricate sources. Research claims need a URL or file path; label anything else "unverified".

## How to verify
- JS tests: from field_sim, `node --test tests_js` (all must stay green; add tests for what you change).
- Python: `.venv\Scripts\python.exe` (bash PATH has no `python`).
- Visual check: headless Chrome screenshot (SwiftShader) saved to the scratchpad directory; actually look at the image.
- A feature is "done" only with evidence (test or screenshot). Say plainly what you could NOT verify.

## Report
Write `docs/overnight/<your-name>.md`: what changed (files), evidence, what is unverified, follow-ups for other owners. Keep it under 80 lines.

## Domain facts
- Pedro frame: inches, origin audience-left corner, x right, y toward far wall, heading radians. Field 144x144.
- Pedro 2.1.0 chain callbacks never fire; do not rely on them.
- Official ball hold limit is 4; keep `holdLimit` at 4.
- Keyboard: W/S forward/back, Q/E strafe, A/D turn, G flower.
- Tabs are exactly: 3D view, Auto planner, Pedro.
