# Overnight setup: three sessions, one per account

Run these yourself. Each session uses only its own account's usage. If a session hits its limit it stops and resumes after the reset. Do not move work between accounts automatically.
Check Anthropic's terms for using multiple personal accounts before relying on this.

## One-time login (PowerShell)
```powershell
mkdir $HOME\.claude-acct2, $HOME\.claude-acct3, $HOME\.claude-acct4
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct2"; claude   # /login with account 2, then exit
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct3"; claude   # /login with account 3, then exit
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct4"; claude   # /login with account 4, then exit
```
Credentials stay in those folders under your Windows profile. Do not copy or share them.

## Each night: three terminals, started in
`C:\Users\Jindal\StudioProjects\MakEMinds_Sigmaz\field_sim`

```powershell
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct2"; claude   # terminal 1: paste BRIEF 1
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct3"; claude   # terminal 2: paste BRIEF 2
$env:CLAUDE_CONFIG_DIR = "$HOME\.claude-acct4"; claude   # terminal 3: paste BRIEF 3
```
Use Sonnet in each (`/model`). Do not start anything on your main account.

## Shared preamble (put at the top of every brief)
Read field_sim/docs/OVERNIGHT_RULES.md first. Nobody is available to answer questions: decide, record the decision, continue. Edit only the files listed for you. Never run git add/commit/stash/push. Check the clock; stop starting new work at 06:30 and write your report to docs/overnight/<name>.md before 07:00. Use at most 2 Sonnet subagents.

## BRIEF 1: planner-ui
You own: web/js/pathui.js, autoui.js, pathing.js, autos.js, web/css/planner.css, tests_js/path*.test.mjs, tests_js/auto*.test.mjs.
Rebuild the Auto planner editing like FRC PathPlanner:
1. The robot is NOT draggable. Edit only by waypoints (anchors) with bezier control handles and a rotation handle. Click empty field = append a connected waypoint; drag anchor/handle to move; right-click or Delete removes; click a segment to insert. Draw the robot only as a footprint at the start/selected waypoint and during preview.
2. Alliance-limited view: show only the half of the field relevant to the chosen alliance, as PathPlanner does. Get the zone geometry from the BIOBUZZ game manual (docs/ first, then the web; cite). Blue/Red toggle crops and flips; mirror must keep working; full-field is an optional toggle.
3. Match PathPlanner's dark theme and colours. Take real values from github.com/mjansen4857/pathplanner (Flutter source under pathplanner/lib: theme and path painter). Record the sourced hex values in your report.
4. Keep the data model fields that autoexport.js and ppformat.js read intact; run their tests.
5. Keep the onPreviewPath/run hooks working for the 3D view.
Add tests for new editing logic. Screenshot at 1600x900 and 1280x720 (headless Chrome). Do not claim mouse behaviour works unless you drove it with simulated pointer events.

## BRIEF 2: sim-physics
You own: web/js/balls.js, collision.js, hivecycle.js, hivemesh.js, shotrules.js, shot_core.js, aimcontrol.js, flowerarm.js, flowerarm3d.js, robotmodel.js, robotspec.js, web/data/flower_arm.json, web/data/robot_spec.json, and their tests.
1. Intake grabs balls too much. Make capture realistic: only balls that touch the roller inside the intake mouth width are taken, with a small bounded pull radius, no vacuuming from beside or behind, no scooping through the robot body. Put the numbers in named constants.
2. The drop-down flower does not come down. Find the real cause (toggled by G and the auto 'flower' command; app.js calls flowerArm.update, findFlowerContact, feedFromFlower and passes flowerArmDeg to built.animate). Prove with a headless Chrome check that the angle changes and the mesh rotates. Make the arm geometry, animation and ball pickup match the manual and VEX Push Back style drop-down arms (research, cite). If the cause is in app.js, write the exact patch in your report and do not edit app.js.
3. Move the robot model toward MaxBot (381x375 mm frame, 104 mm mecanum wheels, front and rear intakes, turret ring, one flywheel with hood). Use the Memory Vault notes if you have access. Keep robotglb.js loading working.
4. Audit shots/hive: shots score only through hive entrance openings; hive tips; no tunnelling or stuck balls. Keep holdLimit at 4.
Add regression tests that fail without each fix.

## BRIEF 3: integrator-java
You own: web/js/app.js, web/index.html, web/css/ui.css, web/js/autorunner.js, web/js/driver.js, autoexport.js, ppformat.js, scheduler.js, pedrofollow.js, pedro_bridge.py, tools_java/*, and TeamCode/src (main and test).
1. Any auto from the planner must run in the 3D view: Run in sim switches to 3D, resets the field, drives the real Pedro follower trajectory (pedro_bridge live follow, else the in-browser fallback), executes intake/shoot/flower/wait in the real physics, shows progress, Stop and Restart. Re-read the files of the other two sessions before integrating and at the end.
2. Remove the stale label "Show shootable area (2D map)".
3. Pedro export: compile an exported sample with javac against the Pedro 2.1.0 jar and make it fit how TeamCode uses Pedro. Do not use chain callbacks (broken in 2.1.0). Do not invent tuned constants.
4. Java: read docs/reviews/opus_ftc_review.md, sonnet_a_shooter_review.md, sonnet_b_pedro_review.md. Check each finding against the code, then fix the clearly correct ones (turret zero lost between auto and teleop, flywheel MIN_RPM readiness, ball count on reverse, demo-auto timeouts and try/finally, Robot.stop() not cancelling the follower, FAR_CELL_UP, LQR clamped voltage, turret limit overlap). Do not change MAX_BALLS (needs the user's answer) and do not build TeleOp auto-park. Check git status first: an earlier run was stopped midway and may have left partial edits in TeamCode, so review them rather than overwrite.
Try ../gradlew.bat to compile SimJson, SimBridge, Robot.java and run SimJsonTest. Report fixed / rejected / deferred per finding.
