# flower report (BRIEF 2 items 2 and 3)

## Item 2: flower arm "not coming down"
Owned files edited: web/js/flowerarm.js, tests_js/flowerarm.test.mjs. Nothing else touched.

Investigation (headless Chrome, SwiftShader, own server on :8800, stopped after):
- Pressing G (and `flowerArm.deploy()` + `__sim.step`) changes state stowed -> deploying -> deployed, angle 90 -> -35, `built.parts.flowerArm.hinge.rotation.y` -1.571 -> +0.611, paddle world pos (72, 38.7, 3.3). Close-up screenshots stowed vs deployed confirm the mesh rotates and drops (scratchpad fl_1.png, fl_2.png).
- So with the current files and no data/robot.glb the arm DOES come down. I could not reproduce a dead arm. Earlier partial edits (files dated Nov 6 19:5x) appear to have already fixed the logic; the tree is untracked so I cannot diff.
- Remaining real causes found:
  1. FIXED: `new FlowerArm(flowerArmCfg || undefined)` in app.js crashed (`this.cfg.angles` of undefined) if data/flower_arm.json fails to load. FlowerArm now merges the config over a full DEFAULT_FLOWER_CFG (exported). Regression test added.
  2. LIKELY, UNVERIFIED (no robot.glb here): in app.js, when data/robot.glb loads, `robot.children.forEach(c => c.visible = false)` hides the whole procedural robot including the flower arm, and the CAD model has no animated arm. G then toggles state but nothing visible moves. app.js patch (after `robot.add(r.model.group);`):
     `if (built.parts.flowerArm) { built.parts.flowerArm.group.visible = true; }`
     (the arm group is a direct child of `robot`; keeps the procedural arm drawn on the CAD body until the CAD arm is animated). Also guard: `const flowerArm = new FlowerArm(flowerArmCfg || undefined)` is now safe as is.
  3. Visual: arm is small (8.2 in link, -35 deg) and sits behind the front plates/turret from the default iso camera, so it reads as barely moving. Geometry numbers left unchanged (all placeholders, no CAD; flower_arm.json cad_todo still open).
- Flower geometry cross-check (docs/FIELD_CONSTANTS.md): scoring volume z 4.25-21.05, ball stack bottom z ~1.4; tip z 3.3 deployed and reach 1-19 are consistent with pushing a stack out.

Research (limited): VEX Push Back match loaders use a pivoting tongue/paddle that swings into the loader tube, friction rubber on the block (VEX Forum: https://www.vexforum.com/t/help-me-please-i-need-a-match-loader-and-feedback-on-how-to-improve/142541 and /mechanism-for-getting-the-blocks-out-of-loader/141339, from search snippets only; both pages returned 403 to fetch). Current geometry (pivoting paddle with rubber lip) matches that style. BIOBUZZ manual pages (ftc-resources.firstinspires.org manual-10, cm-html V1) returned 403 / binary; flower scoring facts (volume between top and middle ring, G418 no meddling with FLOWERs, scoring window last 60 s) come from third-party summaries https://ftc.uprobotics.tech/questions/78 and are unverified against the manual. Whether robots may pull pollen from a flower stack: UNVERIFIED, check G418 before trusting feedFromFlower.

## Item 3: MaxBot-like model
Already implemented in the tree, verified by existing tests (robotmodel/robotspec): 381x375 mm frame, 4x104 mm mecanum, dual front/rear folding intakes, turret ring 105 mm bore, one 96 mm flywheel with hood, gates, odometry pods. robotglb.js loading untouched and its tests pass. Memory Vault was not connected, so no new values were pulled; I changed nothing and invented no constants.

## Evidence
`node --test tests_js/*.test.mjs`: 467 pass, 2 fail. Both failures are pathui tests (planner-ui owner, in progress), not mine. Added 2 tests: arm without config deploys/feeds; logic angle drives hinge rotation and paddle world position drops >8 in.
Note `node --test tests_js` (directory form) fails on this Node 24; use the `*.test.mjs` glob.

## Not verified
CAD-glb path, real hardware angles/times, manual rule on bottom collection.
