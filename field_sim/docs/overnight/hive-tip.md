# Hive tip model (area: hive-tip)

Owned files: web/js/hivecycle.js, web/js/hivemesh.js, web/js/hivetip_model.js, tests_js/hivetip*.test.mjs, this file.
This task did not edit any code. The model below was already in place; this file documents it and records the test run.

## 1. Model summary

- Pure model: `web/js/hivetip_model.js` (no DOM, no state). Dynamics: `HiveDynamics` in `web/js/hivecycle.js`.
- Units: "pollen-equivalent weight". One pollen = `pollenWeight` = 1. One nectar = `nectarWeight` = 0.091 / 0.055 = 1.6545.
- Threshold: `threshold` = 8 pollen-equivalent units. Pure pollen tips at 8.
- Tip weight of counted balls: `tipWeight = pollen * 1 + nectar * 1.6545`.
- `shouldTip(counts)`: `tipWeight >= 8 - 1e-9`.
- `tipProgress(counts)`: `clamp(tipWeight / 8, 0, 1)`. Monotone in each count. `HiveDynamics.tipProgress` in weight mode is the same quantity over the up cell.
- `nectarToTip()` = `ceil(8 / 1.6545)` = 5 nectar alone (derived from the weight, not measured).
- `ballsNeeded(counts, type)` = `ceil(remaining / weight(type))`, 0 when already tipped.
- Counts are clamped: negative or NaN becomes 0, fractions round down.
- Mixed examples (linear sum, ASSUMED): 4 pollen + 1 nectar = 5.65 (no tip); 4 pollen + 2 nectar = 7.31 (no tip, `tipProgress` 0.91); 4 pollen + 3 nectar = 8.96 (tip).
- Dynamics default is `mode: 'weight'` (`HIVE_PARAMS` in hivecycle.js). `mode: 'count'` still exists and uses `ballsToTip` (set to 8 now) as a plain ball count.
- Only balls in the currently up cell, with the hive at rest, have `counts = true`. `reset()` stores 3 starting nectar in the up cell with `counts = false`. On a tip, every ball gets `counts = false` (the new up cell starts empty). `onTip(state, count)` fires on a natural tip only, not on `setState`.

## 2. Assumptions table

| Item | Value | SOURCED or ASSUMED | URL or reason |
|---|---|---|---|
| Pollen tip threshold | 8 pollen | SOURCED (user-authoritative). Manual silent. | User's own observation of the hive. Trigger wording from the game postcard (search snippet only, not fetched): https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf |
| Max held balls | 4 | SOURCED | User; docs/OVERNIGHT_RULES.md ("Official ball hold limit is 4"). This is the robot carry limit, not an up-cell limit. |
| pollenWeight | 1 | ASSUMED | Unit scale. Only the nectar/pollen ratio matters. |
| nectarWeight | 1.6545 | ASSUMED (the ratio is third-party, the tip effect is not) | Mass ratio 0.091 lb / 0.055 lb from https://andymark.com/products/ftc-2026-27-biobuzz-game-sets (search snippet, not fetched). Plex also lists pollen 0.055 lb. Using mass as tip weight is an assumption. |
| Nectar count to tip | 5 alone (derived) | ASSUMED | No source found for a nectar tip count. Derived as ceil(8 / 1.6545). |
| Mixed nectar + pollen rule | Linear weight sum | ASSUMED | No source found for a mixed-piece tip rule. |
| Starting nectar in up cell | 3 | SOURCED to the manual per repo comments (10.3.1.B); CONFLICT | hivecycle.js:14, :51. docs/overnight/research_manual.md:37 says 5. Not resolved here. |
| Starting nectar counts toward tip | No (0) | ASSUMED | Decision in hivecycle.js header. Needed so an empty up cell still needs 8 pollen (user's fact). |
| Only up-cell balls count | Yes | ASSUMED | Modelling choice. Manual 10.5.1 (per repo header) says tips come from launching into the up cell. |
| Post-tip cell empties of count | Yes (counts = false) | ASSUMED | Manual does not say whether balls already in the new up cell count. |
| Down-cell balls | Stored, do not count, not removed (releaseOnTip = false) | ASSUMED | Not modelled as resisting the next tip. |
| Pollen diameter | Not used by the model | n/a | Digests disagree (about 3 in from a search snippet; 2.8 in in the repo, research_manual.md:31). |
| "4 pollen tip the hive" | Rejected | UNCONFIRMED | Team build-thread anecdote (chiefdelphi 12737). Contradicts the user's 8, so not used. |

## 3. Research log

Method: the research digests supplied for this task (research 1 to 5) plus local file reads. No web fetch was done in this task. Items are labelled as the digests labelled them.

- Manual (docs/overnight/research_manual.md, Sec 8, 9.6, 10.3.1, 10.5.1): no tip count. Says tips come from launching into the up cell and that "enough" scoring elements are needed. Confirms the 8-pollen fact is outside the manual.
- Postcard https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf: trigger wording "when enough POLLEN or NECTAR is launched into the CELL the HIVE will TIP". No count. Not fetched.
- Blog https://www.firstinspires.org/community-blog/game-preview-field-elements: pollen about 3 in (search snippet, not fetched). Not used.
- https://www.detroitautomationacademy.com/signal/edition-01/: a tip unlocks one more nectar. Third party. Says nothing about the tip count.
- https://youtubesummary.com/summary/sUH3z5a5S9I: nectar "heavier, can tip faster". No figure. Secondary.
- https://www.chiefdelphi.com/t/12737-2026-2027-season-build: "4 pollen ... tip over using momentum". Anecdote, unconfirmed, rejected above.
- AndyMark and Plex (search snippets): nectar 0.091 lb, pollen 0.055 lb. Mass only.
- Local code (hivecycle.js, hivetip_model.js, research_manual.md): ballsToTip was 4 (a wrong guess, per user). Now 8 via threshold.
- FRC sims: lbarks27/FRC-2025-Example, Autodesk Synthesis (titles only). No hive-tip or threshold-counting logic found.
- BIOBUZZ resources page https://www.firstinspires.org/community-blog/biobuzz-kickoff-and-season-resources: overview only.
- https://github.com/8696-Trobotix/Decode: 2025-26 robot code. Not relevant.

Not found: a sourced nectar tip count, a nectar or pollen mass in grams, a mixed-tip rule, a sourced starting-nectar count that agrees with the manual copy.

## 4. How to change the values

- Threshold, weights, hold limit, starting nectar: `HIVE_TIP` in `web/js/hivetip_model.js`. Change the value there and update its SOURCE ledger tag.
- Dynamics defaults: `HIVE_PARAMS` in `web/js/hivecycle.js` (`mode`, `nectarWeight`, `pollenWeight`, `tipWeightThreshold`, `startingNectar`).
- At run time: `HiveDynamics.setRule({ mode, ballsToTip, nectarWeight, pollenWeight, tipWeightThreshold })`. Takes effect on the next ball. `mode` must be 'count' or 'weight'. `setRule` does not change `startingNectar` (only `reset()` reads it).
- Measuring nectar weight without a tip count: put 10 pollen and 10 nectar on a kitchen scale. nectarWeight = (nectar grams / 10) / (pollen grams / 10). This checks the mass assumption only.
- Measuring the real tip count: on the real hive, launch only nectar into the up cell, one at a time, and count the launches at which it tips (repeat several times). Then do the same with pollen to confirm 8. Then test a few mixed loads (for example 4 pollen + k nectar) to check the linear-sum rule. Replace the ASSUMED rows with the measured numbers.

## 5. Tests and results

Test files owned by this area: tests_js/hivetip_model.test.mjs, tests_js/hivetip.test.mjs, tests_js/hivetip_mixed.test.mjs, tests_js/hivecycle.test.mjs.

- Hive-specific files (`node --test tests_js/hivetip*.test.mjs tests_js/hivecycle.test.mjs`): 47 tests, 47 pass, 0 fail, 0 skipped.
- Full suite (`node --test tests_js/*.test.mjs` from field_sim, shell glob form): 555 tests, 550 pass, 4 fail, 1 skipped. Log: `C:\Users\Jindal\AppData\Local\Temp\claude\1bf884c3629a4194\cab1052d-b484-449d-8151-8eddbe3da6ad\scratchpad\hive-tip-test.log`
- Spot check: nectarToTip() = 5; shouldTip(4 pollen, 2 nectar) = false; shouldTip(4 pollen, 3 nectar) = true; tipProgress(4 pollen, 2 nectar) = 0.914.

Failing tests, outside owned files (reported, not fixed):

1. tests_js/physics3d.test.mjs:44 "intake: balls touching / just ahead of the roller inside the mouth width are taken, front and rear". Got 4, expected 5. Not related to the hive model.
2. tests_js/shootend2end.test.mjs:44 "e2e: a solver shot from the real turret/hood/rpm scores and tips the hive (default rule, either alliance)". "red did not tip" (0 !== 1).
3. tests_js/shootend2end.test.mjs:58 "e2e: continuous auto fire every 0.15 s ... keeps hitting the opening". "only 8/30 scored (flywheel rpm sag)".
4. tests_js/shootend2end.test.mjs:95 "e2e: unlimited shots are recycled ... stay fast". "caps 8".

Possible cause for 2 to 4, NOT verified: shootend2end.test.mjs builds the hive with `ballsToTip: 1000` or `100000` to stop tipping. The default mode is now 'weight' (threshold 8), so `ballsToTip` is ignored and the hive tips at 8 pollen. The owner of shootend2end.test.mjs should set `mode: 'count'` or a weight threshold in those builds, then re-run. This is a follow-up for that owner, not a change made here.
