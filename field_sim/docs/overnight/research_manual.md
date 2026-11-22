# BIOBUZZ 2026-27 game manual facts (research)

Scope: read-only research. Only this file was written. No code or other docs changed.

Sources used:
- [S1] Online manual HTML: https://ftc-resources.firstinspires.org/ftc/game/cm-html (fetched via WebFetch in 3 windows, offsets 0 / 100000 / 200000 / 300000; the summarizer quoted section numbers). The version/TU number of this copy was NOT confirmed. Quotes below are summarizer output, not copy-checked by eye.
- [S2] Repo: field_sim/web/js/matchrules.js (cites "TU08"), field_sim/docs/FIELD_CONSTANTS.md (cites "TU07"), TeamCode/src/main/java/commands/field/FieldConstants.java:5 (cites "TU07").
- [S3] Kickoff/community pages (firstinspires.org) confirm season date and that the full manual shipped at kickoff; no rule text there.

Status legend: CONFIRMED = quoted in S1 excerpt; REPO = repo claim, not seen in S1; UNVERIFIED = neither.

## Version warning
- Repo cites two different manual versions: matchrules.js:2,8 says TU08; FIELD_CONSTANTS.md:1-7 and FieldConstants.java:5 say TU07. Pick one and check it against the current manual before trusting section numbers. Not resolved here.

## Field zones
| Fact | Value | Status | Source |
|---|---|---|---|
| Loading zone size | approx 23 x 11 in, "infinitely tall volume" (Sec 9.3) | CONFIRMED (S1). Repo CAD 22.69 x 11.0 (FIELD_CONSTANTS.md:70-71, 99) | S1; S2 |
| Alliance area size | approx 97 x 54 in (Sec 9.3) | CONFIRMED (S1). Repo 96.8 x 54 (FIELD_CONSTANTS.md:74, 101) | S1; S2 |
| Garden size | not in S1 excerpt | REPO only: 22.69 x 2.0 (FIELD_CONSTANTS.md:72-73, 100) | S2 |
| Drive team location | stages in own alliance area (Sec 9.5); may not leave it during match (G422) | CONFIRMED | S1 |
| Start: robot | must be on own alliance side, not in scoring volume of a flower, not in loading zone (G304.A/D/E); touch perimeter wall; 18 in cube starting config (R102) | CONFIRMED | S1 |
| Start: hive tilt | one cell down, one cell up (Sec 10.3.1) | CONFIRMED | S1 |
| Start: start-position coordinates / Fig 11-1 | Figure 11-1 not readable from text | UNVERIFIED | - |
| Launch zone | no definition in any excerpt read; "LAUNCHED" is used but no zone is named | CONFIRMED absent in S1 (portions 0-300k only). Sections 10.3-10.5 partly seen; G-rules 407-427 partly seen | S1 |
| Gate | not mentioned in any excerpt read | UNVERIFIED (absent from fetched text; repo's "gate" hits are for the robot's own subsystem, e.g. FIELD_CONSTANTS and sonnet_a_shooter_review.md, not the field) | S1 |
| Repo "legal area" | repo says "game rules have no launch zone", so legal area is an assumption (README.md:54) | consistent with S1 | S2 |

## Pollen (scoring ball)
| Fact | Value | Status | Source |
|---|---|---|---|
| Diameter | approx 2.8 in (7.1 cm), polyethylene, yellow (Sec 9.8) | CONFIRMED (S1). Repo 2.80 in (FIELD_CONSTANTS.md:77) | S1; S2 |
| Mass | not in excerpts | UNVERIFIED | - |
| Count per match | 40 pollen, 8 red nectar, 8 blue nectar (Sec 9.8) | CONFIRMED (S1). Not stated in repo rules | S1 |
| Robot preload | 4 pre-loaded pollen each (Sec 10.3.4; G304.G) | CONFIRMED (S1). Repo cites "10.3.1" (matchrules.js:10, 44): section number mismatch, the value matches | S1; S2 |
| Hold limit | "No more than 4 at a time. A ROBOT may not simultaneously CONTROL more than 4 SCORING ELEMENTS" (G407) | CONFIRMED (S1). Repo holdLimit 4 (matchrules.js:9) matches | S1; S2 |
| Hold limit tolerance | repo says 5 momentary tolerated, 6+ strategic (matchrules.js:9) | UNVERIFIED (not in S1 excerpt) | S2 |

## Nectar (human-entered)
- Repo: 5 nectar per alliance area at start (10.3.1), one per own-hive tip (G426), all remaining at 60 s left (matchrules.js:31-35). S1 confirms the G426 60-second rule ("when 60 seconds or less remain ... all remaining NECTAR can be entered") and "With 60 seconds left ... ALLIANCES can enter all remaining NECTAR" (Sec 10.1). The "5 at start" and "one per own tip" are UNVERIFIED in S1. Nectar diameter 3.62 in (FIELD_CONSTANTS.md:77) UNVERIFIED.

## Flowers
| Fact | Value | Status | Source |
|---|---|---|---|
| Interaction | POLLEN and NECTAR placed into the top; POLLEN can be removed from the bottom (Sec 9.7; G418) | CONFIRMED | S1 |
| Opening | approx 4 in (10.15 cm) diameter (Sec 9.7). Repo 3.96 in (FIELD_CONSTANTS.md:63-65) | CONFIRMED (S1) | S1; S2 |
| Height | opening approx 21.5 in (Sec 9.7). Repo z 21.40 (FIELD_CONSTANTS.md:96) | CONFIRMED only approximate (S1 says 9.7 p70 in repo) | S2 |
| Scoring | only way to score is placing elements into the top (Sec 10.5.2) | CONFIRMED | S1 |
| Scoring timing | no flower scoring until 60 s remain (G410; excerpt states G410 text: may not enter NECTAR into scoring volume until last 60 s) | CONFIRMED (S1); matches repo matchrules.js:17 | S1; S2 |
| Ownership | alliance with the top-most NECTAR of its color owns the flower (Sec 10.5.2) | CONFIRMED (S1). Repo: flowers have no fixed owner (FIELD_CONSTANTS.md:58) | S1; S2 |
| Points in owned flower | 2 per pollen/nectar (Table 10-2, see below) | CONFIRMED value, column reading UNVERIFIED | S1 |
| Bottom nectar bonus | 5 pts per flower, TELEOP (Table 10-2) | CONFIRMED value, column reading UNVERIFIED | S1 |
| Flower positions | repo CAD: (48.6,140.1), (140.1,95.4), (95.4,3.9), (3.9,48.6) (FIELD_CONSTANTS.md:61-64) | REPO CAD only. Manual figure reading (Fig 9-2 per repo) UNVERIFIED in S1 | S2 |

## Hive
| Fact | Value | Status | Source |
|---|---|---|---|
| Structure | two cells, both red or both blue (Sec 9.6.2) | CONFIRMED | S1 |
| Bi-stable | holds position until enough elements launched into the up-facing cell (Sec 9.6) | CONFIRMED | S1 |
| Balls to tip | NOT stated in any excerpt. Sec 8: "When enough SCORING ELEMENTS are LAUNCHED into the CELL" | CONFIRMED absent. Repo `ballsToTip = 4` is an ASSUMPTION (hivecycle.js:24-26, 46; README.md:64) | S1; S2 |
| Scoring method | "LAUNCHING into the upward-facing CELL is the only allowed way to earn a HIVE TIP" (Sec 10.5.1) | CONFIRMED | S1 |
| Tip definition | hive moves to the other stable state and the damper on the previously non-contacting side begins contacting the frame (Sec 10.5.1) | CONFIRMED | S1 |
| Tip cause | G417: tip only induced by launching pollen/nectar into the upward cell | CONFIRMED | S1 |
| Deflect from tipped hive | G409: robot may not catch or deflect a scoring element released by a tipped hive until ... (text cut off) | CONFIRMED start of rule only; end UNVERIFIED | S1 |
| Cell opening | approx 20 in wide x 14 in tall (Sec 9.6.2). Repo CAD: pentagon 20.04 x 14.00 in the opening plane (FIELD_CONSTANTS.md:27, 90) | CONFIRMED approx (S1); CAD detail REPO | S1; S2 |
| Cell scoring points | 2 per ball remaining in cell at end (TELEOP) (Table 10-2) | CONFIRMED value, column reading UNVERIFIED | S1 |
| Hive dimensions / pivot / tilt | repo CAD: 30 deg tilt, pivot height 43.95 in, centre-to-centre 25.5 in (FIELD_CONSTANTS.md:24-42, 84-88) | REPO. Pivot 43.95 and cell depth 12.04 tie to "9.6.1 / Fig 9-9" per repo; not seen in S1 | S2 |

## Period lengths
| Period | Length | Status | Source |
|---|---|---|---|
| AUTO | 30 s (Sec 10.4) | CONFIRMED. Repo timing.auto 30 (matchrules.js:12) | S1; S2 |
| Transition | 8 s, "for scoring purposes" (Sec 10.4) | CONFIRMED. Repo 8 (matchrules.js:13) | S1; S2 |
| TELEOP | 2:00 (Sec 10.4) | CONFIRMED. Repo 120 (matchrules.js:14) | S1; S2 |
| Endgame | not a named period in S1. Repo 30 s is labeled "sim convention only" (matchrules.js:15) | UNVERIFIED as a manual period; repo assumption | S2 |

## Points (Table 10-2)
The S1 table column layout was garbled in extraction. Values are the summarizer's reading.

| Item | Points | Phase (as read) | Status | Repo |
|---|---|---|---|---|
| LEAVE | 3 | AUTO | value CONFIRMED, phase as read | matchrules.js:22 |
| PARK | 5 | AUTO and TELEOP | value CONFIRMED (Sec 10.5.4 for "at least partially in LOADING ZONE") | matchrules.js:23-24 |
| HIVE TIP | 20 | AUTO and TELEOP | value CONFIRMED; AUTO counting of pre-TELEOP tips per matchrules.js:19 UNVERIFIED | matchrules.js:19-20 |
| Pollen/nectar remaining in cell | 2 | TELEOP ("likely") | UNVERIFIED phase | matchrules.js:21 |
| Bottom nectar bonus | 5 | TELEOP | value CONFIRMED, phase as read | matchrules.js:25 |
| Pollen/nectar in owned flower | 2 | TELEOP | value CONFIRMED, phase as read | matchrules.js:26 |
| Pollen/nectar in garden | 1 | TELEOP | value CONFIRMED, phase as read | matchrules.js:27 |
| Win / tie | 3 RP / 1 RP | ranking | CONFIRMED values | matchrules.js:28-29 |
| Swarm RP, Pollinator 1 and 2 RP | 1 RP each | ranking | CONFIRMED in table, not used by repo | none |

## Repo items with no manual support found
- `ballsToTip = 4` (hivecycle.js:46): not in manual. Keep as assumption.
- README.md:64 assumptions: ball 1, audience cell 3, far cell 5 points. Table 10-2 gives 2 per cell ball, not 3 or 5. driver.js:279-285 uses ball 1 and cell 2 as "official". Mismatch worth a look (do not rely on README assumption list).
- Endgame 30 s label (matchrules.js:15) is not a manual period.

## Unverified / could not check
- Pollen mass; hold rule tolerance wording (5 momentary).
- Gate and launch zone: absent from fetched text.
- Start positions in coordinates (Fig 11-1), full G409 wording.
- Exact Table 10-2 column layout; garden and flower phase assignments.
- Which manual version the cm-html page is (TU07 vs TU08 vs other).
- A PDF copy was saved by WebFetch, but it could not be rendered (pdftoppm missing), so it was not used.

## Follow-ups for other owners (no edits made)
1. Reconcile TU07 vs TU08 citations across matchrules.js, FIELD_CONSTANTS.md and FieldConstants.java; re-read the current manual version.
2. Fix preload citation: manual says 10.3.4 / G304.G, matchrules.js cites 10.3.1.
3. Confirm points for AUTO/TELEOP phases of Table 10-2 from a clean copy (the extraction was garbled).
4. Reconcile README/driver.js point assumptions with Table 10-2 (ball 1 / cell 2-3-5 mismatch).
5. Decide whether `ballsToTip` needs a note in the Java/UI ("not in manual").
