# Vendored: Pedro Pathing Visualizer

- Source: https://github.com/Pedro-Pathing/Visualizer
- Commit: 97104e027af00d49b8bd61e29bcb74a9d7e62b90
- License: Apache-2.0 (LICENSE copied to `web/visualizer/LICENSE.txt`, NOTICE to `web/visualizer/NOTICE.txt`)
- Built output (served at `/visualizer/`): `field_sim/web/visualizer/` (about 1.3 MB). Source clone: `third_party/visualizer-src/` (has its own .git; never `git add` it, never run tools with it as cwd).
- Stack: Svelte 5 + Vite 8 + TypeScript + Tailwind 4. Default field is already "BIOBUZZ Field (2026-2027)" (biobuzz-dark/light.webp ship in the repo). FIELD_SIZE = 141.5 in (`src/config/defaults.ts`; 144 in tiles minus 1.25 in walls), TILE_SIZE = 24. UI has a custom field-image upload (settings `fieldMap: "custom"`, `customFieldImage` base64).

## Rebuild (from any cwd that is NOT visualizer-src)
```
cd %TEMP%
git clone https://github.com/Pedro-Pathing/Visualizer field_sim/third_party/visualizer-src   # then: git -C ...visualizer-src checkout 97104e0
git -C field_sim/third_party/visualizer-src apply ../patches/0001-relative-asset-paths.patch
npx --yes pnpm@12.15.0 --dir field_sim/third_party/visualizer-src install --frozen-lockfile --ignore-scripts
npx --yes pnpm@12.15.0 --dir field_sim/third_party/visualizer-src run build
# copy dist -> field_sim/web/visualizer, delete sw.js, copy LICENSE -> LICENSE.txt, NOTICE -> NOTICE.txt
```
(pnpm 12.15.0 is the `packageManager` pin; obtained via npx, no global install.) `vite.config.ts` already has `base: "./"`, so no config change was needed.

## Patches (`patches/`)
- `0001-relative-asset-paths.patch`: the app hard-coded absolute URLs at runtime (`/fields/...`, `/robot.png`, `/loading.svg`, `/logo-duck.svg`, `/gif.worker.js`, `/JefferyThePotato.png`, `/MecanumDuck.png`) which break under `/visualizer/`; changed to `./...`. Also disables service-worker registration (src/main.ts) because sw.js precaches absolute `/` paths and would cache the sim root. sw.js removed from the output. (The patch was made with `git diff` in the clone; the working tree in visualizer-src currently has it applied.)

## Dependency audit
`pnpm install --frozen-lockfile --ignore-scripts` ran from the locked versions; lifecycle scripts not run. Direct deps: @fontsource-variable/instrument-sans, @lucide/svelte, @types/lodash, gif.js, lodash, modern-screenshot, shiki. Dev: svelte, vite, vitest, eslint, prettier, tailwind, playwright, happy-dom, typescript and their plugins. All well-known packages; nothing suspicious found by name. Not done: no `pnpm audit`, no source review of transitive deps. Note `pnpm-workspace.yaml` allows builds of esbuild and workerd (skipped by --ignore-scripts; the build still worked). The optimizer feature does a network `fetch` to OPTIMIZER_BASE_URL (src/lib/optimizer) and is experimental; unused offline.

## Known limitations
- Exported Java/Kotlin (src/lib/codegen) targets a NEWER Pedro API (`com.pedropathing.api.Paths`, `PoseFactory`, `Interpolator`) than core 2.1.0 (PathBuilder/HeadingInterpolator). Use the sim's own exporter, not the Visualizer's.
- Not browser-tested (no Chrome opened); verified only by HTTP 200 on assets.
- Saved settings live in browser localStorage per origin.

## Project file schema (src/io/project.ts, PROJECT_VERSION "2.0.0")
Extension `.pp.json` (legacy `.pp` still opens). Top-level keys: `version`, `timestamp`, `startPoint`, `lines`, `shapes`, `sequence`, `fieldPoints`, `robot`, `activePaths`.
- `startPoint`: {x, y, headingDeg, locked?, name?}
- `lines[]` (Path): common {id, color, name?, locked?, waitBefore?, waitAfter?, waitBeforeMs?, waitAfterMs?, waitBeforeName?, waitAfterName?} plus either
  - Atomic: {kind:"atomic", endPoint:{x,y,name?,locked?}, controlPoints:[{x,y,locked?}], throughPoints?:[{x,y,locked?}], heading}
  - Compound: {kind:"compound", segments:[Path], heading?}
  - waitBefore/After: {name?, durationMs, position?:"before"|"after"}
- `heading`: {type:"linear",startDeg,endDeg} | {type:"constant",degrees} | {type:"tangential",reverse} | {type:"piecewise",piecewiseHeading:{segments:[{startProgress,endProgress,interpolationType:"linear"|"constant"|"tangential"|"facing-point",reversed?,continueFromPrevious?,parameters?:{startDeg?,endDeg?,degrees?,point?:{x,y}}}]}}
- `sequence[]`: {kind:"path", lineId} | {kind:"wait", id, name, durationMs, locked?}
- `fieldPoints[]`: {x, y, locked?, color?, radius?, opacity?}
- `shapes[]`: obstacle polygons {id, name?, vertices:[{x,y,locked?}], color, fillColor}
- `robot`: {forwardVelocity, strafeVelocity, aVelocity, kFriction, rLength, rWidth, safetyMargin, maxAcceleration, maxDeceleration?}
- `activePaths`: string[] of other saved file names
Units: inches, degrees, origin bottom-left of the 141.5 in floor, heading CCW from +x.
