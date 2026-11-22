# PedroSampler

Samples routes with the real Pedro Pathing 2.1.0 core classes (BezierLine, BezierCurve, Path, Pose, HeadingInterpolator), so the sim shows what the robot drives.

Build: `powershell -ExecutionPolicy Bypass -File field_sim\tools_java\build_sampler.ps1`
(finds `core-2.1.0.jar` in `~/.gradle/caches/modules-2/files-2.1/com.pedropathing/core` and a JDK: `JAVA_HOME`, Android Studio `jbr`, or PATH; output in `out/`).
Env overrides for `pedro_bridge.py`: `PEDRO_CORE_JAR`, `JAVA_HOME`.

Run: `java -cp "out;<core jar>" PedroSampler < route.json` (use `:` on Linux/macOS).

Only the core jar is needed; no Android/FTC classes. `PathBuilder` was not used (it needs a Follower); `Path` is built directly.
Note: `Path.getTFromPathCompletion` is inconsistent in 2.1.0 (BezierCurve expects distance, BezierLine a fraction), so arc-length t is found by bisecting Pedro's own `getPathCompletion(t)`.

## Input (headings in degrees)
```
{"start":{"x":0,"y":0,"h":0}, "step":1.0,
 "segments":[{"type":"line"|"curve",
              "points":[{"x":..,"y":..}, ...],        // full control points; a single point chains from previous end
              "heading":{"mode":"constant"|"linear"|"tangent", "start":deg, "end":deg, "reversed":bool}}]}
```
Defaults: heading start = previous end heading (or start.h), end = start.

## Output
```
{"start":{..}, "total":len,
 "segments":[{"type","mode","length","offset","samples":[{"x","y","h"(deg),"t","s"(arc length in segment)}]}]}
```
On failure: `{"error":"...","trace":"..."}`.

# PedroFollowSim (real Follower, simulated robot)

`PedroFollowSim.java` instantiates Pedro 2.1.0's real `Follower` (PIDFs, Kalman-filtered drive error, centripetal correction,
hold/braking, path-end constraints, callbacks) on a simulated `Localizer` and a simulated mecanum `Drivetrain` (the real
`Mecanum.calculateDrive` wheel math + first-order motor/zero-power dynamics), stepped at `dt` (default 5 ms, output every `stride`=2 ticks).
Build is automatic (`pedro_bridge.build_follow()`), or run `build_follow.ps1`. The classes that read the wall clock (PIDF controllers,
PoseTracker, Timer, TemporalCallback, Follower) are compiled from the official sources jar with `System.nanoTime()` replaced by
`sim/.../SimClock`, so the run is deterministic and faster than real time. With no sources jar it falls back to real time (`info.realtimeClock: true`).

Endpoints (run_sim.py): `POST /api/pedro/follow`, `POST /api/pedro/live/start|step|stop`. JS client: `web/js/pedrofollow.js`.

Request: `{start:{x,y,h(deg)}, segments:[{type,points,heading,callbacks:[{t|ms,name}],waitAfter}], dt, stride, maxTime, settle, holdEnd, maxPower,
robot:{xVelocity,yVelocity,tauX,tauY,tauW,maxAccel,omegaMax}, constants:{mass,forwardZeroPowerAcceleration,lateralZeroPowerAcceleration,
centripetalScaling,translationalPIDF:[p,i,d,f],headingPIDF:[..],drivePIDF:[p,i,d,t,f],usePredictiveBraking,...}, pathConstraints:{tValue,timeoutMs,braking,brakingStart}}`.
Response: `{ticks:[{t,x,y,h(deg),vx,vy,w(deg/s),p:[LF,LR,RF,RR],seg,phase}], events:[{t,kind,name,seg}], finished, endReason, info}`.
Defaults mirror TeamCode `pedroPathing/Constants.java`, which currently uses LIBRARY DEFAULT follower constants, `PathConstraints(0.99,100,1,1)` and default mecanum xVelocity/yVelocity.

Event kinds: pathStart, pathEnd, waitStart, waitEnd, done, timeout, `callback` (reliable: fired by the sim from `getPathCompletion`),
`pedroCallback` (Pedro's own PathChain callback; in 2.1.0 `PathChain.getNextPathCallbacks` mis-indexes the first fetch
(`getLastCallbackIndex()` returns 0), so chain callbacks do not fire as expected on the real robot either).
Robot dynamics (motor time constants, accel limit) are modelled, not measured; walls/collisions are not simulated here.
