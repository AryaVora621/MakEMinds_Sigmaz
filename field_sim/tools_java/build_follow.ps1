# Builds PedroFollowSim (+ clock-patched Pedro classes) by calling the same code the server uses.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$py = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) { $py = 'python' }
& $py -I -c "import sys; sys.path.insert(0, r'$root'); import pedro_bridge as b; e = b.build_follow(force=True); print(e or 'built tools_java/out/follow (realtime clock: %s)' % b.follow_realtime_clock()); sys.exit(1 if e else 0)"
if ($LASTEXITCODE -ne 0) { throw 'build failed' }
