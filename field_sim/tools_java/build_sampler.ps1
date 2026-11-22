# Builds PedroSampler.java against the real Pedro core jar. Prints the javac/jar it used.
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$jar = Get-ChildItem "$env:USERPROFILE\.gradle\caches\modules-2\files-2.1\com.pedropathing\core" -Recurse -Filter 'core-*.jar' -ErrorAction SilentlyContinue |
  Where-Object { $_.Name -notmatch 'sources|javadoc' } | Sort-Object FullName -Descending | Select-Object -First 1
if (-not $jar) { throw 'Pedro core jar not found in Gradle cache (build the Android project once).' }
$javac = $null
$cands = @()
if ($env:JAVA_HOME) { $cands += "$env:JAVA_HOME\bin\javac.exe" }
$cands += "C:\Program Files\Android\Android Studio\jbr\bin\javac.exe", "$env:LOCALAPPDATA\Programs\Android Studio\jbr\bin\javac.exe"
foreach ($c in $cands) { if (Test-Path $c) { $javac = $c; break } }
if (-not $javac) { $g = Get-Command javac -ErrorAction SilentlyContinue; if ($g) { $javac = $g.Source } }
if (-not $javac) { throw 'javac not found (set JAVA_HOME).' }
$out = Join-Path $here 'out'
New-Item -ItemType Directory -Force $out | Out-Null
Write-Host "javac: $javac"; Write-Host "jar:   $($jar.FullName)"
& $javac -encoding UTF-8 -cp $jar.FullName -d $out (Join-Path $here 'PedroSampler.java')
if ($LASTEXITCODE -ne 0) { throw 'javac failed' }
Write-Host "built -> $out"
