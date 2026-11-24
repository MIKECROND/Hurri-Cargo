# Host-side nav control simulator: one-click build + run + report.
#
# What it does:
#   1) extracts the frozen baseline nav sources (git commit below) into %TEMP%\nav_harness\baseline_src
#      and verifies SHA-256, so the "before" behaviour stays reproducible after the working tree moves on;
#   2) compiles the REAL nav_core.c twice:
#        - baseline : frozen sources from commit NAV_BASELINE_COMMIT (no Nav_Update -> Nav_Task driven
#                     through an osDelay() stub + setjmp)
#        - current  : working-tree Applications/nav/utils/nav_core.c (Nav_Update() called every 5ms)
#      linked against the same plant/OPS model in sim_plant.c / sim_ops.c / sim_platform.c;
#   3) runs scenario suites (straight 100cm @30/50/70/100cm/s, short 5cm, diagonal, in-place 90/180deg,
#      PASS retarget, 3-point sequence) x command-path delays {0,10,20,50,100}ms;
#   4) runs runtime-tunable sweeps (current build: `nav par` names, no recompile) and macro sweeps
#      (compile-time NAV_* overrides via a forced-include header);
#   5) runs model-uncertainty matrix (wheel tau / ground slip / OPS rate / control-loop jitter);
#   6) writes CSVs and a Markdown report through report.py.
#
# Outputs (default): %TEMP%\nav_harness\
#   build\                       object files and executables
#   baseline_src\                frozen baseline sources (verified)
#   out\runs_<tag>.csv           one row per (scenario, delay, model variant)
#   out\<tag>_<scenario>_d<ms>.csv   traces for representative scenarios
#   out\nav_sim_report.md        generated Markdown summary
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File Applications\nav\tests\harness\run_sim.ps1
#   ... -SkipBuild              reuse existing executables
#   ... -NoSweep                skip tunable/macro parameter sweeps (fast baseline/delay only)
#   ... -NoUncertainty          skip model-uncertainty matrix
#   ... -OutDir D:\tmp\navsim   change output root
#
# NOTE: ASCII-only on purpose (Windows PowerShell 5.1 reads BOM-less files as ANSI).
# Chinese documentation lives in ../README.md and the generated report.

[CmdletBinding()]
param(
    [string]$Repo = '',
    [string]$Gcc = 'D:\devtool\MinGW\bin\gcc.exe',
    [string]$OutDir = (Join-Path $env:TEMP 'nav_harness'),
    [string]$Python = 'py',
    [switch]$SkipBuild,
    [switch]$NoSweep,
    [switch]$NoUncertainty,
    [switch]$NoTrace,
    [switch]$NavLog
)

# Native tools (gcc/git) write warnings to stderr; keep 'Continue' so a warning cannot abort the
# run -- every external step below is checked explicitly through $LASTEXITCODE / throw.
$ErrorActionPreference = 'Continue'

# Frozen baseline: git commit of the nav sources that "before" runs are built from.
$BaselineCommit = 'b6f3ddb40c25e5e7085af1388425e2570a07e12a'
$BaselineFiles = @(
    @{ Path = 'Applications/nav/utils/nav_core.c'; Sha = '4cbee425052b01af76b633da8189173a788ceeeeaea71bc5751fc1bcd8d05ae1' },
    @{ Path = 'Applications/nav/utils/nav_core.h'; Sha = '8899ff92c4e299fc6d75de0f3338363473975c4ca13fa4ea403fff678fb14538' },
    @{ Path = 'Applications/nav/nav_config.h';  Sha = 'a18f4789bfac6490959e448059c3b158ff39d9957e1df8185adcaa8cddff8bc0' }
)

if (-not (Test-Path $Gcc)) {
    throw "gcc not found: $Gcc (use -Gcc to override)"
}
if (-not $Repo) {
    $Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
}

$HarnessDir = $PSScriptRoot
$StubsDir = Join-Path $HarnessDir 'stubs'
$BuildDir = Join-Path $OutDir 'build'
$BaselineSrc = Join-Path $OutDir 'baseline_src'
$ResultDir = Join-Path $OutDir 'out'
$ReportPath = Join-Path $ResultDir 'nav_sim_report.md'

foreach ($d in @($OutDir, $BuildDir, $BaselineSrc, $ResultDir)) {
    New-Item -ItemType Directory -Force -Path $d | Out-Null
}

function Write-Stage([string]$text) {
    Write-Host ''
    Write-Host "=== $text ==="
}

# ---------------------------------------------------------------- baseline snapshot
Write-Stage '[1/7] Extract frozen baseline nav sources'
foreach ($f in $BaselineFiles) {
    $name = Split-Path $f.Path -Leaf
    $dest = Join-Path $BaselineSrc $name
    Push-Location $BaselineSrc
    try {
        $spec = "$BaselineCommit`:$($f.Path)"
        $gitCmd = 'git -C "' + $Repo + '" show "' + $spec + '" > ' + $name
        cmd /c $gitCmd
        if ($LASTEXITCODE -ne 0) { throw "git show failed for $($f.Path) (exit $LASTEXITCODE)" }
    } finally {
        Pop-Location
    }
    $actual = (Get-FileHash $dest -Algorithm SHA256).Hash.ToLower()
    if ($actual -ne $f.Sha) {
        throw "baseline hash mismatch for $($f.Path): expected $($f.Sha), got $actual"
    }
    Write-Host ("  {0,-45} {1}" -f $name, $actual.Substring(0, 16))
}

# ---------------------------------------------------------------- include paths
$CommonInclude = @(
    '-I', $StubsDir,
    '-I', $HarnessDir,
    '-I', (Join-Path $Repo 'Applications\nav\utils'),
    '-I', (Join-Path $Repo 'Applications\nav\localization'),
    '-I', (Join-Path $Repo 'Applications\nav'),
    '-I', (Join-Path $Repo 'Applications\chassis'),
    '-I', (Join-Path $Repo 'Applications\chassis\motion'),
    '-I', (Join-Path $Repo 'Applications\chassis\kinematics'),
    '-I', (Join-Path $Repo 'Drivers\User\RTOS'),
    '-I', (Join-Path $Repo 'Drivers\User\OPS')
)
$CompileFlags = @('-std=c99', '-O2', '-Wall', '-Wextra', '-msse2', '-mfpmath=sse', '-fno-strict-aliasing')
$LinkFlags = @('-lm')

$CommonSources = @(
    (Join-Path $HarnessDir 'sim_platform.c'),
    (Join-Path $HarnessDir 'sim_plant.c'),
    (Join-Path $HarnessDir 'sim_ops.c'),
    (Join-Path $Repo 'Applications\nav\utils\nav_math.c'),
    (Join-Path $Repo 'Applications\chassis\kinematics\kinematics.c')
)

$gccExe = $Gcc

function Compile-Object([string]$src, [string]$obj, [string[]]$extraFlags, [string[]]$extraInclude) {
    $argv = @($CompileFlags) + $extraFlags + @($CommonInclude) + $extraInclude + @('-c', $src, '-o', $obj)
    $logf = Join-Path $BuildDir 'gcc_stderr.log'
    & $gccExe @argv 2> $logf
    if ($LASTEXITCODE -ne 0) {
        Get-Content $logf | Select-Object -First 40 | Write-Host
        throw "gcc failed compiling $src"
    }
}

function Link-Exe([string]$exe, [string[]]$objs) {
    $argv = @($objs) + @('-o', $exe) + $LinkFlags
    $logf = Join-Path $BuildDir 'gcc_stderr.log'
    & $gccExe @argv 2> $logf
    if ($LASTEXITCODE -ne 0) {
        Get-Content $logf | Select-Object -First 40 | Write-Host
        throw "gcc link failed: $exe"
    }
}

# ---------------------------------------------------------------- common objects
if (-not $SkipBuild) {
    Write-Stage '[2/7] Compile common simulator objects'
    foreach ($src in $CommonSources) {
        $name = [System.IO.Path]::GetFileNameWithoutExtension($src)
        $obj = Join-Path $BuildDir "common_$name.o"
        Compile-Object $src $obj @() @()
        Write-Host "  $name.o"
    }
}

$CommonObjs = @()
foreach ($src in $CommonSources) {
    $name = [System.IO.Path]::GetFileNameWithoutExtension($src)
    $CommonObjs += (Join-Path $BuildDir "common_$name.o")
}

# ---------------------------------------------------------------- baseline build
$BaselineExe = Join-Path $BuildDir 'navsim_baseline.exe'
$CurrentExe = Join-Path $BuildDir 'navsim_current.exe'
$BaselineInc = @('-I', $BaselineSrc)

if (-not $SkipBuild) {
    Write-Stage '[3/7] Compile baseline (frozen) and current (working tree) nav cores'
    $baseNavObj = Join-Path $BuildDir 'baseline_nav_core.o'
    $baseRunnerObj = Join-Path $BuildDir 'baseline_sim_runner.o'
    Compile-Object (Join-Path $BaselineSrc 'nav_core.c') $baseNavObj @() $BaselineInc
    Compile-Object (Join-Path $HarnessDir 'sim_runner.c') $baseRunnerObj @() $BaselineInc
    Link-Exe $BaselineExe (@($CommonObjs) + @($baseNavObj, $baseRunnerObj))
    Write-Host "  $BaselineExe"

    $curNavObj = Join-Path $BuildDir 'current_nav_core.o'
    $curRunnerObj = Join-Path $BuildDir 'current_sim_runner.o'
    $hasUpdate = Select-String -Path (Join-Path $Repo 'Applications\nav\utils\nav_core.h') -Pattern 'void\s+Nav_Update\s*\(\s*void\s*\)' -Quiet
    if ($hasUpdate) {
        Write-Host '  Nav_Update() detected -> direct 5ms stepping'
        $curExtra = @('-DSIM_HAVE_NAV_UPDATE')
    } else {
        Write-Host '  Nav_Update() not found -> legacy Nav_Task + osDelay stub stepping'
        $curExtra = @()
    }
    Compile-Object (Join-Path $Repo 'Applications\nav\utils\nav_core.c') $curNavObj $curExtra @()
    Compile-Object (Join-Path $HarnessDir 'sim_runner.c') $curRunnerObj $curExtra @()
    Link-Exe $CurrentExe (@($CommonObjs) + @($curNavObj, $curRunnerObj))
    Write-Host "  $CurrentExe"
} else {
    Write-Stage '[3/7] skipped build (-SkipBuild)'
    if (-not (Test-Path $BaselineExe) -or -not (Test-Path $CurrentExe)) {
        throw '-SkipBuild given but executables are missing'
    }
}

function Remove-OldRuns {
    Get-ChildItem $ResultDir -Filter 'runs_*.csv' -ErrorAction SilentlyContinue | Remove-Item -Force
    Get-ChildItem $ResultDir -Filter '*.csv' -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notlike 'runs_*' } | Remove-Item -Force
}

Remove-OldRuns

# ---------------------------------------------------------------- run suites
function Invoke-Runner([string]$exe, [string[]]$runnerArgs, [string]$what) {
    Write-Host "  run: $what"
    & $exe @runnerArgs
    if ($LASTEXITCODE -ne 0) { throw "runner failed: $what (exit $LASTEXITCODE)" }
}

Write-Stage '[4/7] Baseline vs current: scenario x delay suites'
$commonArgs = @('--out-dir', $ResultDir)
$traceArgs = @()
if (-not $NoTrace) { $traceArgs = @('--trace') }
$verboseArgs = @()
if ($NavLog) { $verboseArgs = @('--verbose') }
Invoke-Runner $BaselineExe ($commonArgs + @('--tag', 'baseline', '--suite', 'baseline')) 'baseline (frozen) delay suite'
Invoke-Runner $CurrentExe ($commonArgs + @('--tag', 'current', '--suite', 'baseline') + $traceArgs) 'current  delay suite'
Invoke-Runner $BaselineExe ($commonArgs + @('--tag', 'baseline_quick', '--suite', 'quick')) 'baseline quick suite'
Invoke-Runner $CurrentExe ($commonArgs + @('--tag', 'current_quick', '--suite', 'quick')) 'current  quick suite'

if (-not $NoUncertainty) {
    Write-Stage '[5/7] Model uncertainty matrix (tau / slip / OPS rate / loop jitter)'
    Invoke-Runner $CurrentExe ($commonArgs + @('--tag', 'current_unc', '--suite', 'uncertainty')) 'current uncertainty suite'
}

# ---------------------------------------------------------------- tunable sweeps
# Runtime tunables (Nav_SetTunable names). Fast: no recompile.
$TunableSweeps = @(
    @{ Name = 'stop_settle_ms';      Values = @(100, 150, 200, 300) },
    @{ Name = 'yaw_kp';              Values = @(1.2, 1.8, 2.4) },
    @{ Name = 'yaw_kd';              Values = @(0.3, 0.6) },
    @{ Name = 'yaw_max_speed';       Values = @(60, 100) },
    @{ Name = 'near_gain';           Values = @(1.5, 3.1, 5.0) },
    @{ Name = 'traj_speed_cap';      Values = @(30, 50, 80) },
    @{ Name = 'startup_speed';       Values = @(1.0, 1.5) },
    @{ Name = 'xy_kp';               Values = @(0.8, 1.2) },
    @{ Name = 'align_dist';          Values = @(3, 5) }
)

if (-not $NoSweep) {
    Write-Stage '[6/7] Runtime tunable sweeps (current build, no recompile)'
    foreach ($sweep in $TunableSweeps) {
        foreach ($v in $sweep.Values) {
            $tag = 'sw_' + $sweep.Name + '_' + $v
            $tuneArg = $sweep.Name + '=' + $v
            Invoke-Runner $CurrentExe ($commonArgs + @('--tag', $tag, '--suite', 'quick', '--tune', $tuneArg)) `
                "tune $tuneArg"
        }
    }

    # Compile-time macro sweeps for keys that are NOT runtime tunables.
    $MacroSweeps = @(
        @{ Macro = 'NAV_ARRIVE_CHECK_COUNT';           Values = @('1', '2', '3') },
        @{ Macro = 'NAV_YAW_ZERO_CROSS_LOCK_MS';       Values = @('0', '8', '20') }
    )
    Write-Host '  compile-time macro sweeps (nav_core.c recompiled per value):'
    foreach ($sweep in $MacroSweeps) {
        foreach ($v in $sweep.Values) {
            $name = $sweep.Macro + '_' + ($v -replace 'f$', '' -replace '\.', 'p')
            $tag = 'mx_' + $name
            $ovr = Join-Path $BuildDir ('override_' + $name + '.h')
            $lines = @(
                '/* generated by run_sim.ps1: forced-include nav_config.h override for a macro sweep */',
                '#include "nav_config.h"',
                ('#undef {0}' -f $sweep.Macro),
                ('#define {0} {1}' -f $sweep.Macro, $v)
            )
            Set-Content -Path $ovr -Value $lines -Encoding Ascii
            $obj = Join-Path $BuildDir ('mx_' + $name + '_nav_core.o')
            $extra = @('-include', $ovr)
            Compile-Object (Join-Path $Repo 'Applications\nav\utils\nav_core.c') $obj $extra @()
            $runnerObj = Join-Path $BuildDir 'current_sim_runner.o'
            $exe = Join-Path $BuildDir ('navsim_mx_' + $name + '.exe')
            Link-Exe $exe (@($CommonObjs) + @($obj, $runnerObj))
            Invoke-Runner $exe ($commonArgs + @('--tag', $tag, '--suite', 'quick')) "macro $($sweep.Macro)=$v"
        }
    }
}

# ---------------------------------------------------------------- report
Write-Stage '[7/7] Generate Markdown report'
$reportArgs = @((Join-Path $HarnessDir 'report.py'), '--out-dir', $ResultDir, '--out', $ReportPath)
& $Python @reportArgs
if ($LASTEXITCODE -ne 0) { throw "report.py failed (exit $LASTEXITCODE)" }

Write-Host ''
Write-Host 'Results:'
Write-Host "  CSVs   : $ResultDir"
Write-Host "  Report : $ReportPath"
Get-ChildItem $ResultDir -Filter 'runs_*.csv' | ForEach-Object {
    $rows = (Get-Content $_.FullName | Measure-Object -Line).Lines - 1
    Write-Host ("  {0,-40} {1,6} rows" -f $_.Name, $rows)
}
exit 0
