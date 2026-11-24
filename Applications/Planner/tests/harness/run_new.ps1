# One-click evaluation of the v2 REFACTORED planner (square body 30x30, working-tree sources).
#
# Steps:
#   1) Compile new_runner + edge_runner + profile_runner against Applications/Planner/utils/*.c
#      (REQUIRED: -DPLANNER_HOST_TEST, -msse2 -mfpmath=sse to match Cortex-M4 single precision, -lm)
#   2) Run new_runner(v2): 40 directed point pairs with yaw (2 zones x 5 points), writes
#      newv2_zone*.csv / newv2_paths_zone*.csv / acceptance_v2.csv (independent exact SAT)
#   3) Run edge_runner(v2): boundary/regression cases (channels/yaw/rotation/dynamic), writes
#      edge_results_v2.csv (PASS/FAIL)
#   4) Run profile_runner: waypoint execution-profile pure-function unit tests, writes
#      profile_results.csv (PASS/FAIL)
#   5) summarize.py --v2 -> new_report_v2.md
#   6) compare.py --v2 -> baseline_report_v2.md (old HEAD paths under square truth) +
#      after_report_v2.md (old baseline vs v2, acceptance + edge tables)
#
# All outputs stay under %TEMP%\planner_harness_v2; the repository is not modified.
# Exit code is non-zero if new_runner, edge_runner or profile_runner checks fail.
#
# NOTE: This file is intentionally ASCII-only: Windows PowerShell 5.1 reads BOM-less files as
# ANSI, so Chinese text here would corrupt parsing. Chinese docs live in ../README.md.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_new.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File ...\run_new.ps1 -SkipBuild
#
# Parameters:
#   -Repo     T-Car repo root (default: 4 levels above this script)
#   -Gcc      gcc path (default: D:\devtool\MinGW\bin\gcc.exe)
#   -OutDir   output root (default: %TEMP%\planner_harness_v2)
#   -Python   python launcher (default: auto-detect py/python/python3; explicit value wins)
#   -SkipBuild reuse existing executables

[CmdletBinding()]
param(
    [string]$Repo = '',
    [string]$Gcc = 'D:\devtool\MinGW\bin\gcc.exe',
    [string]$OutDir = (Join-Path $env:TEMP 'planner_harness_v2'),
    [string]$Python = '',
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Gcc)) {
    throw "gcc not found: $Gcc (use -Gcc to override)"
}

# Resolve a usable Python launcher. An explicit -Python value always wins.
if (-not $Python) {
    foreach ($candidate in @('py', 'python', 'python3')) {
        $found = Get-Command $candidate -ErrorAction SilentlyContinue
        if ($found) {
            $Python = $found.Source
            break
        }
    }
    if (-not $Python) {
        throw 'python launcher not found (tried py, python, python3; use -Python to override)'
    }
}
$env:PYTHONIOENCODING = 'utf-8'

if (-not $Repo) {
    $Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
}
$Repo = (Resolve-Path -LiteralPath $Repo).Path
$HarnessDir = (Resolve-Path -LiteralPath $PSScriptRoot).Path
$OutDir = [System.IO.Path]::GetFullPath($OutDir)

$PlannerDir = Join-Path $Repo 'Applications\Planner'
$UtilsDir = Join-Path $PlannerDir 'utils'
$TestsDir = Join-Path $PlannerDir 'tests'
$BaselineResults = Join-Path $TestsDir 'results'

$RequiredFiles = @(
    (Join-Path $HarnessDir 'summarize.py'),
    (Join-Path $HarnessDir 'compare.py'),
    (Join-Path $HarnessDir 'new_runner.c'),
    (Join-Path $HarnessDir 'edge_runner.c'),
    (Join-Path $HarnessDir 'profile_runner.c'),
    (Join-Path $HarnessDir 'reference_metrics.c'),
    (Join-Path $UtilsDir 'Cstar_map_core.c'),
    (Join-Path $UtilsDir 'planner_algo.c'),
    (Join-Path $UtilsDir 'priority_queue.c')
)
foreach ($file in $RequiredFiles) {
    if (-not (Test-Path -LiteralPath $file)) {
        throw "required file not found: $file"
    }
}

$BuildDir = Join-Path $OutDir 'new_build'
$ResultDir = Join-Path $OutDir 'new_out'
$NewReport = Join-Path $OutDir 'new_report_v2.md'
$BaselineReport = Join-Path $OutDir 'baseline_report_v2.md'
$AfterReport = Join-Path $OutDir 'after_report_v2.md'

foreach ($dir in @($OutDir, $BuildDir, $ResultDir)) {
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
}

if (-not $SkipBuild) {
    Write-Host '[1/6] Compiling new_runner + edge_runner + profile_runner (v2 square body) ...'
    $Common = @(
        '-std=c99', '-O2', '-Wall', '-Wextra',
        '-DPLANNER_HOST_TEST', '-msse2', '-mfpmath=sse',
        '-I', $PlannerDir,
        '-I', $UtilsDir,
        '-I', $HarnessDir,
        (Join-Path $UtilsDir 'Cstar_map_core.c'),
        (Join-Path $UtilsDir 'planner_algo.c'),
        (Join-Path $UtilsDir 'priority_queue.c'),
        (Join-Path $HarnessDir 'reference_metrics.c')
    )
    & $Gcc @Common (Join-Path $HarnessDir 'new_runner.c') -o (Join-Path $BuildDir 'new_runner.exe') -lm
    if ($LASTEXITCODE -ne 0) { throw "gcc failed: new_runner (exit $LASTEXITCODE)" }
    & $Gcc @Common (Join-Path $HarnessDir 'edge_runner.c') -o (Join-Path $BuildDir 'edge_runner.exe') -lm
    if ($LASTEXITCODE -ne 0) { throw "gcc failed: edge_runner (exit $LASTEXITCODE)" }
    & $Gcc @Common (Join-Path $HarnessDir 'profile_runner.c') -o (Join-Path $BuildDir 'profile_runner.exe') -lm
    if ($LASTEXITCODE -ne 0) { throw "gcc failed: profile_runner (exit $LASTEXITCODE)" }
} else {
    Write-Host '[1/6] skipped build (-SkipBuild)'
}

Write-Host '[2/6] Running new_runner(v2) (40 directed pairs with yaw, exact SAT) ...'
& (Join-Path $BuildDir 'new_runner.exe') --out-dir $ResultDir
$RunnerRc = $LASTEXITCODE

Write-Host ''
Write-Host '[3/6] Running edge_runner(v2) (boundary/regression cases) ...'
& (Join-Path $BuildDir 'edge_runner.exe') --out-dir $ResultDir
$EdgeRc = $LASTEXITCODE

Write-Host ''
Write-Host '[4/6] Running profile_runner (waypoint execution-profile unit tests) ...'
& (Join-Path $BuildDir 'profile_runner.exe') --out-dir $ResultDir
$ProfileRc = $LASTEXITCODE

Write-Host ''
Write-Host '[5/6] Generating v2 summary (summarize.py --v2) ...'
& $Python (Join-Path $HarnessDir 'summarize.py') --v2 --new-dir $ResultDir --out $NewReport
if ($LASTEXITCODE -ne 0) { throw 'summarize.py --v2 failed' }

Write-Host ''
Write-Host '[6/6] Generating v2 reports (compare.py --v2: baseline + after) ...'
& $Python (Join-Path $HarnessDir 'compare.py') --v2 --baseline-dir $BaselineResults --new-dir $ResultDir `
    --out-baseline $BaselineReport --out-after $AfterReport
if ($LASTEXITCODE -ne 0) { throw 'compare.py --v2 failed' }

Write-Host ''
Write-Host 'Results:'
Write-Host "  CSV      : $ResultDir"
Write-Host "  new      : $NewReport"
Write-Host "  baseline : $BaselineReport"
Write-Host "  after    : $AfterReport"

if ($RunnerRc -ne 0 -or $EdgeRc -ne 0 -or $ProfileRc -ne 0) {
    Write-Host "[FAIL] new_runner exit=$RunnerRc, edge_runner exit=$EdgeRc, profile_runner exit=$ProfileRc"
    exit 1
}
Write-Host '[OK] all v2 acceptance checks passed'
exit 0
