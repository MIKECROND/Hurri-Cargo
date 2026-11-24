# One-click reproduction of the pre-refactor (HEAD 6413fac) planner baseline.
#
# Steps:
#   1) Export HEAD planner sources from git into %TEMP%\planner_harness\baseline_src (SHA-256 checked).
#   2) Compile baseline_runner with MinGW gcc. -msse2 -mfpmath=sse is REQUIRED to match the
#      single-precision IEEE semantics of the Cortex-M4 FPU; the x87 80-bit default would
#      change (int) map-cell indices and produce spurious A* failures.
#   3) Run 40 directed point pairs (2 zones x 5 points), write CSV files to %TEMP%\planner_harness\out.
#   4) Generate markdown report %TEMP%\planner_harness\baseline_report.md via summarize.py.
#
# All outputs stay under %TEMP%; the repository is not modified.
#
# NOTE: This file is intentionally ASCII-only: Windows PowerShell 5.1 reads BOM-less files as
# ANSI, so Chinese text here would corrupt parsing. Chinese documentation lives in ../README.md.
#
# Usage (from any directory):
#   powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_baseline.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File ...\run_baseline.ps1 -SkipSnapshot
#
# Parameters:
#   -Repo          T-Car repo root (default: 4 levels above this script)
#   -Gcc           gcc path (default: D:\devtool\MinGW\bin\gcc.exe)
#   -OutDir        output root (default: %TEMP%\planner_harness)
#   -Python        python launcher (default: py)
#   -SkipSnapshot  reuse an existing source snapshot

[CmdletBinding()]
param(
    [string]$Repo = '',
    [string]$Gcc = 'D:\devtool\MinGW\bin\gcc.exe',
    [string]$OutDir = (Join-Path $env:TEMP 'planner_harness'),
    [string]$Python = 'py',
    [switch]$SkipSnapshot
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Gcc)) {
    throw "gcc not found: $Gcc (use -Gcc to override)"
}

if (-not $Repo) {
    $Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
}

$HarnessDir = $PSScriptRoot
$SnapDir = Join-Path $OutDir 'baseline_src'
$BuildDir = Join-Path $OutDir 'build'
$ResultDir = Join-Path $OutDir 'out'
$ReportPath = Join-Path $OutDir 'baseline_report.md'

foreach ($dir in @($OutDir, $BuildDir, $ResultDir)) {
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
}

Write-Host '[1/4] Exporting pre-refactor source snapshot (git 6413fac, SHA-256 verified)...'
if (-not $SkipSnapshot) {
    & $Python (Join-Path $HarnessDir 'baseline\snapshot_baseline.py') --repo $Repo --dest $SnapDir
    if ($LASTEXITCODE -ne 0) { throw 'snapshot export failed' }
} else {
    Write-Host '      skipped (-SkipSnapshot), reusing existing snapshot'
}

Write-Host '[2/4] Compiling baseline_runner (-msse2 -mfpmath=sse)...'
$ExePath = Join-Path $BuildDir 'baseline_runner.exe'
$GccArgs = @(
    '-std=c99', '-O2', '-Wall', '-Wextra',
    '-msse2', '-mfpmath=sse',
    '-I', $SnapDir,
    '-I', $HarnessDir,
    (Join-Path $SnapDir 'planner_algo.c'),
    (Join-Path $SnapDir 'Cstar_map_core.c'),
    (Join-Path $SnapDir 'priority_queue.c'),
    (Join-Path $HarnessDir 'reference_metrics.c'),
    (Join-Path $HarnessDir 'baseline\baseline_runner.c'),
    '-o', $ExePath
)
& $Gcc @GccArgs
if ($LASTEXITCODE -ne 0) { throw "gcc failed (exit $LASTEXITCODE)" }

Write-Host '[3/4] Running 40 directed point pairs, writing CSV...'
& $ExePath --out-dir $ResultDir
if ($LASTEXITCODE -ne 0) { throw "baseline_runner failed (exit $LASTEXITCODE)" }

Write-Host '[4/4] Generating markdown report...'
& $Python (Join-Path $HarnessDir 'summarize.py') --results-dir $ResultDir --out $ReportPath
if ($LASTEXITCODE -ne 0) { throw 'report generation failed' }

Write-Host ''
Write-Host '[OK] baseline evaluation finished'
Write-Host "     CSV    : $ResultDir"
Write-Host "     report : $ReportPath"
