# ============================================================================
#  Planner independent review: build & run (writes only under %TEMP%)
#
#  Usage (from repo root):
#    powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\review\run_review.ps1
#
#  Requires: gcc = D:\devtool\MinGW\bin\gcc.exe
#            armcc = D:\devtool\keil\ARM\ARMCC\bin\armcc.exe (via .planner_verify/compile_planner.ps1)
#  Output:   $env:TEMP\planner_review\*.exe and *.log
# ============================================================================
$ErrorActionPreference = "Continue"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
Set-Location $root

$gcc = "D:\devtool\MinGW\bin\gcc.exe"
$out = Join-Path $env:TEMP "planner_review"
New-Item -ItemType Directory -Force $out | Out-Null

$hostFlags = @("-std=c99", "-O2", "-Wall", "-Wextra", "-DPLANNER_HOST_TEST", "-msse2", "-mfpmath=sse",
    "-IApplications/Planner", "-IApplications/Planner/utils")
$implSrc = @(
    "Applications/Planner/utils/Cstar_map_core.c",
    "Applications/Planner/utils/planner_algo.c",
    "Applications/Planner/utils/priority_queue.c"
)
# diag_stages.c includes planner_algo.c directly (to reach static functions), so do NOT link it twice
$implSrcNoAlgo = @(
    "Applications/Planner/utils/Cstar_map_core.c",
    "Applications/Planner/utils/priority_queue.c"
)

function Build-And-Run($name, $extraSrc, $impl) {
    $exe = Join-Path $out "$name.exe"
    $log = Join-Path $out "$name.log"
    Write-Host "=== build $name ==="
    $compileOut = & $gcc @hostFlags $extraSrc $impl -o $exe -lm 2>&1 | Out-String
    if (-not (Test-Path $exe)) {
        Write-Host "[FAIL] gcc build $name"
        $compileOut -split "`r?`n" | Select-Object -First 25 | ForEach-Object { Write-Host "  $_" }
        return 1
    }
    Write-Host "[OK] $exe"
    & $exe *> $log
    Write-Host "[RESULT] $name exit=$LASTEXITCODE log=$log"
    return 0
}

# ---- 1. host-side independent geometry / fuzz tests ----
$fail = 0
$fail += Build-And-Run "fuzz_check" @("Applications/Planner/tests/review/fuzz_check.c") $implSrc
$fail += Build-And-Run "diag_stages" @("Applications/Planner/tests/review/diag_stages.c") $implSrcNoAlgo
$fail += Build-And-Run "port_distance_repro" @("Applications/Planner/tests/review/port_distance_repro.c") $implSrc

# run fuzz_check module by module for easier triage
foreach ($m in @("map", "targeted", "fuzz", "dynamic", "timing")) {
    $log = Join-Path $out "fuzz_$m.log"
    & (Join-Path $out "fuzz_check.exe") $m *> $log
    Write-Host ("[MODULE] {0} -> exit={1}, log={2}" -f $m, $LASTEXITCODE, $log)
}

# ---- 2. firmware compile check (armcc AC5, 5 Planner .c, 0-warning target) ----
Write-Host "=== armcc compile check (5 Planner .c) ==="
$ps = ".planner_verify\compile_planner.ps1"
if (Test-Path $ps) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $ps -OutDir (Join-Path $out "obj") 2>&1 |
        ForEach-Object { Write-Host $_ }
} else {
    Write-Host "[WARN] .planner_verify/compile_planner.ps1 not found, skip armcc check"
}

Write-Host ""
Write-Host "Review log dir: $out"
exit $fail
