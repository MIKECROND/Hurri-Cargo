<#
.SYNOPSIS
    批量格式化 T-Car 自研代码（.c / .h）。

.DESCRIPTION
    使用仓库根目录的 .clang-format 规则，对自研目录执行 clang-format -i。
    自动跳过第三方 / 厂商生成目录（其下已放置 DisableFormat 的 .clang-format）。

.PARAMETER DryRun
    只检查不修改，列出需要格式化的文件。

.EXAMPLE
    .\format_code.ps1 -DryRun
    .\format_code.ps1
#>
param(
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

# 定位 clang-format：优先 PATH，其次 VSCode C/C++ 扩展自带
$clangFormat = (Get-Command clang-format -ErrorAction SilentlyContinue | Select-Object -First 1).Source
if (-not $clangFormat) {
    $bundled = Get-ChildItem "$env:USERPROFILE\.vscode\extensions" -Recurse -Filter clang-format.exe -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($bundled) { $clangFormat = $bundled.FullName }
}
if (-not $clangFormat) {
    Write-Error "未找到 clang-format，请安装 LLVM 或 VSCode C/C++ 扩展"
}

# 跳过格式化的第三方 / 生成目录（相对仓库根目录）
$exclude = @(
    "Drivers\STM32F4xx_HAL_Driver",
    "Drivers\CMSIS",
    "Middlewares",
    "Hurri-Cargo_Code",
    "Applications\letter_shell",
    "MDK-ARM",
    "build",
    ".cmsis"
)

$files = Get-ChildItem -Path $root -Recurse -File |
    Where-Object { $_.Extension -in ".c", ".h" } |
    Where-Object {
        $rel = $_.FullName.Substring($root.Length + 1)
        $skip = $false
        foreach ($dir in $exclude) {
            if ($rel.StartsWith($dir)) { $skip = $true; break }
        }
        -not $skip
    }

Write-Host "clang-format: $clangFormat"
Write-Host "目标文件数: $($files.Count)"

if ($DryRun) {
    # clang-format 的违规提示走 stderr，此处降级为 Continue 避免被当作终止错误
    $ErrorActionPreference = "Continue"
    $changed = 0
    foreach ($f in $files) {
        $null = & $clangFormat --dry-run --Werror -style=file $f.FullName 2>&1
        if ($LASTEXITCODE -ne 0) {
            $changed++
            Write-Host "需格式化: $($f.FullName.Substring($root.Length + 1))"
        }
    }
    Write-Host "检查完成: 需格式化 $changed / $($files.Count) 个文件（未修改）"
} else {
    foreach ($f in $files) {
        & $clangFormat -i -style=file $f.FullName
    }
    Write-Host "格式化完成"
}
