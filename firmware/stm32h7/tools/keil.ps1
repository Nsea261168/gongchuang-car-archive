param(
    [ValidateSet('Build', 'Flash', 'BuildFlash')]
    [string]$Action = 'BuildFlash'
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$mdkDirectory = Join-Path $projectRoot 'MDK-ARM'
$uv4Path = 'F:\STM32\Keil5\UV4\UV4.exe'
$projectName = 'CtrBoard-H7_FDCAN.uvprojx'
$projectPath = Join-Path $mdkDirectory $projectName
$axfPath = Join-Path $mdkDirectory 'CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.axf'

function Assert-FileExists {
    param([string]$Path, [string]$Description)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        Write-Host "[ERROR] Cannot find $Description`: $Path" -ForegroundColor Red
        exit 3
    }
}

function Invoke-Keil {
    param(
        [string[]]$Arguments,
        [string]$LogName
    )

    $logPath = Join-Path $mdkDirectory $LogName
    if (Test-Path -LiteralPath $logPath) {
        Remove-Item -LiteralPath $logPath -Force
    }

    $process = Start-Process -FilePath $uv4Path `
        -ArgumentList $Arguments `
        -WorkingDirectory $mdkDirectory `
        -Wait -PassThru -WindowStyle Hidden

    if (-not (Test-Path -LiteralPath $logPath -PathType Leaf)) {
        Write-Host "[ERROR] Keil did not create $logPath (exit code $($process.ExitCode))." -ForegroundColor Red
        exit 4
    }

    return Get-Content -Raw -LiteralPath $logPath
}

function Invoke-Build {
    Write-Host '==> Keil build' -ForegroundColor Cyan
    $buildText = Invoke-Keil `
        -Arguments @('-b', $projectName, '-j0', '-o', 'build.log') `
        -LogName 'build.log'
    Write-Host $buildText

    if ($buildText -notmatch '0 Error\(s\)') {
        Write-Host '[FAILED] Build contains errors. The previous firmware will not be flashed.' -ForegroundColor Red
        exit 1
    }

    Assert-FileExists -Path $axfPath -Description 'build output AXF'
    Write-Host '[OK] Build completed with 0 Error(s).' -ForegroundColor Green
}

function Invoke-Flash {
    Assert-FileExists -Path $axfPath -Description 'firmware AXF'
    $image = Get-Item -LiteralPath $axfPath
    Write-Host "==> Keil flash (firmware timestamp: $($image.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss')))" -ForegroundColor Cyan

    $flashText = Invoke-Keil `
        -Arguments @('-f', $projectName, '-o', 'flash.log') `
        -LogName 'flash.log'
    Write-Host $flashText

    if ($flashText -match 'Verify OK') {
        Write-Host '[OK] Erase / Program / Verify OK.' -ForegroundColor Green
        return
    }
    if ($flashText -match 'No ST-LINK detected|No ULINK|Cannot Load Flash Device Description') {
        Write-Host '[FAILED] Debug probe or flash configuration is unavailable. Check ST-Link, target power, and Keil target settings.' -ForegroundColor Red
        exit 2
    }

    Write-Host '[FAILED] Keil did not report Verify OK. See MDK-ARM\flash.log.' -ForegroundColor Red
    exit 2
}

Assert-FileExists -Path $uv4Path -Description 'Keil UV4'
Assert-FileExists -Path $projectPath -Description 'Keil project'

switch ($Action) {
    'Build'      { Invoke-Build }
    'Flash'      { Invoke-Flash }
    'BuildFlash' { Invoke-Build; Invoke-Flash }
}
