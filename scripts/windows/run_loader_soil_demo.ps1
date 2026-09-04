[CmdletBinding()]
param(
    [ValidateSet('physics', 'perception')]
    [string]$Mode = 'physics'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$wslProjectRoot = (& wsl -d Ubuntu-24.04 -- wslpath -a $projectRoot).Trim()

if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($wslProjectRoot)) {
    throw 'Could not translate the project path for Ubuntu-24.04.'
}

$westonLog = '\\wsl.localhost\Ubuntu-24.04\mnt\wslg\weston.log'
if (Test-Path -LiteralPath $westonLog) {
    $gfxMode = Get-Content -LiteralPath $westonLog -ErrorAction SilentlyContinue |
        Select-String -Pattern 'use_gfxredir\s*=\s*[01]' |
        Select-Object -Last 1
    if ($null -ne $gfxMode -and $gfxMode.Line -match 'use_gfxredir\s*=\s*0') {
        throw @'
WSLg is in COPY MODE, so the Gazebo process can run while its window stays invisible.
Run .\scripts\windows\repair_wslg_gui.ps1; it repairs WSLg and starts the demo immediately.
'@
    }
}

$launcher = "$wslProjectRoot/scripts/wsl/run_loader_soil_demo.sh"
Write-Host "Starting loader simulation demo in $Mode mode..."
Write-Host 'Close the Gazebo window or press Ctrl+C to stop.'

& wsl -d Ubuntu-24.04 -- bash $launcher $Mode
$launcherStatus = $LASTEXITCODE

if ($launcherStatus -ne 0) {
    throw "Loader demo exited with status $launcherStatus."
}
