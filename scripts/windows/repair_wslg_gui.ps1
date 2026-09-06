[CmdletBinding()]
param(
    [ValidateSet('physics', 'perception')]
    [string]$Mode = 'physics',

    [ValidateSet('auto', 'manual')]
    [string]$ControlMode = 'auto',

    [ValidateSet('none', 'kiss_icp')]
    [string]$Localization = 'none',

    [ValidateSet('soil', 'localization')]
    [string]$Scenario = 'soil'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$wslProjectRoot = (& wsl -d Ubuntu-24.04 -- wslpath -a $projectRoot).Trim()

if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($wslProjectRoot)) {
    throw 'Could not translate the project path for Ubuntu-24.04.'
}
$launcher = "$wslProjectRoot/scripts/wsl/run_loader_soil_demo.sh"

Write-Host 'Repairing the WSLg shared-memory mount point...'
& wsl -d Ubuntu-24.04 -u root -- mkdir -p /mnt/shared_memory
if ($LASTEXITCODE -ne 0) {
    throw 'Could not create /mnt/shared_memory in Ubuntu-24.04.'
}
& wsl -d Ubuntu-24.04 -- mountpoint -q /mnt/shared_memory
if ($LASTEXITCODE -ne 0) {
    & wsl -d Ubuntu-24.04 -u root -- mount -t tmpfs -o mode=1777 tmpfs /mnt/shared_memory
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not mount the temporary WSLg shared-memory filesystem.'
    }
}

Write-Host 'Restarting WSL. This stops all currently running WSL processes...'
& wsl --shutdown
Start-Sleep -Seconds 2

Write-Host "Starting the loader demo immediately in $Mode / $ControlMode mode..."
Write-Host 'Keep this PowerShell window open while Gazebo is running.'
& wsl -d Ubuntu-24.04 -- bash $launcher $Mode $ControlMode $Localization $Scenario
$launcherStatus = $LASTEXITCODE

if ($launcherStatus -ne 0) {
    throw "Loader demo exited with status $launcherStatus."
}
