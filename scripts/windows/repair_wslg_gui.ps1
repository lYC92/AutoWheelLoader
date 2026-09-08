[CmdletBinding()]
param(
    [ValidateSet('physics', 'perception')]
    [string]$Mode = 'physics',

    [ValidateSet('auto', 'manual')]
    [string]$ControlMode = 'auto',

    [ValidateSet('none', 'kiss_icp', 'lio', 'lio_map')]
    [string]$Localization = 'none',

    [ValidateSet('soil', 'soil3d', 'ab', 'localization')]
    [string]$Scenario = 'ab',
    [switch]$SurroundCapture,
    [switch]$CaptureOnly,
    [ValidateRange(1, 1000)][int]$ContinuousCycles = 1,
    [ValidateRange(0, 2147483647)][int]$RandomSeed = 1001
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
$demoEnvironment = @(
    "LOADER_BEV_CAPTURE=$(($SurroundCapture.IsPresent -or $CaptureOnly.IsPresent).ToString().ToLower())",
    "LOADER_CAPTURE_ONLY=$($CaptureOnly.IsPresent.ToString().ToLower())",
    "LOADER_CONTINUOUS_CYCLES=$ContinuousCycles", "LOADER_RANDOM_SEED=$RandomSeed"
)
& wsl -d Ubuntu-24.04 -- env @demoEnvironment bash $launcher $Mode $ControlMode $Localization $Scenario
$launcherStatus = $LASTEXITCODE

if ($launcherStatus -ne 0) {
    throw "Loader demo exited with status $launcherStatus."
}
