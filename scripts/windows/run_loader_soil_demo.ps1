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

$westonLog = '\\wsl.localhost\Ubuntu-24.04\mnt\wslg\weston.log'
if (Test-Path -LiteralPath $westonLog) {
    $gfxMode = Get-Content -LiteralPath $westonLog -ErrorAction SilentlyContinue |
        Select-String -Pattern 'use_gfxredir\s*=\s*[01]' |
        Select-Object -Last 1
    if ($null -ne $gfxMode -and $gfxMode.Line -match 'use_gfxredir\s*=\s*0') {
        # A headless validation can be healthy while WSLg needs repair. Do not
        # terminate it as a side effect of opening a second demo window.
        & wsl -d Ubuntu-24.04 -- pgrep -f '(^|/)gz sim|bash .*scripts/wsl/run_loader_soil_demo.sh' *> $null
        if ($LASTEXITCODE -eq 0) {
            throw 'A loader simulation is already running. Let its test finish or close it before starting WSLg repair; no WSL restart was performed.'
        }
        if ($LASTEXITCODE -ne 1) {
            throw 'Could not check for active simulations. WSLg repair was not started.'
        }
        Write-Warning 'WSLg is in COPY MODE; repairing it before Gazebo starts.'
        Write-Warning 'The repair restarts WSL and stops other processes currently running in WSL.'
        & (Join-Path $PSScriptRoot 'repair_wslg_gui.ps1') -Mode $Mode -ControlMode $ControlMode -Localization $Localization -Scenario $Scenario
        return
    }
}

$launcher = "$wslProjectRoot/scripts/wsl/run_loader_soil_demo.sh"
Write-Host "Starting loader simulation demo in $Mode / $ControlMode mode..."
Write-Host 'Close the Gazebo window or press Ctrl+C to stop.'

& wsl -d Ubuntu-24.04 -- bash $launcher $Mode $ControlMode $Localization $Scenario
$launcherStatus = $LASTEXITCODE

if ($launcherStatus -ne 0) {
    throw "Loader demo exited with status $launcherStatus."
}
