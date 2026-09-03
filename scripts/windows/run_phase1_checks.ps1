$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$wslProjectRoot = (wsl wslpath -a $projectRoot).Trim()

if (-not $wslProjectRoot) {
    throw 'Could not translate the project path for WSL.'
}

$checkScript = "$wslProjectRoot/scripts/wsl/run_phase1_checks.sh"
wsl -d Ubuntu-24.04 -- bash $checkScript
if ($LASTEXITCODE -ne 0) {
    throw "Phase-1 checks failed with exit code $LASTEXITCODE."
}
