$ErrorActionPreference = 'Continue'

Write-Output '=== Windows ==='
cmd /c ver

Write-Output '=== WSL ==='
wsl --status
wsl --version
wsl --list --verbose

Write-Output '=== NVIDIA GPU ==='
nvidia-smi --query-gpu=name,memory.total,driver_version,compute_cap --format=csv,noheader

Write-Output '=== Project disk ==='
Get-PSDrive -Name C | Select-Object Name, Used, Free

