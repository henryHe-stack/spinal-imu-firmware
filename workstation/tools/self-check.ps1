param([string]$Port='', [ValidateRange(5,3600)][int]$Seconds=60)
$root=Split-Path -Parent $PSScriptRoot
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root 'src/self_check/tools/run_check.ps1') -Port $Port -Seconds $Seconds
exit $LASTEXITCODE
