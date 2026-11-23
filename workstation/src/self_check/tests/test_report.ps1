param([Parameter(Mandatory=$true)][string]$FixtureExe)
$ErrorActionPreference='Stop'
. (Join-Path (Split-Path -Parent $PSScriptRoot) 'tools/report.ps1')
$script:checks=0
function Assert-Check([bool]$Value,[string]$Message){
    $script:checks++
    if(-not $Value){throw ('FAIL '+$Message)}
}
function Read-Fixture([string]$Scenario){
    $state=New-DiagState
    & $FixtureExe $Scenario | ForEach-Object {Add-DiagLine $state $_}
    if($LASTEXITCODE -ne 0){throw 'Fixture generation failed'}
    return $state
}
$good=Read-Fixture 'good'
Assert-Check ($good.Blocks -eq 3 -and $good.ParseErrors -eq 0) 'three-node complete blocks'
Assert-Check ($good.Last.Summary.cycles -eq 300) 'cycle count'
Assert-Check ($good.Last.Nodes.Count -eq 3) 'node count'
$clean=Get-DiagVerdict $good
$single=Read-Fixture 'single'
Assert-Check ($single.Blocks -eq 3 -and $single.Last.Nodes.Count -eq 1) 'C4 block contract'
Assert-Check ((Get-DiagVerdict $single) -eq $clean) 'C4 clean'
foreach($scenario in @('readfail','invalid','cal','clock','timing')){
    $bad=Read-Fixture $scenario
    Assert-Check ($bad.ParseErrors -eq 0 -and $bad.Blocks -eq 3) ($scenario+' valid protocol')
    Assert-Check ((Get-DiagVerdict $bad) -ne $clean) ($scenario+' must not pass')
}
$lines=@(& $FixtureExe 'good' | Where-Object {$_})
$truncated=New-DiagState
$lines[0..($lines.Count-2)] | ForEach-Object {Add-DiagLine $truncated $_}
Assert-Check ($truncated.Blocks -eq 2 -and $null -ne $truncated.Pending) 'truncated block not published'
$temp=Join-Path (Split-Path -Parent $PSScriptRoot) 'build/tests/report_parse'
New-Item -ItemType Directory -Path $temp -Force | Out-Null
$null=Export-DiagReport $truncated $temp $true
Assert-Check ($truncated.ParseErrors -eq 1) 'EOF partial block must be reported'
$json=Get-Content -LiteralPath (Join-Path $temp 'result.json') -Raw | ConvertFrom-Json
Assert-Check ($json.replay -and $json.parse_errors -eq 1) 'replay and error provenance'
$missing=New-DiagState
$lines | Where-Object {$_ -notmatch 'NODE .* id=1 '} | ForEach-Object {Add-DiagLine $missing $_}
Assert-Check ($missing.Blocks -eq 0 -and $missing.ParseErrors -gt 0) 'missing nodes cannot pass'
$corrupt=New-DiagState
$lines | ForEach-Object {Add-DiagLine $corrupt ($_ -replace 'cycles=100 ','cycles=BOGUS ')}
Assert-Check ($corrupt.ParseErrors -gt 0 -and (Get-DiagVerdict $corrupt) -ne $clean) 'invalid numeric'
$duplicate=New-DiagState
$lines | ForEach-Object {Add-DiagLine $duplicate ($_ -replace 'END seq=100','END seq=100 seq=100')}
Assert-Check ($duplicate.ParseErrors -gt 0) 'duplicate fields'
$wrong=New-DiagState
$lines | ForEach-Object {Add-DiagLine $wrong ($_ -replace 'mask=7','mask=2')}
Assert-Check ($wrong.Blocks -eq 0) 'mode and mask mismatch'
$reset=Read-Fixture 'good'
$lines | ForEach-Object {Add-DiagLine $reset $_}
Assert-Check ($reset.Resets -gt 0 -and (Get-DiagVerdict $reset) -ne $clean) 'reboot or duplicate sequence'
$decrease=New-DiagState
$lines | ForEach-Object {
    $changed=$_
    if($_ -match '^SUMMARY seq=100 '){$changed=$_ -replace 'uart_errors=0','uart_errors=1'}
    Add-DiagLine $decrease $changed
}
Assert-Check ($decrease.ParseErrors -gt 0) 'cumulative counters cannot decrease'
$stall=Read-Fixture 'good';$stall.Silences=1
Assert-Check ((Get-DiagVerdict $stall) -ne $clean) 'serial silence cannot pass'
$none=New-DiagState
Assert-Check ((Get-DiagVerdict $none) -ne $clean) 'no data cannot pass'
Add-DiagError $none '<script>alert(1)</script>'
$null=Export-DiagReport $none $temp $true
$html=Get-Content -LiteralPath (Join-Path $temp 'report.html') -Raw -Encoding UTF8
Assert-Check ($html -notmatch '<script>') 'HTML encoding'
Assert-Check ($html -match '&lt;script&gt;') 'HTML encoded evidence'
Write-Host ('PASS report parser: '+$script:checks+' assertions')
