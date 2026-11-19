param([string]$HostGcc='')
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -eq 5){$env:PSModulePath=Join-Path $PSHOME 'Modules'}
$projectRoot=Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot
if(-not $HostGcc){
    $found=Get-Command gcc.exe -ErrorAction SilentlyContinue
    if($found){$HostGcc=$found.Source}
    elseif(Test-Path -LiteralPath 'D:/msys64/ucrt64/bin/gcc.exe'){$HostGcc='D:/msys64/ucrt64/bin/gcc.exe'}
}
if(-not $HostGcc){throw 'Host GCC not found. Provide -HostGcc. ARM GCC cannot run Windows tests.'}
$testOut=Join-Path $projectRoot 'build/tests'
New-Item -ItemType Directory -Path $testOut -Force | Out-Null
$env:PATH=(Split-Path -Parent $HostGcc)+';'+$env:PATH
foreach($opt in @('O0','O2')){
    # MinGW ld in this toolchain cannot reliably accept a Chinese absolute path.
    $exe='build/tests/stats_'+$opt+'.exe'
    & $HostGcc ('-'+$opt) -std=c99 -g -Wall -Wextra -Werror -fno-fast-math -ffp-contract=off -Isrc src/diag_stats.c src/diag_format.c tests/test_stats.c -o $exe
    if($LASTEXITCODE -ne 0){throw ('Host compile failed '+$opt)}
    & $exe
    if($LASTEXITCODE -ne 0){throw ('Statistics tests failed '+$opt)}
}
$fixture='build/tests/make_fixture.exe'
& $HostGcc -O2 -std=c99 -Wall -Wextra -Werror -fno-fast-math -ffp-contract=off -Isrc src/diag_stats.c src/diag_format.c tests/make_fixture.c -o $fixture
if($LASTEXITCODE -ne 0){throw 'Fixture compile failed'}
& (Join-Path $PSScriptRoot 'test_report.ps1') -FixtureExe $fixture
if($LASTEXITCODE -ne 0){throw 'Report tests failed'}
$log=Join-Path $testOut 'OFFLINE_TEST_good.txt'
& $fixture 'good' | Out-File -LiteralPath $log -Encoding ASCII
if($LASTEXITCODE -ne 0){throw 'Fixture log failed'}
$collectorOutput=& powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/run_check.ps1 -InputLog $log -NoOpen 2>&1
$collectorOutput | Write-Host
if($LASTEXITCODE -ne 0){throw 'Offline collector integration failed'}
$recordLine=@($collectorOutput | Where-Object {$_ -like 'RECORD_DIR=*'})
if($recordLine.Count -ne 1){throw 'Missing collector output directory'}
$recordPath=$recordLine[0].ToString().Substring(11)
$result=Get-Content -LiteralPath (Join-Path $recordPath 'result.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if(-not $result.replay -or $result.complete_blocks -ne 3 -or $result.parse_errors -ne 0 -or $result.nodes.Count -ne 3){throw 'Collector integration report mismatch'}
$invalidOutput=& powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/run_check.ps1 -Port INVALID -NoOpen 2>&1
if($LASTEXITCODE -ne 1){throw 'Invalid COM input must fail before opening a port'}
$recordLine=@($invalidOutput | Where-Object {$_ -like 'RECORD_DIR=*'})
if($recordLine.Count -ne 1){throw 'Invalid COM input should still save a report'}
$invalidPath=$recordLine[0].ToString().Substring(11)
$invalid=Get-Content -LiteralPath (Join-Path $invalidPath 'result.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if($invalid.complete_blocks -ne 0 -or $invalid.parse_errors -lt 1 -or $invalid.replay){throw 'Invalid COM report mismatch'}
Write-Host 'PASS collector integration: replay and invalid-COM error path'
Write-Host 'PASS_ALL: desktop-only tests; NOT hardware validation.'
