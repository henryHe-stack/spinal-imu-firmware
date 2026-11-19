param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',[string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
if (-not $OutputDirectory) { $OutputDirectory='work/d1_verify_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if ($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) { throw 'Use a new ASCII relative test directory.' }
$testRoot=[IO.Path]::GetFullPath((Join-Path $taskRoot $OutputDirectory))
if (-not $testRoot.StartsWith($taskRoot+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $testRoot)) { throw 'Test output must be new and inside isolated workspace.' }
$null=New-Item -ItemType Directory -Path $testRoot
$build=& (Join-Path $PSScriptRoot 'build.ps1') -Gcc $Gcc -OutputDirectory ($OutputDirectory+'/build') -PassThru
Set-Location -LiteralPath $taskRoot
foreach ($mode in @('three','single')) {
    $boot=if ($mode -eq 'three') { 101 } else { 201 }
    $producerInfo=New-Object Diagnostics.ProcessStartInfo
    $producerInfo.FileName=$build.Demo
    $producerInfo.Arguments='--generate '+$OutputDirectory+'/sim_'+$mode+'.bin --duration 65 --mode '+$mode+' --script --boot '+$boot
    $producerInfo.WorkingDirectory=$taskRoot
    $producerInfo.UseShellExecute=$false
    $producerInfo.CreateNoWindow=$true
    $producerInfo.RedirectStandardError=$true
    $producer=[Diagnostics.Process]::Start($producerInfo)
    if (-not $producer.WaitForExit(10000)) { throw ('C producer still running: '+$mode) }
    $producer.StandardError.ReadToEnd() | Out-File -LiteralPath (Join-Path $testRoot ('producer_'+$mode+'.log')) -Encoding utf8
    if ($producer.ExitCode -ne 0) { throw ('C producer failed: '+$mode) }
}
$fixtureRoot=Join-Path $taskRoot 'data/test_fixtures'
if (-not (Test-Path -LiteralPath $fixtureRoot)) { throw 'D0 release fixtures not found; run D0 tests first.' }
$arguments='--self-test "'+$taskRoot+'" "'+$testRoot+'" "'+$fixtureRoot+'"'
$testInfo=New-Object Diagnostics.ProcessStartInfo
$testInfo.FileName=$build.Dashboard
$testInfo.Arguments=$arguments
$testInfo.WorkingDirectory=$build.BuildRoot
$testInfo.UseShellExecute=$false
$testInfo.CreateNoWindow=$true
$process=[Diagnostics.Process]::Start($testInfo)
if (-not $process.WaitForExit(30000)) { throw 'D1 tests still running after 30 seconds; inspect the owned test process and logs.' }
$process.Refresh()
if ($process.ExitCode -ne 0 -or -not (Test-Path -LiteralPath (Join-Path $testRoot 'MANAGED_COMPLETE.txt'))) {
    if (Test-Path -LiteralPath (Join-Path $testRoot 'dashboard_test.log')) { Get-Content -LiteralPath (Join-Path $testRoot 'dashboard_test.log') }
    throw ('D1 managed tests failed: '+$testRoot)
}
Get-Content -LiteralPath (Join-Path $testRoot 'dashboard_test.log')
$filesToHash=@(Get-ChildItem -LiteralPath $testRoot -File)
$filesToHash | ForEach-Object { $h=Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256; $h.Hash+' '+$_.Name } |
    Out-File -LiteralPath (Join-Path $testRoot 'sha256.txt') -Encoding utf8
[IO.File]::WriteAllText((Join-Path $testRoot 'RUN_COMPLETE.txt'),'PASS D1 PC-only verification. No physical devices or serial ports tested.',[Text.UTF8Encoding]::new($false))
Write-Host ('D1 verification complete: '+$testRoot)
