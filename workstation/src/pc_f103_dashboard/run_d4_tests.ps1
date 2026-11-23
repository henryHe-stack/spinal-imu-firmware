param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',[string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
if (-not $OutputDirectory) { $OutputDirectory='work/d4_verify_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if ($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) { throw 'Use a new ASCII relative output directory.' }
$testRoot=[IO.Path]::GetFullPath((Join-Path $taskRoot $OutputDirectory))
if (-not $testRoot.StartsWith($taskRoot+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $testRoot)) { throw 'Output must be new and inside the isolated workspace.' }
$null=New-Item -ItemType Directory -Path $testRoot
# The existing UI/SIM/record/replay suite builds the updated program once.
& (Join-Path $PSScriptRoot 'run_dashboard_tests.ps1') -Gcc $Gcc -OutputDirectory ($OutputDirectory+'/d1_regression')
$buildRoot=Join-Path $testRoot 'd1_regression/build'
$fixtureRoot=Join-Path $taskRoot 'data/test_fixtures'
$info=New-Object Diagnostics.ProcessStartInfo
$info.FileName=Join-Path $buildRoot 'NeckF103Dashboard.exe'
$info.Arguments='--serial-self-test "'+$taskRoot+'" "'+$testRoot+'" "'+$fixtureRoot+'"'
$info.WorkingDirectory=$buildRoot
$info.UseShellExecute=$false
$info.CreateNoWindow=$true
$process=[Diagnostics.Process]::Start($info)
if (-not $process.WaitForExit(30000)) { throw 'D4 tests exceeded 30 seconds; inspect the owned process and logs.' }
$process.Refresh()
if (Test-Path -LiteralPath (Join-Path $testRoot 'd4_serial_test.log')) { Get-Content -Encoding UTF8 -LiteralPath (Join-Path $testRoot 'd4_serial_test.log') }
if ($process.ExitCode -ne 0 -or -not (Test-Path -LiteralPath (Join-Path $testRoot 'D4_COMPLETE.txt'))) { throw ('D4 tests failed: '+$testRoot) }
Write-Output ('D4 software verification complete: '+$testRoot)
