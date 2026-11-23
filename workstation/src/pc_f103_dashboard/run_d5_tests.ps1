param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',[string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
if(-not $OutputDirectory) { $OutputDirectory='work/d5_pc_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) { throw 'Use a new ASCII relative output directory.' }
$out=[IO.Path]::GetFullPath((Join-Path $taskRoot $OutputDirectory))
if(-not $out.StartsWith($taskRoot+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $out)) { throw 'Output must be new and inside the isolated workspace.' }
New-Item -ItemType Directory -Path $out | Out-Null
& (Join-Path $PSScriptRoot 'build.ps1') -Gcc $Gcc -OutputDirectory ($OutputDirectory+'/build')
$info=New-Object Diagnostics.ProcessStartInfo
$info.FileName=Join-Path $out 'build/NeckF103Dashboard.exe'
$info.Arguments='--control-self-test "'+$taskRoot+'" "'+$out+'" "'+(Join-Path $taskRoot 'data/test_fixtures')+'"'
$info.WorkingDirectory=Join-Path $out 'build'; $info.UseShellExecute=$false; $info.CreateNoWindow=$true
$process=[Diagnostics.Process]::Start($info)
if(-not $process.WaitForExit(30000)) { throw 'D5 PC tests exceeded 30 seconds.' }
$process.Refresh()
Get-Content -LiteralPath (Join-Path $out 'd5_control_test.log') -Encoding UTF8
if($process.ExitCode -ne 0 -or -not(Test-Path -LiteralPath (Join-Path $out 'D5_COMPLETE.txt'))) { throw 'D5 PC tests failed.' }
Write-Output ('D5 PC tests: '+$out)
