param([ValidateSet('O0','O2')][string]$Optimization='O2',
      [string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',
      [string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
if(-not $OutputDirectory) { $OutputDirectory='HostTests/d2_producer_'+$Optimization+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..') -or
   [IO.Path]::IsPathRooted($OutputDirectory)) { throw 'Use a new ASCII relative output directory.' }
$out=[IO.Path]::GetFullPath((Join-Path $root $OutputDirectory))
if(-not $out.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $out)) {
    throw 'Output must be a new subdirectory of stm32.'
}
$compiler=(Get-Command $Gcc -ErrorAction Stop).Source
$savedPath=$env:PATH
$env:PATH=(Split-Path -Parent $compiler)+';'+$savedPath
Push-Location -LiteralPath $root
try {
    $null=New-Item -ItemType Directory -Path $out
    $sources=@('Telemetry/Tests/test_neck_wire_producer.c','Telemetry/Src/neck_wire_producer.c',
        'Telemetry/Src/neck_wire.c','Algorithm/Src/neck_tilt.c','Algorithm/Src/neck_pipeline.c',
        'Algorithm/Src/neck_posture.c','Algorithm/Src/neck_relative.c','Algorithm/Src/neck_quality.c',
        'Application/Src/neck_processing.c','Application/Src/neck_simulator.c')
    $exe=$OutputDirectory+'/test_neck_wire_producer.exe'
    & $compiler -std=c99 ('-'+$Optimization) -Wall -Wextra -Werror -pedantic -fno-fast-math -ffp-contract=off `
        -IAlgorithm/Inc -IApplication/Inc -ITelemetry/Inc @sources -o $exe -lm
    if($LASTEXITCODE -ne 0) { throw 'D2 host compile failed.' }
    & (Join-Path $root $exe) 2>&1 | Tee-Object -FilePath (Join-Path $out 'test.log')
    if($LASTEXITCODE -ne 0) { throw 'D2 host assertions failed.' }
    'PASS: pure-C producer / D0 codec; no HAL, RTOS or physical sensor execution.' |
        Set-Content -LiteralPath (Join-Path $out 'RUN_COMPLETE.txt') -Encoding UTF8
    Write-Output ('D2 host test output: '+$out)
} finally { Pop-Location; $env:PATH=$savedPath }
