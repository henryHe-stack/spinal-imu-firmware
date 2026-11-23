param([string]$Gcc = 'D:\msys64\ucrt64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$outputRoot = Join-Path $projectRoot 'work/d3_sensor_build'
$compiler = Get-Command $Gcc -ErrorAction SilentlyContinue
if (-not $compiler) { $compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue }
if (-not $compiler) { throw 'Host gcc.exe not found. Install/enable a C99 GCC in PATH.' }
$savedPath = $env:PATH
$env:PATH = (Split-Path -Parent $compiler.Source) + ';' + $env:PATH
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
Set-Location -LiteralPath $PSScriptRoot

foreach ($optimization in @('O0','O2')) {
    # Pass ASCII relative paths to MinGW GCC; this workspace has Chinese path segments.
    $testExe = '../../work/d3_sensor_build/test_bno055_' + $optimization + '.exe'
    & $compiler.Source '-std=c99' '-Wall' '-Wextra' '-Werror' ('-' + $optimization) `
        '-IInc' '-I../Application/Inc' '-I../Algorithm/Inc' '-I../Telemetry/Inc' `
        'Src/neck_bno055.c' 'Src/neck_sensor_acquisition.c' `
        '../Algorithm/Src/neck_quality.c' '../Telemetry/Src/neck_wire.c' `
        '../Telemetry/Src/neck_wire_producer.c' `
        'tests/test_bno055.c' `
        '-lm' '-o' $testExe
    if ($LASTEXITCODE -ne 0) { throw ('Host compilation failed: ' + $optimization) }
    & (Join-Path $outputRoot ('test_bno055_' + $optimization + '.exe')) 2>&1 |
        Tee-Object -FilePath (Join-Path $outputRoot ('test_bno055_' + $optimization + '.log'))
    if ($LASTEXITCODE -ne 0) { throw ('Fake-bus test failed: ' + $optimization) }
}
Write-Output 'D3 host fake-bus checks passed; no physical sensor, timing or sync verified.'
