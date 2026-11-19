param([string]$Gcc = 'D:\msys64\ucrt64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
$output = 'HostTests/c4_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff')
New-Item -ItemType Directory -Path $output | Out-Null
$savedPath = $env:PATH
$env:PATH = (Split-Path -Parent $Gcc) + ';' + $env:PATH
try {
    foreach ($opt in @('O0','O2')) {
        $exe = $output + '/test_c4_' + $opt + '.exe'
        & $Gcc -std=c99 ('-' + $opt) -Wall -Wextra -Werror -pedantic -fno-fast-math -ffp-contract=off `
            -IAlgorithm/Inc -IApplication/Inc -ISensors/Inc -ITelemetry/Inc `
            Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c `
            Algorithm/Src/neck_quality.c Algorithm/Src/neck_relative.c `
            Application/Src/neck_processing.c Application/Src/neck_c4_port.c `
            Telemetry/Src/neck_wire.c Telemetry/Src/neck_wire_producer.c tests/test_neck_c4.c -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw 'C4 host compile failed.' }
        & (Join-Path (Get-Location) $exe) 2>&1 | Tee-Object -FilePath ($output + '/' + $opt + '.log')
        if ($LASTEXITCODE -ne 0) { throw 'C4 host assertions failed.' }
    }
    Write-Output ('C4 results: ' + (Join-Path (Get-Location) $output))
} finally { $env:PATH = $savedPath }
