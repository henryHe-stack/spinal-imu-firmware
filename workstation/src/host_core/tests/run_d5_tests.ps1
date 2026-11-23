param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe')
$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
$output='HostTests/d5_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff')
New-Item -ItemType Directory -Path $output | Out-Null
$savedPath=$env:PATH
$env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
try {
    foreach($opt in @('O0','O2')) {
        $exe=$output+'/test_d5_'+$opt+'.exe'
        & $Gcc -std=c99 ('-'+$opt) -Wall -Wextra -Werror -pedantic -fno-fast-math -ffp-contract=off `
            -IAlgorithm/Inc -IApplication/Inc -ISensors/Inc -ITelemetry/Inc `
            Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c `
            Algorithm/Src/neck_quality.c Algorithm/Src/neck_relative.c Application/Src/neck_processing.c `
            Application/Src/neck_measurement_port.c Application/Src/neck_settings.c `
            Telemetry/Src/neck_wire.c Telemetry/Src/neck_control.c tests/test_neck_d5.c -lm -o $exe
        if($LASTEXITCODE -ne 0) { throw 'D5 compilation failed.' }
        & (Join-Path (Get-Location) $exe) 2>&1 | Tee-Object -FilePath ($output+'/'+$opt+'.log')
        if($LASTEXITCODE -ne 0) { throw 'D5 tests failed.' }
        $appExe=$output+'/test_app_'+$opt+'.exe'
        & $Gcc -std=c99 ('-'+$opt) -Wall -Wextra -Werror -pedantic -fno-fast-math -ffp-contract=off `
            -Itests/d5_stubs -IAlgorithm/Inc -IApplication/Inc -ISensors/Inc -ITelemetry/Inc `
            Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c `
            Algorithm/Src/neck_quality.c Algorithm/Src/neck_relative.c Algorithm/Src/neck_clock.c `
            Application/Src/neck_processing.c Application/Src/neck_simulator.c Application/Src/neck_settings.c `
            Telemetry/Src/neck_wire.c Telemetry/Src/neck_control.c Telemetry/Src/neck_wire_producer.c `
            tests/test_neck_app_d5.c -lm -o $appExe
        if($LASTEXITCODE -ne 0) { throw 'D5 application test compilation failed.' }
        & (Join-Path (Get-Location) $appExe) 2>&1 | Tee-Object -FilePath ($output+'/app_'+$opt+'.log')
        if($LASTEXITCODE -ne 0) { throw 'D5 application tests failed.' }
    }
    Write-Output ('D5 portable tests: '+(Join-Path (Get-Location) $output))
} finally { $env:PATH=$savedPath }
