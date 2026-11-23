param([string]$Gcc = 'D:\msys64\ucrt64\bin\gcc.exe', [string]$OutputDirectory = '',
    [ValidateSet('O0','O2')][string]$Optimization = 'O2')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot
if (-not $OutputDirectory) { $OutputDirectory = 'HostTests/run_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if ($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) {
    throw 'Use a new ASCII relative output directory inside this project.'
}
$outputPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $OutputDirectory))
if (-not $outputPath.StartsWith($projectRoot + '\', [StringComparison]::OrdinalIgnoreCase) -or
    (Test-Path -LiteralPath $outputPath)) { throw 'Output must be a new directory inside this project.' }
$compiler = Get-Command $Gcc -ErrorAction Stop
$compilerPath = $compiler.Source
$savedSearchPath = $env:PATH
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $savedSearchPath
try {
    New-Item -ItemType Directory -Path $outputPath | Out-Null
    $flags = @('-std=c99',('-' + $Optimization),'-Wall','-Wextra','-Werror','-pedantic',
        '-fno-fast-math','-ffp-contract=off','-IAlgorithm/Inc','-Itests','-IApplication/Inc')
    $version = & $compilerPath --version
    if ($LASTEXITCODE -ne 0) { throw 'Cannot execute host C compiler.' }
    $version | Select-Object -First 1
    Write-Output ('Host C optimization: ' + $Optimization)
    $legacyExe = $OutputDirectory + '/test_tilt.exe'
    & $compilerPath @flags Algorithm/Src/neck_tilt.c tests/neck_tilt_host_wrapper.c tests/test_neck_tilt.c -o $legacyExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'Legacy tilt C compilation failed.' }
    & (Join-Path $projectRoot $legacyExe) 2>&1 | Tee-Object -FilePath (Join-Path $outputPath 'tilt_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'Legacy tilt assertions failed.' }
    $postureExe = $OutputDirectory + '/test_posture.exe'
    & $compilerPath @flags Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c tests/test_neck_posture.c -o $postureExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'Pipeline/posture C compilation failed.' }
    & (Join-Path $projectRoot $postureExe) ($OutputDirectory + '/pipeline_posture.csv') 2>&1 |
        Tee-Object -FilePath (Join-Path $outputPath 'posture_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'Pipeline/posture assertions failed.' }
    $relativeExe = $OutputDirectory + '/test_relative.exe'
    & $compilerPath @flags Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c Algorithm/Src/neck_relative.c Application/Src/neck_simulator.c tests/test_neck_relative.c -o $relativeExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'Relative quaternion C compilation failed.' }
    & (Join-Path $projectRoot $relativeExe) ($OutputDirectory + '/relative_pipeline.csv') 2>&1 |
        Tee-Object -FilePath (Join-Path $outputPath 'relative_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'Relative quaternion assertions failed.' }
    $qualityExe = $OutputDirectory + '/test_quality.exe'
    & $compilerPath @flags Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c Algorithm/Src/neck_relative.c Algorithm/Src/neck_quality.c Algorithm/Src/neck_clock.c Application/Src/neck_processing.c Application/Src/neck_simulator.c tests/test_neck_quality.c -o $qualityExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'S07 quality/timeout compilation failed.' }
    & (Join-Path $projectRoot $qualityExe) ($OutputDirectory + '/quality_runtime.csv') 2>&1 |
        Tee-Object -FilePath (Join-Path $outputPath 'quality_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'S07 quality/timeout assertions failed.' }
    $replayExe = $OutputDirectory + '/test_replay.exe'
    $replaySources = @('Algorithm/Src/neck_tilt.c','Algorithm/Src/neck_pipeline.c','Algorithm/Src/neck_posture.c',
        'Algorithm/Src/neck_relative.c','Algorithm/Src/neck_quality.c','Application/Src/neck_processing.c','Application/Src/neck_simulator.c',
        'HostTools/Src/neck_replay_io.c','HostTools/Src/neck_replay.c','HostTools/Src/neck_replay_fixtures.c','HostTools/Src/neck_replay_plot.c',
        'tests/test_neck_replay.c')
    & $compilerPath @flags -IHostTools/Inc @replaySources -o $replayExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'S08 replay compilation failed.' }
    & (Join-Path $projectRoot $replayExe) ($OutputDirectory + '/replay_fixtures') 2>&1 |
        Tee-Object -FilePath (Join-Path $outputPath 'replay_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'S08 replay assertions failed.' }
    $contractExe = $OutputDirectory + '/test_contract.exe'
    & $compilerPath @flags -IIntegration Algorithm/Src/neck_tilt.c Algorithm/Src/neck_pipeline.c Algorithm/Src/neck_posture.c `
        Algorithm/Src/neck_relative.c Algorithm/Src/neck_quality.c Algorithm/Src/neck_clock.c `
        Application/Src/neck_processing.c Application/Src/neck_simulator.c Integration/neck_owner_step_example.c `
        tests/test_neck_contract.c -o $contractExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'S09 integration-contract compilation failed.' }
    & (Join-Path $projectRoot $contractExe) 2>&1 | Tee-Object -FilePath (Join-Path $outputPath 'contract_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'S09 integration-contract assertions failed.' }
    Write-Output ('C-only tests passed. Results: ' + $outputPath)
} finally { $env:PATH = $savedSearchPath }
