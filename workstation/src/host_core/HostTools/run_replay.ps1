param(
    [ValidateSet('Generate','Replay')][string]$Action = 'Generate',
    [ValidateSet('baseline','silence_faults','timing_faults','motion')][string]$Scenario = 'silence_faults',
    [string]$InputCsv = '', [string]$ConfigCsv = '', [string]$TruthCsv = '',
    [string]$OutputDirectory = '', [string]$Gcc = 'D:\msys64\ucrt64\bin\gcc.exe',
    [ValidateSet('O0','O2')][string]$Optimization = 'O2'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
function Assert-RelativeAscii([string]$Name) {
    if (-not $Name -or $Name -notmatch '^[A-Za-z0-9_./-]+$' -or $Name.Contains('..') -or [IO.Path]::IsPathRooted($Name)) {
        throw 'Use ASCII relative paths inside this STM32 project. Paths with .. are not allowed.'
    }
    $resolved = [IO.Path]::GetFullPath((Join-Path $projectRoot $Name))
    if (-not $resolved.StartsWith($projectRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Path leaves project.' }
    return $resolved
}
if (-not $OutputDirectory) { $OutputDirectory = 'ReplayRuns/' + $Scenario + '_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
$outputPath = Assert-RelativeAscii $OutputDirectory
if (Test-Path -LiteralPath $outputPath) { throw 'Output directory already exists; choose a new name. No overwrite.' }
if ($Action -eq 'Replay') {
    foreach ($name in @($InputCsv,$ConfigCsv)) {
        $inputPath = Assert-RelativeAscii $name
        if (-not (Test-Path -LiteralPath $inputPath -PathType Leaf)) { throw ('Missing input: ' + $name) }
    }
    if ($TruthCsv) {
        $truthPath = Assert-RelativeAscii $TruthCsv
        if (-not (Test-Path -LiteralPath $truthPath -PathType Leaf)) { throw 'Missing optional truth.' }
    }
}
$compiler = (Get-Command $Gcc -ErrorAction Stop).Source
$savedSearchPath = $env:PATH
$env:PATH = (Split-Path -Parent $compiler) + ';' + $savedSearchPath
Push-Location -LiteralPath $projectRoot
try {
    New-Item -ItemType Directory -Path 'HostTools/bin' -Force | Out-Null
    $outputParent = Split-Path -Parent $outputPath
    if (-not (Test-Path -LiteralPath $outputParent)) { New-Item -ItemType Directory -Path $outputParent -Force | Out-Null }
    $sources = @('Algorithm/Src/neck_tilt.c','Algorithm/Src/neck_pipeline.c','Algorithm/Src/neck_posture.c',
        'Algorithm/Src/neck_relative.c','Algorithm/Src/neck_quality.c','Application/Src/neck_processing.c',
        'Application/Src/neck_simulator.c','HostTools/Src/neck_replay_io.c','HostTools/Src/neck_replay.c',
        'HostTools/Src/neck_replay_fixtures.c','HostTools/Src/neck_replay_plot.c','HostTools/Src/neck_replay_main.c')
    $flags = @('-std=c99',('-' + $Optimization),'-Wall','-Wextra','-Werror','-pedantic','-fno-fast-math','-ffp-contract=off',
        '-IAlgorithm/Inc','-IApplication/Inc','-IHostTools/Inc')
    & $compiler @flags @sources -o HostTools/bin/neck_replay.exe -lm
    if ($LASTEXITCODE -ne 0) { throw 'C replay compilation failed.' }
    if ($Action -eq 'Generate') {
        & ./HostTools/bin/neck_replay.exe generate $Scenario $OutputDirectory
        if ($LASTEXITCODE -ne 0) { throw 'C dataset generation failed.' }
        $replayArgs = @('replay',($OutputDirectory + '/inputs.csv'),($OutputDirectory + '/config.csv'),
            ($OutputDirectory + '/result'),($OutputDirectory + '/truth.csv'))
        $reportPath = Join-Path $outputPath 'result/index.html'
    } else {
        $replayArgs = @('replay',$InputCsv,$ConfigCsv,$OutputDirectory)
        if ($TruthCsv) { $replayArgs += $TruthCsv }
        $reportPath = Join-Path $outputPath 'index.html'
    }
    & ./HostTools/bin/neck_replay.exe @replayArgs
    if ($LASTEXITCODE -ne 0) { throw 'C replay failed; inspect FAILED.txt and partial CSVs.' }
    Write-Output ('Open in browser: ' + $reportPath)
} finally { Pop-Location; $env:PATH = $savedSearchPath }
