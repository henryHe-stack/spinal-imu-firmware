param([string]$ToolchainBin = '', [string]$OutputDirectory = '', [string]$FirmwareElf = 'Debug/NeckTilt_F103C8.elf')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutputDirectory) { $OutputDirectory = 'HostTests/target_audit_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
foreach ($name in @($OutputDirectory,$FirmwareElf)) {
    if ($name -notmatch '^[A-Za-z0-9_./-]+$' -or $name.Contains('..') -or [IO.Path]::IsPathRooted($name)) { throw 'Use ASCII relative paths inside this project.' }
}
$outputPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $OutputDirectory))
if (-not $outputPath.StartsWith($projectRoot + '\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $outputPath)) { throw 'Output must be a NEW project subdirectory.' }
if (-not (Test-Path -LiteralPath (Join-Path $projectRoot $FirmwareElf))) { throw 'Build firmware first. This script reads ELF only; never connects to a board.' }
if (-not $ToolchainBin) {
    $compiler = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($compiler) { $ToolchainBin = Split-Path -Parent $compiler.Source }
    else {
        foreach ($installPath in @('D:\ST','C:\ST','C:\Program Files\STMicroelectronics')) {
            if (Test-Path -LiteralPath $installPath) {
                $candidate = Get-ChildItem -LiteralPath $installPath -Recurse -Filter arm-none-eabi-gcc.exe -File -ErrorAction SilentlyContinue |
                    Sort-Object FullName -Descending | Select-Object -First 1
                if ($candidate) { $ToolchainBin = $candidate.DirectoryName; break }
            }
        }
    }
}
if (-not $ToolchainBin) { throw 'ARM GCC not found. Pass -ToolchainBin.' }
$gcc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$gdb = Join-Path $ToolchainBin 'arm-none-eabi-gdb.exe'
Push-Location -LiteralPath $projectRoot
try {
    New-Item -ItemType Directory -Path $outputPath | Out-Null
    $flags = @('-std=c99','-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-Os','-Wall','-Wextra','-Werror','-pedantic',
        '-fno-fast-math','-ffp-contract=off','-fstack-usage','-IAlgorithm/Inc','-IApplication/Inc','-IIntegration')
    & $gcc @flags -c tests/neck_target_abi.c -o ($OutputDirectory + '/neck_target_abi.o')
    if ($LASTEXITCODE -ne 0) { throw 'Target ABI compile assertions failed.' }
    & $gcc @flags -c Integration/neck_owner_step_example.c -o ($OutputDirectory + '/neck_owner_step_example.o')
    if ($LASTEXITCODE -ne 0) { throw 'C owner example failed ARM compilation.' }
    Write-Output 'ARM: 25 target ABI compile assertions and optional owner example passed. Objects NOT firmware-linked.'
    # A deliberate negative compilation must retain the missing-driver barrier.
    $guardLog = $OutputDirectory + '/hardware_guard.log'
    $savedErrorPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $gcc @flags -DNECK_INPUT_MODE=2 -c tests/neck_target_abi.c -o ($OutputDirectory + '/hardware_guard.o') 2>&1 |
            Out-File -LiteralPath $guardLog -Encoding utf8 -ErrorAction Stop
        $guardExit = $LASTEXITCODE
    } finally { $ErrorActionPreference = $savedErrorPreference }
    if ($guardExit -eq 0 -or -not (Select-String -LiteralPath $guardLog -SimpleMatch 'driver and hardware reference are not implemented')) {
        throw 'Hardware compile barrier was not enforced.'
    }
    Write-Output 'Expected negative compile: hardware-mode missing-driver #error remains enforced.'
    $typeNames = @('NeckFrame','NeckAcquisitionFrame','NeckQualityOutput','NeckQualityContext','NeckPipelineContext',
        'NeckPostureContext','NeckRelativeContext','NeckProcessingContext','NeckProcessingOutput','NeckPoseBatch',
        'NeckRelativeDiagnostic','NeckAlarmEvent','NeckAppStats','NeckTickClock','TCB_t','Queue_t','BlockLink_t')
    $expected = @(152,328,56,256,384,160,304,1976,1616,632,200,48,124,24,84,72,8)
    $args = @('-q','-batch','-nx','-ex','set pagination off')
    foreach ($typeName in $typeNames) { $args += @('-ex',('p sizeof(' + $typeName + ')')) }
    $args += $FirmwareElf
    $sizes = & $gdb @args
    if ($LASTEXITCODE -ne 0) { throw 'ELF type inspection failed.' }
    $numbers = @(foreach ($raw in $sizes) { if ($raw -match '^\$\d+ = (\d+)$') { [int]$Matches[1] } })
    if ($numbers.Count -ne $typeNames.Count) { throw 'Missing target ELF types.' }
    for ($i=0;$i -lt $typeNames.Count;$i++) {
        if ($numbers[$i] -ne $expected[$i]) { throw ('Target type changed: ' + $typeNames[$i]) }
        Write-Output ($typeNames[$i] + '=' + $numbers[$i])
    }
    $app = [IO.File]::ReadAllText((Join-Path $projectRoot 'Application/Src/neck_app.c'))
    $queueTypes = @('NeckAcquisitionFrame','NeckPoseBatch','uint8_t','NeckAlarmEvent','NeckRelativeDiagnostic')
    $depths = @(2,2,1,8,1); $itemSizes = @(328,632,1,48,200)
    $allocations = @()
    function Heap-Allocation([int]$Payload) { return [int]([math]::Ceiling(($Payload+8)/8.0)*8) }
    for ($i=0;$i -lt $queueTypes.Count;$i++) {
        $pattern = 'xQueueCreate\(' + $depths[$i] + 'U, sizeof\(' + $queueTypes[$i] + '\)\)'
        if (-not [regex]::IsMatch($app,$pattern)) { throw ('Queue declaration changed: ' + $queueTypes[$i]) }
        $bytes = Heap-Allocation (72+$depths[$i]*$itemSizes[$i]); $allocations += $bytes
        Write-Output ('heap queue ' + $queueTypes[$i] + ': ' + $bytes)
    }
    $tasks = @(@('acquisition_task',256,3),@('algorithm_task',896,2),@('telemetry_task',576,1))
    foreach ($task in $tasks) {
        $pattern = 'xTaskCreate\(' + $task[0] + ', "[^"]+", ' + $task[1] + 'U, NULL, ' + $task[2] + 'U,'
        if (-not [regex]::IsMatch($app,$pattern)) { throw ('Task declaration changed: ' + $task[0]) }
        $bytes = (Heap-Allocation ([int]$task[1]*4)) + (Heap-Allocation 84); $allocations += $bytes
        Write-Output ('heap task ' + $task[0] + ': ' + $bytes)
    }
    $config = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Inc/FreeRTOSConfig.h'))
    if ($config -notmatch 'configMINIMAL_STACK_SIZE \(\(uint16_t\)128\)' -or
        $config -notmatch 'configTOTAL_HEAP_SIZE \(\(size_t\)\(11 \* 1024\)\)') { throw 'Heap or idle stack changed; re-audit.' }
    $idle = (Heap-Allocation (128*4)) + (Heap-Allocation 84); $allocations += $idle
    $sum = [int]($allocations | Measure-Object -Sum).Sum
    if ($sum -ne 10752) { throw 'Heap estimate changed.' }
    Write-Output ('heap idle=' + $idle + '; allocated=' + $sum + '; available_after_end_block=11256; estimated_free=504 B')
    Write-Output 'STATIC ESTIMATE ONLY: heap_4 8-B header/alignment, no other allocations; not board measurement.'
    $tags = @('SIM_QUALITY_V1','SIM_POSTURE_V2','SIM_DIAG_V3','SIM_EVENT_V1','SIM_REL_V2')
    $columns = @(16,15,15,10,17)
    for ($i=0;$i -lt $tags.Count;$i++) {
        $format = [regex]::Match($app,'"' + $tags[$i] + ',[^"\r\n]+"')
        if (-not $format.Success -or $format.Value.Split(',').Count -ne $columns[$i]) { throw ('Serial schema drift: ' + $tags[$i]) }
        Write-Output ($tags[$i] + ' columns=' + $columns[$i])
    }
    Get-Content -LiteralPath ($OutputDirectory + '/neck_owner_step_example.su')
    Write-Output 'S09 target/header/queue/serial audit passed; no HAL/FreeRTOS execution or hardware connection.'
} finally { Pop-Location }
