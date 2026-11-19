param([ValidateSet(2,7)][int]$NodeMask = 2, [string]$ToolchainBin = '',
    [ValidateSet('hardware-diagnostic','hardware-c4','hardware-three')][string]$InputMode = 'hardware-diagnostic')
$ErrorActionPreference = 'Stop'
$stm32Root = Split-Path -Parent $PSScriptRoot
$taskRoot = Split-Path -Parent $stm32Root
if ($InputMode -eq 'hardware-c4' -and $NodeMask -ne 2) { throw 'C4 requires mask 2.' }
$buildDirectory = if ($InputMode -eq 'hardware-three') { 'DebugThree' }
    elseif ($InputMode -eq 'hardware-c4') { 'DebugC4' }
    elseif ($NodeMask -eq 7) { 'DebugD3_Three' } else { 'DebugD3' }
$elf = Join-Path $stm32Root ($buildDirectory + '/NeckTilt_F103C8.elf')
if (-not (Test-Path -LiteralPath $elf)) {
    throw ('Build the D3 mask=' + $NodeMask + ' image first.')
}
if (-not $ToolchainBin) {
    $command = Get-Command arm-none-eabi-size.exe -ErrorAction SilentlyContinue
    if ($command) { $ToolchainBin = Split-Path -Parent $command.Source }
}
if (-not $ToolchainBin) {
    foreach ($base in @('D:\ST','C:\ST','C:\Program Files\STMicroelectronics')) {
        if (-not (Test-Path -LiteralPath $base)) { continue }
        $candidate = Get-ChildItem -LiteralPath $base -Recurse -Filter arm-none-eabi-size.exe -File `
            -ErrorAction SilentlyContinue | Sort-Object FullName -Descending | Select-Object -First 1
        if ($candidate) { $ToolchainBin = $candidate.DirectoryName; break }
    }
}
if (-not $ToolchainBin) { throw 'ARM size tool not found.' }
$sizeTool = Join-Path $ToolchainBin 'arm-none-eabi-size.exe'
$gdbTool = Join-Path $ToolchainBin 'arm-none-eabi-gdb.exe'
$abi = @(& $gdbTool -q -batch -nx -ex 'p sizeof(NeckProcessingContext)' `
    -ex 'p sizeof(NeckAlarmEvent)' -ex 'p sizeof(NeckAcquisitionFrame)' $elf)
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect hardware ABI.' }
$abiSizes = @(foreach ($line in $abi) { if ($line -match '^\$\d+ = (\d+)$') { [int]$matches[1] } })
if ($abiSizes.Count -ne 3 -or $abiSizes[0] -ne 1976 -or $abiSizes[1] -ne 48 -or $abiSizes[2] -ne 328) {
    throw 'Hardware ABI changed; review static and queue budgets.'
}
$sections = @{}
[long]$ramEnd=0x20000000
foreach ($line in @(& $sizeTool -A $elf)) {
    if ($line -match '^\s*(\.data|\.bss|\._user_heap_stack)\s+(\d+)\s+(\d+)') {
        $ramEnd=[math]::Max($ramEnd,([long]$matches[3]+[long]$matches[2]))
        $sections[$matches[1]] = [int]$matches[2]
    }
}
if ($LASTEXITCODE -ne 0 -or $sections.Count -ne 3) { throw 'Cannot inspect D3 ELF sections.' }
$appSource = [IO.File]::ReadAllText((Join-Path $stm32Root 'Application/Src/neck_app.c'))
$eventDepth = if ($InputMode -ne 'hardware-diagnostic') { 4 } else { 1 }
if ($appSource -notmatch 'xTaskCreate\(acquisition_task, "D3Probe", 384U, NULL, 3U, NULL\)' -or
    $appSource -notmatch ('event_queue = xQueueCreate\(' + $eventDepth + 'U, sizeof\(NeckAlarmEvent\)\)')) {
    throw 'D3 task stack or diagnostic event queue changed; revise heap estimate.'
}
$ramTotal = $ramEnd-0x20000000 # Include alignment gaps between sections.
$heapUsable = 11256 # heap_4 end marker deducted from configTOTAL_HEAP_SIZE=11*1024.
$d2TaskQueueEstimate = 9912 # D5 SIM: 584 B snapshot and new 40 B reply queue.
$d3TaskQueueEstimate = $d2TaskQueueEstimate + (384 - 256) * 4 - (8 - $eventDepth) * 48
$heapEstimateLeft = $heapUsable - $d3TaskQueueEstimate
$lines = @(
    'Hardware software-only static resource audit; no board execution.'
    ('Mode=' + $InputMode + '; context ABI=1976 B; event queue depth=' + $eventDepth)
    ('Image=' + $elf)
    ('NodeMask=' + $NodeMask)
    ('RAM .data=' + $sections['.data'] + ' B .bss=' + $sections['.bss'] +
        ' B MSP reserve=' + $sections['._user_heap_stack'] + ' B')
    ('RAM total=' + $ramTotal + '/20480 B; unallocated address space=' + (20480 - $ramTotal) + ' B')
    ('heap_4 usable=' + $heapUsable + ' B; D2 task/queue estimate=' + $d2TaskQueueEstimate +
        ' B; acquisition stack +512 B; event queue saving=' + ((8-$eventDepth)*48) + ' B; estimated post-start free=' +
        $heapEstimateLeft + ' B')
    'Estimate excludes unexpected dynamic allocations and is not a runtime stack high-water measurement.'
)
if ($ramTotal -gt 20480 -or $heapEstimateLeft -le 0) { throw ($lines -join [Environment]::NewLine) }
if ($heapEstimateLeft -lt 1536) {
    $lines += 'WARNING: estimated heap margin below 1.5 KiB; do not declare hardware release-ready.'
}
$outRoot = Join-Path $taskRoot 'work/d3_sensor_build'
New-Item -ItemType Directory -Path $outRoot -Force | Out-Null
$outFile = Join-Path $outRoot ('audit_' + $InputMode + '_mask' + $NodeMask + '.txt')
[IO.File]::WriteAllLines($outFile, $lines, [Text.UTF8Encoding]::new($false))
$lines
Write-Output ('Audit saved: ' + $outFile)
