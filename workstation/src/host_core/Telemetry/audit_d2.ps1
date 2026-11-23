param([string]$ToolchainBin='', [string]$OutputDirectory='')
# Read-only ELF/ABI/static-budget audit; never connects to a board.
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
if(-not $OutputDirectory) { $OutputDirectory='HostTests/d2_audit_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..') -or
    [IO.Path]::IsPathRooted($OutputDirectory)) { throw 'Use a new ASCII relative output directory.' }
$out=[IO.Path]::GetFullPath((Join-Path $root $OutputDirectory))
if(-not $out.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $out)) {
    throw 'Output must be a new subdirectory of stm32.'
}
$elf=Join-Path $root 'Debug/NeckTilt_F103C8.elf'
if(-not(Test-Path -LiteralPath $elf)) { throw 'Build D2 firmware first.' }
if(-not $ToolchainBin) {
    $compiler=Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if($compiler) { $ToolchainBin=Split-Path -Parent $compiler.Source }
    else {
        foreach($base in @('D:\ST','C:\ST','C:\Program Files\STMicroelectronics')) {
            if(Test-Path -LiteralPath $base) {
                $candidate=Get-ChildItem -LiteralPath $base -Recurse -Filter arm-none-eabi-gdb.exe -File -ErrorAction SilentlyContinue |
                    Sort-Object FullName -Descending | Select-Object -First 1
                if($candidate) { $ToolchainBin=$candidate.DirectoryName; break }
            }
        }
    }
}
if(-not $ToolchainBin) { throw 'ARM toolchain not found.' }
$gdb=Join-Path $ToolchainBin 'arm-none-eabi-gdb.exe'
$size=Join-Path $ToolchainBin 'arm-none-eabi-size.exe'
$lines=[Collections.Generic.List[string]]::new()
$typeNames=@('NeckAcquisitionFrame','NeckWireSnapshot','NeckAlarmEvent','Queue_t','TCB_t','BlockLink_t','CommandReply')
$gdbArgs=@('-q','-batch','-nx','-ex','set pagination off')
foreach($name in $typeNames) { $gdbArgs+=@('-ex',('p sizeof('+$name+')')) }
$gdbArgs+=$elf
$raw=@(& $gdb @gdbArgs)
if($LASTEXITCODE -ne 0) { throw 'GDB ABI inspection failed.' }
$sizes=@(foreach($line in $raw) { if($line -match '^\$\d+ = (\d+)$') { [int]$matches[1] } })
if($sizes.Count -ne $typeNames.Count) { throw 'Missing ABI type information.' }
for($i=0;$i -lt $typeNames.Count;$i++) { $lines.Add($typeNames[$i]+'='+$sizes[$i]+' B') }
if($sizes[0] -ne 328 -or $sizes[1] -ne 584 -or $sizes[2] -ne 48 -or
   $sizes[3] -ne 72 -or $sizes[4] -ne 84 -or $sizes[5] -ne 8 -or $sizes[6] -ne 40) { throw 'D2 ABI changed; review queue budget.' }
$app=[IO.File]::ReadAllText((Join-Path $root 'Application/Src/neck_app.c'))
$queues=@(@('NeckAcquisitionFrame',2,328),@('NeckWireSnapshot',1,584),
    @('uint8_t',1,1),@('NeckAlarmEvent',8,48),@('CommandReply',1,40))
function Alloc([int]$payload) { [int]([math]::Ceiling(($payload+8)/8.0)*8) }
$total=0
foreach($q in $queues) {
    if($app -notmatch ('xQueueCreate\('+[int]$q[1]+'U, sizeof\('+$q[0]+'\)\)')) { throw ('Queue drift: '+$q[0]) }
    $cost=Alloc(72+[int]$q[1]*[int]$q[2]); $total+=$cost
    $lines.Add('queue '+$q[0]+' depth='+$q[1]+' heap_estimate='+$cost+' B')
}
$tasks=@(@('acquisition_task',256,3),@('algorithm_task',896,2),@('telemetry_task',576,1))
foreach($task in $tasks) {
    if($app -notmatch ('xTaskCreate\('+[string]$task[0]+', "[^"]+", '+[int]$task[1]+'U, NULL, '+[int]$task[2]+'U,')) {
        throw ('Task drift: '+$task[0])
    }
    $cost=(Alloc ([int]$task[1]*4))+(Alloc 84); $total+=$cost
    $lines.Add('task '+$task[0]+' stack_words='+$task[1]+' heap_estimate='+$cost+' B')
}
$idle=(Alloc (128*4))+(Alloc 84); $total+=$idle
$free=11256-$total
if($total -ne 9912 -or $free -lt 1024) { throw ('Heap budget unexpected: allocated='+$total+' free='+$free) }
$lines.Add('heap_4 available=11256 B; allocated_estimate='+$total+' B; free_estimate='+$free+' B')
$sections=@(& $size -A $elf)
if($LASTEXITCODE -ne 0) { throw 'Size inspection failed.' }
$data=0; $bss=0; $msp=0; [long]$ramEnd=0x20000000
foreach($line in $sections) {
    if($line -match '^\s*(\.data|\.bss|\._user_heap_stack)\s+(\d+)\s+(\d+)') {
        $ramEnd=[math]::Max($ramEnd,([long]$matches[3]+[long]$matches[2]))
        if($matches[1] -eq '.data') { $data=[int]$matches[2] }
        elseif($matches[1] -eq '.bss') { $bss=[int]$matches[2] } else { $msp=[int]$matches[2] }
    }
}
$ramSpan=$ramEnd-0x20000000
if(-not $bss -or -not $msp -or $ramSpan -gt 20480) { throw 'RAM budget overflow or missing size data.' }
$lines.Add('RAM data='+$data+' B bss='+$bss+' B MSP_section='+$msp+' B total_with_alignment='+$ramSpan+'/20480 B')
$lines.Add('FLASH from build.ps1 link report; ELF file SHA256='+((Get-FileHash -LiteralPath $elf -Algorithm SHA256).Hash))
$lines.Add('Stack .su frames are not call-chain maxima; board high-water/time still unmeasured.')
$lines.Add('PASS static D2 resource audit; no hardware execution.')
$null=New-Item -ItemType Directory -Path $out
[IO.File]::WriteAllLines((Join-Path $out 'audit.txt'),$lines,[Text.UTF8Encoding]::new($false))
$lines
Write-Output ('Audit saved: '+$out)
