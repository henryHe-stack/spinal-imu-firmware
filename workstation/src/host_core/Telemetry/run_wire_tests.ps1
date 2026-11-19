param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',
    [ValidateSet('O0','O2')][string]$Optimization='O2',
    [string]$OutputDirectory='', [string]$ArmGcc='')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot
if (-not $OutputDirectory) { $OutputDirectory='HostTests/d0_wire_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if ($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) {
    throw 'Use a new ASCII relative output directory inside stm32.'
}
$outputPath=[IO.Path]::GetFullPath((Join-Path $projectRoot $OutputDirectory))
if (-not $outputPath.StartsWith($projectRoot+'\',[StringComparison]::OrdinalIgnoreCase) -or
    (Test-Path -LiteralPath $outputPath)) { throw 'Output must be a new directory inside stm32; no overwrite.' }
$compilerPath=(Get-Command $Gcc -ErrorAction Stop).Source
$savedSearchPath=$env:PATH
$env:PATH=(Split-Path -Parent $compilerPath)+';'+$savedSearchPath
try {
    $null=New-Item -ItemType Directory -Path (Join-Path $outputPath 'fixtures')
    $version=& $compilerPath --version
    if ($LASTEXITCODE -ne 0) { throw 'Host compiler unavailable.' }
    $version | Select-Object -First 1
    $flags=@('-std=c99',('-'+$Optimization),'-Wall','-Wextra','-Werror','-pedantic',
        '-fno-fast-math','-ffp-contract=off','-ITelemetry/Inc')
    $exe=$OutputDirectory+'/test_neck_wire.exe'
    & $compilerPath @flags Telemetry/Src/neck_wire.c Telemetry/Tests/test_neck_wire.c -o $exe
    if ($LASTEXITCODE -ne 0) { throw 'D0 C compilation failed.' }
    & (Join-Path $projectRoot $exe) ($OutputDirectory+'/fixtures') 2>&1 |
        Tee-Object -FilePath (Join-Path $outputPath 'wire_test.log')
    if ($LASTEXITCODE -ne 0) { throw 'D0 protocol tests failed.' }
    if ($ArmGcc) {
        $armCompiler=(Get-Command $ArmGcc -ErrorAction Stop).Source
        & $armCompiler -std=c99 -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -Wall -Wextra -Werror -pedantic `
            -ffunction-sections -fdata-sections -fstack-usage -ITelemetry/Inc -c Telemetry/Src/neck_wire.c `
            -o ($OutputDirectory+'/neck_wire_arm.o')
        if ($LASTEXITCODE -ne 0) { throw 'D0 Cortex-M3 object compilation failed.' }
        $armSize=Join-Path (Split-Path -Parent $armCompiler) 'arm-none-eabi-size.exe'
        & $armSize ($OutputDirectory+'/neck_wire_arm.o') | Tee-Object -FilePath (Join-Path $outputPath 'arm_size.log')
        if ($LASTEXITCODE -ne 0) { throw 'ARM object size check failed.' }
        & $armCompiler -std=c99 -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -Wall -Wextra -Werror -pedantic `
            -ITelemetry/Inc -c Telemetry/Tests/neck_wire_target_layout.c -o ($OutputDirectory+'/wire_layout_arm.o')
        if ($LASTEXITCODE -ne 0) { throw 'ARM layout probe compilation failed.' }
        $armObjdump=Join-Path (Split-Path -Parent $armCompiler) 'arm-none-eabi-objdump.exe'
        & $armObjdump -s -j .rodata ($OutputDirectory+'/wire_layout_arm.o') |
            Tee-Object -FilePath (Join-Path $outputPath 'arm_layout.log')
        if ($LASTEXITCODE -ne 0) { throw 'ARM layout probe inspection failed.' }
    }
    $sources=@('Telemetry/Inc/neck_wire.h','Telemetry/Src/neck_wire.c','Telemetry/Tests/test_neck_wire.c',
        'Telemetry/Tests/neck_wire_target_layout.c','Telemetry/run_wire_tests.ps1')
    $sources | ForEach-Object { $hash=Get-FileHash -LiteralPath $_ -Algorithm SHA256; $hash.Hash+' '+$_ } |
        Out-File -LiteralPath (Join-Path $outputPath 'source_sha256.txt') -Encoding utf8
    Get-ChildItem -LiteralPath (Join-Path $outputPath 'fixtures') -File | ForEach-Object {
        $hash=Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256; $hash.Hash+' '+$_.Name
    } | Out-File -LiteralPath (Join-Path $outputPath 'fixture_sha256.txt') -Encoding utf8
    ('D0 C-only codec/parser/receiver tests passed; optimization='+$Optimization+
     '; ARM object checked='+[bool]$ArmGcc+'; no UART/RTOS/hardware execution.') |
        Out-File -LiteralPath (Join-Path $outputPath 'RUN_COMPLETE.txt') -Encoding utf8
    Write-Output ('D0 passed: '+$outputPath)
} finally { $env:PATH=$savedSearchPath }
