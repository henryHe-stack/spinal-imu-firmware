param([ValidateSet('Core','Host','Self','Protocol','Bench','All')][string]$Suite='All',[string]$Gcc='')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
if(-not $Gcc){
    $tool=Get-Command gcc.exe -ErrorAction SilentlyContinue
    if($tool){$Gcc=$tool.Source}
    elseif(Test-Path 'D:/msys64/ucrt64/bin/gcc.exe'){$Gcc='D:/msys64/ucrt64/bin/gcc.exe'}
}
if(-not $Gcc -and $Suite -ne 'Protocol'){throw 'Windows GCC required. Supply -Gcc or add gcc.exe to PATH.'}
function Invoke-Check([string]$Relative,[string[]]$Parameters){
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root $Relative) @Parameters
    if($LASTEXITCODE -ne 0){throw "Test failed: $Relative"}
}
if($Suite -in 'Core','All'){
    Invoke-Check 'src/host_core/tests/run_c_tests.ps1' @('-Gcc',$Gcc)
    Invoke-Check 'src/host_core/tests/run_d5_tests.ps1' @('-Gcc',$Gcc)
    Invoke-Check 'src/host_core/Telemetry/run_wire_tests.ps1' @('-Gcc',$Gcc)
}
if($Suite -in 'Host','All'){
    Invoke-Check 'src/pc_f103_dashboard/run_d4_tests.ps1' @('-Gcc',$Gcc)
    Invoke-Check 'src/pc_f103_dashboard/run_d5_tests.ps1' @('-Gcc',$Gcc)
}
if($Suite -in 'Self','All'){
    Invoke-Check 'src/self_check/tests/run_tests.ps1' @('-HostGcc',$Gcc)
}
if($Suite -in 'Protocol','All'){
    Push-Location -LiteralPath (Join-Path $root 'src/HostApp')
    try{
        & node --test tests/protocol.test.mjs tests/signals.test.mjs tests/local-wire.test.mjs
        if($LASTEXITCODE -ne 0){throw 'JavaScript protocol tests failed'}
    }finally{Pop-Location}
}
if($Suite -in 'Bench','All'){
    Push-Location -LiteralPath (Join-Path $root 'src/stm32')
    $oldPath=$env:PATH
    try{
        $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
        [void][IO.Directory]::CreateDirectory((Join-Path (Get-Location) 'build/bench'))
        & $Gcc -municode -std=c11 -Wall -Wextra -Werror -Itests/bench -IBSP/Inc -ICommon/Inc -IServices/Inc -IDevices/Inc -IConfig/Inc tests/bench/verify.c BSP/Src/bsp_time.c Devices/Src/bno055.c Devices/Src/tca9548a.c Services/Src/sensor_service.c Services/Src/protocol_service.c -o build/bench/verify.exe
        if($LASTEXITCODE -ne 0){throw 'Bench test build failed'}
        & (Join-Path (Get-Location) 'build/bench/verify.exe') (Join-Path $root 'app/neck_wire_host.dll')
        if($LASTEXITCODE -ne 0){throw 'Bench tests failed'}
    }finally{$env:PATH=$oldPath;Pop-Location}
}
Write-Host "TEST_OK $Suite"
