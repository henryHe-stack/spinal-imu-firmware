param(
    [ValidateSet('App','Firmware50','Firmware100','SelfC4','SelfThree','All')][string]$Target='App',
    [string]$Gcc=''
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
function Invoke-Step([string]$File,[string[]]$Parameters) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $File @Parameters
    if($LASTEXITCODE -ne 0){throw "Build failed: $File"}
}
if($Target -in 'App','All'){
    $stepArgs=@('-OutputDirectory',(Join-Path $root 'app'))
    if($Gcc){$stepArgs+=@('-Gcc',$Gcc)}
    Invoke-Step (Join-Path $root 'src/HostApp/desktop/build.ps1') $stepArgs
}
foreach($mode in 'Firmware50','Firmware100'){
    if($Target -ne $mode -and $Target -ne 'All'){continue}
    $stepArgs=@('-Preset','Release')
    if($mode -eq 'Firmware100'){$stepArgs+='-Usb100Hz'}
    Invoke-Step (Join-Path $root 'src/stm32/build.ps1') $stepArgs
    $build=if($mode -eq 'Firmware50'){'Release'}else{'Release-USB100'}
    $dest=Join-Path $root $(if($mode -eq 'Firmware50'){'firmware/main_50hz'}else{'firmware/main_100hz_usb'})
    [void][IO.Directory]::CreateDirectory($dest)
    foreach($ext in '.elf','.hex','.bin'){
        Copy-Item -LiteralPath (Join-Path $root ('src/stm32/build/'+$build+'/Spinal_tester'+$ext)) -Destination $dest -Force
    }
}
foreach($mode in 'c4','three'){
    $key=if($mode -eq 'c4'){'SelfC4'}else{'SelfThree'}
    if($Target -ne $key -and $Target -ne 'All'){continue}
    Invoke-Step (Join-Path $root 'src/self_check/build.ps1') @('-Mode',$mode)
    $stem=if($mode -eq 'c4'){'TimingDiag_C4'}else{'TimingDiag_Three'}
    $dest=Join-Path $root ('firmware/self_check_'+$mode)
    [void][IO.Directory]::CreateDirectory($dest)
    foreach($ext in '.elf','.hex','.bin'){
        Copy-Item -LiteralPath (Join-Path $root ('src/self_check/build/'+$mode+'/'+$stem+$ext)) -Destination $dest -Force
    }
}
Write-Host "BUILD_OK $Target"
