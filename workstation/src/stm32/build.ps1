param(
    [ValidateSet('all','clean')][string]$Target = 'all',
    [ValidateSet('Debug','Release')][string]$Preset = 'Debug',
    [switch]$Usb100Hz,
    [string]$ToolchainBin = '',
    [string]$CmakeBin = '',
    [string]$NinjaBin = ''
)
$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -eq 5) { $env:PSModulePath = Join-Path $PSHOME 'Modules' }
Set-Location -LiteralPath $PSScriptRoot

$buildDir = Join-Path $PSScriptRoot ('build/' + $Preset)
if ($Usb100Hz) { $buildDir += '-USB100' }
if ($Target -eq 'clean') {
    if (Test-Path -LiteralPath $buildDir) {
        Remove-Item -LiteralPath $buildDir -Recurse -Force
    }
    Write-Output ('Cleaned CMake build directory: ' + $buildDir)
    exit 0
}

function Find-Tool([string]$exeName, [string]$hintDir) {
    if ($hintDir -and (Test-Path -LiteralPath (Join-Path $hintDir $exeName))) {
        return (Join-Path $hintDir $exeName)
    }
    $cmd = Get-Command $exeName -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $fastMatches = Get-Item "D:\ST\*\STM32CubeIDE\plugins\*externaltools*\tools\bin\$exeName", "C:\ST\*\STM32CubeIDE\plugins\*externaltools*\tools\bin\$exeName" -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1
    if ($fastMatches) { return $fastMatches.FullName }
    foreach ($root in @('D:\ST','C:\ST',"$env:LOCALAPPDATA\stm32cube\bundles",'C:\Program Files\STMicroelectronics')) {
        if (Test-Path -LiteralPath $root) {
            $found = Get-ChildItem -LiteralPath $root -Recurse -Filter $exeName -File -ErrorAction SilentlyContinue |
                Sort-Object FullName -Descending | Select-Object -First 1
            if ($found) { return $found.FullName }
        }
    }
    throw ('Required tool not found: ' + $exeName)
}

$gccPath = Find-Tool 'arm-none-eabi-gcc.exe' $ToolchainBin
$cmakePath = Find-Tool 'cmake.exe' $CmakeBin
$ninjaPath = Find-Tool 'ninja.exe' $NinjaBin
$toolchainDir = Split-Path -Parent $gccPath
$cmakeDir = Split-Path -Parent $cmakePath
$ninjaDir = Split-Path -Parent $ninjaPath

$savedPath = $env:PATH
$env:PATH = "$toolchainDir;$cmakeDir;$ninjaDir;$savedPath"
try {
    Write-Output ('Toolchain: ' + $gccPath)
    Write-Output ('CMake:     ' + $cmakePath)
    Write-Output ('Ninja:     ' + $ninjaPath)
    $mode = if ($Usb100Hz) { 'ON' } else { 'OFF' }
    & $cmakePath --preset $Preset -B $buildDir "-DNECK_USB_100HZ=$mode"
    if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }
    & $cmakePath --build $buildDir
    if ($LASTEXITCODE -ne 0) { throw 'CMake build failed.' }

    $elf = Join-Path $buildDir 'Spinal_tester.elf'
    $hex = Join-Path $buildDir 'Spinal_tester.hex'
    $bin = Join-Path $buildDir 'Spinal_tester.bin'
    $objcopy = Join-Path $toolchainDir 'arm-none-eabi-objcopy.exe'
    $size = Join-Path $toolchainDir 'arm-none-eabi-size.exe'

    & $objcopy -O ihex $elf $hex
    if ($LASTEXITCODE -ne 0) { throw 'HEX export failed.' }
    & $objcopy -O binary $elf $bin
    if ($LASTEXITCODE -ne 0) { throw 'BIN export failed.' }
    & $size $elf
    if ($LASTEXITCODE -ne 0) { throw 'Size inspection failed.' }
    Write-Output ('BUILD_OK: ' + $elf + ' USB100=' + $mode)
} finally {
    $env:PATH = $savedPath
}
