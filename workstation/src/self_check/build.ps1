param([ValidateSet('all','clean')][string]$Target='all',
      [ValidateSet('c4','three')][string]$Mode='three',
      [string]$ToolchainBin='', [switch]$Marker, [switch]$InternalSensorClock)
$ErrorActionPreference='Stop'
# CubeIDE may inherit a PowerShell 7 module path while invoking Windows PS 5.1.
if($PSVersionTable.PSVersion.Major -eq 5){$env:PSModulePath=Join-Path $PSHOME 'Modules'}
Set-Location -LiteralPath $PSScriptRoot
$out=Join-Path $PSScriptRoot ('build/'+$Mode)
if (-not [IO.Path]::GetFullPath($out).StartsWith($PSScriptRoot+'\build\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Unsafe build path.' }
$stem=if($Mode -eq 'c4'){'TimingDiag_C4'}else{'TimingDiag_Three'}
if($Target -eq 'clean') {
    if(Test-Path -LiteralPath $out) {
        Get-ChildItem -LiteralPath $out -Recurse -File | Where-Object {$_.Extension -in '.o','.d','.su','.elf','.bin','.hex','.map','.json','.log'} | ForEach-Object {
            if(-not $_.FullName.StartsWith($out+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Unsafe clean.'}
            Remove-Item -LiteralPath $_.FullName
        }
    }
    exit 0
}
if(-not $ToolchainBin) {
    $found=Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if($found){$ToolchainBin=Split-Path -Parent $found.Source}
}
if(-not $ToolchainBin) {
    foreach($root in @('D:\ST','C:\ST','C:\Program Files\STMicroelectronics')) {
        if(Test-Path -LiteralPath $root) {
            $found=Get-ChildItem -LiteralPath $root -Filter arm-none-eabi-gcc.exe -Recurse -File -ErrorAction SilentlyContinue | Select-Object -First 1
            if($found){$ToolchainBin=$found.DirectoryName;break}
        }
    }
}
if(-not $ToolchainBin){throw 'ARM GCC not found; provide -ToolchainBin.'}
$gcc=Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$mask=if($Mode -eq 'c4'){2}else{7}
$version=(Get-Content -LiteralPath VERSION -Raw).Trim()
if((Get-Content -LiteralPath src/diag_stats.h -Raw) -notmatch ('DIAG_VERSION "'+[regex]::Escape($version)+'"')){throw 'VERSION/header mismatch.'}
New-Item -ItemType Directory -Path $out -Force | Out-Null
# Remove only stale final products of this exact mode before attempting a build.
foreach($extension in @('.elf','.hex','.bin','.map','.json')) {
    $previous=Join-Path $out ($stem+$extension)
    if(Test-Path -LiteralPath $previous){Remove-Item -LiteralPath $previous}
}
$inc=@('src','firmware','vendor/platform','vendor/sensor','vendor/Drivers/STM32F1xx_HAL_Driver/Inc',
 'vendor/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy','vendor/Drivers/CMSIS/Include',
 'vendor/Drivers/CMSIS/Device/ST/STM32F1xx/Include',
 'vendor/Middlewares/Third_Party/FreeRTOS/Source/include',
 'vendor/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3')
$flags=@('-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-Os','-g3','-std=c99',
 '-ffunction-sections','-fdata-sections','-fno-common','-fno-fast-math','-ffp-contract=off',
 '-fstack-usage','-Wall','-Wextra','-Werror','-DUSE_HAL_DRIVER','-DSTM32F103xB',('-DDIAG_NODE_MASK='+$mask),
 ('-DDIAG_ENABLE_MARKER='+[int]$Marker.IsPresent),('-DDIAG_EXTERNAL_CLOCK='+[int](-not $InternalSensorClock.IsPresent)))
foreach($path in $inc){$flags+='-I'+$path}
$sources=@('src/diag_stats.c','src/diag_format.c','firmware/main.c','firmware/diag_timer.c','firmware/diag_app.c',
 'vendor/platform/system_stm32f1xx.c','vendor/platform/syscalls.c','vendor/platform/stm32f1xx_hal_msp.c',
 'vendor/platform/stm32f1xx_hal_timebase_tim.c','vendor/platform/stm32f1xx_it.c',
 'vendor/sensor/neck_bno055.c','vendor/sensor/neck_sensor_bus_stm32.c')
foreach($file in @('stm32f1xx_hal.c','stm32f1xx_hal_cortex.c','stm32f1xx_hal_dma.c','stm32f1xx_hal_flash.c',
 'stm32f1xx_hal_flash_ex.c','stm32f1xx_hal_gpio.c','stm32f1xx_hal_gpio_ex.c','stm32f1xx_hal_i2c.c',
 'stm32f1xx_hal_pwr.c','stm32f1xx_hal_rcc.c','stm32f1xx_hal_rcc_ex.c','stm32f1xx_hal_tim.c',
 'stm32f1xx_hal_tim_ex.c','stm32f1xx_hal_uart.c')){$sources+='vendor/Drivers/STM32F1xx_HAL_Driver/Src/'+$file}
foreach($file in @('tasks.c','queue.c','list.c','portable/GCC/ARM_CM3/port.c','portable/MemMang/heap_4.c')){
 $sources+='vendor/Middlewares/Third_Party/FreeRTOS/Source/'+$file
}
$objects=@()
foreach($source in $sources){
 $object='build/'+$Mode+'/'+$source.Replace('.c','.o')
 New-Item -ItemType Directory -Path (Split-Path -Parent $object) -Force | Out-Null
 & $gcc @flags -MMD -MP -c $source -o $object
 if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$source)}
 $objects+=$object
}
$startup='build/'+$Mode+'/startup.o'
& $gcc @flags -x assembler-with-cpp -c vendor/platform/startup_stm32f103xb.s -o $startup
if($LASTEXITCODE -ne 0){throw 'Startup compile failed.'}
$elf='build/'+$Mode+'/'+$stem+'.elf';$map='build/'+$Mode+'/'+$stem+'.map'
& $gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -T vendor/platform/STM32F103C8_FLASH.ld --specs=nano.specs --specs=nosys.specs "-Wl,--gc-sections,-Map=$map" '-Wl,--print-memory-usage' @objects $startup '-Wl,--start-group' -lc -lm -lnosys '-Wl,--end-group' -o $elf
if($LASTEXITCODE -ne 0){throw 'Link failed.'}
& (Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe') -O ihex $elf ('build/'+$Mode+'/'+$stem+'.hex')
if($LASTEXITCODE -ne 0){throw 'HEX export failed.'}
& (Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe') -O binary $elf ('build/'+$Mode+'/'+$stem+'.bin')
if($LASTEXITCODE -ne 0){throw 'BIN export failed.'}
& (Join-Path $ToolchainBin 'arm-none-eabi-size.exe') $elf
if($LASTEXITCODE -ne 0){throw 'Size inspection failed.'}
$trackedInputs=@(Get-ChildItem -LiteralPath src,firmware,vendor -Recurse -File)+@(Get-Item -LiteralPath build.ps1,VERSION)
$hashes=@($trackedInputs | Sort-Object FullName | ForEach-Object {
 [ordered]@{path=$_.FullName.Substring($PSScriptRoot.Length+1).Replace('\','/');sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}
})
[ordered]@{version=$version;mode=$Mode;node_mask=$mask;marker=$Marker.IsPresent;external_sensor_clock=(-not $InternalSensorClock.IsPresent);
 compiler=(& $gcc --version | Select-Object -First 1);compile_flags=$flags;verification='ARM build only; no hardware execution';sources=$hashes} |
 ConvertTo-Json -Depth 5 | Out-File -LiteralPath (Join-Path $out ($stem+'.json')) -Encoding UTF8
Write-Output ('BUILD_OK '+$elf+' (hardware-untested; read-window timestamps, not sensor sample times)')
