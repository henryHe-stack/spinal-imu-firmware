param([string]$Gcc='D:\msys64\ucrt64\bin\gcc.exe',[string]$OutputDirectory='',
    [switch]$NativeOnly,[switch]$PassThru)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $taskRoot
if (-not $OutputDirectory) { $OutputDirectory='work/d1_build_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff') }
if ($OutputDirectory -notmatch '^[A-Za-z0-9_/-]+$' -or $OutputDirectory.Contains('..')) { throw 'Use a new ASCII relative build directory.' }
$buildRoot=[IO.Path]::GetFullPath((Join-Path $taskRoot $OutputDirectory))
if (-not $buildRoot.StartsWith($taskRoot+'\',[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $buildRoot)) { throw 'Build output must be new and inside the isolated workspace.' }
$compiler=(Get-Command $Gcc -ErrorAction Stop).Source
$savedSearchPath=$env:PATH
$env:PATH=(Split-Path -Parent $compiler)+';'+$savedSearchPath
try {
    $null=New-Item -ItemType Directory -Path $buildRoot
    $flags=@('-std=c99','-O2','-Wall','-Wextra','-Werror','-pedantic','-fno-fast-math','-ffp-contract=off',
        '-static-libgcc','-Ihost_core/Algorithm/Inc','-Ihost_core/Application/Inc','-Ihost_core/Telemetry/Inc','-Ihost_core/Telemetry/Host')
    $wire=@('host_core/Telemetry/Src/neck_wire.c','host_core/Telemetry/Host/neck_wire_host.c')
    $core=@('host_core/Algorithm/Src/neck_tilt.c','host_core/Algorithm/Src/neck_pipeline.c','host_core/Algorithm/Src/neck_posture.c',
        'host_core/Algorithm/Src/neck_relative.c','host_core/Algorithm/Src/neck_quality.c',
        'host_core/Application/Src/neck_processing.c','host_core/Application/Src/neck_simulator.c')
    & $compiler @flags -shared @wire @core -o ($OutputDirectory+'/neck_wire_host.dll') -lm 2>&1 |
        Out-File -LiteralPath (Join-Path $buildRoot 'native_build.log') -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw 'Native bridge compilation failed; see native_build.log.' }
    & $compiler @flags @wire @core host_core/Telemetry/Host/neck_demo.c -o ($OutputDirectory+'/neck_demo.exe') -lm 2>&1 |
        Out-File -LiteralPath (Join-Path $buildRoot 'demo_build.log') -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw 'Shared C simulator compilation failed; see demo_build.log.' }
    if (-not $NativeOnly) {
        $csc=Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
        if (-not (Test-Path -LiteralPath $csc)) { throw '.NET Framework x64 C# compiler not found.' }
        $managed=Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'src') -Filter '*.cs' -File | Select-Object -ExpandProperty FullName
        & $csc /nologo /target:winexe /platform:x64 /optimize+ /warnaserror+ /utf8output `
            /reference:System.dll /reference:System.Core.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll `
            (('/out:')+(Join-Path $buildRoot 'NeckF103Dashboard.exe')) @managed 2>&1 |
            Out-File -LiteralPath (Join-Path $buildRoot 'managed_build.log') -Encoding utf8
        if ($LASTEXITCODE -ne 0) { throw 'Dashboard compilation failed; see managed_build.log.' }
    }
    $sourceFiles=@($wire)+@($core)+@('host_core/Telemetry/Inc/neck_wire.h','host_core/Telemetry/Host/neck_wire_host.h',
        'host_core/Telemetry/Host/neck_demo.c','pc_f103_dashboard/build.ps1')
    $sourceFiles+=@(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'host_core/Algorithm/Inc') -Filter '*.h' -File | Select-Object -ExpandProperty FullName)
    $sourceFiles+=@('host_core/Application/Inc/neck_app.h','host_core/Application/Inc/neck_simulator.h','host_core/Application/Inc/neck_processing.h')
    if (-not $NativeOnly) { $sourceFiles+=@($managed) }
    $sourceFiles | ForEach-Object { $hash=Get-FileHash -LiteralPath $_ -Algorithm SHA256; $hash.Hash+' '+$_ } |
        Out-File -LiteralPath (Join-Path $buildRoot 'source_sha256.txt') -Encoding utf8
    Write-Host ('Built D1 PC-only binaries: '+$buildRoot)
    if ($PassThru) { [pscustomobject]@{ BuildRoot=$buildRoot; TaskRoot=$taskRoot;
        Dashboard=(Join-Path $buildRoot 'NeckF103Dashboard.exe'); Demo=(Join-Path $buildRoot 'neck_demo.exe') } }
} finally { $env:PATH=$savedSearchPath }
