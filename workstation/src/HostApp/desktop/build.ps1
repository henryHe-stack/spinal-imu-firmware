param([string]$SdkPackage,[string]$OutputDirectory,[string]$Gcc='')
$ErrorActionPreference='Stop'
$ui=(Get-Item -LiteralPath $PSScriptRoot).Parent.FullName
$project=(Get-Item -LiteralPath $ui).Parent.FullName
$repository=Split-Path -Parent $project
$sharedUi=Join-Path (Split-Path -Parent $repository) 'HostApp'
$out=if($OutputDirectory){[IO.Path]::GetFullPath($OutputDirectory)}else{Join-Path $repository 'app'}
if(-not $SdkPackage){$SdkPackage=Join-Path $repository 'dependencies/Microsoft.Web.WebView2.1.0.2903.40.nupkg'}
if(-not $Gcc) {
    $tool=Get-Command gcc.exe -ErrorAction SilentlyContinue
    if($tool){$Gcc=$tool.Source}
    elseif(Test-Path 'D:/msys64/ucrt64/bin/gcc.exe'){$Gcc='D:/msys64/ucrt64/bin/gcc.exe'}
    else{throw 'Windows GCC required. Supply -Gcc.'}
}
if ((Get-FileHash -LiteralPath $SdkPackage -Algorithm SHA256).Hash -ne 'EF128016DD1E51C59178C827ED5B8AA3322C57AFA8675D930F8109505542AD74') {throw 'Unexpected WebView2 SDK package'}
[void][IO.Directory]::CreateDirectory($out)
$assets=Join-Path $out 'ui'
[void][IO.Directory]::CreateDirectory($assets)
foreach($name in 'js','vendor','index.html','styles.css','favicon.svg'){Copy-Item -LiteralPath (Join-Path $sharedUi $name) -Destination $assets -Recurse -Force}
Copy-Item -LiteralPath (Join-Path $ui 'js/local-wire.js') -Destination (Join-Path $assets 'js/local-wire.js') -Force
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip=[IO.Compression.ZipFile]::OpenRead($SdkPackage)
try {
    foreach($name in 'lib/net462/Microsoft.Web.WebView2.Core.dll','lib/net462/Microsoft.Web.WebView2.WinForms.dll','build/native/x64/WebView2Loader.dll','LICENSE.txt'){
        $target=if($name -eq 'LICENSE.txt'){'WebView2-LICENSE.txt'}else{[IO.Path]::GetFileName($name)}
        [IO.Compression.ZipFileExtensions]::ExtractToFile($zip.GetEntry($name),(Join-Path $out $target),$true)
    }
} finally { $zip.Dispose() }
$compiler=Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
$buildRelative='build/native_'+[guid]::NewGuid().ToString('N')
$native=& (Join-Path $project 'pc_f103_dashboard/build.ps1') -NativeOnly -PassThru -Gcc $Gcc -OutputDirectory $buildRelative
foreach($file in 'neck_wire_host.dll','neck_demo.exe') {Copy-Item -LiteralPath (Join-Path $native.BuildRoot $file) -Destination $out -Force}
$samples=Join-Path $out 'samples'
[void][IO.Directory]::CreateDirectory($samples)
Get-ChildItem -LiteralPath (Join-Path $project 'data/示例数据') -Filter '*.bin' | Copy-Item -Destination $samples -Force
$measurement=Join-Path $project 'pc_f103_dashboard/src'
$sources=@('Dashboard.cs','DeviceControl.cs','DeviceControlForm.cs','MeasurementProfile.cs','Recording.cs','SerialLink.cs','Sources.cs','WireModel.cs') | ForEach-Object { Join-Path $measurement $_ }
& $compiler /nologo /target:winexe /platform:x64 /optimize+ /utf8output /codepage:65001 /reference:System.Windows.Forms.dll /reference:System.Drawing.dll "/reference:$out/Microsoft.Web.WebView2.Core.dll" "/reference:$out/Microsoft.Web.WebView2.WinForms.dll" "/out:$out/颈部姿态三维工作站.exe" (Join-Path $PSScriptRoot 'DesktopHost.cs') (Join-Path $PSScriptRoot 'MeasurementDisplay.cs') @sources
if ($LASTEXITCODE -ne 0){throw 'Desktop build failed'}
