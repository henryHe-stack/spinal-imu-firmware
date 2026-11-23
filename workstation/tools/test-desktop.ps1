param([string]$CaptureDirectory='')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$app=Join-Path $root 'app'
if(-not $CaptureDirectory){
    $profiles=Get-ChildItem -LiteralPath (Join-Path $root 'src/work') -Filter measurement-profile.xml -File -Recurse -ErrorAction SilentlyContinue
    $record=$profiles | Where-Object { Test-Path -LiteralPath (Join-Path $_.Directory.FullName 'capture.bin') } | Sort-Object FullName | Select-Object -Last 1
    if(-not $record){throw 'Run tools/test.ps1 -Suite Host first, or supply -CaptureDirectory.'}
    $CaptureDirectory=$record.Directory.FullName
}
$check=Join-Path $app 'check.exe'
if(Test-Path -LiteralPath $check){throw 'app/check.exe already exists'}
try{
    $compiler=Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
    & $compiler /nologo /platform:x64 /target:exe /codepage:65001 /reference:UIAutomationClient.dll /reference:UIAutomationTypes.dll /reference:WindowsBase.dll "/lib:$env:WINDIR/Microsoft.NET/Framework64/v4.0.30319/WPF" /reference:System.Drawing.dll /reference:System.Windows.Forms.dll "/reference:$app/Microsoft.Web.WebView2.Core.dll" "/reference:$app/Microsoft.Web.WebView2.WinForms.dll" "/reference:$app/颈部姿态三维工作站.exe" "/out:$check" (Join-Path $root 'src/HostApp/desktop/WindowCheck.cs')
    if($LASTEXITCODE -ne 0){throw 'Window test build failed'}
    & $check $CaptureDirectory
    if($LASTEXITCODE -ne 0){throw 'Window checks failed'}
}finally{
    if(Test-Path -LiteralPath $check){Remove-Item -LiteralPath $check}
}
