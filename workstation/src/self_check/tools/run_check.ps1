param([string]$Port='', [ValidateRange(5,3600)][int]$Seconds=60,
      [string]$InputLog='', [switch]$NoOpen)
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -eq 5){$env:PSModulePath=Join-Path $PSHOME 'Modules'}
. (Join-Path $PSScriptRoot 'report.ps1')
$projectRoot=Split-Path -Parent $PSScriptRoot
$record=Join-Path $projectRoot ('records/'+(Get-Date -Format 'yyyyMMdd_HHmmss')+'_'+[guid]::NewGuid().ToString('N').Substring(0,6))
New-Item -ItemType Directory -Path $record -Force | Out-Null
$state=New-DiagState
$logPath=Join-Path $record 'log.txt'
$writer=New-Object IO.StreamWriter($logPath,$false,(New-Object Text.UTF8Encoding($false)))
$serial=$null;$reader=$null;$failure=''
try {
    if($InputLog){
        Write-Host '正在解析已有日志，不是当前连接样机实测。'
        $reader=[IO.File]::OpenText((Resolve-Path -LiteralPath $InputLog).Path)
        while(($line=$reader.ReadLine()) -ne $null){$writer.WriteLine($line);Add-DiagLine $state $line}
    } else {
        if(-not $Port){
            Write-Host ('电脑当前列出的串口：'+(([IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join '、'))
            $Port=Read-Host '请输入实际COM口 如COM5'
        }
        $Port=$Port.Trim().ToUpperInvariant()
        if($Port -notmatch '^COM[1-9][0-9]*$'){throw 'COM口格式不正确'}
        Write-Host '请关闭占用该COM口的上位机和PuTTY。程序仅接收，不发送设备命令。'
        $serial=New-Object IO.Ports.SerialPort($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
        $serial.Handshake=[IO.Ports.Handshake]::None;$serial.DtrEnable=$false;$serial.RtsEnable=$false
        $serial.ReadTimeout=500;$serial.NewLine="`n";$serial.Encoding=[Text.Encoding]::ASCII
        $serial.Open()
        Write-Host ('已打开 '+$Port+'，请轻按并松开板上RESET，随后自动记录'+$Seconds+'秒。')
        $watch=[Diagnostics.Stopwatch]::StartNew();$lastBlockAt=0.0;$inSilence=$false
        # Allow the already-started last report up to one second to finish.
        while($watch.Elapsed.TotalSeconds -lt $Seconds -or ($state.Pending -and $watch.Elapsed.TotalSeconds -lt $Seconds+1)){
            $old=$state.Blocks
            try {
                $line=$serial.ReadLine()
                $writer.WriteLine($line.TrimEnd("`r"));$writer.Flush();Add-DiagLine $state $line
            } catch [TimeoutException] { }
            if($state.Blocks -gt $old){
                $lastBlockAt=$watch.Elapsed.TotalSeconds;$inSilence=$false
                Write-Host ('收到报告 采集轮数='+$state.Last.Summary.cycles+' 状态='+$state.Last.Summary.state)
            }
            if($state.Blocks -and $watch.Elapsed.TotalSeconds-$lastBlockAt -gt 5 -and -not $inSilence){
                $state.Silences++;$inSilence=$true;Write-Host '超过5秒没有完整报告，已记录断流。'
            }
        }
    }
} catch {
    $failure=$_.Exception.Message;Add-DiagError $state $failure;Write-Host ('接收失败：'+$failure)
} finally {
    if($serial){if($serial.IsOpen){$serial.Close()};$serial.Dispose()}
    if($reader){$reader.Dispose()};$writer.Dispose()
}
$verdict=Export-DiagReport $state $record ([bool]$InputLog)
Write-Host $verdict
Write-Host ('结果文件夹：'+$record)
Write-Host ('RECORD_DIR='+$record)
if(-not $NoOpen){
    try {Start-Process -FilePath (Join-Path $record 'report.html')}
    catch {Write-Host '报告已保存，请到结果文件夹手动双击 report.html。'}
}
if($failure){exit 1}
