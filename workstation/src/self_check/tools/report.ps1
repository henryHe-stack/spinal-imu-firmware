# ASCII diagnostic text is untrusted input. Only complete matching blocks publish.
function New-DiagState {
    return @{ Header=$null; Pending=$null; First=$null; Last=$null; Blocks=0; Unknown=0;
        ParseErrors=0; Resets=0; Silences=0; Errors=New-Object 'System.Collections.Generic.List[string]' }
}
function Add-DiagError($State,[string]$Message) {
    $State.ParseErrors++
    if($State.Errors.Count -lt 20){$State.Errors.Add($Message)}
}
function Convert-DiagFields([string]$Line) {
    if($Line.Length -gt 2048){throw 'Line too long.'}
    $tokens=$Line.Split(@(' '),[StringSplitOptions]::RemoveEmptyEntries)
    $data=@{Kind=$tokens[0]}
    for($i=1;$i -lt $tokens.Count;$i++) {
        if($tokens[$i] -notmatch '^([a-z][a-z0-9_]*)=([A-Za-z0-9_.-]+)$'){throw 'Malformed field.'}
        $key=$Matches[1];$value=$Matches[2]
        if($data.ContainsKey($key)){throw 'Duplicate field.'}
        $data[$key]=$value
    }
    return $data
}
function Require-DiagNumbers($Fields,[string[]]$Keys) {
    foreach($key in $Keys){
        [uint32]$number=0
        if(-not $Fields.ContainsKey($key) -or -not [uint32]::TryParse([string]$Fields[$key],[ref]$number)){throw ('Invalid number '+$key)}
        $Fields[$key]=[uint64]$number
    }
}
function Add-DiagLine($State,[string]$Line) {
    try {
        $line=$Line.Trim()
        if(-not $line){return}
        if($line -notmatch '^(IMU_SELF_CHECK|SUMMARY|NODE|END|WAIT)( |$)'){$State.Unknown++;return}
        $f=Convert-DiagFields $line
        switch($f.Kind) {
            'IMU_SELF_CHECK' {
                Require-DiagNumbers $f @('mask','period_us','tolerance_us','skew_limit_us','timer_hz','external_clock')
                if($f.version -ne '0.1.0' -or $f.source -ne 'HARDWARE' -or
                   $f.mode -notin @('C4','THREE') -or $f.mask -ne $(if($f.mode -eq 'C4'){2}else{7}) -or
                   $f.period_us -ne 10000 -or $f.tolerance_us -ne 2000 -or $f.skew_limit_us -ne 10000 -or
                   $f.clock -ne 'HSI8' -or $f.timer -ne 'TIM3' -or $f.timer_hz -ne 1000000 -or
                   $f.external_clock -notin @(0,1) -or $f.marker -notin @('OFF','PA8') -or
                   $f.sample_time -ne 'UNKNOWN' -or $f.drdy -ne 'DISABLED'){throw 'Unsupported diagnostic identity/configuration.'}
                if($State.Pending){Add-DiagError $State '前一报告块不完整';$State.Pending=$null}
                if($State.Header -and ($State.Header.mode -ne $f.mode -or $State.Header.external_clock -ne $f.external_clock -or $State.Header.marker -ne $f.marker)){
                    throw 'Configuration changed inside one recording.'
                }
                $State.Header=$f
            }
            'SUMMARY' {
                if(-not $State.Header){throw 'Missing diagnostic header.'}
                if($State.Pending){Add-DiagError $State '新汇总覆盖了未结束报告块'}
                Require-DiagNumbers $f @('seq','cycles','periods','period_avg_us','period_min_us','period_max_us',
                    'period_outside','cycle_max_us','overrun','skew_max_us','clock_errors','uart_errors',
                    'report_drops','heap_bytes','acq_stack_words','tx_stack_words')
                if($f.seq -ne $f.cycles -or $f.cycles -eq 0 -or $f.periods -ne $f.cycles-1 -or
                   $f.period_min_us -gt $f.period_avg_us -or $f.period_avg_us -gt $f.period_max_us -or
                   $f.period_outside -gt $f.periods -or $f.overrun -gt $f.cycles -or
                   $f.state -notin @('RUN_OK','CAL_PENDING','TIMING_WARN','READ_ERROR','DATA_ERROR','CLOCK_ERROR')){throw 'Inconsistent summary.'}
                $State.Pending=@{Summary=$f;Nodes=@{}}
            }
            'NODE' {
                if(-not $State.Pending){throw 'Node without summary.'}
                Require-DiagNumbers $f @('seq','id','attempts','ok','fail','invalid','cal_pending','read_max_us','result','sys_cal','acc_cal','qnorm2_milli')
                $summary=$State.Pending.Summary
                if($f.seq -ne $summary.seq -or $f.id -gt 2 -or
                   ($State.Header.mask -band (1 -shl [int]$f.id)) -eq 0 -or
                   $f.attempts -ne $summary.cycles -or $f.ok+$f.fail -ne $f.attempts -or
                   $f.invalid -gt $f.ok -or $f.cal_pending -gt $f.ok -or
                   $f.sys_cal -notin @(0,1,2,3,255) -or $f.acc_cal -notin @(0,1,2,3,255) -or
                   $State.Pending.Nodes.ContainsKey([int]$f.id)){throw 'Inconsistent/duplicate node.'}
                $State.Pending.Nodes[[int]$f.id]=$f
            }
            'END' {
                Require-DiagNumbers $f @('seq')
                if(-not $State.Pending -or $f.seq -ne $State.Pending.Summary.seq){throw 'Mismatched END.'}
                $expected=if($State.Header.mask -eq 2){1}else{3}
                if($State.Pending.Nodes.Count -ne $expected){throw 'Incomplete node set.'}
                if($State.Last -and $f.seq -le $State.Last.Summary.seq){$State.Resets++;throw 'Sequence reset or reversal.'}
                if($State.Last){
                    foreach($key in @('period_max_us','period_outside','cycle_max_us','overrun','skew_max_us','clock_errors','uart_errors','report_drops')){
                        if($State.Pending.Summary[$key] -lt $State.Last.Summary[$key]){throw ('Cumulative counter decreased: '+$key)}
                    }
                    foreach($id in $State.Pending.Nodes.Keys){foreach($key in @('attempts','ok','fail','invalid','cal_pending','read_max_us')){
                        if($State.Pending.Nodes[$id][$key] -lt $State.Last.Nodes[$id][$key]){throw ('Node counter decreased: '+$key)}
                    }}
                }
                if(-not $State.First){$State.First=$State.Pending}
                $State.Last=$State.Pending;$State.Pending=$null;$State.Blocks++
            }
            'WAIT' { }
        }
    } catch {
        Add-DiagError $State $_.Exception.Message
        $State.Pending=$null
    }
}
function Get-DiagVerdict($State) {
    if(-not $State.Header -or $State.Blocks -lt 2){return '数据不足 请检查固件 接线和程序是否继续运行'}
    if($State.ParseErrors -or $State.Resets){return '日志异常或设备重启 需要排查'}
    if($State.Silences){return '本次发生断流 需要排查'}
    $s=$State.Last.Summary
    if($s.clock_errors){return '计时异常 需要排查'}
    foreach($n in $State.Last.Nodes.Values){
        if($n.fail){return '存在传感器读取失败 需要排查'}
        if($n.invalid){return '存在异常数值 需要排查'}
    }
    if($s.overrun -or $s.period_outside -or $s.skew_max_us -gt 10000){return '存在采集时序告警 需要排查'}
    if($s.uart_errors -or $s.report_drops){return '报告传输存在异常 需要排查'}
    if(-not $s.heap_bytes -or -not $s.acq_stack_words -or -not $s.tx_stack_words){return '运行资源余量异常 需要排查'}
    foreach($n in $State.Last.Nodes.Values){
        if($n.sys_cal -ne 3 -or $n.acc_cal -ne 3){return '读取运行正常 但传感器校准尚未完成'}
    }
    if($s.state -ne 'RUN_OK'){return '固件仍报告异常状态 需要排查'}
    return '本次程序自检未发现异常'
}
function Export-DiagReport($State,[string]$Directory,[bool]$Replay) {
    if($State.Pending){Add-DiagError $State '记录结束时最后一个报告块不完整';$State.Pending=$null}
    $verdict=Get-DiagVerdict $State
    $encode={param($value) [System.Net.WebUtility]::HtmlEncode([string]$value)}
    $rows=New-Object 'System.Collections.Generic.List[string]'
    if($State.Last){
        $s=$State.Last.Summary
        foreach($pair in @(@('累计采集轮数',$s.cycles),@('平均周期 微秒',$s.period_avg_us),@('最短周期 微秒',$s.period_min_us),
          @('最长周期 微秒',$s.period_max_us),@('周期超出8至12毫秒次数',$s.period_outside),@('一轮读取最大耗时 微秒',$s.cycle_max_us),
          @('读取窗口中点最大差 微秒',$s.skew_max_us),@('读取耗时达到10毫秒次数',$s.overrun),@('计时错误',$s.clock_errors),
          @('串口发送错误',$s.uart_errors),@('报告快照覆盖次数',$s.report_drops),@('剩余RTOS堆 字节',$s.heap_bytes),
          @('采集任务最小剩余栈 word',$s.acq_stack_words),@('报告任务最小剩余栈 word',$s.tx_stack_words))){
            $rows.Add('<tr><td>'+(& $encode $pair[0])+'</td><td>'+(& $encode $pair[1])+'</td></tr>')
        }
    }
    $nodes=New-Object 'System.Collections.Generic.List[string]'
    if($State.Last){foreach($id in @(0,1,2)){if($State.Last.Nodes.ContainsKey($id)){
        $n=$State.Last.Nodes[$id];$label=@('C0','C4','C7')[$id]
        $values=@($label,$n.ok,$n.fail,$n.invalid,($n.sys_cal.ToString()+'/'+$n.acc_cal),$n.read_max_us,$n.result)
        $cells=($values | ForEach-Object {'<td>'+(& $encode $_)+'</td>'}) -join ''
        $nodes.Add('<tr>'+$cells+'</tr>')
    }}}
    $source=if($Replay){'已有日志解析 不是本次连接样机实测'}else{'本次串口接收 来自声明为 HARDWARE 的专用诊断固件'}
    $mode=if($State.Header){$State.Header.mode}else{'未知'}
    $errors=($State.Errors | ForEach-Object {'<li>'+(& $encode $_)+'</li>'}) -join ''
    $html=@"
<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>IMU程序自检报告</title>
<style>body{font-family:Segoe UI,Microsoft YaHei,sans-serif;max-width:1000px;margin:36px auto;padding:0 20px;line-height:1.7;color:#1d2933}h1{font-size:27px}h2{font-size:19px;margin-top:28px}table{border-collapse:collapse;width:100%;margin:14px 0}td,th{border:1px solid #d9d9d9;padding:9px 12px;text-align:left}th{background:#e4edf4}.result{font-size:21px;font-weight:600}</style>
<h1>IMU程序自检报告</h1><p class="result">$(& $encode $verdict)</p>
<p>来源：$(& $encode $source)<br>模式：$(& $encode $mode)；完整报告块：$($State.Blocks)；日志错误：$($State.ParseErrors)；断流次数：$($State.Silences)</p>
<p>本报告检查读取与程序运行情况。时间来自MCU自己的TIM3，不是独立计时精度证明；读取窗口时间不是IMU内部采样时间。采样同步、安装方向和角度精度未由本报告验证。</p>
<h2>读取与时间统计</h2><p>统计从本次固件启动、传感器初始化完成后的首轮采集累计。出现过的读取错误与时序告警会保留；当前校准状态单独显示。1毫秒等于1000微秒。</p>
<table><tr><th>项目</th><th>结果</th></tr>$($rows -join '')</table>
<h2>节点结果</h2><table><tr><th>节点</th><th>成功次数</th><th>失败次数</th><th>异常数值次数</th><th>系统校准和加速度校准</th><th>最长读取 微秒</th><th>最后驱动结果</th></tr>$($nodes -join '')</table>
<p>驱动结果0表示该次读取成功；校准3/3表示芯片上报两项校准完成，不代表测量精度已经验收。1 word等于4字节；栈余量只是已运行到的路径观察值。</p>
<h2>下一步</h2><p>数据不足：核对专用固件、COM口和继续运行。读取失败：查看节点与最后驱动结果。校准未完成：先按芯片校准流程处理。时序告警：保留日志核对总线耗时与调度，不降低门限掩盖问题。</p>
<ul>$errors</ul><p>同目录 log.txt 是原始文本，result.json 是结构化结果。此报告没有修改设备参数或发送控制命令。</p></html>
"@
    [IO.File]::WriteAllText((Join-Path $Directory 'report.html'),$html,(New-Object Text.UTF8Encoding($false)))
    $serialSummary=if($State.Last){$State.Last.Summary}else{$null}
    $serialNodes=@(if($State.Last){$State.Last.Nodes.GetEnumerator() | Sort-Object Name | ForEach-Object {$_.Value}})
    [ordered]@{verdict=$verdict;replay=$Replay;mode=$mode;complete_blocks=$State.Blocks;parse_errors=$State.ParseErrors;
        silences=$State.Silences;summary=$serialSummary;nodes=$serialNodes;errors=@($State.Errors)} |
        ConvertTo-Json -Depth 8 | Out-File -LiteralPath (Join-Path $Directory 'result.json') -Encoding UTF8
    return $verdict
}
