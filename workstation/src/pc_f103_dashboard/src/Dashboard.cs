using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;

namespace NeckF103
{
    public sealed class TracePanel : Control
    {
        private static readonly Color[] Colors={Color.FromArgb(105,145,180),Color.FromArgb(188,158,100),Color.FromArgb(34,161,209),Color.FromArgb(227,150,51),Color.FromArgb(20,199,135),Color.FromArgb(235,99,154)};
        public string Heading="",Detail="等待数据"; public string[] Labels=new string[0];
        public PlotSample[] Samples=new PlotSample[0]; public int StartIndex,Count=6; public double EndTime,MaxGap=.125;
        public bool Current=false;
        public TracePanel() { DoubleBuffered=true; BackColor=Color.FromArgb(24,34,50); ForeColor=Color.FromArgb(224,232,245); Dock=DockStyle.Fill; }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e); Graphics g=e.Graphics; g.SmoothingMode=SmoothingMode.AntiAlias;
            using(Font title=new Font("Microsoft YaHei UI",10,FontStyle.Bold)) g.DrawString(Heading,title,Brushes.White,12,8);
            using(Brush b=new SolidBrush(Color.FromArgb(162,180,202))) g.DrawString(Detail,Font,b,12,29);
            Rectangle area=new Rectangle(46,91,Math.Max(1,Width-62),Math.Max(1,Height-119));
            double start=Math.Max(0,EndTime-30),end=Math.Max(start+5,EndTime),range=60;
            for(int i=0;i<Samples.Length;i++) for(int j=0;j<Count;j++) { double v=Samples[i].Values[StartIndex+j]; if(!double.IsNaN(v)) range=Math.Max(range,Math.Ceiling(Math.Abs(v)/30)*30); }
            range=Math.Min(360,range);
            using(Pen grid=new Pen(Color.FromArgb(49,65,87))) using(Brush dim=new SolidBrush(Color.FromArgb(151,169,193)))
            {
                for(int i=0;i<5;i++) { float y=area.Top+area.Height*i/4f; g.DrawLine(grid,area.Left,y,area.Right,y);
                    g.DrawString((range-range*i/2).ToString("0",CultureInfo.InvariantCulture),Font,dim,3,y-7); }
                for(int i=0;i<=3;i++) { float x=area.Left+area.Width*i/3f; g.DrawLine(grid,x,area.Top,x,area.Bottom);
                    g.DrawString((start+(end-start)*i/3).ToString("F1",CultureInfo.InvariantCulture)+"s",Font,dim,x-16,area.Bottom+5); }
            }
            GraphicsState prior=g.Save(); g.SetClip(area);
            for(int j=0;j<Count;j++)
            {
                bool have=false; PointF previous=new PointF(); double priorTime=0;
                using(Pen pen=new Pen(Colors[StartIndex+j],1.8f))
                {
                    for(int i=0;i<Samples.Length;i++)
                    {
                        PlotSample s=Samples[i]; double value=s.Values[StartIndex+j];
                        if(double.IsNaN(value) || s.Time<start || s.Time>end) { have=false; continue; }
                        PointF point=new PointF(area.Left+(float)((s.Time-start)/(end-start)*area.Width),area.Top+(float)((range-value)/(range*2)*area.Height));
                        if(have && s.Time>=priorTime && s.Time-priorTime<=MaxGap) g.DrawLine(pen,previous,point);
                        previous=point; priorTime=s.Time; have=true;
                    }
                }
            }
            g.Restore(prior);
            for(int j=0;j<Count;j++)
            {
                double v=Samples.Length==0?double.NaN:Samples[Samples.Length-1].Values[StartIndex+j];
                string value=Current && !double.IsNaN(v)?v.ToString("F1",CultureInfo.InvariantCulture)+"°":"—";
                int columns=Count==6?3:3; float x=12+(Width-24)*(j%columns)/(float)columns,y=51+(j/columns)*18;
                using(Brush b=new SolidBrush(Colors[StartIndex+j])) g.DrawString(Labels.Length>j?Labels[j]+" "+value:value,Font,b,x,y);
            }
        }
    }
    public sealed class Dashboard : Form
    {
        public readonly DataEngine Engine; private readonly bool ownsEngine;
        private readonly ComboBox source=new ComboBox(),port=new ComboBox(),basis=new ComboBox(),rate=new ComboBox();
        private readonly ComboBox baud=new ComboBox();
        private readonly CheckBox script=new CheckBox(),pause=new CheckBox();
        private readonly Button start=new Button(),stop=new Button(),record=new Button();
        private readonly Label banner=new Label(),session=new Label(),alarms=new Label(),link=new Label(),replayInfo=new Label();
        private readonly TextBox filePath=new TextBox(),recordPath=new TextBox(),journal=new TextBox();
        private readonly TrackBar seek=new TrackBar(); private readonly ListView nodes=new ListView();
        private readonly TracePanel[] charts=new TracePanel[5]; private readonly Button[] simButtons=new Button[8];
        private readonly System.Windows.Forms.Timer timer=new System.Windows.Forms.Timer(); private string lastJournal=""; private bool updatingSeek;
        public Dashboard(DataEngine engine,bool owns)
        {
            Engine=engine; ownsEngine=owns; Text="F103C8 颈部监测 · 实时显示与记录回放"; ClientSize=new Size(1400,960); MinimumSize=new Size(1170,850);
            Font=new Font("Microsoft YaHei UI",9); BackColor=Color.FromArgb(237,241,246); AutoScaleMode=AutoScaleMode.None;
            TableLayoutPanel layout=new TableLayoutPanel { Dock=DockStyle.Fill,ColumnCount=1,RowCount=8,Padding=new Padding(12) };
            float[] rows={49,38,38,37,33,0,135,35};
            for(int i=0;i<rows.Length;i++) layout.RowStyles.Add(rows[i]==0?new RowStyle(SizeType.Percent,100):new RowStyle(SizeType.Absolute,rows[i]));
            Controls.Add(layout);
            banner.Dock=DockStyle.Fill; banner.Font=new Font(Font.FontFamily,14,FontStyle.Bold); banner.TextAlign=ContentAlignment.MiddleLeft;
            banner.ForeColor=Color.FromArgb(143,86,14); banner.Text="未启动 · 请先选择电脑模拟数据源"; layout.Controls.Add(banner,0,0);
            FlowLayoutPanel sources=Row(); source.DropDownStyle=ComboBoxStyle.DropDownList; source.Width=210;
            source.Items.AddRange(new object[] {"SIM 三节点（共享C算法）","SIM 单节点（仅C4）","USB 串口 D0","记录 / BIN 回放","HC-05 蓝牙串口 D0"}); source.SelectedIndex=0;
            script.Text="SIM演示脚本：归零 + 故障"; script.Checked=true; script.Width=220; script.Padding=new Padding(0,4,0,0);
            start.Text="启动"; stop.Text="停止"; start.Click+=delegate { Try(StartSource); }; stop.Click+=delegate { Try(Engine.Stop); RefreshView(); };
            port.Width=100; port.DropDownStyle=ComboBoxStyle.DropDown; RefreshPorts(); Button refresh=Button("刷新COM",RefreshPorts);
            baud.DropDownStyle=ComboBoxStyle.DropDownList; baud.Width=86; baud.Items.AddRange(new object[]{"115200","230400"}); baud.SelectedIndex=0;
            sources.Controls.AddRange(new Control[] {source,script,start,stop,port,refresh,
                Button("板端控制…",delegate { using(DeviceControlForm form=new DeviceControlForm(Engine)) form.ShowDialog(this); }),
                baud}); layout.Controls.Add(sources,0,1);
            FlowLayoutPanel actions=Row(); string[] names={"SIM请求归零","SIM取消归零","SIM停输入","SIM恢复输入","SIM停传输","SIM恢复传输"}; string[] commands={"ZERO","CANCEL","INPUT_OFF","INPUT_ON","TX_OFF","TX_ON"};
            for(int i=0;i<6;i++) { string command=commands[i]; simButtons[i]=Button(names[i],delegate { Engine.Command(command); }); simButtons[i].Width=105; actions.Controls.Add(simButtons[i]); }
            simButtons[6]=Button("SIM C4失效",delegate { Engine.Command("FAULT_C4"); }); simButtons[7]=Button("SIM C4恢复",delegate { Engine.Command("RESTORE_C4"); }); actions.Controls.Add(simButtons[6]); actions.Controls.Add(simButtons[7]);
            record.Text="开始记录"; record.Width=95; record.Click+=delegate { Try(ToggleRecord); }; actions.Controls.Add(record); actions.Controls.Add(Button("PC测量配置",delegate { MeasurementProfile.Edit(Engine,this); })); layout.Controls.Add(actions,0,2);
            FlowLayoutPanel playback=Row(); filePath.Width=325; Button browse=Button("选择回放BIN",BrowseReplay); pause.Text="暂停回放"; pause.Width=85;
            pause.CheckedChanged+=delegate { Engine.PauseReplay(pause.Checked); }; rate.DropDownStyle=ComboBoxStyle.DropDownList; rate.Items.AddRange(new object[]{"1×","2×","4×"}); rate.SelectedIndex=0; rate.Width=50;
            rate.SelectedIndexChanged+=delegate { Engine.Speed(rate.SelectedIndex==0?1:rate.SelectedIndex==1?2:4); };
            seek.Width=240; seek.Maximum=1000; seek.TickStyle=TickStyle.None; seek.Height=28; seek.MouseUp+=delegate { if(!updatingSeek && Engine.Plan!=null) Engine.Seek(Engine.Plan.Duration*(ulong)seek.Value/1000); };
            basis.Width=140; basis.DropDownStyle=ComboBoxStyle.DropDownList; basis.Items.AddRange(new object[]{"相对角：原始","相对角：零位修正"}); basis.SelectedIndex=0; basis.SelectedIndexChanged+=delegate { RefreshView(); };
            playback.Controls.AddRange(new Control[]{filePath,browse,pause,rate,seek,basis}); layout.Controls.Add(playback,0,3);
            session.Dock=DockStyle.Fill; session.TextAlign=ContentAlignment.MiddleLeft; layout.Controls.Add(session,0,4);
            TableLayoutPanel plots=new TableLayoutPanel { Dock=DockStyle.Fill,ColumnCount=3,RowCount=3 };
            plots.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,40)); plots.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,30)); plots.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,30));
            plots.RowStyles.Add(new RowStyle(SizeType.Percent,50)); plots.RowStyles.Add(new RowStyle(SizeType.Percent,50)); plots.RowStyles.Add(new RowStyle(SizeType.Absolute,55));
            string[] headings={"C4 重力代理倾角（非通用Euler）","整体 C7 → C0","上段 C4 → C0","下段 C7 → C4","基座 WORLD → C7"};
            for(int i=0;i<5;i++) { charts[i]=new TracePanel {Heading=headings[i],Margin=new Padding(4)}; charts[i].Font=new Font(Font.FontFamily,8.5f); }
            charts[0].Labels=new string[]{"原始前屈","原始侧倾","LP前屈","LP侧倾","修正前屈","修正侧倾"}; plots.Controls.Add(charts[0],0,0);
            charts[1].Labels=charts[2].Labels=charts[3].Labels=charts[4].Labels=new string[]{"前屈","右侧倾","轴向旋转"};
            for(int i=1;i<5;i++) { charts[i].Count=3; charts[i].MaxGap=.25; plots.Controls.Add(charts[i],1+(i-1)%2,(i-1)/2); }
            nodes.Dock=DockStyle.Fill; nodes.View=View.Details; nodes.FullRowSelect=true; nodes.GridLines=true; nodes.Columns.Add("节点",45); nodes.Columns.Add("加速度/姿态",98); nodes.Columns.Add("校准A/S",64); nodes.Columns.Add("质量码",78); nodes.Columns.Add("零位 / ref",90);
            TableLayoutPanel status=new TableLayoutPanel {Dock=DockStyle.Fill,ColumnCount=1,RowCount=2}; status.RowStyles.Add(new RowStyle(SizeType.Absolute,60)); status.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            alarms.Dock=DockStyle.Fill; alarms.TextAlign=ContentAlignment.MiddleLeft; alarms.Font=new Font(Font.FontFamily,10,FontStyle.Bold); status.Controls.Add(alarms,0,0); status.Controls.Add(nodes,0,1); plots.Controls.Add(status,0,1);
            link.Dock=DockStyle.Fill; link.TextAlign=ContentAlignment.MiddleLeft; plots.Controls.Add(link,0,2); plots.SetColumnSpan(link,3); layout.Controls.Add(plots,0,5);
            journal.Dock=DockStyle.Fill; journal.Multiline=true; journal.ReadOnly=true; journal.ScrollBars=ScrollBars.Vertical; journal.BackColor=Color.White; layout.Controls.Add(journal,0,6);
            FlowLayoutPanel footer=Row(); recordPath.ReadOnly=true; recordPath.Width=810; replayInfo.AutoSize=true; replayInfo.Padding=new Padding(0,5,0,0); footer.Controls.AddRange(new Control[]{recordPath,replayInfo}); layout.Controls.Add(footer,0,7);
            timer.Interval=50; timer.Tick+=delegate { Try(RefreshView); }; Shown+=delegate { timer.Start(); };
            FormClosing+=delegate(object sender,FormClosingEventArgs e) { timer.Stop(); if(ownsEngine) { try { Engine.Dispose(); } catch(Exception ex) { e.Cancel=true; timer.Start(); MessageBox.Show(this,ex.Message,"无法安全停止"); } } };
            FormClosed+=delegate { timer.Dispose(); }; RefreshView();
        }
        private FlowLayoutPanel Row() { return new FlowLayoutPanel {Dock=DockStyle.Fill,WrapContents=false,Margin=Padding.Empty}; }
        private Button Button(string name,Action action) { Button b=new Button {Text=name,AutoSize=true,Height=29}; b.Click+=delegate { Try(action); }; return b; }
        private void Try(Action action) { try { action(); } catch(Exception e) { MessageBox.Show(this,e.Message,"F103C8",MessageBoxButtons.OK,MessageBoxIcon.Warning); } }
        private void RefreshPorts() { string selected=port.Text; port.Items.Clear(); foreach(string name in SerialPort.GetPortNames()) port.Items.Add(name); port.Text=selected; }
        private void BrowseReplay()
        {
            using(OpenFileDialog dialog=new OpenFileDialog {Title="选择自己的capture.bin，或示例数据中的演示BIN",Filter="D0二进制 (*.bin)|*.bin",InitialDirectory=Path.Combine(Engine.TaskRoot,"data","示例数据")})
            { if(dialog.ShowDialog(this)==DialogResult.OK) { filePath.Text=dialog.FileName; source.SelectedIndex=3; } }
        }
        internal void ConnectDevice(string name,int speed)
        {
            Engine.StartSerial(name,speed); source.SelectedIndex=2; port.Text=name;
            baud.SelectedItem=speed.ToString(); RefreshView();
        }
        private void StartSource()
        {
            if(source.SelectedIndex<2) Engine.StartSimulation(source.SelectedIndex==1,script.Checked);
            else if(source.SelectedIndex==2 || source.SelectedIndex==4) { Engine.StartSerial(port.Text.Trim(),int.Parse(baud.Text),source.SelectedIndex==4); }
            else { if(filePath.Text.Length==0) throw new IOException("请先选择回放BIN"); Engine.StartReplay(filePath.Text); pause.Checked=false; rate.SelectedIndex=0; }
            RefreshView();
        }
        private void ToggleRecord()
        { if(Engine.Pipe.Recorder!=null) Engine.Pipe.StopCapture(); else { if(!Engine.Active || Engine.Kind=="REPLAY" || Engine.Pipe.Snapshot(Engine.Now).Session==null) throw new IOException("只可记录已登记的实时数据源，回放不能二次记录"); Engine.Pipe.StartCapture(Engine.TaskRoot,Engine.Now); } RefreshView(); }
        public static string Level(int value) { return value==1?"正常":value==2?"警告":value==3?"危险":value==4?"后伸类别":"UNKNOWN"; }
        private static string Zero(int value) { string[] names={"未归零","采集中","等预热","已提交","输入失败","方向不明确","已取消","已超时"}; return value>=0 && value<names.Length?names[value]:value.ToString(); }
        private static string Timer(ulong value) { return (value*1e-6).ToString("F1",CultureInfo.InvariantCulture); }
        public void RefreshView() { Render(Engine.Pipe.Snapshot(Engine.Now),Engine.Active); }
        public void Render(ViewSnapshot s,bool active)
        {
            bool registered=s.Session!=null,linkFresh=active && registered && (s.HostFlags&1)==0;
            bool c4Fresh=linkFresh && (s.HostFlags&2)==0,relativeFresh=linkFresh && (s.HostFlags&4)==0;
            SessionData registration=registered?s.Session.Registration:null;
            string origin=registered?(registration.Source==0?"SIM · 模拟数据（不是样机实测）":"HARDWARE 标签 · 尚需实物核验"):"未登记";
            if(Engine.Kind=="REPLAY") origin="历史回放 / "+origin;
            string profile=Engine.Pipe.Profile==null?"":" · "+Engine.Pipe.Profile.Name+(Engine.Pipe.Profile.Registered?" · 读窗中点估时":" · 安装未确认，仅诊断");
            banner.Text=origin+"    |    "+Engine.State+profile+(!linkFresh?" · 当前值不可用":""); banner.ForeColor=registration!=null && registration.Source==0?Color.FromArgb(146,85,9):Color.FromArgb(40,92,142);
            session.Text=registered?"boot="+s.Session.Boot+"  session="+s.Session.Session+"  config="+s.Session.Config+"  profile="+registration.Profile+"  模式="+(registration.Mode==1?"单C4":"三节点")+"  世界系="+registration.World+"  固件=0x"+registration.Firmware.ToString("X8")+"  设备质量超时="+(registration.QualityTimeoutUs/1000)+"ms":"等待有效SESSION；串口连接成功不代表已取得有效姿态";
            for(int i=0;i<5;i++) { charts[i].Samples=s.Plots[i]; charts[i].EndTime=s.Time; charts[i].Current=i==0?c4Fresh:relativeFresh; charts[i].StartIndex=i==0?0:basis.SelectedIndex==1?3:0; }
            C4Data c=s.C4==null?null:s.C4.C4;
            charts[0].Detail=c==null?"等待C4结果": "零位="+Zero(c.Zero)+"  ref="+c.Reference+"  epoch="+c.Epoch+"  mask=0x"+c.Mask.ToString("X2")+(!c4Fresh?" · 历史 / 陈旧":"");
            for(int i=1;i<5;i++) { PairData p=s.Relative==null?null:s.Relative.Relative.Pairs[i-1]; charts[i].Detail=registration!=null && registration.Mode==1?"单C4模式：未提供相对姿态":p==null?"等待相对结果": "ref="+s.Relative.Relative.Reference+"  有效="+p.Mask+"  flags=0x"+p.Flags.ToString("X")+(!relativeFresh?" · 历史 / 陈旧":" · 未设置相对报警"); charts[i].Invalidate(); } charts[0].Invalidate();
            if(c4Fresh && c!=null && c.Axes[0].Valid!=0 && c.Axes[1].Valid!=0)
            {
                AxisData f=c.Axes[0],l=c.Axes[1]; alarms.Text="前屈："+Level(f.Instant)+" / 持续 "+Level(f.Alarm)+"  警/危 "+Timer(f.WarningUs)+"/"+Timer(f.DangerUs)+"s\n侧倾："+Level(l.Instant)+" / 持续 "+Level(l.Alarm)+"  警/危 "+Timer(l.WarningUs)+"/"+Timer(l.DangerUs)+"s";
                alarms.ForeColor=f.Alarm==3 || l.Alarm==3?Color.Firebrick:f.Alarm==4?Color.Purple:f.Alarm==2 || l.Alarm==2?Color.DarkOrange:Color.FromArgb(30,102,65);
            }
            else { alarms.Text="持续检测：UNKNOWN / 当前输入不可用\n失效解除不表示已恢复正常"; alarms.ForeColor=Color.Gray; }
            nodes.BeginUpdate(); nodes.Items.Clear();
            for(int i=0;i<3;i++) { StatusData st=s.Status==null?null:s.Status.Status; NodeData n=st==null?null:st.Nodes[i]; bool excluded=registration!=null && (registration.NodeMask&(1<<i))==0;
                string usable=excluded?"未接入":n==null || !linkFresh?"未知 / 历史":((st.AccelMask&(1<<i))!=0?"可用":"失效")+" / "+((st.QuatMask&(1<<i))!=0?"可用":"失效");
                ListViewItem row=new ListViewItem(new string[]{i==0?"C0":i==1?"C4":"C7",usable,n==null?"—":(n.AccelCal==255?"?":n.AccelCal.ToString())+"/"+(n.SystemCal==255?"?":n.SystemCal.ToString()),n==null?"—":"0x"+n.Quality.ToString("X"),n==null?"—":Zero(n.Zero)+" / "+n.Reference}); nodes.Items.Add(row); } nodes.EndUpdate();
            ulong[] a=s.Stats; string device=s.Status==null?"设备计数未知":"设备输入丢弃="+s.Status.Status.InputDrops+"  遥测丢弃="+s.Status.Status.TelemetryDrops+"  事件丢弃="+s.Status.Status.EventDrops;
            link.Text=(Engine.Kind=="SERIAL"?"本段接收=":"接收累计=")+s.Messages+"  事件="+s.AlarmEvents+"  CRC错误="+a[1]+"  长度/版本/类型/负载="+a[2]+"/"+a[3]+"/"+a[4]+"/"+a[5]+"  TX序号缺口="+a[10]+"  拒收="+a[9]+"  旧epoch="+s.OldEpochDrops+"\nPC门控=0x"+s.HostFlags.ToString("X")+"（链路500ms / 姿态250ms，非端到端延迟）  "+device;
            SerialLinkSnapshot serialState=Engine.SerialStatus;
            if(serialState!=null) link.Text+="\n串口连接="+serialState.Connections+" / 尝试="+serialState.Attempts+"  失败="+serialState.Failures+"  断流="+serialState.Silences+"  累计="+serialState.Bytes+" B  接收="+serialState.BytesPerSecond.ToString("F0")+" B/s / "+serialState.MessagesPerSecond.ToString("F1")+"条D0/s";
            string logs=string.Join(Environment.NewLine,s.Events); if(logs!=lastJournal) { journal.Text=logs; journal.SelectionStart=journal.TextLength; journal.ScrollToCaret(); lastJournal=logs; }
            for(int i=0;i<8;i++) simButtons[i].Enabled=active && Engine.Kind=="SIM";
            record.Text=Engine.Pipe.Recorder==null?"开始记录":"停止记录"; record.Enabled=active && Engine.Kind!="REPLAY" && registered;
            CaptureRecorder r=Engine.Pipe.LastRecorder;
            recordPath.Text=r==null?Path.Combine(Engine.TaskRoot,"data","我的记录"):r.DirectoryPath+"  ["+(r.Error.Length!=0?"失败："+r.Error:r.Completed?"COMPLETE":r.Finished?"已结束":"写入 / 排空中")+"]";
            if(Engine.Plan!=null) { updatingSeek=true; seek.Value=Engine.Plan.Duration==0?0:(int)Math.Min(1000,Engine.Now*1000/Engine.Plan.Duration); updatingSeek=false; replayInfo.Text=Timer(Engine.Now)+" / "+Timer(Engine.Plan.Duration)+"s"; }
            else replayInfo.Text="绘图刷新20Hz · 单位°";
        }
    }
}
