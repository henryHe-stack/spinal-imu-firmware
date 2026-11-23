// Reference the packaged workstation EXE; run alongside its DLLs and UI.
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Windows.Automation;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Web.WebView2.WinForms;

internal static class WindowCheck
{
    [DllImport("user32.dll",CharSet=CharSet.Unicode)]
    private static extern IntPtr FindWindow(string cls,string title);
    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr handle,out int pid);
    private static int result=1;
    [STAThread] private static int Main(string[] args)
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        var form=(Form)Activator.CreateInstance(typeof(NeckF103.Dashboard).Assembly.GetType("DesktopHost",true),true);
        form.Shown+=async delegate {
            var tabs=(TabControl)form.Controls[0];
            var web=tabs.TabPages[0].Controls.OfType<WebView2>().Single();
            try {
                for(int i=0;i<80 && (web.CoreWebView2==null || web.Source==null || web.Source.Host!="spinal-imu.invalid");i++) await Task.Delay(200);
                await Until(web,"document.querySelector('#demoButton') && document.readyState==='complete'");
                await web.ExecuteScriptAsync("document.querySelector('#demoButton').click()");
                await Until(web,"Number(document.querySelector('#historyCount').textContent)>3");
                Console.WriteLine("PASS: embedded model received demo frames");
                await web.ExecuteScriptAsync("document.querySelector('#tab-curves').click()");
                await Until(web,"!document.querySelector('#window-curves').hidden");
                await web.ExecuteScriptAsync("document.querySelector('#tab-diagnostics').click()");
                await Until(web,"!document.querySelector('#window-diagnostics').hidden");
                await web.ExecuteScriptAsync("document.querySelector('#tab-pose').click();document.querySelector('#timelineTicks button').click()");
                await Until(web,"document.querySelector('#timelineMode').textContent==='历史'");
                await web.ExecuteScriptAsync("document.querySelector('#liveButton').click();document.querySelector('#demoButton').click()");
                Console.WriteLine("PASS: curves, diagnostics, history and live controls");
                bool opened=false;
                using(var timer=new Timer { Interval=100 }) {
                    timer.Tick+=delegate {
                        var dialog=Application.OpenForms.Cast<Form>().FirstOrDefault(f=>f.Text=="选择设备串口");
                        if(dialog!=null){ opened=true; dialog.DialogResult=DialogResult.Cancel; }
                    };
                    timer.Start();
                    await web.ExecuteScriptAsync("document.querySelector('#connectButton').click()");
                    for(int i=0;i<50 && !opened;i++) await Task.Delay(100);
                }
                if(!opened) throw new InvalidOperationException("native COM picker absent");
                await Until(web,"!document.querySelector('#connectButton').disabled");
                Console.WriteLine("PASS: Windows COM picker and cancellation");
                await CheckCsv(web);
                await CheckMeasurement(tabs,web,args);
                result=0;
            } catch(Exception e){Console.Error.WriteLine(e);}
            finally { form.Close(); }
        };
        Application.Run(form);return result;
    }
    private static async Task CheckMeasurement(TabControl tabs,WebView2 web,string[] args)
    {
        tabs.SelectedIndex=1;
        var dashboard=tabs.SelectedTab.Controls.OfType<NeckF103.Dashboard>().Single();
        var engine=dashboard.Engine;
        var source=(ComboBox)typeof(NeckF103.Dashboard).GetField("source",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(dashboard);
        var start=(Button)typeof(NeckF103.Dashboard).GetField("start",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(dashboard);
        source.SelectedIndex=1; start.PerformClick();
        for(int i=0;i<50 && engine.Pipe.Snapshot(engine.Now).C4==null;i++) await Task.Delay(100);
        if(engine.Pipe.Snapshot(engine.Now).C4==null) throw new IOException("single C4 simulation");
        var recording=engine.Pipe.StartCapture(Path.Combine(Path.GetTempPath(),"SpinalImuWindowQA"),engine.Now);
        await Task.Delay(350); engine.Pipe.StopCapture(); if(!recording.Wait(5000) || !recording.Completed) throw new IOException("capture");
        engine.Stop();engine.StartReplay(recording.DirectoryPath); await Task.Delay(450);engine.Stop();
        engine.StartReplay(Path.Combine(engine.TaskRoot,"data","示例数据","三节点_65秒演示.bin"));
        engine.Seek(3000000);await Task.Delay(300);
        if(engine.Pipe.Snapshot(engine.Now).Relative==null) throw new IOException("three-node result replay");
        engine.Stop();Console.WriteLine("PASS: embedded 07 single-node source, recording and both BIN replays");
        if(args.Length!=1) throw new ArgumentException("Supply RAW capture directory");
        engine.StartReplay(args[0]); engine.Seek(engine.Plan.Duration);tabs.SelectedIndex=0;
        await Until(web,"Number(document.querySelector('#historyCount').textContent)>=190");
        if(engine.Pipe.Snapshot(engine.Now).Relative==null) throw new IOException("missing simultaneous relative measurement");
        Console.WriteLine("PASS: RAW replay feeds formal solver and 3D history within one process");
        engine.Stop();
        await Task.Delay(160);
        var serial=new RecordedSerial(args[0]);
        var factory=new Func<string,int,NeckF103.ISerialConnection>((port,baud)=>serial);
        typeof(NeckF103.DataEngine).GetField("serialFactory",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(engine,factory);
        tabs.SelectedIndex=1; source.SelectedIndex=2;
        ((ComboBox)typeof(NeckF103.Dashboard).GetField("port",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(dashboard)).Text="QA_D0";
        start.PerformClick(); tabs.SelectedIndex=0;
        await Until(web,"document.querySelector('#connectionText').textContent==='共用测量串口' && Number(document.querySelector('#historyCount').textContent)>=190");
        if(engine.Pipe.Snapshot(engine.Now).C4==null || serial.Opens!=1) throw new IOException("shared COM source");
        await web.ExecuteScriptAsync("document.querySelector('#connectButton').click()");
        await Task.Delay(100);
        if(tabs.SelectedIndex!=1 || !engine.Active || serial.Opens!=1) throw new IOException("tab handoff disturbed acquisition");
        engine.Stop(); Console.WriteLine("PASS: one fake serial connection feeds both tabs; navigation keeps acquisition active");
    }
    private sealed class RecordedSerial : NeckF103.ISerialConnection
    {
        private readonly MemoryStream data;
        public int Opens;
        public RecordedSerial(string dir) {
            data=new MemoryStream(File.ReadAllBytes(Path.Combine(dir,"bootstrap_session.bin"))
                .Concat(File.ReadAllBytes(Path.Combine(dir,"capture.bin"))).ToArray());
        }
        public void Open(){Opens++;}
        public void DiscardInput(){}
        public int Read(byte[] buf){
            System.Threading.Thread.Sleep(12);
            int count=data.Read(buf,0,Math.Min(buf.Length,1024));
            if(count==0) throw new TimeoutException();
            return count;
        }
        public void Write(byte[] buf){}
        public void Close(){}
        public void Dispose(){}
    }
    private static async Task CheckCsv(WebView2 web)
    {
        web.CoreWebView2.WebMessageReceived += (s,e)=> {
            var m=e.TryGetWebMessageAsString();
            Console.WriteLine("export bridge: "+m.Substring(0,Math.Min(24,m.Length)));
        };
        string target=Path.Combine(Path.GetTempPath(),"spinal-csv-check-"+Guid.NewGuid().ToString("N")+".csv");
        int pid=Process.GetCurrentProcess().Id;
        var fill=Task.Run(async delegate {
            for(int i=0;i<60;i++) {
                var hwnd=FindWindow("#32770","保存姿态数据");
                int owner=0;
                if(hwnd!=IntPtr.Zero) GetWindowThreadProcessId(hwnd,out owner);
                var dialog=owner==pid?AutomationElement.FromHandle(hwnd):null;
                if(dialog!=null) {
                    var box=dialog.FindFirst(TreeScope.Descendants,new PropertyCondition(AutomationElement.AutomationIdProperty,"1001"));
                    var ok=dialog.FindFirst(TreeScope.Descendants,new PropertyCondition(AutomationElement.AutomationIdProperty,"1"));
                    if(box!=null && ok!=null) {
                        ((ValuePattern)box.GetCurrentPattern(ValuePattern.Pattern)).SetValue(target);
                        ((InvokePattern)ok.GetCurrentPattern(InvokePattern.Pattern)).Invoke();return;
                    }
                    if(i==20) foreach(AutomationElement child in dialog.FindAll(TreeScope.Descendants,Condition.TrueCondition))
                        Console.WriteLine("dialog control: "+child.Current.AutomationId+" "+child.Current.ControlType.ProgrammaticName);
                }
                if(i==30) foreach(AutomationElement w in AutomationElement.RootElement.FindAll(TreeScope.Children,new PropertyCondition(AutomationElement.ProcessIdProperty,pid)))
                    Console.WriteLine("app window: "+w.Current.Name);
                await Task.Delay(200);
            }
            throw new TimeoutException("Windows CSV save dialog");
        });
        await web.ExecuteScriptAsync("document.querySelector('#exportButton').click()");
        await fill;
        for(int i=0;i<50 && !File.Exists(target);i++) await Task.Delay(100);
        var lines=File.ReadAllLines(target);
        if(lines.Length<4 || !lines[0].StartsWith("received_at,sequence,node,")) throw new IOException("CSV content");
        File.Delete(target);
        Console.WriteLine("PASS: CSV saved from native dialog, "+lines.Length+" rows including header");
    }
    private static async Task Until(WebView2 web,string predicate)
    {
        for(int i=0;i<75;i++) {
            if((await web.ExecuteScriptAsync("Boolean("+predicate+")"))=="true") return;
            await Task.Delay(200);
        }
        throw new TimeoutException(predicate+"; "+await web.ExecuteScriptAsync("JSON.stringify({url:location.href,title:document.title,webgl:!!document.createElement('canvas').getContext('webgl2'),buttons:document.querySelectorAll('button').length,model:document.querySelector('#modelState')?.textContent})"));
    }
}
