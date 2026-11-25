using System;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows.Forms;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;
using NeckF103;

internal sealed class DesktopHost : Form
{
    private const string Origin = "https://spinal-imu.invalid/";
    private readonly WebView2 view = new WebView2 { Dock = DockStyle.Fill };
    private readonly Timer serialTimer = new Timer { Interval = 20 };
    private readonly byte[] serialBuffer = new byte[16384];
    private SerialPort serial;
    private string connection;
    private bool choosing;
    private bool saving;
    private readonly TabControl tabs=new TabControl { Dock=DockStyle.Fill };
    private readonly DataEngine engine;
    private readonly Dashboard measurement;
    private readonly MeasurementDisplay display=new MeasurementDisplay();
    private bool pageReady;

    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new DesktopHost());
    }

    private DesktopHost()
    {
        Text = "颈部姿态三维工作站";
        ClientSize = new Size(1360, 880);
        MinimumSize = new Size(820, 580);
        StartPosition = FormStartPosition.CenterScreen;
        WindowState = FormWindowState.Maximized;
        string build=AppDomain.CurrentDomain.BaseDirectory;
        engine=new DataEngine(MeasurementFiles.Root(build),build);
        engine.BeforeStart=delegate {
            if(serial!=null) throw new InvalidOperationException("请在三维页断开预览串口，再启动正式测量。");
        };
        engine.Pipe.DisplayBytes=delegate(byte[] data,int count) {
            display.Append(engine.Kind,data,count);
        };
        measurement=new Dashboard(engine,false) {TopLevel=false,FormBorderStyle=FormBorderStyle.None,Dock=DockStyle.Fill};
        var threeTab=new TabPage("三维观察"); threeTab.Controls.Add(view);
        var measureTab=new TabPage("正式测量与记录") { AutoScroll=true, AutoScrollMinSize=measurement.MinimumSize };
        measureTab.Controls.Add(measurement);
        tabs.TabPages.Add(threeTab); tabs.TabPages.Add(measureTab); Controls.Add(tabs);
        measurement.Show();
        Shown += Initialize;
        serialTimer.Interval=40;
        serialTimer.Tick += delegate { ReadAvailable(null,EventArgs.Empty); if(pageReady) display.Flush(engine,Send); };
        serialTimer.Start();
        FormClosing += delegate(object sender,FormClosingEventArgs e) {
            try { ClosePort(); engine.Dispose(); measurement.Close(); }
            catch(Exception ex) { e.Cancel=true; MessageBox.Show(this,ex.Message,Text); }
        };
        FormClosed += delegate { serialTimer.Dispose(); };
    }

    private async void Initialize(object sender, EventArgs args)
    {
        try {
            string ui = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "ui");
            if (!File.Exists(Path.Combine(ui, "index.html"))) throw new FileNotFoundException("缺少 ui 文件夹，请将程序与配套文件放在一起。");
            string profile = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "SpinalImuDesktop", "WebView2");
            var environment = await CoreWebView2Environment.CreateAsync(null, profile);
            await view.EnsureCoreWebView2Async(environment);
            var web = view.CoreWebView2;
            web.SetVirtualHostNameToFolderMapping("spinal-imu.invalid", ui, CoreWebView2HostResourceAccessKind.DenyCors);
            web.Settings.IsStatusBarEnabled = false;
            web.Settings.AreHostObjectsAllowed = false;
            web.NavigationStarting += (s, e) => { e.Cancel = !IsAppPage(e.Uri); };
            web.NewWindowRequested += (s, e) => { e.Handled = true; };
            web.WebMessageReceived += Receive;
            web.Navigate(Origin + "index.html");
        } catch (Exception ex) {
            if (!IsDisposed) { tabs.SelectedIndex=1; MessageBox.Show(this, "三维页加载失败，正式测量可继续使用。\n" + ex.Message, Text); }
        }
    }

    private static bool IsAppPage(string address)
    {
        Uri uri;
        return Uri.TryCreate(address, UriKind.Absolute, out uri)
            && uri.GetLeftPart(UriPartial.Path) == Origin + "index.html";
    }

    private void Receive(object sender, CoreWebView2WebMessageReceivedEventArgs e)
    {
        if (!IsAppPage(e.Source)) return;
        string message;
        try { message = e.TryGetWebMessageAsString(); } catch (ArgumentException) { return; }
        if (String.IsNullOrEmpty(message)) return;
        if (message=="measurement-ready") { pageReady=true; display.Refresh(); return; }
        if (message=="measurement-tab") { tabs.SelectedIndex=1; return; }
        if (message.StartsWith("csv\nreceived_at,sequence,node,",StringComparison.Ordinal)) {
            if (!saving) {
                saving=true;
                BeginInvoke((Action)delegate { SaveCsv(message.Substring(4)); });
            }
            return;
        }
        string[] parts = message.Split('|');
        uint id;
        if (parts.Length < 2 || !uint.TryParse(parts[1], out id)) return;
        if (parts[0] == "close" && parts[1] == connection) {
            ClosePort(); Send("closed|" + parts[1]); return;
        }
        int baud;
        if (parts.Length != 3 || parts[0] != "open" || !int.TryParse(parts[2], out baud)
            || !new[] {115200,230400,460800,921600}.Contains(baud)) return;
        if (serial != null || choosing || engine.Active) { Send("error|" + parts[1] + "|请先在正式测量页停止当前数据源"); return; }
        choosing = true;
        BeginInvoke((Action)delegate { OpenPort(parts[1], baud); });
    }

    private void OpenPort(string id, int baud)
    {
        try {
            using (var dialog = new Form {Text="选择设备串口", ClientSize=new Size(420,112),
                FormBorderStyle=FormBorderStyle.FixedDialog, MinimizeBox=false, MaximizeBox=false,
                StartPosition=FormStartPosition.CenterParent, ShowInTaskbar=false}) {
                var choice = new ComboBox {Left=16,Top=42,Width=175,DropDownStyle=ComboBoxStyle.DropDownList};
                var refresh = new Button {Text="刷新",Left=199,Top=40};
                var ok = new Button {Text="连接",Left=293,Top=40,DialogResult=DialogResult.OK};
                var previewOnly=new CheckBox {Text="仅预览 A5 5A / NDJSON / CSV",AutoSize=true,Left=16,Top=77,Checked=baud!=115200 && baud!=230400,Enabled=baud==115200 || baud==230400};
                Action scan = delegate {
                    var prior = choice.SelectedItem;
                    choice.Items.Clear(); choice.Items.AddRange(SerialPort.GetPortNames().OrderBy(x=>x).ToArray());
                    if (prior!=null && choice.Items.Contains(prior)) choice.SelectedItem=prior;
                    else if (choice.Items.Count>0) choice.SelectedIndex=0;
                    ok.Enabled=choice.SelectedIndex>=0;
                };
                refresh.Click += delegate { scan(); };
                dialog.Controls.AddRange(new Control[]{new Label {Text="请选择 COM 口。默认连接 D0 正式测量。",AutoSize=true,Top=12,Left=16}, choice,refresh,ok,previewOnly});
                dialog.AcceptButton=ok; scan();
                if (dialog.ShowDialog(this)!=DialogResult.OK || choice.SelectedItem==null) { Send("cancel|"+id); return; }
                if(!previewOnly.Checked) {
                    measurement.ConnectDevice(choice.SelectedItem.ToString(),baud);
                    Send("cancel|"+id); display.Refresh(); return;
                }
                serial = new SerialPort(choice.SelectedItem.ToString(),baud,Parity.None,8,StopBits.One);
                serial.ReadTimeout=50; serial.ReadBufferSize=65536;
                serial.Open(); connection=id;
                Send("opened|"+id); serialTimer.Start();
            }
        } catch(Exception ex) { ClosePort(); Send("error|"+id+"|"+ex.Message); }
        finally { choosing=false; }
    }

    private void ReadAvailable(object sender, EventArgs e)
    {
        if (serial==null) return;
        try {
            int count=Math.Min(serial.BytesToRead,serialBuffer.Length);
            if (count>0) {
                int n=serial.Read(serialBuffer,0,count);
                if(n>0) Send("data|"+connection+"|"+Convert.ToBase64String(serialBuffer,0,n));
            }
        } catch(TimeoutException) { }
        catch(Exception ex) { string id=connection; ClosePort(); Send("error|"+id+"|"+ex.Message); }
    }

    private void ClosePort()
    {
        connection=null;
        var port=serial; serial=null;
        if(port!=null) { try { port.Dispose(); } catch(IOException) { } }
    }

    private void Send(string message)
    {
        if(!IsDisposed && !Disposing && view.CoreWebView2!=null) view.CoreWebView2.PostWebMessageAsString(message);
    }

    private void SaveCsv(string csv)
    {
        string temp=null;
        try {
            using(var dialog=new SaveFileDialog {Title="保存姿态数据",Filter="CSV 文件 (*.csv)|*.csv",
                DefaultExt="csv",FileName="spinal_imu_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".csv",OverwritePrompt=true}) {
                if(dialog.ShowDialog(this)!=DialogResult.OK) return;
                string target=dialog.FileName;
                temp=Path.Combine(Path.GetDirectoryName(target),Guid.NewGuid().ToString("N")+".tmp");
                File.WriteAllText(temp,csv,new System.Text.UTF8Encoding(true));
                if(File.Exists(target)) File.Replace(temp,target,null); else File.Move(temp,target);
                temp=null;
            }
        } catch(Exception ex) { MessageBox.Show(this,"保存 CSV 失败。\n"+ex.Message,Text); }
        finally { saving=false; if(temp!=null && File.Exists(temp)) File.Delete(temp); }
    }
}
