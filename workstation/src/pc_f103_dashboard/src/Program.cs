using System;
using System.IO;
using System.Windows.Forms;

namespace NeckF103
{
    internal static class Program
    {
        [STAThread] private static int Main(string[] args)
        {
            Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
            try
            {
                string build=AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
                if(args.Length==4 && args[0]=="--self-test") return SelfTests.Run(args[1],args[2],args[3],build);
                if(args.Length==4 && args[0]=="--serial-self-test") return D4SerialTests.Run(args[1],args[2],args[3],build);
                if(args.Length==4 && args[0]=="--control-self-test") return D5ControlTests.Run(args[1],args[2],args[3],build);
                if(args.Length!=2 || args[0]!="--root") throw new ArgumentException("请使用独立工作区的启动脚本（需要 --root 参数）");
                string root=Path.GetFullPath(args[1]); if(!build.StartsWith(root.TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)) throw new IOException("程序必须在独立工作区内运行");
                Application.Run(new Dashboard(new DataEngine(root,build),true)); return 0;
            }
            catch(Exception e) { MessageBox.Show(e.ToString(),"F103C8启动失败"); return 1; }
        }
    }
}
