using System;
using System.IO;
using NeckF103;

internal static class MeasurementFiles
{
    internal static string Root(string build)
    {
        var parent = new DirectoryInfo(build).Parent;
        if(parent != null && Directory.Exists(Path.Combine(parent.FullName,"src","pc_f103_dashboard"))) {
            string repositoryRoot=parent.FullName;
            string repositorySamples=Path.Combine(repositoryRoot,"data","示例数据");
            Directory.CreateDirectory(repositorySamples);
            foreach(string file in Directory.GetFiles(Path.Combine(build,"samples"),"*.bin")) {
                string dest=Path.Combine(repositorySamples,Path.GetFileName(file));
                if(!File.Exists(dest)) File.Copy(file,dest);
            }
            return repositoryRoot;
        }
        if(parent != null && Directory.Exists(Path.Combine(parent.FullName,"pc_f103_dashboard"))
            && Directory.Exists(Path.Combine(parent.FullName,"data"))) return parent.FullName;
        string root=Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),"SpinalImuDesktop","Measurement");
        string samples=Path.Combine(root,"data","示例数据");
        Directory.CreateDirectory(samples);
        foreach(string file in Directory.GetFiles(Path.Combine(build,"samples"),"*.bin")) {
            string dest=Path.Combine(samples,Path.GetFileName(file));
            if(!File.Exists(dest)) File.Copy(file,dest);
        }
        return root;
    }
}

// The measurement engine owns the device, recorder and replay clock. Slow rendering
// can discard preview bytes independently of the measurement pipeline.
internal sealed class MeasurementDisplay
{
    private readonly object gate=new object();
    private readonly MemoryStream pending=new MemoryStream();
    private bool reset=true;
    private string shown="";
    internal void Refresh() { lock(gate) reset=true; }
    internal void Append(string kind,byte[] bytes,int count)
    {
        lock(gate) {
            if(bytes==null || pending.Length+count>256*1024) { pending.SetLength(0); reset=true; }
            if(bytes!=null && (kind=="SERIAL" || kind=="REPLAY")) pending.Write(bytes,0,count);
        }
    }
    internal void Flush(DataEngine engine,Action<string> send)
    {
        string kind=engine.Active ? engine.Kind : "";
        byte[] bytes;
        bool restart;
        lock(gate) {
            restart=reset || shown!=kind; reset=false;
            if(kind.Length==0) {
                pending.SetLength(0);
                if(shown.Length!=0) send("measure-end");
                shown="";return;
            }
            bytes=pending.Length==0?null:pending.ToArray(); pending.SetLength(0);
            shown=kind;
        }
        if(restart) send("measure-start|"+kind);
        if(bytes!=null) send("measure-data|"+Convert.ToBase64String(bytes));
    }
}
