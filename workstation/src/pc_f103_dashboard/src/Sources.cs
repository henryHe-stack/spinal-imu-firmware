using System;
using System.Diagnostics;
using System.IO;
using System.Threading;

namespace NeckF103
{
    public sealed class DataEngine : IDisposable
    {
        public readonly Pipeline Pipe=new Pipeline(); public readonly string TaskRoot,BuildRoot;
        public readonly DeviceControl Device;
        public volatile bool Active; public volatile string State="未启动"; public string Kind="NONE";
        private volatile bool cancel; private Thread worker,stderrWorker; private Process process; private ISerialConnection serial;
        private readonly object serialGate=new object(); private readonly ManualResetEvent serialStop=new ManualResetEvent(false);
        private readonly Func<string,int,ISerialConnection> serialFactory; private readonly int retryMilliseconds;
        private SerialLinkSession serialSession;
        private readonly Stopwatch clock=new Stopwatch(); private long replayTime,seekRequest=-1;
        private readonly object replayGate=new object(); private bool paused; private double speed=1;
        public ReplayPlan Plan; public string LastError="";
        internal Action BeforeStart=null;
        public DataEngine(string taskRoot,string buildRoot)
            : this(taskRoot,buildRoot,delegate(string name,int baud) { return new WindowsSerialConnection(name,baud); },1000) { }
        internal DataEngine(string taskRoot,string buildRoot,Func<string,int,ISerialConnection> factory,int retryMs)
        {
            TaskRoot=taskRoot; BuildRoot=buildRoot; serialFactory=factory; retryMilliseconds=retryMs;
            Device=new DeviceControl(Pipe); Pipe.SegmentReset=Device.Reset;
            Pipe.Accepted=delegate(WireMessage m,ulong now) { if(Kind=="SERIAL") Device.Receive(m,now); };
        }
        public void DeviceCommand(uint opcode,uint key=0,uint value=0)
        { if(Kind!="SERIAL" || !Active) throw new InvalidOperationException("板端命令仅用于运行中的串口来源"); Device.Request(opcode,key,value,Now); }
        public SerialLinkSnapshot SerialStatus { get { SerialLinkSession s=serialSession; return Kind=="SERIAL" && s!=null?s.Snapshot(Now):null; } }
        public static ulong Microseconds(Stopwatch watch)
        { ulong ticks=(ulong)watch.ElapsedTicks,f=(ulong)Stopwatch.Frequency; return ticks/f*1000000+(ticks%f)*1000000/f; }
        public ulong Now { get { return Kind=="REPLAY"?(ulong)Math.Max(0,Interlocked.Read(ref replayTime)):Microseconds(clock); } }
        private void Begin(string kind,MeasurementProfile profile=null)
        { if(BeforeStart!=null) BeforeStart(); Stop(); Kind=kind; Pipe.Profile=profile; Pipe.Reset(); LastError=""; cancel=false; serialStop.Reset(); Active=true; clock.Restart(); Plan=null; serialSession=null; }
        public void StartSimulation(bool single,bool scripted)
        {
            Begin("SIM"); State=single?"SIM 单C4":"SIM 三节点";
            uint boot=unchecked((uint)DateTime.UtcNow.Ticks); if(boot==0) boot=1;
            ProcessStartInfo info=new ProcessStartInfo(Path.Combine(BuildRoot,"neck_demo.exe"));
            info.Arguments="--realtime --duration 0 --boot "+boot+" --mode "+(single?"single":"three")+(scripted?" --script":"");
            info.WorkingDirectory=BuildRoot; info.UseShellExecute=false; info.CreateNoWindow=true;
            info.RedirectStandardOutput=true; info.RedirectStandardInput=true; info.RedirectStandardError=true;
            try { process=Process.Start(info); }
            catch { Active=false; throw; }
            stderrWorker=new Thread(delegate() { try { string line; while((line=process.StandardError.ReadLine())!=null) Pipe.Note(line,Now); } catch(IOException) { } }); stderrWorker.IsBackground=true; stderrWorker.Start();
            worker=new Thread(delegate()
            {
                try { byte[] buffer=new byte[4096]; int n; while(!cancel && (n=process.StandardOutput.BaseStream.Read(buffer,0,buffer.Length))>0) Pipe.Feed(buffer,n,Now);
                    Pipe.Gap(Now); if(!cancel && process.WaitForExit(500) && process.ExitCode!=0) throw new IOException("C模拟器退出码 "+process.ExitCode); }
                catch(Exception e) { if(!cancel) { LastError=e.Message; Pipe.Note("数据源错误："+e.Message,Now); } }
                finally { Active=false; State=cancel?"已停止":"模拟器结束"; }
            }); worker.IsBackground=true; worker.Start();
        }
        public void Command(string command)
        {
            if(Kind!="SIM" || process==null || !Active) throw new InvalidOperationException("此命令仅供正在运行的模拟器，非MCU串口命令");
            process.StandardInput.WriteLine(command); process.StandardInput.Flush(); Pipe.Note("PC_SIM_REQUEST "+command+"（等待C端处理/参考提交）",Now);
        }
        public void StartSerial(string name,int baud,bool bluetooth=false)
        {
            if(string.IsNullOrWhiteSpace(name) || (baud!=115200 && (bluetooth || baud!=230400))) throw new ArgumentException("请选择有效COM口。50 Hz 使用115200，100 Hz 有线版使用230400。");
            string profilePath=Path.Combine(TaskRoot,MeasurementProfile.Filename);
            Begin("SERIAL",File.Exists(profilePath)?MeasurementProfile.Load(profilePath):new MeasurementProfile());
            State="等待D0协议串口";
            serialSession=new SerialLinkSession(Pipe,(bluetooth?"HC-05 蓝牙":"USB 串口")+" "+name);
            worker=new Thread(delegate()
            {
                try
                {
                    while(!cancel)
                    {
                        ISerialConnection connection=null;
                        try
                        {
                            serialSession.Attempt(Now); State=serialSession.Snapshot(Now).State;
                            connection=serialFactory(name,baud);
                            lock(serialGate) { if(cancel) break; serial=connection; }
                            connection.Open();
                            if(cancel) break;
                            connection.DiscardInput(); serialSession.Opened(Now); LastError="";
                            byte[] bytes=new byte[4096];
                            while(!cancel)
                            {
                                byte[] outgoing=Device.Next(Now);
                                if(outgoing!=null) connection.Write(outgoing);
                                try
                                {
                                    int n=connection.Read(bytes);
                                    if(cancel) break;
                                    if(n==0) throw new EndOfStreamException("串口读返回0字节");
                                    serialSession.Feed(bytes,n,Now);
                                }
                                catch(TimeoutException) { if(!cancel) serialSession.Tick(Now); }
                                if(!cancel && serialSession.RequiresReopen(Now))
                                    throw new IOException("连续3秒无有效D0消息，重新打开COM");
                                State=serialSession.Snapshot(Now).State;
                            }
                        }
                        catch(Exception e)
                        {
                            if(!cancel) { LastError=e.Message; serialSession.Failed(e.Message,Now); State=serialSession.Snapshot(Now).State; }
                        }
                        finally
                        {
                            lock(serialGate) { if(object.ReferenceEquals(serial,connection)) serial=null; }
                            if(connection!=null) { try { connection.Dispose(); } catch(Exception e) { if(!cancel) Pipe.Note("COM释放失败："+e.Message,Now); } }
                        }
                        if(!cancel) serialStop.WaitOne(retryMilliseconds);
                    }
                }
                finally { serialSession.Stopped(Now); Active=false; State="串口已停止"; }
            }); worker.IsBackground=true; worker.Start();
        }
        private void RestoreReplay()
        {
            Pipe.Reset();
            lock(Pipe.Sync)
            {
                if(Plan.Bootstrap.Length!=0) { Pipe.Feed(Plan.Bootstrap,Plan.Bootstrap.Length,0); if(Plan.HasSeed) Pipe.Bridge.Seed(Plan.SeedTx,Plan.SeedEmit); }
                if(Plan.Prefix.Length!=0) Pipe.Feed(Plan.Prefix,Plan.Prefix.Length,0);
            }
        }
        public void StartReplay(string path)
        {
            ReplayPlan plan=ReplayPlan.Load(path);
            string snapshot=Path.Combine(Path.GetDirectoryName(Path.GetFullPath(path)),MeasurementProfile.Filename);
            Begin("REPLAY",File.Exists(snapshot)?MeasurementProfile.Load(snapshot):null); Plan=plan; Interlocked.Exchange(ref replayTime,0); seekRequest=-1;
            lock(replayGate) { paused=false; speed=1; } State="回放："+plan.Timing;
            worker=new Thread(delegate()
            {
                try
                {
                    RestoreReplay(); int index=0; ulong virtualUs=0,prior=Microseconds(clock); byte[] bytes=new byte[4096];
                    using(FileStream file=File.OpenRead(Plan.Path))
                    {
                        while(!cancel)
                        {
                            ulong wall=Microseconds(clock),delta=wall-prior; prior=wall;
                            bool stop; double rate; lock(replayGate) { stop=paused; rate=speed; }
                            long seek=Interlocked.Exchange(ref seekRequest,-1);
                            if(seek>=0) { virtualUs=(ulong)seek; RestoreReplay(); index=0; }
                            else if(!stop) { ulong add=(ulong)(delta*rate); virtualUs=Math.Min(Plan.Duration,virtualUs+add); }
                            while(index<Plan.Chunks.Count && Plan.Chunks[index].Time<=virtualUs && !cancel)
                            {
                                ReplayChunk c=Plan.Chunks[index++]; file.Position=c.Offset;
                                if(c.Length==0) Pipe.Gap(c.Time);
                                else { int n=file.Read(bytes,0,c.Length); if(n!=c.Length) throw new EndOfStreamException(); Pipe.Feed(bytes,n,c.Time); }
                            }
                            Interlocked.Exchange(ref replayTime,checked((long)virtualUs));
                            if(index==Plan.Chunks.Count) State="回放结束（历史结果）"; else State=stop?"回放暂停":"回放 "+rate+"×";
                            Thread.Sleep(10);
                        }
                    }
                }
                catch(Exception e) { if(!cancel) { LastError=e.Message; Pipe.Note("回放错误："+e.Message,Now); } }
                finally { Active=false; }
            }); worker.IsBackground=true; worker.Start();
        }
        public void PauseReplay(bool value) { lock(replayGate) paused=value; }
        public void Speed(double value) { if(value!=1 && value!=2 && value!=4) throw new ArgumentOutOfRangeException("value"); lock(replayGate) speed=value; }
        public void Seek(ulong us) { if(Kind!="REPLAY" || Plan==null) return; Interlocked.Exchange(ref seekRequest,checked((long)Math.Min(us,Plan.Duration))); }
        public void Stop()
        {
            cancel=true; serialStop.Set(); CaptureRecorder recording=Pipe.StopCapture() ?? Pipe.LastRecorder;
            if(process!=null) { try { if(!process.HasExited) { process.StandardInput.WriteLine("QUIT"); process.StandardInput.Flush(); if(!process.WaitForExit(800)) { process.Kill(); process.WaitForExit(800); } } } catch(InvalidOperationException) { } catch(IOException) { } }
            lock(serialGate)
            { if(serial!=null) { try { serial.Close(); } catch(IOException) { } catch(InvalidOperationException) { } } }
            if(worker!=null && worker.IsAlive && !worker.Join(2000)) throw new IOException("读取线程尚未退出，暂不释放或替换接收状态");
            if(stderrWorker!=null && stderrWorker.IsAlive) stderrWorker.Join(500);
            if(process!=null) { process.Dispose(); process=null; } worker=null; stderrWorker=null;
            if(recording!=null) recording.Wait(3000); Active=false; State="已停止";
        }
        public void Dispose() { Stop(); Pipe.Dispose(); serialStop.Dispose(); }
    }
}
