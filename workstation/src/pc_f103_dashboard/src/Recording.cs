using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading;

namespace NeckF103
{
    public static class TaskFiles
    {
        public static string NewRun(string taskRoot,string relative,string label)
        {
            string root=Path.GetFullPath(taskRoot).TrimEnd(Path.DirectorySeparatorChar);
            string parent=Path.GetFullPath(Path.Combine(root,relative));
            if(!parent.StartsWith(root+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)) throw new IOException("Run outside isolated workspace");
            Directory.CreateDirectory(parent);
            string path=Path.Combine(parent,label+"_"+DateTime.Now.ToString("yyyyMMdd_HHmmss_fff",CultureInfo.InvariantCulture)+"_"+Guid.NewGuid().ToString("N").Substring(0,6));
            if(Directory.Exists(path)) throw new IOException("Run exists"); Directory.CreateDirectory(path); return path;
        }
        public static void TextNew(string path,string text)
        { using(FileStream f=new FileStream(path,FileMode.CreateNew,FileAccess.Write,FileShare.Read)) using(StreamWriter w=new StreamWriter(f,new UTF8Encoding(false))) w.Write(text); }
        public static string Number(ulong v) { return v.ToString(CultureInfo.InvariantCulture); }
    }
    public sealed class CaptureRecorder
    {
        private sealed class Item { public byte[] Bytes; public WireMessage Message; public string Command; public ulong Time; public int Result,Source; public bool Boundary,EventNew,OldEpoch,Gap; }
        private readonly Queue<Item> queue=new Queue<Item>(); private readonly object gate=new object();
        private readonly Thread worker; private readonly BridgeSnapshot start; private readonly ulong startUs;
        private readonly int maxBytes; private int queuedBytes; private bool stopping;
        public readonly string DirectoryPath; public volatile bool Completed,Finished; public volatile string Error="";
        public CaptureRecorder(string directory,BridgeSnapshot snapshot,ulong origin,int capacity)
        { DirectoryPath=directory; start=snapshot; startUs=origin; maxBytes=capacity; worker=new Thread(WriteLoop); worker.IsBackground=true; worker.Start(); }
        private ulong Time(ulong t) { return t>=startUs?t-startUs:0; }
        private void Enqueue(Item item,int cost)
        {
            lock(gate)
            {
                if(stopping) return;
                if(queuedBytes+cost>maxBytes || queue.Count>=8192) { Error="记录缓冲溢出：保留部分数据，不标记完成"; stopping=true; Monitor.PulseAll(gate); return; }
                queue.Enqueue(item); queuedBytes+=cost; Monitor.PulseAll(gate);
            }
        }
        public void Raw(byte[] b,int n,ulong t) { byte[] copy=new byte[n]; Array.Copy(b,copy,n); Enqueue(new Item { Bytes=copy,Time=Time(t) },n); }
        public void Gap(ulong t) { Enqueue(new Item { Gap=true,Time=Time(t) },64); }
        public void Message(WireMessage m,int result,bool boundary,ulong t,int source,bool eventNew,bool oldEpoch)
        { Enqueue(new Item { Message=m,Result=result,Boundary=boundary,Time=Time(t),Source=source,EventNew=eventNew,OldEpoch=oldEpoch },m.Canonical.Length+256); }
        public void Command(string text,ulong t) { Enqueue(new Item { Command=text.Replace('\r',' ').Replace('\n',' '),Time=Time(t) },text.Length+64); }
        public void Stop() { lock(gate) { stopping=true; Monitor.PulseAll(gate); } }
        public bool Wait(int ms) { return worker.Join(ms); }
        private static StreamWriter Writer(string path) { return new StreamWriter(new FileStream(path,FileMode.CreateNew,FileAccess.Write,FileShare.Read),new UTF8Encoding(false),65536); }
        private static string Angle(int v,bool valid) { return valid && v!=int.MinValue?(v/1000.0).ToString("F3",CultureInfo.InvariantCulture):""; }
        private static void Angles(StreamWriter writer,Item item)
        {
            WireMessage m=item.Message; if(item.Result!=0 || item.OldEpoch) return;
            string prefix=TaskFiles.Number(item.Time)+","+item.Source+","+m.Boot+","+m.Session+","+m.Config+","+m.Emit+",";
            if(m.Type==2)
            {
                C4Data c=m.C4; string row=prefix+"C4,"+c.Epoch+","+c.Reference+","+c.Quality+","+c.Mask;
                for(int i=0;i<6;i++) row+=","+Angle(c.Angles[i],(c.Mask&(1U<<i))!=0);
                writer.WriteLine(row+","+c.Axes[0].Instant+","+c.Axes[0].Alarm+","+c.Axes[1].Instant+","+c.Axes[1].Alarm);
            }
            else if(m.Type==3)
            {
                RelativeData r=m.Relative;
                for(int i=0;i<4;i++) { PairData p=r.Pairs[i]; string row=prefix+"PAIR"+i+","+r.Epoch+","+r.Reference+","+r.Quality+","+p.Mask;
                    for(int j=0;j<6;j++) row+=","+Angle(p.Angles[j],(p.Mask&(j<3?1:2))!=0); writer.WriteLine(row+",,,,"); }
            }
        }
        private void WriteLoop()
        {
            try
            {
                TaskFiles.TextNew(Path.Combine(DirectoryPath,"metadata.txt"),"NECK_F103_D1_CAPTURE_V1\nstart_utc="+DateTime.UtcNow.ToString("o",CultureInfo.InvariantCulture)+
                    "\nstart_tx="+start.LastTx+"\nstart_emit="+start.LastEmit+"\nmode=RECEIVED_WIRE_RESULTS_NOT_ALGORITHM_RECOMPUTATION\n");
                using(FileStream p=new FileStream(Path.Combine(DirectoryPath,"parser_prefix.bin"),FileMode.CreateNew,FileAccess.Write)) p.Write(start.Prefix,0,start.Prefix.Length);
                using(FileStream s=new FileStream(Path.Combine(DirectoryPath,"bootstrap_session.bin"),FileMode.CreateNew,FileAccess.Write)) s.Write(start.Session,0,start.Session.Length);
                using(FileStream raw=new FileStream(Path.Combine(DirectoryPath,"capture.bin"),FileMode.CreateNew,FileAccess.Write,FileShare.Read,65536))
                using(StreamWriter chunks=Writer(Path.Combine(DirectoryPath,"chunks.csv")))
                using(StreamWriter decoded=Writer(Path.Combine(DirectoryPath,"decoded.csv")))
                using(StreamWriter receive=Writer(Path.Combine(DirectoryPath,"receive.csv")))
                using(StreamWriter angles=Writer(Path.Combine(DirectoryPath,"angles.csv")))
                using(StreamWriter events=Writer(Path.Combine(DirectoryPath,"events.csv")))
                using(StreamWriter commands=Writer(Path.Combine(DirectoryPath,"commands.log")))
                {
                    chunks.WriteLine("arrival_us,offset,length"); receive.WriteLine("arrival_us,rx_result,boundary,old_epoch,canonical_W1_fields");
                    angles.WriteLine("arrival_us,source,boot,session,config,emit_us,group,epoch,reference,quality,mask,raw_1,raw_2,raw_3_or_lp1,corr_1_or_lp2,corr_2,corr_3,instant_flex,alarm_flex,instant_lateral,alarm_lateral");
                    events.WriteLine("arrival_us,source,boot,session,event_id,event_time_us,axis,bits,previous_level,level,direction,reference,epoch");
                    for(;;)
                    {
                        Item item;
                        lock(gate) { while(queue.Count==0 && !stopping) Monitor.Wait(gate,100); if(queue.Count==0 && stopping) break;
                            item=queue.Dequeue(); queuedBytes-=item.Gap?64:item.Bytes!=null?item.Bytes.Length:item.Message!=null?item.Message.Canonical.Length+256:item.Command.Length+64; }
                        if(item.Gap) chunks.WriteLine(item.Time+","+raw.Position+",0");
                        else if(item.Bytes!=null) { chunks.WriteLine(item.Time+","+raw.Position+","+item.Bytes.Length); raw.Write(item.Bytes,0,item.Bytes.Length); }
                        else if(item.Command!=null) commands.WriteLine(item.Time+","+item.Command);
                        else
                        {
                            WireMessage m=item.Message; decoded.WriteLine(m.Canonical);
                            receive.WriteLine(item.Time+","+item.Result+","+(item.Boundary?1:0)+","+(item.OldEpoch?1:0)+","+m.Canonical); Angles(angles,item);
                            if(item.EventNew) { EventData e=m.Event; events.WriteLine(item.Time+","+item.Source+","+m.Boot+","+m.Session+","+e.Id+","+e.Time+","+e.Axis+","+e.Bits+","+e.Previous+","+e.Level+","+e.Direction+","+e.Reference+","+e.Epoch); }
                        }
                    }
                }
                if(Error.Length==0)
                {
                    StringBuilder hashes=new StringBuilder(); foreach(string file in Directory.GetFiles(DirectoryPath)) using(FileStream f=File.OpenRead(file)) using(SHA256 sha=SHA256.Create())
                        hashes.Append(BitConverter.ToString(sha.ComputeHash(f)).Replace("-","")).Append(' ').Append(Path.GetFileName(file)).Append('\n');
                    TaskFiles.TextNew(Path.Combine(DirectoryPath,"sha256.txt"),hashes.ToString()); TaskFiles.TextNew(Path.Combine(DirectoryPath,"COMPLETE.txt"),"D1 capture drained and closed. Wire results, not sensor accuracy evidence.\n"); Completed=true;
                }
            }
            catch(Exception e) { Error=e.Message; }
            finally
            {
                lock(gate) { stopping=true; queue.Clear(); queuedBytes=0; }
                if(Error.Length!=0) { try { TaskFiles.TextNew(Path.Combine(DirectoryPath,"FAILED.txt"),Error+"\n"); } catch(IOException) { } }
                Finished=true;
            }
        }
    }
    public sealed class Pipeline : IDisposable
    {
        public readonly object Sync=new object(); public ViewModel View=new ViewModel(); public NativeBridge Bridge;
        public CaptureRecorder Recorder,LastRecorder;
        public Action<WireMessage,ulong> Accepted;
        internal MeasurementProfile Profile;
        public Action SegmentReset;
        // Optional in-process 3D display tap. The receiver copies bytes before Feed returns.
        internal Action<byte[],int> DisplayBytes=null;
        public Pipeline() { Reset(); }
        public void Reset() { Reset(false); }
        private void Reset(bool preserveJournal)
        { lock(Sync) { if(Recorder!=null) throw new InvalidOperationException("Stop recording before reset");
                string[] journal=preserveJournal?View.Journal():new string[0];
                if(Bridge!=null) Bridge.Dispose(); Bridge=new NativeBridge(); Bridge.Received=Receive; View=new ViewModel();
                if(Profile!=null) Bridge.SetProfile(Profile);
                if(DisplayBytes!=null) DisplayBytes(null,0);
                foreach(string line in journal) View.Note(line);
                if(SegmentReset!=null) SegmentReset(); } }
        public ulong AcceptedMessages { get { lock(Sync) return View.Messages; } }
        public void EndSerialSegment(string reason,ulong time)
        {
            // Close the recording and clear parser/session/pose in one lock.
            // A UI record request cannot slip between these operations.
            lock(Sync) { Note(reason,time); StopCapture(); Reset(true); }
        }
        private void Receive(WireMessage m,int result,bool boundary,ulong time)
        {
            ulong priorEvents=View.AlarmEvents,priorOld=View.OldEpochDrops; View.Accept(m,result,boundary);
            int source=View.Session==null?255:View.Session.Registration.Source;
            if(Recorder!=null) Recorder.Message(m,result,boundary,time,source,View.AlarmEvents!=priorEvents,View.OldEpochDrops!=priorOld);
            if((result==0 || result==1) && Accepted!=null) Accepted(m,time);
        }
        public void Feed(byte[] b,int count,ulong t) { lock(Sync) { if(Recorder!=null) Recorder.Raw(b,count,t); Bridge.Feed(b,count,t); if(DisplayBytes!=null) DisplayBytes(b,count); } }
        public void Gap(ulong t) { lock(Sync) { if(Recorder!=null) Recorder.Gap(t); Bridge.Gap(t); } }
        public void Note(string text,ulong t) { lock(Sync) { View.Note(text); if(Recorder!=null) Recorder.Command(text,t); } }
        public ViewSnapshot Snapshot(ulong time) { lock(Sync) return View.Snapshot(Bridge.Stats(),Bridge.Poll(time)); }
        public CaptureRecorder StartCapture(string root,ulong time)
        { lock(Sync) { if(Recorder!=null) throw new InvalidOperationException("Already recording");
                if(View.Session==null || (Bridge.Poll(time)&1)!=0) throw new InvalidOperationException("等待有效SESSION后再开始记录");
                string dir=TaskFiles.NewRun(root,"data/我的记录","capture");
                if(Profile!=null) Profile.Save(Path.Combine(dir,MeasurementProfile.Filename));
                Recorder=new CaptureRecorder(dir,Bridge.Snapshot(),time,4*1024*1024); LastRecorder=Recorder; return Recorder; } }
        public CaptureRecorder StopCapture() { lock(Sync) { CaptureRecorder r=Recorder; Recorder=null; if(r!=null) r.Stop(); return r; } }
        public void Dispose() { CaptureRecorder r=StopCapture(); if(r!=null) r.Wait(3000); lock(Sync) { if(Bridge!=null) Bridge.Dispose(); Bridge=null; } }
    }
    public sealed class ReplayChunk { public ulong Time; public long Offset; public int Length; }
    public sealed class ReplayPlan
    {
        public string Path; public byte[] Prefix=new byte[0],Bootstrap=new byte[0]; public uint SeedTx; public ulong SeedEmit;
        public readonly List<ReplayChunk> Chunks=new List<ReplayChunk>(); public ulong Duration; public bool HasSeed; public string Timing;
        public static ReplayPlan Load(string path)
        {
            if(Directory.Exists(path)) path=System.IO.Path.Combine(path,"capture.bin");
            string dir=System.IO.Path.GetDirectoryName(path); ReplayPlan plan=new ReplayPlan(); plan.Path=path;
            long size=new FileInfo(path).Length; if(size>256L*1024*1024) throw new IOException("Replay size limit 256 MB");
            if(File.Exists(System.IO.Path.Combine(dir,"metadata.txt")) && System.IO.Path.GetFileName(path)=="capture.bin")
            {
                if(!File.Exists(System.IO.Path.Combine(dir,"COMPLETE.txt"))) throw new IOException("记录未完成或失败，不能作为完整回放打开");
                Dictionary<string,string> fields=new Dictionary<string,string>(); string[] meta=File.ReadAllLines(System.IO.Path.Combine(dir,"metadata.txt"));
                if(meta.Length==0 || meta[0]!="NECK_F103_D1_CAPTURE_V1") throw new FormatException("Capture metadata version");
                for(int i=1;i<meta.Length;i++) { int eq=meta[i].IndexOf('='); if(eq>0) fields[meta[i].Substring(0,eq)]=meta[i].Substring(eq+1); }
                plan.SeedTx=uint.Parse(fields["start_tx"],CultureInfo.InvariantCulture); plan.SeedEmit=ulong.Parse(fields["start_emit"],CultureInfo.InvariantCulture);
                plan.Prefix=File.ReadAllBytes(System.IO.Path.Combine(dir,"parser_prefix.bin")); plan.Bootstrap=File.ReadAllBytes(System.IO.Path.Combine(dir,"bootstrap_session.bin"));
                if(plan.Prefix.Length>290 || plan.Bootstrap.Length>290) throw new FormatException("Capture bootstrap size"); plan.HasSeed=plan.Bootstrap.Length!=0;
                long next=0; ulong last=0; bool header=true;
                foreach(string row in File.ReadLines(System.IO.Path.Combine(dir,"chunks.csv")))
                {
                    if(header) { if(row!="arrival_us,offset,length") throw new FormatException("Chunk schema"); header=false; continue; }
                    string[] p=row.Split(','); if(p.Length!=3) throw new FormatException("Chunk columns");
                    ulong t=ulong.Parse(p[0],NumberStyles.None,CultureInfo.InvariantCulture); long off=long.Parse(p[1],NumberStyles.None,CultureInfo.InvariantCulture); int n=int.Parse(p[2],NumberStyles.None,CultureInfo.InvariantCulture);
                    if(t<last || off!=next || n<0 || n>4096 || off>size-n) throw new FormatException("Chunk time/bounds");
                    plan.Chunks.Add(new ReplayChunk { Time=t,Offset=off,Length=n }); next+=n; last=t; if(plan.Chunks.Count>2000000) throw new IOException("Replay chunk limit");
                }
                if(next!=size) throw new FormatException("Capture length differs from index"); plan.Duration=last; plan.Timing="原接收时间回放";
            }
            else
            {
                // Standalone D0 stream: validation uses the C codec, no PC pose algorithm.
                using(FileStream file=File.OpenRead(path)) using(NativeBridge probe=new NativeBridge())
                {
                    List<WireMessage> got=new List<WireMessage>(); probe.Received=delegate(WireMessage m,int result,bool boundary,ulong t) { got.Add(m); };
                    ulong first=0,last=0; bool seen=false; byte[] header=new byte[8];
                    while(file.Position<size)
                    {
                        long off=file.Position; if(file.Read(header,0,8)!=8) throw new FormatException("Truncated standalone stream; use raw diagnostics instead");
                        int length=34+header[6]+(header[7]<<8); if(length>290 || off>size-length) throw new FormatException("Standalone frame length");
                        byte[] b=new byte[length]; Array.Copy(header,b,8); if(file.Read(b,8,length-8)!=length-8) throw new EndOfStreamException();
                        got.Clear(); probe.Feed(b,b.Length,0); if(got.Count!=1) throw new FormatException("Standalone BIN must contain valid D0 frames");
                        ulong emit=got[0].Emit; if(!seen) { first=emit; seen=true; } ulong time=emit>=first?emit-first:last;
                        if(time<last) time=last; plan.Chunks.Add(new ReplayChunk { Offset=off,Length=length,Time=time }); last=time;
                        if(plan.Chunks.Count>2000000) throw new IOException("Replay frame limit");
                    }
                    plan.Duration=last; plan.Timing="按设备发送时间演示，非实测接收延迟";
                }
            }
            return plan;
        }
    }
}
