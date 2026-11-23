using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
namespace NeckF103
{
    internal static class D5ControlTests
    {
        private static int checks; private static readonly StringBuilder log=new StringBuilder();
        private static void Check(bool value,string name) { checks++; if(!value) throw new Exception("FAIL "+name); }
        private static void Put(byte[] b,int at,ulong value,int size=4) { for(int i=0;i<size;i++) b[at+i]=(byte)(value>>(8*i)); }
        private static void Crc(byte[] b) {
            ushort crc=0xffff;
            for(int i=4;i<b.Length-2;i++) { crc^=(ushort)(b[i]<<8); for(int j=0;j<8;j++) crc=(ushort)((crc<<1)^((crc&0x8000)!=0?0x1021:0)); }
            Put(b,b.Length-2,crc,2);
        }
        private static byte[] Session(byte[] fixture,uint tx,ulong time,uint session=1,uint cap=63,uint period=10000) {
            byte[] b=fixture.Take(106).ToArray(); Put(b,8,tx); Put(b,12,501); Put(b,16,session); Put(b,20,1); Put(b,24,time,8);
            Put(b,36,0xd5000001); Put(b,44,0x01070301); Put(b,48,period); Put(b,52,period*3); Put(b,100,cap); Crc(b); return b;
        }
        private static byte[] RawFrame(uint tx,ulong emit,uint seq) {
            byte[] b=new byte[154]; b[0]=0x4e; b[1]=0x43; b[2]=0x4b; b[3]=0x31; b[4]=1; b[5]=6; Put(b,6,120,2);
            Put(b,8,tx); Put(b,12,501); Put(b,16,1); Put(b,20,1); Put(b,24,emit,8);
            Put(b,32,seq); Put(b,36,20000); b[40]=7;
            for(int i=0;i<3;i++) {
                int o=44+i*36; uint mid=(uint)(emit-(ulong)(3-i)*500);
                Put(b,o,mid-200); Put(b,o+4,mid+200); Put(b,o+8,mid);
                Put(b,o+12,16384,2); Put(b,o+24,981,2); b[o+32]=25; b[o+33]=0xff; Put(b,o+34,1,2);
            }
            Crc(b); return b;
        }
        private static byte[] Reply(WireMessage command,uint tx,ulong time,uint high=0) {
            byte[] b=NativeBridge.Command(command.Boot,command.Session,command.Config,time,command.RequestId,command.Opcode,new uint[]{high,0,1,1});
            b[5]=17; Put(b,8,tx); Crc(b); return b;
        }
        private static WireMessage Decode(byte[] b) {
            WireMessage decoded=null; using(NativeBridge bridge=new NativeBridge()) { bridge.Received=delegate(WireMessage m,int r,bool boundary,ulong t) { decoded=m; }; bridge.Feed(b,b.Length,0); }
            Check(decoded!=null,"shared C codec decoded command/frame"); return decoded;
        }
        private static void Until(Func<bool> predicate,string name) {
            Stopwatch w=Stopwatch.StartNew(); while(!predicate() && w.ElapsedMilliseconds<3000) Thread.Sleep(10); Check(predicate(),name);
        }
        private sealed class FakeSerial : ISerialConnection
        {
            private readonly object gate=new object(); private readonly Queue<byte[]> queue=new Queue<byte[]>();
            private readonly byte[] fixture; private readonly Stopwatch watch=Stopwatch.StartNew();
            private uint tx,high; private byte[] lastZero; private bool closed;
            public volatile int ZeroWrites,ZeroExecutions,WriterThread;
            public FakeSerial(byte[] value) { fixture=value; }
            public void Open() { }
            public void DiscardInput() { }
            public void Close() { lock(gate) closed=true; }
            public void Dispose() { Close(); }
            private ulong Time { get { return 100000+DataEngine.Microseconds(watch); } }
            public int Read(byte[] b) {
                Thread.Sleep(15); lock(gate) { if(closed) throw new IOException("closed");
                    byte[] next=queue.Count>0?queue.Dequeue():Session(fixture,tx++,Time); Array.Copy(next,b,next.Length); return next.Length; }
            }
            public void Write(byte[] b) {
                lock(gate) { WriterThread=Thread.CurrentThread.ManagedThreadId; WireMessage m=Decode(b);
                    if(m.Opcode==2) {
                        ZeroWrites++; if(lastZero==null) { lastZero=(byte[])b.Clone(); ZeroExecutions++; high=m.RequestId; return; }
                        Check(lastZero.SequenceEqual(b),"retry is byte-identical, same request ID");
                    }
                    queue.Enqueue(Reply(m,tx++,Time,high));
                }
            }
        }
        public static int Run(string root,string output,string fixtures,string build) {
            try {
                byte[] fixture=File.ReadAllBytes(Path.Combine(fixtures,"normal_all.bin"));
                using(Pipeline p=new Pipeline()) {
                    DeviceControl control=new DeviceControl(p); p.Accepted=control.Receive; p.SegmentReset=control.Reset;
                    byte[] s=Session(fixture,0,100000); p.Feed(s,s.Length,0);
                    WireMessage query=Decode(control.Next(0)); Check(query.Opcode==1,"automatic high-water query");
                    byte[] reply=Reply(query,1,100100,8); p.Feed(reply,reply.Length,100);
                    Check(control.Ready(100),"control ready only after query reply");
                    CaptureRecorder capture=p.StartCapture(output,100);
                    control.Request(2,0,0,200); byte[] command=control.Next(200); WireMessage zero=Decode(command);
                    Check(zero.RequestId==9 && zero.Boot==501,"high-water ID and device binding");
                    Check(control.Next(300000)==null,"retry delay");
                    s=Session(fixture,2,500000); p.Feed(s,s.Length,400000);
                    Check(command.SequenceEqual(control.Next(400201)),"exact retry after 400ms");
                    reply=Reply(zero,3,500001); p.Feed(reply,reply.Length,400202);
                    Check(control.Ready(400202) && control.Status.Contains("等待零位成功"),"ack is not zero commit");
                    p.StopCapture(); Check(capture.Wait(3000) && capture.Completed,"control capture complete");
                    Check(File.ReadAllText(Path.Combine(capture.DirectoryPath,"commands.log")).Contains("identity=501/1/1 args=0/0/0/0"),"command identity and arguments persisted");
                    ReplayPlan plan=ReplayPlan.Load(capture.DirectoryPath);
                    using(Pipeline replay=new Pipeline()) {
                        replay.Feed(plan.Bootstrap,plan.Bootstrap.Length,0); replay.Bridge.Seed(plan.SeedTx,plan.SeedEmit);
                        byte[] raw=File.ReadAllBytes(plan.Path); replay.Feed(raw,raw.Length,400102);
                        Check(replay.Snapshot(400102).Messages==3,"control response capture replays via native codec without transport");
                    }
                    Check(!control.Ready(900203) && control.Next(900203)==null,"stale link refuses commands");
                    control.Request(4,0,0,400203); WireMessage stop=Decode(control.Next(400203));
                    reply=Reply(stop,4,500002); p.Feed(reply,reply.Length,400204);
                    Check(!control.Ready(400204),"lifecycle reply waits for fresh session");
                    s=Session(fixture,5,500003,2); p.Feed(s,s.Length,400205);
                    query=Decode(control.Next(400205)); Check(query.Session==2 && query.Opcode==1,"new session queries instead of retrying old command");
                    p.Reset(); Check(control.Next(400206)==null,"reset cancels pending control");
                }
                log.AppendLine("PASS native command/reply codec, freshness, retry, high-water, session reset");
                using(Pipeline p=new Pipeline()) {
                    DeviceControl control=new DeviceControl(p); WireMessage session=Decode(Session(fixture,0,100)); control.Receive(session,0);
                    WireMessage q=Decode(control.Next(0)); control.Receive(new WireMessage {Type=17,Boot=501,Session=1,Config=1,Emit=101,RequestId=q.RequestId,Opcode=1,CommandArgs=new uint[4]},1);
                    WireMessage ev=new WireMessage {Type=4,Boot=501,Session=1,Config=1,Emit=102,Event=new EventData {Id=0x100000002}};
                    control.Receive(ev,2); WireMessage ack=Decode(control.Next(2)); Check(ack.Opcode==8 && ack.CommandArgs[0]==2 && ack.CommandArgs[1]==1,"64-bit event acknowledgement");
                    control.Receive(ev,3); Check(Decode(control.Next(3)).Opcode==8,"duplicate event acknowledged again");
                    control.Request(2,0,0,4); control.Next(4);
                    for(ulong t=400004;t<=1200004;t+=400000) { session.Emit=t; control.Receive(session,t); control.Next(t); }
                    Check(!control.Ready(1200004) && Decode(control.Next(1200005)).Opcode==1,"lost replies trigger resync not blind zero re-execution");
                }
                log.AppendLine("PASS event re-ACK and bounded retry failure resynchronization");
                FakeSerial port=new FakeSerial(fixture);
                using(DataEngine engine=new DataEngine(root,build,delegate { return port; },20)) {
                    engine.StartSerial("FAKE_D5",115200); Until(delegate { return engine.Device.Ready(engine.Now); },"production worker command bootstrap");
                    int uiThread=Thread.CurrentThread.ManagedThreadId; engine.DeviceCommand(2);
                    Until(delegate { return port.ZeroWrites>=2 && engine.Device.Ready(engine.Now); },"production worker recovers lost response");
                    Check(port.ZeroExecutions==1 && port.WriterThread!=uiThread,"single serial worker, one execution");
                    using(DeviceControlForm form=new DeviceControlForm(engine)) {
                        form.ShowInTaskbar=false; form.Opacity=0; form.Show(); System.Windows.Forms.Application.DoEvents(); form.PerformLayout();
                        using(Bitmap bitmap=new Bitmap(form.Width,form.Height)) { form.DrawToBitmap(bitmap,new Rectangle(Point.Empty,bitmap.Size)); bitmap.Save(Path.Combine(output,"d5_control_ui.png")); }
                    }
                    engine.Stop(); Check(engine.Device.Next(engine.Now)==null,"stop clears control session");
                }
                log.AppendLine("PASS production serial thread / fake duplex transport / UI render");
                using(Pipeline p=new Pipeline()) {
                    uint tx=0; byte[] s=Session(fixture,tx++,20000,1,127,20000); p.Feed(s,s.Length,20000);
                    WireMessage zcmd=new WireMessage {Boot=501,Session=1,Config=1,RequestId=1,Opcode=2};
                    for(uint step=1;step<=200;step++) {
                        ulong t=(ulong)step*20000;
                        if(step==5) { byte[] zr=Reply(zcmd,tx++,t); p.Feed(zr,zr.Length,t); }
                        byte[] rf=RawFrame(tx++,t,step); p.Feed(rf,rf.Length,t);
                    }
                    ViewSnapshot snap=p.Snapshot(4000000);
                    Check(snap.C4!=null,"thin-client C4 non-null (msgs="+snap.Messages+" rej="+p.Bridge.Stats()[9]+" pay="+p.Bridge.Stats()[5]+")");
                    Check(snap.C4.C4.Zero==3 && (snap.C4.C4.Mask&48)==48,"thin-client 50Hz raw frame C4 solved & zeroed on PC host (zero="+snap.C4.C4.Zero+" mask="+snap.C4.C4.Mask+" qual="+snap.C4.C4.Quality+" filt="+snap.C4.C4.Filter+")");
                    Check(snap.Relative!=null && snap.Relative.Relative.Reference>0,"thin-client 50Hz raw frame 3-node relative solved & zeroed on PC host");
                    Check(snap.Status!=null && snap.Status.Status.AccelMask==7,"thin-client 50Hz raw frame status synthesized on PC host");
                }
                log.AppendLine("PASS thin-client 50Hz raw frame -> PC host C algorithm brain & zero commit");
                foreach(int scenario in new[]{0,1,2,3,4,5}) using(Pipeline p=new Pipeline()) {
                    var profile=new MeasurementProfile {Registered=scenario!=3,Name="bench"};
                    if(scenario==0) profile.WorldC0="0.9238795325 0 0 0.3826834324";
                    string path=Path.Combine(output,MeasurementProfile.Filename);
                    profile.Save(path); p.Profile=MeasurementProfile.Load(path); p.Reset();
                    ulong origin=(1UL<<32)-2000000;
                    if(scenario==0) origin=7200000000UL;
                    uint period=scenario==5?10000u:20000u;
                    uint tx=0; byte[] session=Session(fixture,tx++,origin,1,127,period); p.Feed(session,session.Length,0);
                    CaptureRecorder recorded=scenario==0?p.StartCapture(output,0):null;
                    WireMessage cmd=new WireMessage {Boot=501,Session=1,Config=1,RequestId=1,Opcode=2};
                    for(uint step=1;step<=4000000u/period;step++) {
                        ulong time=origin+(ulong)step*period;
                        if(step==5) {byte[] reply=Reply(cmd,tx++,time); p.Feed(reply,reply.Length,time);}
                        byte[] raw=RawFrame(tx++,time,step); Put(raw,36,period);
                        for(int i=0;i<3;i++) {
                            int offset=44+i*36;
                            if(scenario==1) Put(raw,offset+34,3,2);
                            if(scenario==2) {raw[offset+33]=0; Put(raw,offset+34,9,2);}
                        }
                        Crc(raw); p.Feed(raw,raw.Length,time);
                    }
                    ViewSnapshot state=p.Snapshot(origin+4000000);
                    bool valid=scenario==0 || scenario==4 || scenario==5;
                    Check((state.C4.C4.Zero==3)==valid,"late join / coarse / uncalibrated / unregistered / wrap: "+scenario);
                    Check((state.Relative.Relative.Reference>0)==valid,"relative reference: "+scenario);
                    if(recorded!=null) {
                        p.StopCapture(); Check(recorded.Wait(5000) && recorded.Completed,"registered RAW capture finalized");
                        MeasurementProfile archived=MeasurementProfile.Load(Path.Combine(recorded.DirectoryPath,MeasurementProfile.Filename));
                        Check(archived.Registered && archived.WorldC0==profile.WorldC0,"saved exact world transform with recording");
                        Check(Math.Abs(state.Relative.Relative.Pairs[0].Angles[2]-45000)<5,"configured C0 world rotation feeds overall yaw");
                    }
                }
                log.AppendLine("RESULT PASS checks="+checks+"; fake transport only, no board or Bluetooth tested.");
                TaskFiles.TextNew(Path.Combine(output,"d5_control_test.log"),log.ToString()); TaskFiles.TextNew(Path.Combine(output,"D5_COMPLETE.txt"),"PASS software control tests\n"); return 0;
            } catch(Exception e) { log.AppendLine(e.ToString()); TaskFiles.TextNew(Path.Combine(output,"d5_control_test.log"),log.ToString()); return 1; }
        }
    }
}
