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
    internal static class D4SerialTests
    {
        private static int checks,groups; private static readonly StringBuilder log=new StringBuilder();
        private static void Check(bool value,string name) { checks++; if(!value) throw new Exception("FAIL: "+name); }
        private static void Group(string name,Action action) { action(); groups++; log.Append("PASS ").Append(name).Append('\n'); }
        private static void Until(Func<bool> predicate,string name,int timeout=3000)
        { Stopwatch w=Stopwatch.StartNew(); while(!predicate() && w.ElapsedMilliseconds<timeout) Thread.Sleep(10); Check(predicate(),name); }
        private sealed class FakeSerial : ISerialConnection
        {
            private readonly object gate=new object(); private readonly Queue<object> incoming=new Queue<object>();
            public volatile bool OpenCalled,Discarded,Closed,Disposed; public bool FailOpen,BlockOpen;
            public readonly ManualResetEvent OpenRelease=new ManualResetEvent(false);
            public void Push(object item) { lock(gate) { incoming.Enqueue(item); Monitor.PulseAll(gate); } }
            public void Open()
            { OpenCalled=true; if(FailOpen) throw new IOException("injected open failure"); if(BlockOpen && !OpenRelease.WaitOne(3000)) throw new IOException("injected open timeout"); }
            public void DiscardInput() { lock(gate) { incoming.Clear(); Discarded=true; } }
            public int Read(byte[] bytes)
            {
                lock(gate)
                {
                    if(incoming.Count==0 && !Closed) Monitor.Wait(gate,20);
                    if(Closed) throw new IOException("injected closed connection");
                    if(incoming.Count==0) throw new TimeoutException();
                    object item=incoming.Dequeue(); Exception error=item as Exception; if(error!=null) throw error;
                    byte[] data=(byte[])item; Array.Copy(data,bytes,data.Length); return data.Length;
                }
            }
            public void Close() { lock(gate) { Closed=true; Monitor.PulseAll(gate); } OpenRelease.Set(); }
            public void Write(byte[] bytes) { if(Closed) throw new IOException("closed"); }
            public void Dispose() { Close(); Disposed=true; }
        }
        public static int Run(string root,string output,string fixtures,string build)
        {
            try
            {
                root=Path.GetFullPath(root); output=Path.GetFullPath(output);
                if(!output.StartsWith(root.TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)) throw new IOException("Test output outside workspace");
                byte[] normal=File.ReadAllBytes(Path.Combine(fixtures,"normal_all.bin"));
                byte[] c4=File.ReadAllBytes(Path.Combine(fixtures,"c4.bin"));
                Group("fragmented and coalesced D0 stream, CRC rejection, single/three-node labels",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        SerialLinkSession link=new SerialLinkSession(p,"FAKE HC-05"); link.Attempt(0); link.Opened(0);
                        byte[] bad=(byte[])c4.Clone(); bad[40]^=1; link.Feed(bad,bad.Length,10);
                        int pos=0; int[] sizes={1,7,19,3,211,4096}; ulong t=100;
                        for(int i=0;pos<normal.Length;i++) { int n=Math.Min(sizes[i%sizes.Length],normal.Length-pos); byte[] part=normal.Skip(pos).Take(n).ToArray(); link.Feed(part,n,t++); pos+=n; }
                        ViewSnapshot s=p.Snapshot(t); Check(s.Messages==5 && s.C4!=null && s.Relative!=null && s.Status!=null,"five real parser message types");
                        Check(s.Stats[1]==1 && s.Session.Registration.Mode==3,"CRC failure counted; source mode preserved");
                        link.Tick(300000); Check(p.Snapshot(300000).Session!=null && (p.Snapshot(300000).HostFlags&6)==6,"pose timeout precedes link boundary");
                        link.Opened(400000); byte[] single=File.ReadAllBytes(Path.Combine(fixtures,"single_node.bin")); link.Feed(single,single.Length,400100);
                        s=p.Snapshot(400100); Check(s.Session.Registration.NodeMask==2 && s.Relative==null,"single mode has no invented relative angles");
                        Check(link.Snapshot(400100).Connections==2 && link.Snapshot(400100).Accepted==8,"cumulative link counters survive parser reset");
                    }
                });
                Group("silent open COM clears session, references and partial frame; recording stops at boundary",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        SerialLinkSession link=new SerialLinkSession(p,"FAKE HC-05"); link.Opened(0); link.Feed(normal,normal.Length,2000);
                        CaptureRecorder r=p.StartCapture(output,3000);
                        byte[] head=c4.Take(20).ToArray(); link.Feed(head,head.Length,4000);
                        Check(p.Bridge.Snapshot().Prefix.Length==20,"partial frame buffered before silence");
                        link.Tick(502001); ViewSnapshot s=p.Snapshot(502001);
                        Check(s.Session==null && s.C4==null && s.Relative==null && s.Status==null && s.Plots[0].Length==0,"silence invalidates all displayed state");
                        Check(p.Bridge.Snapshot().Prefix.Length==0 && p.Recorder==null,"partial parser and recording cleared atomically");
                        Check(r.Wait(3000) && r.Completed,"recording finalized after link silence");
                        Check(File.ReadAllText(Path.Combine(r.DirectoryPath,"commands.log")).Contains("超过500ms"),"boundary reason persisted in capture");
                        Check(ReplayPlan.Load(r.DirectoryPath).Chunks.Count==1,"partial capture remains loadable for review");
                        bool denied=false; try { p.StartCapture(output,502002); } catch(InvalidOperationException) { denied=true; } Check(denied,"cannot record unregistered reconnect");
                        byte[] tail=c4.Skip(20).ToArray(); link.Feed(tail,tail.Length,510000); link.Feed(c4,c4.Length,520000);
                        Check(p.Snapshot(520000).C4==null && p.Snapshot(520000).Session==null,"tail/pose before new SESSION rejected");
                        link.Feed(normal,normal.Length,530000);
                        Check(p.Snapshot(530000).Session!=null && p.Snapshot(530000).C4!=null,"same device can re-register after silence");
                        Check(p.Recorder==null,"recording does not silently resume across outage");
                        link.Tick(530001); Check(link.Snapshot(530001).Silences==1,"one silence transition");
                        TaskFiles.TextNew(Path.Combine(output,"serial_capture_path.txt"),r.DirectoryPath+"\n");
                    }
                });
                Group("noise cannot renew D0 freshness; timeout checked before resumed bytes",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        SerialLinkSession link=new SerialLinkSession(p,"FAKE USB"); link.Opened(0); link.Feed(normal,normal.Length,0);
                        byte[] junk={0,3,7,0,3,7};
                        for(ulong t=100000;t<=600000;t+=100000) link.Feed(junk,junk.Length,t);
                        Check(p.Snapshot(600000).Session==null,"continuous junk does not keep registration");
                        link.Tick(2000000); Check(link.Snapshot(2000000).Silences==1,"no repeated resets while awaiting SESSION");
                        Check(link.Snapshot(2000000).BytesPerSecond>0 && link.Snapshot(2000000).Accepted==5,"rates distinguish bytes from accepted messages");
                        link.Feed(normal,normal.Length,2100000); link.Feed(c4,c4.Length,2700001);
                        Check(p.Snapshot(2700001).Session==null && p.Snapshot(2700001).C4==null,"late bytes trigger expiry before feeding");
                        link.Failed("injected disconnect",2800000);
                        bool denied=false; try { link.Feed(normal,normal.Length,2800001); } catch(InvalidOperationException) { denied=true; } Check(denied,"closed connection cannot feed");
                    }
                });
                Group("production worker: failed open, disposal, reconnect, bootstrap gating, clean stop",delegate
                {
                    FakeSerial first=new FakeSerial {FailOpen=true},second=new FakeSerial(),third=new FakeSerial(); int index=0;
                    FakeSerial[] ports={first,second,third};
                    using(DataEngine engine=new DataEngine(root,build,delegate(string name,int baud)
                        { int i=Interlocked.Increment(ref index)-1; if(i>=ports.Length) throw new IOException("no more fake connections"); return ports[i]; },20))
                    {
                        second.Push(normal); // Existing driver backlog must be discarded at Open.
                        engine.StartSerial("FAKE_COM",115200,true);
                        Until(delegate { return second.Discarded; },"successful second open");
                        Check(first.Disposed && engine.Pipe.Snapshot(engine.Now).Session==null,"failed-open resource disposed and backlog discarded");
                        second.Push(normal); Until(delegate { return engine.Pipe.Snapshot(engine.Now).Status!=null; },"worker accepts D0");
                        CaptureRecorder r=engine.Pipe.StartCapture(output,engine.Now);
                        second.Push(c4.Take(23).ToArray()); second.Push(new IOException("injected unplug"));
                        Until(delegate { return third.Discarded; },"worker reconnects after unplug");
                        Check(second.Disposed && engine.Pipe.Snapshot(engine.Now).Session==null,"reconnect clears registration");
                        Check(r.Wait(3000) && r.Completed,"unplug recording finalized");
                        third.Push(c4); Until(delegate { return engine.SerialStatus.Bytes>=(ulong)(normal.Length+23+c4.Length); },"orphan delivered");
                        Check(engine.Pipe.Snapshot(engine.Now).C4==null,"worker requires new SESSION");
                        third.Push(normal); Until(delegate { return engine.Pipe.Snapshot(engine.Now).C4!=null; },"worker restored");
                        SerialLinkSnapshot s=engine.SerialStatus;
                        Check(s.Connections==2 && s.Attempts==3 && s.Failures==2,"open/reconnect failure accounting");
                        using(Dashboard form=new Dashboard(engine,false))
                        {
                            form.ShowInTaskbar=false; form.Opacity=0; form.Text+=" [D4模拟串口测试，非实物]"; form.Show();
                            System.Windows.Forms.Application.DoEvents(); form.PerformLayout(); form.RefreshView();
                            using(Bitmap bitmap=new Bitmap(form.Width,form.Height)) { form.DrawToBitmap(bitmap,new Rectangle(Point.Empty,bitmap.Size)); bitmap.Save(Path.Combine(output,"d4_serial_preview.png"),System.Drawing.Imaging.ImageFormat.Png); }
                        }
                        engine.Stop(); int attempts=index; Thread.Sleep(60);
                        Check(!engine.Active && third.Disposed && index==attempts,"stop joins reader without reopening");
                        Check(engine.Pipe.Snapshot(engine.Now).Session==null,"stop cannot retain a live session");
                    }
                });
                Group("silent open COM is reopened after three seconds without accepted D0",delegate
                {
                    FakeSerial first=new FakeSerial(),second=new FakeSerial(); int count=0;
                    using(DataEngine engine=new DataEngine(root,build,delegate
                        { return Interlocked.Increment(ref count)==1?first:second; },20))
                    {
                        engine.StartSerial("FAKE_COM",115200,true); Until(delegate { return first.Discarded; },"first silent connection opened");
                        first.Push(normal); Until(delegate { return engine.Pipe.Snapshot(engine.Now).Session!=null; },"watchdog baseline registered");
                        Until(delegate { return second.Discarded; },"watchdog reopens still-open silent COM",4500);
                        Check(first.Disposed && engine.Pipe.Snapshot(engine.Now).Session==null,"watchdog drops old receiver");
                        Check(engine.SerialStatus.Silences==1 && engine.SerialStatus.Connections==2,"watchdog boundaries counted");
                        engine.Stop();
                    }
                });
                Group("stop during retry and pending open",delegate
                {
                    using(DataEngine engine=new DataEngine(root,build,delegate { throw new IOException("injected unavailable COM"); },1000))
                    {
                        engine.StartSerial("FAKE_COM",115200); Until(delegate { return engine.SerialStatus.Failures>=1; },"waiting retry");
                        Stopwatch w=Stopwatch.StartNew(); engine.Stop(); Check(w.ElapsedMilliseconds<700,"retry delay is cancelable");
                    }
                    FakeSerial blocked=new FakeSerial {BlockOpen=true};
                    using(DataEngine engine=new DataEngine(root,build,delegate { return blocked; },1000))
                    {
                        engine.StartSerial("FAKE_COM",115200); Until(delegate { return blocked.OpenCalled; },"pending fake open");
                        engine.Stop(); Check(blocked.Disposed && !blocked.Discarded && !engine.Active,"canceled open cannot publish registration");
                    }
                });
                log.Append("RESULT PASS groups=").Append(groups).Append(" checks=").Append(checks).Append('\n');
                log.Append("All transports were fakes; no physical COM, HC-05, radio timing or sensor accuracy tested.\n");
                TaskFiles.TextNew(Path.Combine(output,"d4_serial_test.log"),log.ToString()); TaskFiles.TextNew(Path.Combine(output,"D4_COMPLETE.txt"),"PASS software serial lifecycle\n"); return 0;
            }
            catch(Exception e) { log.Append(e.ToString()).Append('\n'); TaskFiles.TextNew(Path.Combine(output,"d4_serial_test.log"),log.ToString()); return 1; }
        }
    }
}
