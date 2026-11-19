using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;

namespace NeckF103
{
    internal static class SelfTests
    {
        private static int checks,groups; private static readonly StringBuilder log=new StringBuilder();
        private static void Check(bool value,string name) { checks++; if(!value) throw new Exception("FAIL: "+name); }
        private static void Group(string name,Action test) { test(); groups++; log.Append("PASS ").Append(name).Append('\n'); }
        private static void Feed(NativeBridge bridge,byte[] bytes,ulong time)
        { int pos=0,k=0; int[] sizes={1,7,38,3,289,19,4096}; while(pos<bytes.Length) { int n=Math.Min(sizes[k++%sizes.Length],bytes.Length-pos); byte[] b=new byte[n]; Array.Copy(bytes,pos,b,0,n); bridge.Feed(b,n,time); pos+=n; } }
        private static List<WireMessage> Decode(string file)
        { List<WireMessage> got=new List<WireMessage>(); using(NativeBridge b=new NativeBridge()) { b.Received=delegate(WireMessage m,int result,bool boundary,ulong t){ got.Add(m); }; Feed(b,File.ReadAllBytes(file),1000); b.Gap(1000); } return got; }
        private static string[] Reference(string root,string build,string relative)
        {
            ProcessStartInfo info=new ProcessStartInfo(Path.Combine(build,"neck_demo.exe"),"--dump "+relative); info.WorkingDirectory=root;
            info.UseShellExecute=false; info.CreateNoWindow=true; info.RedirectStandardOutput=true; info.RedirectStandardError=true;
            using(Process p=Process.Start(info)) { string text=p.StandardOutput.ReadToEnd(),error=p.StandardError.ReadToEnd(); p.WaitForExit(); Check(p.ExitCode==0,"C dump: "+error); return text.Split(new char[]{'\r','\n'},StringSplitOptions.RemoveEmptyEntries); }
        }
        private static void Until(Func<bool> predicate,int ms,string name)
        { Stopwatch watch=Stopwatch.StartNew(); while(!predicate() && watch.ElapsedMilliseconds<ms) Thread.Sleep(20); Check(predicate(),name); }
        private static void ReplayInto(Pipeline pipeline,ReplayPlan plan,ulong until)
        {
            pipeline.Reset(); if(plan.Bootstrap.Length!=0) { pipeline.Feed(plan.Bootstrap,plan.Bootstrap.Length,0); if(plan.HasSeed) pipeline.Bridge.Seed(plan.SeedTx,plan.SeedEmit); }
            if(plan.Prefix.Length!=0) pipeline.Feed(plan.Prefix,plan.Prefix.Length,0);
            using(FileStream file=File.OpenRead(plan.Path)) foreach(ReplayChunk c in plan.Chunks)
            { if(c.Time>until) break; if(c.Length==0) pipeline.Gap(c.Time); else { byte[] bytes=new byte[c.Length]; file.Position=c.Offset; Check(file.Read(bytes,0,bytes.Length)==bytes.Length,"Replay read"); pipeline.Feed(bytes,bytes.Length,c.Time); } }
        }
        private static void Preview(DataEngine engine,string output,string name,ulong now)
        {
            using(Dashboard form=new Dashboard(engine,false))
            { form.ShowInTaskbar=false; form.Opacity=0; form.Show(); System.Windows.Forms.Application.DoEvents(); form.PerformLayout(); form.Render(engine.Pipe.Snapshot(now),true);
                using(Bitmap bitmap=new Bitmap(form.Width,form.Height)) { form.DrawToBitmap(bitmap,new Rectangle(Point.Empty,bitmap.Size)); bitmap.Save(Path.Combine(output,name),System.Drawing.Imaging.ImageFormat.Png); } }
        }
        public static int Run(string root,string output,string fixtures,string build)
        {
            try
            {
                root=Path.GetFullPath(root); output=Path.GetFullPath(output);
                if(!output.StartsWith(root.TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)) throw new IOException("Test output outside isolated workspace");
                string three=Path.Combine(output,"sim_three.bin"),single=Path.Combine(output,"sim_single.bin");
                List<WireMessage> threeMessages=null,singleMessages=null;
                Group("D0 five message types + integer/unit/zero semantics",delegate
                {
                    List<WireMessage> got=Decode(Path.Combine(fixtures,"normal_all.bin")); Check(got.Count==5,"five messages");
                    for(int i=0;i<5;i++) Check(got[i].Type==i+1,"type order"); Check(got[1].C4.Angles[2]==0 && got[1].C4.Mask==63,"valid numerical zero");
                    Check(got[2].Relative.Pairs[3].Angles[5]==-302,"signed relative integer"); Check(got[3].Event.Bits==10,"clear|invalidated");
                    Check(got[4].Status.Nodes[2].AccelTime==1000001 && got[4].Status.Nodes[1].SystemCal==255,"status fields and unknown calibration");
                    Check(ViewModel.Value(0,true)==0 && double.IsNaN(ViewModel.Value(0,false)),"zero vs invalid");
                    Check(double.IsNaN(ViewModel.Value(int.MinValue,true)),"invalid sentinel");
                });
                Group("C stream parser corruption recovery and partial gap",delegate
                {
                    string[] cases={"bad_crc_then_ok.bin","bad_length_then_ok.bin","unknown_version_then_ok.bin","truncated_then_ok.bin"};
                    for(int i=0;i<cases.Length;i++) using(NativeBridge b=new NativeBridge())
                    { int got=0; b.Received=delegate { got++; }; Feed(b,File.ReadAllBytes(Path.Combine(fixtures,cases[i])),100); b.Gap(200);
                        ulong[] s=b.Stats(); Check(got==1,"recover "+cases[i]); Check(s[i==0?1:i==1?2:i==2?3:7]>0,"diagnostic "+cases[i]); }
                });
                Group("shared C producer vs native/managed canonical rows",delegate
                {
                    threeMessages=Decode(three); singleMessages=Decode(single);
                    string relative=output.Substring(root.Length+1).Replace('\\','/');
                    string[][] references={Reference(root,build,relative+"/sim_three.bin"),Reference(root,build,relative+"/sim_single.bin")};
                    List<WireMessage>[] parsed={threeMessages,singleMessages};
                    for(int j=0;j<2;j++) { Check(references[j].Length==parsed[j].Count,"canonical count"); for(int i=0;i<references[j].Length;i++) Check(references[j][i]==parsed[j][i].Canonical,"canonical row "+j+"/"+i); }
                    TaskFiles.TextNew(Path.Combine(output,"canonical_three.csv"),string.Join("\n",references[0])+"\n"); TaskFiles.TextNew(Path.Combine(output,"canonical_single.csv"),string.Join("\n",references[1])+"\n");
                });
                Group("single C4 / three-node zero and actual C alarm events",delegate
                {
                    Check(threeMessages[0].Registration.Mode==3 && singleMessages[0].Registration.NodeMask==2,"mode registration");
                    foreach(List<WireMessage> list in new List<WireMessage>[] {threeMessages,singleMessages})
                    {
                        Check(list.Any(m=>m.Type==2 && m.C4.Zero==3 && (m.C4.Mask&48)==48),"C4 zero committed");
                        Check(list.Any(m=>m.Type==4 && m.Event.Level==2 && m.Emit==15290000),"actual warning 15.29s");
                        Check(list.Any(m=>m.Type==4 && m.Event.Level==3 && m.Emit==30120000),"actual danger 30.12s");
                        Check(list.Any(m=>m.Type==4 && m.Event.Bits==10 && m.Emit==34030000),"timeout invalidated clear 34.03s");
                        Check(list.Any(m=>m.Type==2 && m.Emit>=34030000 && m.Emit<35000000 && m.C4.Mask==0 && (m.C4.Quality&128)!=0),"input timeout invalid mask");
                    }
                    Check(!singleMessages.Any(m=>m.Type==3),"single no fabricated relative packets");
                    Check(threeMessages.Any(m=>m.Type==3 && m.Relative.Reference>0 && m.Relative.Pairs.All(p=>(p.Mask&2)!=0)),"three relative reference committed");
                    Check(threeMessages.Any(m=>m.Type==3 && m.Emit>=45000000 && m.Emit<47000000 && (m.Relative.Pairs[0].Mask&1)!=0 && (m.Relative.Pairs[3].Mask&1)!=0 && m.Relative.Pairs[1].Mask==0 && m.Relative.Pairs[2].Mask==0),"partial C4 failure preserves whole/base pairs");
                });
                Group("host stale independently from heartbeat / zero display gate",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        p.Feed(File.ReadAllBytes(Path.Combine(fixtures,"normal_all.bin")),814,1000); ViewSnapshot snap=p.Snapshot(251001); Check((snap.HostFlags&6)==6,"pose stale at 250ms"); Check((snap.HostFlags&1)==0,"link still fresh");
                        Check((p.Snapshot(501001).HostFlags&1)!=0,"link stale at 500ms");
                        p.Reset(); p.Feed(File.ReadAllBytes(Path.Combine(fixtures,"session.bin")),106,0); p.Feed(File.ReadAllBytes(Path.Combine(fixtures,"invalid_c4.bin")),170,100);
                        Check(p.View.C4.C4.Mask==0,"timeout mask"); Check(p.View.Snapshot(p.Bridge.Stats(),0).Plots[0].Last().Values.All(double.IsNaN),"invalid plot gaps not zeros");
                        WireMessage current=p.View.C4,old=WireMessage.Parse(p.View.C4.Canonical); double time=p.View.Time; old.C4.Epoch=1; old.Emit+=10000000;
                        p.View.Accept(old,0,false); Check(p.View.OldEpochDrops==1 && object.ReferenceEquals(p.View.C4,current) && p.View.Time==time,"old epoch cannot replace pose or shift plot time");
                    }
                });
                Group("bounded chart/event state and reference/boot segmentation",delegate
                {
                    ViewModel v=new ViewModel(); WireMessage sess=threeMessages[0]; v.Accept(sess,1,false);
                    WireMessage m=threeMessages.First(x=>x.Type==2 && x.Emit>3000000);
                    for(int i=0;i<4000;i++) { WireMessage copy=WireMessage.Parse(m.Canonical); copy.Emit+=(ulong)i*10000; copy.C4.Decision=copy.Emit; v.Accept(copy,0,false); }
                    Check(v.Snapshot(new ulong[12],0).Plots[0].Length<=1200,"chart bound");
                    WireMessage ev=threeMessages.First(x=>x.Type==4); v.Accept(ev,0,false); v.Accept(ev,0,false); Check(v.AlarmEvents==1,"event ID dedup");
                    for(int i=0;i<250;i++) v.Note("test "+i); Check(v.Snapshot(new ulong[12],0).Events.Length==200,"journal bound");
                    WireMessage newBoot=WireMessage.Parse(sess.Canonical); newBoot.Boot=999; v.Accept(newBoot,1,true);
                    Check(v.C4==null && v.Snapshot(new ulong[12],0).Plots[0].Length==0,"boot clears current/reference/curve segment");
                });
                Group("native session boundary rejects delayed retired sessions/data",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        byte[] normal=File.ReadAllBytes(Path.Combine(fixtures,"normal_all.bin")); p.Feed(normal,normal.Length,0);
                        byte[] newSession=File.ReadAllBytes(three).Take(106).ToArray(); p.Feed(newSession,newSession.Length,100);
                        Check(p.View.Session.Boot==101 && p.View.C4==null && p.View.Relative==null,"native new identity segmented");
                        p.Feed(normal,normal.Length,200); Check(p.View.Session.Boot==101 && p.View.C4==null,"retired backlog cannot rollback"); Check(p.Bridge.Stats()[9]>=5,"retired frames rejected");
                    }
                });
                Group("mid-frame recording snapshot, exact raw replay, gaps and CSV invalid blanks",delegate
                {
                    using(Pipeline p=new Pipeline())
                    {
                        byte[] bytes=File.ReadAllBytes(Path.Combine(fixtures,"normal_all.bin")); byte[] head=bytes.Take(123).ToArray(); p.Feed(head,head.Length,1000);
                        CaptureRecorder recorder=p.StartCapture(root,2000); Check(p.Bridge.Snapshot().Prefix.Length==17,"record prefix snapshot");
                        byte[] rest=bytes.Skip(123).ToArray(); p.Feed(rest,rest.Length,3000); p.Gap(4000); p.StopCapture(); Check(recorder.Wait(5000) && recorder.Completed,"record drain");
                        ReplayPlan plan=ReplayPlan.Load(recorder.DirectoryPath); Check(plan.Prefix.Length==17 && plan.Chunks.Last().Length==0,"replay prefix and gap marker");
                        List<string> rows=new List<string>(); using(NativeBridge b=new NativeBridge())
                        {
                            b.Feed(plan.Bootstrap,plan.Bootstrap.Length,0); b.Seed(plan.SeedTx,plan.SeedEmit); b.Received=delegate(WireMessage m,int r,bool boundary,ulong t) { rows.Add(m.Canonical); }; b.Feed(plan.Prefix,plan.Prefix.Length,0);
                            using(FileStream f=File.OpenRead(plan.Path)) foreach(ReplayChunk c in plan.Chunks) { if(c.Length==0) b.Gap(c.Time); else { byte[] data=new byte[c.Length]; f.Position=c.Offset; f.Read(data,0,data.Length); b.Feed(data,data.Length,c.Time); } }
                        }
                        Check(rows.SequenceEqual(File.ReadAllLines(Path.Combine(recorder.DirectoryPath,"decoded.csv"))),"decoded rows exact after replay");
                        TaskFiles.TextNew(Path.Combine(output,"capture_roundtrip_path.txt"),recorder.DirectoryPath+"\n");
                        p.Reset(); p.Feed(File.ReadAllBytes(Path.Combine(fixtures,"session.bin")),106,0); CaptureRecorder invalid=p.StartCapture(root,0);
                        p.Feed(File.ReadAllBytes(Path.Combine(fixtures,"invalid_c4.bin")),170,100); p.StopCapture(); Check(invalid.Wait(5000) && invalid.Completed,"invalid record complete");
                        string[] columns=File.ReadAllLines(Path.Combine(invalid.DirectoryPath,"angles.csv"))[1].Split(','); Check(columns.Skip(11).Take(6).All(x=>x==""),"invalid angles exported blank");
                    }
                });
                Group("record buffer overflow is visible FAILED, not COMPLETE",delegate
                {
                    string directory=TaskFiles.NewRun(root,"work","d1_overflow"); using(NativeBridge b=new NativeBridge())
                    { CaptureRecorder r=new CaptureRecorder(directory,b.Snapshot(),0,64); r.Raw(new byte[4096],4096,0); r.Stop(); Check(r.Wait(5000) && !r.Completed && r.Error.Length!=0,"overflow failed");
                        Check(File.Exists(Path.Combine(directory,"FAILED.txt")) && !File.Exists(Path.Combine(directory,"COMPLETE.txt")),"failure marker"); bool rejected=false; try { ReplayPlan.Load(directory); } catch(IOException) { rejected=true; } Check(rejected,"incomplete replay refused"); }
                });
                Group("offscreen UI normal/danger, single and invalid quality",delegate
                {
                    using(DataEngine engine=new DataEngine(root,build))
                    {
                        engine.Kind="REPLAY"; ReplayPlan plan=ReplayPlan.Load(three); engine.State="测试预览：按原C结果显示";
                        ReplayInto(engine.Pipe,plan,32000000); Preview(engine,output,"ui_three_danger.png",32000000);
                        ReplayInto(engine.Pipe,plan,34400000); Preview(engine,output,"ui_timeout.png",34400000);
                        ReplayInto(engine.Pipe,ReplayPlan.Load(single),16000000); Preview(engine,output,"ui_single.png",16000000);
                    }
                });
                Group("live C process, manual loss, capture, restart and replay seek",delegate
                {
                    using(DataEngine engine=new DataEngine(root,build))
                    {
                        engine.StartSimulation(false,true); Until(delegate { ViewSnapshot s=engine.Pipe.Snapshot(engine.Now); return s.C4!=null && s.C4.C4.Zero==3; },6500,"live zero commit");
                        uint boot=engine.Pipe.Snapshot(engine.Now).Session.Boot; CaptureRecorder r=engine.Pipe.StartCapture(root,engine.Now);
                        engine.Command("INPUT_OFF"); Until(delegate { ViewSnapshot s=engine.Pipe.Snapshot(engine.Now); return s.C4!=null && s.C4.C4.Mask==0 && (s.C4.C4.Quality&128)!=0; },1500,"live input timeout");
                        engine.Command("INPUT_ON"); Until(delegate { ViewSnapshot s=engine.Pipe.Snapshot(engine.Now); return s.C4!=null && (s.C4.C4.Mask&3)==3; },2000,"live input recovered");
                        engine.Command("TX_OFF"); Until(delegate { return (engine.Pipe.Snapshot(engine.Now).HostFlags&1)!=0; },1500,"live PC link stale");
                        engine.Command("TX_ON"); Until(delegate { return (engine.Pipe.Snapshot(engine.Now).HostFlags&1)==0; },1500,"live wire recovered");
                        engine.Stop(); Check(r.Wait(5000) && r.Completed,"live recording complete"); ReplayPlan recorded=ReplayPlan.Load(r.DirectoryPath); Check(recorded.Chunks.Count>5,"live captured chunks");
                        engine.StartSimulation(true,false); Until(delegate { return engine.Pipe.Snapshot(engine.Now).Session!=null; },2000,"restart new session");
                        ViewSnapshot restarted=engine.Pipe.Snapshot(engine.Now); Check(restarted.Session.Boot!=boot && restarted.Session.Registration.Mode==1 && restarted.Relative==null,"restart isolated single mode"); engine.Stop();
                        engine.StartSimulation(true,false); Until(delegate { ViewSnapshot s=engine.Pipe.Snapshot(engine.Now); return s.C4!=null && (s.C4.C4.Mask&3)==3; },2000,"live single quality ready");
                        Check(engine.Pipe.Snapshot(engine.Now).C4.C4.Reference==0,"no automatic zero without script"); engine.Command("ZERO");
                        Until(delegate { ViewSnapshot s=engine.Pipe.Snapshot(engine.Now); return s.C4!=null && s.C4.C4.Zero==3; },5000,"manual single zero committed"); Check(engine.Pipe.Snapshot(engine.Now).Relative==null,"single still no relative after zero"); engine.Stop();
                        engine.StartReplay(three); engine.PauseReplay(true); engine.Seek(32000000); Until(delegate { return engine.Now==32000000; },2000,"seek32s");
                        Check(engine.Pipe.Snapshot(engine.Now).C4.C4.Axes[0].Alarm==3,"seek rebuilt danger"); Thread.Sleep(70); Check(engine.Now==32000000,"pause freezes replay clock");
                        engine.Seek(4000000); Until(delegate { return engine.Now==4000000; },2000,"seek back4s"); Check(engine.Pipe.Snapshot(engine.Now).AlarmEvents==0,"no future events after backward seek");
                        engine.Speed(4); engine.PauseReplay(false); Until(delegate { return engine.Now>=4200000; },1500,"4x playback"); engine.Stop();
                        TaskFiles.TextNew(Path.Combine(output,"live_capture_path.txt"),r.DirectoryPath+"\n");
                    }
                });
                log.Append("RESULT PASS groups=").Append(groups).Append(" checks=").Append(checks).Append('\n');
                log.Append("three_messages=").Append(threeMessages.Count).Append(" single_messages=").Append(singleMessages.Count).Append('\n');
                TaskFiles.TextNew(Path.Combine(output,"dashboard_test.log"),log.ToString()); TaskFiles.TextNew(Path.Combine(output,"MANAGED_COMPLETE.txt"),"PASS\n"); return 0;
            }
            catch(Exception e) { log.Append(e.ToString()).Append('\n'); try { TaskFiles.TextNew(Path.Combine(output,"dashboard_test.log"),log.ToString()); } catch(IOException) { } return 1; }
        }
    }
}
