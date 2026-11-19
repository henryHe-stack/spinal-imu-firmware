using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.InteropServices;

namespace NeckF103
{
    public sealed class AxisData { public int Valid,Instant,Alarm,Direction; public ulong WarningUs,DangerUs,ExtensionUs; }
    public sealed class SessionData
    {
        public uint Profile,Firmware,World,PeriodUs,QualityTimeoutUs,CutoffMilliHz,SkewUs,C4PeriodUs,RelativePeriodUs,StatusPeriodUs,Capabilities;
        public int Source,Mode,NodeMask,Origin; public ulong DurationUs; public int[] Thresholds=new int[4];
    }
    public sealed class C4Data
    {
        public ulong Input,Frame,Decision,Deadline; public uint Epoch,Reference,Quality,Filter,Posture,Mask;
        public int TimeFlags,Zero; public int[] Angles=new int[6]; public AxisData[] Axes=new AxisData[2];
    }
    public sealed class PairData { public int Mask,Status; public uint Flags; public int[] Angles=new int[6]; public ulong SkewUs; }
    public sealed class RelativeData
    { public ulong Input,Frame,Decision,Deadline; public uint Epoch,Reference,Quality,World; public int TimeFlags; public PairData[] Pairs=new PairData[4]; }
    public sealed class EventData
    { public ulong Id,Input,Frame,Time; public uint Reference,Epoch,Bits,Posture; public int Axis,Previous,Level,Direction,TimeFlags; }
    public sealed class NodeData
    { public ulong AccelTime,QuatTime; public uint Quality,Filter,Reference; public int Status,AccelCal,SystemCal,TimeFlags,Zero; }
    public sealed class StatusData
    {
        public ulong Input,Frame,Decision; public uint Epoch,Quality,PendingRef,InputDrops,TelemetryDrops,EventDrops,UartErrors;
        public int AccelMask,QuatMask,State,Accepted,ZeroRequest; public NodeData[] Nodes=new NodeData[3];
    }
    public sealed class WireMessage
    {
        public int Type; public uint Tx,Boot,Session,Config; public ulong Emit;
        public string Canonical; public SessionData Registration; public C4Data C4; public RelativeData Relative;
        public EventData Event; public StatusData Status;
        public uint RequestId,Opcode,CommandResult; public uint[] CommandArgs;
        private sealed class Reader
        {
            private readonly string[] fields; private int index=1;
            public Reader(string row) { fields=row.Split(','); if(fields[0]!="W1") throw new FormatException("Canonical row version"); }
            public ulong U() { return ulong.Parse(fields[index++],NumberStyles.None,CultureInfo.InvariantCulture); }
            public uint Q() { return checked((uint)U()); }
            public int I() { return int.Parse(fields[index++],NumberStyles.AllowLeadingSign,CultureInfo.InvariantCulture); }
            public void End() { if(index!=fields.Length) throw new FormatException("Canonical field count"); }
        }
        public static WireMessage Parse(string row)
        {
            Reader r=new Reader(row); WireMessage m=new WireMessage(); m.Canonical=row;
            m.Type=r.I(); m.Tx=r.Q(); m.Boot=r.Q(); m.Session=r.Q(); m.Config=r.Q(); m.Emit=r.U();
            if(m.Type==1)
            {
                SessionData s=new SessionData(); s.Profile=r.Q(); s.Firmware=r.Q(); s.World=r.Q();
                s.Source=r.I(); s.Mode=r.I(); s.NodeMask=r.I(); s.Origin=r.I();
                s.PeriodUs=r.Q(); s.QualityTimeoutUs=r.Q(); s.CutoffMilliHz=r.Q(); s.DurationUs=r.U();
                for(int i=0;i<4;i++) s.Thresholds[i]=r.I();
                s.SkewUs=r.Q(); s.C4PeriodUs=r.Q(); s.RelativePeriodUs=r.Q(); s.StatusPeriodUs=r.Q(); s.Capabilities=r.Q(); m.Registration=s;
            }
            else if(m.Type==2)
            {
                C4Data c=new C4Data(); c.Input=r.U(); c.Frame=r.U(); c.Decision=r.U(); c.Deadline=r.U();
                c.Epoch=r.Q(); c.Reference=r.Q(); c.Quality=r.Q(); c.Filter=r.Q(); c.Posture=r.Q(); c.Mask=r.Q(); c.TimeFlags=r.I(); c.Zero=r.I();
                for(int i=0;i<6;i++) c.Angles[i]=r.I();
                for(int i=0;i<2;i++) { AxisData a=new AxisData(); a.Valid=r.I(); a.Instant=r.I(); a.Alarm=r.I(); a.Direction=r.I(); a.WarningUs=r.U(); a.DangerUs=r.U(); a.ExtensionUs=r.U(); c.Axes[i]=a; }
                m.C4=c;
            }
            else if(m.Type==3)
            {
                RelativeData v=new RelativeData(); v.Input=r.U(); v.Frame=r.U(); v.Decision=r.U(); v.Deadline=r.U();
                v.Epoch=r.Q(); v.Reference=r.Q(); v.Quality=r.Q(); v.World=r.Q(); v.TimeFlags=r.I();
                for(int i=0;i<4;i++) { PairData p=new PairData(); p.Mask=r.I(); p.Status=r.I(); p.Flags=r.Q(); for(int j=0;j<6;j++) p.Angles[j]=r.I(); p.SkewUs=r.U(); v.Pairs[i]=p; } m.Relative=v;
            }
            else if(m.Type==4)
            {
                EventData e=new EventData(); e.Id=r.U(); e.Input=r.U(); e.Frame=r.U(); e.Time=r.U();
                e.Reference=r.Q(); e.Epoch=r.Q(); e.Bits=r.Q(); e.Posture=r.Q();
                e.Axis=r.I(); e.Previous=r.I(); e.Level=r.I(); e.Direction=r.I(); e.TimeFlags=r.I(); m.Event=e;
            }
            else if(m.Type==5)
            {
                StatusData s=new StatusData(); s.Input=r.U(); s.Frame=r.U(); s.Decision=r.U(); s.Epoch=r.Q(); s.Quality=r.Q();
                s.AccelMask=r.I(); s.QuatMask=r.I(); s.State=r.I(); s.Accepted=r.I(); s.PendingRef=r.Q(); s.ZeroRequest=r.I();
                s.InputDrops=r.Q(); s.TelemetryDrops=r.Q(); s.EventDrops=r.Q(); s.UartErrors=r.Q();
                for(int i=0;i<3;i++) { NodeData n=new NodeData(); n.AccelTime=r.U(); n.QuatTime=r.U(); n.Quality=r.Q(); n.Filter=r.Q(); n.Reference=r.Q(); n.Status=r.I(); n.AccelCal=r.I(); n.SystemCal=r.I(); n.TimeFlags=r.I(); n.Zero=r.I(); s.Nodes[i]=n; } m.Status=s;
            }
            else if(m.Type==16 || m.Type==17) {
                m.RequestId=r.Q(); m.Opcode=r.Q(); m.CommandResult=r.Q(); m.CommandArgs=new uint[4];
                for(int i=0;i<4;i++) m.CommandArgs[i]=r.Q();
            }
            else throw new FormatException("Unsupported canonical type");
            r.End(); return m;
        }
    }
    public sealed class BridgeSnapshot { public byte[] Prefix,Session; public uint LastTx; public ulong LastEmit; }
    public sealed class NativeBridge : IDisposable
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate void Callback(int result,int boundary,ulong time,[MarshalAs(UnmanagedType.LPStr)] string row,IntPtr user);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern IntPtr neck_host_create();
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern void neck_host_destroy(IntPtr h);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_feed(IntPtr h,byte[] b,uint n,ulong t,Callback cb,IntPtr user);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_gap(IntPtr h,ulong t,Callback cb,IntPtr user);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern uint neck_host_poll(IntPtr h,ulong t);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_stats(IntPtr h,[Out] ulong[] a,uint n);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_snapshot(IntPtr h,[Out] byte[] p,uint cap,out uint n,[Out] byte[] s,uint scap,out uint sn,out uint tx,out ulong emit);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_seed(IntPtr h,uint tx,ulong emit);
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_command(uint boot,uint session,uint config,ulong observed,uint request,uint opcode,uint[] args,[Out] byte[] bytes,uint capacity);
        public static byte[] Command(uint boot,uint session,uint config,ulong observed,uint request,uint opcode,uint[] args)
        {
            if(args==null || args.Length!=4) throw new ArgumentException("Command args");
            byte[] bytes=new byte[62]; int size=neck_host_command(boot,session,config,observed,request,opcode,args,bytes,(uint)bytes.Length);
            if(size!=bytes.Length) throw new InvalidOperationException("Command encode failed");
            return bytes;
        }
        [DllImport("neck_wire_host.dll",CallingConvention=CallingConvention.Cdecl)] private static extern int neck_host_profile(IntPtr h,double[] q,uint[] settings,int registered);
        public void SetProfile(MeasurementProfile p)
        {
            if(neck_host_profile(handle,p.Rotations(),p.Settings(),p.Registered?1:0)!=1)
                throw new ArgumentException("检查单位四元数、截止频率 100–20000 mHz、持续时间 100–120000 ms 和阈值（危险值大于警告值，上限 90000）");
        }
        private IntPtr handle; private readonly Callback callback;
        public Action<WireMessage,int,bool,ulong> Received; public string Error="";
        public NativeBridge() { handle=neck_host_create(); if(handle==IntPtr.Zero) throw new OutOfMemoryException("Native bridge"); callback=OnRow; }
        private void OnRow(int result,int boundary,ulong time,string row,IntPtr user)
        { try { if(Received!=null) Received(WireMessage.Parse(row),result,boundary!=0,time); } catch(Exception e) { Error=e.Message; } }
        public void Feed(byte[] bytes,int count,ulong time)
        { if(handle==IntPtr.Zero || neck_host_feed(handle,bytes,checked((uint)count),time,callback,IntPtr.Zero)<0) throw new InvalidOperationException("Native feed (max 4096 bytes)"); if(Error.Length!=0) throw new FormatException(Error); }
        public void Gap(ulong time) { if(neck_host_gap(handle,time,callback,IntPtr.Zero)<0) throw new InvalidOperationException("Native gap"); if(Error.Length!=0) throw new FormatException(Error); }
        public uint Poll(ulong time) { return neck_host_poll(handle,time); }
        public ulong[] Stats() { ulong[] a=new ulong[12]; if(neck_host_stats(handle,a,12)!=12) throw new InvalidOperationException("Native stats"); return a; }
        public BridgeSnapshot Snapshot()
        {
            byte[] p=new byte[290],s=new byte[290]; uint n,sn,tx; ulong emit;
            if(neck_host_snapshot(handle,p,290,out n,s,290,out sn,out tx,out emit)!=1) throw new InvalidOperationException("Native snapshot");
            Array.Resize(ref p,(int)n); Array.Resize(ref s,(int)sn); return new BridgeSnapshot { Prefix=p,Session=s,LastTx=tx,LastEmit=emit };
        }
        public void Seed(uint tx,ulong emit) { if(neck_host_seed(handle,tx,emit)!=1) throw new FormatException("Invalid capture seed"); }
        public void Dispose() { if(handle!=IntPtr.Zero) { neck_host_destroy(handle); handle=IntPtr.Zero; } }
    }
    public sealed class PlotSample { public double Time; public double[] Values; }
    public sealed class ViewSnapshot
    {
        public WireMessage Session,C4,Relative,Status; public PlotSample[][] Plots;
        public string[] Events; public ulong[] Stats; public uint HostFlags; public ulong Messages,AlarmEvents,OldEpochDrops;
        public string Notice; public double Time;
    }
    public sealed class ViewModel
    {
        public WireMessage Session,C4,Relative,Status; public ulong Messages,AlarmEvents,OldEpochDrops;
        public string Notice="等待SESSION登记"; public double Time;
        private readonly List<PlotSample>[] plots=new List<PlotSample>[5];
        private readonly Queue<string> events=new Queue<string>(); private readonly HashSet<ulong> eventIds=new HashSet<ulong>(); private readonly Queue<ulong> ids=new Queue<ulong>();
        private uint epoch,c4Ref,relativeRef; private bool epochKnown; private ulong origin;
        public ViewModel() { for(int i=0;i<5;i++) plots[i]=new List<PlotSample>(); }
        private double T(ulong value) { return value>=origin?(value-origin)*1e-6:-(double)(origin-value)*1e-6; }
        public void Note(string text) { Notice=text; events.Enqueue(text); while(events.Count>200) events.Dequeue(); }
        public string[] Journal() { return events.ToArray(); }
        public void ClearSegment()
        { for(int i=0;i<5;i++) plots[i].Clear(); C4=Relative=Status=null; epochKnown=false; c4Ref=relativeRef=0; eventIds.Clear(); ids.Clear(); Time=0; }
        private void Add(int group,double t,double[] values)
        { List<PlotSample> p=plots[group]; p.Add(new PlotSample { Time=t,Values=values }); while(p.Count>1200 || (p.Count>1 && p[0].Time<t-30)) p.RemoveAt(0); }
        private void Break(int group,double t) { Add(group,t,new double[] { double.NaN,double.NaN,double.NaN,double.NaN,double.NaN,double.NaN }); }
        private bool Epoch(uint value,double t)
        {
            if(epochKnown && value!=epoch) { uint delta=unchecked(value-epoch); if(delta>=0x80000000U) { OldEpochDrops++; return false; }
                for(int i=0;i<5;i++) Break(i,t); C4=Relative=null; }
            epoch=value; epochKnown=true; return true;
        }
        public static double Value(int value,bool valid) { return valid && value!=int.MinValue?value/1000.0:double.NaN; }
        public void Accept(WireMessage m,int result,bool boundary)
        {
            if(result!=0 && result!=1) return;
            Messages++;
            if(m.Type==1)
            {
                if(Session==null || boundary) { ClearSegment(); origin=m.Emit; Note("会话登记 boot="+m.Boot+" session="+m.Session+" source="+(m.Registration.Source==0?"SIM":"HARDWARE标签")); }
                Session=m; return;
            }
            double emitted=T(m.Emit);
            if(m.Type==4)
            {
                Time=Math.Max(Time,emitted);
                EventData e=m.Event; if(!eventIds.Add(e.Id)) return;
                ids.Enqueue(e.Id); if(ids.Count>4096) eventIds.Remove(ids.Dequeue()); AlarmEvents++;
                string kind=(e.Bits&8)!=0?"失效" : (e.Bits&2)!=0?"解除" : (e.Bits&4)!=0?"升/降级" : "触发";
                Note("t="+T(e.Time).ToString("F3",CultureInfo.InvariantCulture)+"s "+(e.Axis==0?"前屈":"侧倾")+" "+kind+" "+e.Previous+"→"+e.Level+" bits="+e.Bits+" ref="+e.Reference+" (boot "+m.Boot+")");
                return;
            }
            if(m.Type==2)
            {
                C4Data c=m.C4; double t=T((c.TimeFlags&2)!=0?c.Decision:m.Emit); if(!Epoch(c.Epoch,t)) return;
                Time=emitted; if(c.Reference!=c4Ref) { Break(0,t); c4Ref=c.Reference; } C4=m;
                double[] v=new double[6]; for(int i=0;i<6;i++) v[i]=Value(c.Angles[i],(c.Mask&(1U<<i))!=0); Add(0,t,v);
            }
            else if(m.Type==3)
            {
                RelativeData r=m.Relative; double t=T((r.TimeFlags&2)!=0?r.Decision:m.Emit); if(!Epoch(r.Epoch,t)) return;
                Time=emitted; if(r.Reference!=relativeRef) { for(int i=1;i<5;i++) Break(i,t); relativeRef=r.Reference; } Relative=m;
                for(int i=0;i<4;i++) { PairData p=r.Pairs[i]; double[] v=new double[6]; for(int j=0;j<6;j++) v[j]=Value(p.Angles[j],(p.Mask&(j<3?1:2))!=0); Add(i+1,t,v); }
            }
            else if(m.Type==5) { if(Epoch(m.Status.Epoch,emitted)) { Status=m; Time=emitted; } }
        }
        public ViewSnapshot Snapshot(ulong[] stats,uint hostFlags)
        {
            PlotSample[][] p=new PlotSample[5][]; for(int i=0;i<5;i++) p[i]=plots[i].ToArray();
            return new ViewSnapshot { Session=Session,C4=C4,Relative=Relative,Status=Status,Plots=p,Events=events.ToArray(),
                Stats=stats,HostFlags=hostFlags,Messages=Messages,AlarmEvents=AlarmEvents,OldEpochDrops=OldEpochDrops,Notice=Notice,Time=Time };
        }
    }
}
