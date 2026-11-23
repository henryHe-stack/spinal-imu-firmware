using System;
namespace NeckF103
{
    // State is serialized with Pipeline.Sync. Only the serial worker writes COM.
    public sealed class DeviceControl
    {
        private readonly Pipeline pipe;
        private WireMessage session; private ulong observed,lastArrival,lastSend;
        private uint nextId=1,pendingId,pendingOp; private byte[] pending,ack;
        private int attempts; private bool synced;
        public string Status="未连接支持控制的固件";
        public DeviceControl(Pipeline value) { pipe=value; }
        public void Reset()
        {
            lock(pipe.Sync) {
                if(pending!=null) pipe.Note("连接/会话中断：先前命令结果未知，不自动跨会话重试",lastArrival);
                session=null; pending=ack=null; synced=false; attempts=0; nextId=1;
                Status="等待支持控制的SESSION";
            }
        }
        private void Begin(uint opcode,uint[] args,ulong now)
        {
            if(session==null || pending!=null || nextId==0) throw new InvalidOperationException("控制未就绪或已有命令在途");
            pendingId=nextId++; pendingOp=opcode;
            pending=NativeBridge.Command(session.Boot,session.Session,session.Config,observed,pendingId,opcode,args);
            attempts=0; lastSend=now; Status="请求 "+pendingId+" / 操作 "+opcode+" 等待板端应答";
            pipe.Note(Status+" identity="+session.Boot+"/"+session.Session+"/"+session.Config+
                " args="+string.Join("/",args),now);
        }
        public bool Ready(ulong now)
        { lock(pipe.Sync) return session!=null && synced && pending==null && nextId!=0 && now>=lastArrival && now-lastArrival<=500000; }
        public void Request(uint opcode,uint key,uint value,ulong now)
        {
            lock(pipe.Sync) {
                bool fresh=session!=null && pending==null && nextId!=0 && now>=lastArrival && now-lastArrival<=500000;
                if(!fresh || (opcode!=1 && !synced)) throw new InvalidOperationException("等待控制登记完成、前一命令应答及新鲜串口数据");
                if(opcode<1 || opcode>7) throw new ArgumentOutOfRangeException("opcode");
                Begin(opcode,new uint[]{key,value,0,0},now);
            }
        }
        public void Receive(WireMessage m,ulong now)
        {
            lock(pipe.Sync) {
                observed=m.Emit; lastArrival=now;
                if(m.Type==1) {
                    if((m.Registration.Capabilities&16)==0) { Reset(); Status="此固件只支持显示，不支持板端命令"; return; }
                    if(session==null || session.Boot!=m.Boot || session.Session!=m.Session || session.Config!=m.Config) {
                        Reset(); session=m; observed=m.Emit; lastArrival=now;
                        Begin(1,new uint[4],now);
                    }
                    return;
                }
                if(session==null || m.Boot!=session.Boot || m.Session!=session.Session || m.Config!=session.Config) return;
                if(m.Type==4 && (session.Registration.Capabilities&32)!=0) {
                    ack=NativeBridge.Command(m.Boot,m.Session,m.Config,m.Emit,1,8,
                        new uint[]{(uint)m.Event.Id,(uint)(m.Event.Id>>32),0,0});
                }
                if(m.Type!=17 || pending==null || m.RequestId!=pendingId || m.Opcode!=pendingOp) return;
                pending=null;
                if(m.CommandResult==0) {
                    if(m.Opcode==1) { nextId=m.CommandArgs[0]==uint.MaxValue?0:Math.Max(nextId,m.CommandArgs[0]+1); synced=nextId!=0; }
                    Status="板端应答：操作="+m.Opcode+" 结果=0 参数="+string.Join("/",m.CommandArgs);
                    if(m.Opcode==2) Status+="；归零请求已接收，需等待零位成功";
                    if(m.Opcode==4 || m.Opcode==5 || m.Opcode==6) { synced=false; Status+="；等待新SESSION确认切换"; }
                } else Status="板端拒绝：操作="+m.Opcode+" 错误码="+m.CommandResult;
                pipe.Note(Status,now);
            }
        }
        public byte[] Next(ulong now)
        {
            lock(pipe.Sync) {
                if(session==null || now<lastArrival || now-lastArrival>500000) return null;
                if(ack!=null) { byte[] bytes=ack; ack=null; return bytes; }
                if(pending==null || (attempts!=0 && now-lastSend<400000)) return null;
                if(attempts>=3) {
                    Status="命令应答超时：执行结果未知，请查询状态后决定是否重试";
                    pipe.Note(Status,now); pending=null; synced=false;
                    if(nextId!=0) Begin(1,new uint[4],now); return null;
                }
                attempts++; lastSend=now; return (byte[])pending.Clone();
            }
        }
    }
}
