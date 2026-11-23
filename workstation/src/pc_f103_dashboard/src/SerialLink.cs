using System;
using System.IO.Ports;

namespace NeckF103
{
    // This seam exercises the production reconnect loop without opening a COM port.
    public interface ISerialConnection : IDisposable
    {
        void Open();
        void DiscardInput();
        int Read(byte[] bytes);
        void Write(byte[] bytes);
        void Close();
    }
    internal sealed class WindowsSerialConnection : ISerialConnection
    {
        private readonly SerialPort port;
        public WindowsSerialConnection(string name,int baud)
        {
            port=new SerialPort(name,baud,Parity.None,8,StopBits.One);
            port.ReadTimeout=100; port.WriteTimeout=200;
            port.Handshake=Handshake.None; port.DtrEnable=false; port.RtsEnable=false;
            port.ReadBufferSize=16384; port.DiscardNull=false;
        }
        public void Open() { port.Open(); }
        public void DiscardInput() { port.DiscardInBuffer(); }
        public int Read(byte[] bytes) { return port.Read(bytes,0,bytes.Length); }
        public void Write(byte[] bytes) { port.Write(bytes,0,bytes.Length); }
        public void Close() { port.Close(); }
        public void Dispose() { port.Dispose(); }
    }
    public sealed class SerialLinkSnapshot
    {
        public string State;
        public ulong Attempts,Connections,Failures,Silences,Bytes,Accepted;
        public double BytesPerSecond,MessagesPerSecond;
    }
    public sealed class SerialLinkSession
    {
        public const ulong SilenceTimeoutUs=500000;
        public const ulong ReopenTimeoutUs=3000000;
        private readonly object gate=new object(); private readonly Pipeline pipe; private readonly string label;
        private bool opened,silenceLatched; private ulong lastValid,windowStart,windowBytes,windowMessages;
        private ulong attempts,connections,failures,silences,bytes,accepted;
        private double byteRate,messageRate; private string state="未连接";
        public SerialLinkSession(Pipeline pipeline,string name) { pipe=pipeline; label=name; }
        private void Rates(ulong now)
        {
            if(now>=windowStart && now-windowStart>=1000000)
            {
                double seconds=(now-windowStart)*1e-6;
                byteRate=windowBytes/seconds; messageRate=windowMessages/seconds;
                windowBytes=windowMessages=0; windowStart=now;
            }
        }
        public void Attempt(ulong now)
        { lock(gate) { attempts++; Rates(now); state=label+"：正在打开COM（尝试 "+attempts+"）"; } }
        public void Opened(ulong now)
        {
            lock(gate)
            {
                opened=true; silenceLatched=false; lastValid=now; connections++;
                state=label+"：COM已打开，等待SESSION";
                pipe.EndSerialSegment(state+"；连接序号="+connections,now);
            }
        }
        public void Tick(ulong now)
        {
            lock(gate)
            {
                Rates(now);
                // Use accepted protocol messages, not arbitrary bytes: noise must
                // not keep an old registration or pose alive. Test before Feed too.
                if(opened && !silenceLatched && now>=lastValid && now-lastValid>SilenceTimeoutUs)
                {
                    silences++; silenceLatched=true;
                    state=label+"：无有效D0消息，等待新SESSION";
                    pipe.EndSerialSegment(state+"（超过500ms；本段记录结束）",now);
                }
            }
        }
        public void Feed(byte[] data,int count,ulong now)
        {
            lock(gate)
            {
                if(!opened) throw new InvalidOperationException("Serial link is not open");
                if(data==null || count<=0 || count>data.Length || count>4096) throw new ArgumentOutOfRangeException("count");
                Tick(now);
                ulong before=pipe.AcceptedMessages;
                pipe.Feed(data,count,now);
                ulong received=pipe.AcceptedMessages-before;
                bytes+=(ulong)count; windowBytes+=(ulong)count;
                if(received!=0)
                {
                    accepted+=received; windowMessages+=received; lastValid=now; silenceLatched=false;
                    state=label+"：D0已登记 / 接收中";
                }
            }
        }
        public void Failed(string reason,ulong now)
        {
            lock(gate)
            {
                opened=false; failures++;
                state=label+"：连接失败或中断，1秒后重试";
                pipe.EndSerialSegment(state+"："+reason,now);
            }
        }
        public bool RequiresReopen(ulong now)
        { lock(gate) return opened && now>=lastValid && now-lastValid>=ReopenTimeoutUs; }
        public void Stopped(ulong now)
        {
            lock(gate)
            {
                opened=false; state=label+"：已停止";
                pipe.EndSerialSegment(state,now);
            }
        }
        public SerialLinkSnapshot Snapshot(ulong now)
        {
            lock(gate)
            {
                bool expired=now>=windowStart && now-windowStart>=2000000;
                return new SerialLinkSnapshot { State=state,Attempts=attempts,Connections=connections,
                    Failures=failures,Silences=silences,Bytes=bytes,Accepted=accepted,
                    BytesPerSecond=expired?0:byteRate,MessagesPerSecond=expired?0:messageRate };
            }
        }
    }
}
