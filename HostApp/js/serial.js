import { TelemetryFramer, encodeCommand } from "./wire.js";

export class SerialSource {
  constructor({ onLine, onFrame, onReply, onProtocolError, onBytes, onState, onError }) {
    this.onLine = onLine;
    this.onBytes = onBytes;
    this.onState = onState;
    this.onError = onError;
    this.port = null;
    this.onFrame = onFrame; this.onReply = onReply; this.onProtocolError = onProtocolError;
    this.reader = null;
    this.keepReading = false;
    this.writePending = Promise.resolve();
  }

  static supported() {
    return "serial" in navigator;
  }

  async connect(baudRate) {
    if (!SerialSource.supported()) throw new Error("当前浏览器不支持 Web Serial，请使用桌面版 Chrome 或 Edge");
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    this.keepReading = true;
    this.onState("connected");
    this.reading = this.readLoop();
  }

  async disconnect() {
    this.keepReading = false;
    await this.reader?.cancel().catch(() => {});
    await this.reading; // readLoop alone owns reader release and port close.
  }

  async readLoop() {
    const framer = new TelemetryFramer({ onLine: this.onLine, onFrame: this.onFrame,
      onReply: this.onReply, onError: this.onProtocolError });
    try {
      while (this.port?.readable && this.keepReading) {
        const reader = this.port.readable.getReader();
        this.reader = reader;
        try {
          while (this.keepReading) {
            const { value, done } = await reader.read();
            if (done) { this.keepReading = false; break; }
            this.onBytes?.(value.byteLength);
            framer.push(value);
          }
        } finally {
          reader.releaseLock();
          this.reader = null;
        }
      }
    } catch (error) {
      if (this.keepReading) this.onError(error);
    } finally {
      this.keepReading = false;
      await this.writePending.catch(() => {});
      const port = this.port; this.port = null;
      await port?.close().catch(() => {});
      this.onState("disconnected");
    }
  }

  sendCommand(id, requestId, value = 0) {
    // Serialize quick clicks across different buttons: Web Serial allows one
    // writer lock, and each binary command must remain contiguous.
    this.writePending = this.writePending.catch(() => {}).then(async () => {
      if (!this.keepReading || !this.port?.writable) throw new Error("串口未连接或不可写");
      const writer = this.port.writable.getWriter();
      try { await writer.write(encodeCommand(id, requestId, value)); }
      finally { writer.releaseLock(); }
    });
    return this.writePending;
  }
}
