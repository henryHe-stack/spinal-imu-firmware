export class SerialSource {
  constructor({ onLine, onBytes, onState, onError }) {
    this.onLine = onLine;
    this.onBytes = onBytes;
    this.onState = onState;
    this.onError = onError;
    this.port = null;
    this.reader = null;
    this.keepReading = false;
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
    this.readLoop();
  }

  async disconnect() {
    this.keepReading = false;
    if (this.reader) {
      await this.reader.cancel().catch(() => {});
      this.reader.releaseLock();
      this.reader = null;
    }
    if (this.port) {
      await this.port.close().catch(() => {});
      this.port = null;
    }
    this.onState("disconnected");
  }

  async readLoop() {
    const decoder = new TextDecoder();
    let buffer = "";
    try {
      while (this.port?.readable && this.keepReading) {
        this.reader = this.port.readable.getReader();
        try {
          while (this.keepReading) {
            const { value, done } = await this.reader.read();
            if (done) break;
            this.onBytes(value.byteLength);
            buffer += decoder.decode(value, { stream: true });
            const lines = buffer.split(/\r?\n/);
            buffer = lines.pop() ?? "";
            for (const line of lines) this.onLine(line);
          }
        } finally {
          this.reader.releaseLock();
          this.reader = null;
        }
      }
    } catch (error) {
      if (this.keepReading) this.onError(error);
    } finally {
      if (this.keepReading) await this.disconnect();
    }
  }
}
