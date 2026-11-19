import { WireInput } from "./local-wire.js";
export class SerialSource {
  constructor({ onLine, onFrame, onBoundary, onIssue, onBytes, onState, onError }) {
    this.callbacks = {onLine,onFrame,onBoundary,onIssue};
    this.onBytes = onBytes;
    this.onState = onState;
    this.onError = onError;
    this.port = null;
    this.reader = null;
    this.keepReading = false;
    this.desktop = location.hostname === "spinal-imu.invalid" && window.chrome?.webview;
    this.requestId = 0;
    this.desktop?.addEventListener?.("message", event => this.desktopMessage(event.data));
  }

  static supported() {
    return Boolean((location.hostname === "spinal-imu.invalid" && window.chrome?.webview) || "serial" in navigator);
  }

  desktopMessage(message) {
    if (typeof message !== "string") return;
    const [kind,id,...body] = message.split("|");
    if (id !== String(this.requestId)) return;
    if (kind === "data" && this.keepReading) {
      const bytes = Uint8Array.from(atob(body[0]), char => char.charCodeAt(0));
      this.onBytes(bytes.length); this.input.push(bytes);
    } else if (kind === "opened") {
      this.keepReading = true; this.onState("connected");
      this.pendingOpen?.resolve(); this.pendingOpen = null;
    } else if (["cancel","closed","error"].includes(kind)) {
      this.keepReading = false; this.onState("disconnected");
      if (kind === "error") {
        const error = new Error(body.join("|"));
        if (this.pendingOpen) this.pendingOpen.reject(error); else this.onError(error);
      } else this.pendingOpen?.resolve();
      this.pendingOpen = null;
      this.pendingClose?.(); this.pendingClose = null;
    }
  }

  async connect(baudRate) {
    if (this.desktop) {
      this.input = new WireInput(this.callbacks);
      const opened = new Promise((resolve,reject) => { this.pendingOpen={resolve,reject}; });
      this.desktop.postMessage(`open|${++this.requestId}|${baudRate}`);
      return opened;
    }
    if (!SerialSource.supported()) throw new Error("当前浏览器不支持 Web Serial，请使用桌面版 Chrome 或 Edge");
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    this.keepReading = true;
    this.onState("connected");
    this.reading = this.readLoop();
  }

  async disconnect() {
    if (this.desktop) {
      if (!this.keepReading) return;
      const closed = new Promise(resolve => { this.pendingClose=resolve; });
      this.desktop.postMessage(`close|${this.requestId}`);
      return closed;
    }
    this.keepReading = false;
    if (this.reader) {
      await this.reader.cancel().catch(() => {});
    }
    await this.reading;
    if (this.port) {
      await this.port.close().catch(() => {});
      this.port = null;
    }
    this.onState("disconnected");
  }

  async readLoop() {
    const input = new WireInput(this.callbacks);
    try {
      while (this.port?.readable && this.keepReading) {
        this.reader = this.port.readable.getReader();
        try {
          while (this.keepReading) {
            const { value, done } = await this.reader.read();
            if (done) break;
            this.onBytes(value.byteLength);
            input.push(value);
          }
        } finally {
          this.reader.releaseLock();
          this.reader = null;
        }
        break;
      }
    } catch (error) {
      if (this.keepReading) this.onError(error);
    } finally {
      if (this.keepReading) {
        this.keepReading = false;
        await this.port?.close().catch(()=>{});
        this.port = null;
        this.onState("disconnected");
      }
    }
  }
}
