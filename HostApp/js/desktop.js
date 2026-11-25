import { TelemetryFramer } from "./wire.js";

export const desktopHost = location.hostname === "spinal-imu.invalid" ? window.chrome?.webview : null;

function decodeBytes(base64) {
  return Uint8Array.from(atob(base64), (character) => character.charCodeAt(0));
}

export class DesktopSerialSource {
  constructor(callbacks) {
    this.callbacks = callbacks;
    this.requestId = 0;
    desktopHost.addEventListener("message", ({ data }) => this.receive(data));
  }

  connect(baudRate) {
    this.framer = new TelemetryFramer({
      ...this.callbacks, onError: this.callbacks.onProtocolError,
    });
    return new Promise((resolve, reject) => {
      this.opening = { resolve, reject };
      desktopHost.postMessage(`open|${++this.requestId}|${baudRate}`);
    });
  }

  disconnect() {
    return new Promise((resolve) => {
      this.closing = resolve;
      desktopHost.postMessage(`close|${this.requestId}`);
    });
  }

  receive(message) {
    if (typeof message !== "string" || message.startsWith("measure-")) return;
    const [kind, id, ...payload] = message.split("|");
    if (id !== String(this.requestId)) return;
    if (kind === "data") {
      const bytes = decodeBytes(payload[0]);
      this.callbacks.onBytes(bytes.length);
      this.framer.push(bytes);
    } else if (kind === "opened") {
      this.callbacks.onState("connected");
      this.opening?.resolve();
      this.opening = null;
    } else if (kind === "closed" || kind === "cancel" || kind === "error") {
      this.callbacks.onState("disconnected");
      if (kind === "error") {
        const error = new Error(payload.join("|"));
        if (this.opening) this.opening.reject(error);
        else this.callbacks.onError(error);
      } else this.opening?.resolve();
      this.opening = null;
      this.closing?.();
      this.closing = null;
    }
  }
}

export async function attachMeasurement({ start, end, onBytes, onFrame, onBoundary, onIssue }) {
  const { WireInput } = await import("./local-wire.js");
  let input;
  desktopHost.addEventListener("message", ({ data }) => {
    if (typeof data !== "string") return;
    if (data.startsWith("measure-start|")) {
      start(data.slice("measure-start|".length));
      input = new WireInput({
        onLine() {},
        onFrame(frame, note) {
          const missing = frame.invalid.map((node) => ({ node, valid: false, q: null, timestampUs: 0 }));
          onFrame({ ...frame, samples: [...frame.samples, ...missing] }, note);
        },
        onBoundary, onIssue,
      });
    } else if (data.startsWith("measure-data|") && input) {
      const bytes = decodeBytes(data.slice("measure-data|".length));
      onBytes(bytes.length);
      input.push(bytes);
    } else if (data === "measure-end") {
      input = null;
      end();
    }
  });
  desktopHost.postMessage("measurement-ready");
}
