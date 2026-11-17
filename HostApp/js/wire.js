// Firmware v1, little endian. This byte framer does not decode binary as text.
// A synchronization pair selects binary mode; otherwise a newline selects
// legacy text. Corrupt binary payloads must never become text on a 0x0A byte.
export function crc16(bytes) {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit++) crc = ((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1) & 0xffff;
  }
  return crc;
}

export function encodeCommand(id, requestId, value = 0) {
  const bytes = new Uint8Array(17), view = new DataView(bytes.buffer);
  bytes.set([0xa5, 0x5a, 1, 16, 9, 0]);
  view.setUint32(6, requestId, true); bytes[10] = id; view.setUint32(11, value, true);
  view.setUint16(15, crc16(bytes.subarray(2, 15)), true);
  return bytes;
}

export function decodePacket(bytes) {
  if (bytes.length < 8 || bytes[0] !== 0xa5 || bytes[1] !== 0x5a || bytes[2] !== 1 ||
      bytes.length !== 8 + (bytes[4] | bytes[5] << 8) || bytes.length > 192 ||
      crc16(bytes.subarray(2, -2)) !== (bytes.at(-2) | bytes.at(-1) << 8)) throw new Error("二进制帧头、长度或 CRC 无效");
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (bytes[3] === 17) {
    if (bytes.length !== 18) throw new Error("应答长度不符");
    return { kind: "reply", requestId: view.getUint32(6, true), command: bytes[10], status: bytes[11], value: view.getUint32(12, true) };
  }
  if (bytes[3] !== 1) throw new Error("未知二进制帧类型");
  const count = bytes[10];
  if (!count || count > 3 || bytes.length !== 13 + count * 40) throw new Error("采样帧长度不符");
  const samples = [];
  for (let index = 0; index < count; index++) {
    const p = 11 + index * 40, node = ["C0", "C4", "C7"][bytes[p]];
    if (!node || samples.some((s) => s.node === node)) throw new Error("节点编号重复或无效");
    const flags = view.getUint16(p + 2, true), valid = Boolean(flags & 1) && !(flags & 4);
    const raw = [0, 1, 2, 3].map((i) => view.getInt16(p + 16 + i * 2, true));
    const norm = Math.hypot(...raw);
    if (valid && norm < 1) throw new Error("有效帧四元数为零");
    const q = valid ? Object.fromEntries(["w", "x", "y", "z"].map((axis, i) => [axis, raw[i] / norm])) : null;
    const rawCal = bytes[p + 37];
    samples.push({ node, flags, valid, q, timestampStartUs: view.getUint32(p + 4, true),
      timestampEndUs: view.getUint32(p + 8, true), timestampUs: view.getUint32(p + 12, true),
      acceleration: valid ? [0, 1, 2].map((i) => view.getInt16(p + 24 + i * 2, true) / 100) : null,
      gyroscope: valid ? [0, 1, 2].map((i) => view.getInt16(p + 30 + i * 2, true) / 16) : null,
      temperature: view.getInt8(p + 36), calibration: (rawCal >> 6) & 3, calibrationRaw: rawCal,
      systemStatus: bytes[p + 38], systemError: bytes[p + 39], tcaChannel: bytes[p + 1],
    });
  }
  return { kind: "samples", sequence: view.getUint32(6, true), samples, protocol: "BINARY v1", vectorUnits: "SI" };
}

// Extend a 32-bit device clock. Align all nodes to C0, including a frame
// straddling the wrap. Preserve raw values for reproducible CSV export.
export class DeviceClock {
  apply(frame) {
    const raw = frame.samples[0].timestampUs;
    if (this.previous !== undefined) {
      if (raw < this.previous && this.previous - raw > 0x80000000) this.epoch += 0x100000000;
      else if (raw < this.previous) this.epoch = 0; // Actual reboot/backward clock.
    } else this.epoch = 0;
    this.previous = raw;
    const anchor = this.epoch + raw;
    for (const sample of frame.samples) {
      const mid = sample.timestampUs;
      sample.timestampRawUs = mid;
      sample.timestampUs = anchor + ((mid - raw) | 0);
      sample.timestampStartUs = sample.timestampUs + ((sample.timestampStartUs - mid) | 0);
      sample.timestampEndUs = sample.timestampUs + ((sample.timestampEndUs - mid) | 0);
    }
    return frame;
  }
}

export class TelemetryFramer {
  constructor({ onFrame, onLine, onReply, onError }) {
    Object.assign(this, { onFrame, onLine, onReply, onError });
    this.buffer = []; this.mode = null; this.clock = new DeviceClock();
  }
  push(chunk) {
    for (const byte of chunk) {
      this.buffer.push(byte);
      this.drain();
      if (this.buffer.length > 4096) { this.buffer = []; this.onError?.(new Error("输入帧超长，已重新同步")); }
    }
  }
  drain() {
    while (this.buffer.length) {
      if (this.mode === "text") {
        if (this.buffer.at(-1) !== 10) return;
        this.onLine?.(new TextDecoder().decode(new Uint8Array(this.buffer)).trim()); this.buffer = []; return;
      }
      const magic = this.buffer.findIndex((b, i) => b === 0xa5 && this.buffer[i + 1] === 0x5a);
      if (magic < 0) {
        if (this.mode === "binary") { this.buffer = this.buffer.at(-1) === 0xa5 ? [0xa5] : []; return; }
        // Before locking to binary, allow ordinary legacy NDJSON/CSV lines.
        if (this.buffer.at(-1) === 10) { this.mode = "text"; continue; }
        return;
      }
      if (magic) this.buffer.splice(0, magic);
      this.mode = "binary";
      if (this.buffer.length < 6) return;
      const size = this.buffer[4] | this.buffer[5] << 8, total = size + 8;
      if (this.buffer[2] !== 1 || total > 192) {
        this.buffer.shift(); this.onError?.(new Error("二进制协议版本或长度无效，正在重新同步")); continue;
      }
      if (this.buffer.length < total) return;
      const bytes = new Uint8Array(this.buffer.slice(0, total));
      const expected = bytes[total - 2] | bytes[total - 1] << 8;
      if (crc16(bytes.subarray(2, total - 2)) !== expected) {
        this.buffer.shift(); this.onError?.(new Error("CRC 校验失败，已丢弃并重新同步")); continue;
      }
      this.buffer.splice(0, total); this.mode = "binary";
      try {
        const packet = decodePacket(bytes);
        if (packet.kind === "reply") this.onReply?.(packet); else this.onFrame?.(this.clock.apply(packet));
      } catch (error) { this.onError?.(error); }
    }
  }
}
