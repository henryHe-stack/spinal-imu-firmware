import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { crc16, decodePacket, DeviceClock, encodeCommand, TelemetryFramer } from "../js/wire.js";
import { signalMetadata } from "../js/signals.js";
import { SerialSource } from "../js/serial.js";

const fixture = JSON.parse(readFileSync(new URL("./fixtures/firmware-v1.json", import.meta.url)));
const bytes = (hex) => Uint8Array.from(Buffer.from(hex, "hex"));
function repair(packet) {
  new DataView(packet.buffer, packet.byteOffset, packet.byteLength).setUint16(packet.length - 2, crc16(packet.subarray(2, -2)), true);
  return packet;
}

test("JS command bytes exactly match C parser fixture and standard CRC check", () => {
  assert.equal(crc16(new TextEncoder().encode("123456789")), 0x29b1);
  assert.deepEqual(encodeCommand(5, 0x12345678, 20), bytes(fixture.command));
  assert.deepEqual(decodePacket(bytes(fixture.reply)), { kind: "reply", requestId: 0x12345678, command: 5, status: 0, value: 20 });
});

test("binary samples survive every transport split, embedded newline and glued replies", () => {
  const sample = bytes(fixture.sample), reply = bytes(fixture.reply);
  for (let split = 0; split <= sample.length; split++) {
    const frames = [], replies = [], errors = [];
    const framer = new TelemetryFramer({ onFrame: (f) => frames.push(f), onReply: (r) => replies.push(r), onError: (e) => errors.push(e) });
    framer.push(sample.subarray(0, split));
    framer.push(Uint8Array.from([...sample.subarray(split), ...reply, ...sample]));
    assert.equal(frames.length, 2); assert.equal(replies.length, 1); assert.equal(errors.length, 0);
    assert.equal(frames[0].samples[2].timestampUs, 1012250);
  }
});

test("invalid nodes stay null and valid zero/duplicate/corrupt packets are rejected", () => {
  const invalid = bytes(fixture.sample); invalid[13] = 4; // sample0.flags, low byte.
  const frame = decodePacket(repair(invalid));
  assert.equal(frame.samples[0].valid, false);
  assert.equal(frame.samples[0].q, null); assert.equal(frame.samples[0].acceleration, null);
  const zero = bytes(fixture.sample); zero.fill(0, 27, 35); assert.throws(() => decodePacket(repair(zero)));
  const duplicate = bytes(fixture.sample); duplicate[51] = 0; assert.throws(() => decodePacket(repair(duplicate)));
  const corrupt = bytes(fixture.sample); corrupt[27] ^= 1; assert.throws(() => decodePacket(corrupt));
  assert.throws(() => decodePacket(Uint8Array.of(0xa5, 0x5a)));
});

test("framer rejects malformed headers and resynchronizes following a truncated frame", () => {
  const sample = bytes(fixture.sample), frames = [];
  const framer = new TelemetryFramer({ onFrame: (f) => frames.push(f), onError: () => {} });
  const badVersion = sample.slice(); badVersion[2] = 2;
  const oversize = sample.slice(); oversize[4] = 255;
  framer.push(Uint8Array.from([...badVersion, ...oversize, ...sample.subarray(0, 25), ...sample, ...sample]));
  assert.equal(frames.length, 2);
});

test("legacy text stays available and overlong input is bounded", () => {
  const lines = [], errors = [];
  const framer = new TelemetryFramer({ onLine: (l) => lines.push(l), onError: (e) => errors.push(e) });
  framer.push(new TextEncoder().encode('{"seq":1}\nIMU,2,C0,0,1,0,0,0,3\n'));
  assert.deepEqual(lines, ['{"seq":1}', "IMU,2,C0,0,1,0,0,0,3"]);
  framer.push(new Uint8Array(9000).fill(65));
  assert.ok(errors.length >= 2); assert.ok(framer.buffer.length <= 4096);
});

test("node clocks align across intra-frame wrap and reboot is distinct from wrap", () => {
  const clock = new DeviceClock();
  const make = (times) => ({ samples: times.map((t) => ({ timestampUs: t, timestampStartUs: (t - 20) >>> 0, timestampEndUs: (t + 20) >>> 0 })) });
  const crossing = clock.apply(make([0xfffffff0, 0x10, 0x30]));
  assert.equal(crossing.samples[2].timestampUs - crossing.samples[0].timestampUs, 64);
  assert.equal(crossing.samples[1].timestampEndUs - crossing.samples[1].timestampStartUs, 40);
  const next = clock.apply(make([0x100, 0x120, 0x140]));
  assert.equal(next.samples[0].timestampUs, 0x100000100);
  assert.equal(next.samples[0].timestampRawUs, 0x100);
  const reboot = clock.apply(make([10, 30, 50])); assert.equal(reboot.samples[0].timestampUs, 10);
});

test("chart metadata distinguishes binary SI vectors from legacy values", () => {
  assert.equal(signalMetadata("acceleration", { vectorUnits: "SI" }).unit, "m/s²");
  assert.equal(signalMetadata("gyroscope", { vectorUnits: "SI" }).unit, "°/s");
  assert.equal(signalMetadata("acceleration", {}).unit, "原始值");
});

test("serial disconnect releases a pending reader once and concurrent commands stay intact", async () => {
  let writes = [], closed = 0;
  const source = new SerialSource({ onState: () => {}, onError: (e) => { throw e; } });
  source.keepReading = true;
  source.port = {
    readable: new ReadableStream({ start(controller) { controller.enqueue(bytes(fixture.sample)); } }),
    writable: new WritableStream({ write(chunk) { writes.push(chunk); } }),
    close: async () => { closed++; },
  };
  source.reading = source.readLoop();
  await Promise.all([source.sendCommand(5, 1, 20), source.sendCommand(2, 2)]);
  assert.equal(writes.length, 2); assert.equal(writes[0][10], 5); assert.equal(writes[1][10], 2);
  await source.disconnect();
  assert.equal(closed, 1); assert.equal(source.reader, null); assert.equal(source.port, null);
});
