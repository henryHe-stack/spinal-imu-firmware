import test from "node:test";
import assert from "node:assert/strict";
import { parseTelemetryLine, validateCompleteFrame } from "../js/protocol.js";
import { eulerDegreesToQuaternion, quaternionToEulerDegrees, relativeQuaternion, quaternionAngleDegrees } from "../js/math.js";
import { decodePacket, DeviceClock, TelemetryFramer } from "../js/wire.js";
import fs from "node:fs";

test("parses a complete NDJSON frame", () => {
  const frame = parseTelemetryLine(JSON.stringify({ seq: 7, samples: [
    { node: "C0", timestamp_us: 1002, q: [1, 0, 0, 0], cal: 3 },
    { node: "C4", timestamp_us: 1001, q: [1, 0, 0, 0], cal: 2 },
    { node: "C7", timestamp_us: 1000, q: [1, 0, 0, 0], cal: 1 },
  ] }));
  assert.equal(frame.sequence, 7);
  assert.equal(frame.protocol, "NDJSON");
  assert.equal(validateCompleteFrame(frame), true);
});

test("parses a CSV node sample", () => {
  const frame = parseTelemetryLine("IMU,12,C4,22000,1,0,0,0,3,0,9.81,0,0,0,0,1");
  assert.equal(frame.samples[0].node, "C4");
  assert.equal(frame.samples[0].acceleration[1], 9.81);
  assert.equal(validateCompleteFrame(frame), false);
});

test("normalizes raw quaternion values", () => {
  const frame = parseTelemetryLine('{"seq":1,"node":0,"t":9,"q":[16384,0,0,0]}');
  assert.deepEqual(frame.samples[0].q, { w: 1, x: 0, y: 0, z: 0 });
});

test("round-trips Euler angles and relative angle", () => {
  const q = eulerDegreesToQuaternion(10, 20, 30);
  const e = quaternionToEulerDegrees(q);
  assert.ok(Math.abs(e.roll - 10) < 1e-9);
  assert.ok(Math.abs(e.pitch - 20) < 1e-9);
  assert.ok(Math.abs(e.yaw - 30) < 1e-9);
  assert.ok(Math.abs(quaternionAngleDegrees(relativeQuaternion(q, q))) < 1e-9);
});

test("rejects malformed telemetry", () => {
  assert.throws(() => parseTelemetryLine("IMU,1,C0"));
  assert.throws(() => parseTelemetryLine('{"seq":1,"node":"bad","q":[1,0,0,0]}'));
});

test("decodes the production C wire fixture and preserves SI units", () => {
  const fixture = JSON.parse(fs.readFileSync(new URL("./fixtures/firmware-v1.json", import.meta.url)));
  const bytes = Uint8Array.from(fixture.sample.match(/../g).map((hex) => parseInt(hex, 16)));
  const frame = decodePacket(bytes);
  assert.equal(frame.sequence, 42);
  assert.equal(frame.vectorUnits, "SI");
  assert.deepEqual(frame.samples[0].acceleration, [-1.23, 9.81, 0]);
  assert.deepEqual(frame.samples[0].gyroscope, [0, 0, -2]);
  assert.equal(frame.samples[0].calibration, 3);
});

test("binary framer resynchronizes after CRC corruption and handles clock wrap", () => {
  const fixture = JSON.parse(fs.readFileSync(new URL("./fixtures/firmware-v1.json", import.meta.url)));
  const source = Uint8Array.from(fixture.sample.match(/../g).map((hex) => parseInt(hex, 16)));
  const broken = source.slice(); broken[20] ^= 0x80;
  let frames = 0, errors = 0;
  const framer = new TelemetryFramer({ onFrame: () => { frames += 1; }, onError: () => { errors += 1; } });
  framer.push(Uint8Array.from([9, 8, ...broken, ...source]));
  assert.equal(frames, 1); assert.ok(errors >= 1);
  const clock = new DeviceClock();
  const first = { samples: [{ timestampUs: 0xfffffff0, timestampStartUs: 0xfffffff0, timestampEndUs: 0xfffffff0 }] };
  const second = { samples: [{ timestampUs: 0x00000010, timestampStartUs: 0x10, timestampEndUs: 0x10 }] };
  clock.apply(first); clock.apply(second);
  assert.equal(second.samples[0].timestampUs, 0x100000010);
});
