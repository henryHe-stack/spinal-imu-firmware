import test from "node:test";
import assert from "node:assert/strict";
import { eulerDegreesToQuaternion } from "../js/math.js";
import { envelopePoints, frameSignals, nearestFrameIndex, sessionPosition } from "../js/signals.js";
import { parseTelemetryLine } from "../js/protocol.js";

const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-7, `${a} != ${b}`);
const snapshot = {
  samples: Object.fromEntries(["C0", "C4", "C7"].map((node, index) => [node, {
    q: eulerDegreesToQuaternion([30, 15, 0][index], 0, 0), acceleration: [0, 9.81, 0], gyroscope: [1, 2, 3],
  }])),
};

test("chart signals agree with known node rotations and quaternion relative angles", () => {
  const data = frameSignals(snapshot);
  close(data.nodes.C0.orientation[0], 30); close(data.nodes.C4.orientation[0], 15);
  close(data.relative.C0C4, 15); close(data.relative.C4C7, 15); close(data.relative.C0C7, 30);
  const neutral = Object.fromEntries(Object.entries(snapshot.samples).map(([node, sample]) => [node, sample.q]));
  const zero = frameSignals(snapshot, neutral);
  for (const node of ["C0", "C4", "C7"]) close(zero.nodes[node].orientation[0], 0);
  for (const value of Object.values(zero.relative)) close(value, 0);
  assert.deepEqual(zero.nodes.C0.acceleration, [0, 9.81, 0]);
});

test("optional vectors remain absent rather than becoming fictitious zero readings", () => {
  for (const line of ['{"seq":1,"node":"C0","q":[1,0,0,0]}', "IMU,1,C0,0,1,0,0,0,3"]) {
    const sample = parseTelemetryLine(line).samples[0];
    assert.equal(sample.acceleration, null); assert.equal(sample.gyroscope, null);
  }
  assert.throws(() => parseTelemetryLine('{"seq":1,"node":"C0","q":[1,0,0,0],"accel":[1,2]}'));
});

test("session axis preserves irregular device intervals and isolates clock resets", () => {
  const first = { timestampUs: 1_000_000, receivedAt: 10_000, ...sessionPosition(null, {}) };
  const second = { timestampUs: 1_175_000, receivedAt: 10_010 };
  Object.assign(second, sessionPosition(first, second));
  close(second.sessionSeconds, 0.175); assert.equal(second.clockReset, false);
  const reset = sessionPosition(second, { timestampUs: 0, receivedAt: 10_110 });
  close(reset.sessionSeconds, 0.275); assert.equal(reset.clockReset, true);
  const history = [first, second, { sessionSeconds: 0.8 }];
  assert.equal(nearestFrameIndex(history, 0.17), 1); assert.equal(nearestFrameIndex(history, -10), 0);
  assert.equal(nearestFrameIndex(history, 5), 2); assert.equal(nearestFrameIndex([], 1), -1);
});

test("display decimation retains brief peaks, missing data and Euler wrap breaks", () => {
  const points = Array.from({ length: 10_000 }, (_, i) => ({ time: i, value: i === 4312 ? 99 : 0 }));
  const envelope = envelopePoints(points, 100);
  assert.ok(envelope.some((point) => point.value === 99)); assert.ok(envelope.length <= 400);
  const gaps = envelopePoints([{ time: 0, value: 179 }, { time: 1, value: -179 }, { time: 2, value: null }], 1, true);
  assert.ok(gaps.find((point) => point.time === 1).gap);
  assert.ok(gaps.some((point) => point.value === null));
});
