import test from "node:test";
import assert from "node:assert/strict";
import { eulerDegreesToQuaternion as euler, inverseQuaternion, multiplyQuaternion } from "../js/math.js";
import { captureSegmentReference, parseRegistration, segmentPose } from "../js/neck-relative.js";

const nodes = ["C0", "C4", "C7"];
const unit = [1, 0, 0, 0];
function frame(poses, timestamps = [100_000, 102_000, 104_000]) {
  return { timestampUs: Math.max(...timestamps), samples: Object.fromEntries(nodes.map((node, i) =>
    [node, { q: poses[i], timestampUs: timestamps[i], valid: !!poses[i] }])) };
}
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);

test("S06 uses reference-to-target rotations and retains signed axes", () => {
  const result = segmentPose(frame([euler(0, -25, 0), euler(0, -10, 0), euler(0, 5, 0)]));
  near(result.overall.signed.flexion, -30);
  near(result.upper.signed.flexion, -15);
  near(result.lower.signed.flexion, -15);
  near(result.base.signed.flexion, 5);
  const mixed = segmentPose(frame([euler(-8, 11, -17), euler(0, 0, 0), euler(0, 0, 0)]));
  near(mixed.overall.signed.flexion, 11); near(mixed.overall.signed.lateral, -8); near(mixed.overall.signed.axial, -17);
});

test("S06 applies world transform on the left, inverse mount on the right and neutral last", () => {
  const mount = euler(13, 0, 11), world = euler(0, -24, 31);
  const arr = (q) => [q.w, q.x, q.y, q.z];
  const identity = { sensorToBody: unit, fusionToCommon: unit };
  const registration = parseRegistration({ C0: { sensorToBody: arr(mount), fusionToCommon: arr(world) }, C4: identity, C7: identity });
  const sample = (deg) => frame([multiplyQuaternion(multiplyQuaternion(inverseQuaternion(world), euler(0, deg, 0)), mount), euler(0, 0, 0), euler(0, 0, 0)]);
  const reference = captureSegmentReference(sample(12), registration);
  near(segmentPose(sample(10), registration, reference).overall.signed.flexion, -2);
  near(segmentPose(sample(12), registration, reference).overall.signed.flexion, 0);
  assert.throws(() => parseRegistration({ C0: identity }), /C4/);
  assert.throws(() => parseRegistration({ C0: { ...identity, sensorToBody: [0, 0, 0, 0] } }), /C0/);
});

test("S06 suppresses invalid data, skew, and singular Euler components without inventing angles", () => {
  const q = euler(0, 0, 0), poses = [euler(0, 90, 0), q, q];
  const result = segmentPose(frame(poses));
  assert.equal(result.overall.signed, null); assert.ok(result.overall.q);
  const single = frame([q, null, null]);
  assert.equal(captureSegmentReference(single, null), null);
  assert.equal(segmentPose(single).overall.q, undefined);
  const late = frame(poses, [0, 9000, 10001]);
  assert.equal(segmentPose(late).overall.q, undefined);
  assert.ok(segmentPose(late).upper.q);
  late.samples.C0.quaternionNorm = 0.98;
  assert.equal(segmentPose(late).upper.q, undefined);
});
