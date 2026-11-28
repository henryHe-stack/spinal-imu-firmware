import { multiplyQuaternion, relativeQuaternion } from "./math.js";
import { NODE_IDS } from "./protocol.js";

// Browser port of workstation/src/host_core/Algorithm/Src/neck_relative.c (S06).
// Inputs are Hamilton active wxyz, q: sensor -> local fusion world.
export const SEGMENTS = Object.freeze({ overall: ["C7", "C0"], upper: ["C4", "C0"], lower: ["C7", "C4"], base: [null, "C7"] });
const unit = { w: 1, x: 0, y: 0, z: 0 };
const radToDeg = 180 / Math.PI;
const components = ["w", "x", "y", "z"];

function normalized(q, tolerance = 0.001) {
  const values = components.map((axis) => q?.[axis]);
  const norm = Math.hypot(...values);
  if (!values.every(Number.isFinite) || norm === 0 || Math.abs(norm - 1) > tolerance) return null;
  const sign = values.find((v) => v !== 0) < 0 ? -1 : 1;
  return Object.fromEntries(components.map((axis, i) => [axis, (sign * values[i] / norm) || 0]));
}

export function parseRegistration(input) {
  const parse = (row) => Array.isArray(row) && row.length === 4 && normalized(Object.fromEntries(components.map((axis, i) => [axis, row[i]])));
  return Object.fromEntries(NODE_IDS.map((node) => {
    const mount = parse(input?.[node]?.sensorToBody);
    const world = parse(input?.[node]?.fusionToCommon);
    if (!mount || !world) throw new Error(`${node} 需要两个按 [w,x,y,z] 排列的单位四元数`);
    return [node, { mount, world }];
  }));
}

function angles(q) {
  const { w, x, y, z } = q;
  const r00 = 1 - 2 * (y * y + z * z), r10 = 2 * (x * y + w * z);
  const cosine = Math.hypot(r00, r10);
  if (cosine <= 1e-6) return null;
  return {
    flexion: Math.atan2(2 * (w * y - x * z), cosine) * radToDeg,
    lateral: Math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y)) * radToDeg,
    axial: Math.atan2(r10, r00) * radToDeg,
  };
}

export function segmentPose(snapshot, registration = null, neutral = null) {
  const keys = Object.keys(SEGMENTS);
  const poses = {}, faults = {};
  for (const node of NODE_IDS) {
    const sample = snapshot?.samples?.[node];
    const q = normalized(sample?.q);
    if (!sample || sample.valid === false || !q) faults[node] = "节点不可用";
    else if (sample.quaternionNorm !== undefined && Math.abs(sample.quaternionNorm - 1) > 0.001) faults[node] = "四元数模长异常";
    else if (sample.sampleTimeKnown === false || !Number.isFinite(sample.timestampUs) || !Number.isFinite(snapshot.timestampUs)) faults[node] = "无设备时间";
    else if (sample.timestampUs > snapshot.timestampUs || snapshot.timestampUs - sample.timestampUs > 30_000) faults[node] = "节点数据过旧";
    else {
      const { mount, world } = registration?.[node] ?? { mount: unit, world: unit };
      poses[node] = normalized(multiplyQuaternion(world, multiplyQuaternion(q, { w: mount.w, x: -mount.x, y: -mount.y, z: -mount.z })));
    }
  }
  return Object.fromEntries(keys.map((key) => {
    const [ref, target] = SEGMENTS[key];
    const fault = faults[target] || (ref && faults[ref]);
    if (fault) return [key, { reason: fault }];
    if (ref && Math.abs(snapshot.samples[ref].timestampUs - snapshot.samples[target].timestampUs) > 10_000)
      return [key, { reason: "节点时间差超限" }];
    const raw = ref ? normalized(relativeQuaternion(poses[ref], poses[target])) : poses[target];
    const q = neutral?.[key] ? normalized(relativeQuaternion(neutral[key], raw)) : raw;
    const signed = angles(q);
    return [key, { raw, q, signed, reason: signed ? "" : "欧拉角奇异" }];
  }));
}

export function captureSegmentReference(snapshot, registration) {
  const pairs = Object.entries(segmentPose(snapshot, registration));
  return pairs.every(([, pair]) => pair.raw) ? Object.fromEntries(pairs.map(([id, pair]) => [id, pair.raw])) : null;
}
