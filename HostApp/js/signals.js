// Shared signal definitions: charts and numeric readouts must use the same
// quaternion convention. Binary v1 vectors use m/s² and °/s; legacy text
// vectors retain the transmitted units and are labelled separately.
import { applyNeutral, quaternionAngleDegrees, quaternionToEulerDegrees, relativeQuaternion } from "./math.js";
import { NODE_IDS } from "./protocol.js";

export const AXIS_COLORS = ["#2867c4", "#11999e", "#be8149"];
export const PAIRS = [
  { key: "C0C4", from: "C0", to: "C4", label: "C0–C4", color: "#2867c4" },
  { key: "C4C7", from: "C4", to: "C7", label: "C4–C7", color: "#11999e" },
  { key: "C0C7", from: "C0", to: "C7", label: "C0–C7", color: "#8b69b7" },
];
export const SIGNALS = {
  orientation: { label: "姿态角", unit: "°", axes: ["Roll", "Pitch", "Yaw"], note: "姿态角 / ° · 与模型使用相同中立位" },
  acceleration: { label: "加速度", unit: "原始值", axes: ["X", "Y", "Z"], note: "加速度 / 传输原始值 · 单位以固件输出约定为准" },
  gyroscope: { label: "角速度", unit: "原始值", axes: ["X", "Y", "Z"], note: "角速度 / 传输原始值 · 单位以固件输出约定为准" },
};

export function signalMetadata(signal, snapshot) {
  const base = SIGNALS[signal];
  if (signal === "orientation" || snapshot?.vectorUnits !== "SI") return base;
  const unit = signal === "acceleration" ? "m/s²" : "°/s";
  return { ...base, unit, note: `${base.label} / ${unit} · 固件 BNO055 单位换算` };
}

export function frameSignals(snapshot, neutral = {}) {
  const nodes = {}, quaternions = {};
  for (const node of NODE_IDS) {
    const sample = snapshot.samples[node];
    const q = applyNeutral(sample.q, neutral[node]);
    quaternions[node] = q;
    const euler = quaternionToEulerDegrees(q);
    nodes[node] = {
      orientation: [euler.roll, euler.pitch, euler.yaw],
      acceleration: sample.acceleration ?? null,
      gyroscope: sample.gyroscope ?? null,
    };
  }
  const relative = Object.fromEntries(PAIRS.map(({ key, from, to }) => [key,
    quaternionAngleDegrees(relativeQuaternion(quaternions[from], quaternions[to])),
  ]));
  return { nodes, relative };
}

// The device may restart or wrap its timestamp. Keep a monotonic session axis
// without rewriting the raw timestamp: only that transition uses host elapsed
// time, and the renderer breaks the trace there rather than joining the reset.
export function sessionPosition(previous, snapshot) {
  if (!previous) return { sessionSeconds: 0, clockReset: false };
  const deviceDelta = (snapshot.timestampUs - previous.timestampUs) / 1_000_000;
  const clockReset = deviceDelta < 0;
  const delta = deviceDelta > 0 ? deviceDelta : Math.max(0.001, (snapshot.receivedAt - previous.receivedAt) / 1000);
  return { sessionSeconds: previous.sessionSeconds + delta, clockReset };
}

export function nearestFrameIndex(history, time) {
  if (!history.length) return -1;
  let low = 0, high = history.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (history[middle].sessionSeconds < time) low = middle + 1;
    else high = middle;
  }
  return low > 0 && Math.abs(history[low - 1].sessionSeconds - time) <= Math.abs(history[low].sessionSeconds - time) ? low - 1 : low;
}

// Min/max binning keeps brief motion peaks while bounding drawing work by
// viewport width. The order within each bin is preserved. Nulls break a trace;
// wrapping Euler angles break at ±180° instead of drawing false vertical spikes.
export function envelopePoints(points, bins, breakAtWrap = false) {
  const result = [], size = Math.max(1, Math.ceil(points.length / Math.max(1, bins)));
  for (let start = 0; start < points.length; start += size) {
    const end = Math.min(start + size, points.length);
    const selected = new Set([start, end - 1]);
    let min = -1, max = -1;
    for (let i = start; i < end; i += 1) {
      if (!Number.isFinite(points[i].value) || points[i].gap || (breakAtWrap && i > 0 && Math.abs(points[i].value - points[i - 1].value) > 180)) {
        selected.add(i); if (i > start) selected.add(i - 1);
      }
      if (!Number.isFinite(points[i].value)) continue;
      if (min === -1 || points[i].value < points[min].value) min = i;
      if (max === -1 || points[i].value > points[max].value) max = i;
    }
    if (min !== -1) { selected.add(min); selected.add(max); }
    for (const i of [...selected].sort((a, b) => a - b)) result.push({ ...points[i], gap: points[i].gap || (breakAtWrap && i > 0 && Math.abs(points[i].value - points[i - 1].value) > 180) });
  }
  return result;
}
