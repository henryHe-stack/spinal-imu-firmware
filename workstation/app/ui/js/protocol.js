export const NODE_IDS = ["C0", "C4", "C7"];

const NODE_ALIASES = new Map([
  ["0", "C0"], ["C0", "C0"],
  ["1", "C4"], ["C4", "C4"],
  ["2", "C7"], ["C7", "C7"],
]);

function finiteNumber(value, name) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new Error(`${name} 不是有效数值`);
  return number;
}

function normalizeQuaternion(input) {
  let values;
  if (Array.isArray(input)) values = input;
  else if (input && typeof input === "object") values = [input.w, input.x, input.y, input.z];
  else throw new Error("缺少四元数 q=[w,x,y,z]");

  const [w, x, y, z] = values.map((value, index) => finiteNumber(value, `q${index}`));
  const norm = Math.hypot(w, x, y, z);
  if (norm < 1e-8) throw new Error("四元数模长为零");
  return { w: w / norm, x: x / norm, y: y / norm, z: z / norm };
}

function normalizeVector(input) {
  if (input === undefined || input === null) return null;
  if (!Array.isArray(input) || input.length < 3) throw new Error("三轴向量必须包含三个数值");
  return input.slice(0, 3).map((value, index) => finiteNumber(value, `vector${index}`));
}

function normalizeNode(value) {
  const key = String(value ?? "").trim().toUpperCase();
  const node = NODE_ALIASES.get(key);
  if (!node) throw new Error(`未知节点 ${value}`);
  return node;
}

function normalizeSample(input) {
  return {
    node: normalizeNode(input.node ?? input.id ?? input.node_id),
    timestampUs: finiteNumber(input.timestamp_us ?? input.timestampUs ?? input.t ?? 0, "timestamp_us"),
    q: normalizeQuaternion(input.q ?? input.quaternion),
    acceleration: normalizeVector(input.acceleration ?? input.accel),
    gyroscope: normalizeVector(input.gyroscope ?? input.gyro),
    calibration: Math.max(0, Math.min(3, finiteNumber(input.calibration ?? input.cal ?? 0, "calibration"))),
    flags: finiteNumber(input.flags ?? 0, "flags"),
  };
}

function parseJson(line) {
  const raw = JSON.parse(line);
  const rawSamples = Array.isArray(raw.samples) ? raw.samples : [raw];
  return {
    sequence: finiteNumber(raw.sequence ?? raw.seq ?? 0, "sequence"),
    samples: rawSamples.map(normalizeSample),
    protocol: "NDJSON",
  };
}

function parseCsv(line) {
  const columns = line.split(",").map((item) => item.trim());
  if (columns[0]?.toUpperCase() === "IMU") columns.shift();
  if (columns.length < 8) throw new Error("CSV 至少需要 seq,node,timestamp,w,x,y,z,cal");
  const [sequence, node, timestampUs, w, x, y, z, calibration, ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, flags = 0] = columns;
  return {
    sequence: finiteNumber(sequence, "sequence"),
    samples: [normalizeSample({
      node, timestamp_us: timestampUs, q: [w, x, y, z], calibration,
      acceleration: columns.length >= 11 ? [ax, ay, az] : null, gyroscope: columns.length >= 14 ? [gx, gy, gz] : null, flags,
    })],
    protocol: "CSV",
  };
}

export function parseTelemetryLine(line) {
  const trimmed = line.trim();
  if (!trimmed) return null;
  return trimmed.startsWith("{") ? parseJson(trimmed) : parseCsv(trimmed);
}

export function validateCompleteFrame(frame) {
  const present = new Set(frame.samples.map((sample) => sample.node));
  return NODE_IDS.every((node) => present.has(node));
}
