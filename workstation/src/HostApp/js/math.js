export function multiplyQuaternion(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

export function inverseQuaternion(q) {
  const normSquared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return { w: q.w / normSquared, x: -q.x / normSquared, y: -q.y / normSquared, z: -q.z / normSquared };
}

export function relativeQuaternion(from, to) {
  return multiplyQuaternion(inverseQuaternion(from), to);
}

export function quaternionAngleDegrees(q) {
  const w = Math.min(1, Math.max(-1, Math.abs(q.w)));
  return (2 * Math.acos(w) * 180) / Math.PI;
}

export function quaternionToEulerDegrees(q) {
  const sinRoll = 2 * (q.w * q.x + q.y * q.z);
  const cosRoll = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinRoll, cosRoll);

  const sinPitch = 2 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinPitch) >= 1 ? Math.sign(sinPitch) * Math.PI / 2 : Math.asin(sinPitch);

  const sinYaw = 2 * (q.w * q.z + q.x * q.y);
  const cosYaw = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(sinYaw, cosYaw);
  const scale = 180 / Math.PI;
  return { roll: roll * scale, pitch: pitch * scale, yaw: yaw * scale };
}

export function applyNeutral(q, neutral) {
  return neutral ? relativeQuaternion(neutral, q) : q;
}

export function eulerDegreesToQuaternion(roll, pitch, yaw) {
  const [r, p, y] = [roll, pitch, yaw].map((value) => value * Math.PI / 360);
  return {
    w: Math.cos(r) * Math.cos(p) * Math.cos(y) + Math.sin(r) * Math.sin(p) * Math.sin(y),
    x: Math.sin(r) * Math.cos(p) * Math.cos(y) - Math.cos(r) * Math.sin(p) * Math.sin(y),
    y: Math.cos(r) * Math.sin(p) * Math.cos(y) + Math.sin(r) * Math.cos(p) * Math.sin(y),
    z: Math.cos(r) * Math.cos(p) * Math.sin(y) - Math.sin(r) * Math.sin(p) * Math.cos(y),
  };
}
