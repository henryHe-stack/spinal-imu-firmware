import { eulerDegreesToQuaternion } from "./math.js";

export class DemoSource {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.timer = null;
    this.sequence = 0;
    this.startedAt = 0;
  }

  start() {
    this.startedAt = performance.now();
    this.timer = window.setInterval(() => this.emit(), 20);
  }

  stop() {
    if (this.timer) window.clearInterval(this.timer);
    this.timer = null;
  }

  emit() {
    const t = (performance.now() - this.startedAt) / 1000;
    const baseYaw = 8 * Math.sin(t * 0.55);
    const values = {
      C7: [2 * Math.sin(t), 2.5 * Math.sin(t * 0.4), baseYaw * 0.25],
      C4: [7 * Math.sin(t * 0.85), 10 * Math.sin(t * 0.48), baseYaw * 0.65],
      C0: [12 * Math.sin(t * 0.85 + 0.14), 18 * Math.sin(t * 0.48 + 0.08), baseYaw],
    };
    const timestamp = Math.round(t * 1_000_000);
    const samples = Object.entries(values).map(([node, angles], index) => ({
      node,
      timestampUs: timestamp + index * 1100,
      q: eulerDegreesToQuaternion(...angles),
      acceleration: [0.05 * Math.sin(t), 9.81, 0.03 * Math.cos(t)],
      gyroscope: angles.map((angle) => angle * 0.02),
      calibration: 3,
      flags: 1,
    }));
    this.onFrame({ sequence: this.sequence++, samples, protocol: "DEMO" }, JSON.stringify({ seq: this.sequence - 1, samples }));
  }
}
