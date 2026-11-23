import { test, expect } from "@playwright/test";
import { readFileSync } from "node:fs";
import { crc16 } from "../js/wire.js";

const binaryFixture = JSON.parse(readFileSync(new URL("./fixtures/firmware-v1.json", import.meta.url)));
function binaryFrame(sequence, invalid = false) {
  const frame = Uint8Array.from(Buffer.from(binaryFixture.sample, "hex"));
  const view = new DataView(frame.buffer); view.setUint32(6, sequence, true);
  for (let i = 0; i < 3; i++) for (const offset of [4, 8, 12]) view.setUint32(11 + i * 40 + offset, 1_000_000 + sequence * 20_000 + i * 5000 + offset * 10, true);
  if (invalid) frame[13] = 4;
  view.setUint16(frame.length - 2, crc16(frame.subarray(2, -2)), true);
  return [...frame];
}
async function openBinary(page) {
  await page.addInitScript(() => {
    window.testSerialWrites = [];
    const port = { open: async () => {}, close: async () => {},
      readable: new ReadableStream({ start(controller) { window.testSerialReceive = (bytes) => controller.enqueue(Uint8Array.from(bytes)); } }),
      writable: new WritableStream({ write(chunk) { window.testSerialWrites.push([...chunk]); } }),
    };
    Object.defineProperty(navigator, "serial", { configurable: true, value: { requestPort: async () => port } });
  });
  const errors = []; page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("http://127.0.0.1:4173/"); await page.locator("#connectButton").click();
  return errors;
}

test("binary startup and recovery preserve the last complete pose and physical units", async ({ page }) => {
  const errors = await openBinary(page);
  await page.evaluate((data) => window.testSerialReceive(data), binaryFrame(1, true));
  await expect(page.locator('[data-node="C0"] .node-state')).toHaveText("无效 / 重连中");
  await expect(page.locator("#historyCount")).toHaveText("0");
  await page.evaluate((data) => window.testSerialReceive(data), binaryFrame(2));
  await expect(page.locator("#historyCount")).toHaveText("1");
  await page.evaluate((data) => window.testSerialReceive(data), binaryFrame(3, true));
  await expect(page.locator("#historyCount")).toHaveText("1");
  await expect(page.locator("#modelState")).toContainText("保留上次完整姿态");
  await page.evaluate((data) => window.testSerialReceive(data), binaryFrame(4));
  await expect(page.locator("#historyCount")).toHaveText("2");
  await page.locator("#tab-curves").click(); await page.locator("#curveSignal").selectOption("acceleration");
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["-1.23", "9.81", "0.00"]);
  await expect(page.locator('[data-chart="C0"] .signal-unit')).toHaveText("m/s²");
  await page.locator("#connectButton").click();
  await expect(page.locator("#connectionText")).toHaveText("等待连接");
  expect(errors).toEqual([]);
});

test("binary controls write commands and display matched replies without losing telemetry", async ({ page }) => {
  const errors = await openBinary(page);
  await page.locator("#tab-diagnostics").click();
  await page.locator('[data-device-command="5"]').click();
  await expect.poll(() => page.evaluate(() => window.testSerialWrites.length)).toBe(1);
  const sent = await page.evaluate(() => window.testSerialWrites[0]);
  expect(sent[10]).toBe(5); expect(sent[11]).toBe(20);
  const reply = Uint8Array.from(Buffer.from(binaryFixture.reply, "hex"));
  new DataView(reply.buffer).setUint32(6, 1, true);
  new DataView(reply.buffer).setUint16(16, crc16(reply.subarray(2, -2)), true);
  await page.evaluate((data) => window.testSerialReceive(data), [...reply, ...binaryFrame(1)]);
  await expect(page.locator("#commandResult")).toContainText("请求 1 · 命令 5 · 成功");
  await expect(page.locator("#frameCount")).toHaveText("1");
  expect(errors).toEqual([]);
});

test("binary CRC failure is visible and next valid frame still renders on mobile", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  const errors = await openBinary(page);
  const broken = binaryFrame(1); broken[27] ^= 0x40;
  await page.evaluate((data) => window.testSerialReceive(data), [...broken, ...binaryFrame(2)]);
  await expect(page.locator("#historyCount")).toHaveText("1");
  await page.locator("#tab-diagnostics").click();
  await expect(page.locator("#parseErrors")).toHaveText("1");
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

// In-memory serial transport, not a hardware verification. Known poses keep
// camera and history tests deterministic even while reception remains active.
async function openKnownPoses(page, withVectors = false) {
  await page.addInitScript((includeVectors) => {
    const samples = (angle) => ["C0", "C4", "C7"].map((node, index) => {
      const half = angle * [1, 0.5, 0][index] * Math.PI / 360;
      return { node, timestamp_us: 1_000_000, q: [Math.cos(half), Math.sin(half), 0, 0], cal: 3,
        ...(includeVectors ? { accel: [angle, 9.81, index], gyro: [angle / 10, index, -2] } : {}),
      };
    });
    const frames = [0, 15, 30].map((angle, i) => ({ seq: i + 1, samples: samples(angle).map((sample) => ({ ...sample, timestamp_us: 1_000_000 + i * 100_000 })) }));
    Object.defineProperty(navigator, "serial", { configurable: true, value: {
      requestPort: async () => ({
        open: async () => {}, close: async () => {},
        readable: new ReadableStream({ start(controller) {
          controller.enqueue(new TextEncoder().encode(frames.map((frame) => JSON.stringify(frame)).join("\n") + "\n"));
        } }),
      }),
    } });
  }, withVectors);
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => { if (message.type() === "error") errors.push(message.text()); });
  await page.goto("http://127.0.0.1:4173/");
  await page.locator("#connectButton").click();
  await expect(page.locator("#historyCount")).toHaveText("3");
  await page.waitForTimeout(250); // Let the specified render interpolation settle.
  return errors;
}

test("embedded timeline seeks exact poses and returns to the latest frame", async ({ page }) => {
  const errors = await openKnownPoses(page);
  await expect(page.locator(".spine-panel .timeline-panel")).toHaveCount(1);
  const before = await page.locator("#spineCanvas").screenshot();
  await page.locator("#timelineTicks button").first().click();
  await expect(page.locator("#timelineMode")).toHaveText("历史");
  await expect(page.locator("#timelineCurrent")).toHaveText("1.000 s");
  await expect(page.locator('[data-node="C0"] [data-field="roll"]')).toHaveText("+0.0°");
  const firstPose = await page.locator("#spineCanvas").screenshot();
  expect(before.equals(firstPose)).toBe(false);
  await page.locator("#nextFrame").click();
  await expect(page.locator("#timelineCurrent")).toHaveText("1.100 s");
  await expect(page.locator('[data-node="C0"] [data-field="roll"]')).toHaveText("+15.0°");
  await page.locator("#timelineRange").fill("0");
  await expect(page.locator("#timelineCurrent")).toHaveText("1.000 s");
  const chart = await page.locator("#timelineCanvas").boundingBox();
  await page.mouse.click(chart.x + chart.width / 2, chart.y + chart.height / 2);
  await expect(page.locator("#timelineCurrent")).toHaveText("1.100 s");
  await page.locator("#previousFrame").click();
  await page.locator("#playbackButton").click();
  await expect(page.locator("#timelineCurrent")).toHaveText("1.200 s");
  await expect(page.locator("#timelineMode")).toHaveText("历史");
  await page.locator("#liveButton").click();
  await expect(page.locator("#timelineMode")).toHaveText("实时");
  await expect(page.locator('[data-node="C0"] [data-field="roll"]')).toHaveText("+30.0°");
  expect(errors).toEqual([]);
});

test("transparent seat, presets and orbit controls work on a frozen pose", async ({ page }) => {
  const errors = await openKnownPoses(page);
  await page.locator("#timelineTicks button").first().click();
  const canvas = page.locator("#spineCanvas");
  await page.screenshot({ path: "qa/refined-default.png", fullPage: true });
  const initial = await canvas.screenshot();
  await page.locator("#seatGhost").uncheck();
  const opaque = await canvas.screenshot(); expect(initial.equals(opaque)).toBe(false);
  await page.locator("#seatGhost").check();
  await page.getByRole("button", { name: "颈部聚焦", exact: true }).click();
  await expect(page.locator('[data-preset="neck"]')).toHaveAttribute("aria-pressed", "true");
  const closeup = await canvas.screenshot(); expect(closeup.equals(initial)).toBe(false);
  await page.screenshot({ path: "qa/refined-neck.png", fullPage: true });
  await page.locator('[data-preset="driver"]').click();
  await page.locator('[data-view="spine"] [data-action="zoom-in"]').click();
  const zoomed = await canvas.screenshot(); expect(zoomed.equals(initial)).toBe(false);
  const b = await canvas.boundingBox();
  await page.mouse.move(b.x + b.width * 0.45, b.y + b.height * 0.5);
  await page.mouse.down(); await page.mouse.move(b.x + b.width * 0.65, b.y + b.height * 0.5, { steps: 5 }); await page.mouse.up();
  const rotated = await canvas.screenshot(); expect(rotated.equals(zoomed)).toBe(false);
  // Verify a separate node view, rather than relying solely on the main scene.
  const imu = page.locator("#imuCanvasC0"), imuBefore = await imu.screenshot();
  await page.locator('[data-view="C0"] [data-action="zoom-in"]').click({ force: true });
  expect(imuBefore.equals(await imu.screenshot())).toBe(false);
  await page.screenshot({ path: "qa/refined-desktop-verified.png", fullPage: true });
  expect(errors).toEqual([]);
});

test("incoming live data does not overwrite the selected historical pose", async ({ page }) => {
  await page.goto("http://127.0.0.1:4173/?demo=1");
  await expect.poll(async () => Number((await page.locator("#historyCount").textContent()).replaceAll(",", ""))).toBeGreaterThan(12);
  await page.locator("#timelineTicks button").first().click();
  const selectedTime = await page.locator("#timelineCurrent").textContent();
  const selectedQuaternion = await page.locator('[data-node="C0"] [data-field="quaternion"]').textContent();
  const count = Number(await page.locator("#historyCount").textContent());
  await expect.poll(async () => Number((await page.locator("#historyCount").textContent()).replaceAll(",", ""))).toBeGreaterThan(count + 5);
  await expect(page.locator("#timelineCurrent")).toHaveText(selectedTime);
  await expect(page.locator('[data-node="C0"] [data-field="quaternion"]')).toHaveText(selectedQuaternion);
  await page.locator("#liveButton").click();
  await expect(page.locator("#timelineCurrent")).not.toHaveText(selectedTime);
});

test("mobile layout keeps time controls immediately after the model without overflow", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  const errors = await openKnownPoses(page);
  const geometry = await page.evaluate(() => {
    const stage = document.querySelector(".spine-stage").getBoundingClientRect();
    const timeline = document.querySelector(".timeline-panel").getBoundingClientRect();
    const nodes = document.querySelector(".imu-column").getBoundingClientRect();
    return { modelBottom: stage.bottom, timelineTop: timeline.top, timelineBottom: timeline.bottom, nodesTop: nodes.top, width: document.documentElement.scrollWidth, viewport: innerWidth };
  });
  expect(geometry.timelineTop).toBeGreaterThan(geometry.modelBottom);
  expect(geometry.timelineBottom).toBeLessThan(geometry.nodesTop);
  expect(geometry.width).toBeLessThanOrEqual(geometry.viewport);
  await page.locator("#timelineTicks button").first().click();
  await expect(page.locator("#timelineCurrent")).toHaveText("1.000 s");
  await page.screenshot({ path: "qa/refined-mobile-verified.png", fullPage: true });
  expect(errors).toEqual([]);
});

test("curve window plots three known IMUs and selects the same frame as the model", async ({ page }) => {
  const errors = await openKnownPoses(page);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  await expect(page.locator("#window-pose")).toBeHidden();
  await expect(page.locator("#window-curves")).toBeVisible();
  await expect(page.locator("#window-diagnostics")).toBeHidden();
  await expect(page.locator("#curveTimelineSlot .timeline-panel")).toHaveCount(1);
  await expect(page.locator("#nodeCharts canvas")).toHaveCount(3);
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["30.00°", "0.00°", "0.00°"]);
  await expect(page.locator('[data-chart="C4"] .signal-readouts strong').first()).toHaveText("15.00°");
  await expect(page.locator("#curveSelectedTime")).toHaveText("1.200 s");
  await page.screenshot({ path: "qa/curves-nodes-desktop.png", fullPage: true });
  const canvas = page.locator('[data-chart="C0"] canvas'), box = await canvas.boundingBox();
  const before = await canvas.screenshot();
  await canvas.click({ position: { x: 49 + (box.width - 63) / 2, y: box.height / 2 } });
  await expect(page.locator("#curveSelectedTime")).toHaveText("1.100 s");
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong').first()).toHaveText("15.00°");
  expect(before.equals(await canvas.screenshot())).toBe(false);
  await page.getByRole("button", { name: "相对角", exact: true }).click();
  await expect(page.locator("#nodeCharts")).toBeHidden();
  await expect(page.locator('[data-chart="relative"] .signal-readouts strong')).toHaveText(["7.50°", "7.50°", "15.00°"]);
  await page.screenshot({ path: "qa/curves-relative-desktop.png", fullPage: true });
  await page.getByRole("tab", { name: "姿态重建", exact: true }).click();
  await expect(page.locator(".spine-panel .timeline-panel")).toHaveCount(1);
  await expect(page.locator('[data-node="C0"] [data-field="roll"]')).toHaveText("+15.0°");
  await expect(page.locator("#c0c7Angle")).toHaveText("15.0°");
  await expect(page.locator("#timelineCurrent")).toHaveText("1.100 s");
  expect(errors).toEqual([]);
});

test("signal controls, zoom, keyboard selection and neutral reference remain consistent", async ({ page }) => {
  const errors = await openKnownPoses(page);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  const canvas = page.locator('[data-chart="C0"] canvas');
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong').first()).toHaveText("30.00°");
  await page.getByRole("button", { name: "Roll", exact: true }).click();
  await expect(page.getByRole("button", { name: "Roll", exact: true })).toHaveAttribute("aria-pressed", "false");
  await page.locator("#curveSignal").selectOption("acceleration");
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["—", "—", "—"]);
  await expect(page.locator("#curveReference")).toContainText("传输原始值");
  await page.locator("#curveSignal").selectOption("gyroscope");
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["—", "—", "—"]);
  await page.locator("#curveSignal").selectOption("orientation");
  await canvas.hover(); await page.mouse.wheel(0, -200);
  await expect(page.locator("#curveHint")).toContainText("手动范围");
  await page.locator("#resetCurveRange").click();
  await expect(page.locator("#curveHint")).not.toContainText("手动范围");
  await page.locator("#neutralButton").click();
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong').first()).toHaveText("0.00°");
  await page.getByRole("button", { name: "相对角", exact: true }).click();
  await expect(page.locator('[data-chart="relative"] .signal-readouts strong')).toHaveText(["0.00°", "0.00°", "0.00°"]);
  await page.locator('[data-chart="relative"] canvas').focus(); await page.keyboard.press("Home");
  await expect(page.locator("#curveSelectedTime")).toHaveText("1.000 s");
  await expect(page.locator('[data-chart="relative"] .signal-readouts strong')).toHaveText(["15.00°", "15.00°", "30.00°"]);
  await page.getByRole("button", { name: "IMU 数据", exact: true }).click();
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong').first()).toHaveText("-30.00°");
  expect(errors).toEqual([]);
});

test("window switching keeps acquisition running and historical selection stable", async ({ page }) => {
  await page.goto("http://127.0.0.1:4173/?demo=1");
  await expect.poll(async () => Number(await page.locator("#historyCount").textContent())).toBeGreaterThan(10);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  await page.locator("#timelineTicks button").first().click();
  const time = await page.locator("#timelineCurrent").textContent();
  const count = Number(await page.locator("#historyCount").textContent());
  await page.getByRole("tab", { name: "采集诊断", exact: true }).click();
  await expect(page.locator("#window-curves")).toBeHidden();
  await expect(page.locator("#window-pose")).toBeHidden();
  await expect(page.locator("#window-diagnostics")).toBeVisible();
  await expect.poll(async () => Number(await page.locator("#historyCount").textContent())).toBeGreaterThan(count + 5);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  await expect(page.locator("#curveSelectedTime")).toHaveText(time);
  await page.locator("#liveButton").click();
  await expect(page.locator("#curveMode")).toHaveText("实时");
  await expect(page.locator("#curveSelectedTime")).not.toHaveText(time);
  await page.locator("#tab-curves").focus(); await page.keyboard.press("ArrowLeft");
  await expect(page.locator("#tab-pose")).toBeFocused();
  await expect(page.locator("#window-pose")).toBeVisible();
});

test("mobile curve window stays within the viewport and preserves selected data", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  const errors = await openKnownPoses(page);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong').first()).toHaveText("30.00°");
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  await page.screenshot({ path: "qa/curves-mobile.png", fullPage: true });
  await page.getByRole("button", { name: "相对角", exact: true }).click();
  await expect(page.locator('[data-chart="relative"] .signal-readouts strong')).toHaveText(["15.00°", "15.00°", "30.00°"]);
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test("transmitted acceleration and gyro vectors remain unscaled through seeking", async ({ page }) => {
  const errors = await openKnownPoses(page, true);
  await page.getByRole("tab", { name: "数据曲线", exact: true }).click();
  await page.locator("#curveSignal").selectOption("acceleration");
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["30.00", "9.81", "0.00"]);
  await expect(page.locator('[data-chart="C4"] .signal-readouts strong')).toHaveText(["30.00", "9.81", "1.00"]);
  await page.locator("#curveSignal").selectOption("gyroscope");
  await expect(page.locator('[data-chart="C7"] .signal-readouts strong')).toHaveText(["3.00", "2.00", "-2.00"]);
  await page.locator("#previousFrame").click();
  await expect(page.locator('[data-chart="C0"] .signal-readouts strong')).toHaveText(["1.50", "0.00", "-2.00"]);
  await page.locator("#curveSignal").selectOption("acceleration");
  await expect(page.locator('[data-chart="C4"] .signal-readouts strong')).toHaveText(["15.00", "9.81", "1.00"]);
  expect(errors).toEqual([]);
});
