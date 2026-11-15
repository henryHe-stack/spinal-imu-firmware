import { DemoSource } from "./demo.js";
import { applyNeutral, quaternionAngleDegrees, quaternionToEulerDegrees, relativeQuaternion } from "./math.js";
import { NODE_IDS, parseTelemetryLine, validateCompleteFrame } from "./protocol.js";
import { SerialSource } from "./serial.js";
import { PoseTimeline } from "./timeline.js";
import { SignalCurves } from "./curves.js";
import { bindViewControls, ImuView, SpineView, startRendering } from "./visualization.js";
import { bindWorkspaceWindows } from "./workspace.js";

const $ = (selector, root = document) => root.querySelector(selector);
const elements = {
  baudRate: $("#baudRate"), connect: $("#connectButton"), demo: $("#demoButton"), neutral: $("#neutralButton"), export: $("#exportButton"),
  lamp: $("#connectionLamp"), connection: $("#connectionText"), sampleRate: $("#sampleRate"), frameCount: $("#frameCount"), lostCount: $("#lostCount"),
  timestamp: $("#latestTimestamp"), modelState: $("#modelState"), syncSpan: $("#syncSpan"), parseErrors: $("#parseErrors"), byteCount: $("#byteCount"),
  latestLine: $("#latestLine"), protocol: $("#protocolBadge"), toast: $("#toast"),
  timelineRange: $("#timelineRange"), timelineTicks: $("#timelineTicks"), timelineCanvas: $("#timelineCanvas"), timelineCurrent: $("#timelineCurrent"),
  timelineMode: $("#timelineMode"), historyCount: $("#historyCount"), playback: $("#playbackButton"), live: $("#liveButton"),
  timelineHostTime: $("#timelineHostTime"), previousFrame: $("#previousFrame"), nextFrame: $("#nextFrame"), playbackSpeed: $("#playbackSpeed"),
};

const cards = Object.fromEntries(NODE_IDS.map((node) => [node, $(`.imu-card[data-node="${node}"]`)]));
const imuViews = Object.fromEntries(NODE_IDS.map((node) => [node, new ImuView($(`#imuCanvas${node}`), node)]));
const spineView = new SpineView($("#spineCanvas"));
const views = { spine: spineView, ...imuViews };
bindViewControls(views);
startRendering(Object.values(views));

const state = {
  mode: "idle", latest: {}, nodeSequences: {}, neutral: {}, frameCount: 0, lostCount: 0, parseErrors: 0, bytes: 0,
  lastSequence: null, lastHistorySequence: null, framesThisWindow: 0, rateWindowAt: performance.now(), rows: [],
};

const timeline = new PoseTimeline({
  range: elements.timelineRange, ticks: elements.timelineTicks, canvas: elements.timelineCanvas,
  current: elements.timelineCurrent, mode: elements.timelineMode, count: elements.historyCount,
  playButton: elements.playback, liveButton: elements.live, onSelect: renderSnapshot,
  hostTime: elements.timelineHostTime, previousButton: elements.previousFrame, nextButton: elements.nextFrame, speed: elements.playbackSpeed,
});
const curves = new SignalCurves({ timeline, getNeutral: () => state.neutral });
timeline.angleFor = (snapshot) => curves.signals(snapshot).relative.C0C7;
bindWorkspaceWindows({ onChange: (name) => {
  curves.setActive(name === "curves");
  // The same canvas may have moved between panels; paint after layout settles.
  requestAnimationFrame(() => timeline.draw());
} });

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("visible");
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => elements.toast.classList.remove("visible"), 2600);
}

function setConnection(mode, label) {
  state.mode = mode;
  elements.connection.textContent = label;
  elements.lamp.className = `status-lamp ${mode === "serial" || mode === "demo" ? "status-live" : mode === "error" ? "status-error" : "status-idle"}`;
  elements.connect.querySelector("span:last-child").textContent = mode === "serial" ? "断开串口" : "连接串口";
  elements.demo.textContent = mode === "demo" ? "停止模拟数据" : "运行模拟数据";
}

function formatAngle(value) { return `${value >= 0 ? "+" : ""}${value.toFixed(1)}°`; }
function formatBytes(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

function adjustedSample(node, sample) {
  return { ...sample, q: applyNeutral(sample.q, state.neutral[node]) };
}

function updateCard(node, sample, immediate, displayMode) {
  const card = cards[node];
  const adjusted = adjustedSample(node, sample);
  const euler = quaternionToEulerDegrees(adjusted.q);
  $("[data-field='roll']", card).textContent = formatAngle(euler.roll);
  $("[data-field='pitch']", card).textContent = formatAngle(euler.pitch);
  $("[data-field='yaw']", card).textContent = formatAngle(euler.yaw);
  $("[data-field='quaternion']", card).textContent = `q = [${adjusted.q.w.toFixed(3)}, ${adjusted.q.x.toFixed(3)}, ${adjusted.q.y.toFixed(3)}, ${adjusted.q.z.toFixed(3)}]`;
  $("[data-field='calibration']", card).textContent = `CAL ${Math.round(sample.calibration)}/3`;
  $(".node-state", card).textContent = displayMode;
  $(".node-state", card).classList.toggle("live", displayMode === "实时");
  imuViews[node].update(adjusted, immediate);
  return adjusted;
}

function updateRelativeAngles(samples) {
  const pairs = [["C0", "C4", "c0c4Angle"], ["C4", "C7", "c4c7Angle"], ["C0", "C7", "c0c7Angle"]];
  for (const [from, to, target] of pairs) {
    $(`#${target}`).textContent = `${quaternionAngleDegrees(relativeQuaternion(samples[from].q, samples[to].q)).toFixed(1)}°`;
  }
}

function renderSnapshot(snapshot, immediate = false) {
  if (!snapshot) return;
  const adjusted = {};
  const displayMode = timeline.live ? "实时" : "回放";
  for (const node of NODE_IDS) adjusted[node] = updateCard(node, snapshot.samples[node], immediate, displayMode);
  spineView.update(adjusted, immediate);
  updateRelativeAngles(adjusted);
  elements.modelState.textContent = timeline.live ? "驾驶员三节点实时驱动" : "历史姿态回放";
  elements.timestamp.textContent = Number.isFinite(snapshot.timestampUs) ? `${(snapshot.timestampUs / 1_000_000).toFixed(3)} s` : "—";
}

function buildCompleteSnapshot(frame) {
  const allCurrent = NODE_IDS.every((node) => state.latest[node] && state.nodeSequences[node] === frame.sequence);
  if (!allCurrent || frame.sequence === state.lastHistorySequence) return null;
  state.lastHistorySequence = frame.sequence;
  const samples = Object.fromEntries(NODE_IDS.map((node) => [node, structuredClone(state.latest[node])]));
  const timestampUs = Math.max(...NODE_IDS.map((node) => samples[node].timestampUs));
  return {
    sequence: frame.sequence, timestampUs, receivedAt: Date.now(), samples,
    angleDegrees: quaternionAngleDegrees(relativeQuaternion(samples.C7.q, samples.C0.q)),
  };
}

function processFrame(frame, rawLine = "") {
  if (state.lastSequence !== null && frame.sequence > state.lastSequence + 1) state.lostCount += frame.sequence - state.lastSequence - 1;
  if (frame.sequence > state.lastSequence || state.lastSequence === null) state.lastSequence = frame.sequence;
  for (const sample of frame.samples) {
    state.latest[sample.node] = sample;
    state.nodeSequences[sample.node] = frame.sequence;
  }

  const snapshot = buildCompleteSnapshot(frame);
  if (snapshot) timeline.add(snapshot);
  else if (timeline.live) {
    // A single-node bring-up stream remains visible before all three nodes
    // have produced a matching acquisition cycle for model reconstruction.
    for (const sample of frame.samples) updateCard(sample.node, sample, false, "实时");
  }
  state.frameCount += 1;
  state.framesThisWindow += 1;
  elements.frameCount.textContent = state.frameCount.toLocaleString();
  elements.lostCount.textContent = state.lostCount.toLocaleString();
  const timestamps = frame.samples.map((sample) => sample.timestampUs);
  elements.syncSpan.textContent = validateCompleteFrame(frame) ? `${((Math.max(...timestamps) - Math.min(...timestamps)) / 1000).toFixed(2)} ms` : "等待完整帧";
  elements.protocol.textContent = `协议：${frame.protocol}`;
  if (rawLine) elements.latestLine.textContent = rawLine.length > 220 ? `${rawLine.slice(0, 220)}…` : rawLine;
  appendRows(frame);
}

function appendRows(frame) {
  const receivedAt = new Date().toISOString();
  for (const sample of frame.samples) {
    const q = adjustedSample(sample.node, sample).q;
    const e = quaternionToEulerDegrees(q);
    state.rows.push([receivedAt, frame.sequence, sample.node, sample.timestampUs, q.w, q.x, q.y, q.z, e.roll, e.pitch, e.yaw, sample.calibration, sample.flags]);
  }
  if (state.rows.length > 100_000) state.rows.splice(0, 10_000);
}

function handleLine(line) {
  if (!line.trim()) return;
  try {
    const frame = parseTelemetryLine(line);
    if (frame) processFrame(frame, line);
  } catch (error) {
    state.parseErrors += 1;
    elements.parseErrors.textContent = state.parseErrors.toLocaleString();
    elements.latestLine.textContent = `解析失败：${error.message} · ${line.slice(0, 150)}`;
  }
}

const serial = new SerialSource({
  onLine: handleLine,
  onBytes: (count) => { state.bytes += count; elements.byteCount.textContent = formatBytes(state.bytes); },
  onState: (serialState) => setConnection(serialState === "connected" ? "serial" : "idle", serialState === "connected" ? "串口采集中" : "等待连接"),
  onError: (error) => { setConnection("error", "串口异常"); showToast(error.message); },
});
const demo = new DemoSource(processFrame);

elements.connect.addEventListener("click", async () => {
  try {
    if (state.mode === "serial") await serial.disconnect();
    else {
      if (state.mode === "demo") demo.stop();
      await serial.connect(Number(elements.baudRate.value));
    }
  } catch (error) {
    setConnection("error", "连接失败"); showToast(error.message);
  }
});

elements.demo.addEventListener("click", async () => {
  if (state.mode === "demo") { demo.stop(); setConnection("idle", "等待连接"); return; }
  if (state.mode === "serial") await serial.disconnect();
  demo.start(); setConnection("demo", "模拟数据运行中");
});

elements.neutral.addEventListener("click", () => {
  const reference = timeline.currentSnapshot()?.samples ?? state.latest;
  if (!NODE_IDS.every((node) => reference[node])) { showToast("需要先接收到 C0、C4、C7 三个节点的数据"); return; }
  for (const node of NODE_IDS) state.neutral[node] = { ...reference[node].q };
  curves.invalidate();
  renderSnapshot(timeline.currentSnapshot(), true);
  timeline.draw();
  showToast("显示中的三节点姿态已设为中立位，曲线已同步更新");
});

elements.export.addEventListener("click", () => {
  if (!state.rows.length) { showToast("当前没有可导出的采集数据"); return; }
  const header = "received_at,sequence,node,timestamp_us,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,calibration,flags\n";
  const csv = header + state.rows.map((row) => row.join(",")).join("\n");
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([csv], { type: "text/csv;charset=utf-8" }));
  link.download = `spinal_imu_${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
  link.click(); URL.revokeObjectURL(link.href);
});

window.setInterval(() => {
  const now = performance.now();
  const seconds = (now - state.rateWindowAt) / 1000;
  elements.sampleRate.textContent = (state.framesThisWindow / seconds).toFixed(1);
  state.framesThisWindow = 0; state.rateWindowAt = now;
}, 1000);

if (!SerialSource.supported()) elements.connect.title = "需要桌面版 Chrome 或 Edge 的 Web Serial 支持";
if (new URLSearchParams(window.location.search).get("demo") === "1") {
  demo.start(); setConnection("demo", "模拟数据运行中");
}
