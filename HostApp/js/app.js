import { DemoSource } from "./demo.js";
import { attachMeasurement, DesktopSerialSource, desktopHost } from "./desktop.js";
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
  lastSequence: null, lastHistorySequence: null, framesThisWindow: 0, rateWindowAt: performance.now(), rows: [], dataGap: false,
};
const pendingCommands = new Map();

function beginSource() {
  // A newly connected device may reuse a sequence seen in an older session.
  // Never assemble its partial frame using previous devices' node samples.
  state.latest = {}; state.nodeSequences = {};
  state.lastSequence = null; state.lastHistorySequence = null; state.dataGap = true;
}

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
  elements.lamp.className = `status-lamp ${mode === "serial" || mode === "demo" || mode === "measure" ? "status-live" : mode === "error" ? "status-error" : "status-idle"}`;
  elements.connect.querySelector("span:last-child").textContent = mode === "measure" ? "查看正式测量" : mode === "serial" ? "断开串口" : "连接串口";
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
  if (sample.valid === false || !sample.q) {
    for (const field of ["roll", "pitch", "yaw", "quaternion", "calibration"]) $(`[data-field='${field}']`, card).textContent = "—";
    $(".node-state", card).textContent = "无效 / 重连中";
    $(".node-state", card).classList.remove("live");
    return null; // Freeze the last valid chip pose; never invent identity data.
  }
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
  const displayMode = timeline.live && state.measureKind !== "REPLAY" ? "实时" : "回放";
  for (const node of NODE_IDS) adjusted[node] = updateCard(node, snapshot.samples[node], immediate, displayMode);
  spineView.update(adjusted, immediate);
  updateRelativeAngles(adjusted);
  elements.modelState.textContent = displayMode === "实时" ? "驾驶员三节点实时驱动" : "历史姿态回放";
  elements.timestamp.textContent = Number.isFinite(snapshot.timestampUs) ? `${(snapshot.timestampUs / 1_000_000).toFixed(3)} s` : "—";
}

function buildCompleteSnapshot(frame) {
  const allCurrent = NODE_IDS.every((node) => state.latest[node]?.q && state.latest[node].valid !== false && state.nodeSequences[node] === frame.sequence);
  if (!allCurrent || frame.sequence === state.lastHistorySequence) return null;
  state.lastHistorySequence = frame.sequence;
  const samples = Object.fromEntries(NODE_IDS.map((node) => [node, structuredClone(state.latest[node])]));
  const timestampUs = Math.max(...NODE_IDS.map((node) => samples[node].timestampUs));
  return {
    sequence: frame.sequence, timestampUs, receivedAt: Date.now(), samples, vectorUnits: frame.vectorUnits,
    dataGap: state.dataGap,
    angleDegrees: quaternionAngleDegrees(relativeQuaternion(samples.C7.q, samples.C0.q)),
  };
}

function processFrame(frame, rawLine = "") {
  state.lastFrameAt = performance.now();
  if (state.lastSequence !== null && frame.sequence !== state.lastSequence) {
    const delta = (frame.sequence - state.lastSequence) >>> 0;
    if (delta < 0x80000000) state.lostCount += delta - 1;
    else { state.lastHistorySequence = null; state.dataGap = true; }
    if (delta !== 1) state.dataGap = true;
  }
  state.lastSequence = frame.sequence;
  for (const sample of frame.samples) {
    state.latest[sample.node] = sample;
    state.nodeSequences[sample.node] = frame.sequence;
  }
  // Reception continues during historical playback, including quality gaps.
  if (frame.samples.some((sample) => sample.valid === false)) state.dataGap = true;

  const snapshot = buildCompleteSnapshot(frame);
  if (snapshot) { timeline.add(snapshot); state.dataGap = false; }
  else if (timeline.live) {
    // A single-node bring-up stream remains visible before all three nodes
    // have produced a matching acquisition cycle for model reconstruction.
    for (const sample of frame.samples) updateCard(sample.node, sample, false, "实时");
    if (frame.samples.some((sample) => sample.valid === false)) {
      elements.modelState.textContent = "节点未就绪 · 保留上次完整姿态";
      for (const id of ["c0c4Angle", "c4c7Angle", "c0c7Angle"]) $(`#${id}`).textContent = "—";
    }
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
    if (sample.valid === false || !sample.q) {
      state.rows.push([receivedAt, frame.sequence, sample.node, sample.timestampRawUs ?? sample.timestampUs, "", "", "", "", "", "", "", sample.calibration, sample.flags]);
      continue;
    }
    const q = adjustedSample(sample.node, sample).q;
    const e = quaternionToEulerDegrees(q);
    state.rows.push([receivedAt, frame.sequence, sample.node, sample.timestampRawUs ?? sample.timestampUs, q.w, q.x, q.y, q.z, e.roll, e.pitch, e.yaw, sample.calibration, sample.flags]);
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

const serial = new (desktopHost ? DesktopSerialSource : SerialSource)({
  onLine: handleLine,
  onFrame: (frame) => processFrame(frame, `BINARY v1 · seq=${frame.sequence} · ${frame.samples.length} nodes`),
  onReply: (reply) => {
    window.clearTimeout(pendingCommands.get(reply.requestId)); pendingCommands.delete(reply.requestId);
    $("#commandResult").textContent = `请求 ${reply.requestId} · 命令 ${reply.command} · ${reply.status === 0 ? "成功" : `状态 ${reply.status}`} · 返回 ${reply.value}`;
  },
  onProtocolError: (error) => { state.parseErrors++; elements.parseErrors.textContent = state.parseErrors.toLocaleString(); elements.latestLine.textContent = error.message; },
  onBytes: (count) => { state.bytes += count; elements.byteCount.textContent = formatBytes(state.bytes); },
  onState: (serialState) => {
    if (serialState === "connected") beginSource();
    setConnection(serialState === "connected" ? "serial" : "idle", serialState === "connected" ? "串口采集中" : "等待连接");
  },
  onError: (error) => { setConnection("error", "串口异常"); showToast(error.message); },
});
const demo = new DemoSource(processFrame);

function markDesktopStale(label) {
  if (!timeline.live) return;
  for (const node of NODE_IDS) updateCard(node, { valid: false }, true, "等待数据");
  for (const id of ["c0c4Angle", "c4c7Angle", "c0c7Angle"]) $("#"+id).textContent = "—";
  elements.modelState.textContent = label;
}

if (desktopHost) {
  $(".device-commands").style.display = "none";
  $("#desktopMeasurementHelp").hidden = false;
  const resetMeasurement = () => {
    beginSource();
    Object.assign(state, { neutral: {}, rows: [], frameCount: 0, lostCount: 0, lastFrameAt: null });
    timeline.reset(); curves.invalidate();
    elements.frameCount.textContent = elements.lostCount.textContent = "0";
    elements.timestamp.textContent = elements.syncSpan.textContent = "—";
    markDesktopStale("等待三节点原始姿态");
  };
  attachMeasurement({
    start(kind) {
      demo.stop(); resetMeasurement();
      state.measureKind = kind;
      const label = kind === "SERIAL" ? "共用测量串口" : kind === "REPLAY" ? "正式测量 BIN 回放" : "测量算法模拟";
      setConnection("measure", label);
      elements.demo.disabled = true;
    },
    end() {
      state.measureKind = null;
      setConnection("idle", "测量已停止");
      elements.demo.disabled = false;
      markDesktopStale("测量已停止");
    },
    onBytes(count) { state.bytes += count; elements.byteCount.textContent = formatBytes(state.bytes); },
    onFrame: processFrame,
    onBoundary: resetMeasurement,
    onIssue(message) { elements.latestLine.textContent = message; },
  }).catch((error) => showToast(error.message));
}

elements.connect.addEventListener("click", async () => {
  if (state.mode === "measure") { desktopHost.postMessage("measurement-tab"); return; }
  elements.connect.disabled = elements.demo.disabled = true;
  try {
    if (state.mode === "serial") await serial.disconnect();
    else {
      if (state.mode === "demo") demo.stop();
      await serial.connect(Number(elements.baudRate.value));
    }
  } catch (error) {
    setConnection("error", "连接失败"); showToast(error.message);
  } finally {
    elements.connect.disabled = false; elements.demo.disabled = state.mode === "measure";
  }
});

elements.demo.addEventListener("click", async () => {
  if (state.mode === "demo") { demo.stop(); setConnection("idle", "等待连接"); return; }
  if (state.mode === "serial") await serial.disconnect();
  beginSource(); demo.start(); setConnection("demo", "模拟数据运行中");
});

elements.neutral.addEventListener("click", () => {
  const liveSerial = state.mode === "measure" && state.measureKind === "SERIAL";
  const reference = timeline.live && liveSerial ? state.latest : timeline.currentSnapshot()?.samples ?? state.latest;
  if ((timeline.live && liveSerial && performance.now() - state.lastFrameAt > 500) ||
      !NODE_IDS.every((node) => reference[node]?.q)) { showToast("需要先接收到 C0、C4、C7 三个节点的有效数据"); return; }
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
  if (desktopHost) { desktopHost.postMessage("csv\n" + csv); return; }
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([csv], { type: "text/csv;charset=utf-8" }));
  link.download = `spinal_imu_${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
  link.click(); URL.revokeObjectURL(link.href);
});

window.setInterval(() => {
  const now = performance.now();
  if (state.mode === "measure" && state.measureKind === "SERIAL" && state.lastFrameAt !== null && now - state.lastFrameAt > 500)
    markDesktopStale("等待新帧");
  const seconds = (now - state.rateWindowAt) / 1000;
  elements.sampleRate.textContent = (state.framesThisWindow / seconds).toFixed(1);
  state.framesThisWindow = 0; state.rateWindowAt = now;
}, 1000);

if (!desktopHost && !SerialSource.supported()) elements.connect.title = "需要桌面版 Chrome 或 Edge 的 Web Serial 支持";
let requestId = 0;
document.querySelectorAll("[data-device-command]").forEach((button) => button.addEventListener("click", async () => {
  if (state.mode !== "serial") { showToast("设备命令需要连接真实串口"); return; }
  const id = ++requestId;
  try {
    button.disabled = true;
    $("#commandResult").textContent = `请求 ${id} 已发送，等待设备应答`;
    pendingCommands.set(id, window.setTimeout(() => {
      pendingCommands.delete(id);
      $("#commandResult").textContent = `请求 ${id} 应答超时，请检查串口、固件和设备状态`;
    }, 3000));
    await serial.sendCommand(Number(button.dataset.deviceCommand), id,
      button.dataset.deviceCommand === "5" ? Number($("#devicePeriod").value) : 0);
  } catch (error) {
    window.clearTimeout(pendingCommands.get(id)); pendingCommands.delete(id);
    $("#commandResult").textContent = `请求 ${id} 发送失败：${error.message}`; showToast(error.message);
  }
  finally { button.disabled = false; }
}));
if (new URLSearchParams(window.location.search).get("demo") === "1") {
  demo.start(); setConnection("demo", "模拟数据运行中");
}
