import { AXIS_COLORS, PAIRS, SIGNALS, signalMetadata, envelopePoints, frameSignals, nearestFrameIndex } from "./signals.js";
import { NODE_IDS } from "./protocol.js";

const $ = (selector) => document.querySelector(selector);
const formatValue = (value, unit) => Number.isFinite(value) ? `${value.toFixed(2)}${unit === "°" ? "°" : ""}` : "—";
const timeLabel = (seconds) => `${seconds.toFixed(3)} s`;
const MARGIN = { left: 49, right: 14, top: 22, bottom: 35 };

// The timeline owns immutable raw frames and selection. This view only caches
// derived signals, invalidating them when the neutral reference changes. All
// charts share one x domain, hover cursor and selected frame; no second buffer.
export class SignalCurves {
  constructor({ timeline, getNeutral }) {
    this.timeline = timeline; this.getNeutral = getNeutral;
    this.active = false; this.mode = "nodes"; this.signal = "orientation";
    this.visible = [true, true, true]; this.customDomain = null; this.hoverIndex = -1;
    this.cache = new WeakMap(); this.cards = []; this.timer = null; this.lastDraw = 0;
    this.nodes = $("#nodeCharts"); this.relative = $("#relativeCharts");
    for (const node of NODE_IDS) this.cards.push(this.makeCard(this.nodes, node, { C0: "枕骨区", C4: "中颈区", C7: "颈胸交界" }[node]));
    this.cards.push(this.makeCard(this.relative, "relative", "区域相对旋转角"));
    for (const button of document.querySelectorAll("[data-curve-mode]")) button.addEventListener("click", () => this.setMode(button.dataset.curveMode));
    $("#curveSignal").addEventListener("change", (event) => { this.signal = event.target.value; this.visible = [true, true, true]; this.clearHover(); this.renderLegend(); this.schedule(); });
    $("#curveTimeWindow").addEventListener("change", () => { this.customDomain = null; this.clearHover(); this.schedule(); });
    $("#resetCurveRange").addEventListener("click", () => { this.customDomain = null; this.clearHover(); this.schedule(); });
    this.timeline.subscribe(() => this.schedule());
    this.renderLegend();
  }

  signals(snapshot) {
    if (!this.cache.has(snapshot)) this.cache.set(snapshot, frameSignals(snapshot, this.getNeutral()));
    return this.cache.get(snapshot);
  }

  invalidate() { this.cache = new WeakMap(); this.clearHover(); this.schedule(); }
  clearHover() { this.hoverIndex = -1; for (const card of this.cards) card.tooltip.hidden = true; }
  setActive(active) { this.active = active; this.clearHover(); this.drag = null; if (active) this.schedule(); }

  schedule() {
    if (!this.active || this.timer !== null) return;
    // Acquisition may run at 50–100 Hz; chart painting is capped at 10 Hz.
    this.timer = window.setTimeout(() => {
      this.timer = null;
      if (!this.active) return;
      this.lastDraw = performance.now(); this.draw();
    }, Math.max(0, 100 - (performance.now() - this.lastDraw)));
  }

  setMode(mode) {
    this.mode = mode; this.visible = [true, true, true]; this.clearHover();
    this.nodes.hidden = mode !== "nodes"; this.relative.hidden = mode !== "relative";
    $("#signalControl").hidden = mode !== "nodes";
    for (const button of document.querySelectorAll("[data-curve-mode]")) button.setAttribute("aria-pressed", String(button.dataset.curveMode === mode));
    this.renderLegend(); this.schedule();
  }

  descriptors() {
    return this.mode === "relative" ? PAIRS : SIGNALS[this.signal].axes.map((label, index) => ({ label, color: AXIS_COLORS[index] }));
  }

  renderLegend() {
    const legend = $("#curveLegend"); legend.replaceChildren();
    this.descriptors().forEach(({ label, color }, index) => {
      const button = document.createElement("button"); button.type = "button";
      button.style.setProperty("--trace-color", color); button.setAttribute("aria-pressed", String(this.visible[index]));
      const line = document.createElement("i"); button.append(line, document.createTextNode(label));
      button.addEventListener("click", () => { this.visible[index] = !this.visible[index]; button.setAttribute("aria-pressed", String(this.visible[index])); this.schedule(); });
      legend.append(button);
    });
  }

  makeCard(parent, key, title) {
    const root = document.createElement("section"); root.className = "signal-card"; root.dataset.chart = key;
    const header = document.createElement("header");
    const titleWrap = document.createElement("div"), name = document.createElement("strong"), subtitle = document.createElement("span");
    name.textContent = key === "relative" ? title : key; subtitle.textContent = key === "relative" ? "三组节点对比" : title;
    titleWrap.append(name, subtitle); header.append(titleWrap);
    const unit = document.createElement("span"); unit.className = "signal-unit"; header.append(unit);
    const plot = document.createElement("div"); plot.className = "signal-plot";
    const canvas = document.createElement("canvas"); canvas.tabIndex = 0; canvas.setAttribute("role", "slider");
    canvas.setAttribute("aria-label", `${key === "relative" ? "相对角" : key} 曲线时间游标`);
    const tooltip = document.createElement("div"); tooltip.className = "signal-tooltip"; tooltip.hidden = true;
    plot.append(canvas, tooltip);
    const readouts = document.createElement("div"); readouts.className = "signal-readouts";
    const values = Array.from({ length: 3 }, () => {
      const item = document.createElement("div"), label = document.createElement("span"), value = document.createElement("strong");
      item.append(label, value); readouts.append(item); return { item, label, value };
    });
    root.append(header, plot, readouts); parent.append(root);
    const card = { root, key, unit, plot, canvas, tooltip, values };
    new ResizeObserver(() => this.schedule()).observe(plot);
    canvas.addEventListener("pointermove", (event) => this.onPointerMove(event, card));
    canvas.addEventListener("pointerleave", () => { if (!this.drag) { this.hoverIndex = -1; tooltip.hidden = true; this.schedule(); } });
    canvas.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || !this.timeline.history.length) return;
      this.drag = { card, startX: event.clientX, domain: this.domain(), moved: false };
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointerup", (event) => {
      if (!this.drag || this.drag.card !== card) return;
      const moved = this.drag.moved; this.drag = null;
      if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
      if (!moved) this.selectAt(event, card);
      this.schedule();
    });
    canvas.addEventListener("pointercancel", () => { this.drag = null; this.hoverIndex = -1; tooltip.hidden = true; this.schedule(); });
    canvas.addEventListener("wheel", (event) => {
      if (this.timeline.history.length < 2) return;
      event.preventDefault();
      const [start, end] = this.domain(), fraction = this.fractionAt(event, card), anchor = start + fraction * (end - start);
      const extent = this.extent(), span = Math.min(extent[1] - extent[0], Math.max(0.01, (end - start) * Math.exp(Math.sign(event.deltaY) * 0.22)));
      this.customDomain = this.clampDomain([anchor - span * fraction, anchor + span * (1 - fraction)]);
      this.schedule();
    }, { passive: false });
    canvas.addEventListener("keydown", (event) => {
      const steps = { ArrowLeft: -1, ArrowRight: 1 };
      if (event.key in steps) { event.preventDefault(); this.timeline.seek(this.timeline.index + steps[event.key]); }
      if (event.key === "Home" || event.key === "End") { event.preventDefault(); this.timeline.seek(event.key === "Home" ? 0 : this.timeline.history.length - 1); }
    });
    return card;
  }

  extent() {
    const history = this.timeline.history;
    return history.length ? [history[0].sessionSeconds, Math.max(history[0].sessionSeconds + 0.01, history.at(-1).sessionSeconds)] : [0, 1];
  }

  clampDomain(domain) {
    const [min, max] = this.extent(), span = Math.min(max - min, Math.max(0.01, domain[1] - domain[0]));
    const start = Math.max(min, Math.min(max - span, domain[0]));
    return [start, start + span];
  }

  domain() {
    if (this.customDomain) return this.clampDomain(this.customDomain);
    const extent = this.extent(), windowValue = $("#curveTimeWindow").value;
    if (windowValue === "all") return extent;
    const seconds = Number(windowValue), selected = this.timeline.currentSnapshot();
    if (extent[1] - extent[0] <= seconds) return extent;
    const end = this.timeline.live ? extent[1] : Math.min(extent[1], (selected?.sessionSeconds ?? extent[1]) + seconds / 2);
    return this.clampDomain([end - seconds, end]);
  }

  fractionAt(event, card) {
    const bounds = card.canvas.getBoundingClientRect();
    return Math.max(0, Math.min(1, (event.clientX - bounds.left - MARGIN.left) / Math.max(1, bounds.width - MARGIN.left - MARGIN.right)));
  }

  indexAt(event, card) {
    const [start, end] = this.domain();
    return nearestFrameIndex(this.timeline.history, start + this.fractionAt(event, card) * (end - start));
  }

  selectAt(event, card) {
    const index = this.indexAt(event, card);
    if (index >= 0) this.timeline.seek(index);
  }

  onPointerMove(event, card) {
    if (this.drag?.card === card) {
      const delta = event.clientX - this.drag.startX;
      if (Math.abs(delta) > 4) this.drag.moved = true;
      if (this.drag.moved) {
        const width = card.canvas.clientWidth - MARGIN.left - MARGIN.right;
        const shift = -delta / width * (this.drag.domain[1] - this.drag.domain[0]);
        this.customDomain = this.clampDomain(this.drag.domain.map((value) => value + shift));
        card.tooltip.hidden = true; this.hoverIndex = -1; this.schedule(); return;
      }
    }
    this.hoverIndex = this.indexAt(event, card);
    const snapshot = this.timeline.history[this.hoverIndex];
    if (!snapshot) return;
    const signals = this.valuesFor(snapshot, card.key), unit = this.mode === "relative" ? "°" : signalMetadata(this.signal, snapshot).unit;
    card.tooltip.replaceChildren();
    const time = document.createElement("strong"); time.textContent = `设备 ${timeLabel(snapshot.timestampUs / 1_000_000)}`; card.tooltip.append(time);
    this.descriptors().forEach(({ label, color }, index) => {
      if (!this.visible[index]) return;
      const line = document.createElement("span"); line.style.color = color; line.textContent = `${label}  ${formatValue(signals[index], unit)}`; card.tooltip.append(line);
    });
    card.tooltip.hidden = false;
    const left = event.clientX - card.canvas.getBoundingClientRect().left;
    card.tooltip.style.left = `${Math.max(4, Math.min(left + 12, card.canvas.clientWidth - 164))}px`;
    this.schedule();
  }

  valuesFor(snapshot, key) {
    const data = this.signals(snapshot);
    return key === "relative" ? PAIRS.map(({ key: pair }) => data.relative[pair]) : data.nodes[key][this.signal] ?? [null, null, null];
  }

  draw() {
    const history = this.timeline.history, selected = this.timeline.currentSnapshot(), descriptors = this.descriptors();
    const mode = this.timeline.live ? "实时" : this.timeline.playing ? "回放" : "历史";
    $("#curveMode").textContent = mode; $("#curveMode").classList.toggle("history", !this.timeline.live);
    $("#curveSelectedTime").textContent = selected ? timeLabel(selected.timestampUs / 1_000_000) : "—";
    $("#curveReference").textContent = this.mode === "relative" ? "最短相对旋转角 / ° · 与模型使用相同中立位" : signalMetadata(this.signal, selected).note;
    const domain = this.domain();
    const first = Math.max(0, nearestFrameIndex(history, domain[0]) - 1), last = Math.min(history.length - 1, nearestFrameIndex(history, domain[1]) + 1);
    const frames = history.slice(first, last + 1);
    $("#curveHint").textContent = history.length ? `会话 ${domain[0].toFixed(2)}–${domain[1].toFixed(2)} s · 缓存 ${history.length.toLocaleString()} 帧${this.customDomain ? " · 手动范围" : ""}` : "等待完整三节点数据；可先运行模拟数据。";
    for (const card of this.cards) {
      if ((card.key === "relative") !== (this.mode === "relative")) continue;
      const unit = this.mode === "relative" ? "°" : signalMetadata(this.signal, selected).unit;
      card.unit.textContent = unit;
      const values = selected ? this.valuesFor(selected, card.key) : [null, null, null];
      card.values.forEach((item, index) => {
        item.label.textContent = descriptors[index].label; item.label.style.color = descriptors[index].color;
        item.value.textContent = formatValue(values[index], unit); item.value.dataset.value = Number.isFinite(values[index]) ? values[index].toFixed(6) : "";
        item.item.classList.toggle("trace-muted", !this.visible[index]);
      });
      this.drawCard(card, frames, domain, descriptors, selected);
    }
  }

  drawCard(card, frames, domain, descriptors, selected) {
    const canvas = card.canvas, width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width || !height) return;
    const dpr = Math.min(devicePixelRatio, 2), ctx = canvas.getContext("2d");
    canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr); ctx.scale(dpr, dpr);
    const { left, right, top, bottom } = MARGIN, plotWidth = width - left - right, plotHeight = height - top - bottom;
    const x = (time) => left + (time - domain[0]) / (domain[1] - domain[0]) * plotWidth;
    const series = descriptors.map((_, index) => frames.map((frame) => ({ time: frame.sessionSeconds,
      value: frame.vectorUnits !== selected?.vectorUnits && this.signal !== "orientation" && this.mode !== "relative" ? null : this.valuesFor(frame, card.key)[index],
      gap: frame.clockReset || frame.dataGap })));
    let min = 0, max = 0, hasValues = false;
    series.forEach((points, index) => {
      if (!this.visible[index]) return;
      for (const { value } of points) if (Number.isFinite(value)) { hasValues = true; min = Math.min(min, value); max = Math.max(max, value); }
    });
    const pad = Math.max((max - min) * 0.14, this.signal === "orientation" || this.mode === "relative" ? 2 : 0.1);
    if (this.mode !== "relative") min -= pad;
    max += pad;
    const y = (value) => top + (max - value) / (max - min) * plotHeight;
    ctx.font = "10px Inter, Segoe UI, sans-serif"; ctx.lineWidth = 1;
    for (let row = 0; row < 5; row += 1) {
      const value = min + (max - min) * row / 4, py = y(value);
      ctx.strokeStyle = "#e8eef5"; ctx.beginPath(); ctx.moveTo(left, py); ctx.lineTo(width - right, py); ctx.stroke();
      ctx.fillStyle = "#8697ad"; ctx.textAlign = "right"; ctx.fillText(Math.abs(value) < 0.001 ? "0" : value.toFixed(Math.max(Math.abs(min), Math.abs(max)) < 2 ? 2 : 1), left - 9, py + 3);
    }
    for (let col = 0; col < 4; col += 1) {
      const time = domain[0] + (domain[1] - domain[0]) * col / 3, px = x(time);
      ctx.strokeStyle = "#f0f3f8"; ctx.beginPath(); ctx.moveTo(px, top); ctx.lineTo(px, height - bottom); ctx.stroke();
      ctx.fillStyle = "#8697ad"; ctx.textAlign = col === 3 ? "right" : col === 0 ? "left" : "center";
      ctx.fillText(time.toFixed(domain[1] - domain[0] < 1 ? 2 : 1), px, height - bottom + 17);
    }
    ctx.textAlign = "right"; ctx.fillStyle = "#9babbc"; ctx.font = "9px Inter, Microsoft YaHei UI, sans-serif"; ctx.fillText("会话时间 / s", width - right, height - 3);
    ctx.save(); ctx.beginPath(); ctx.rect(left, top - 1, plotWidth, plotHeight + 2); ctx.clip();
    series.forEach((points, index) => {
      if (!this.visible[index]) return;
      const decimated = envelopePoints(points, Math.ceil(plotWidth / 2), this.mode === "nodes" && this.signal === "orientation");
      ctx.strokeStyle = descriptors[index].color; ctx.lineWidth = 1.7; ctx.lineJoin = "round"; ctx.beginPath();
      let previous = null;
      for (const point of decimated) {
        if (!Number.isFinite(point.value)) { previous = null; continue; }
        if (!previous || point.gap) ctx.moveTo(x(point.time), y(point.value));
        else ctx.lineTo(x(point.time), y(point.value));
        previous = point;
      }
      ctx.stroke();
      // Single-frame streams still show an honest point, not an empty line.
      if (points.length === 1 && Number.isFinite(points[0].value)) { ctx.fillStyle = descriptors[index].color; ctx.beginPath(); ctx.arc(x(points[0].time), y(points[0].value), 2.5, 0, Math.PI * 2); ctx.fill(); }
    });
    const hover = this.timeline.history[this.hoverIndex], cursor = hover ?? selected;
    if (cursor) {
      ctx.strokeStyle = hover ? "#899ab1" : this.timeline.live ? "#789bc7" : "#b58c42"; ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x(cursor.sessionSeconds), top); ctx.lineTo(x(cursor.sessionSeconds), height - bottom); ctx.stroke(); ctx.setLineDash([]);
      this.valuesFor(cursor, card.key).forEach((value, index) => {
        if (!this.visible[index] || !Number.isFinite(value)) return;
        ctx.fillStyle = "#fff"; ctx.strokeStyle = descriptors[index].color; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(x(cursor.sessionSeconds), y(value), 3, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      });
    }
    ctx.restore();
    if (!hasValues || !this.visible.some(Boolean)) {
      ctx.fillStyle = "#f8fafdde"; ctx.fillRect(left, top, plotWidth, plotHeight);
      ctx.textAlign = "center"; ctx.font = "12px Microsoft YaHei UI, sans-serif"; ctx.fillStyle = "#8193aa";
      ctx.fillText(!this.visible.some(Boolean) ? "勾选图例以显示曲线" : !frames.length ? "等待完整三节点数据" : "当前帧未提供此信号", left + plotWidth / 2, top + plotHeight / 2);
    }
    canvas.setAttribute("aria-valuemin", String(this.timeline.history[0]?.sessionSeconds ?? 0));
    canvas.setAttribute("aria-valuemax", String(this.timeline.history.at(-1)?.sessionSeconds ?? 0));
    canvas.setAttribute("aria-valuenow", String(selected?.sessionSeconds ?? 0));
    canvas.setAttribute("aria-valuetext", selected ? `设备时间 ${timeLabel(selected.timestampUs / 1_000_000)}` : "等待采集数据");
  }
}
