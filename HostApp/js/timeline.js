import { sessionPosition } from "./signals.js";

const MAX_HISTORY_FRAMES = 30_000;
const formatDeviceTime = (us) => `${(us / 1_000_000).toFixed(3)} s`;
function formatHostTime(ms) {
  const date = new Date(ms), pad = (value, width = 2) => String(value).padStart(width, "0");
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
}

// Selected history and incoming acquisition have separate state. Seeking never
// stops reception. Call onSelect with an immutable, complete three-node frame.
export class PoseTimeline {
  constructor({ range, ticks, canvas, current, hostTime, mode, count, playButton, liveButton, previousButton, nextButton, speed, onSelect }) {
    Object.assign(this, { range, ticks, canvas, current, hostTime, mode, count, playButton, liveButton, previousButton, nextButton, speed, onSelect });
    this.history = []; this.index = 0; this.live = true; this.playing = false; this.playTimer = null;
    this.nextId = 0; this.tickWrites = 0; this.lastDrawAt = 0;
    this.listeners = new Set();
    this.angleFor = (snapshot) => snapshot.angleDegrees;
    range.addEventListener("input", () => this.seek(Number(range.value)));
    playButton.addEventListener("click", () => this.togglePlayback());
    liveButton.addEventListener("click", () => this.goLive());
    previousButton.addEventListener("click", () => this.seek(this.index - 1));
    nextButton.addEventListener("click", () => this.seek(this.index + 1));
    ticks.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-id]");
      if (button) this.seek(Math.max(0, this.history.findIndex((item) => item.historyId === Number(button.dataset.id))));
    });
    canvas.addEventListener("pointerdown", (event) => {
      const bounds = canvas.getBoundingClientRect();
      this.seek(Math.round(Math.max(0, Math.min(1, (event.clientX - bounds.left) / bounds.width)) * (this.history.length - 1)));
    });
    new ResizeObserver(() => this.draw()).observe(canvas);
  }

  add(snapshot) {
    this.history.push({ ...snapshot, ...sessionPosition(this.history.at(-1), snapshot), historyId: this.nextId++ });
    let expiredSelection = false;
    if (this.history.length > MAX_HISTORY_FRAMES) {
      const removed = this.history.length - MAX_HISTORY_FRAMES;
      this.history.splice(0, removed); expiredSelection = !this.live && this.index < removed;
      this.index = Math.max(0, this.index - removed);
    }
    this.range.max = String(this.history.length - 1); this.range.disabled = false;
    this.count.textContent = this.history.length.toLocaleString();
    for (const button of [this.playButton, this.previousButton, this.nextButton]) button.disabled = this.history.length < 2;
    if (this.live) this.index = this.history.length - 1;
    this.range.value = String(this.index);
    if (this.live || expiredSelection) this.present(false);
    this.tickWrites += 1;
    if (this.tickWrites >= 15 || this.history.length < 7) { this.renderTicks(); this.tickWrites = 0; }
    // Keep stream-rate parsing independent from chart paint frequency.
    if (performance.now() - this.lastDrawAt > 60) this.draw();
    this.notify();
  }

  subscribe(listener) { this.listeners.add(listener); return () => this.listeners.delete(listener); }
  notify() { for (const listener of this.listeners) listener(); }

  currentSnapshot() { return this.history[this.index] ?? null; }

  present(immediate) {
    const snapshot = this.currentSnapshot(); if (!snapshot) return;
    this.current.textContent = formatDeviceTime(snapshot.timestampUs);
    this.hostTime.textContent = `接收于 ${formatHostTime(snapshot.receivedAt)}`;
    this.current.dataset.sequence = String(snapshot.sequence);
    this.onSelect(snapshot, immediate);
    this.notify();
  }

  seek(index) {
    if (!this.history.length) return;
    this.stopPlayback(); this.live = false;
    this.index = Math.max(0, Math.min(this.history.length - 1, Math.round(index)));
    this.range.value = String(this.index);
    this.updateMode(); this.present(true); this.draw();
  }

  goLive() {
    this.stopPlayback(); this.live = true;
    this.index = Math.max(0, this.history.length - 1); this.range.value = String(this.index);
    this.updateMode(); this.present(true); this.draw();
  }

  togglePlayback() {
    if (this.playing) { this.stopPlayback(); return; }
    if (this.history.length < 2) return;
    this.live = false;
    if (this.index >= this.history.length - 1) this.index = 0;
    this.playing = true; this.playButton.textContent = "Ⅱ";
    this.playButton.setAttribute("aria-label", "暂停历史姿态");
    this.updateMode(); this.range.value = String(this.index); this.present(true);
    this.scheduleNext();
  }

  scheduleNext() {
    if (!this.playing) return;
    if (this.index >= this.history.length - 1) { this.stopPlayback(); return; }
    const current = this.history[this.index], next = this.history[this.index + 1];
    const deviceDelta = (next.timestampUs - current.timestampUs) / 1000;
    const hostDelta = Math.max(1, next.receivedAt - current.receivedAt);
    const delay = (deviceDelta > 0 ? deviceDelta : hostDelta) / Number(this.speed.value);
    this.playTimer = window.setTimeout(() => {
      this.index = Math.min(this.index + 1, this.history.length - 1);
      this.range.value = String(this.index); this.present(true); this.draw(); this.scheduleNext();
    }, Math.min(60_000, Math.max(1, delay)));
  }

  stopPlayback() {
    if (this.playTimer !== null) window.clearTimeout(this.playTimer);
    this.playTimer = null; this.playing = false; this.playButton.textContent = "▶";
    this.playButton.setAttribute("aria-label", "播放历史姿态"); this.updateMode();
  }

  updateMode() {
    this.mode.textContent = this.live ? "实时" : this.playing ? "回放" : "历史";
    this.mode.classList.toggle("history", !this.live);
    this.liveButton.setAttribute("aria-pressed", String(this.live));
    this.notify();
  }

  renderTicks() {
    this.ticks.replaceChildren(); if (!this.history.length) return;
    const last = this.history.length - 1;
    const indexes = [...new Set(Array.from({ length: 6 }, (_, i) => Math.round(last * i / 5)))];
    for (const index of indexes) {
      const snapshot = this.history[index], button = document.createElement("button");
      button.type = "button"; button.dataset.id = String(snapshot.historyId);
      button.textContent = formatDeviceTime(snapshot.timestampUs);
      button.title = `跳转至第 ${snapshot.sequence} 帧`;
      this.ticks.append(button);
    }
  }

  draw() {
    if (!this.canvas.clientWidth) return;
    this.lastDrawAt = performance.now();
    const ctx = this.canvas.getContext("2d"), dpr = Math.min(devicePixelRatio, 2);
    const width = Math.max(1, this.canvas.clientWidth), height = Math.max(1, this.canvas.clientHeight);
    this.canvas.width = Math.round(width * dpr); this.canvas.height = Math.round(height * dpr); ctx.scale(dpr, dpr);
    ctx.strokeStyle = "#e4edf8"; ctx.lineWidth = 1;
    for (let row = 1; row < 4; row += 1) { const y = height * row / 4; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
    if (this.history.length < 2) return;
    // Render the entire retained interval with a bounded point count; older
    // selected timestamps remain visible rather than disappearing off-chart.
    const last = this.history.length - 1, step = Math.max(1, Math.ceil(this.history.length / 650));
    let maxAngle = 15; for (const frame of this.history) maxAngle = Math.max(maxAngle, this.angleFor(frame));
    const point = (index) => [index / last * width, height - 4 - this.angleFor(this.history[index]) / maxAngle * (height - 22)];
    ctx.beginPath(); ctx.moveTo(...point(0));
    for (let i = step; i < last; i += step) ctx.lineTo(...point(i));
    ctx.lineTo(...point(last)); ctx.strokeStyle = "#598ccc"; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.lineTo(width, height); ctx.lineTo(0, height); ctx.closePath();
    const fill = ctx.createLinearGradient(0, 0, 0, height); fill.addColorStop(0, "#598ccc28"); fill.addColorStop(1, "#598ccc02"); ctx.fillStyle = fill; ctx.fill();
    const [x, y] = point(this.index); ctx.strokeStyle = this.live ? "#8bb4dd" : "#c7a35f";
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.beginPath(); ctx.arc(Math.min(width - 3, Math.max(3, x)), y, 3, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  }
}
