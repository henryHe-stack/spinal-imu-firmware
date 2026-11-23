import * as THREE from "three";
import { OrbitControls } from "../vendor/OrbitControls.js";
import { addGround, createCockpit, createDriver, createVertebra } from "./driver-model.js";

const NODE_COLORS = { C0: 0x2867c4, C4: 0x11999e, C7: 0x8b69b7 };
const CERVICAL_ORIGIN = new THREE.Vector3(0, 3.99, -0.07);
const NODE_INDEX = { C7: 0, C4: 3, C0: 7 };

function sensorQuaternion(sample) {
  return new THREE.Quaternion(sample.q.x, sample.q.y, sample.q.z, sample.q.w).normalize();
}

class InteractiveView {
  constructor(canvas, cameraPosition, target, minDistance, maxDistance) {
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(34, 1, 0.08, 100);
    this.camera.position.copy(cameraPosition);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.25;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.copy(target);
    this.controls.enableDamping = true; this.controls.dampingFactor = 0.09;
    this.controls.minDistance = minDistance; this.controls.maxDistance = maxDistance;
    this.controls.screenSpacePanning = true;
    this.controls.update(); this.controls.saveState();
    this.scene.add(new THREE.HemisphereLight(0xeaf5ff, 0x647789, 2));
    for (const [color, intensity, xyz] of [[0xffffff, 2.8, [4, 8, 6]], [0xc6defa, 1.4, [-5, 4, -3]], [0xffffff, 1, [1, 1, -6]]]) {
      const light = new THREE.DirectionalLight(color, intensity); light.position.set(...xyz); this.scene.add(light);
    }
    this.lastRenderAt = performance.now();
  }

  zoom(factor) {
    const offset = this.camera.position.clone().sub(this.controls.target).multiplyScalar(factor);
    offset.setLength(THREE.MathUtils.clamp(offset.length(), this.controls.minDistance, this.controls.maxDistance));
    this.camera.position.copy(this.controls.target).add(offset); this.controls.update();
  }

  reset() { this.controls.reset(); }

  render() {
    const canvas = this.renderer.domElement;
    // Switching work windows must not resize hidden WebGL views to 1×1 or
    // spend GPU time rendering a model that is currently out of view.
    if (!canvas.clientWidth || !canvas.clientHeight) { this.lastRenderAt = performance.now(); return; }
    const width = canvas.clientWidth, height = canvas.clientHeight;
    const ratio = this.renderer.getPixelRatio();
    if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
      this.renderer.setSize(width, height, false); this.camera.aspect = width / height; this.camera.updateProjectionMatrix();
    }
    const now = performance.now(), deltaSeconds = Math.min((now - this.lastRenderAt) / 1000, 0.1);
    this.lastRenderAt = now;
    this.animate?.(deltaSeconds);
    this.controls.update(); this.renderer.render(this.scene, this.camera);
  }
}

export class ImuView extends InteractiveView {
  constructor(canvas, node) {
    super(canvas, new THREE.Vector3(3.1, 2.9, 3.9), new THREE.Vector3(), 2, 11);
    this.model = new THREE.Group();
    const base = new THREE.Mesh(new THREE.BoxGeometry(1.65, 0.14, 1.12), new THREE.MeshStandardMaterial({ color: 0x32576b, roughness: 0.46, metalness: 0.18 }));
    this.model.add(base);
    const chip = new THREE.Mesh(new THREE.BoxGeometry(0.73, 0.21, 0.63), new THREE.MeshStandardMaterial({ color: NODE_COLORS[node], roughness: 0.4, metalness: 0.20 }));
    chip.position.y = 0.17; this.model.add(chip);
    const pinMaterial = new THREE.MeshStandardMaterial({ color: 0xb8a470, metalness: 0.65, roughness: 0.3 });
    for (const side of [-1, 1]) for (let i = 0; i < 6; i += 1) {
      const pin = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.035, 0.16), pinMaterial); pin.position.set((i - 2.5) * 0.24, 0.09, side * 0.51); this.model.add(pin);
    }
    this.model.add(new THREE.AxesHelper(1.15)); this.scene.add(this.model);
    const grid = new THREE.GridHelper(4, 8, 0xbdcedb, 0xe0e8ee); grid.position.y = -0.55; this.scene.add(grid);
    this.target = new THREE.Quaternion();
  }
  update(sample, immediate = false) { this.target.copy(sensorQuaternion(sample)); if (immediate) this.model.quaternion.copy(this.target); }
  animate(delta) { this.model.quaternion.slerp(this.target, 1 - Math.exp(-20 * delta)); }
}

function nodeLabel(name, color) {
  const canvas = document.createElement("canvas"); canvas.width = 240; canvas.height = 100;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#ffffff"; ctx.strokeStyle = color; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.roundRect(4, 4, 232, 92, 18); ctx.fill(); ctx.stroke();
  ctx.fillStyle = color; ctx.font = "600 42px Segoe UI"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(name, 120, 52);
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
  const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, depthWrite: false, toneMapped: false }));
  label.scale.set(0.49, 0.205, 1); label.renderOrder = 20;
  return label;
}

export class SpineView extends InteractiveView {
  constructor(canvas) {
    super(canvas, new THREE.Vector3(7.4, 5.25, 10.5), new THREE.Vector3(0, 2.8, 0.55), 0.8, 25);
    this.cockpit = createCockpit(); this.driver = createDriver();
    this.scene.add(this.cockpit.group, this.driver.body, this.driver.head); addGround(this.scene);
    this.cervical = []; this.markers = {}; this.labels = {}; this.targets = [];
    const boneMaterial = new THREE.MeshStandardMaterial({ color: 0x87a6bc, roughness: 0.48, metalness: 0.10 });
    // Fixed thoracolumbar reference; its vertebrae do not imply extra sensors.
    for (let index = 0; index < 17; index += 1) {
      const t = index / 16, bone = createVertebra(0.17 - t * 0.033, boneMaterial);
      bone.position.set(0, 1.86 + t * 2.01, -0.08 + Math.sin(t * Math.PI * 1.65) * 0.11); this.scene.add(bone);
    }
    for (let index = 0; index < 8; index += 1) {
      const name = Object.keys(NODE_INDEX).find((node) => NODE_INDEX[node] === index);
      const bone = createVertebra(0.13 - index * 0.003, new THREE.MeshStandardMaterial({ color: name ? NODE_COLORS[name] : 0x71b7cc, roughness: 0.4, metalness: 0.18, emissive: name ? NODE_COLORS[name] : 0x000000, emissiveIntensity: name ? 0.13 : 0 }));
      this.cervical.push(bone); this.targets.push(new THREE.Quaternion()); this.scene.add(bone);
    }
    for (const [name, index] of Object.entries(NODE_INDEX)) {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(0.205, 0.014, 10, 48), new THREE.MeshBasicMaterial({ color: NODE_COLORS[name] }));
      ring.rotation.x = Math.PI / 2;
      const marker = new THREE.Group(); marker.add(ring); this.markers[name] = marker; this.scene.add(marker);
      const label = nodeLabel(name, `#${NODE_COLORS[name].toString(16).padStart(6, "0")}`); this.labels[name] = label; this.scene.add(label);
      const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: NODE_COLORS[name], depthTest: false, transparent: true, opacity: 0.75 }));
      line.renderOrder = 19; marker.userData.leader = line; this.scene.add(line);
    }
    this.driver.head.userData.target = new THREE.Quaternion();
    this.neckShell = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.20, 1, 24), new THREE.MeshStandardMaterial({ color: 0xadc5d5, roughness: 0.55, transparent: true, opacity: 0.20, depthWrite: false }));
    this.scene.add(this.neckShell);
    this.placeAnatomy();
    this.setPreset("driver");
  }

  setSeatGhost(ghost) { this.cockpit.setGhost(ghost); }
  setBodyGhost(ghost) { this.driver.setGhost(ghost); }

  setPreset(name) {
    const presets = {
      driver: { position: [7.4, 5.25, 10.5], target: [0, 2.8, 0.55] },
      back: { position: [-7.5, 5.25, -8.2], target: [0, 2.9, 0.15] },
      neck: { position: [2.1, 4.9, -3.1], target: [0, 4.37, -0.07] },
    };
    const preset = presets[name] ?? presets.driver;
    this.camera.position.set(...preset.position); this.controls.target.set(...preset.target);
    this.controls.update(); this.controls.saveState();
  }

  update(samples, immediate = false) {
    const q0 = sensorQuaternion(samples.C0), q4 = sensorQuaternion(samples.C4), q7 = sensorQuaternion(samples.C7);
    this.cervical.forEach((bone, i) => {
      this.targets[i].copy(i <= 3 ? q7.clone().slerp(q4, i / 3) : q4.clone().slerp(q0, (i - 3) / 4));
      if (immediate) bone.quaternion.copy(this.targets[i]);
    });
    this.driver.head.userData.target.copy(q0);
    if (immediate) { this.driver.head.quaternion.copy(q0); this.placeAnatomy(); }
  }

  placeAnatomy() {
    const position = CERVICAL_ORIGIN.clone();
    this.cervical.forEach((bone, index) => {
      bone.position.copy(position);
      if (index < 7) position.add(new THREE.Vector3(0, 0.093, 0).applyQuaternion(bone.quaternion.clone().slerp(this.cervical[index + 1].quaternion, 0.5)));
    });
    this.driver.head.position.copy(position);
    const neckDirection = position.clone().sub(CERVICAL_ORIGIN);
    this.neckShell.position.copy(CERVICAL_ORIGIN).add(position).multiplyScalar(0.5);
    this.neckShell.scale.y = neckDirection.length();
    this.neckShell.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), neckDirection.normalize());
    for (const [name, index] of Object.entries(NODE_INDEX)) {
      const bone = this.cervical[index], marker = this.markers[name]; marker.position.copy(bone.position); marker.quaternion.copy(bone.quaternion);
      const label = this.labels[name];
      const worldPerPixel = 2 * this.camera.position.distanceTo(bone.position) * Math.tan(THREE.MathUtils.degToRad(this.camera.fov) / 2) / Math.max(1, this.renderer.domElement.clientHeight);
      label.scale.set(47 * worldPerPixel, 20 * worldPerPixel, 1);
      // Stagger leaders in screen space; the anchor follows the actual node.
      const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 0).normalize();
      label.position.copy(bone.position).addScaledVector(right, name === "C4" ? 0.9 : 0.65);
      label.position.y += name === "C0" ? 0.12 : name === "C7" ? -0.10 : 0;
      const attribute = marker.userData.leader.geometry.getAttribute("position");
      attribute.setXYZ(0, bone.position.x, bone.position.y, bone.position.z);
      attribute.setXYZ(1, label.position.x, label.position.y, label.position.z); attribute.needsUpdate = true;
      marker.userData.leader.geometry.computeBoundingSphere();
    }
  }

  animate(delta) {
    const factor = 1 - Math.exp(-20 * delta);
    this.cervical.forEach((bone, index) => bone.quaternion.slerp(this.targets[index], factor));
    this.driver.head.quaternion.slerp(this.driver.head.userData.target, factor);
    this.placeAnatomy();
  }
}

export function bindViewControls(views) {
  document.querySelectorAll(".view-tools").forEach((toolbar) => {
    toolbar.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-action]"), view = views[toolbar.dataset.view];
      if (!button || !view) return;
      if (button.dataset.action === "zoom-in") view.zoom(0.82);
      if (button.dataset.action === "zoom-out") view.zoom(1.22);
      if (button.dataset.action === "reset") view.reset();
    });
  });
  document.querySelectorAll("[data-preset]").forEach((button) => button.addEventListener("click", () => {
    views.spine.setPreset(button.dataset.preset);
    document.querySelectorAll("[data-preset]").forEach((other) => other.setAttribute("aria-pressed", String(other === button)));
  }));
  document.querySelector("#seatGhost").addEventListener("change", (event) => views.spine.setSeatGhost(event.target.checked));
  document.querySelector("#bodyGhost").addEventListener("change", (event) => views.spine.setBodyGhost(event.target.checked));
}

export function startRendering(views) {
  function tick() { for (const view of views) view.render(); requestAnimationFrame(tick); }
  tick();
}
