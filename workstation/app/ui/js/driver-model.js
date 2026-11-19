import * as THREE from "three";

// Procedural, locally generated study mannequin. Dimensions are illustrative
// scene units, not subject measurements. Only the cervical chain is IMU-driven.
const UP = new THREE.Vector3(0, 1, 0);
const V = (x, y, z) => new THREE.Vector3(x, y, z);

function ellipsoid(parent, position, scale, material, segments = 24) {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(1, segments, 16), material);
  mesh.position.copy(position); mesh.scale.copy(scale);
  parent.add(mesh);
  return mesh;
}

// Elliptical cross sections produce a continuous shoulder/waist/chest contour.
function loft(profile, material, segments = 36) {
  const positions = [], indices = [];
  profile.forEach(([height, width, depth, offsetZ = 0]) => {
    for (let column = 0; column <= segments; column += 1) {
      const angle = column / segments * Math.PI * 2;
      positions.push(Math.cos(angle) * width, height, Math.sin(angle) * depth + offsetZ);
    }
  });
  for (let row = 0; row < profile.length - 1; row += 1) {
    for (let column = 0; column < segments; column += 1) {
      const a = row * (segments + 1) + column, b = a + segments + 1;
      indices.push(a, b, a + 1, a + 1, b, b + 1);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geometry.setIndex(indices); geometry.computeVertexNormals();
  return new THREE.Mesh(geometry, material);
}

function limb(parent, start, end, radiusStart, radiusEnd, material) {
  const length = start.distanceTo(end);
  const shape = new THREE.Mesh(new THREE.CylinderGeometry(radiusEnd, radiusStart, length, 20, 1), material);
  shape.position.copy(start).add(end).multiplyScalar(0.5);
  shape.quaternion.setFromUnitVectors(UP, end.clone().sub(start).normalize());
  parent.add(shape);
  ellipsoid(parent, start, V(radiusStart, radiusStart * 1.15, radiusStart), material);
  ellipsoid(parent, end, V(radiusEnd, radiusEnd * 1.1, radiusEnd), material);
}

function roundedPanel(size, radius, material) {
  const [width, height, depth] = size;
  const halfW = width / 2 - radius, halfH = height / 2 - radius;
  const shape = new THREE.Shape();
  shape.moveTo(-halfW, -height / 2); shape.lineTo(halfW, -height / 2);
  shape.quadraticCurveTo(width / 2, -height / 2, width / 2, -halfH);
  shape.lineTo(width / 2, halfH); shape.quadraticCurveTo(width / 2, height / 2, halfW, height / 2);
  shape.lineTo(-halfW, height / 2); shape.quadraticCurveTo(-width / 2, height / 2, -width / 2, halfH);
  shape.lineTo(-width / 2, -halfH); shape.quadraticCurveTo(-width / 2, -height / 2, -halfW, -height / 2);
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: depth - radius, bevelEnabled: true, bevelSegments: 4, steps: 1, bevelSize: radius / 2, bevelThickness: radius / 2, curveSegments: 12 });
  geometry.translate(0, 0, -depth / 2);
  return new THREE.Mesh(geometry, material);
}

function curveLine(parent, points, color, opacity = 1, radius = 0.008) {
  const curve = new THREE.CatmullRomCurve3(points);
  const mesh = new THREE.Mesh(new THREE.TubeGeometry(curve, 32, radius, 6, false), new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false }));
  parent.add(mesh);
}

// Collapse static anatomy into one draw per material. The complete mannequin
// retains its contours without hundreds of per-finger/per-joint draw calls.
function mergeStaticMeshes(group) {
  group.updateMatrixWorld(true);
  const batches = new Map();
  group.traverse((part) => {
    if (!part.isMesh) return;
    const source = part.geometry.index ? part.geometry.toNonIndexed() : part.geometry.clone();
    source.applyMatrix4(part.matrixWorld);
    const key = part.material.uuid;
    if (!batches.has(key)) batches.set(key, { material: part.material, geometries: [] });
    batches.get(key).geometries.push(source);
  });
  group.clear();
  for (const { material, geometries } of batches.values()) {
    const length = geometries.reduce((sum, geometry) => sum + geometry.attributes.position.array.length, 0);
    const positions = new Float32Array(length), normals = new Float32Array(length);
    let offset = 0;
    for (const geometry of geometries) {
      positions.set(geometry.attributes.position.array, offset); normals.set(geometry.attributes.normal.array, offset);
      offset += geometry.attributes.position.array.length; geometry.dispose();
    }
    const combined = new THREE.BufferGeometry();
    combined.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    combined.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    group.add(new THREE.Mesh(combined, material));
  }
}

function makeHand(parent, origin, side, material) {
  const hand = new THREE.Group(); hand.position.copy(origin); hand.rotation.z = side * -0.3;
  ellipsoid(hand, V(0, 0, 0), V(0.14, 0.2, 0.075), material);
  for (let finger = 0; finger < 4; finger += 1) {
    const x = (finger - 1.5) * 0.065, length = [0.17, 0.215, 0.2, 0.155][finger];
    limb(hand, V(x, 0.12, 0), V(x, 0.12 + length * 0.7, 0.055), 0.028, 0.023, material);
    limb(hand, V(x, 0.12 + length * 0.7, 0.055), V(x, 0.13 + length * 0.78, 0.15), 0.023, 0.019, material);
  }
  limb(hand, V(side * 0.11, -0.04, 0.015), V(side * 0.22, 0.08, 0.13), 0.045, 0.029, material);
  parent.add(hand);
}

function makeHead(material) {
  const group = new THREE.Group();
  const skull = loft([
    [0.10, 0.10, 0.12, 0.02], [0.16, 0.23, 0.24, 0.06], [0.25, 0.31, 0.30, 0.05],
    [0.38, 0.36, 0.34, 0.02], [0.53, 0.38, 0.37, 0], [0.68, 0.36, 0.36, -0.02],
    [0.80, 0.31, 0.31, -0.03], [0.89, 0.23, 0.23, -0.03], [0.94, 0.10, 0.10, -0.03], [0.95, 0, 0, -0.03],
  ], material);
  group.add(skull);
  // Restrained facial landmarks keep the mannequin readable in profile.
  ellipsoid(group, V(0, 0.43, 0.356), V(0.073, 0.11, 0.13), material);
  ellipsoid(group, V(0, 0.20, 0.26), V(0.17, 0.09, 0.08), material);
  for (const side of [-1, 1]) ellipsoid(group, V(side * 0.367, 0.43, -0.005), V(0.055, 0.108, 0.075), material);
  const detail = new THREE.MeshStandardMaterial({ color: 0x8c9eac, roughness: 0.8 });
  for (const side of [-1, 1]) {
    ellipsoid(group, V(side * 0.145, 0.55, 0.334), V(0.081, 0.018, 0.012), detail);
    curveLine(group, [V(side * 0.07, 0.595, 0.331), V(side * 0.14, 0.61, 0.333), V(side * 0.22, 0.594, 0.309)], 0x91a3b1, 0.8, 0.009);
  }
  curveLine(group, [V(-0.10, 0.28, 0.317), V(0, 0.272, 0.33), V(0.10, 0.28, 0.317)], 0x91a3b1, 0.8, 0.006);
  return group;
}

export function createDriver() {
  const body = new THREE.Group(); body.name = "driver-body";
  const shell = new THREE.MeshPhysicalMaterial({ color: 0xb4c8d6, roughness: 0.38, metalness: 0.08, clearcoat: 0.28, transparent: true, opacity: 0.28, depthWrite: false });
  const limbs = new THREE.MeshPhysicalMaterial({ color: 0xb8cad8, roughness: 0.48, clearcoat: 0.22, transparent: true, opacity: 0.68, depthWrite: false });
  const headMaterial = new THREE.MeshStandardMaterial({ color: 0xc7d3dd, roughness: 0.56, metalness: 0.04 });
  const torso = loft([
    [1.62, 0.36, 0.29], [1.80, 0.57, 0.35], [2.02, 0.58, 0.34], [2.24, 0.49, 0.30],
    [2.51, 0.48, 0.29, -0.02], [2.81, 0.57, 0.34, -0.03], [3.12, 0.69, 0.39, -0.02],
    [3.44, 0.75, 0.38, -0.01], [3.66, 0.78, 0.33], [3.82, 0.70, 0.27],
    [3.94, 0.48, 0.21], [4.02, 0.22, 0.16],
  ], shell);
  body.add(torso);
  ellipsoid(body, V(0, 1.83, 0.03), V(0.64, 0.42, 0.39), shell);
  for (const side of [-1, 1]) {
    const shoulder = V(side * 0.76, 3.70, 0), elbow = V(side * 1.01, 2.96, 0.87), wrist = V(side * 0.61, 3.18, 2.13);
    limb(body, shoulder, elbow, 0.225, 0.165, limbs);
    limb(body, elbow, wrist, 0.172, 0.10, limbs);
    makeHand(body, V(side * 0.61, 3.19, 2.20), side, limbs);
    const hip = V(side * 0.38, 1.70, 0.12), knee = V(side * 0.48, 1.57, 1.75), ankle = V(side * 0.48, 0.36, 1.97);
    limb(body, hip, knee, 0.29, 0.23, limbs);
    limb(body, knee, ankle, 0.225, 0.115, limbs);
    ellipsoid(body, V(side * 0.48, 0.17, 2.16), V(0.17, 0.14, 0.36), limbs);
    curveLine(body, [V(side * 0.18, 3.92, 0.16), V(side * 0.40, 3.85, 0.23), V(side * 0.68, 3.74, 0.20)], 0x6d94af, 0.38, 0.01);
  }
  const head = makeHead(headMaterial); head.name = "driver-head";
  mergeStaticMeshes(body); mergeStaticMeshes(head);
  return {
    body, head,
    setGhost(ghost) {
      shell.opacity = ghost ? 0.28 : 0.92;
      limbs.opacity = ghost ? 0.68 : 0.96;
    },
  };
}

export function createCockpit() {
  const group = new THREE.Group(); group.name = "cockpit";
  const seat = new THREE.Group(); seat.name = "seat-context"; group.add(seat);
  const fabric = new THREE.MeshStandardMaterial({ color: 0x678397, roughness: 0.9, transparent: true, opacity: 0.065, depthWrite: false });
  const border = new THREE.LineBasicMaterial({ color: 0x8199aa, transparent: true, opacity: 0.20, depthWrite: false });
  const seatPiece = (size, position, rotationX = 0) => {
    const panel = roundedPanel(size, 0.12, fabric);
    panel.position.copy(position); panel.rotation.x = rotationX; seat.add(panel);
    // Sparse silhouette; no wire grid over the subject or the cervical data.
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(panel.geometry, 42), border);
    edges.position.copy(panel.position); edges.rotation.copy(panel.rotation); seat.add(edges);
    return panel;
  };
  seatPiece([2.03, 2.82, 0.24], V(0, 2.93, -0.56), -0.07);
  seatPiece([1.93, 0.28, 2.15], V(0, 1.38, 0.44));
  seatPiece([1.04, 0.71, 0.23], V(0, 4.77, -0.47));
  for (const side of [-1, 1]) {
    seatPiece([0.24, 2.25, 0.27], V(side * 0.95, 2.87, -0.33), -0.07);
    const rail = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.06, 1.6), fabric); rail.position.set(side * 0.70, 0.22, 0.40); seat.add(rail);
  }
  const controls = new THREE.MeshStandardMaterial({ color: 0x8b9eac, roughness: 0.42, metalness: 0.25, transparent: true, opacity: 0.68, depthWrite: false });
  const wheel = new THREE.Group(); wheel.position.set(0, 3.24, 2.36); wheel.rotation.x = -0.22;
  wheel.add(new THREE.Mesh(new THREE.TorusGeometry(0.68, 0.055, 14, 72), controls));
  const hub = roundedPanel([0.38, 0.24, 0.12], 0.055, controls); wheel.add(hub);
  for (const angle of [Math.PI / 6, Math.PI * 5 / 6, -Math.PI / 2]) {
    limb(wheel, V(Math.cos(angle) * 0.16, Math.sin(angle) * 0.16, 0), V(Math.cos(angle) * 0.62, Math.sin(angle) * 0.62, 0), 0.018, 0.023, controls);
  }
  group.add(wheel);
  for (const side of [-1, 1]) {
    const pedal = roundedPanel([0.23, 0.32, 0.06], 0.04, controls); pedal.position.set(side * 0.48, 0.13, 2.48); pedal.rotation.x = -1.08; group.add(pedal);
  }
  return {
    group,
    setGhost(ghost) { fabric.opacity = ghost ? 0.065 : 0.76; border.opacity = ghost ? 0.20 : 0.40; },
  };
}

export function createVertebra(radius, material) {
  const vertebra = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius * 1.08, radius * 0.48, 28), material);
  body.scale.z = 0.76; body.position.z = 0.045; vertebra.add(body);
  const arch = new THREE.Mesh(new THREE.TorusGeometry(radius * 0.69, radius * 0.16, 8, 22, Math.PI * 1.6), material);
  arch.rotation.x = Math.PI / 2; arch.rotation.z = Math.PI * 0.2; arch.position.z = -radius * 0.44; vertebra.add(arch);
  ellipsoid(vertebra, V(0, -0.012, -radius * 1.08), V(radius * 0.17, radius * 0.20, radius * 0.62), material, 16);
  for (const side of [-1, 1]) ellipsoid(vertebra, V(side * radius * 1.05, 0, -radius * 0.2), V(radius * 0.5, radius * 0.15, radius * 0.22), material, 16);
  return vertebra;
}

export function addGround(scene) {
  const canvas = document.createElement("canvas"); canvas.width = 128; canvas.height = 128;
  const ctx = canvas.getContext("2d"), gradient = ctx.createRadialGradient(64, 64, 0, 64, 64, 64);
  gradient.addColorStop(0, "rgba(67,100,124,.16)"); gradient.addColorStop(1, "rgba(67,100,124,0)");
  ctx.fillStyle = gradient; ctx.fillRect(0, 0, 128, 128);
  const shadow = new THREE.Mesh(new THREE.PlaneGeometry(5.5, 5.5), new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(canvas), transparent: true, depthWrite: false }));
  shadow.rotation.x = -Math.PI / 2; shadow.position.set(0, 0.01, 0.7); scene.add(shadow);
  const grid = new THREE.GridHelper(12, 24, 0xc4d2dd, 0xe1e8ee); grid.material.transparent = true; grid.material.opacity = 0.48; scene.add(grid);
}
