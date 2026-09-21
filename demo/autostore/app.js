// Shared playback app. Expects global PLAN (plan.js / plan_column.js).
try {
window.addEventListener('error', (e) => {
  const el = document.getElementById('stats');
  if (el) el.textContent = 'JS 错误: ' + (e.error && e.error.stack
    ? e.error.stack.split('\n').slice(0, 3).join(' | ')
    : e.message + ' @' + e.filename + ':' + e.lineno);
});
const ROWS = PLAN.rows, COLS = PLAN.cols, T = PLAN.T;
// embed mode (?embed=1): page-styled light theme, HUD hidden, controls
// driven by the parent page through postMessage
const EMBED = new URLSearchParams(location.search).has('embed');
const CELL = 1.0, BLOCK_H = 0.62, AGENT_H = 0.46;
const FLY = 0.55;

// ---------------- scene ----------------
const scene = new THREE.Scene();
const BG = EMBED ? 0xf3f4f6 : 0x0b0e14;
scene.background = new THREE.Color(BG);
scene.fog = new THREE.Fog(BG, 34, 90);

const camera = new THREE.PerspectiveCamera(46, innerWidth / innerHeight, .1, 200);
let renderer = null;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(innerWidth, innerHeight);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  document.body.appendChild(renderer.domElement);
} catch (e) {
  document.getElementById('stats').textContent = 'WebGL 不可用: ' + e.message;
}

scene.add(new THREE.HemisphereLight(0xbfd4ff, EMBED ? 0x8a8272 : 0x30281e, EMBED ? .75 : .55));
const sun = new THREE.DirectionalLight(0xfff2dd, 1.05);
sun.position.set(14, 22, 8);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
sun.shadow.camera.left = -16; sun.shadow.camera.right = 16;
sun.shadow.camera.top = 16; sun.shadow.camera.bottom = -16;
scene.add(sun);

function cellXZ(r, c) { return [c - COLS / 2 + .5, r - ROWS / 2 + .5]; }

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(COLS * CELL + 8, ROWS * CELL + 8),
  new THREE.MeshStandardMaterial({ color: EMBED ? 0xfaf8f2 : 0x161c2b, roughness: .95 }));
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.01;
ground.receiveShadow = true;
scene.add(ground);

function addTiles(cells, color) {
  const mat = new THREE.MeshStandardMaterial({ color, roughness: .85 });
  for (const [r, c] of cells) {
    const tile = new THREE.Mesh(new THREE.PlaneGeometry(CELL * .92, CELL * .92), mat);
    tile.rotation.x = -Math.PI / 2;
    const [x, z] = cellXZ(r, c);
    tile.position.set(x, 0.002, z);
    tile.receiveShadow = true;
    scene.add(tile);
  }
}
addTiles(PLAN.depot, EMBED ? 0x9dc4a8 : 0x1f4d33);

const gridHelper = new THREE.GridHelper(Math.max(ROWS, COLS) + 8, Math.max(ROWS, COLS) + 8,
  EMBED ? 0xe3e0d6 : 0x25304a, EMBED ? 0xece9e0 : 0x1b2436);
gridHelper.position.y = 0.001;
scene.add(gridHelper);

// gantry rail plane + corner posts (viewer-only)
if (PLAN.meta && PLAN.meta.gantry) {
  const railY = PLAN.meta.gantry * BLOCK_H;
  const rail = new THREE.Mesh(
    new THREE.PlaneGeometry(COLS * CELL, ROWS * CELL),
    new THREE.MeshStandardMaterial({ color: 0x8fa3c0, transparent: true,
                                     opacity: .10, side: THREE.DoubleSide }));
  rail.rotation.x = -Math.PI / 2;
  rail.position.y = railY;
  scene.add(rail);
  const postMat = new THREE.MeshStandardMaterial({ color: 0x46536b, roughness: .6, metalness: .3 });
  for (const [r, c] of [[0, 0], [0, COLS], [ROWS, 0], [ROWS, COLS]]) {
    const post = new THREE.Mesh(new THREE.CylinderGeometry(.06, .06, railY), postMat);
    post.position.set(c - COLS / 2, railY / 2, r - ROWS / 2);
    scene.add(post);
  }
}

// ---------------- terrain events ----------------
// classify events by replaying: height up = place, down = remove
const terrainEvents = PLAN.terrain.slice().sort((a, b) => a.t - b.t);
{
  const cur = Array.from({ length: ROWS }, () => new Array(COLS).fill(0));
  for (const ev of terrainEvents) {
    ev.isPlace = ev.h > cur[ev.r][ev.c];
    cur[ev.r][ev.c] = ev.h;
  }
}
const placeEvents = terrainEvents.filter(ev => ev.isPlace);
const removeEvents = terrainEvents.filter(ev => !ev.isPlace);
// pair each removal with the latest matching placement (same cell, block height)
for (const rem of removeEvents) {
  for (let j = placeEvents.length - 1; j >= 0; j--) {
    const pl = placeEvents[j];
    if (pl.r === rem.r && pl.c === rem.c && pl.h === rem.h + 1 &&
        pl.t <= rem.t && pl.removedAt === undefined) {
      pl.removedAt = rem.t;
      pl.removerAgent = rem.agent;
      break;
    }
  }
}

const blockGeo = new THREE.BoxGeometry(CELL * .98, BLOCK_H, CELL * .98);
const blockMats = [
  new THREE.MeshStandardMaterial({ color: 0xe8b04a, roughness: .8 }),
  new THREE.MeshStandardMaterial({ color: 0xdea23e, roughness: .8 }),
  new THREE.MeshStandardMaterial({ color: 0xd39634, roughness: .8 }),
  new THREE.MeshStandardMaterial({ color: 0xc98b2c, roughness: .8 }),
  new THREE.MeshStandardMaterial({ color: 0xbf7f24, roughness: .8 }),
  new THREE.MeshStandardMaterial({ color: 0xb5741d, roughness: .8 }),
];
// scaffold (temporary) blocks: cool steel gray, clearly distinct
const scaffoldMats = [
  new THREE.MeshStandardMaterial({ color: 0x9aa7bd, roughness: .6, metalness: .25 }),
  new THREE.MeshStandardMaterial({ color: 0x8b99b0, roughness: .6, metalness: .25 }),
  new THREE.MeshStandardMaterial({ color: 0x7d8ba3, roughness: .6, metalness: .25 }),
  new THREE.MeshStandardMaterial({ color: 0x707e96, roughness: .6, metalness: .25 }),
  new THREE.MeshStandardMaterial({ color: 0x64738b, roughness: .6, metalness: .25 }),
];
const edgeMat = new THREE.LineBasicMaterial({ color: 0x8a6420, transparent: true, opacity: .5 });
const scaffoldEdgeMat = new THREE.LineBasicMaterial({ color: 0x46536b, transparent: true, opacity: .6 });
const blockEdges = new THREE.EdgesGeometry(blockGeo);

// optional per-cell colors (PLAN.colors: {"r,c": "#rrggbb"}) override the
// default height palette for permanent (non-scaffold) blocks
const colorMats = {};
function cellMatHex(hex) {
  if (!colorMats[hex]) {
    colorMats[hex] = new THREE.MeshStandardMaterial({
      color: new THREE.Color(hex), roughness: .75 });
  }
  return colorMats[hex];
}
function cellMat(r, c, h) {
  const hex = (PLAN.colors3 && PLAN.colors3[r + ',' + c + ',' + h])
      || (PLAN.colors && PLAN.colors[r + ',' + c]);
  if (!hex) return null;
  if (!colorMats[hex]) {
    const isGlass = /^#(?:9cc9dc|a8d0e6)$/i.test(hex);
    colorMats[hex] = new THREE.MeshStandardMaterial({
      color: new THREE.Color(hex),
      roughness: isGlass ? .18 : .75,
      metalness: isGlass ? .05 : 0,
      transparent: isGlass,
      opacity: isGlass ? .34 : 1,
      side: isGlass ? THREE.DoubleSide : THREE.FrontSide,
    });
  }
  return colorMats[hex];
}

function finalHeightAt(r, c) {
  if (PLAN.final_heights) return PLAN.final_heights[r][c];
  return Infinity; // no final map (pyramid): everything is target structure
}

const LEGO = !!(PLAN.meta && PLAN.meta.lego);
// gantry mode: robots ride a rail plane at meta.gantry block-heights;
// carried bricks hang below the cart instead of sitting on the head
const GANTRY = (PLAN.meta && PLAN.meta.gantry) || 0;
const studGeo = LEGO ? new THREE.CylinderGeometry(CELL * 0.3, CELL * 0.3, BLOCK_H * 0.22, 16) : null;
// merge multi-cell LEGO bricks into ONE seamless box: group brick-marked
// events by (t, agent); the anchor event carries the merged mesh (centered
// on the brick), sibling events render nothing
if (LEGO) {
  const groups = {};
  for (const ev of placeEvents) {
    if (ev.bw === undefined) continue;
    const key = ev.t + ':' + ev.agent;
    (groups[key] = groups[key] || []).push(ev);
  }
  for (const key in groups) {
    const grp = groups[key];
    let r0 = 1e9, r1 = -1e9, c0 = 1e9, c1 = -1e9;
    for (const e of grp) {
      r0 = Math.min(r0, e.r); r1 = Math.max(r1, e.r);
      c0 = Math.min(c0, e.c); c1 = Math.max(c1, e.c);
    }
    grp[0]._anchor = true;
    grp[0]._cr = (r0 + r1) / 2;
    grp[0]._cc = (c0 + c1) / 2;
    grp[0]._spanR = r1 - r0 + 1;
    grp[0]._spanC = c1 - c0 + 1;
    for (let i = 1; i < grp.length; i++) grp[i]._hidden = true;
  }
}
const blockMeshes = placeEvents.map(ev => {
  const g = new THREE.Group();
  if (ev._hidden) { g.visible = false; scene.add(g); return g; }
  const scaffold = ev.h > finalHeightAt(ev.r, ev.c);
  const mats = scaffold ? scaffoldMats : blockMats;
  const custom = scaffold ? null
      : (ev.color ? cellMatHex(ev.color) : cellMat(ev.r, ev.c, ev.h));
  const material = custom || mats[Math.min(ev.h - 1, mats.length - 1)];
  let mesh;
  if (ev._anchor) {
    const geo = new THREE.BoxGeometry(CELL * ev._spanC - 0.02, BLOCK_H, CELL * ev._spanR - 0.02);
    mesh = new THREE.Mesh(geo, material);
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat));
    if (LEGO && !scaffold) {
      for (let dr = 0; dr < ev._spanR; dr++) {
        for (let dc = 0; dc < ev._spanC; dc++) {
          const stud = new THREE.Mesh(studGeo, material);
          stud.position.set((dc - (ev._spanC - 1) / 2) * CELL,
                            BLOCK_H / 2 + BLOCK_H * 0.11,
                            (dr - (ev._spanR - 1) / 2) * CELL);
          stud.castShadow = true;
          g.add(stud);
        }
      }
    }
  } else {
    mesh = new THREE.Mesh(blockGeo, material);
    if (LEGO && !scaffold) {
      const stud = new THREE.Mesh(studGeo, material);
      stud.position.y = BLOCK_H / 2 + BLOCK_H * 0.11;
      stud.castShadow = true;
      g.add(stud);
    }
  }
  mesh.castShadow = true; mesh.receiveShadow = true;
  g.add(mesh);
  if (!ev._anchor) {
    g.add(new THREE.LineSegments(blockEdges, scaffold ? scaffoldEdgeMat : edgeMat));
  }
  g.visible = false;
  scene.add(g);
  return g;
});

// Parts such as vertical wheels and long shafts cannot be encoded by the
// terrain height field. Each is tied to a real brick placement and appears
// only after that validated anchor task completes.
const anchoredParts = [];
function partMaterial(color, emissive = 0x000000) {
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(color || '#222222'),
    roughness: .65,
    metalness: .12,
    emissive,
  });
}
function addBox(group, sx, sy, sz, x, y, z, mat) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat);
  mesh.position.set(x, y, z);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  group.add(mesh);
  return mesh;
}
function worldPoint(p) {
  const [x, z] = cellXZ(p.r, p.c);
  return new THREE.Vector3(x, p.z * BLOCK_H, z);
}
function addCylinderBetween(group, a, b, radius, mat) {
  const v = new THREE.Vector3(b.x - a.x, b.y - a.y, b.z - a.z);
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, v.length(), 14), mat);
  mesh.position.set((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
  mesh.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0), v.clone().normalize());
  mesh.castShadow = true;
  group.add(mesh);
  return mesh;
}
function makeWheel(spec) {
  const g = new THREE.Group();
  const mat = partMaterial(spec.color || '#75411f');
  const radius = spec.radius || 1.65;
  const thick = spec.width || .22;
  const rim = new THREE.Mesh(
    new THREE.TorusGeometry(radius, Math.max(.08, radius * .09), 10, 32), mat);
  rim.castShadow = true;
  g.add(rim);
  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(radius * .16, radius * .16, thick * 1.8, 16), mat);
  hub.rotation.x = Math.PI / 2;
  hub.castShadow = true;
  g.add(hub);
  for (let i = 0; i < 8; i++) {
    const spoke = addBox(g, radius * 1.7, radius * .075, thick,
                         0, 0, 0, mat);
    spoke.rotation.z = i * Math.PI / 4;
  }
  g.position.copy(worldPoint(spec.at));
  return g;
}
function makeShaft(spec) {
  const g = new THREE.Group();
  addCylinderBetween(g, worldPoint(spec.from), worldPoint(spec.to),
                     spec.radius || .07,
                     partMaterial(spec.color || '#171b24'));
  return g;
}
function makeLamp(spec) {
  const g = new THREE.Group();
  const dark = partMaterial(spec.color || '#11151b');
  const glow = partMaterial(spec.glass || '#ffd84d', 0x6b4b00);
  const body = new THREE.Mesh(new THREE.CylinderGeometry(.22, .22, .52, 12), dark);
  body.castShadow = true;
  g.add(body);
  const glass = new THREE.Mesh(new THREE.CylinderGeometry(.14, .14, .28, 12), glow);
  glass.position.y = -.02;
  g.add(glass);
  const hook = addBox(g, .09, .35, .09, 0, .39, 0, dark);
  hook.rotation.z = .25;
  g.position.copy(worldPoint(spec.at));
  return g;
}
function makeHorse(spec) {
  const g = new THREE.Group();
  const white = partMaterial(spec.color || '#f1f1eb');
  const harness = partMaterial(spec.harness || '#6b3824');
  const mane = partMaterial(spec.mane || '#b7b2a8');
  addBox(g, 3.15, 1.38, 1.18, 0, 1.75, 0, white);
  for (const x of [-1.05, .92]) for (const z of [-.39, .39]) {
    const leg = addBox(g, .38, 1.85, .38, x, .45, z, white);
    leg.rotation.z = x < 0 ? -.07 : .07;
  }
  const neck = addBox(g, .72, 1.65, .82, -1.35, 2.55, 0, white);
  neck.rotation.z = -.34;
  const head = addBox(g, 1.02, .72, .78, -1.92, 3.22, 0, white);
  head.rotation.z = -.18;
  addBox(g, .48, .36, .7, -2.5, 3.02, 0, white);
  addBox(g, .13, .38, .14, -1.98, 3.78, -.22, white).rotation.z = -.2;
  addBox(g, .13, .38, .14, -1.98, 3.78, .22, white).rotation.z = -.2;
  for (let i = 0; i < 6; i++) {
    addBox(g, .13, .34, .12, -1.05 - i * .17, 3.15 - i * .15, 0, mane);
  }
  addCylinderBetween(g,
    new THREE.Vector3(1.5, 2.05, 0),
    new THREE.Vector3(2.35, 2.75, 0), .08, mane);
  addBox(g, 1.55, .08, .88, -1.62, 3.18, 0, harness);
  addBox(g, .09, 1.05, .9, -1.95, 3.05, 0, harness);
  addBox(g, 2.8, .08, 1.26, .1, 1.78, 0, harness);
  g.position.copy(worldPoint(spec.at));
  g.rotation.y = spec.yaw || 0;
  g.scale.setScalar(spec.scale || 1);
  return g;
}
function makeWindow(spec) {
  const g = new THREE.Group();
  const frame = partMaterial(spec.color || '#11151b');
  const glass = new THREE.MeshStandardMaterial({
    color: new THREE.Color(spec.glass || '#b9d9e8'),
    roughness: .12,
    transparent: true,
    opacity: .28,
    side: THREE.DoubleSide,
  });
  const w = spec.width || 3.4;
  const h = spec.height || 2.45;
  const t = spec.thickness || .13;
  addBox(g, w, .18, t, 0, h / 2, 0, frame);
  addBox(g, w, .18, t, 0, -h / 2, 0, frame);
  addBox(g, .18, h, t, -w / 2, 0, 0, frame);
  addBox(g, .18, h, t, w / 2, 0, 0, frame);
  addBox(g, w - .3, h - .3, t * .35, 0, 0, 0, glass);
  g.position.copy(worldPoint(spec.at));
  if (spec.plane === 'end') g.rotation.y = Math.PI / 2;
  return g;
}
function makeDoor(spec) {
  const g = new THREE.Group();
  const panel = partMaterial(spec.color || '#11151b');
  const trim = partMaterial(spec.trim || '#303742');
  const w = spec.width || 2.2;
  const h = spec.height || 3.25;
  addBox(g, w, h, .14, 0, 0, 0, panel);
  addBox(g, w - .22, .09, .17, 0, h * .34, 0, trim);
  addBox(g, w - .22, .09, .17, 0, -h * .34, 0, trim);
  const handle = new THREE.Mesh(
    new THREE.CylinderGeometry(.065, .065, .28, 10), trim);
  handle.rotation.x = Math.PI / 2;
  handle.position.set(w * .3, 0, .14);
  g.add(handle);
  g.position.copy(worldPoint(spec.at));
  if (spec.plane === 'end') g.rotation.y = Math.PI / 2;
  return g;
}
function makeSeat(spec) {
  const g = new THREE.Group();
  const mat = partMaterial(spec.color || '#161b24');
  const cushion = addBox(g, 1.45, .34, 1.65, 0, .36, 0, mat);
  cushion.rotation.z = -.04;
  const back = addBox(g, .34, 1.25, 1.65, -.55, 1.0, 0, mat);
  back.rotation.z = -.3;
  g.position.copy(worldPoint(spec.at));
  g.rotation.y = spec.yaw || 0;
  return g;
}
function makeCanopy(spec) {
  const g = new THREE.Group();
  const mat = partMaterial(spec.color || '#11151b');
  const radius = spec.radius || 1.05;
  const width = spec.width || 5.7;
  const curved = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, width, 24, 1, false, 0, Math.PI),
    mat);
  curved.rotation.x = Math.PI / 2;
  curved.castShadow = true;
  g.add(curved);
  g.position.copy(worldPoint(spec.at));
  g.rotation.y = spec.yaw || 0;
  return g;
}
function makeWheelArch(spec) {
  const g = new THREE.Group();
  const mat = partMaterial(spec.color || '#11151b');
  const arch = new THREE.Mesh(
    new THREE.TorusGeometry(spec.radius || 1.82, spec.tube || .14, 9, 28, Math.PI),
    mat);
  arch.castShadow = true;
  g.add(arch);
  g.position.copy(worldPoint(spec.at));
  return g;
}
function makeFootboard(spec) {
  const g = new THREE.Group();
  const mat = partMaterial(spec.color || '#11151b');
  addBox(g, spec.length || 4.5, .18, spec.width || .62, 0, 0, 0, mat);
  g.position.copy(worldPoint(spec.at));
  if (spec.axis === 'row') g.rotation.y = Math.PI / 2;
  return g;
}
for (const spec of ((PLAN.meta && PLAN.meta.attachments) || [])) {
  let group = null;
  if (spec.kind === 'wheel') group = makeWheel(spec);
  else if (spec.kind === 'shaft') group = makeShaft(spec);
  else if (spec.kind === 'lamp') group = makeLamp(spec);
  else if (spec.kind === 'horse') group = makeHorse(spec);
  else if (spec.kind === 'window') group = makeWindow(spec);
  else if (spec.kind === 'door') group = makeDoor(spec);
  else if (spec.kind === 'seat') group = makeSeat(spec);
  else if (spec.kind === 'canopy') group = makeCanopy(spec);
  else if (spec.kind === 'wheelArch') group = makeWheelArch(spec);
  else if (spec.kind === 'footboard') group = makeFootboard(spec);
  if (!group) continue;
  const a = spec.anchor;
  const ev = placeEvents.find(e => e.r === a.r && e.c === a.c && e.h === a.h);
  group.visible = false;
  scene.add(group);
  anchoredParts.push({ group, t: ev ? ev.t : T });
}

// viewer-only suspender drop lines (PLAN.meta.suspenders): thin vertical
// rods from the deck level up to the cable block, appearing when the cable
// block above them is placed
const suspenderRods = [];
if (PLAN.meta && PLAN.meta.suspenders) {
  const rodMat = new THREE.MeshStandardMaterial({ color: 0xb03018, roughness: .6 });
  for (const s of PLAN.meta.suspenders) {
    const ev = placeEvents.find(e => e.r === s.r && e.c === s.c && e.h === s.top);
    const len = (s.top - 1 - s.bot) * BLOCK_H;
    if (!ev || len <= 0) continue;
    const rod = new THREE.Mesh(new THREE.BoxGeometry(0.1, len, 0.1), rodMat);
    const [x, z] = cellXZ(s.r, s.c);
    rod.position.set(x, s.bot * BLOCK_H + len / 2, z);
    rod.visible = false;
    scene.add(rod);
    suspenderRods.push({ rod, t: ev.t });
  }
}

// viewer-only horizontal tower struts (PLAN.meta.struts): thin bars between
// the two legs of a portal tower, appearing once both legs reach their z
const strutBars = [];
if (PLAN.meta && PLAN.meta.struts) {
  const strutMat = new THREE.MeshStandardMaterial({ color: 0xe0482a, roughness: .7 });
  for (const s of PLAN.meta.struts) {
    const evA = placeEvents.find(e => e.r === s.r0 && e.c === s.c && e.h === s.z);
    const evB = placeEvents.find(e => e.r === s.r1 && e.c === s.c && e.h === s.z);
    if (!evA || !evB) continue;
    const len = (s.r1 - s.r0) * CELL;
    const thick = s.thick ? BLOCK_H * 1.0 : BLOCK_H * .5;
    const bar = new THREE.Mesh(new THREE.BoxGeometry(s.thick ? 0.9 : 0.24, thick, len + (s.thick ? 0.9 : 0)), strutMat);
    const [xa, za] = cellXZ(s.r0, s.c);
    const [, zb] = cellXZ(s.r1, s.c);
    bar.position.set(xa, (s.z - 0.5) * BLOCK_H, (za + zb) / 2);
    bar.visible = false;
    scene.add(bar);
    strutBars.push({ bar, t: Math.max(evA.t, evB.t) });
  }
}

// height field over time (sequential replay), memoized per integer step
let hfCacheT = -1, hfCache = null;
function heightsAt(t) {
  if (t === hfCacheT) return hfCache;
  const h = Array.from({ length: ROWS }, () => new Array(COLS).fill(0));
  for (const ev of terrainEvents) {
    if (ev.t <= t) h[ev.r][ev.c] = ev.h;
  }
  hfCacheT = t;
  hfCache = h;
  return h;
}

// ---------------- agents ----------------
const agentColors = [0x4a90d9, 0x50c8b4, 0xd96a9c, 0x9a7ce8, 0x6ab04c, 0xe07b4f];
const agents = PLAN.agents.map((a, i) => {
  const g = new THREE.Group();
  g.rotation.order = 'YXZ'; // yaw first, then pitch (lean follows facing)
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(CELL * .62, AGENT_H, CELL * .62),
    new THREE.MeshStandardMaterial({ color: agentColors[i % agentColors.length], roughness: .5, metalness: .15 }));
  body.castShadow = true;
  body.position.y = AGENT_H / 2;
  g.add(body);
  const eye = new THREE.Mesh(
    new THREE.BoxGeometry(CELL * .5, AGENT_H * .22, CELL * .12),
    new THREE.MeshStandardMaterial({ color: 0x0b0e14, roughness: .3 }));
  eye.position.set(0, AGENT_H * .68, CELL * .26);
  g.add(eye);
  const carriedDefaultMat = new THREE.MeshStandardMaterial({ color: 0xffcf6e, roughness: .7, emissive: 0x332200 });
  const carried = new THREE.Mesh(
    GANTRY ? new THREE.BoxGeometry(CELL * .92, BLOCK_H, CELL * .92)
           : new THREE.BoxGeometry(CELL * .5, BLOCK_H * .55, CELL * .5),
    carriedDefaultMat);
  carried.castShadow = true;
  carried.position.y = GANTRY ? -BLOCK_H * .62 : AGENT_H + BLOCK_H * .32;
  g.add(carried);
  let rope = null, hook = null;
  if (GANTRY) {
    rope = new THREE.Mesh(
      new THREE.CylinderGeometry(.03, .03, 1, 6),
      new THREE.MeshStandardMaterial({ color: 0x333a45, roughness: .4 }));
    rope.visible = false;
    g.add(rope);
    hook = new THREE.Mesh(
      new THREE.BoxGeometry(CELL * .22, BLOCK_H * .18, CELL * .22),
      new THREE.MeshStandardMaterial({ color: 0x222831, metalness: .5, roughness: .3 }));
    hook.visible = false;
    g.add(hook);
  }
  scene.add(g);
  return { group: g, carried, carriedDefaultMat, rope, hook, path: a.path, carry: a.carry, idx: i };
});
// terrain events indexed by acting agent, for facing/lean animation
const actByAgent = agents.map(() => []);
for (const ev of terrainEvents) {
  if (ev.agent === undefined) continue;
  if (!actByAgent[ev.agent]) throw new Error(`bad terrain agent ${ev.agent}`);
  actByAgent[ev.agent].push(ev);
}
// gantry v2: persistent per-brick meshes from exported timelines
const BRICKS = (GANTRY && PLAN.bricks) ? PLAN.bricks.map(b => {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(CELL * .92, BLOCK_H * .96, CELL * .92),
    cellMatHex(b.color));
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);
  return { mesh, segs: b.segs };
}) : null;

function brickPose(segs, tf) {
  let seg = segs[0];
  for (const s of segs) {
    const end = s[2] === null ? Infinity : s[2];
    if (tf >= s[1] && tf < end) { seg = s; break; }
    if (tf >= s[1]) seg = s;
  }
  const railY = GANTRY * BLOCK_H;
  const HANGY = railY - BLOCK_H * .62;
  if (seg[0] === 'rest') {
    const [x, z] = cellXZ(seg[3], seg[4]);
    return [x, (seg[5] + .5) * BLOCK_H, z];
  }
  if (seg[0] === 'ride') {
    const a = PLAN.agents[seg[3]];
    const p0 = a.path[Math.min(Math.floor(tf), a.path.length - 1)];
    const p1 = a.path[Math.min(Math.floor(tf) + 1, a.path.length - 1)];
    const u = tf - Math.floor(tf);
    const [x0, z0] = cellXZ(p0[0], p0[1]);
    const [x1, z1] = cellXZ(p1[0], p1[1]);
    return [x0 + (x1 - x0) * u, HANGY, z0 + (z1 - z0) * u];
  }
  // hoist: brick-center path between hang height and its rest height
  const [x, z] = cellXZ(seg[3], seg[4]);
  const restY = (seg[5] + .5) * BLOCK_H;
  const u = Math.max(0, Math.min(1,
      (tf - seg[1]) / Math.max(1, seg[2] - seg[1])));
  let y;
  if (seg[6] === 'lift')       // stay, then rise to the cart
    y = u < .5 ? restY : restY + (HANGY - restY) * (u * 2 - 1);
  else                         // descend from the cart, then stay
    y = u < .5 ? HANGY + (restY - HANGY) * (u * 2) : restY;
  return [x, y, z];
}

// hoist service windows per agent (gantry: rope down / grab / rope up)
const hoistsByAgent = agents.map(() => []);
for (const h of (PLAN.hoists || [])) hoistsByAgent[h.agent].push(h);

// deposit times per agent (scrap hand-off at the depot)
const depositsByAgent = agents.map(() => []);
for (const d of (PLAN.deposits || [])) depositsByAgent[d.agent].push(d.t);

// ---------------- playback state ----------------
const initialT = new URLSearchParams(location.search).get('t');
// let the clock run one step past the last event so the final block's
// flight animation (FLY) can land instead of freezing mid-air
const T_END = T + 1;
let simT = initialT === 'end' ? T_END : Math.max(0, Math.min(T_END, Number(initialT) || 0));
let playing = initialT !== 'end';
let speed = 6;
let lastStatsT = -1;
const timeline = document.getElementById('timeline');
timeline.max = T_END;
const statsEl = document.getElementById('stats');

function lerp(a, b, u) { return a + (b - a) * u; }

function updateWorld(tf) {
  const t0 = Math.floor(tf), u = tf - t0;
  const h0 = heightsAt(t0);

  if (BRICKS) {
    for (const b of BRICKS) {
      const [x, y, z] = brickPose(b.segs, tf);
      b.mesh.position.set(x, y, z);
    }
    for (const g of blockMeshes) g.visible = false;
  }
  // blocks: fly from placer on placement; fly to remover on removal
  for (let i = 0; i < placeEvents.length && !BRICKS; i++) {
    const ev = placeEvents[i], g = blockMeshes[i];
    const gone = ev.removedAt !== undefined && tf >= ev.removedAt + FLY;
    if (tf < ev.t || gone) { g.visible = false; continue; }
    g.visible = true;
    const [tx, tz] = cellXZ(ev._cr !== undefined ? ev._cr : ev.r, ev._cc !== undefined ? ev._cc : ev.c);
    const ty = (ev.h - 1) * BLOCK_H + BLOCK_H / 2;
    g.rotation.y = 0;
    let liftWin = null;
    if (GANTRY && ev.removedAt !== undefined && PLAN.hoists)
      for (const h of PLAN.hoists)
        if (h.kind === 'lift' && h.t1 === ev.removedAt &&
            h.r === ev.r && h.c === ev.c) { liftWin = h; break; }
    if (liftWin) {
      if (tf >= liftWin.t1) { g.visible = false; continue; }
      if (tf >= liftWin.t0) {
        // second half of the hoist: the block rides the hook up along the
        // same brick-center path the hook animation uses
        const u = (tf - liftWin.t0) /
            Math.max(1, liftWin.t1 - liftWin.t0);
        if (u >= .5) {
          const railY = GANTRY * BLOCK_H;
          const restW = railY - BLOCK_H * .62;     // world hang height
          const k = 2 - u * 2;                     // 1 -> 0 (rest -> cart)
          const y = ty * k + restW * (1 - k);
          g.position.set(tx, y, tz);
          g.scale.setScalar(1);
          continue;
        }
      }
      g.position.set(tx, ty, tz);
      g.scale.setScalar(1);
      continue;
    }
    if (ev.removedAt !== undefined && tf >= ev.removedAt) {
      // removal flight: from resting place to the remover's head
      const ra = PLAN.agents[ev.removerAgent];
      const rp = ra.path[Math.min(ev.removedAt, ra.path.length - 1)];
      const [fx, fz] = cellXZ(rp[0], rp[1]);
      const fy = GANTRY ? GANTRY * BLOCK_H - BLOCK_H * .45
                        : (ev.h - 1) * BLOCK_H + AGENT_H + BLOCK_H * 0.32;
      const k = (tf - ev.removedAt) / FLY;
      const s = k * k * (3 - 2 * k);
      g.position.set(lerp(tx, fx, s), lerp(ty, fy, s) + Math.sin(Math.PI * s) * 0.24, lerp(tz, fz, s));
      g.scale.setScalar(GANTRY ? 1 : 1 - 0.45 * s);
      continue;
    }
    const age = tf - ev.t;
    let hoisted = false;
    if (GANTRY && ev.agent !== undefined && PLAN.hoists)
      for (const h of PLAN.hoists)
        if (h.agent === ev.agent && h.t1 === ev.t &&
            h.r === ev.r && h.c === ev.c) { hoisted = true; break; }
    const from = (ev.agent !== undefined && !hoisted) ? PLAN.agents[ev.agent].path[Math.min(ev.t, PLAN.agents[ev.agent].path.length - 1)] : null;
    if (age < FLY && from) {
      const [fx, fz] = cellXZ(from[0], from[1]);
      const fy = GANTRY ? GANTRY * BLOCK_H - BLOCK_H * .45
                        : h0[from[0]][from[1]] * BLOCK_H + AGENT_H + BLOCK_H * 0.32;
      const k = age / FLY;
      const s = k * k * (3 - 2 * k);
      g.position.set(lerp(fx, tx, s), lerp(fy, ty, s) + Math.sin(Math.PI * s) * 0.28, lerp(fz, tz, s));
      g.scale.setScalar(GANTRY ? 1 : 0.55 + 0.45 * s);
      g.rotation.y = GANTRY ? 0 : (1 - s) * 0.8;
    } else {
      g.position.set(tx, ty, tz);
      g.scale.setScalar(1);
    }
  }

  for (const p of anchoredParts) p.group.visible = tf >= p.t + FLY;

  // agents
  for (const s of suspenderRods) s.rod.visible = tf >= s.t + FLY;
  for (const s of strutBars) s.bar.visible = tf >= s.t + FLY;
  let carryingCount = 0;
  for (const a of agents) {
    const p0 = a.path[Math.min(t0, a.path.length - 1)];
    const p1 = a.path[Math.min(t0 + 1, a.path.length - 1)];
    const [x0, z0] = cellXZ(p0[0], p0[1]);
    const [x1, z1] = cellXZ(p1[0], p1[1]);
    const y0 = GANTRY ? GANTRY * BLOCK_H : h0[p0[0]][p0[1]] * BLOCK_H;
    const y1 = GANTRY ? GANTRY * BLOCK_H : h0[p1[0]][p1[1]] * BLOCK_H;
    let px, py, pz, squash = 1;
    if (Math.abs(y1 - y0) > 1e-6) {
      if (u < 0.25) {
        const k = u / 0.25;
        px = x0; pz = z0; py = y0;
        squash = 1 - 0.22 * k;
      } else if (u < 0.85) {
        const k = (u - 0.25) / 0.6;
        px = lerp(x0, x1, k); pz = lerp(z0, z1, k);
        py = lerp(y0, y1, k) + Math.sin(Math.PI * k) * BLOCK_H * 0.55;
        squash = 1 + 0.12 * Math.sin(Math.PI * k);
      } else {
        const k = (u - 0.85) / 0.15;
        px = x1; pz = z1; py = y1;
        squash = 1 - 0.18 * Math.sin(Math.PI * k);
      }
    } else {
      px = lerp(x0, x1, u); py = lerp(y0, y1, u); pz = lerp(z0, z1, u);
      if (x1 !== x0 || z1 !== z0) py += Math.abs(Math.sin(Math.PI * u)) * 0.035;
    }
    a.group.position.set(px, py, pz);
    a.group.scale.set(1 / Math.sqrt(squash), squash, 1 / Math.sqrt(squash));
    if (x1 !== x0 || z1 !== z0) a.group.rotation.y = Math.atan2(x1 - x0, z1 - z0);
    // face + lean toward the acted cell around pick-from-terrain/placement
    a.group.rotation.x = 0;
    let hideCarried = false;
    for (const ev of actByAgent[a.idx]) {
      const rel = tf - (ev.t - 0.35);
      if (rel >= 0 && rel <= 0.35 + FLY) {
        const [tx, tz] = cellXZ(ev._cr !== undefined ? ev._cr : ev.r, ev._cc !== undefined ? ev._cc : ev.c);
        a.group.rotation.y = Math.atan2(tx - px, tz - pz);
        a.group.rotation.x = 0.28 * Math.sin(Math.PI * rel / (0.35 + FLY));
        if (!GANTRY && !ev.isPlace && tf >= ev.t && tf < ev.t + FLY) hideCarried = true;
        break;
      }
    }
    const c = a.carry[Math.min(t0, a.carry.length - 1)] === 1;
    a.carried.visible = c && !hideCarried && !BRICKS;
    // ---- gantry hoist animation: rope + hook travel, load descent ----
    if (GANTRY && a.rope) {
      let win = null, lastLift = null;
      for (const h of hoistsByAgent[a.idx]) {
        if (tf >= h.t0 && tf < h.t1) win = h;
        if (h.kind === 'lift' && h.t1 <= tf &&
            (lastLift === null || h.t1 > lastLift.t1)) lastLift = h;
      }
      // the brick in hand keeps its true color at all times
      if (c && lastLift && !win && !BRICKS) {
        a.carried.material = cellMatHex(lastLift.color);
        a.carried.visible = !hideCarried;
      }
      if (win) {
        const u = (tf - win.t0) / Math.max(1, win.t1 - win.t0);
        const railY = GANTRY * BLOCK_H;             // cart plane (local y=0)
        const HANG = BLOCK_H * .62;                 // brick center below hook
        const restY = -HANG;                        // hanging at the cart
        const stopY = (win.z + .5) * BLOCK_H - railY;  // brick center at rest
        const k = u < .5 ? (u * 2) : (2 - u * 2);   // 0->1->0 travel profile
        const brickY = restY + (stopY - restY) * k; // brick-center path
        a.rope.visible = true;
        a.hook.visible = true;
        a.hook.position.y = brickY + HANG;
        a.rope.position.y = (brickY + HANG) / 2;
        a.rope.scale.y = Math.max(0.05, Math.abs(brickY + HANG));
        a.carried.material = cellMatHex(win.color);
        if (win.kind === 'drop') {
          // v2 bricks render themselves; legacy carried only without them
          a.carried.visible = !BRICKS;
          a.carried.position.y = u < .5 ? brickY : stopY;
        } else {
          // lift: the grounded block mesh rises along the same path (see
          // block loop); the in-hand mesh takes over only after t1
          a.carried.visible = false;
        }
      } else {
        a.rope.visible = false;
        a.hook.visible = false;
        if (c) a.carried.position.y = -BLOCK_H * .62;
      }
    }
    if (c && GANTRY) carryingCount++;
    if (c && !GANTRY) {
      // color the carried block by its destiny: the agent's next placement
      // gets that block's final color; scrap from a removal stays steel gray
      let prevEv = null, nextEv = null;
      for (const ev of actByAgent[a.idx]) {
        if (ev.t <= tf) prevEv = ev; else { nextEv = ev; break; }
      }
      let mat = null;
      const deposits = depositsByAgent[a.idx] || [];
      const scrapped = prevEv && !prevEv.isPlace &&
        !deposits.some(dt => dt > prevEv.t && dt <= tf);
      if (scrapped) {
        mat = scaffoldMats[0];
      } else if (nextEv && nextEv.isPlace) {
        const sc = nextEv.h > finalHeightAt(nextEv.r, nextEv.c);
        mat = sc ? scaffoldMats[0] : cellMat(nextEv.r, nextEv.c, nextEv.h);
      }
      a.carried.material = mat || a.carriedDefaultMat;
      if (LEGO && nextEv && nextEv.bw) {
        a.carried.scale.set(Math.min(nextEv.bw, 5) * 0.85, 1, Math.min(nextEv.bh, 5) * 0.85);
      } else {
        a.carried.scale.set(1, 1, 1);
      }
      carryingCount++;
    }
  }

  // stats: rewrite DOM only when the displayed step changes (per-frame
  // innerHTML rewrites cause constant reflow and visible stutter)
  if (t0 !== lastStatsT) {
    lastStatsT = t0;
    const t0show = Math.min(t0, T);  // padding frames still display T
    let placed = 0, removed = 0;
    for (const ev of placeEvents) if (ev.t <= tf) placed++;
    for (const ev of removeEvents) if (ev.t <= tf) removed++;
    const extra = (PLAN.meta && PLAN.meta.statsLine)
      ? PLAN.meta.statsLine
      : (PLAN.pyramid ? `金字塔 ${PLAN.pyramid.base}×${PLAN.pyramid.base} 底座 · ${PLAN.pyramid.levels} 层` : '');
    statsEl.innerHTML = (lang === 'zh')
      ? `时间步 <b>${t0show}</b> / ${T} &nbsp;·&nbsp; 结构中 <b>${placed - removed}</b> 块` +
        ` (放 ${placed} / 拆 ${removed})` +
        `<br>机器人 ${agents.length} 台，搬运中 ${carryingCount} 台` +
        (extra ? `<br>${extra}` : '')
      : `Step <b>${t0show}</b> / ${T} &nbsp;·&nbsp; <b>${placed - removed}</b> blocks in structure` +
        ` (${placed} placed / ${removed} removed)` +
        `<br>${agents.length} robots, ${carryingCount} carrying`;
    timeline.value = t0;
  }
}

// ---------------- camera orbit ----------------
let camTheta = 0.9, camPhi = 0.42, camDist = Math.max(26, Math.max(ROWS, COLS) * 1.35);
const camTarget = new THREE.Vector3(0, 1.2, 0);
const CAM_T0 = camTarget.clone();
const PAN_LIM = Math.max(ROWS, COLS) * 0.75 + 6;
let dragging = false, panning = false, lastX = 0, lastY = 0;
if (renderer) {
  renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());
  renderer.domElement.addEventListener('mousedown', e => {
    dragging = true;
    // Isaac Sim / Unity style: hold Cmd (mac) or Ctrl (win/linux), or use
    // the middle button, to pan the camera instead of orbiting
    panning = e.metaKey || e.ctrlKey || e.button === 1;
    if (e.button === 1) e.preventDefault();
    lastX = e.clientX; lastY = e.clientY;
  });
  renderer.domElement.addEventListener('dblclick', () => {
    camTarget.copy(CAM_T0);  // double-click: recenter
  });
  const MAXDIST = Math.max(66, camDist * 1.6);
  renderer.domElement.addEventListener('wheel', e => {
    e.preventDefault();  // keep the (parent) page from scrolling / zooming
    let d = e.deltaY;
    if (e.deltaMode === 1) d *= 33;        // line-based wheels
    const k = e.ctrlKey ? 0.011 : 0.0022;  // trackpad pinch sends ctrlKey
    camDist = Math.min(MAXDIST, Math.max(7, camDist * Math.exp(d * k)));
  }, { passive: false });
}
addEventListener('mouseup', () => { dragging = false; panning = false; });
addEventListener('mousemove', e => {
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
  if (panning || e.metaKey || e.ctrlKey) {
    // pan in the camera's screen plane, scaled by distance
    const f = camDist * 0.0016;
    const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
    camTarget.addScaledVector(right, -dx * f);
    camTarget.addScaledVector(up, dy * f);
    camTarget.x = Math.max(-PAN_LIM, Math.min(PAN_LIM, camTarget.x));
    camTarget.z = Math.max(-PAN_LIM, Math.min(PAN_LIM, camTarget.z));
    camTarget.y = Math.max(0, Math.min(PAN_LIM, camTarget.y));
    return;
  }
  camTheta -= dx * 0.005;
  camPhi = Math.min(1.35, Math.max(0.12, camPhi + dy * 0.004));
});

function updateCamera(dt) {
  if (!dragging) camTheta += dt * 0.03;
  camera.position.set(
    camTarget.x + camDist * Math.cos(camPhi) * Math.sin(camTheta),
    camTarget.y + camDist * Math.sin(camPhi),
    camTarget.z + camDist * Math.cos(camPhi) * Math.cos(camTheta));
  camera.lookAt(camTarget);
  camera.updateMatrix();
}

// ---------------- controls ----------------
const playBtn = document.getElementById('playBtn');
playBtn.onclick = () => {
  playing = !playing;
  playBtn.textContent = lang === 'zh' ? (playing ? '暂停' : '播放')
                                      : (playing ? 'Pause' : 'Play');
};
document.getElementById('resetBtn').onclick = () => { simT = 0; };
const endBtn = document.getElementById('endBtn');
if (endBtn) endBtn.onclick = () => { simT = T_END; playing = false; };
document.getElementById('speed').oninput = e => { speed = +e.target.value; };
timeline.oninput = e => { simT = +e.target.value; };

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  if (renderer) renderer.setSize(innerWidth, innerHeight);
});

// ---------------- HUD extras: scene nav, collapse, EN/中 toggle ----------
const SCENES = [
  ['index.html', '整板搭桌子', 'Table (exact board)'],
  ['horse.html', '木马', 'Trojan Horse'],
  ['dense.html', '高密度重排', 'Dense Re-sort'],
  ['red.html', '底层翻顶层', 'Bottom Layer to Top'],
  ['report.html', '算法报告', 'Report'],
];
let lang = localStorage.getItem('demoLang') || 'zh';
// per-page English copy: [h1, sub, legend lines (dot colors reused in order)]
const PAGE_EN = {
  'red.html': ['Bottom Layer to Top · Column Rotation Pipeline',
    '5×5 box, 100 bricks: every column has a red brick at the bottom under three grays, one spare layer on top · 8 robots run 4 focus columns in parallel: dig grays into neighbors, hold the red airborne, refill the bottom with borrowed grays, cap with red',
    ['Red bricks (each column z=0 → top z=3)', 'Gray bricks (dug out, parked, refilled)', 'Top-rail robots (brick under the hull = hoisting)', 'Focus columns']],
  'index.html': ['Multi-Robot Pyramid Construction',
    'Offline LaCAM-TAPF planning · layer by layer, inside-out · moves limited to |Δh| ≤ 1',
    ['Sandstone blocks (pyramid)', 'Robots (bright cap = carrying)', 'Depot (block pickup)']],
  'column.html': ['Column + Scaffold Stair',
    'Build the stair → raise the column → cap it → strip the stair back to the depot',
    ['Structure (column)', 'Scaffold (temporary stair, removed)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
  'bridge.html': ['Twin-Tower Arch Bridge · Suspended Erection',
    'Minecraft-style side placement: cantilever the deck out from both ends, then strip the stairs',
    ['Structure (towers + floating deck)', 'Scaffold (stairs, removed)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
  'temple.html': ['Temple: Base, Walls, Floating Roof',
    'Wall waves converge · roof cantilevered over the courtyard · both stairs stripped afterwards',
    ['Structure (base ring + walls + roof)', 'Scaffold (east & north stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'colonnade.html': ['Colonnade Temple · 14 Pillars, Floating Roof',
    'Ring scaffold wall and pillars grow hand-in-hand · roof closes by cantilever · wall stripped to the base',
    ['Structure (pillars + roof + base ring)', 'Scaffold (ring wall + stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'scene.html': ['Giza Site: Two Pyramids + Obelisk',
    '10 robots on site · pyramids self-ramping · the obelisk gets a stair, stripped afterwards',
    ['Structure (pyramids + obelisk)', 'Scaffold (obelisk stair)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
  'horse.html': ['Trojan Horse',
    'Four leg columns · floating belly (hollow below) · stepped neck · cantilevered head · ring scaffold fully stripped',
    ['Horse (legs, belly, neck, head)', 'Scaffold (ring wall + 4 stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'goldengate.html': ['Golden Gate Bridge',
    'Portal towers (h16), catenary cables with suspenders, through deck, both-shore depots; work walls stripped afterwards',
    ['Structure (towers + deck + cables, international orange)', 'Scaffold (work walls + pockets, removed)', 'Robots (bright cap = carrying)', 'Depot (both shores)']],
  'uscgate.html': ['USC Gate · Cardinal & Gold',
    'Two cardinal brick pillars + gold lintel, gold USC inlay on the plaza',
    ['Structure (gate + USC ground inlay)', 'Scaffold (stairs, removed)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
  'tommy.html': ['Tommy Trojan · USC Warrior Statue',
    'Raising the Sword of Knowledge, holding the Shield of Courage; the blade is an ascending float chain erected from a scaffold wall, stripped afterwards',
    ['Statue (pedestal + warrior + sword & shield)', 'Scaffold (stair-wall, removed)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
  'brickcar.html': ['BrickGPT Brick Car',
    'A StableText2Brick structure converted to voxels: solid columns + floating shell, windows filled as glass',
    ['Brick structure (red body / glass windows)', 'Scaffold (stair, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'brickchair.html': ['BrickGPT Brick Chair',
    'A StableText2Brick structure: 1x1 legs served by side stairs, self-raising 2-thick back, gold finish',
    ['Brick structure (gold)', 'Scaffold (stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'brickguitar.html': ['BrickGPT Brick Guitar',
    'Wood-tone body and neck; the head spire is served by scaffold stacked on the finished body, stripped afterwards',
    ['Brick structure (wood)', 'Scaffold (stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'brickpiano.html': ['BrickGPT Brick Piano',
    'Black grand piano: the legs under the floating lid are built with temporary in-footprint stairs, stripped before the lid closes',
    ['Brick structure (black / glass)', 'Scaffold (stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'brougham.html': ['Victorian Horse-Drawn Brougham',
    'MOC-194603 brick list: carriage body AND horse are laid brick by brick; wheels, shafts and lamps are tied to anchor bricks; scaffold stripped afterwards',
    ['Brick structure (black body / glass / red seat)', 'Scaffold (delivery stairs, removed)', 'Robots (bright cap = carrying)', 'Depot']],
  'symbot.html': ['SymBot · Symbotic-style Case Bot',
    'Charcoal frame, tall green-faced front tower, low rear module, light-grey case sitting in the central cargo bay, green skirt band',
    ['Structure (the bot)', 'Scaffold (removed)', 'Robots (bright cap = carrying)', 'Depot (pickup / scrap return)']],
};
const hud = document.getElementById('hud');
if (hud && EMBED) hud.style.display = 'none';
if (EMBED) document.body.style.background = '#eef2f7';
if (hud) {
  const here = location.pathname.split('/').pop() || 'index.html';
  const h1el = hud.querySelector('h1');
  const subEl = hud.querySelector('.sub');
  const legendEl = document.getElementById('legend');
  const zh = {
    h1: h1el ? h1el.textContent : '',
    sub: subEl ? subEl.textContent : '',
    legend: legendEl ? legendEl.innerHTML : '',
  };
  const dots = zh.legend
    ? Array.from(zh.legend.matchAll(/<span class="dot"[^>]*><\/span>/g), m => m[0])
    : [];
  const renderCopy = () => {
    const en = PAGE_EN[here];
    if (lang === 'en' && en) {
      if (h1el) h1el.textContent = en[0];
      if (subEl) subEl.textContent = en[1];
      if (legendEl) {
        legendEl.innerHTML = en[2].map((t, i) => (dots[i] || '') + t).join('<br>');
      }
      hud.dataset.minititle = en[0];
    } else {
      if (h1el) h1el.textContent = zh.h1;
      if (subEl) subEl.textContent = zh.sub;
      if (legendEl) legendEl.innerHTML = zh.legend;
      hud.dataset.minititle = zh.h1;
    }
  };
  hud.dataset.minititle = h1el ? h1el.textContent : 'demo';
  const bar = document.createElement('div');
  bar.id = 'hudbar';
  const langBtn = document.createElement('button');
  const collBtn = document.createElement('button');
  bar.appendChild(langBtn);
  bar.appendChild(collBtn);
  hud.appendChild(bar);
  const nav = document.createElement('div');
  nav.id = 'scenenav';
  hud.appendChild(nav);
  const renderNav = () => {
    nav.innerHTML = '<b style="color:#8b98b3">' +
      (lang === 'zh' ? '所有场景：' : 'All scenes: ') + '</b>' +
      SCENES.map(([f, zh, en]) =>
        `<a href="${f}" class="${f === here ? 'cur' : ''}">${lang === 'zh' ? zh : en}</a>`
      ).join('');
  };
  const renderBar = () => {
    langBtn.textContent = lang === 'zh' ? 'EN' : '中';
    collBtn.textContent = hud.classList.contains('collapsed')
      ? (lang === 'zh' ? '展开' : 'Show')
      : (lang === 'zh' ? '隐藏' : 'Hide');
    playBtn.textContent = lang === 'zh' ? (playing ? '暂停' : '播放')
                                        : (playing ? 'Pause' : 'Play');
    const rb = document.getElementById('resetBtn');
    if (rb) rb.textContent = lang === 'zh' ? '重来' : 'Reset';
    const sl = document.querySelector('#controls label');
    if (sl && sl.firstChild) sl.firstChild.textContent = lang === 'zh' ? '速度 ' : 'Speed ';
  };
  langBtn.onclick = () => {
    lang = lang === 'zh' ? 'en' : 'zh';
    localStorage.setItem('demoLang', lang);
    renderNav();
    renderBar();
    renderCopy();
    lastStatsT = -1;  // force the stats line to re-render in the new language
  };
  collBtn.onclick = () => {
    hud.classList.toggle('collapsed');
    renderBar();
  };
  renderNav();
  renderBar();
  renderCopy();
}

// ---------------- embed control API ----------------
if (EMBED) {
  window.addEventListener('message', ev => {
    const m = ev.data && ev.data.lacamCmd;
    if (!m) return;
    if (m.cmd === 'toggle') playing = !playing;
    else if (m.cmd === 'play') playing = true;
    else if (m.cmd === 'pause') playing = false;
    else if (m.cmd === 'reset') { simT = 0; playing = true; }
    else if (m.cmd === 'end') { simT = T_END; playing = false; }
    else if (m.cmd === 'speed') speed = +m.value;
    else if (m.cmd === 'seek') { simT = Math.max(0, Math.min(T_END, +m.value)); playing = false; }
  });
  setInterval(() => {
    if (window.parent !== window) {
      window.parent.postMessage({ lacamState: {
        t: Math.min(Math.floor(simT), T), simT: simT, T: T, T_END: T_END,
        playing: playing, speed: speed } }, '*');
    }
  }, 120);
}

// ---------------- main loop ----------------
let prev = performance.now();
function animate(now) {
  requestAnimationFrame(animate);
  const dt = Math.max(0, Math.min((now - prev) / 1000, 0.1));
  prev = now;
  if (playing) {
    simT += dt * speed;
    if (simT >= T_END) simT = T_END;
  }
  updateWorld(simT);
  updateCamera(dt);
  if (renderer) {
    try {
      renderer.render(scene, camera);
    } catch (e) {
      renderer = null;  // software GL failed: keep HUD playback alive
    }
  }
}
requestAnimationFrame(animate);

} catch (__e) {
  const el = document.getElementById('stats');
  if (el) el.textContent = '启动错误: ' + __e.message + ' :: ' + (__e.stack || '').split('\n').slice(0,2).join(' | ');
}
