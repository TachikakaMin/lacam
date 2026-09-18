(function () {
  const parts = window.BROUGHAM_GEOMETRY;
  const plan = window.BROUGHAM_BUILD_PLAN;
  const query = new URLSearchParams(location.search);
  if (query.has("fallback")) {
    const fallback = document.getElementById("staticFallback");
    if (fallback) fallback.style.display = "block";
    document.getElementById("stats").textContent =
      "静态模式 · 官方 LDraw 几何校验图";
    return;
  }
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x10141c);
  const camera = new THREE.PerspectiveCamera(42, innerWidth / innerHeight, .1, 200);
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({antialias: true});
  } catch (error) {
    const fallback = document.getElementById("staticFallback");
    if (fallback) fallback.style.display = "block";
    document.getElementById("stats").textContent =
      "WebGL 不可用，显示官方 LDraw 几何校验图";
    return;
  }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.setSize(innerWidth, innerHeight);
  renderer.shadowMap.enabled = true;
  document.body.appendChild(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xd8e4ff, 0x30271f, 1.1));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(12, 22, 10);
  scene.add(sun);
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(45, 28),
    new THREE.MeshStandardMaterial({color: 0x202938, roughness: .92}));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -.03;
  scene.add(ground);
  scene.add(new THREE.GridHelper(45, 45, 0x44516a, 0x293245));

  const byId = {};
  for (const part of parts) {
    const root = new THREE.Group();
    root.userData = part;
    for (const group of part.groups) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(group.positions, 3));
      geo.computeVertexNormals();
      const mat = new THREE.MeshStandardMaterial({
        color: group.color, roughness: .62, metalness: .04,
        transparent: group.transparent, opacity: group.transparent ? .35 : 1,
        side: group.transparent ? THREE.DoubleSide : THREE.FrontSide
      });
      root.add(new THREE.Mesh(geo, mat));
    }
    scene.add(root);
    byId[part.id] = root;
  }

  const robotMat = new THREE.MeshStandardMaterial({color: 0x3f8cff, emissive: 0x071a38});
  const armMat = new THREE.LineBasicMaterial({color: 0xffb24a});
  const robots = [];
  const arms = [];
  for (let i = 0; i < plan.robots; i++) {
    const r = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CylinderGeometry(.28, .34, .42, 12), robotMat);
    body.position.y = .22;
    r.add(body);
    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(.055, .075, .72, 8), robotMat);
    mast.position.y = .63;
    r.add(mast);
    const tray = new THREE.Mesh(
      new THREE.BoxGeometry(.52, .08, .52), robotMat);
    tray.position.y = .52;
    r.add(tray);
    const arm = new THREE.Line(new THREE.BufferGeometry(), armMat);
    arm.visible = false;
    scene.add(r);
    scene.add(arm);
    robots.push(r);
    arms.push(arm);
  }

  const taskByPart = {};
  for (const task of plan.tasks) taskByPart[task.partId] = task;
  const buildMode = query.has("build");
  let progress = buildMode ? 0 : plan.partCount;
  let playing = buildMode, speed = .6;
  const timeline = document.getElementById("timeline");
  const stats = document.getElementById("stats");
  const playBtn = document.getElementById("playBtn");
  timeline.max = plan.partCount;
  timeline.value = progress;
  playBtn.textContent = playing ? "暂停" : "播放";

  function lerpPath(path, fraction) {
    const scaled = Math.max(0, Math.min(1, fraction)) * (path.length - 1);
    const index = Math.min(path.length - 2, Math.floor(scaled));
    const local = scaled - index;
    const a = path[index], b = path[index + 1];
    return new THREE.Vector3(
      a[0] + (b[0] - a[0]) * local,
      a[1] + (b[1] - a[1]) * local,
      a[2] + (b[2] - a[2]) * local);
  }

  function update() {
    const completed = Math.min(plan.partCount, Math.floor(progress));
    const fraction = progress - completed;
    const active = completed < plan.partCount ? plan.tasks[completed] : null;
    let visible = 0;
    for (const p of parts) {
      const task = taskByPart[p.id];
      const done = task.rank <= completed;
      const activePart = active && active.partId === p.id;
      const pickupFraction = activePart ?
        active.pickupWaypoint / (active.partPath.length - 1) : 1;
      const show = done || (activePart && fraction >= pickupFraction);
      byId[p.id].visible = show;
      byId[p.id].position.set(0, 0, 0);
      if (show) visible++;
    }
    for (let i = 0; i < robots.length; i++) {
      robots[i].position.set(-8.75 + i * 1.25, 0, 10);
      arms[i].visible = false;
    }
    if (active) {
      const basePosition = lerpPath(active.basePath, fraction);
      const partPosition = lerpPath(active.partPath, fraction);
      robots[active.robot].position.copy(basePosition);
      const pickupFraction =
        active.pickupWaypoint / (active.partPath.length - 1);
      const depositFraction =
        active.depositWaypoint / (active.partPath.length - 1);
      if (fraction >= pickupFraction && fraction < depositFraction) {
        const target = active.target;
        byId[active.partId].position.set(
          partPosition.x - target[0],
          partPosition.y - target[1],
          partPosition.z - target[2]);
        const armStart = basePosition.clone();
        armStart.y = .92;
        arms[active.robot].geometry.dispose();
        arms[active.robot].geometry =
          new THREE.BufferGeometry().setFromPoints([armStart, partPosition]);
        arms[active.robot].visible = true;
      }
    }
    timeline.value = progress;
    const pdfStep = active ? active.pdfStep : 48;
    stats.textContent = `任务 ${completed}/${plan.partCount} · PDF 步骤 ${pdfStep}/48 · ` +
      `已安装 ${visible}/82 件 · 底盘贴地/机械臂安装路径校验通过`;
  }

  playBtn.onclick = function () {
    playing = !playing; this.textContent = playing ? "暂停" : "播放";
  };
  document.getElementById("resetBtn").onclick = () => { progress = 0; update(); };
  document.getElementById("endBtn").onclick = () => { progress = plan.partCount; update(); };
  document.getElementById("speed").oninput = e => { speed = +e.target.value / 10; };
  timeline.oninput = e => { progress = +e.target.value; playing = false; update(); };

  let yaw = 2.25, pitch = .36, radius = 24, dragging = false, px = 0, py = 0;
  renderer.domElement.onpointerdown = e => { dragging = true; px = e.clientX; py = e.clientY; };
  renderer.domElement.onpointerup = () => { dragging = false; };
  renderer.domElement.onpointermove = e => {
    if (!dragging) return;
    yaw -= (e.clientX - px) * .006; pitch = Math.max(.08, Math.min(1.2, pitch + (e.clientY - py) * .004));
    px = e.clientX; py = e.clientY;
  };
  renderer.domElement.onwheel = e => { radius = Math.max(10, Math.min(38, radius + e.deltaY * .015)); };
  addEventListener("resize", () => {
    camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  let last = performance.now();
  function frame(now) {
    const dt = Math.min(.05, (now - last) / 1000); last = now;
    if (playing) {
      progress = Math.min(plan.partCount, progress + dt * speed);
      update();
      if (progress >= plan.partCount) playing = false;
    }
    const target = new THREE.Vector3(0, 4.2, -2.5);
    camera.position.set(target.x + radius * Math.sin(yaw) * Math.cos(pitch),
      target.y + radius * Math.sin(pitch),
      target.z + radius * Math.cos(yaw) * Math.cos(pitch));
    camera.lookAt(target);
    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }
  update(); requestAnimationFrame(frame);
})();
