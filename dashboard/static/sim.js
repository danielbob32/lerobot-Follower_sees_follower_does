// sim.js — SO-101 3D viewport. Phase 1: ghost arm only, driven by /api/state.
(function () {
  const VIEW_EL = document.getElementById('viewport');
  const STATUS_EL = document.getElementById('sim-status');

  function setStatus(text, isErr) {
    STATUS_EL.textContent = text;
    STATUS_EL.classList.toggle('err', !!isErr);
  }

  // --- three.js scene ----------------------------------------------------
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1115);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
  camera.position.set(0.4, 0.3, 0.5);
  camera.up.set(0, 0, 1); // Z-up matches URDF convention

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x0f1115, 1);
  VIEW_EL.appendChild(renderer.domElement);

  function sizeToContainer() {
    const w = VIEW_EL.clientWidth;
    const h = VIEW_EL.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  sizeToContainer();
  window.addEventListener('resize', sizeToContainer);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.15);
  controls.update();

  // Lights
  const hemi = new THREE.HemisphereLight(0xffffff, 0x222233, 0.9);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(0.5, 0.5, 1);
  scene.add(dir);

  // Ground plane grid
  const grid = new THREE.GridHelper(1, 10, 0x262b35, 0x161a21);
  grid.rotation.x = Math.PI / 2; // align with Z-up world
  scene.add(grid);

  // --- URDF load ---------------------------------------------------------
  function endEffectorWorld(robot) {
    const candidates = ["gripper_frame_link", "moving_jaw_so101_v1_link", "gripper_link", "wrist_roll_link", "end_effector_link"];
    for (let i = 0; i < candidates.length; i++) {
      const name = candidates[i];
      if (robot.links && robot.links[name]) {
        const v = new THREE.Vector3();
        robot.links[name].getWorldPosition(v);
        return v;
      }
    }
    const names = Object.keys(robot.links || {});
    const last = robot.links[names[names.length - 1]];
    const v = new THREE.Vector3();
    if (last) last.getWorldPosition(v);
    return v;
  }

  function tintRobot(robot, opts) {
    robot.traverse(function (n) {
      if (n.isMesh) {
        n.material = new THREE.MeshStandardMaterial({
          color: opts.color,
          transparent: opts.transparent,
          opacity: opts.opacity,
          metalness: 0.1,
          roughness: 0.9,
        });
      }
    });
  }

  const loader = new URDFLoader();
  loader.loadMeshCb = function (path, manager, onLoad) {
    const stl = new THREE.STLLoader(manager);
    stl.load(path, function (geom) {
      const mat = new THREE.MeshStandardMaterial({
        color: 0x7b8496, transparent: true, opacity: 0.35,
        metalness: 0.1, roughness: 0.9,
      });
      onLoad(new THREE.Mesh(geom, mat));
    });
  };

  let ghostRobot = null;
  let targetRobot = null;
  let endEffectorHandle = null;
  let calibration = null;
  const JOINT_ORDER = ["shoulder_pan","shoulder_lift","elbow_flex"];

  loader.load('/models/so101/so101.urdf', function (robot) {
    ghostRobot = robot;
    tintRobot(ghostRobot, { color: 0x7b8496, transparent: true, opacity: 0.35 });
    scene.add(ghostRobot);

    loader.load('/models/so101/so101.urdf', function (robot2) {
      targetRobot = robot2;
      tintRobot(targetRobot, { color: 0xd8dde5, transparent: false, opacity: 1.0 });
      targetRobot.visible = false;
      scene.add(targetRobot);

      endEffectorHandle = new THREE.Mesh(
        new THREE.SphereGeometry(0.015, 16, 12),
        new THREE.MeshStandardMaterial({
          color: 0x4c8df6, emissive: 0x4c8df6, emissiveIntensity: 0.2
        })
      );
      endEffectorHandle.visible = false;
      scene.add(endEffectorHandle);
      window.SIM = { ghostRobot, targetRobot, endEffectorHandle };
      setStatus('ghost live');
      document.getElementById('engage-btn').disabled = false;
    });
  }, null, function (err) {
    console.error('URDF load failed', err);
    setStatus('Failed to load SO-101 model. Check dashboard/static/models/so101/.', true);
    document.getElementById('engage-btn').disabled = true;
  });

  // --- CCD IK -----------------------------------------------------------
  function ccdSolve(robot, targetPos, iterations) {
    if (!robot) return 0;
    const ee = new THREE.Vector3();
    const jointPos = new THREE.Vector3();
    const axisWorld = new THREE.Vector3();
    const toEe = new THREE.Vector3();
    const toTarget = new THREE.Vector3();

    for (let iter = 0; iter < iterations; iter++) {
      for (let ji = JOINT_ORDER.length - 1; ji >= 0; ji--) {
        const name = JOINT_ORDER[ji];
        const joint = robot.joints[name];
        if (!joint) continue;

        ee.copy(endEffectorWorld(robot));
        joint.getWorldPosition(jointPos);
        toEe.copy(ee).sub(jointPos);
        toTarget.copy(targetPos).sub(jointPos);
        if (toEe.lengthSq() < 1e-10 || toTarget.lengthSq() < 1e-10) continue;
        toEe.normalize(); toTarget.normalize();

        axisWorld.copy(joint.axis).transformDirection(joint.matrixWorld);

        const eeDot = toEe.dot(axisWorld);
        const tgDot = toTarget.dot(axisWorld);
        const eeProj = toEe.clone().addScaledVector(axisWorld, -eeDot).normalize();
        const tgProj = toTarget.clone().addScaledVector(axisWorld, -tgDot).normalize();

        let angle = Math.acos(Math.min(1, Math.max(-1, eeProj.dot(tgProj))));
        const cross = new THREE.Vector3().crossVectors(eeProj, tgProj);
        if (cross.dot(axisWorld) < 0) angle = -angle;
        if (!isFinite(angle) || Math.abs(angle) < 1e-5) continue;

        const newVal = joint.angle + angle;
        const lo = joint.limit && typeof joint.limit.lower === 'number' ? joint.limit.lower : -Math.PI;
        const hi = joint.limit && typeof joint.limit.upper === 'number' ? joint.limit.upper :  Math.PI;
        joint.setJointValue(Math.max(lo, Math.min(hi, newVal)));
      }
    }

    ee.copy(endEffectorWorld(robot));
    return ee.distanceTo(targetPos);
  }

  // --- Drag handling ----------------------------------------------------
  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  let dragging = false;
  const dragPlane = new THREE.Plane();
  const dragPoint = new THREE.Vector3();
  let ikResidual = 0;

  function setMouseNDC(ev) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndc.x =  ((ev.clientX - rect.left) / rect.width)  * 2 - 1;
    ndc.y = -((ev.clientY - rect.top)  / rect.height) * 2 + 1;
  }

  renderer.domElement.addEventListener('pointerdown', function (ev) {
    if (!endEffectorHandle || !endEffectorHandle.visible) return;
    setMouseNDC(ev);
    raycaster.setFromCamera(ndc, camera);
    const hits = raycaster.intersectObject(endEffectorHandle, false);
    if (hits.length > 0) {
      dragging = true;
      controls.enabled = false;
      dragPlane.setFromNormalAndCoplanarPoint(
        camera.getWorldDirection(new THREE.Vector3()).negate(),
        endEffectorHandle.position.clone());
      renderer.domElement.setPointerCapture(ev.pointerId);
      ev.preventDefault();
    }
  });

  renderer.domElement.addEventListener('pointermove', function (ev) {
    if (!dragging) return;
    setMouseNDC(ev);
    raycaster.setFromCamera(ndc, camera);
    if (raycaster.ray.intersectPlane(dragPlane, dragPoint)) {
      ikResidual = ccdSolve(targetRobot, dragPoint, 30);
      const ee = endEffectorWorld(targetRobot);
      endEffectorHandle.position.copy(ee);
      const color = (ikResidual > 0.02) ? 0xe5484d : 0x4c8df6;
      endEffectorHandle.material.color.setHex(color);
      endEffectorHandle.material.emissive.setHex(color);
      pushTarget();
    }
  });

  renderer.domElement.addEventListener('pointerup', function (ev) {
    if (!dragging) return;
    dragging = false;
    controls.enabled = true;
    try { renderer.domElement.releasePointerCapture(ev.pointerId); } catch (e) {}
  });

  // --- Calibration fetch + telemetry -> ghost pose ----------------------
  fetch('/calibration.json').then(function (r) { return r.json(); })
                            .then(function (c) { calibration = c; });

  function ticksToRad(tick, j) {
    return ((tick - j.tick_zero) / calibration.tick_per_rad) * j.tick_sign;
  }

  function displayRad(tick, j) {
    // Ticks -> radians, plus a visual offset so the URDF's zero pose can be
    // shifted to match whatever physical pose the user considers "home".
    return ticksToRad(tick, j) + (j.angle_offset_rad || 0);
  }

  function applyFollowerPose(motorsById) {
    if (!ghostRobot || !calibration) return;
    calibration.joints.forEach(function (j) {
      const motor = motorsById[j.id];
      if (!motor || motor.pos === null || motor.pos === undefined) return;
      const rad = displayRad(motor.pos, j);
      const urdfJoint = ghostRobot.joints[j.name];
      if (urdfJoint) urdfJoint.setJointValue(rad);
    });
  }

  let lastFollowerByName = {};
  window.addEventListener('so101:state', function (ev) {
    const data = ev.detail;
    const byId = {};
    data.motors.filter(function (m) { return m.role === 'follower'; })
               .forEach(function (m) { byId[m.id] = m; });
    applyFollowerPose(byId);
    if (calibration) {
      lastFollowerByName = {};
      calibration.joints.forEach(function (j) {
        const m = byId[j.id];
        if (m && typeof m.pos === 'number') lastFollowerByName[j.name] = m.pos;
      });
    }
  });

  // --- API client + engage/release + heartbeat ---------------------------
  let engaged = false;
  let heartbeatTimer = null;
  let postInFlight = false;
  let pendingTargetTicks = null;
  const engageBtn = document.getElementById('engage-btn');

  async function post(path, body) {
    const r = await fetch(path, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : null,
    });
    const text = await r.text();
    let data = {};
    try { data = JSON.parse(text); } catch (e) {}
    return { ok: r.ok, status: r.status, data };
  }

  function radToTick(rad, j) {
    // Inverse of displayRad: subtract the visual offset before converting
    // back to ticks, so slider/IK-driven URDF angles produce the right tick.
    const raw = rad - (j.angle_offset_rad || 0);
    return Math.round(j.tick_zero + j.tick_sign * raw * calibration.tick_per_rad);
  }

  function syncTargetRobotFromSliders() {
    if (!targetRobot || !calibration) return;
    [['wrist_flex','slider-wrist-pitch'],
     ['wrist_roll','slider-wrist-roll'],
     ['gripper',   'slider-gripper']].forEach(function (pair) {
      const name = pair[0], sliderId = pair[1];
      const j = calibration.joints.find(function (x) { return x.name === name; });
      if (!j) return;
      const tick = +document.getElementById(sliderId).value;
      const urdfJoint = targetRobot.joints[name];
      if (urdfJoint) urdfJoint.setJointValue(displayRad(tick, j));
    });
  }

  function currentSimTargetTicks() {
    if (!targetRobot || !calibration) return null;
    syncTargetRobotFromSliders();
    const bySlider = {
      wrist_flex: +document.getElementById('slider-wrist-pitch').value,
      wrist_roll: +document.getElementById('slider-wrist-roll').value,
      gripper:    +document.getElementById('slider-gripper').value,
    };
    return calibration.joints.map(function (j) {
      if (j.name in bySlider) return bySlider[j.name];
      const urdfJoint = targetRobot.joints[j.name];
      return radToTick(urdfJoint ? urdfJoint.angle : 0, j);
    });
  }

  async function pushTarget() {
    if (!engaged) return;
    const ticks = currentSimTargetTicks();
    if (!ticks) return;
    if (postInFlight) { pendingTargetTicks = ticks; return; }
    postInFlight = true;
    try {
      await post('/api/target', { joints: ticks });
    } catch (e) {
      setStatus('control offline — retrying', true);
    } finally {
      postInFlight = false;
      if (pendingTargetTicks) {
        const t = pendingTargetTicks; pendingTargetTicks = null;
        post('/api/target', { joints: t }).catch(function () {});
      }
    }
  }

  async function setEngaged(on) {
    const res = await post(on ? '/api/engage' : '/api/release', null);
    if (on && res.status === 409) {
      setStatus('another client has control', true);
      return;
    }
    if (!res.ok) {
      setStatus((on ? 'engage' : 'release') + ' failed', true);
      return;
    }
    engaged = on;
    engageBtn.textContent = on ? 'Release Control' : 'Engage';
    engageBtn.classList.toggle('active', on);
    targetRobot.visible = on;
    endEffectorHandle.visible = on;
    if (on) {
      if (ghostRobot && calibration) {
        calibration.joints.forEach(function (j) {
          const urdfJoint = targetRobot.joints[j.name];
          const ghostJoint = ghostRobot.joints[j.name];
          if (urdfJoint && ghostJoint) urdfJoint.setJointValue(ghostJoint.angle);
        });
      }
      if (lastFollowerByName.wrist_flex !== undefined)
        document.getElementById('slider-wrist-pitch').value = lastFollowerByName.wrist_flex;
      if (lastFollowerByName.wrist_roll !== undefined)
        document.getElementById('slider-wrist-roll').value  = lastFollowerByName.wrist_roll;
      if (lastFollowerByName.gripper !== undefined)
        document.getElementById('slider-gripper').value     = lastFollowerByName.gripper;
      setStatus('sim control — heartbeat OK');
      heartbeatTimer = setInterval(function () {
        post('/api/heartbeat', null).catch(function () {});
      }, 500);
      pushTarget();
    } else {
      setStatus('ghost live');
      if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
    }
  }

  engageBtn.addEventListener('click', function () { setEngaged(!engaged); });

  ['slider-wrist-pitch', 'slider-wrist-roll', 'slider-gripper'].forEach(function (id) {
    document.getElementById(id).addEventListener('input', pushTarget);
  });

  // --- Align-ghost tuning sliders ---------------------------------------
  function buildAlignSliders() {
    if (!calibration) { setTimeout(buildAlignSliders, 200); return; }
    const container = document.getElementById('align-sliders');
    container.replaceChildren();
    calibration.joints.forEach(function (j, idx) {
      const row = document.createElement('div');
      row.className = 'align-row';
      const label = document.createElement('label');
      const name = document.createElement('span');
      name.textContent = j.name;
      const val = document.createElement('span');
      val.id = 'align-val-' + idx;
      val.textContent = (j.angle_offset_rad * 180 / Math.PI).toFixed(0) + '°';
      label.appendChild(name); label.appendChild(val);
      const input = document.createElement('input');
      input.type = 'range';
      input.min = '-180'; input.max = '180'; input.step = '1';
      input.value = String(Math.round((j.angle_offset_rad || 0) * 180 / Math.PI));
      input.dataset.idx = idx;
      input.addEventListener('input', function () {
        const deg = +input.value;
        const rad = deg * Math.PI / 180;
        calibration.joints[idx].angle_offset_rad = rad;
        val.textContent = deg + '°';
      });
      row.appendChild(label); row.appendChild(input);
      container.appendChild(row);
    });
  }
  buildAlignSliders();

  async function saveOffsets() {
    const statusEl = document.getElementById('align-status');
    const offsets = calibration.joints.map(function (j) {
      return { name: j.name, angle_offset_rad: j.angle_offset_rad || 0 };
    });
    statusEl.textContent = 'saving…';
    statusEl.className = 'align-status';
    try {
      const res = await post('/api/save_offsets', { offsets: offsets });
      if (res.ok) {
        statusEl.textContent = 'saved';
        statusEl.className = 'align-status ok';
      } else {
        statusEl.textContent = 'save failed';
        statusEl.className = 'align-status err';
      }
    } catch (e) {
      statusEl.textContent = 'save failed';
      statusEl.className = 'align-status err';
    }
  }

  document.getElementById('align-save').addEventListener('click', saveOffsets);

  async function resetOffsets() {
    if (!calibration) return;
    calibration.joints.forEach(function (j, idx) {
      j.angle_offset_rad = 0;
      const input = document.querySelector('#align-sliders input[data-idx="' + idx + '"]');
      const val = document.getElementById('align-val-' + idx);
      if (input) input.value = '0';
      if (val) val.textContent = '0°';
    });
    await saveOffsets();
  }

  document.getElementById('align-reset').addEventListener('click', resetOffsets);

  // --- Calibration: capture home + flip signs ---------------------------
  async function reloadCalibration() {
    const r = await fetch('/calibration.json', { cache: 'no-store' });
    calibration = await r.json();
    buildAlignSliders();
    buildFlipButtons();
  }

  function buildFlipButtons() {
    if (!calibration) return;
    const container = document.getElementById('flip-buttons');
    container.replaceChildren();
    calibration.joints.forEach(function (j) {
      const btn = document.createElement('button');
      btn.className = 'flip-btn' + (j.tick_sign < 0 ? ' inverted' : '');
      const name = document.createElement('span');
      name.textContent = j.name;
      const sign = document.createElement('span');
      sign.className = 'sign';
      sign.textContent = (j.tick_sign > 0 ? '+1' : '-1');
      btn.appendChild(name); btn.appendChild(sign);
      btn.addEventListener('click', async function () {
        const res = await post('/api/flip_sign', { name: j.name });
        if (res.ok) { await reloadCalibration(); }
      });
      container.appendChild(btn);
    });
  }

  async function captureHome() {
    const statusEl = document.getElementById('capture-status');
    statusEl.textContent = 'capturing…';
    statusEl.className = 'align-status';
    try {
      const res = await post('/api/capture_home', null);
      if (res.ok) {
        statusEl.textContent = 'home captured';
        statusEl.className = 'align-status ok';
        await reloadCalibration();
      } else {
        statusEl.textContent = (res.data && res.data.message) || 'capture failed';
        statusEl.className = 'align-status err';
      }
    } catch (e) {
      statusEl.textContent = 'capture failed';
      statusEl.className = 'align-status err';
    }
  }

  document.getElementById('capture-home').addEventListener('click', captureHome);

  // Populate flip buttons once calibration is loaded (may already be loaded).
  function waitForCalibration() {
    if (calibration) { buildFlipButtons(); return; }
    setTimeout(waitForCalibration, 200);
  }
  waitForCalibration();

  // --- Render loop -------------------------------------------------------
  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    if (engaged) syncTargetRobotFromSliders();
    if (endEffectorHandle && endEffectorHandle.visible && targetRobot) {
      endEffectorHandle.position.copy(endEffectorWorld(targetRobot));
    }
    renderer.render(scene, camera);
  }
  animate();
})();
