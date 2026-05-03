/**
 * ScaraUIController.js — DOM event wiring and HUD updates for the SCARA arm.
 * Bridges ScaraSceneManager (3D) with HTML sliders/buttons/text.
 * Uses the generalized N-DOF IK (DLS) solver.
 */
import * as THREE from 'three';
import { ik_dls, fk, jacobian, SCARA_DH_CONFIG } from '../math/KinematicsNDOF.js';
import PickAndPlaceStateMachine, { STATE, STATE_META } from '../logic/PickAndPlace.js';

import CameraPanel from './CameraPanel.js';

const $ = id => document.getElementById(id);
const DEG = v => v * Math.PI / 180;
const RAD = v => v * 180 / Math.PI;
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export default class ScaraUIController {
  constructor(sceneManager) {
    this.sm = sceneManager;
    this._tab = 'fk';

    // Joint state: [θ1, θ2, d3, θ4]
    this.q = [0, 0, 0, 0];

    // Raycaster
    this._raycaster = new THREE.Raycaster();
    this._mouse = new THREE.Vector2();
    this._isDragging = false;

    // Pick & Place state machine
    this._pnp = new PickAndPlaceStateMachine(sceneManager);
    this._pnp.onStateChange = (state, meta) => this._onTaskStateChange(state, meta);
    this._pnp.onUpdate = (info) => this._onTaskUpdate(info);
    this._taskRAF = null;
    this._lastTime = 0;

    // Interactive click state
    this._interactiveClickPhase = 0; // 0=none, 1=waiting pick, 2=waiting drop


    // ── TransformControls object selection ──
    // Mark payload as draggable
    this.sm.payload.userData.draggable = true;
    this.sm.payload.userData.role = 'payload';

    // Wire up drag sync callback
    this.sm.onObjectDragged = (mesh) => this._onSceneObjectDragged(mesh);

    // ── Camera Panel (lazy-mounted) ──
    this._cameraPanel = null;
    this._camRenderSkip = 0;

    // BroadcastChannel for math dashboard sync
    this._mathChannel = new BroadcastChannel('jinx_math_sync');
    // Command channel — receive FK/IK commands from dashboard
    this._cmdChannel = new BroadcastChannel('jinx_math_cmd');
    this._cmdChannel.onmessage = (e) => {
      const d = e.data;
      if (d.robot !== 'scara') return;
      if (d.type === 'fk' && d.q) {
        this.q = [...d.q];
        // Sync hidden FK sliders
        $('fk-q1').value = RAD(d.q[0]).toFixed(0);
        $('fk-q2').value = RAD(d.q[1]).toFixed(0);
        $('fk-q3').value = (d.q[2] * 1000).toFixed(0);
        $('fk-q4').value = RAD(d.q[3]).toFixed(0);
        this._updateScene(null);
      } else if (d.type === 'ik' && d.target) {
        $('ik-xt').value = d.target[0];
        $('ik-yt').value = d.target[1];
        $('ik-zt').value = d.target[2];
        this._onIKSlider();
      }
    };

    // Link card DOM refs
    this._selectedLink = -1;
    this._linkCard = $('linkCard');
    this._linkSlider = $('linkCardSlider');
    this._linkTitle = $('linkCardTitle');
    this._linkValue = $('linkCardValue');
    this._linkMin = $('linkCardMin');
    this._linkMax = $('linkCardMax');
    this._linkDefault = $('linkCardDefault');

    this._bindEvents();
    this._bindLinkCard();
  }

  /* ═══════════ Event Wiring ═══════════ */
  _bindEvents() {
    // Tab switching
    document.querySelectorAll('#tabBar .tab').forEach(btn => {
      btn.addEventListener('click', () => this.setTab(btn.dataset.tab));
    });

    // FK sliders
    ['fk-q1', 'fk-q2', 'fk-q3', 'fk-q4'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => this._onFKSlider());
    });

    // IK sliders
    ['ik-xt', 'ik-yt', 'ik-zt', 'ik-lambda', 'ik-maxiter'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => this._onIKSlider());
    });

    // Jacobian sliders
    ['jxd', 'jyd', 'jzd'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => this._onJacSlider());
    });

    // Raycaster (drag target sphere + interactive click)
    const dom = this.sm.renderer.domElement;
    dom.addEventListener('pointerdown', e => this._onPointerDown(e));
    window.addEventListener('pointermove', e => this._onPointerMove(e));
    window.addEventListener('pointerup', () => this._onPointerUp());

    // Task button
    const taskBtn = $('taskBtn');
    if (taskBtn) taskBtn.addEventListener('click', () => this._toggleTask());

    // Reset button
    const resetBtn = $('resetBtn');
    if (resetBtn) resetBtn.addEventListener('click', () => this._resetPose());



    // Camera button
    const camBtn = $('cameraBtn');
    if (camBtn) camBtn.addEventListener('click', () => this._toggleCamera());

    // Math dashboard
    const mathBtn = $('btn-math-panel');
    if (mathBtn) mathBtn.addEventListener('click', () => window.open('./math-dashboard.html?robot=scara', '_blank'));
  }

  /* ═══════════ Tabs ═══════════ */
  setTab(t) {
    this._tab = t;
    ['fk', 'ik', 'jac', 'task'].forEach(id => {
      const pane = $('pane-' + id);
      if (pane) pane.classList.toggle('on', id === t);
    });
    document.querySelectorAll('#tabBar .tab').forEach(el => {
      el.classList.toggle('active', el.dataset.tab === t);
    });
    this.update();
  }

  /* ═══════════ Reset Pose ═══════════ */
  _resetPose() {
    // Stop any running task
    if (this._pnp.running) {
      this._pnp.stop();
      cancelAnimationFrame(this._taskRAF);
      this._taskRAF = null;
      const btn = $('taskBtn');
      if (btn) { btn.textContent = '▶  Start Pick & Place'; btn.classList.remove('stop'); }
    }
    this._interactiveClickPhase = 0;
    const hint = $('clickHint');
    if (hint) hint.style.display = 'none';

    // Reset joints to home
    this.q = [0, 0, 0, 0];

    // Reset sliders
    $('fk-q1').value = 0; $('fk-q2').value = 0; $('fk-q3').value = 0; $('fk-q4').value = 0;
    $('fk-q1v').textContent = '0'; $('fk-q2v').textContent = '0';
    $('fk-q3v').textContent = '0'; $('fk-q4v').textContent = '0';

    // Reset trail
    this.sm.resetTrail();

    // Update scene
    this._updateScene(null);

    // Reset task cards
    $('task-state').textContent = 'IDLE';
    $('task-state').style.color = '#8080a8';
    $('task-cycle').textContent = '0';
    $('task-progress').textContent = '—';
    $('taskBar').style.width = '0%';
  }

  /* ═══════════ FK Slider Handler ═══════════ */
  _onFKSlider() {
    const q1 = DEG(+$('fk-q1').value);
    const q2 = DEG(+$('fk-q2').value);
    const q3 = +$('fk-q3').value / 1000; // mm → m
    const q4 = DEG(+$('fk-q4').value);

    $('fk-q1v').textContent = (+$('fk-q1').value).toFixed(0);
    $('fk-q2v').textContent = (+$('fk-q2').value).toFixed(0);
    $('fk-q3v').textContent = (+$('fk-q3').value).toFixed(0);
    $('fk-q4v').textContent = (+$('fk-q4').value).toFixed(0);

    this.q = [q1, q2, q3, q4];
    this._updateScene(null);
  }

  /* ═══════════ IK Slider Handler ═══════════ */
  _onIKSlider() {
    const xt = +$('ik-xt').value;
    const yt = +$('ik-yt').value;
    const zt = +$('ik-zt').value;
    const lambda = +$('ik-lambda').value;
    const maxIter = +$('ik-maxiter').value;

    $('ik-xtv').textContent = xt.toFixed(3);
    $('ik-ytv').textContent = yt.toFixed(3);
    $('ik-ztv').textContent = zt.toFixed(3);
    $('ik-lambdav').textContent = lambda.toFixed(2);
    $('ik-maxiterv').textContent = maxIter.toFixed(0);

    // Run DLS IK
    const target = [xt, yt, zt];
    const result = ik_dls(target, this.q, SCARA_DH_CONFIG, maxIter, lambda);

    this.q = result.q;

    // Update IK solution cards
    $('ik-t1').textContent = RAD(result.q[0]).toFixed(1) + '°';
    $('ik-t2').textContent = RAD(result.q[1]).toFixed(1) + '°';
    $('ik-d3').textContent = (result.q[2] * 1000).toFixed(1) + ' mm';
    $('ik-t4').textContent = RAD(result.q[3]).toFixed(1) + '°';
    $('ik-err').textContent = result.error.toExponential(2) + ' m';
    $('ik-conv').textContent = result.converged ? '✓ Yes' : '✗ No';
    $('ik-conv').style.color = result.converged ? 'var(--grn)' : 'var(--red2)';
    $('ik-iter').textContent = result.iterations;

    // Alert
    const alertEl = $('ikAlert');
    if (alertEl) alertEl.classList.toggle('on', !result.converged);

    // HUD status
    $('hStat').textContent = result.converged ? '✓ Valid' : '⚠ No Convergence';
    $('hStat').className = result.converged ? 'ok' : 'bad';
    $('hErr').textContent = result.error.toExponential(2) + ' m';

    // Update FK sliders to reflect IK solution
    $('fk-q1').value = RAD(result.q[0]).toFixed(0);
    $('fk-q2').value = RAD(result.q[1]).toFixed(0);
    $('fk-q3').value = (result.q[2] * 1000).toFixed(0);
    $('fk-q4').value = RAD(result.q[3]).toFixed(0);
    $('fk-q1v').textContent = RAD(result.q[0]).toFixed(0);
    $('fk-q2v').textContent = RAD(result.q[1]).toFixed(0);
    $('fk-q3v').textContent = (result.q[2] * 1000).toFixed(0);
    $('fk-q4v').textContent = RAD(result.q[3]).toFixed(0);

    // Target sphere position: DH(x,y,z) -> Three(x, z, -y)
    const pTgt3 = new THREE.Vector3(xt, zt, -yt);
    this._updateScene(pTgt3, result);
  }

  /* ═══════════ Jacobian Slider Handler ═══════════ */
  _onJacSlider() {
    const xt = +$('jxd').value;
    const yt = +$('jyd').value;
    const zt = +$('jzd').value;

    $('jxdv').textContent = xt.toFixed(3);
    $('jydv').textContent = yt.toFixed(3);
    $('jzdv').textContent = zt.toFixed(3);

    // Solve IK to get q for this target
    const result = ik_dls([xt, yt, zt], this.q, SCARA_DH_CONFIG, 150, 0.08);
    this.q = result.q;

    const pTgt3 = new THREE.Vector3(xt, zt, -yt);
    this._updateScene(pTgt3, result);
    this._updateJacobian();
  }

  /* ═══════════ Jacobian Display ═══════════ */
  _updateJacobian() {
    const { J, cols: n } = jacobian(this.q, SCARA_DH_CONFIG);
    const ids = [
      ['j11', 'j12', 'j13', 'j14'],
      ['j21', 'j22', 'j23', 'j24'],
      ['j31', 'j32', 'j33', 'j34']
    ];

    let maxVal = 1e-9;
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < n; c++)
        maxVal = Math.max(maxVal, Math.abs(J[r * n + c]));

    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < n; c++) {
        const el = $(ids[r][c]);
        if (!el) continue;
        const v = J[r * n + c];
        const norm = v / maxVal;
        el.textContent = v.toFixed(3);
        if (Math.abs(v) < 0.001) {
          el.style.background = 'rgba(80,80,100,0.3)';
          el.style.color = '#555580';
        } else if (norm > 0) {
          el.style.background = `rgba(32,${Math.round(80 + norm * 120)},60,0.25)`;
          el.style.color = `rgb(50,${Math.round(160 + norm * 95)},80)`;
        } else {
          el.style.background = `rgba(${Math.round(80 + (-norm) * 120)},30,30,0.25)`;
          el.style.color = `rgb(${Math.round(160 + (-norm) * 95)},50,50)`;
        }
      }
    }
  }

  /* ═══════════ Scene + HUD Update ═══════════ */
  _updateScene(pTgt3, ikResult) {
    const result = this.sm.updateScene(this.q, pTgt3);
    const pos = result.position;

    // HUD joint values
    $('h1').textContent = RAD(this.q[0]).toFixed(1) + '°';
    $('h2').textContent = RAD(this.q[1]).toFixed(1) + '°';
    $('h3').textContent = (this.q[2] * 1000).toFixed(1) + ' mm';
    $('h4').textContent = RAD(this.q[3]).toFixed(1) + '°';
    $('hHead').textContent = `(${pos[0].toFixed(3)}, ${pos[1].toFixed(3)}, ${pos[2].toFixed(3)}) m`;

    // EE readout cards
    $('ox').textContent = pos[0].toFixed(4) + ' m';
    $('oy').textContent = pos[1].toFixed(4) + ' m';
    $('oz').textContent = pos[2].toFixed(4) + ' m';

    // FK pane EE cards
    const fkox = $('fk-ox'), fkoy = $('fk-oy'), fkoz = $('fk-oz');
    if (fkox) fkox.textContent = pos[0].toFixed(4) + ' m';
    if (fkoy) fkoy.textContent = pos[1].toFixed(4) + ' m';
    if (fkoz) fkoz.textContent = pos[2].toFixed(4) + ' m';

    // Update Jacobian if on that tab
    if (this._tab === 'jac') this._updateJacobian();

    // ── Perception camera viewport render (throttled) ──
    // FIX 2: reuse result.position instead of calling fk() again
    if (this._cameraPanel && this._cameraPanel.isVisible) {
      this._camRenderSkip++;
      if (this._camRenderSkip % 3 === 0) {
        this.sm.renderPerceptionView(this._cameraPanel.viewportCanvas);
      }
      this._cameraPanel.updateProjection(pos);
    }

    // ── Broadcast to Math Dashboard ──
    const jac = jacobian(this.q, SCARA_DH_CONFIG);
    const n = jac.cols;
    // Convert flat J to 2D array for dashboard
    const J2d = [];
    for (let r = 0; r < 3; r++) {
      const row = [];
      for (let c = 0; c < n; c++) row.push(jac.J[r * n + c]);
      J2d.push(row);
    }
    // Compute manipulability
    const JJt = new Float64Array(9);
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let k = 0; k < n; k++) sum += jac.J[i * n + k] * jac.J[j * n + k];
        JJt[i * 3 + j] = sum;
      }
    const det = JJt[0] * (JJt[4] * JJt[8] - JJt[5] * JJt[7])
              - JJt[1] * (JJt[3] * JJt[8] - JJt[5] * JJt[6])
              + JJt[2] * (JJt[3] * JJt[7] - JJt[4] * JJt[6]);
    const mu = Math.sqrt(Math.max(0, det));
    this._mathChannel.postMessage({
      robot: 'scara',
      q: [...this.q],
      ee: [...pos],
      jacobian: J2d,
      mu,
      detJ: det,
      reach: Math.sqrt(pos[0] ** 2 + pos[1] ** 2),
      error: ikResult?.error ?? 0,
      converged: ikResult?.converged ?? true,
      iterations: ikResult?.iterations ?? 0,
      status: ikResult ? (ikResult.converged ? 'converged' : 'failed') : 'fk'
    });
  }

  /* ═══════════ Master Update (initial call) ═══════════ */
  update() {
    if (this._tab === 'fk') {
      this._onFKSlider();
    } else if (this._tab === 'ik') {
      this._onIKSlider();
    } else if (this._tab === 'jac') {
      this._onJacSlider();
    }
  }

  /* ═══════════ Pick & Place Task ═══════════ */
  _toggleTask() {
    const btn = $('taskBtn');
    const mode = $('taskMode')?.value || 'auto';

    if (this._pnp.running) {
      // Stop
      this._pnp.stop();
      cancelAnimationFrame(this._taskRAF);
      this._taskRAF = null;
      btn.textContent = '▶  Start Pick & Place';
      btn.classList.remove('stop');
      $('task-state').textContent = 'IDLE';
      $('task-state').style.color = '#8080a8';
      this._interactiveClickPhase = 0;
      const hint = $('clickHint');
      if (hint) hint.style.display = 'none';
    } else {
      // Start
      this.sm.resetTrail();
      this._pnp.setInitialQ([...this.q]);


      if (mode === 'interactive') {
        this._pnp.startInteractive();
        this._interactiveClickPhase = 1; // waiting for pick click
        const hint = $('clickHint');
        if (hint) hint.style.display = 'block';
      } else {
        this._pnp.start();
        this._interactiveClickPhase = 0;
        const hint = $('clickHint');
        if (hint) hint.style.display = 'none';
      }

      btn.textContent = '⏹  Stop Pick & Place';
      btn.classList.add('stop');
      this._lastTime = performance.now();
      this._runTask();
    }
  }

  _runTask() {
    if (!this._pnp.running) return;
    const now = performance.now();
    const dt = Math.min((now - this._lastTime) / 1000, 0.05);
    this._lastTime = now;

    this._pnp.update(dt);
    this._taskRAF = requestAnimationFrame(() => this._runTask());
  }

  _onTaskStateChange(state, meta) {
    const el = $('task-state');
    if (el) {
      el.textContent = meta.label;
      el.style.color = meta.color;
    }
    $('task-cycle').textContent = this._pnp.cycleCount;

    // Update waypoint display
    const pick = this._pnp.pickPos;
    const drop = this._pnp.dropPos;
    $('task-pick').textContent = `(${pick[0].toFixed(2)}, ${pick[1].toFixed(2)}, ${pick[2].toFixed(2)})`;
    $('task-drop').textContent = `(${drop[0].toFixed(2)}, ${drop[1].toFixed(2)}, ${drop[2].toFixed(2)})`;

    // Interactive mode: update click phase
    if (this._pnp.interactive) {
      if (state === STATE.IDLE) {
        this._interactiveClickPhase = 1; // waiting for pick
      } else if (state === STATE.HOLDING) {
        this._interactiveClickPhase = 2; // waiting for drop
      } else {
        this._interactiveClickPhase = 0;
      }
    }
  }

  _onTaskUpdate(info) {
    this.q = info.q;

    // DH(x,y,z) → Three(x, z, -y)
    const pTgt3 = new THREE.Vector3(info.target[0], info.target[2], -info.target[1]);
    this._updateScene(pTgt3, { error: info.error, converged: info.converged, iterations: 0 });

    // Trail
    if (this.sm.eeWorldPos) this.sm.addTrailPoint(this.sm.eeWorldPos);

    // Task cards
    $('task-progress').textContent = (info.progress * 100).toFixed(0) + '%';
    $('task-err').textContent = info.error.toExponential(2) + ' m';
    $('task-err').style.color = info.converged ? 'var(--grn)' : 'var(--red2)';
    $('task-target').textContent =
      `(${info.target[0].toFixed(3)}, ${info.target[1].toFixed(3)}, ${info.target[2].toFixed(3)})`;
    $('task-cycle').textContent = this._pnp.cycleCount;

    // Progress bar
    $('taskBar').style.width = (info.progress * 100) + '%';
    $('taskBarLabel').textContent = STATE_META[info.state]?.label || info.state;

    // Sync FK sliders
    $('fk-q1').value = RAD(this.q[0]).toFixed(0);
    $('fk-q2').value = RAD(this.q[1]).toFixed(0);
    $('fk-q3').value = (this.q[2] * 1000).toFixed(0);
    $('fk-q4').value = RAD(this.q[3]).toFixed(0);
    $('fk-q1v').textContent = RAD(this.q[0]).toFixed(0);
    $('fk-q2v').textContent = RAD(this.q[1]).toFixed(0);
    $('fk-q3v').textContent = (this.q[2] * 1000).toFixed(0);
    $('fk-q4v').textContent = RAD(this.q[3]).toFixed(0);

    // HUD
    $('hStat').textContent = info.state;
    $('hStat').className = 'ok';
    $('hErr').textContent = info.error.toExponential(2) + ' m';


  }

  /* ═══════════ Raycaster ═══════════ */
  _onPointerDown(e) {
    if (e.button !== 0) return;
    const rect = this.sm.renderer.domElement.getBoundingClientRect();
    this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this._raycaster.setFromCamera(this._mouse, this.sm.camera);

    // Interactive click-to-pick/place
    if (this._interactiveClickPhase > 0) {
      const pt = new THREE.Vector3();
      if (this._raycaster.ray.intersectPlane(this.sm.dragPlane, pt)) {
        // Three.js → DH: x→x, -z→y
        const dhX = clamp(pt.x, -0.55, 0.55);
        const dhY = clamp(-pt.z, -0.55, 0.55);

        if (this._interactiveClickPhase === 1) {
          // Click 1: set payload position and start pick
          this._pnp.setInteractivePick(dhX, dhY);
        } else if (this._interactiveClickPhase === 2) {
          // Click 2: set drop zone and start transfer
          this._pnp.setInteractiveDrop(dhX, dhY);
        }
        return; // consume click, don't drag
      }
    }

    // ── Check link meshes for click-to-resize ──
    const linkHits = this._raycaster.intersectObjects(this.sm.linkMeshArray, false);
    if (linkHits.length > 0) {
      const hitMesh = linkHits[0].object;
      const idx = this.sm.linkMeshes.findIndex(e => e.mesh === hitMesh);
      if (idx >= 0) {
        this.showLinkCard(idx);
        return; // consume click
      }
    }

    // Normal drag behavior — also check for draggable scene objects
    const draggables = [this.sm.payload, this.sm.dropZoneMesh].filter(Boolean);
    const hits = this._raycaster.intersectObjects(draggables, false);

    if (hits.length > 0 && !this._pnp.running) {
      // Attach TransformControls to the clicked object
      const hitMesh = hits[0].object;
      this.sm.transformControls.attach(hitMesh);
      return; // consume click
    }

    // If clicking on empty space, detach TransformControls
    // FIX 4: use .dragging flag instead of unreliable getHelper() raycast
    if (this.sm.transformControls.object && !this.sm.transformControls.dragging) {
      this.sm.transformControls.detach();
    }

    // Legacy: drag target sphere for IK
    if (this._raycaster.intersectObject(this.sm.targetSphere).length > 0) {
      this._isDragging = true;
      this.sm.controls.enabled = false;
      document.body.style.cursor = 'grabbing';
    }
  }

  _onPointerMove(e) {
    if (!this._isDragging) return;
    const rect = this.sm.renderer.domElement.getBoundingClientRect();
    this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this._raycaster.setFromCamera(this._mouse, this.sm.camera);
    const pt = new THREE.Vector3();
    if (this._raycaster.ray.intersectPlane(this.sm.dragPlane, pt)) {
      const x = clamp(pt.x, -0.55, 0.55);
      const y = clamp(-pt.z, -0.55, 0.55);

      if (this._tab === 'jac') {
        $('jxd').value = x; $('jyd').value = y;
        this._onJacSlider();
      } else {
        if (this._tab !== 'ik') this.setTab('ik');
        $('ik-xt').value = x; $('ik-yt').value = y;
        this._onIKSlider();
      }
    }
  }

  _onPointerUp() {
    // FIX 3: only re-enable OrbitControls if the legacy IK-sphere drag was
    // responsible for disabling it — TransformControls manages its own state
    // via the 'dragging-changed' event on the scene manager.
    if (this._isDragging) {
      this.sm.controls.enabled = true;
      document.body.style.cursor = 'default';
    }
    this._isDragging = false;
  }

  /* ═══════════ Scene Object Drag Sync ═══════════ */

  /**
   * Called when a draggable scene object (payload or dropZone) is moved
   * via TransformControls. Syncs the new position back to:
   *  - The PickAndPlace state machine targets
   *  - The visual glow/ring markers
   */
  _onSceneObjectDragged(mesh) {
    // Three.js coords: x, y(up), z
    // DH coords:       x, -z, y
    const threeX = mesh.position.x;
    const threeZ = mesh.position.z;
    const dhX = threeX;
    const dhY = -threeZ;

    if (mesh.userData.role === 'payload') {
      // FIX 5: use public setPayloadPosition instead of sm._payloadGlow (private)
      // setPayloadPosition already syncs the glow ring — no duplicate call needed
      this._pnp.setPickPos(dhX, dhY, this._pnp.pickPos[2]);
      this.sm.setPayloadPosition(threeX, 0.013, threeZ);
    } else if (mesh.userData.role === 'dropZone') {
      // Update drop position in the state machine
      this._pnp.setDropPos(dhX, dhY, this._pnp.dropPos[2]);
      // Sync ring and dot visuals
      this.sm.setDropZonePosition(threeX, threeZ);
      // Override mesh position back (setDropZonePosition already does it)
    }

    // Update task cards if visible
    const pickEl = $('task-pick');
    const dropEl = $('task-drop');
    if (pickEl) {
      const p = this._pnp.pickPos;
      pickEl.textContent = `(${p[0].toFixed(2)}, ${p[1].toFixed(2)}, ${p[2].toFixed(2)})`;
    }
    if (dropEl) {
      const d = this._pnp.dropPos;
      dropEl.textContent = `(${d[0].toFixed(2)}, ${d[1].toFixed(2)}, ${d[2].toFixed(2)})`;
    }
  }



  /* ═══════════ Perception Camera ═══════════ */
  _toggleCamera() {
    if (!this._cameraPanel) {
      this._cameraPanel = new CameraPanel({
        mountRoot: document.getElementById('cw'),
        startX: 20, startY: 420,
        getEEPos: () => {
          const { position } = fk(this.q, SCARA_DH_CONFIG);
          return position;
        },
        onChange: (params) => {
          this.sm.updatePerceptionCamera(
            params.tx, params.ty, params.tz,
            params.rx, params.ry, params.rz,
            params.fov
          );
          this.sm.setPerceptionActive(true);
        },
      });
    }

    this._cameraPanel.toggle();

    // Sync perception camera visibility with panel
    if (this._cameraPanel.isVisible) {
      this.sm.setPerceptionActive(true);
      // Trigger initial camera update
      const p = this._cameraPanel.params;
      this.sm.updatePerceptionCamera(
        p.tx, p.ty, p.tz,
        p.rx * Math.PI / 180, p.ry * Math.PI / 180, p.rz * Math.PI / 180,
        p.fov
      );
    } else {
      this.sm.setPerceptionActive(false);
    }
  }

  /* ═══════════ Link Card ═══════════ */

  _bindLinkCard() {
    if (!this._linkSlider || !this._linkCard) return;

    // Slider drag → resize link in real time
    this._linkSlider.addEventListener('input', () => {
      if (this._selectedLink < 0) return;
      const newLen = parseFloat(this._linkSlider.value);
      this.sm.resizeLink(this._selectedLink, newLen);
      this._linkValue.textContent = newLen.toFixed(3) + ' m';
      // Re-run FK to update the 3D arm
      this._updateScene(null);
    });

    // Close button
    const closeBtn = $('linkCardClose');
    if (closeBtn) closeBtn.addEventListener('click', () => this.hideLinkCard());

    // Reset to default
    const resetBtn = $('linkCardReset');
    if (resetBtn) resetBtn.addEventListener('click', () => {
      if (this._selectedLink < 0) return;
      const entry = this.sm.linkMeshes[this._selectedLink];
      const defLen = entry.defaultLen;
      this._linkSlider.value = defLen;
      this.sm.resizeLink(this._selectedLink, defLen);
      this._linkValue.textContent = defLen.toFixed(3) + ' m';
      this._updateScene(null);
    });
  }

  /**
   * Show the link detail card for a given link index.
   * @param {number} index - Link index
   */
  showLinkCard(index) {
    if (!this._linkCard) return;
    const entry = this.sm.linkMeshes[index];
    if (!entry) return;

    this._selectedLink = index;
    this.sm.highlightLink(index);

    // Populate the card
    this._linkTitle.textContent = entry.label;
    const currentLen = SCARA_DH_CONFIG[entry.dhIndex][entry.dhKey];
    this._linkSlider.min = entry.min;
    this._linkSlider.max = entry.max;
    this._linkSlider.step = 0.005;
    this._linkSlider.value = currentLen;
    this._linkValue.textContent = currentLen.toFixed(3) + ' m';
    this._linkMin.textContent = entry.min.toFixed(2);
    this._linkMax.textContent = entry.max.toFixed(2);
    this._linkDefault.textContent = 'default: ' + entry.defaultLen.toFixed(2) + ' m';

    // Show with animation
    this._linkCard.classList.add('visible');
  }

  /** Hide the link detail card and clear the highlight */
  hideLinkCard() {
    if (!this._linkCard) return;
    this._linkCard.classList.remove('visible');
    this.sm.clearHighlight();
    this._selectedLink = -1;
  }
}
