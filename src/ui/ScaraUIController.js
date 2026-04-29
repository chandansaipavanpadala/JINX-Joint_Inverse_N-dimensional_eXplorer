/**
 * ScaraUIController.js — DOM event wiring and HUD updates for the SCARA arm.
 * Bridges ScaraSceneManager (3D) with HTML sliders/buttons/text.
 * Uses the generalized N-DOF IK (DLS) solver.
 */
import * as THREE from 'three';
import { ik_dls, fk, jacobian, SCARA_DH_CONFIG } from '../math/KinematicsNDOF.js';
import PickAndPlaceStateMachine, { STATE, STATE_META } from '../logic/PickAndPlace.js';

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

    this._bindEvents();
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

    // Target sphere position: DH(x,y,z) → Three(x, z, -y)
    const pTgt3 = new THREE.Vector3(xt, zt, -yt);
    this._updateScene(pTgt3);
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
    this._updateScene(pTgt3);
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
  _updateScene(pTgt3) {
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
    this._updateScene(pTgt3);

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

    // Normal drag behavior
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
    this._isDragging = false;
    this.sm.controls.enabled = true;
    document.body.style.cursor = 'default';
  }
}
