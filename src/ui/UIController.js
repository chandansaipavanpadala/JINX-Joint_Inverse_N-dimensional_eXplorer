/**
 * UIController.js — DOM event wiring, simulation loop, and HUD updates
 * Bridges the SceneManager (3D) with HTML sliders/buttons/text.
 */
import * as THREE from 'three';
import {
  ikMat, fkMat, jacMat, det3, clamp, RAD, V3, DEG,
  T2MIN, T2MAX, T3MIN, T3MAX, linkLengths
} from '../math/Kinematics.js';
import { trapProfile } from '../math/Trajectory.js';
import { projectPixelsTo3D, transformCameraToRobot } from '../math/CameraModel.js';

const $ = id => document.getElementById(id);

export default class UIController {
  constructor(sceneManager) {
    this.sm = sceneManager;
    this._tab = 'ik';
    // Simulation state
    this.simRunning = false;
    this.simT = 0;
    this.simRAF = null;
    this.simN = 0;
    this.qCurrent = [0, DEG(45), DEG(80)];
    this.qTarget = [0, DEG(45), DEG(80)];
    this.trapT = 0;
    this.trapTTotal = 0.5;
    this.trailBuf = [];
    // Raycaster
    this._raycaster = new THREE.Raycaster();
    this._mouse = new THREE.Vector2();
    this._isDragging = false;
    // Link selection
    this._selectedLink = -1;
    this._linkCard = $('linkCard');
    this._linkSlider = $('linkCardSlider');
    this._linkTitle = $('linkCardTitle');
    this._linkValue = $('linkCardValue');
    this._linkMin = $('linkCardMin');
    this._linkMax = $('linkCardMax');
    this._linkDefault = $('linkCardDefault');
    // BroadcastChannel for math dashboard sync
    this._mathChannel = new BroadcastChannel('jinx_math_sync');
    // Command channel — receive FK/IK commands from dashboard
    this._cmdChannel = new BroadcastChannel('jinx_math_cmd');
    this._cmdChannel.onmessage = (e) => {
      const d = e.data;
      if (d.robot !== 'rrr') return;
      if (d.type === 'fk' && d.q) {
        // d.q is in radians — set the IK target sliders to match
        const RAD2DEG = 180 / Math.PI;
        $('xd').value = 0; $('yd').value = 0;
        // Direct scene update with the received joint angles
        const fv = fkMat(d.q[0], d.q[1], d.q[2]);
        const pTgt3 = V3(fv.x, 0.025, -fv.y);
        this.sm.updateScene(d.q[0], d.q[1], d.q[2], pTgt3);
        // Update HUD
        $('h1').textContent = (d.q[0] * RAD2DEG).toFixed(1) + '°';
        $('h2').textContent = (d.q[1] * RAD2DEG).toFixed(1) + '°';
        $('h3').textContent = (d.q[2] * RAD2DEG).toFixed(1) + '°';
        $('hHead').textContent = `(${fv.x.toFixed(3)}, ${fv.y.toFixed(3)}, ${fv.z.toFixed(3)}) m`;
      } else if (d.type === 'ik' && d.target) {
        $('xd').value = d.target[0];
        $('yd').value = d.target[1];
        this.update();
      }
    };
    this._bindEvents();
    this._bindLinkCard();
  }

  /* ═══════════ Event Wiring ═══════════ */
  _bindEvents() {
    // Tabs
    document.querySelectorAll('#tabBar .tab').forEach(btn => {
      btn.addEventListener('click', () => this.setTab(btn.dataset.tab));
    });
    // IK / Jacobian sliders
    ['xd', 'yd', 'jxd', 'jyd', 'dx', 'dy', 'dz'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => this.update());
    });
    ['elbow'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('change', () => this.update());
    });
    // Camera sliders
    ['cfx', 'cfy', 'ccx', 'ccy', 'cu', 'cv_', 'czw', 'ctx', 'cty', 'ctz'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => {
        $(id + 'v').textContent = parseFloat(el.value).toFixed(2);
        this.computeCameraIK();
      });
    });
    // Trap sliders
    ['vmax', 'amax'].forEach(id => {
      const el = $(id);
      if (el) el.addEventListener('input', () => this.drawTrapProfile());
    });
    // Sim button
    $('sbtn').addEventListener('click', () => this.toggleSim());
    // Math dashboard
    const mathBtn = $('btn-math-panel');
    if (mathBtn) mathBtn.addEventListener('click', () => window.open('./math-dashboard.html?robot=rrr', '_blank'));
    // Camera apply
    $('camApplyBtn').addEventListener('click', () => this.applyCameraIK());
    // Raycaster
    const dom = this.sm.renderer.domElement;
    dom.addEventListener('pointerdown', e => this._onPointerDown(e));
    window.addEventListener('pointermove', e => this._onPointerMove(e));
    window.addEventListener('pointerup', () => this._onPointerUp());
    // Reset
    const resetBtn = $('resetBtn');
    if (resetBtn) resetBtn.addEventListener('click', () => this._resetPose());
  }

  /* ═══════════ Reset Pose ═══════════ */
  _resetPose() {
    // Stop simulation
    if (this.simRunning) this.toggleSim();
    // Reset to default
    const defaultQ = [0, DEG(45), DEG(80)];
    this.qCurrent = [...defaultQ];
    this.qTarget = [...defaultQ];
    // Reset sliders
    $('xd').value = 0.20; $('yd').value = 0.42;
    $('jxd').value = 0.20; $('jyd').value = 0.42;
    // Reset trail
    this.trailBuf = [];
    this.sm.resetTrail();
    this.update();
  }

  /* ═══════════ Tabs ═══════════ */
  setTab(t) {
    this._tab = t;
    ['ik', 'jac', 'sim', 'cam'].forEach(id => {
      $('pane-' + id).classList.toggle('on', id === t);
    });
    document.querySelectorAll('#tabBar .tab').forEach(el => {
      el.classList.toggle('active', el.dataset.tab === t);
    });
    if (t === 'cam') this.computeCameraIK();
    if (t === 'sim' || t === 'cam') this.drawTrapProfile();
    this.update();
  }

  /* ═══════════ Master Update ═══════════ */
  update() {
    const tab = this._tab;
    const dx = +$('dx').value, dy = +$('dy').value, dz = +$('dz').value;
    $('dxv').textContent = dx.toFixed(2);
    $('dyv').textContent = dy.toFixed(2);
    $('dzv').textContent = dz.toFixed(2);

    let xd, yd;
    if (tab === 'jac') {
      xd = +$('jxd').value; yd = +$('jyd').value;
      $('jxdv').textContent = xd.toFixed(2);
      $('jydv').textContent = yd.toFixed(2);
    } else {
      xd = +$('xd').value; yd = +$('yd').value;
      $('xdv').textContent = xd.toFixed(2);
      $('ydv').textContent = yd.toFixed(2);
    }

    const eSign = (tab === 'ik') ? +$('elbow').value : 1;
    const zhand = 0.025;
    const pdx = xd + dx, pdy = yd + dy, pdz = zhand + dz;
    const sol = ikMat(pdx, pdy, pdz, eSign);

    $('ikAlert').classList.toggle('on', !sol && tab === 'ik');
    if (!sol) {
      $('hStat').textContent = 'Unreachable';
      $('hStat').className = 'bad';
      return;
    }

    const limOk = sol.t2 >= T2MIN - 0.01 && sol.t2 <= T2MAX + 0.01
      && sol.t3 >= T3MIN - 0.01 && sol.t3 <= T3MAX + 0.01;
    const t1 = sol.t1;
    const t2 = clamp(sol.t2, T2MIN, T2MAX);
    const t3 = clamp(sol.t3, T3MIN, T3MAX);
    $('limAlert').classList.toggle('on', !limOk && tab === 'ik');
    $('hStat').textContent = limOk ? '✓ Valid' : '⚠ Limit';
    $('hStat').className = limOk ? 'ok' : 'bad';

    const fv = fkMat(t1, t2, t3);
    const err = Math.hypot(fv.x - pdx, fv.y - pdy, fv.z - pdz);
    const J = jacMat(t1, t2, t3);
    const mu = Math.abs(det3(J));

    if (tab === 'ik') {
      $('ik-t1').textContent = RAD(t1).toFixed(1) + '°';
      $('ik-t2').textContent = RAD(sol.t2).toFixed(1) + (limOk ? '' : '⚠') + '°';
      $('ik-t3').textContent = RAD(sol.t3).toFixed(1) + (limOk ? '' : '⚠') + '°';
      $('ik-err').textContent = err.toExponential(2) + ' m';
      $('ik-r').textContent = fv.r.toFixed(4) + ' m';
      $('ik-z').textContent = fv.z.toFixed(4) + ' m';
      $('ik-mu').textContent = mu.toFixed(5);
    }

    const pTgt3 = V3(xd, zhand, -yd);
    const f = this.sm.updateScene(t1, t2, t3, pTgt3);

    // HUD
    $('h1').textContent = RAD(t1).toFixed(1) + '°';
    $('h2').textContent = RAD(t2).toFixed(1) + '°';
    $('h3').textContent = RAD(t3).toFixed(1) + '°';
    $('hHead').textContent = `(${f.x.toFixed(3)}, ${f.y.toFixed(3)}, ${f.z.toFixed(3)}) m`;
    $('hMu').textContent = mu.toFixed(5);
    // EE cards
    $('ox').textContent = f.x.toFixed(4) + ' m';
    $('oy').textContent = f.y.toFixed(4) + ' m';
    $('oz').textContent = f.z.toFixed(4) + ' m';
    $('or_').textContent = f.r.toFixed(4) + ' m';

    this._updateJacobian(t1, t2, t3, f, J, mu);

    // ── Broadcast to Math Dashboard ──
    this._mathChannel.postMessage({
      robot: 'rrr',
      q: [t1, t2, t3],
      ee: [f.x, f.y, f.z],
      jacobian: J,
      mu,
      detJ: det3(J),
      reach: f.r,
      error: err,
      converged: !!sol,
      status: limOk ? 'valid' : 'limit'
    });
  }

  /* ═══════════ Jacobian Display ═══════════ */
  _updateJacobian(t1, t2, t3, f, J, mu) {
    const ids = [['j11', 'j12', 'j13'], ['j21', 'j22', 'j23'], ['j31', 'j32', 'j33']];
    const maxVal = Math.max(...J.flat().map(Math.abs), 1e-9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      const el = $(ids[i][j]);
      const v = J[i][j], norm = v / maxVal;
      el.textContent = v.toFixed(3);
      if (Math.abs(v) < 0.001) {
        el.style.background = 'rgba(80,80,100,0.3)'; el.style.color = '#555580';
      } else if (norm > 0) {
        el.style.background = `rgba(32,${Math.round(80 + norm * 120)},60,0.25)`;
        el.style.color = `rgb(50,${Math.round(160 + norm * 95)},80)`;
      } else {
        el.style.background = `rgba(${Math.round(80 + (-norm) * 120)},30,30,0.25)`;
        el.style.color = `rgb(${Math.round(160 + (-norm) * 95)},50,50)`;
      }
    }
    const d = det3(J);
    $('detJ').textContent = d.toFixed(4);
    $('jmu').textContent = mu.toFixed(5);
    $('sinT3').textContent = Math.sin(t3).toFixed(4);
    $('jreach').textContent = f.r.toFixed(4) + ' m';
    const pct = Math.min(mu / 0.04 * 100, 100);
    $('muBar').style.width = pct + '%';
    const ramp = Math.round(pct * 2.55);
    $('muBar').style.background = `rgb(${255 - ramp},${ramp},60)`;
    $('muLabel').textContent = 'μ = ' + mu.toFixed(5);
    $('singWarn').style.display = Math.abs(Math.sin(t3)) < 0.10 ? 'block' : 'none';
    $('singWarn2').style.display = Math.abs(f.r) < 0.04 ? 'block' : 'none';
  }

  /* ═══════════ Camera Pinhole ═══════════ */
  computeCameraIK() {
    const fx = +$('cfx').value, fy = +$('cfy').value;
    const cx = +$('ccx').value, cy = +$('ccy').value;
    const u = +$('cu').value, v = +$('cv_').value, Zw = +$('czw').value;
    const tx = +$('ctx').value, ty = +$('cty').value, tz = +$('ctz').value;

    const { Xw, Yw } = projectPixelsTo3D(u, v, Zw, fx, fy, cx, cy);
    const { xr, yr, zr } = transformCameraToRobot(Xw, Yw, Zw, tx, ty, tz);

    $('camXw').textContent = Xw.toFixed(4) + ' m';
    $('camYw').textContent = Yw.toFixed(4) + ' m';
    $('camZw').textContent = Zw.toFixed(4) + ' m';
    $('camRx').textContent = xr.toFixed(4) + ' m';
    $('camRy').textContent = yr.toFixed(4) + ' m';
    $('camRz').textContent = zr.toFixed(4) + ' m';

    this.sm.updatePovCamera(tx, ty, -tz);

    const dxv = +$('dx').value, dyv = +$('dy').value, dzv = +$('dz').value;
    const sol = ikMat(xr + dxv, yr + dyv, zr + dzv, 1);

    $('camAlert').classList.toggle('on', !sol);
    if (sol) {
      $('camIK').textContent =
        `θ₁=${RAD(sol.t1).toFixed(1)}° θ₂=${RAD(sol.t2).toFixed(1)}° θ₃=${RAD(sol.t3).toFixed(1)}°`;
    } else {
      $('camIK').textContent = 'Out of reach';
    }
    return { sol, xr, yr, zr };
  }

  applyCameraIK() {
    const { sol, xr, yr, zr } = this.computeCameraIK();
    if (!sol) return;
    const t1 = sol.t1, t2 = clamp(sol.t2, T2MIN, T2MAX), t3 = clamp(sol.t3, T3MIN, T3MAX);
    const pTgt = V3(xr, zr, -yr);
    this.sm.updateScene(t1, t2, t3, pTgt);
    $('hStat').textContent = 'Camera IK';
    $('hStat').className = 'ok';
  }

  /* ═══════════ Trap Profile Drawing ═══════════ */
  drawTrapProfile() {
    const cvs = $('trapCvs'); if (!cvs) return;
    const w = cvs.clientWidth || 280, h = cvs.clientHeight || 68;
    cvs.width = w; cvs.height = h;
    const ctx = cvs.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    const vmax_ = +$('vmax').value || 1.2, amax_ = +$('amax').value || 2.0;
    const T = 1.0, N = 100;
    // Grid
    ctx.strokeStyle = 'rgba(80,80,120,0.25)'; ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) { ctx.beginPath(); ctx.moveTo(i * w / 4, 0); ctx.lineTo(i * w / 4, h); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
    // s(t)
    ctx.beginPath(); ctx.strokeStyle = '#20d080'; ctx.lineWidth = 1.8;
    for (let i = 0; i <= N; i++) { const t = i / N * T; const { s } = trapProfile(t, T, vmax_, amax_); const px = i / N * w, py = h - s * (h - 8) - 4; i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py); }
    ctx.stroke();
    // sdot
    ctx.beginPath(); ctx.strokeStyle = '#c8a800'; ctx.lineWidth = 1.4; ctx.setLineDash([3, 2]);
    for (let i = 0; i <= N; i++) { const t = i / N * T; const { sdot } = trapProfile(t, T, vmax_, amax_); const px = i / N * w, py = h / 2 - sdot / vmax_ * (h / 2 - 8); i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py); }
    ctx.stroke(); ctx.setLineDash([]);
    // sddot
    ctx.beginPath(); ctx.strokeStyle = 'rgba(255,80,80,0.7)'; ctx.lineWidth = 1.2; ctx.setLineDash([2, 3]);
    for (let i = 0; i <= N; i++) { const t = i / N * T; const { sddot } = trapProfile(t, T, vmax_, amax_); const norm = sddot / amax_; const px = i / N * w, py = h / 2 - norm * (h / 2 - 8); i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py); }
    ctx.stroke(); ctx.setLineDash([]);
    // Legend
    ctx.font = '9px Inter';
    ctx.fillStyle = '#20d080'; ctx.fillText('s(t)', 4, 12);
    ctx.fillStyle = '#c8a800'; ctx.fillText('ṡ(t)', 28, 12);
    ctx.fillStyle = 'rgba(255,80,80,0.9)'; ctx.fillText('s̈(t)', 52, 12);
    // t1/t2 lines
    const t1_ = Math.min(vmax_ / amax_, T / 2), t2_ = T - t1_;
    ctx.strokeStyle = 'rgba(200,168,0,0.35)'; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    [t1_, t2_].forEach(tt => { ctx.beginPath(); ctx.moveTo(tt / T * w, 0); ctx.lineTo(tt / T * w, h); ctx.stroke(); });
    ctx.setLineDash([]);
  }

  /* ═══════════ Trajectory Drawing ═══════════ */
  drawTraj() {
    const cvs = $('trajCvs'); if (!cvs) return;
    const w = cvs.clientWidth, h = cvs.clientHeight || 80;
    cvs.width = w; cvs.height = h;
    const ctx = cvs.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    if (this.trailBuf.length < 2) return;
    const sx = x => (x + 0.5) / 1.1 * w, sy = y => h / 2 - (y / 0.4) * (h * 0.4);
    ctx.strokeStyle = 'rgba(80,80,120,0.3)'; ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) { ctx.beginPath(); ctx.moveTo(i * w / 4, 0); ctx.lineTo(i * w / 4, h); ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
    ctx.fillStyle = 'rgba(200,162,0,0.6)'; ctx.beginPath(); ctx.arc(sx(0), sy(0), 4, 0, Math.PI * 2); ctx.fill();
    for (let i = 1; i < this.trailBuf.length; i++) {
      const a = i / this.trailBuf.length;
      ctx.strokeStyle = `rgba(130,100,255,${a * 0.8})`; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(sx(this.trailBuf[i - 1].x), sy(this.trailBuf[i - 1].y));
      ctx.lineTo(sx(this.trailBuf[i].x), sy(this.trailBuf[i].y)); ctx.stroke();
    }
    if (this.trailBuf.length > 0) {
      const last = this.trailBuf[this.trailBuf.length - 1];
      ctx.fillStyle = '#20c8ff'; ctx.beginPath(); ctx.arc(sx(last.x), sy(last.y), 5, 0, Math.PI * 2); ctx.fill();
    }
    ctx.fillStyle = 'rgba(140,120,200,0.6)'; ctx.font = '9px Inter'; ctx.fillText('Top view (x-y)', 4, 10);
  }

  /* ═══════════ Simulation ═══════════ */
  toggleSim() {
    this.simRunning = !this.simRunning;
    const btn = $('sbtn');
    if (this.simRunning) {
      btn.textContent = '⏹  Stop Simulation'; btn.classList.add('stop');
      $('simStats').style.display = 'block';
      this.trailBuf = []; this.sm.resetTrail(); this.trapT = 0;
      this._runSim();
    } else {
      btn.textContent = '▶  Play Simulation'; btn.classList.remove('stop');
      cancelAnimationFrame(this.simRAF);
    }
  }

  _runSim() {
    if (!this.simRunning) return;
    const spd = +$('spd').value, hR_ = +$('hR').value;
    const vmax_ = +$('vmax').value || 1.2, amax_ = +$('amax').value || 2.0;
    $('spdv').textContent = spd.toFixed(1) + '×';
    $('hRv').textContent = hR_.toFixed(2) + ' m';
    $('vmaxv').textContent = vmax_.toFixed(2);
    $('amaxv').textContent = amax_.toFixed(2);

    const dt = 0.016 * spd;
    this.simT += dt; this.simN++;

    const dx = +$('dx').value, dy = +$('dy').value, dz = +$('dz').value;
    const xH = clamp(0.25 + hR_ * 0.5 * Math.cos(this.simT), 0.05, 0.45);
    const yH = clamp(0.08 * Math.sin(this.simT), -0.15, 0.15);
    const zH = 0.025;
    const pdx = xH + dx, pdy = yH + dy, pdz = zH + dz;

    const solNew = ikMat(pdx, pdy, pdz, 1);
    if (solNew) {
      const newQ = [solNew.t1, clamp(solNew.t2, T2MIN, T2MAX), clamp(solNew.t3, T3MIN, T3MAX)];
      const maxDelta = Math.max(...newQ.map((q, i) => Math.abs(q - this.qCurrent[i])));
      if (maxDelta > 0.005) {
        this.trapTTotal = Math.max(maxDelta / vmax_ * 1.5, 2 * Math.sqrt(maxDelta / amax_), 0.04);
        this.qTarget = [...newQ]; this.trapT = 0;
      }
      this.trapT += dt;
      const { s, sdot, sddot, phase } = trapProfile(this.trapT, this.trapTTotal, vmax_, amax_);
      const t1 = this.qCurrent[0] + s * (this.qTarget[0] - this.qCurrent[0]);
      const t2 = this.qCurrent[1] + s * (this.qTarget[1] - this.qCurrent[1]);
      const t3 = this.qCurrent[2] + s * (this.qTarget[2] - this.qCurrent[2]);
      if (this.trapT >= this.trapTTotal) { this.qCurrent = [...this.qTarget]; this.trapT = this.trapTTotal; }

      const pTgt3 = V3(xH, zH, -yH);
      this.sm.updateScene(t1, t2, t3, pTgt3);
      this.sm.addTrailPoint(pTgt3);

      this.trailBuf.push({ x: xH, y: yH });
      if (this.trailBuf.length > this.sm.TRAIL_N) this.trailBuf.shift();
      this.drawTraj();

      const fv = fkMat(t1, t2, t3);
      const err = Math.hypot(fv.x - pdx, fv.y - pdy, fv.z - pdz);
      const J = jacMat(t1, t2, t3), mu = Math.abs(det3(J));
      const lim = solNew.t2 < T2MIN - 0.01 || solNew.t2 > T2MAX + 0.01 || solNew.t3 < T3MIN - 0.01 || solNew.t3 > T3MAX + 0.01;
      $('simF').textContent = (this.simN % 9999) + (lim ? ' [LIM]' : '');
      $('simE').textContent = (err * 1e9).toExponential(2) + ' nm';
      $('simT2').textContent = RAD(t2).toFixed(1) + '°';
      $('simMu').textContent = mu.toFixed(4);
      $('simPhase').textContent = phase;
      $('simS').textContent = s.toFixed(4);
      $('simSdot').textContent = sdot.toFixed(3);
      $('simSddot').textContent = sddot.toFixed(2);
      this.drawTrapProfile();

      // ── Broadcast to Math Dashboard during simulation ──
      this._mathChannel.postMessage({
        robot: 'rrr',
        q: [t1, t2, t3],
        ee: [fv.x, fv.y, fv.z],
        jacobian: J,
        mu,
        detJ: det3(J),
        reach: fv.r,
        error: err,
        converged: true,
        status: 'simulating'
      });
    }
    this.simRAF = requestAnimationFrame(() => this._runSim());
  }

  /* ═══════════ Raycaster ═══════════ */
  _onPointerDown(e) {
    if (e.button !== 0) return;
    const rect = this.sm.renderer.domElement.getBoundingClientRect();
    this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this._raycaster.setFromCamera(this._mouse, this.sm.camera);

    // ── Check link meshes first ──
    const linkHits = this._raycaster.intersectObjects(this.sm.linkMeshArray, false);
    if (linkHits.length > 0) {
      const hitMesh = linkHits[0].object;
      const idx = this.sm.linkMeshes.findIndex(e => e.mesh === hitMesh);
      if (idx >= 0) {
        this.showLinkCard(idx);
        return; // Don't start dragging
      }
    }

    // ── Check target sphere (existing drag logic) ──
    if (this._raycaster.intersectObject(this.sm.targetSphere).length > 0) {
      this._isDragging = true;
      this.sm.controls.enabled = false;
      document.body.style.cursor = 'grabbing';
      return;
    }

    // ── Empty space click — dismiss card ──
    this.hideLinkCard();
  }
  _onPointerMove(e) {
    if (!this._isDragging) return;
    const rect = this.sm.renderer.domElement.getBoundingClientRect();
    this._mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this._mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this._raycaster.setFromCamera(this._mouse, this.sm.camera);
    const pt = new THREE.Vector3();
    if (this._raycaster.ray.intersectPlane(this.sm.dragPlane, pt)) {
      const x = clamp(pt.x, -0.15, 0.85), y = clamp(-pt.z, -0.30, 0.30);
      if (this._tab === 'jac') { $('jxd').value = x; $('jyd').value = y; }
      else { $('xd').value = x; $('yd').value = y; }
      this.update();
    }
  }
  _onPointerUp() {
    this._isDragging = false;
    this.sm.controls.enabled = true;
    document.body.style.cursor = 'default';
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
      this.update();
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
      this.update();
    });
  }

  /**
   * Show the link detail card for a given link index.
   * @param {number} index - Link index (0=L1, 1=L2, 2=L3)
   */
  showLinkCard(index) {
    if (!this._linkCard) return;
    const entry = this.sm.linkMeshes[index];
    if (!entry) return;

    this._selectedLink = index;
    this.sm.highlightLink(index);

    // Populate the card
    this._linkTitle.textContent = entry.label;
    const currentLen = linkLengths[entry.key];
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
