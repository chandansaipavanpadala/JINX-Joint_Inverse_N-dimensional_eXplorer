/**
 * PickAndPlace.js — Pick & Place State Machine for SCARA Arm
 *
 * Supports two modes:
 *   AUTO:        IDLE → PRE_GRASP → GRASP → LIFT → TRANSFER → PLACE → RETURN → (ping-pong)
 *   INTERACTIVE: IDLE → (wait click) → PRE_GRASP → GRASP → LIFT → HOLDING → (wait click) → TRANSFER → PLACE → RETURN → IDLE
 *
 * Pure logic module — no DOM manipulation.
 */

import { ik_dls, fk, SCARA_DH_CONFIG } from '../math/KinematicsNDOF.js';

/* ═══════════════════════════════════════════════════════════════
   State Definitions
   ═══════════════════════════════════════════════════════════════ */

export const STATE = Object.freeze({
  IDLE:       'IDLE',
  PRE_GRASP:  'PRE_GRASP',
  GRASP:      'GRASP',
  LIFT:       'LIFT',
  HOLDING:    'HOLDING',
  TRANSFER:   'TRANSFER',
  PLACE:      'PLACE',
  RETURN:     'RETURN',
});

export const STATE_META = Object.freeze({
  [STATE.IDLE]:       { label: '⏸  Idle',                              color: '#8080a8' },
  [STATE.PRE_GRASP]:  { label: '🎯  Pre-Grasp — Moving above payload', color: '#ffcc00' },
  [STATE.GRASP]:      { label: '⬇  Grasping — Descending',             color: '#ff9900' },
  [STATE.LIFT]:       { label: '⬆  Lifting — Ascending with payload',  color: '#00f07f' },
  [STATE.HOLDING]:    { label: '✋  Holding — Click table to set drop', color: '#00e5ff' },
  [STATE.TRANSFER]:   { label: '➡  Transferring — Moving to drop zone', color: '#00e5ff' },
  [STATE.PLACE]:      { label: '⬇  Placing — Descending to drop',      color: '#6c5ce7' },
  [STATE.RETURN]:     { label: '↩  Returning — Ascending & homing',    color: '#00e5ff' },
});

/* ═══════════════════════════════════════════════════════════════
   Default Waypoint Configuration
   ═══════════════════════════════════════════════════════════════ */

const DEFAULTS = {
  pickPos:    [0.40, 0.15, 0.35],
  dropPos:    [0.30, -0.20, 0.35],
  safeZ:      0.35,
  graspZ:     0.15,
  durations: {
    [STATE.PRE_GRASP]: 1.2,
    [STATE.GRASP]:     0.8,
    [STATE.LIFT]:      0.6,
    [STATE.TRANSFER]:  1.4,
    [STATE.PLACE]:     0.8,
    [STATE.RETURN]:    1.2,
  },
};


export default class PickAndPlaceStateMachine {

  constructor(sceneManager, options = {}) {
    this.sm = sceneManager;

    this._origPickPos = options.pickPos || [...DEFAULTS.pickPos];
    this._origDropPos = options.dropPos || [...DEFAULTS.dropPos];
    this._pickPos   = [...this._origPickPos];
    this._dropPos   = [...this._origDropPos];
    this._safeZ     = options.safeZ  || DEFAULTS.safeZ;
    this._graspZ    = options.graspZ || DEFAULTS.graspZ;
    this._durations = { ...DEFAULTS.durations, ...(options.durations || {}) };

    this._state     = STATE.IDLE;
    this._t         = 0;
    this._startPos  = [0, 0, 0];
    this._endPos    = [0, 0, 0];
    this._duration  = 1;
    this._q         = [0, 0, 0, 0];
    this._running   = false;
    this._cycleCount = 0;
    this._interactive = false;

    this.onStateChange = null;
    this.onUpdate      = null;
  }

  /* ═══════════ Public API ═══════════ */

  get state()        { return this._state; }
  get running()      { return this._running; }
  get q()            { return this._q; }
  get cycleCount()   { return this._cycleCount; }
  get pickPos()      { return this._pickPos; }
  get dropPos()      { return this._dropPos; }
  get interactive()  { return this._interactive; }

  setInitialQ(q) { this._q = [...q]; }
  setPickPos(x, y, z) { this._pickPos = [x, y, z]; }
  setDropPos(x, y, z) { this._dropPos = [x, y, z]; }

  /** Start auto loop */
  start() {
    if (this._running) return;
    this._interactive = false;
    this._running = true;
    this._cycleCount = 0;
    // Reset pick/drop to originals for clean start
    this._pickPos = [...this._origPickPos];
    this._dropPos = [...this._origDropPos];
    this._transitionTo(STATE.PRE_GRASP);
  }

  /** Start interactive mode — waits for click coordinates */
  startInteractive() {
    if (this._running) return;
    this._interactive = true;
    this._running = true;
    this._cycleCount = 0;
    this._state = STATE.IDLE;
    if (this.onStateChange) this.onStateChange(STATE.IDLE, STATE_META[STATE.IDLE]);
  }

  /** Provide pick coordinate (interactive mode click 1) */
  setInteractivePick(x, y) {
    if (!this._interactive || this._state !== STATE.IDLE) return;
    this._pickPos = [x, y, this._safeZ];
    // Move payload visual to that location (Three.js coords)
    this.sm.setPayloadPosition(x, 0.013, -y);
    this._transitionTo(STATE.PRE_GRASP);
  }

  /** Provide drop coordinate (interactive mode click 2) */
  setInteractiveDrop(x, y) {
    if (!this._interactive || this._state !== STATE.HOLDING) return;
    this._dropPos = [x, y, this._safeZ];
    // Move drop zone visual
    this.sm.setDropZonePosition(x, -y);
    this._transitionTo(STATE.TRANSFER);
  }

  stop() {
    this._running = false;
    if (this._state === STATE.LIFT || this._state === STATE.TRANSFER ||
        this._state === STATE.PLACE || this._state === STATE.HOLDING) {
      this.sm.detachPayloadToWorld();
    }
    this._state = STATE.IDLE;
    if (this.onStateChange) this.onStateChange(STATE.IDLE, STATE_META[STATE.IDLE]);
  }

  update(dt) {
    if (!this._running || this._state === STATE.IDLE || this._state === STATE.HOLDING) return null;

    this._t += dt;
    const progress = Math.min(this._t / this._duration, 1.0);

    // Cubic ease-in-out
    const s = progress < 0.5
      ? 4 * progress * progress * progress
      : 1 - Math.pow(-2 * progress + 2, 3) / 2;

    const target = [
      this._startPos[0] + s * (this._endPos[0] - this._startPos[0]),
      this._startPos[1] + s * (this._endPos[1] - this._startPos[1]),
      this._startPos[2] + s * (this._endPos[2] - this._startPos[2]),
    ];

    const ik = ik_dls(target, this._q, SCARA_DH_CONFIG, 80, 0.06);
    this._q = ik.q;

    const info = {
      q: this._q, target, state: this._state,
      progress, converged: ik.converged, error: ik.error,
    };

    if (this.onUpdate) this.onUpdate(info);
    if (progress >= 1.0) this._onStateComplete();

    return info;
  }

  /* ═══════════ State Transitions ═══════════ */

  _transitionTo(newState) {
    this._state = newState;
    this._t = 0;
    this._duration = this._durations[newState] || 1.0;

    const fkResult = fk(this._q, SCARA_DH_CONFIG);
    this._startPos = [...fkResult.position];

    switch (newState) {
      case STATE.PRE_GRASP:
        this._endPos = [this._pickPos[0], this._pickPos[1], this._safeZ];
        break;
      case STATE.GRASP:
        this._endPos = [this._pickPos[0], this._pickPos[1], this._graspZ];
        break;
      case STATE.LIFT:
        this._endPos = [this._pickPos[0], this._pickPos[1], this._safeZ];
        break;
      case STATE.TRANSFER:
        this._endPos = [this._dropPos[0], this._dropPos[1], this._safeZ];
        break;
      case STATE.PLACE:
        this._endPos = [this._dropPos[0], this._dropPos[1], this._graspZ];
        break;
      case STATE.RETURN:
        this._endPos = [0.30, 0.0, this._safeZ]; // neutral home position
        break;
    }

    if (this.onStateChange) this.onStateChange(newState, STATE_META[newState]);
  }

  _onStateComplete() {
    switch (this._state) {
      case STATE.PRE_GRASP:
        this._transitionTo(STATE.GRASP);
        break;

      case STATE.GRASP:
        this.sm.attachPayloadToEE();
        this._transitionTo(STATE.LIFT);
        break;

      case STATE.LIFT:
        if (this._interactive) {
          // Pause and wait for drop click
          this._state = STATE.HOLDING;
          if (this.onStateChange) this.onStateChange(STATE.HOLDING, STATE_META[STATE.HOLDING]);
        } else {
          this._transitionTo(STATE.TRANSFER);
        }
        break;

      case STATE.TRANSFER:
        this._transitionTo(STATE.PLACE);
        break;

      case STATE.PLACE:
        this.sm.detachPayloadToWorld();
        this._transitionTo(STATE.RETURN);
        break;

      case STATE.RETURN:
        this._cycleCount++;
        if (this._interactive) {
          // Back to IDLE, wait for next click
          this._state = STATE.IDLE;
          if (this.onStateChange) this.onStateChange(STATE.IDLE, STATE_META[STATE.IDLE]);
        } else {
          // Auto mode: swap pick/drop for ping-pong, reset Z heights
          const tmp = [...this._pickPos];
          this._pickPos = [this._dropPos[0], this._dropPos[1], this._safeZ];
          this._dropPos = [tmp[0], tmp[1], this._safeZ];
          // Move payload visual to new pick location (DH x,y → Three x, -y)
          this.sm.setPayloadPosition(this._pickPos[0], 0.013, -this._pickPos[1]);
          this._transitionTo(STATE.PRE_GRASP);
        }
        break;

      default:
        this.stop();
    }
  }
}
