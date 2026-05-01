/**
 * WaypointTask.js — Generic Waypoint Planner & Executor
 * PDF §Motion Planning: "Waypoint planner"
 *
 * Manages an ordered list of 3D coordinates and smoothly interpolates
 * the IK target through them using trapezoidal velocity profiles.
 *
 * Usage:
 *   const wp = new WaypointTask();
 *   wp.addWaypoint({ x: 0.4, y: 0.0, z: 0.2 });
 *   wp.addWaypoint({ x: 0.5, y: 0.1, z: 0.1 });
 *   wp.play();
 *   // In RAF loop: const target = wp.update(dt);
 *
 * Pure logic — no DOM, no Three.js dependencies.
 */

import { trapProfile } from '../math/Trajectory.js';

export class WaypointTask {
  constructor() {
    /** @type {{ x: number, y: number, z: number }[]} */
    this.waypoints = [];

    /** Execution mode: 'once' | 'loop' */
    this.mode = 'once';

    /** Speed scalar — segment duration = baseDuration / speed */
    this.speed = 1.0;

    /** Base duration for each segment (seconds) */
    this.baseDuration = 2.0;

    /** Trapezoidal profile parameters */
    this.vmax = 1.5;
    this.amax = 3.0;

    // ── Internal state ──
    this._state = 'IDLE'; // 'IDLE' | 'RUNNING' | 'PAUSED' | 'COMPLETE'
    this._segmentIndex = 0;
    this._timer = 0;
    this._segmentDuration = 0;
    this._pStart = [0, 0, 0];
    this._pTarget = [0, 0, 0];
    this._currentPos = [0, 0, 0];

    // ── Callbacks ──
    /** @type {((state: string) => void) | null} */
    this.onStateChange = null;

    /** @type {((segIdx: number, total: number) => void) | null} */
    this.onSegmentChange = null;
  }

  // ═══════════════════════════════════════
  //  Waypoint Management
  // ═══════════════════════════════════════

  /**
   * Add a waypoint. Can pass {x,y,z} or [x,y,z].
   * @param {{ x: number, y: number, z: number } | number[]} pt
   * @param {number} [index] — Insert at index (default: append)
   */
  addWaypoint(pt, index) {
    const wp = Array.isArray(pt)
      ? { x: pt[0], y: pt[1], z: pt[2] }
      : { x: pt.x, y: pt.y, z: pt.z };

    if (typeof index === 'number' && index >= 0 && index <= this.waypoints.length) {
      this.waypoints.splice(index, 0, wp);
    } else {
      this.waypoints.push(wp);
    }
  }

  /**
   * Remove waypoint at index.
   * @param {number} index
   */
  removeWaypoint(index) {
    if (index >= 0 && index < this.waypoints.length) {
      this.waypoints.splice(index, 1);
    }
  }

  /**
   * Move waypoint from `fromIdx` to `toIdx`.
   * @param {number} fromIdx
   * @param {number} toIdx
   */
  moveWaypoint(fromIdx, toIdx) {
    if (fromIdx < 0 || fromIdx >= this.waypoints.length) return;
    toIdx = Math.max(0, Math.min(this.waypoints.length - 1, toIdx));
    const [wp] = this.waypoints.splice(fromIdx, 1);
    this.waypoints.splice(toIdx, 0, wp);
  }

  /** Clear all waypoints and reset state. */
  clearWaypoints() {
    this.waypoints = [];
    this.stop();
  }

  /** @returns {number} Total number of waypoints */
  get count() { return this.waypoints.length; }

  // ═══════════════════════════════════════
  //  Playback Control
  // ═══════════════════════════════════════

  /** Start executing the path from the beginning. */
  play() {
    if (this.waypoints.length < 2) return;

    this._segmentIndex = 0;
    this._timer = 0;
    this._setupSegment(0);
    this._setState('RUNNING');
  }

  /** Stop execution and reset. */
  stop() {
    this._setState('IDLE');
    this._segmentIndex = 0;
    this._timer = 0;
  }

  /** Pause execution (preserves position). */
  pause() {
    if (this._state === 'RUNNING') {
      this._setState('PAUSED');
    }
  }

  /** Resume from pause. */
  resume() {
    if (this._state === 'PAUSED') {
      this._setState('RUNNING');
    }
  }

  // ═══════════════════════════════════════
  //  Update Loop (call every frame)
  // ═══════════════════════════════════════

  /**
   * Advance the waypoint task by dt seconds.
   * @param {number} dt - Delta time (seconds), clamped internally.
   * @param {number[]} [currentPos] - Current EE position [x,y,z] for first-frame init.
   * @returns {number[] | null} Target [x,y,z] in DH coords, or null if idle.
   */
  update(dt, currentPos) {
    if (this._state !== 'RUNNING') return null;
    if (this.waypoints.length < 2) {
      this.stop();
      return null;
    }

    // First-frame initialization
    if (this._timer === 0 && this._segmentIndex === 0 && currentPos) {
      this._pStart = [...currentPos];
    }

    this._timer += dt;

    // Evaluate trapezoidal profile for this segment
    let s = 1.0;
    if (this._timer < this._segmentDuration) {
      const profile = trapProfile(this._timer, this._segmentDuration, this.vmax, this.amax);
      s = profile.s;
    }

    // Cartesian interpolation
    const x = this._pStart[0] + (this._pTarget[0] - this._pStart[0]) * s;
    const y = this._pStart[1] + (this._pTarget[1] - this._pStart[1]) * s;
    const z = this._pStart[2] + (this._pTarget[2] - this._pStart[2]) * s;
    this._currentPos = [x, y, z];

    // Segment completion check
    if (this._timer >= this._segmentDuration) {
      const nextSeg = this._segmentIndex + 1;

      if (nextSeg < this.waypoints.length - 1) {
        // Advance to next segment
        this._segmentIndex = nextSeg;
        this._setupSegment(nextSeg);
        this._timer = 0;
      } else {
        // Final waypoint reached
        if (this.mode === 'loop') {
          // Loop: restart from waypoint 0
          this._segmentIndex = 0;
          this._setupSegment(0);
          this._timer = 0;
        } else {
          // Once: mark complete
          this._setState('COMPLETE');
          return [this._pTarget[0], this._pTarget[1], this._pTarget[2]];
        }
      }
    }

    return [x, y, z];
  }

  // ═══════════════════════════════════════
  //  Internal
  // ═══════════════════════════════════════

  /**
   * Set up interpolation for segment `idx` → `idx + 1`.
   * @param {number} idx
   */
  _setupSegment(idx) {
    const from = this.waypoints[idx];
    const to = this.waypoints[idx + 1];

    this._pStart = [from.x, from.y, from.z];
    this._pTarget = [to.x, to.y, to.z];

    // Duration proportional to distance, scaled by speed
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const dz = to.z - from.z;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Longer segments take more time (min 0.5s, scaled by baseDuration)
    this._segmentDuration = Math.max(0.5, (dist / 0.3) * this.baseDuration) / this.speed;

    if (this.onSegmentChange) {
      this.onSegmentChange(idx, this.waypoints.length - 1);
    }
  }

  /**
   * @param {string} newState
   */
  _setState(newState) {
    if (this._state === newState) return;
    this._state = newState;
    if (this.onStateChange) this.onStateChange(newState);
  }

  // ═══════════════════════════════════════
  //  Getters
  // ═══════════════════════════════════════

  /** @returns {string} Current state: IDLE | RUNNING | PAUSED | COMPLETE */
  get state() { return this._state; }

  /** @returns {boolean} */
  get isRunning() { return this._state === 'RUNNING'; }

  /** @returns {number} Current segment index */
  get currentSegment() { return this._segmentIndex; }

  /** @returns {number} Total number of segments */
  get totalSegments() { return Math.max(0, this.waypoints.length - 1); }

  /** @returns {number} Overall progress 0..1 */
  get progress() {
    if (this.waypoints.length < 2) return 0;
    const segs = this.waypoints.length - 1;
    const segProgress = this._segmentDuration > 0
      ? Math.min(1, this._timer / this._segmentDuration)
      : 0;
    return (this._segmentIndex + segProgress) / segs;
  }

  /** @returns {number[]} Current interpolated position */
  get currentPosition() { return [...this._currentPos]; }

  /** @returns {number[]} Current segment target */
  get currentTarget() { return [...this._pTarget]; }
}
