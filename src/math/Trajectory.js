/**
 * Trajectory.js — Trapezoidal Velocity Profile Generator (Eq. 74)
 *
 * Generates smooth motion profiles with three phases:
 *   1. Acceleration  (0 → v_max)
 *   2. Cruise        (constant v_max)
 *   3. Deceleration  (v_max → 0)
 *
 * When the move is too short to reach v_max, the profile degrades
 * to a triangular shape (accel → decel, no cruise).
 *
 * Pure math — no DOM, no rendering.
 */

import { clamp } from './Kinematics.js';

/**
 * Evaluate the trapezoidal velocity profile at time `t`.
 *
 * @param {number} t     - Current time (s)
 * @param {number} T     - Total motion duration (s)
 * @param {number} vmax  - Maximum velocity (rad/s)
 * @param {number} amax  - Maximum acceleration (rad/s²)
 * @returns {{ s: number, sdot: number, sddot: number, phase: string }}
 *   s      — normalised position  [0, 1]
 *   sdot   — velocity
 *   sddot  — acceleration
 *   phase  — human-readable label for the current segment
 */
export function trapProfile(t, T, vmax, amax) {
  // Edge case: zero or negative duration
  if (T <= 0) return { s: 1, sdot: 0, sddot: 0, phase: 'done' };

  const t1 = vmax / amax;

  // Check if triangular (can't reach vmax in time)
  const isTriangle = (2 * t1 > T);

  if (isTriangle) {
    // Symmetric triangular profile: peak at T/2
    const tMid = T / 2;
    // Total area (displacement) = 0.5 * a_peak * tMid^2 * 2 = a_peak * tMid^2
    // where a_peak = 1 / tMid^2 to normalize s to [0, 1]
    // Simplified: s = 2*(t/T)^2 during accel, s = 1 - 2*((T-t)/T)^2 during decel
    if (t < tMid) {
      const ratio = t / T;
      const s = 2 * ratio * ratio;
      return { s: clamp(s, 0, 1), sdot: amax * t, sddot: amax, phase: 'accel▲' };
    } else {
      const ratio = (T - t) / T;
      const s = 1 - 2 * ratio * ratio;
      return { s: clamp(s, 0, 1), sdot: amax * (T - t), sddot: -amax, phase: 'decel▼' };
    }
  }

  // Full trapezoidal profile
  const t2 = T - t1;
  const sAccelEnd = 0.5 * amax * t1 * t1;
  const sDecelStart = 1 - sAccelEnd;

  if (t <= 0) return { s: 0, sdot: 0, sddot: 0, phase: 'wait' };
  if (t >= T) return { s: 1, sdot: 0, sddot: 0, phase: 'done' };

  if (t < t1) {
    // Phase 1: Acceleration
    const s = sAccelEnd * (t / t1) * (t / t1);
    return { s, sdot: amax * t, sddot: amax, phase: 'accel▲' };
  } else if (t <= t2) {
    // Phase 2: Cruise (constant velocity)
    const s = sAccelEnd + vmax * (t - t1);
    return { s: clamp(s, 0, 1), sdot: vmax, sddot: 0, phase: 'cruise→' };
  } else {
    // Phase 3: Deceleration
    const dt = t - t2;
    const s = sDecelStart + vmax * dt - 0.5 * amax * dt * dt;
    return { s: clamp(s, 0, 1), sdot: vmax - amax * dt, sddot: -amax, phase: 'decel▼' };
  }
}
