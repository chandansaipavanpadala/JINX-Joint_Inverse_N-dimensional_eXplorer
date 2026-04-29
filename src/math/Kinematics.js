/**
 * Kinematics.js — Closed-form FK/IK solvers & Analytical Jacobian
 * 
 * 3-DOF RRR Planar Arm (DH Convention)
 *   Link 1: a=0, α=π/2, d=L1, θ=θ1★
 *   Link 2: a=L2, α=0,   d=0,  θ=θ2★
 *   Link 3: a=L3, α=0,   d=0,  θ=θ3★
 *
 * All math extracted verbatim from the monolithic prototype.
 */

import * as THREE from 'three';

/* ═══════════════════ Constants ═══════════════════ */
export const L1 = 0.15;   // Base column height (m)
export const L2 = 0.30;   // Lower arm length  (m)
export const L3 = 0.24;   // Upper arm length  (m)

/* ═══════════════════ Helpers ═══════════════════ */
export const V3 = (x, y, z) => new THREE.Vector3(x, y, z);
export const DEG = (v) => v * Math.PI / 180;
export const RAD = (v) => v * 180 / Math.PI;
export const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

/* ═══════════════════ Joint Limits ═══════════════════ */
export const T2MIN = DEG(15);
export const T2MAX = DEG(115);
export const T3MIN = DEG(20);
export const T3MAX = DEG(145);

/**
 * Forward Kinematics — DH closed-form
 * 
 * Given joint angles (t1, t2, t3), returns:
 *   x, y, z — end-effector position in robot frame
 *   r       — radial reach in XY plane
 *   P0–P3   — joint positions as THREE.Vector3 (for rendering)
 *
 * @param {number} t1 - Joint 1 angle (yaw, rad)
 * @param {number} t2 - Joint 2 angle (shoulder, rad)
 * @param {number} t3 - Joint 3 angle (elbow, rad)
 * @returns {{ x: number, y: number, z: number, r: number, P0: THREE.Vector3, P1: THREE.Vector3, P2: THREE.Vector3, P3: THREE.Vector3 }}
 */
export function fkMat(t1, t2, t3) {
  const c1 = Math.cos(t1), s1 = Math.sin(t1), c2 = Math.cos(t2), s2 = Math.sin(t2);
  const c23 = Math.cos(t2 + t3), s23 = Math.sin(t2 + t3);
  const r = L2 * c2 + L3 * c23;
  return {
    x: r * c1, y: r * s1, z: L1 + L2 * s2 + L3 * s23, r,
    P0: V3(0, 0, 0), P1: V3(0, L1, 0),
    P2: V3(L2 * c1 * c2, L1 + L2 * s2, -L2 * s1 * c2),
    P3: V3(r * c1, L1 + L2 * s2 + L3 * s23, -r * s1)
  };
}

/**
 * Inverse Kinematics — Closed-form geometric solver
 * 
 * Given desired end-effector position (xd, yd, zd) and elbow sign,
 * returns joint angles { t1, t2, t3 } or null if unreachable.
 *
 * Uses atan2 for quadrant-correct solutions.
 * eSign = +1 → elbow-up, eSign = -1 → elbow-down
 *
 * @param {number} xd   - Desired x position (m)
 * @param {number} yd   - Desired y position (m)
 * @param {number} zd   - Desired z position (m)
 * @param {number} [eSign=1] - Elbow configuration sign
 * @returns {{ t1: number, t2: number, t3: number } | null}
 */
export function ikMat(xd, yd, zd, eSign = 1) {
  const t1 = Math.atan2(yd, xd), r = Math.hypot(xd, yd), zp = zd - L1;
  const D2 = r * r + zp * zp, C3 = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  if (Math.abs(C3) > 1.0001) return null;
  const C3c = clamp(C3, -1, 1), s3 = eSign * Math.sqrt(1 - C3c * C3c);
  const t3 = Math.atan2(s3, C3c);
  const t2 = Math.atan2(zp, r) - Math.atan2(L3 * Math.sin(t3), L2 + L3 * Math.cos(t3));
  return { t1, t2, t3 };
}

/**
 * Analytical Jacobian J(q) — 3×3
 * 
 * Maps joint velocities q̇ = [θ̇₁, θ̇₂, θ̇₃] to
 * Cartesian velocities ṗ = [ẋ, ẏ, ż]:
 *
 *   ṗ = J(q) · q̇
 *
 * @param {number} t1 - Joint 1 angle (rad)
 * @param {number} t2 - Joint 2 angle (rad)
 * @param {number} t3 - Joint 3 angle (rad)
 * @returns {number[][]} 3×3 Jacobian matrix
 */
export function jacMat(t1, t2, t3) {
  const c1 = Math.cos(t1), s1 = Math.sin(t1), s2 = Math.sin(t2), c2 = Math.cos(t2);
  const s23 = Math.sin(t2 + t3), c23 = Math.cos(t2 + t3), R = L2 * c2 + L3 * c23;
  return [[-s1 * R, -c1 * (L2 * s2 + L3 * s23), -L3 * c1 * s23],
  [c1 * R, -s1 * (L2 * s2 + L3 * s23), -L3 * s1 * s23],
  [0, L2 * c2 + L3 * c23, L3 * c23]];
}

/**
 * 3×3 Matrix determinant
 * 
 * Used for manipulability index μ = |det(J)|
 *
 * @param {number[][]} m - 3×3 matrix
 * @returns {number} Determinant
 */
export function det3(m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
