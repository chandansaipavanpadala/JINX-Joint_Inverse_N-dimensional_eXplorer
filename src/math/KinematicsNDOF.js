/**
 * KinematicsNDOF.js — Generalized N-DOF Kinematics Engine
 *
 * Pure JavaScript math module. NO DOM, NO Three.js dependencies.
 * Supports arbitrary serial manipulators defined by DH parameter tables.
 *
 * Exports:
 *   fk(q, dhTable)                     — Forward Kinematics (cascaded DH transforms)
 *   jacobian(q, dhTable)               — Analytical Jacobian (6×N, handles R & P joints)
 *   ik_dls(pTarget, qCurrent, dhTable) — Damped Least-Squares numerical IK solver
 *   SCARA_DH_CONFIG                    — Pre-defined 4-DOF SCARA (R-R-P-R) configuration
 *
 * DH Table format (Modified DH Convention — Craig):
 *   { a, alpha, d, theta, type, limitMin, limitMax }
 *     a       — link length  (meters)
 *     alpha   — link twist   (radians)
 *     d       — link offset  (meters)
 *     theta   — joint angle  (radians, offset for revolute)
 *     type    — 'R' (revolute) or 'P' (prismatic)
 *     limitMin — lower joint limit (rad for R, meters for P)
 *     limitMax — upper joint limit (rad for R, meters for P)
 *
 * Matrix layout — column-major flat Float64Array[16]:
 *   [ m00, m10, m20, m30,   ← column 0
 *     m01, m11, m21, m31,   ← column 1
 *     m02, m12, m22, m32,   ← column 2
 *     m03, m13, m23, m33 ]  ← column 3
 *
 * This matches WebGL / Three.js memory order for future interop,
 * while remaining completely framework-independent.
 */

// ═══════════════════════════════════════════════════════════════════════════════
//  §1  MATRIX UTILITIES (4×4 Homogeneous, column-major flat array)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Create a 4×4 identity matrix.
 * @returns {Float64Array} 16-element identity matrix
 */
function mat4Identity() {
  const m = new Float64Array(16);
  m[0] = 1; m[5] = 1; m[10] = 1; m[15] = 1;
  return m;
}

/**
 * Multiply two 4×4 column-major matrices: C = A · B
 * @param {Float64Array} a - Left matrix
 * @param {Float64Array} b - Right matrix
 * @returns {Float64Array} Result matrix
 */
function mat4Mul(a, b) {
  const c = new Float64Array(16);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      c[col * 4 + row] =
        a[0 * 4 + row] * b[col * 4 + 0] +
        a[1 * 4 + row] * b[col * 4 + 1] +
        a[2 * 4 + row] * b[col * 4 + 2] +
        a[3 * 4 + row] * b[col * 4 + 3];
    }
  }
  return c;
}

/**
 * Build a single DH transformation matrix for one link.
 *
 * Standard DH convention (Denavit–Hartenberg):
 *   T = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
 *
 * Resulting 4×4 matrix:
 *   [ cθ  -sθ·cα   sθ·sα   a·cθ ]
 *   [ sθ   cθ·cα  -cθ·sα   a·sθ ]
 *   [ 0    sα      cα       d    ]
 *   [ 0    0       0        1    ]
 *
 * @param {number} theta - Joint angle θ (radians)
 * @param {number} d     - Link offset d (meters)
 * @param {number} a     - Link length a (meters)
 * @param {number} alpha - Link twist α (radians)
 * @returns {Float64Array} 4×4 column-major transformation matrix
 */
function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);

  const m = new Float64Array(16);
  // Column 0
  m[0]  = ct;
  m[1]  = st;
  m[2]  = 0;
  m[3]  = 0;
  // Column 1
  m[4]  = -st * ca;
  m[5]  =  ct * ca;
  m[6]  =  sa;
  m[7]  =  0;
  // Column 2
  m[8]  =  st * sa;
  m[9]  = -ct * sa;
  m[10] =  ca;
  m[11] =  0;
  // Column 3
  m[12] =  a * ct;
  m[13] =  a * st;
  m[14] =  d;
  m[15] =  1;

  return m;
}

/**
 * Extract the origin (translation) from a 4×4 column-major matrix.
 * @param {Float64Array} m - 4×4 matrix
 * @returns {number[]} [x, y, z]
 */
function mat4Origin(m) {
  return [m[12], m[13], m[14]];
}

/**
 * Extract the Z-axis (third column direction) from a 4×4 column-major matrix.
 * @param {Float64Array} m - 4×4 matrix
 * @returns {number[]} [zx, zy, zz]
 */
function mat4ZAxis(m) {
  return [m[8], m[9], m[10]];
}

/**
 * 3D cross product: a × b
 * @param {number[]} a
 * @param {number[]} b
 * @returns {number[]}
 */
function cross3(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/**
 * 3D vector subtraction: a - b
 * @param {number[]} a
 * @param {number[]} b
 * @returns {number[]}
 */
function sub3(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

/**
 * Euclidean norm of a 3D vector.
 * @param {number[]} v
 * @returns {number}
 */
function norm3(v) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * Clamp a scalar to [lo, hi].
 * @param {number} v
 * @param {number} lo
 * @param {number} hi
 * @returns {number}
 */
function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  §2  FORWARD KINEMATICS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Compute Forward Kinematics for an N-DOF serial manipulator.
 *
 * Cascades DH transformation matrices from base to end-effector:
 *   T_total = T_1 · T_2 · … · T_N
 *
 * @param {number[]} q        - Joint variables [q1, q2, …, qN]
 * @param {Object[]} dhTable  - Array of DH parameter objects
 * @returns {{
 *   T: Float64Array,            — 4×4 end-effector transform (column-major)
 *   position: number[],         — [x, y, z] end-effector position
 *   jointTransforms: Float64Array[]  — Intermediate transforms T_0, T_01, T_012, …
 * }}
 */
export function fk(q, dhTable) {
  const n = dhTable.length;
  const jointTransforms = new Array(n + 1);

  // T_0 = Identity (base frame)
  let T = mat4Identity();
  jointTransforms[0] = T;

  for (let i = 0; i < n; i++) {
    const dh = dhTable[i];

    // Determine effective theta and d based on joint type
    let theta_i, d_i;
    if (dh.type === 'P') {
      // Prismatic: variable is d, theta is fixed offset
      theta_i = dh.theta;
      d_i     = dh.d + q[i];
    } else {
      // Revolute (default): variable is theta, d is fixed offset
      theta_i = dh.theta + q[i];
      d_i     = dh.d;
    }

    const Ti = dhMatrix(theta_i, d_i, dh.a, dh.alpha);
    T = mat4Mul(T, Ti);
    jointTransforms[i + 1] = T;
  }

  return {
    T,
    position: mat4Origin(T),
    jointTransforms,
  };
}


// ═══════════════════════════════════════════════════════════════════════════════
//  §3  ANALYTICAL JACOBIAN
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Compute the 6×N Geometric Jacobian for an N-DOF serial manipulator.
 *
 * For each joint i:
 *   • Revolute  (R): Jv_i = Z_{i-1} × (O_n − O_{i-1}),  Jw_i = Z_{i-1}
 *   • Prismatic (P): Jv_i = Z_{i-1},                       Jw_i = [0,0,0]
 *
 * Where Z_{i-1} is the z-axis of the (i-1)-th frame, O_{i-1} its origin,
 * and O_n is the end-effector origin.
 *
 * The Jacobian is returned as a flat array in ROW-MAJOR order [6][N],
 * accessible as J[row * N + col].
 *
 * @param {number[]} q       - Joint variables [q1, …, qN]
 * @param {Object[]} dhTable - DH parameter table
 * @returns {{
 *   J: Float64Array,       — Flat 6×N Jacobian (row-major)
 *   rows: number,          — Always 6
 *   cols: number           — N (number of joints)
 * }}
 */
export function jacobian(q, dhTable) {
  const n = dhTable.length;
  const { jointTransforms, position: On } = fk(q, dhTable);

  // 6×N Jacobian in row-major flat array
  const J = new Float64Array(6 * n);

  for (let i = 0; i < n; i++) {
    const T_prev = jointTransforms[i];  // Frame i-1 (0-indexed: frame before joint i)
    const Zi = mat4ZAxis(T_prev);       // Z-axis of frame i-1
    const Oi = mat4Origin(T_prev);      // Origin of frame i-1

    if (dhTable[i].type === 'P') {
      // ── Prismatic joint ──
      // Linear velocity: Z_{i-1}
      J[0 * n + i] = Zi[0];
      J[1 * n + i] = Zi[1];
      J[2 * n + i] = Zi[2];
      // Angular velocity: zero
      J[3 * n + i] = 0;
      J[4 * n + i] = 0;
      J[5 * n + i] = 0;
    } else {
      // ── Revolute joint (default) ──
      const diff = sub3(On, Oi);
      const Jv = cross3(Zi, diff);
      // Linear velocity: Z_{i-1} × (O_n − O_{i-1})
      J[0 * n + i] = Jv[0];
      J[1 * n + i] = Jv[1];
      J[2 * n + i] = Jv[2];
      // Angular velocity: Z_{i-1}
      J[3 * n + i] = Zi[0];
      J[4 * n + i] = Zi[1];
      J[5 * n + i] = Zi[2];
    }
  }

  return { J, rows: 6, cols: n };
}


// ═══════════════════════════════════════════════════════════════════════════════
//  §4  SMALL DENSE LINEAR ALGEBRA (for DLS solver)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Multiply matrix A (m×n, row-major) by vector x (n),  y = A · x.
 * @param {Float64Array} A - Row-major matrix
 * @param {number} m       - Rows of A
 * @param {number} n       - Cols of A
 * @param {Float64Array|number[]} x - Vector of length n
 * @returns {Float64Array} Result vector of length m
 */
function matvec(A, m, n, x) {
  const y = new Float64Array(m);
  for (let i = 0; i < m; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) s += A[i * n + j] * x[j];
    y[i] = s;
  }
  return y;
}

/**
 * Multiply A^T (transposed m×n → n×m, row-major) by vector x (m),  y = Aᵀ · x.
 * @param {Float64Array} A - Row-major m×n matrix
 * @param {number} m       - Rows of A
 * @param {number} n       - Cols of A
 * @param {Float64Array|number[]} x - Vector of length m
 * @returns {Float64Array} Result vector of length n
 */
function matTvec(A, m, n, x) {
  const y = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    let s = 0;
    for (let i = 0; i < m; i++) s += A[i * n + j] * x[i];
    y[j] = s;
  }
  return y;
}

/**
 * Compute C = A · Aᵀ  for row-major A (m×n). Result is m×m row-major.
 * @param {Float64Array} A
 * @param {number} m
 * @param {number} n
 * @returns {Float64Array} m×m row-major
 */
function matMulTranspose(A, m, n) {
  const C = new Float64Array(m * m);
  for (let i = 0; i < m; i++) {
    for (let j = i; j < m; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += A[i * n + k] * A[j * n + k];
      C[i * m + j] = s;
      C[j * m + i] = s; // symmetric
    }
  }
  return C;
}

/**
 * Solve a small dense symmetric positive-definite system  M·x = b
 * via Cholesky decomposition (LLᵀ).
 *
 * Falls back to regularized solve if the matrix is near-singular.
 *
 * @param {Float64Array} M - m×m row-major SPD matrix (will be modified in-place)
 * @param {number} m       - Dimension
 * @param {Float64Array} b - Right-hand side (length m)
 * @returns {Float64Array} Solution vector x (length m)
 */
function choleskySolve(M, m, b) {
  // In-place Cholesky decomposition: M = L · Lᵀ (lower-triangular in M)
  const L = new Float64Array(m * m);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j <= i; j++) {
      let s = M[i * m + j];
      for (let k = 0; k < j; k++) s -= L[i * m + k] * L[j * m + k];
      if (i === j) {
        if (s <= 0) s = 1e-12; // numerical guard
        L[i * m + j] = Math.sqrt(s);
      } else {
        L[i * m + j] = s / L[j * m + j];
      }
    }
  }

  // Forward substitution: L · y = b
  const y = new Float64Array(m);
  for (let i = 0; i < m; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i * m + k] * y[k];
    y[i] = s / L[i * m + i];
  }

  // Back substitution: Lᵀ · x = y
  const x = new Float64Array(m);
  for (let i = m - 1; i >= 0; i--) {
    let s = y[i];
    for (let k = i + 1; k < m; k++) s -= L[k * m + i] * x[k];
    x[i] = s / L[i * m + i];
  }

  return x;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  §5  DAMPED LEAST-SQUARES (DLS) INVERSE KINEMATICS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Damped Least-Squares (Levenberg–Marquardt) numerical IK solver.
 *
 * Iteratively solves for joint variables q such that FK(q) ≈ pTarget.
 *
 * Update rule (position-only, 3×N Jacobian):
 *   Δq = Jᵀ · (J · Jᵀ + λ² · I)⁻¹ · e
 *
 * Where:
 *   e  = pTarget − FK(q).position       (3×1 position error)
 *   J  = Jv (top 3 rows of the 6×N Jacobian, linear velocity)
 *   λ  = damping factor
 *
 * Joint limits from dhTable are enforced via clamping after each step.
 *
 * @param {number[]} pTarget   - Desired end-effector position [x, y, z]
 * @param {number[]} qCurrent  - Initial joint angles/displacements
 * @param {Object[]} dhTable   - DH parameter table with joint limits
 * @param {number}   [maxIters=100]   - Maximum solver iterations
 * @param {number}   [damping=0.1]    - Damping factor λ
 * @param {number}   [tolerance=1e-4] - Convergence threshold (meters)
 * @returns {{
 *   q: number[],         — Solved joint configuration
 *   position: number[],  — Achieved end-effector position
 *   error: number,       — Final position error (meters)
 *   converged: boolean,  — Whether error < tolerance
 *   iterations: number   — Number of iterations used
 * }}
 */
export function ik_dls(
  pTarget,
  qCurrent,
  dhTable,
  maxIters = 100,
  damping = 0.1,
  tolerance = 1e-4
) {
  const n = dhTable.length;
  const m = 3; // position-only (linear velocity rows of Jacobian)
  const lambda2 = damping * damping;

  // Working copy of joint variables
  const q = Float64Array.from(qCurrent);

  let lastError = Infinity;
  let iters = 0;

  for (let iter = 0; iter < maxIters; iter++) {
    iters = iter + 1;

    // ── Forward Kinematics ──
    const { position: pCurrent } = fk(q, dhTable);

    // ── Position error vector (3×1) ──
    const e = new Float64Array(m);
    e[0] = pTarget[0] - pCurrent[0];
    e[1] = pTarget[1] - pCurrent[1];
    e[2] = pTarget[2] - pCurrent[2];

    const err = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    lastError = err;

    // ── Convergence check ──
    if (err < tolerance) {
      return {
        q: Array.from(q),
        position: pCurrent,
        error: err,
        converged: true,
        iterations: iters,
      };
    }

    // ── Compute Jacobian (extract top 3 rows — linear velocity) ──
    const { J: J6 } = jacobian(q, dhTable);
    // Extract 3×N sub-matrix (rows 0–2 of the 6×N)
    const Jv = new Float64Array(m * n);
    for (let r = 0; r < m; r++) {
      for (let c = 0; c < n; c++) {
        Jv[r * n + c] = J6[r * n + c];
      }
    }

    // ── DLS update: Δq = Jᵀ · (J·Jᵀ + λ²·I)⁻¹ · e ──
    // 1. Compute  A = J · Jᵀ  (m×m)
    const A = matMulTranspose(Jv, m, n);

    // 2. Add damping: A += λ² · I
    for (let i = 0; i < m; i++) {
      A[i * m + i] += lambda2;
    }

    // 3. Solve  A · v = e  →  v = (J·Jᵀ + λ²·I)⁻¹ · e
    const v = choleskySolve(A, m, e);

    // 4. Δq = Jᵀ · v
    const dq = matTvec(Jv, m, n, v);

    // ── Update joint variables and enforce limits ──
    for (let i = 0; i < n; i++) {
      q[i] += dq[i];

      // Clamp to joint limits if defined
      const lo = dhTable[i].limitMin;
      const hi = dhTable[i].limitMax;
      if (lo !== undefined && hi !== undefined) {
        q[i] = clamp(q[i], lo, hi);
      }
    }
  }

  // ── Return best result after max iterations ──
  const { position: pFinal } = fk(q, dhTable);
  return {
    q: Array.from(q),
    position: pFinal,
    error: lastError,
    converged: false,
    iterations: iters,
  };
}


// ═══════════════════════════════════════════════════════════════════════════════
//  §6  SCARA 4-DOF CONFIGURATION  (R-R-P-R)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Standard 4-DOF SCARA arm DH configuration.
 *
 * Kinematic structure:
 *   Joint 1 — Base Yaw       (Revolute)  : rotates entire arm about Z₀
 *   Joint 2 — Elbow Yaw      (Revolute)  : rotates forearm about Z₁
 *   Joint 3 — Z-Translation  (Prismatic) : vertical stroke of the spindle
 *   Joint 4 — Tool Roll      (Revolute)  : end-effector rotation about Z₃
 *
 *   L1 = 0.30 m  (upper arm length)
 *   L2 = 0.25 m  (forearm length)
 *   d1 = 0.35 m  (base column height)
 *
 * DH Parameters (Standard Convention):
 *   ┌───────┬───────┬────────┬───────┬────────┬──────┬──────────┬──────────┐
 *   │ Joint │ a (m) │ α(rad) │ d (m) │ θ(rad) │ Type │ limitMin │ limitMax │
 *   ├───────┼───────┼────────┼───────┼────────┼──────┼──────────┼──────────┤
 *   │   1   │ 0.30  │   0    │ 0.35  │   0    │  R   │  -2.356  │  2.356   │
 *   │   2   │ 0.25  │   π    │  0    │   0    │  R   │  -2.356  │  2.356   │
 *   │   3   │  0    │   0    │  0    │   0    │  P   │   0.000  │  0.200   │
 *   │   4   │  0    │   0    │  0    │   0    │  R   │   -π     │   π      │
 *   └───────┴───────┴────────┴───────┴────────┴──────┴──────────┴──────────┘
 *
 * Notes:
 *   - Joint 2 has α = π to flip the Z-axis, keeping the forearm in the
 *     same horizontal plane (SCARA signature geometry).
 *   - Joint 3 (prismatic) translates along the (now inverted) Z-axis,
 *     providing vertical reach. d_offset = 0, variable d is the stroke.
 *   - Joint 4 is a wrist roll for tool orientation.
 *   - Joint limits: ±135° for yaw joints, 0–200 mm for prismatic stroke,
 *     ±180° for tool roll.
 */
export const SCARA_DH_CONFIG = Object.freeze([
  {
    a: 0.30,
    alpha: 0,
    d: 0.35,
    theta: 0,
    type: 'R',
    limitMin: -2.356,   // -135°
    limitMax:  2.356,   //  135°
  },
  {
    a: 0.25,
    alpha: Math.PI,
    d: 0,
    theta: 0,
    type: 'R',
    limitMin: -2.356,   // -135°
    limitMax:  2.356,   //  135°
  },
  {
    a: 0,
    alpha: 0,
    d: 0,
    theta: 0,
    type: 'P',
    limitMin: 0,        //   0 mm
    limitMax: 0.200,    // 200 mm
  },
  {
    a: 0,
    alpha: 0,
    d: 0,
    theta: 0,
    type: 'R',
    limitMin: -Math.PI, // -180°
    limitMax:  Math.PI, //  180°
  },
]);
