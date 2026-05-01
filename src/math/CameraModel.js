/**
 * CameraModel.js — Pinhole Camera Model (Eq. 63–68)
 *
 * Provides pure mathematical functions for:
 *   1. Intrinsic matrix K (3×3) from focal length and principal point
 *   2. Extrinsic matrix [R|t] (4×4) from Euler angles and translation
 *   3. Forward projection: 3D world → 2D pixel coordinates
 *   4. Inverse projection: pixel (u, v) + depth → 3D camera-frame coords
 *   5. Camera-to-robot frame transform
 *
 * Pure math — no DOM, no Three.js, no rendering.
 */

/**
 * Build the 3×3 intrinsic camera matrix K.
 *
 *   K = [ fx   0  cx ]
 *       [  0  fy  cy ]
 *       [  0   0   1 ]
 *
 * @param {number} fx - Focal length x (pixels)
 * @param {number} fy - Focal length y (pixels)
 * @param {number} cx - Principal point x (pixels)
 * @param {number} cy - Principal point y (pixels)
 * @returns {number[]} Flat 3×3 row-major matrix [9 elements]
 */
export function buildIntrinsicMatrix(fx, fy, cx, cy) {
  return [
    fx,  0, cx,
     0, fy, cy,
     0,  0,  1,
  ];
}

/**
 * Build the 4×4 extrinsic matrix [R|t] from Euler angles (XYZ order)
 * and translation.
 *
 * @param {number} tx - Translation x (m)
 * @param {number} ty - Translation y (m)
 * @param {number} tz - Translation z (m)
 * @param {number} rx - Rotation about X (rad)
 * @param {number} ry - Rotation about Y (rad)
 * @param {number} rz - Rotation about Z (rad)
 * @returns {number[]} Flat 4×4 row-major matrix [16 elements]
 */
export function buildExtrinsicMatrix(tx, ty, tz, rx, ry, rz) {
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cz = Math.cos(rz), sz = Math.sin(rz);

  // R = Rz · Ry · Rx (standard Euler XYZ)
  const r00 = cy * cz;
  const r01 = cz * sx * sy - cx * sz;
  const r02 = sx * sz + cx * cz * sy;
  const r10 = cy * sz;
  const r11 = cx * cz + sx * sy * sz;
  const r12 = cx * sy * sz - cz * sx;
  const r20 = -sy;
  const r21 = cy * sx;
  const r22 = cx * cy;

  return [
    r00, r01, r02, tx,
    r10, r11, r12, ty,
    r20, r21, r22, tz,
      0,   0,   0,  1,
  ];
}

/**
 * Project a 3D world point to 2D pixel coordinates.
 *
 *   p_cam  = [R|t] · [X, Y, Z, 1]ᵀ
 *   [u, v] = K · p_cam  (with perspective divide)
 *
 * @param {number} X - World X (m)
 * @param {number} Y - World Y (m)
 * @param {number} Z - World Z (m)
 * @param {number[]} K   - 3×3 intrinsic matrix (row-major flat)
 * @param {number[]} Ext - 4×4 extrinsic matrix (row-major flat)
 * @returns {{ u: number, v: number, depth: number, valid: boolean }}
 */
export function project3DtoPixel(X, Y, Z, K, Ext) {
  // Transform to camera frame: p_cam = Ext · [X, Y, Z, 1]
  const xc = Ext[0] * X + Ext[1] * Y + Ext[2]  * Z + Ext[3];
  const yc = Ext[4] * X + Ext[5] * Y + Ext[6]  * Z + Ext[7];
  const zc = Ext[8] * X + Ext[9] * Y + Ext[10] * Z + Ext[11];

  // Point behind camera
  if (zc <= 0) return { u: 0, v: 0, depth: zc, valid: false };

  // Perspective projection: K · [xc, yc, zc]ᵀ / zc
  const u = (K[0] * xc + K[1] * yc + K[2] * zc) / zc;
  const v = (K[3] * xc + K[4] * yc + K[5] * zc) / zc;

  return { u, v, depth: zc, valid: true };
}

/**
 * Compute focal length from vertical FOV and sensor height.
 *   fy = (height / 2) / tan(fov / 2)
 *
 * @param {number} fovDeg - Vertical FOV in degrees
 * @param {number} height - Sensor height in pixels
 * @returns {number} Focal length in pixels
 */
export function fovToFocalLength(fovDeg, height) {
  const fovRad = fovDeg * Math.PI / 180;
  return (height / 2) / Math.tan(fovRad / 2);
}

/**
 * Inverse Pinhole Projection (Eq. 64–66)
 *
 * Given pixel coordinates (u, v) and known world-frame depth Z_w,
 * reconstructs the 3D point in the camera coordinate frame.
 *
 * @param {number} u   - Horizontal pixel coordinate
 * @param {number} v   - Vertical pixel coordinate
 * @param {number} Zw  - Known depth / world Z (m)
 * @param {number} fx  - Focal length x (px)
 * @param {number} fy  - Focal length y (px)
 * @param {number} cx  - Principal point x (px)
 * @param {number} cy  - Principal point y (px)
 * @returns {{ Xw: number, Yw: number }}
 */
export function projectPixelsTo3D(u, v, Zw, fx, fy, cx, cy) {
  const Xw = (u - cx) * Zw / fx;
  const Yw = (v - cy) * Zw / fy;
  return { Xw, Yw };
}

/**
 * Camera-to-Robot Frame Transform (Eq. 68)
 *
 * @param {number} Xw  - Camera-frame X (m)
 * @param {number} Yw  - Camera-frame Y (m)
 * @param {number} Zw  - Camera-frame Z / depth (m)
 * @param {number} tx  - Translation x (m)
 * @param {number} ty  - Translation y (m)
 * @param {number} tz  - Translation z (m)
 * @returns {{ xr: number, yr: number, zr: number }}
 */
export function transformCameraToRobot(Xw, Yw, Zw, tx, ty, tz) {
  const xr = Xw + tx;
  const yr = Zw + ty - 0.50;
  const zr = -Yw + tz;
  return { xr, yr, zr };
}
