/**
 * CameraModel.js — Pinhole Camera Model (Eq. 63–68)
 *
 * Provides pure mathematical functions for:
 *   1. Inverse projection: pixel (u, v) + depth → 3D camera-frame coords
 *   2. Extrinsic transform: camera-frame → robot-frame coordinates
 *
 * Assumes the camera is mounted above the robot base, looking downward
 * at the table surface. The rotation matrix R maps:
 *   camera x → robot x
 *   camera y → robot z
 *   camera z → robot -y  (depth axis = forward)
 *
 * Pure math — no DOM, no Three.js, no rendering.
 */

/**
 * Inverse Pinhole Projection (Eq. 64–66)
 *
 * Given pixel coordinates (u, v) and known world-frame depth Z_w,
 * reconstructs the 3D point in the camera coordinate frame.
 *
 *   X_w = (u - c_x) · Z_w / f_x
 *   Y_w = (v - c_y) · Z_w / f_y
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
 * Converts a 3D point from camera coordinates (Xw, Yw, Zw)
 * into the robot base frame using:
 *
 *   p_robot = R · p_cam + t
 *
 * where R accounts for the 90° downward camera mount:
 *   R = [1, 0, 0;  0, 0, 1;  0, -1, 0]
 *
 * Simplified result:
 *   x_robot = X_w + t_x
 *   y_robot = Z_w + t_y − 0.50   (depth → forward, with baseline offset)
 *   z_robot = −Y_w + t_z
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
  const yr = Zw + ty - 0.50;   // depth = forward = robot y direction (approx)
  const zr = -Yw + tz;
  return { xr, yr, zr };
}
