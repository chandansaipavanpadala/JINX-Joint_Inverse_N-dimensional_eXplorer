# JINX: Joint Inverse N-dimensional eXplorer

A web-based, real-time 3D simulator for exploring robotic manipulator kinematics,
Jacobian analysis, and task-space trajectory planning. Built with Three.js, vanilla
JavaScript, and Vite.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Core Architecture](#core-architecture)
3. [Mathematical Models](#mathematical-models)
4. [Directory Structure](#directory-structure)
5. [Local Development Setup](#local-development-setup)
6. [Robot Configurations](#robot-configurations)
7. [License](#license)

---

## Project Overview

JINX provides an interactive environment for studying manipulator kinematics without
requiring MATLAB, ROS, or compiled simulation software. The simulator runs entirely
in the browser and supports closed-form inverse kinematics, real-time Jacobian
computation with singularity detection, trapezoidal velocity profiling, and
camera-based target acquisition via a pinhole camera model.

The initial configuration ships with a 3-DOF RRR (Revolute-Revolute-Revolute)
articulated desk lamp designed for occlusion-avoidance tasks, where the lamp head
must illuminate a moving target on a table surface while avoiding the user's hand.

### Key Capabilities

- Closed-form forward and inverse kinematics with elbow-up / elbow-down selection
- Analytical 3x3 Jacobian with color-coded visualization and manipulability index
- Singularity detection (elbow lock and shoulder singularity)
- Trapezoidal velocity profiling (Eq. 74) with configurable v_max and a_max
- Pinhole camera model (Eq. 63-68) with intrinsic/extrinsic parameter tuning
- Real-time 3D rendering with PBR materials, shadow mapping, and environment lighting
- Interactive target dragging via raycaster on the table plane
- Picture-in-picture camera POV viewport

---

## Core Architecture

The codebase follows a strict separation of concerns across four layers:

```
index.html                Landing page (robot configuration selector)
src/pages/rrr-lamp.html   Simulator page (3-DOF RRR lamp)
src/main.js               Application entry point (bootstrap)
src/math/                  Pure mathematical functions (zero DOM, zero Three.js*)
src/core/                  3D rendering and scene management (Three.js)
src/ui/                    DOM event handling, HUD updates, simulation loop
public/                    Static assets (CSS, icons)
```

*Exception: Kinematics.js imports THREE.Vector3 for joint position vectors returned
by the forward kinematics function, as these are consumed directly by the renderer.

### Data Flow

```
  Slider Input
      |
      v
  UIController.update()
      |
      +---> ikMat() / jacMat() / det3()     [src/math/Kinematics.js]
      +---> trapProfile()                    [src/math/Trajectory.js]
      +---> projectPixelsTo3D()              [src/math/CameraModel.js]
      |
      v
  SceneManager.updateScene(t1, t2, t3, pTgt)  [src/core/SceneManager.js]
      |
      +---> fkMat() for joint positions
      +---> Mesh repositioning (shoulder, elbow, wrist, shade, lights)
      +---> Returns FK result to UIController
      |
      v
  UIController updates DOM text (HUD, cards, Jacobian grid)
```

---

## Mathematical Models

### Denavit-Hartenberg Parameters

The 3-DOF RRR lamp uses standard DH convention with the following parameters:

| i | a_i (m) | alpha_i | d_i (m) | theta_i |
|---|---------|---------|---------|---------|
| 1 | 0       | pi/2    | L1=0.15 | theta_1 |
| 2 | L2=0.30 | 0       | 0       | theta_2 |
| 3 | L3=0.24 | 0       | 0       | theta_3 |

### Forward Kinematics

The end-effector position is computed from the DH chain:

```
r  = L2 * cos(t2) + L3 * cos(t2 + t3)
x  = r * cos(t1)
y  = r * sin(t1)
z  = L1 + L2 * sin(t2) + L3 * sin(t2 + t3)
```

### Inverse Kinematics (Closed-Form Geometric)

Given a desired position (xd, yd, zd):

```
t1 = atan2(yd, xd)
r  = sqrt(xd^2 + yd^2),  zp = zd - L1
C3 = (r^2 + zp^2 - L2^2 - L3^2) / (2 * L2 * L3)
t3 = atan2(e * sqrt(1 - C3^2), C3)          where e = +/-1 (elbow config)
t2 = atan2(zp, r) - atan2(L3*sin(t3), L2 + L3*cos(t3))
```

The solver returns null when |C3| > 1 (target unreachable).

### Analytical Jacobian

The 3x3 Jacobian J(q) maps joint velocities to Cartesian velocities:

```
p_dot = J(q) * q_dot
```

The manipulability index mu = |det(J)| quantifies proximity to singularities.
Two singularity conditions are detected:

- Elbow singularity: sin(t3) approaches 0 (arm fully extended or folded)
- Shoulder singularity: r approaches 0 (arm nearly vertical)

### Trapezoidal Velocity Profile (Eq. 74)

Joint trajectories are interpolated using a normalized trapezoidal profile s(t)
with three phases:

1. Acceleration: s = 0.5 * a_max * t^2 (parabolic ramp)
2. Cruise: s = s1 + v_max * (t - t1) (linear)
3. Deceleration: symmetric parabolic ramp to s = 1

When the motion is too short to reach v_max, the profile degrades to a triangular
shape (direct accel-to-decel transition).

### Pinhole Camera Model (Eq. 63-68)

The camera subsystem performs inverse projection from pixel coordinates to 3D:

```
X_w = (u - cx) * Zw / fx
Y_w = (v - cy) * Zw / fy
```

Followed by an extrinsic transform from camera frame to robot base frame:

```
p_robot = R * p_camera + t
```

where R accounts for the 90-degree downward camera mount orientation.

---

## Directory Structure

```
JINX-Joint_Inverse_N-dimensional_eXplorer/
|-- index.html                    Landing page
|-- vite.config.js                Vite multi-page configuration
|-- package.json                  Dependencies (three, vite)
|-- public/
|   |-- style.css                 Global design tokens and component styles
|   |-- favicon.svg               Application icon
|   +-- icons.svg                 Iconography
|-- src/
|   |-- main.js                   Application entry point
|   |-- math/
|   |   |-- Kinematics.js         FK, IK, Jacobian, det3, constants
|   |   |-- Trajectory.js         Trapezoidal velocity profile
|   |   +-- CameraModel.js        Pinhole camera projection and transform
|   |-- core/
|   |   +-- SceneManager.js       Three.js scene, renderer, meshes, lighting
|   |-- ui/
|   |   +-- UIController.js       DOM events, simulation loop, HUD updates
|   +-- pages/
|       +-- rrr-lamp.html         3-DOF RRR simulator page
+-- README.md
```

---

## Local Development Setup

### Prerequisites

- Node.js (version 18 or later recommended)
- npm (version 9 or later)

### Installation

```bash
git clone https://github.com/chandansaipavanpadala/JINX-Joint_Inverse_N-dimensional_eXplorer.git
cd JINX-Joint_Inverse_N-dimensional_eXplorer
npm install
```

### Development Server

```bash
npm run dev
```

Navigate to `http://localhost:5173/` to view the landing page. Select the
"3-DOF RRR Shadow Lamp" configuration to launch the simulator.

### Production Build

```bash
npm run build
npm run preview
```

---

## Robot Configurations

| Configuration         | DOF | Type | Status      |
|-----------------------|-----|------|-------------|
| 3-DOF RRR Shadow Lamp | 3   | RRR  | Available   |
| SCARA Arm             | 4   | RRPR | Planned     |
| 6-DOF Anthropomorphic | 6   | 6R   | Planned     |

---

## License

This project is open-source. See the repository for license details.
