# JINX: Joint Inverse N-dimensional eXplorer

A browser-based, real-time 3D simulator for exploring robotic manipulator
kinematics, Jacobian analysis, singularity detection, and task-space trajectory
planning. Built with Three.js, vanilla JavaScript, and Vite.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Supported Robot Configurations](#supported-robot-configurations)
3. [Core Architecture](#core-architecture)
4. [Mathematical Foundations](#mathematical-foundations)
5. [Math Dashboard](#math-dashboard)
6. [Directory Structure](#directory-structure)
7. [Local Development Setup](#local-development-setup)
8. [License](#license)

---

## Project Overview

JINX provides an interactive environment for studying serial manipulator
kinematics without requiring MATLAB, ROS, or compiled simulation software.
The simulator runs entirely in the browser and supports:

- Closed-form and iterative inverse kinematics (DLS solver for N-DOF chains)
- Real-time Jacobian computation with color-coded matrix visualization
- Singularity detection and manipulability index tracking
- Trapezoidal velocity profiling with configurable limits
- Pinhole camera model with intrinsic/extrinsic parameter tuning
- Pick-and-place task automation with finite state machine control
- Waypoint-based trajectory planning for welding path execution
- Inter-tab communication via BroadcastChannel for live math dashboard sync
- Interactive link resizing with real-time kinematic mesh updates
- Real-time 3D rendering with PBR materials, shadow mapping, and environment lighting

---

## Supported Robot Configurations

| Configuration              | DOF | Joint Types | Key Features                                      |
|----------------------------|-----|-------------|---------------------------------------------------|
| 3-DOF RRR Desk Lamp        | 3   | RRR         | Closed-form IK, occlusion avoidance, camera model |
| 4-DOF SCARA Arm             | 4   | RRPR        | DLS IK, pick-and-place FSM, link resizing         |
| 6-DOF Welding Robot         | 6   | 6R          | DLS IK, welding spark effects, waypoint paths     |

All three configurations share a unified N-DOF kinematics engine
(`KinematicsNDOF.js`) that computes forward kinematics, the 6xN geometric
Jacobian, and damped least-squares inverse kinematics for arbitrary serial
chains defined by standard DH parameter tables.

---

## Core Architecture

The codebase follows a strict separation of concerns across four layers:

```
index.html                          Landing page (robot selector)
src/pages/rrr-lamp.html             3-DOF RRR lamp simulator
src/pages/scara.html                4-DOF SCARA arm simulator
src/pages/welder.html               6-DOF welding robot simulator
src/pages/math-dashboard.html       Live math dashboard (cross-tab)
src/math/                           Pure mathematical functions (zero DOM)
src/core/                           3D scene management (Three.js)
src/ui/                             DOM event handling, HUD, simulation loop
src/logic/                          Task automation (FSM, waypoints, welding)
public/                             Static assets (CSS, icons)
```

### Data Flow

```
  User Input (sliders / drag / task FSM)
       |
       v
  UIController._updateScene()
       |
       +---> fk(q, dhTable)              [src/math/KinematicsNDOF.js]
       +---> ik_dls(target, q, dhTable)  [src/math/KinematicsNDOF.js]
       +---> jacobian(q, dhTable)        [src/math/KinematicsNDOF.js]
       +---> trapProfile()               [src/math/Trajectory.js]
       |
       v
  SceneManager.updateScene(q, pTarget)   [src/core/*SceneManager.js]
       |
       +---> DH chain FK for mesh positioning
       +---> Mesh repositioning (joints, links, end-effector)
       +---> Trail rendering, spark effects (welder)
       |
       v
  BroadcastChannel('jinx_math_sync')
       |
       v
  Math Dashboard (separate tab)          [src/pages/math-dashboard.html]
       |
       +---> Real-time FK, IK, Jacobian matrix display
       +---> Chart.js analytics (error, velocity, manipulability)
       +---> Bidirectional slider control via 'jinx_math_cmd' channel
```

---

## Mathematical Foundations

### Denavit-Hartenberg Convention

All robots use the standard DH convention. Each joint i is parameterized by
four quantities: link length a_i, link twist alpha_i, link offset d_i, and
joint angle theta_i. The homogeneous transformation from frame i-1 to frame i
is the product of four elementary transforms.

### 3-DOF RRR Desk Lamp

| i | a_i (m) | alpha_i | d_i (m) | theta_i   |
|---|---------|---------|---------|-----------|
| 1 | 0       | pi/2    | 0.15    | theta_1*  |
| 2 | 0.30    | 0       | 0       | theta_2*  |
| 3 | 0.24    | 0       | 0       | theta_3*  |

Forward kinematics (closed-form):

```
r  = L2 * cos(t2) + L3 * cos(t2 + t3)
x  = r * cos(t1)
y  = r * sin(t1)
z  = L1 + L2 * sin(t2) + L3 * sin(t2 + t3)
```

Inverse kinematics (closed-form geometric):

```
t1 = atan2(yd, xd)
r  = sqrt(xd^2 + yd^2),  zp = zd - L1
C3 = (r^2 + zp^2 - L2^2 - L3^2) / (2 * L2 * L3)
t3 = atan2(e * sqrt(1 - C3^2), C3)      [e = +/-1 for elbow config]
t2 = atan2(zp, r) - atan2(L3*sin(t3), L2 + L3*cos(t3))
```

Returns null when |C3| > 1 (target unreachable).

### 4-DOF SCARA Arm

| i | a_i (m) | alpha_i | d_i (m) | theta_i   |
|---|---------|---------|---------|-----------|
| 1 | 0       | 0       | 0.35    | theta_1*  |
| 2 | 0.35    | pi      | 0       | theta_2*  |
| 3 | 0       | 0       | d_3*    | 0         |
| 4 | 0       | 0       | 0       | theta_4*  |

Uses damped least-squares (DLS) iterative IK with configurable damping
factor lambda and maximum iteration count.

### 6-DOF Welding Robot

| i | a_i (m) | alpha_i | d_i (m) | theta_i   |
|---|---------|---------|---------|-----------|
| 1 | 0       | pi/2    | 0.20    | theta_1*  |
| 2 | 0.30    | 0       | 0       | theta_2*  |
| 3 | 0       | pi/2    | 0       | theta_3*  |
| 4 | 0       | -pi/2   | 0.20    | theta_4*  |
| 5 | 0       | pi/2    | 0       | theta_5*  |
| 6 | 0       | 0       | 0.08    | theta_6*  |

Uses the same DLS solver. Includes waypoint-based trajectory planning for
automated welding path execution with spark particle effects.

### Analytical Jacobian

The 6xN geometric Jacobian J(q) maps joint velocities to Cartesian velocities:

```
[v]     [Jv]
[w]  =  [Jw] * q_dot
```

For position-only tasks, the top 3 rows (Jv) are used. The manipulability
index mu = sqrt(det(Jv * Jv^T)) quantifies proximity to singularities.

Singularity conditions detected:

- Elbow singularity: sin(t3) approaches 0 (arm fully extended or folded)
- Shoulder singularity: planar reach approaches 0 (arm nearly vertical)
- Wrist singularity: alignment of wrist axes (6-DOF only)

### Damped Least-Squares IK

For N-DOF chains where closed-form solutions are unavailable, the DLS
iterative solver computes:

```
delta_q = J^T * (J * J^T + lambda^2 * I)^(-1) * e
```

where e is the position error vector and lambda is the damping factor that
prevents instability near singularities.

### Trapezoidal Velocity Profile

Joint trajectories use a normalized trapezoidal profile s(t) with three
phases: acceleration (parabolic ramp), cruise (linear), and deceleration
(symmetric parabolic ramp). When the motion is too short to reach v_max,
the profile degrades to a triangular shape.

### Pinhole Camera Model

The perception camera performs inverse projection from pixel coordinates
to 3D world coordinates using intrinsic parameters (focal length, principal
point) and an extrinsic transform from camera frame to robot base frame.

---

## Math Dashboard

The Math Dashboard is an independent HTML page that opens in a separate
browser tab and receives real-time telemetry from any active robot simulator
via the BroadcastChannel API.

Features:

- **Forward Kinematics Panel**: End-effector position, joint configuration
  cards, interactive FK sliders that drive the robot live, and full DH
  parameter table (dynamically generated per robot type).
- **Inverse Kinematics Panel**: Target position sliders, IK solution display
  with convergence status, iteration count, and FK verification error.
- **Jacobian Panel**: Color-coded NxN Jacobian matrix with per-cell
  magnitude-based coloring (green for positive, red for negative, grey for
  near-zero). Includes determinant, manipulability index, reach metric,
  manipulability bar, and singularity warning.
- **Analytics Panel**: Three Chart.js time-series graphs (position tracking
  error, end-effector velocity, manipulability over time) with 100-point
  rolling window and 3-frame throttled rendering.

The dashboard auto-detects the robot type from the URL query parameter
(`?robot=rrr`, `?robot=scara`, `?robot=welder`) and reconfigures all panels
accordingly. Bidirectional control is supported: adjusting sliders on the
dashboard sends commands back to the simulator via a separate command channel.

---

## Directory Structure

```
JINX-Joint_Inverse_N-dimensional_eXplorer/
|-- index.html                    Landing page (robot selector)
|-- vite.config.js                Vite multi-page build configuration
|-- package.json                  Dependencies (three, chart.js, vite)
|-- public/
|   |-- style.css                 Global design tokens and component styles
|   |-- favicon.svg               Application icon
|   +-- icons.svg                 Iconography
|-- src/
|   |-- main.js                   Entry point (3-DOF RRR)
|   |-- scara-main.js             Entry point (4-DOF SCARA)
|   |-- welder-main.js            Entry point (6-DOF Welder)
|   |-- math/
|   |   |-- Kinematics.js         Closed-form FK/IK/Jacobian (3-DOF RRR)
|   |   |-- KinematicsNDOF.js     Generalized N-DOF FK, Jacobian, DLS IK
|   |   |-- Trajectory.js         Trapezoidal velocity profile
|   |   +-- CameraModel.js        Pinhole camera projection and transform
|   |-- core/
|   |   |-- SceneManager.js       3-DOF RRR scene (Three.js)
|   |   |-- ScaraSceneManager.js  4-DOF SCARA scene (Three.js)
|   |   +-- WelderSceneManager.js 6-DOF welder scene (Three.js)
|   |-- ui/
|   |   |-- UIController.js       3-DOF RRR DOM events and HUD
|   |   |-- ScaraUIController.js  4-DOF SCARA DOM events and HUD
|   |   |-- WelderUIController.js 6-DOF welder DOM events and HUD
|   |   |-- CameraPanel.js        Perception camera floating panel
|   |   |-- FloatingPanel.js      Draggable panel base class
|   |   +-- WaypointPanel.js      Waypoint editor panel (welder)
|   |-- logic/
|   |   |-- PickAndPlace.js       Pick-and-place FSM (SCARA)
|   |   |-- WaypointTask.js       Waypoint trajectory executor
|   |   +-- WeldingTask.js        Welding path automation
|   +-- pages/
|       |-- rrr-lamp.html         3-DOF RRR simulator page
|       |-- scara.html            4-DOF SCARA simulator page
|       |-- welder.html           6-DOF welding robot simulator page
|       +-- math-dashboard.html   Live math dashboard
+-- README.md
```

---

## Local Development Setup

### Prerequisites

- Node.js (version 18 or later)
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

Navigate to `http://localhost:5173/` to view the landing page. Select any
robot configuration to launch its simulator. The math dashboard can be
opened from within any simulator via the dashboard button, or directly at
`http://localhost:5173/src/pages/math-dashboard.html?robot=scara`.

### Production Build

```bash
npm run build
npm run preview
```

---

## License

This project is released under the MIT License. See the LICENSE file for
full terms and conditions.
