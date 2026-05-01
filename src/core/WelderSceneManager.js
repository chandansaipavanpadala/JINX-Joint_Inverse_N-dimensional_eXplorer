/**
 * WelderSceneManager.js — Three.js 3D Scene Controller for the 6-DOF Welding Robot
 *
 * Renders a 6-DOF Anthropomorphic arm driven by the generalized
 * KinematicsNDOF engine, with the same JINX Studio aesthetic as ScaraSceneManager.
 *
 * COORDINATE MAPPING (critical):
 *   DH convention:   X,Y = horizontal plane, Z = vertical (up)
 *   Three.js:        X,Z = horizontal plane, Y = vertical (up)
 *
 *   Three.x =  DH.x  (T[12])
 *   Three.y =  DH.z  (T[14])   ← vertical axis swap
 *   Three.z = -DH.y  (-T[13])  ← preserves right-handedness
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { fk, WELDING_DH_CONFIG } from '../math/KinematicsNDOF.js';

const V3 = (x, y, z) => new THREE.Vector3(x, y, z);

// DH config constants for the 6-DOF arm
const BASE_H = 0.25;   // d1 — base column height
const L_UPPER = 0.35;  // a2 — upper arm
const L_FORE = 0.25;   // a3 — forearm

export class WelderSceneManager {
  constructor(containerId) {
    const wrap = document.getElementById(containerId);

    // ── Scene — dark, smoky welding booth atmosphere ──
    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x06060c);
    this._scene.fog = new THREE.FogExp2(0x06060c, 0.28);

    // ── Camera ──
    this._camera = new THREE.PerspectiveCamera(50, wrap.clientWidth / wrap.clientHeight, 0.01, 10);
    this._camera.position.set(1.2, 1.0, 1.2);

    // ── Renderer ──
    this._renderer = new THREE.WebGLRenderer({ antialias: true });
    this._renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this._renderer.shadowMap.enabled = true;
    this._renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this._renderer.setSize(wrap.clientWidth, wrap.clientHeight);
    this._renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this._renderer.toneMappingExposure = 0.95;
    wrap.insertBefore(this._renderer.domElement, wrap.firstChild);

    // ── Environment map ──
    const pmrem = new THREE.PMREMGenerator(this._renderer);
    pmrem.compileEquirectangularShader();
    const envScene = new THREE.Scene();
    envScene.add(new THREE.Mesh(
      new THREE.BoxGeometry(100, 100, 100),
      new THREE.MeshBasicMaterial({ color: 0x030308, side: THREE.BackSide })
    ));
    // Dim overhead panel — just enough for metallic reflections
    const envL = new THREE.RectAreaLight(0x8090b0, 2.5, 8, 8);
    envL.position.set(0, 12, 0); envL.lookAt(0, 0, 0);
    envScene.add(envL);
    this._scene.environment = pmrem.fromScene(envScene).texture;

    // ── OrbitControls ──
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.target.set(0.15, 0.15, 0);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.07;
    this._controls.minDistance = 0.2;
    this._controls.maxDistance = 4;
    this._camera.lookAt(this._controls.target);

    // ── Init sub-systems ──
    this._initLighting();
    this._initMaterials();
    this._initEnvironment();
    this._initRobotGeometry();
    this._initTargetAndTrail();

    // ── Resize ──
    this._wrap = wrap;
    window.addEventListener('resize', () => {
      const w = this._wrap.clientWidth, h = this._wrap.clientHeight;
      this._camera.aspect = w / h;
      this._camera.updateProjectionMatrix();
      this._renderer.setSize(w, h);
    });
  }

  /* ════════════════════════════════════════════════════════
     Lighting — Dark, moody welding booth
     Low ambient so the spark PointLight dominates when active.
     ════════════════════════════════════════════════════════ */
  _initLighting() {
    const s = this._scene;

    // Very dim ambient — booth feels dark until spark ignites
    s.add(new THREE.AmbientLight(0x304060, 0.45));

    // Single overhead — dim blue-white, casts shadows
    const sun = new THREE.DirectionalLight(0xb0c0e0, 0.65);
    sun.position.set(0.5, 4.0, 0.5);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0003;
    sun.shadow.camera.near = 0.1;
    sun.shadow.camera.far = 12;
    sun.shadow.camera.top = sun.shadow.camera.right = 2.5;
    sun.shadow.camera.bottom = sun.shadow.camera.left = -2.5;
    s.add(sun);

    // Cool rim from behind — silhouette the robot
    const rim = new THREE.DirectionalLight(0x304080, 0.30);
    rim.position.set(-1, 1.5, -2);
    s.add(rim);

    // Faint warm ground bounce
    const bounce = new THREE.DirectionalLight(0x503020, 0.15);
    bounce.position.set(0, -0.5, 0);
    s.add(bounce);
  }

  /* ════════════════════════════════════════════════════════
     PBR Materials
     ════════════════════════════════════════════════════════ */
  _initMaterials() {
    this._mBase = new THREE.MeshPhysicalMaterial({
      color: 0x1a1a2e, roughness: 0.25, metalness: 0.85,
      clearcoat: 0.8, clearcoatRoughness: 0.1, envMapIntensity: 1.5
    });
    this._mArm = new THREE.MeshPhysicalMaterial({
      color: 0xe8e8e8, roughness: 0.12, metalness: 0.6,
      clearcoat: 1.0, clearcoatRoughness: 0.05, envMapIntensity: 1.2
    });
    this._mJoint = new THREE.MeshPhysicalMaterial({
      color: 0xC8A200, roughness: 0.05, metalness: 0.95,
      clearcoat: 1.0, envMapIntensity: 2.0
    });
    this._mEE = new THREE.MeshPhysicalMaterial({
      color: 0xCC2222, roughness: 0.15, metalness: 0.5,
      clearcoat: 1.0, clearcoatRoughness: 0.05, envMapIntensity: 1.4
    });
    this._mTgt = new THREE.MeshPhysicalMaterial({
      color: 0x00e5ff, emissive: 0x0088ff, emissiveIntensity: 1.0,
      roughness: 0.1, metalness: 0.8, clearcoat: 1.0
    });
  }

  /* ════════════════════════════════════════════════════════
     Environment — Industrial Welding Booth

     CRITICAL: Welding table surface at Y ≈ 0.0 (same as old table)
     so WeldingTask.js seam coordinates remain valid.
     ════════════════════════════════════════════════════════ */
  _initEnvironment() {
    const s = this._scene;

    // ── 1. Heavy concrete floor with diamond-plate look ──
    const floorMat = new THREE.MeshPhysicalMaterial({
      color: 0x14161c, roughness: 0.82, metalness: 0.20,
      clearcoat: 0.15, clearcoatRoughness: 0.7,
    });
    const floorG = new THREE.PlaneGeometry(8, 8);
    floorG.rotateX(-Math.PI / 2);
    const floor = new THREE.Mesh(floorG, floorMat);
    floor.position.y = -0.76;
    floor.receiveShadow = true;
    s.add(floor);

    // Floor grid — very subtle, dark industrial
    const floorGrid = new THREE.GridHelper(6, 48, 0x161a24, 0x101418);
    floorGrid.position.y = -0.756;
    s.add(floorGrid);

    // ── 2. Hazard floor markings (yellow/black safety stripes) ──
    const hazardMat = new THREE.MeshStandardMaterial({
      color: 0xcc9900, roughness: 0.85, metalness: 0.05,
      transparent: true, opacity: 0.20
    });
    const hazardZone = new THREE.Mesh(
      new THREE.RingGeometry(0.55, 0.58, 64),
      hazardMat
    );
    hazardZone.rotation.x = -Math.PI / 2;
    hazardZone.position.y = -0.754;
    s.add(hazardZone);

    // ── 3. Heavy-duty welding pedestal (robot base mount) ──
    const pedestalMat = new THREE.MeshPhysicalMaterial({
      color: 0x22252e, roughness: 0.40, metalness: 0.92,
      clearcoat: 0.3, clearcoatRoughness: 0.4, envMapIntensity: 1.2
    });
    // Top plate — Y top face ≈ 0.0
    const pedTop = new THREE.Mesh(
      new THREE.CylinderGeometry(0.11, 0.11, 0.04, 32),
      pedestalMat
    );
    pedTop.position.set(0, -0.02, 0);
    pedTop.castShadow = pedTop.receiveShadow = true;
    s.add(pedTop);

    // Pedestal column — reinforced steel
    const pedCol = new THREE.Mesh(
      new THREE.CylinderGeometry(0.065, 0.085, 0.70, 24),
      pedestalMat
    );
    pedCol.position.set(0, -0.39, 0);
    pedCol.castShadow = true;
    s.add(pedCol);

    // Floor mount plate
    const pedBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.13, 0.14, 0.02, 32),
      pedestalMat
    );
    pedBase.position.set(0, -0.75, 0);
    pedBase.receiveShadow = true;
    s.add(pedBase);

    // Accent ring
    const accentMat = new THREE.MeshPhysicalMaterial({
      color: 0xC8A200, roughness: 0.05, metalness: 0.95,
      clearcoat: 1.0, envMapIntensity: 2.0
    });
    const pedRing = new THREE.Mesh(new THREE.TorusGeometry(0.11, 0.004, 8, 48), accentMat);
    pedRing.rotation.x = Math.PI / 2;
    pedRing.position.set(0, 0.0, 0);
    s.add(pedRing);

    // ── 4. Heavy-duty welding table (thick steel slab) ──
    const TBL_CX = 0.45, TBL_CZ = 0.0;
    const TBL_W = 0.60, TBL_D = 0.50;
    const TBL_SURFACE_Y = -0.02; // matches old table top

    const tblMat = new THREE.MeshPhysicalMaterial({
      color: 0x2a2d35, roughness: 0.50, metalness: 0.88,
      clearcoat: 0.2, envMapIntensity: 1.0
    });
    const tblTop = new THREE.Mesh(
      new THREE.BoxGeometry(TBL_W, 0.05, TBL_D),
      tblMat
    );
    tblTop.position.set(TBL_CX, TBL_SURFACE_Y, TBL_CZ);
    tblTop.castShadow = tblTop.receiveShadow = true;
    s.add(tblTop);
    this._table = tblTop;

    // Table edge trim — bright accent
    const trimMat = new THREE.MeshPhysicalMaterial({
      color: 0x8090a0, roughness: 0.15, metalness: 0.95,
      clearcoat: 0.6, envMapIntensity: 1.4
    });
    // Front and back edges
    [-1, 1].forEach(side => {
      const trim = new THREE.Mesh(
        new THREE.BoxGeometry(TBL_W, 0.008, 0.008),
        trimMat
      );
      trim.position.set(TBL_CX, TBL_SURFACE_Y + 0.025, TBL_CZ + side * (TBL_D / 2));
      s.add(trim);
    });

    // Table legs — thick industrial steel
    const tLegMat = new THREE.MeshPhysicalMaterial({
      color: 0x2a2d35, roughness: 0.50, metalness: 0.85,
      envMapIntensity: 0.8
    });
    [
      [TBL_CX - TBL_W/2 + 0.04, TBL_CZ - TBL_D/2 + 0.04],
      [TBL_CX - TBL_W/2 + 0.04, TBL_CZ + TBL_D/2 - 0.04],
      [TBL_CX + TBL_W/2 - 0.04, TBL_CZ - TBL_D/2 + 0.04],
      [TBL_CX + TBL_W/2 - 0.04, TBL_CZ + TBL_D/2 - 0.04],
    ].forEach(([x, z]) => {
      const leg = new THREE.Mesh(
        new THREE.BoxGeometry(0.04, 0.70, 0.04),
        tLegMat
      );
      leg.position.set(x, TBL_SURFACE_Y - 0.375, z);
      leg.castShadow = leg.receiveShadow = true;
      s.add(leg);
      // Foot pads
      const foot = new THREE.Mesh(
        new THREE.BoxGeometry(0.06, 0.01, 0.06),
        tLegMat
      );
      foot.position.set(x, -0.755, z);
      foot.receiveShadow = true;
      s.add(foot);
    });

    // Table surface grid
    const tGrid = new THREE.GridHelper(0.5, 10, 0x3a3f4a, 0x2a2e38);
    tGrid.position.set(TBL_CX, TBL_SURFACE_Y + 0.026, TBL_CZ);
    s.add(tGrid);

    // ── 5. Safety screens / blast shields ──
    const screenMat = new THREE.MeshPhysicalMaterial({
      color: 0x0a1020,
      roughness: 0.6, metalness: 0.3,
      transparent: true, opacity: 0.35,
      side: THREE.DoubleSide,
    });
    const screenFrameMat = new THREE.MeshPhysicalMaterial({
      color: 0x3a3d48, roughness: 0.35, metalness: 0.90,
      envMapIntensity: 1.0
    });

    // Back screen
    const backScreen = new THREE.Mesh(
      new THREE.PlaneGeometry(1.6, 1.2),
      screenMat
    );
    backScreen.position.set(0.3, -0.16, -0.55);
    backScreen.receiveShadow = true;
    s.add(backScreen);
    // Back screen frame (top rail)
    const bsFrame = new THREE.Mesh(
      new THREE.BoxGeometry(1.65, 0.025, 0.025),
      screenFrameMat
    );
    bsFrame.position.set(0.3, 0.44, -0.55);
    bsFrame.castShadow = true;
    s.add(bsFrame);
    // Back screen frame (bottom rail)
    const bsFrameB = new THREE.Mesh(
      new THREE.BoxGeometry(1.65, 0.025, 0.025),
      screenFrameMat
    );
    bsFrameB.position.set(0.3, -0.76, -0.55);
    s.add(bsFrameB);
    // Back screen vertical posts
    [-1, 1].forEach(side => {
      const post = new THREE.Mesh(
        new THREE.BoxGeometry(0.025, 1.25, 0.025),
        screenFrameMat
      );
      post.position.set(0.3 + side * 0.82, -0.16, -0.55);
      post.castShadow = true;
      s.add(post);
    });

    // Side screen (right)
    const sideScreen = new THREE.Mesh(
      new THREE.PlaneGeometry(0.8, 1.2),
      screenMat
    );
    sideScreen.position.set(1.1, -0.16, -0.15);
    sideScreen.rotation.y = Math.PI / 2;
    sideScreen.receiveShadow = true;
    s.add(sideScreen);
    // Side screen frame
    const ssFrame = new THREE.Mesh(
      new THREE.BoxGeometry(0.025, 1.25, 0.025),
      screenFrameMat
    );
    ssFrame.position.set(1.1, -0.16, -0.55);
    ssFrame.castShadow = true;
    s.add(ssFrame);

    // ── 6. Weld seam outline on table (visual guide) ──
    const seamPts = [
      V3(0.35, 0.05, 0.15),
      V3(0.55, 0.05, 0.15),
      V3(0.55, 0.05, -0.15),
      V3(0.35, 0.05, -0.15),
      V3(0.35, 0.05, 0.15),
    ];
    const seamGeo = new THREE.BufferGeometry().setFromPoints(seamPts);
    const seamLine = new THREE.Line(seamGeo, new THREE.LineDashedMaterial({
      color: 0xff6600, dashSize: 0.02, gapSize: 0.01,
      transparent: true, opacity: 0.5
    }));
    seamLine.computeLineDistances();
    s.add(seamLine);
  }

  /* ════════════════════════════════════════════════════════
     6-DOF Robot Geometry — PBR materials, proper sizing
     ════════════════════════════════════════════════════════ */
  _initRobotGeometry() {
    const s = this._scene;

    // Joint meshes (7 frames: base + 6 joints)
    this._joints = [];
    for (let i = 0; i <= 6; i++) {
      const r = i === 0 ? 0.045 : (i <= 3 ? 0.035 : 0.025);
      const h = i === 0 ? 0.05 : (i <= 3 ? 0.04 : 0.03);
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(r, r, h, 24),
        i === 0 ? this._mBase : this._mJoint
      );
      mesh.castShadow = true;
      s.add(mesh);
      this._joints.push(mesh);
    }

    // Link meshes (6 links connecting successive joints)
    this._links = [];
    for (let i = 0; i < 6; i++) {
      const r = i <= 2 ? 0.018 : 0.012;
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(r, r, 1, 16),
        this._mArm
      );
      mesh.castShadow = true;
      s.add(mesh);
      this._links.push(mesh);
    }

    // End-effector cone (welding torch tip)
    this._tool = new THREE.Mesh(
      new THREE.ConeGeometry(0.015, 0.05, 16),
      this._mEE
    );
    this._tool.castShadow = true;
    s.add(this._tool);

    // Spark effect — PointLight + glow sphere at tool tip
    this._sparkLight = new THREE.PointLight(0x00e5ff, 0, 0.8);
    this._sparkLight.castShadow = false;
    const sparkCore = new THREE.Mesh(
      new THREE.SphereGeometry(0.01, 12, 12),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    );
    this._sparkLight.add(sparkCore);

    // Spark glow sprite
    const sparkGlow = new THREE.Mesh(
      new THREE.SphereGeometry(0.025, 12, 12),
      new THREE.MeshBasicMaterial({
        color: 0x00e5ff, transparent: true, opacity: 0.4, depthWrite: false
      })
    );
    this._sparkLight.add(sparkGlow);
    this._sparkLight.visible = false;
    s.add(this._sparkLight);
  }

  /* ════════════════════════════════════════════════════════
     Target Indicator + Trail
     ════════════════════════════════════════════════════════ */
  _initTargetAndTrail() {
    const s = this._scene;

    // Target sphere
    this._targetMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 20, 20),
      this._mTgt
    );
    this._targetMesh.visible = false;
    s.add(this._targetMesh);

    // Weld trail (tracks the EE path during welding)
    this._trailPts = [];
    this._trailLine = null;
    this._trailMat = new THREE.LineBasicMaterial({
      color: 0x00e5ff, transparent: true, opacity: 0.7
    });

    // Waypoint visualization objects (managed by updateWaypointVisualization)
    this._wpSpheres = [];
    this._wpLine = null;
    this._wpSphereMat = new THREE.MeshPhysicalMaterial({
      color: 0xffcc00, emissive: 0xff9900, emissiveIntensity: 0.6,
      roughness: 0.1, metalness: 0.5, clearcoat: 1.0, transparent: true, opacity: 0.85
    });
    this._wpLineMat = new THREE.LineDashedMaterial({
      color: 0xffcc00, dashSize: 0.015, gapSize: 0.008,
      transparent: true, opacity: 0.55
    });
  }

  /* ════════════════════════════════════════════════════════
     Waypoint Visualization — spheres + connecting line
     Call with an array of { x, y, z } in DH coords.
     Pass empty array to clear.
     ════════════════════════════════════════════════════════ */
  updateWaypointVisualization(waypoints) {
    const s = this._scene;

    // Remove old spheres
    this._wpSpheres.forEach(m => { s.remove(m); m.geometry.dispose(); });
    this._wpSpheres = [];

    // Remove old line
    if (this._wpLine) {
      s.remove(this._wpLine);
      this._wpLine.geometry.dispose();
      this._wpLine = null;
    }

    if (!waypoints || waypoints.length === 0) return;

    // FIX 6: create a new geometry per sphere (not shared) to avoid
    // double-dispose when the list is cleared — all meshes would share
    // the same BufferGeometry and the 2nd dispose would crash WebGL.
    const pts = [];

    waypoints.forEach(wp => {
      // DH → Three.js:  x=DH.x, y=DH.z, z=-DH.y
      const pos = V3(wp.x, wp.z, -wp.y);
      pts.push(pos);

      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.012, 14, 14), // unique geometry per sphere
        this._wpSphereMat
      );
      mesh.position.copy(pos);
      s.add(mesh);
      this._wpSpheres.push(mesh);
    });

    // Connecting line
    if (pts.length >= 2) {
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      this._wpLine = new THREE.Line(geo, this._wpLineMat);
      this._wpLine.computeLineDistances();
      s.add(this._wpLine);
    }
  }

  /* ════════════════════════════════════════════════════════
     Spark Control
     ════════════════════════════════════════════════════════ */
  setSparkActive(isActive) {
    if (this._sparkLight) {
      this._sparkLight.visible = isActive;
      this._sparkLight.intensity = isActive ? 3 : 0;
    }
  }

  /* ════════════════════════════════════════════════════════
     Add Trail Point (called during welding)
     ════════════════════════════════════════════════════════ */
  addTrailPoint(pos3) {
    this._trailPts.push(pos3.clone());
    if (this._trailPts.length < 2) return;

    // Rebuild trail line
    if (this._trailLine) {
      this._scene.remove(this._trailLine);
      this._trailLine.geometry.dispose();
    }
    const geo = new THREE.BufferGeometry().setFromPoints(this._trailPts);
    this._trailLine = new THREE.Line(geo, this._trailMat);
    this._scene.add(this._trailLine);
  }

  resetTrail() {
    this._trailPts = [];
    if (this._trailLine) {
      this._scene.remove(this._trailLine);
      this._trailLine.geometry.dispose();
      this._trailLine = null;
    }
  }

  /* ════════════════════════════════════════════════════════
     Utility: align cylinder between two 3D points
     ════════════════════════════════════════════════════════ */
  _alignCylinder(mesh, pStart, pEnd) {
    const dist = pStart.distanceTo(pEnd);
    if (dist < 0.001) { mesh.scale.set(0.001, 0.001, 0.001); return; }
    mesh.scale.set(1, dist, 1);
    mesh.position.copy(pStart).lerp(pEnd, 0.5);
    mesh.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      pEnd.clone().sub(pStart).normalize()
    );
  }

  /* ════════════════════════════════════════════════════════
     updateScene — maps FK transforms to Three.js meshes

     DH → Three.js:
       Three.x =  DH.x  (T[12])
       Three.y =  DH.z  (T[14])
       Three.z = -DH.y  (-T[13])
     ════════════════════════════════════════════════════════ */
  updateScene(q, pTgt3) {
    const { jointTransforms: JT } = fk(q, WELDING_DH_CONFIG);

    // Convert DH transform → Three.js position
    const dh2three = (T) => V3(T[12], T[14], -T[13]);

    // Extract Three.js positions for all 7 frames (base + 6 joints)
    const positions = [];
    for (let i = 0; i <= 6; i++) {
      const p = dh2three(JT[i]);
      positions.push(p);

      // Position joint meshes
      this._joints[i].position.copy(p);

      // Orient joint cylinders to align with the Z-axis of each DH frame
      const zAxis = V3(JT[i][8], JT[i][10], -JT[i][9]); // DH Z → Three.js
      this._joints[i].quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 1, 0), zAxis.normalize()
      );
    }

    // Stretch link cylinders between successive joints
    for (let i = 0; i < 6; i++) {
      this._alignCylinder(this._links[i], positions[i], positions[i + 1]);
    }

    // End-effector tool — at final frame, oriented along its Z-axis
    const eePos = positions[6];
    const eeZ = V3(JT[6][8], JT[6][10], -JT[6][9]).normalize();
    this._tool.position.copy(eePos);
    // Cone default points along +Y, we want it along eeZ
    this._tool.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), eeZ);

    // Spark — at the tip of the tool (0.025 m along eeZ from eePos)
    const tipOffset = eeZ.clone().multiplyScalar(0.03);
    this._sparkLight.position.copy(eePos).add(tipOffset);

    // Store EE world position for trail
    this._eeWorldPos = eePos.clone();

    // Target sphere (DH coordinates [x,y,z] → Three.js)
    if (pTgt3) {
      this._targetMesh.visible = true;
      this._targetMesh.position.set(pTgt3[0], pTgt3[2], -pTgt3[1]);
    } else {
      this._targetMesh.visible = false;
    }
  }

  /* ════════════════════════════════════════════════════════
     Render Loop
     ════════════════════════════════════════════════════════ */
  start() {
    const loop = () => {
      requestAnimationFrame(loop);
      this._controls.update();

      // Flicker the spark for realism
      if (this._sparkLight.visible) {
        this._sparkLight.intensity = 2.5 + Math.random() * 1.5;
      }

      this._renderer.render(this._scene, this._camera);
    };
    loop();
  }

  /* ════════════════════════════════════════════════════════
     Getters
     ════════════════════════════════════════════════════════ */
  get camera()   { return this._camera; }
  get scene()    { return this._scene; }
  get renderer() { return this._renderer; }
  get controls() { return this._controls; }
  get table()    { return this._table; }
  get eeWorldPos() { return this._eeWorldPos; }
}
