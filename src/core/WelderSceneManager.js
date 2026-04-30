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

    // ── Scene ──
    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x070710);
    this._scene.fog = new THREE.Fog(0x070710, 2.5, 5.0);

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
    this._renderer.toneMappingExposure = 1.3;
    wrap.insertBefore(this._renderer.domElement, wrap.firstChild);

    // ── Environment map ──
    const pmrem = new THREE.PMREMGenerator(this._renderer);
    pmrem.compileEquirectangularShader();
    const envScene = new THREE.Scene();
    envScene.add(new THREE.Mesh(
      new THREE.BoxGeometry(100, 100, 100),
      new THREE.MeshBasicMaterial({ color: 0x050510, side: THREE.BackSide })
    ));
    const envL = new THREE.RectAreaLight(0xffffff, 5, 10, 10);
    envL.position.set(5, 10, 5); envL.lookAt(0, 0, 0);
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
     Lighting — identical to SCARA Studio
     ════════════════════════════════════════════════════════ */
  _initLighting() {
    const s = this._scene;
    s.add(new THREE.AmbientLight(0x6070c0, 0.90));

    const sun = new THREE.DirectionalLight(0xffffff, 1.20);
    sun.position.set(2.0, 3.5, 1.8);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0004;
    sun.shadow.camera.near = 0.1;
    sun.shadow.camera.far = 10;
    sun.shadow.camera.top = sun.shadow.camera.right = 2;
    sun.shadow.camera.bottom = sun.shadow.camera.left = -2;
    s.add(sun);

    const fill = new THREE.DirectionalLight(0xffa060, 0.55);
    fill.position.set(-2, 1.5, -0.5);
    s.add(fill);

    const rim = new THREE.DirectionalLight(0x4080ff, 0.35);
    rim.position.set(-0.5, 0.5, -2);
    s.add(rim);
  }

  /* ════════════════════════════════════════════════════════
     PBR Materials — identical to SCARA Studio
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
     Environment — Floor, Table, Grid (matched to SCARA)
     ════════════════════════════════════════════════════════ */
  _initEnvironment() {
    const s = this._scene;

    // Floor
    const floorG = new THREE.PlaneGeometry(5, 5);
    floorG.rotateX(-Math.PI / 2);
    const floor = new THREE.Mesh(floorG, new THREE.MeshStandardMaterial({ color: 0x0b0b18, roughness: 0.98 }));
    floor.position.y = -0.76;
    floor.receiveShadow = true;
    s.add(floor);
    const floorGrid = new THREE.GridHelper(4, 32, 0x18182a, 0x10101e);
    floorGrid.position.y = -0.756;
    s.add(floorGrid);

    // Wooden Table
    const tblMat = new THREE.MeshStandardMaterial({ color: 0x5A3418, roughness: 0.68, metalness: 0.05 });
    const tblTop = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.04, 0.75), tblMat);
    tblTop.position.set(0.35, -0.02, 0);
    tblTop.castShadow = tblTop.receiveShadow = true;
    s.add(tblTop);
    this._table = tblTop;

    // Table trim
    const trimM = new THREE.MeshStandardMaterial({ color: 0xA07820, roughness: 0.25, metalness: 0.60 });
    s.add(new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.005, 0.005), trimM)).position.set(0.35, -0.0025, 0.375);

    // Table legs
    const legMat = new THREE.MeshStandardMaterial({ color: 0x381E0C, roughness: 0.80, metalness: 0.04 });
    [[-0.14, 0.32], [-0.14, -0.32], [0.84, 0.32], [0.84, -0.32]].forEach(([x, z]) => {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.72, 0.05), legMat);
      leg.position.set(x, -0.40, z);
      leg.castShadow = leg.receiveShadow = true;
      s.add(leg);
    });

    // Table grid
    const tGrid = new THREE.GridHelper(1.0, 20, 0x7A5030, 0x5A3818);
    tGrid.position.set(0.35, 0.001, 0);
    s.add(tGrid);

    // Weld seam outline on the table (visual guide)
    // DH weld corners → Three.js: (dhX, dhZ, -dhY)
    const seamPts = [
      V3(0.35, 0.05, 0.15),   // corner 0
      V3(0.55, 0.05, 0.15),   // corner 1
      V3(0.55, 0.05, -0.15),  // corner 2
      V3(0.35, 0.05, -0.15),  // corner 3
      V3(0.35, 0.05, 0.15),   // close
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
