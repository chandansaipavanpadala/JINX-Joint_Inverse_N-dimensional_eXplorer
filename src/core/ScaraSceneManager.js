/**
 * ScaraSceneManager.js — Three.js 3D Scene Controller for the SCARA Arm
 *
 * Renders a 4-DOF SCARA (R-R-P-R) robot driven by the generalized
 * KinematicsNDOF engine. Zero DOM text manipulation.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { fk, SCARA_DH_CONFIG } from '../math/KinematicsNDOF.js';

const TRAIL_N = 80;
const V3 = (x, y, z) => new THREE.Vector3(x, y, z);

// DH config constants for mesh sizing
const BASE_H  = 0.35;  // d1 — base column height
const L1      = 0.30;  // a1 — upper arm length
const L2      = 0.25;  // a2 — forearm length
const STROKE  = 0.20;  // max prismatic stroke

export default class ScaraSceneManager {

  constructor(canvasId = 'cw', povId = 'camPov') {
    const wrap = document.getElementById(canvasId);
    const povWrap = document.getElementById(povId);

    // ── Scene ──
    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x070710);
    this._scene.fog = new THREE.Fog(0x070710, 2.5, 5.0);

    // ── Camera ──
    this._camera = new THREE.PerspectiveCamera(44, wrap.clientWidth / wrap.clientHeight, 0.01, 10);
    this._camera.position.set(0.9, 1.0, 0.9);

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
    this._controls.target.set(0.20, 0.25, 0);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.07;
    this._controls.minDistance = 0.2;
    this._controls.maxDistance = 4;
    this._camera.lookAt(this._controls.target);

    // ── POV Camera ──
    this._povRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this._povRenderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this._povRenderer.shadowMap.enabled = true;
    this._povRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    this._povRenderer.toneMappingExposure = 1.1;
    this._povRenderer.setSize(200, 150);
    povWrap.appendChild(this._povRenderer.domElement);
    this._povCamera = new THREE.PerspectiveCamera(55, 200 / 150, 0.01, 10);
    this._povCamera.position.set(0.0, 0.60, 0.40);
    this._povCamera.lookAt(0.30, 0.0, 0.0);

    // ── Init sub-systems ──
    this._initLighting();
    this._initMaterials();
    this._initEnvironment();
    this._initScaraGeometry();
    this._initTargetAndTrail();

    // ── Drag plane (horizontal at table surface y ≈ 0.025) ──
    this._dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -0.025);

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
     Lighting
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
     Materials
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
    this._mShaft = new THREE.MeshPhysicalMaterial({
      color: 0x909090, roughness: 0.15, metalness: 0.9,
      envMapIntensity: 1.0
    });
    this._mEE = new THREE.MeshPhysicalMaterial({
      color: 0xCC2222, roughness: 0.15, metalness: 0.5,
      clearcoat: 1.0, clearcoatRoughness: 0.05, envMapIntensity: 1.4
    });
    this._mTgt = new THREE.MeshPhysicalMaterial({
      color: 0x00e5ff, emissive: 0x0088ff, emissiveIntensity: 1.0,
      roughness: 0.1, metalness: 0.8, clearcoat: 1.0
    });
    this._mTrail = new THREE.MeshBasicMaterial({
      color: 0x00e5ff, transparent: true, opacity: 0.6, depthWrite: false
    });
    this._mBeam = new THREE.LineDashedMaterial({
      color: 0x00e5ff, dashSize: 0.02, gapSize: 0.01
    });
  }

  /* ════════════════════════════════════════════════════════
     Environment (Floor, Table, Reachability)
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

    // Table
    const tblMat = new THREE.MeshStandardMaterial({ color: 0x5A3418, roughness: 0.68, metalness: 0.05 });
    const tblTop = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.04, 0.75), tblMat);
    tblTop.position.set(0.35, -0.02, 0);
    tblTop.castShadow = tblTop.receiveShadow = true;
    s.add(tblTop);

    const trimM = new THREE.MeshStandardMaterial({ color: 0xA07820, roughness: 0.25, metalness: 0.60 });
    s.add(new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.005, 0.005), trimM)).position.set(0.35, -0.0025, 0.375);

    // Legs
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

    // Reachability ring (SCARA works in horizontal plane)
    const rMax = L1 + L2;
    const rMin = Math.abs(L1 - L2);
    const reachOuter = new THREE.RingGeometry(rMin, rMax, 64);
    reachOuter.rotateX(-Math.PI / 2);
    const reachMesh = new THREE.Mesh(reachOuter, new THREE.MeshStandardMaterial({
      color: 0x00e5ff, transparent: true, opacity: 0.04,
      side: THREE.DoubleSide, depthWrite: false
    }));
    reachMesh.position.y = 0.002;
    s.add(reachMesh);

    // Reachability ring edges
    const makeCircle = (r, segs = 64) => {
      const pts = [];
      for (let i = 0; i <= segs; i++) {
        const a = (i / segs) * Math.PI * 2;
        pts.push(V3(Math.cos(a) * r, 0.003, Math.sin(a) * r));
      }
      return pts;
    };
    const ringMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.15 });
    s.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(makeCircle(rMax)), ringMat));
    s.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(makeCircle(rMin)), ringMat));
  }

  /* ════════════════════════════════════════════════════════
     SCARA Robot Geometry
     ════════════════════════════════════════════════════════ */
  _initScaraGeometry() {
    const s = this._scene;
    const grp = new THREE.Group();
    s.add(grp);
    this._robotGroup = grp;

    // ── Base pedestal (fixed) ──
    const baseBottom = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.09, 0.02, 32), this._mBase);
    baseBottom.position.y = 0.01;
    baseBottom.castShadow = baseBottom.receiveShadow = true;
    grp.add(baseBottom);

    const baseCol = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.05, BASE_H - 0.02, 24), this._mBase);
    baseCol.position.y = 0.02 + (BASE_H - 0.02) / 2;
    baseCol.castShadow = true;
    grp.add(baseCol);

    // Gold accent at top of base
    const baseAccent = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.052, 0.012, 24), this._mJoint);
    baseAccent.position.y = BASE_H;
    grp.add(baseAccent);

    // ── Joint 1 motor housing (rotates with q1) ──
    this._j1Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 0.035, 24), this._mJoint);
    this._j1Motor.castShadow = true;
    grp.add(this._j1Motor);

    // ── Link 1 — shoulder arm ──
    this._link1 = new THREE.Mesh(
      new THREE.BoxGeometry(0.05, 0.025, L1),
      this._mArm
    );
    this._link1.castShadow = true;
    grp.add(this._link1);

    // ── Joint 2 motor housing ──
    this._j2Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.036, 0.030, 24), this._mJoint);
    this._j2Motor.castShadow = true;
    grp.add(this._j2Motor);

    // ── Link 2 — forearm ──
    this._link2 = new THREE.Mesh(
      new THREE.BoxGeometry(0.042, 0.020, L2),
      this._mArm
    );
    this._link2.castShadow = true;
    grp.add(this._link2);

    // ── Prismatic housing (at elbow tip) ──
    this._prisHousing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.025, 0.025, 0.04, 20),
      this._mBase
    );
    this._prisHousing.castShadow = true;
    grp.add(this._prisHousing);

    // ── Prismatic shaft (slides along Z / -Y in world) ──
    this._shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, STROKE, 16),
      this._mShaft
    );
    this._shaft.castShadow = true;
    grp.add(this._shaft);

    // ── End-Effector ──
    this._eeMesh = new THREE.Mesh(
      new THREE.ConeGeometry(0.018, 0.03, 12),
      this._mEE
    );
    this._eeMesh.castShadow = true;
    grp.add(this._eeMesh);

    // ── Vertical drop-line (visual guide) ──
    this._dropLineMat = new THREE.LineDashedMaterial({
      color: 0xC8A200, dashSize: 0.01, gapSize: 0.008, transparent: true, opacity: 0.4
    });
    this._dropLine = null;
  }

  /* ════════════════════════════════════════════════════════
     Target Sphere & Trail
     ════════════════════════════════════════════════════════ */
  _initTargetAndTrail() {
    const s = this._scene;

    this._mTgtS = new THREE.Mesh(new THREE.SphereGeometry(0.018, 20, 20), this._mTgt);
    this._mTgtS.castShadow = true;
    s.add(this._mTgtS);

    // Glow rings on table surface
    this._glowRings = [0.07, 0.045, 0.025].map((r, i) => {
      const m = new THREE.Mesh(
        new THREE.CircleGeometry(r, 36),
        new THREE.MeshBasicMaterial({
          color: 0x00e5ff, transparent: true,
          opacity: [0.08, 0.14, 0.22][i],
          side: THREE.DoubleSide, depthWrite: false
        })
      );
      m.rotation.x = -Math.PI / 2;
      m.position.y = 0.001;
      s.add(m);
      return m;
    });

    this._beamLine = null;

    // Trail
    this._trailMeshes = Array.from({ length: TRAIL_N }, () => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.005, 6, 6), this._mTrail.clone());
      m.visible = false;
      s.add(m);
      return m;
    });
    this._trailIdx = 0;

    // ── Payload (pick object) ──
    const payloadMat = new THREE.MeshPhysicalMaterial({
      color: 0xff6600, roughness: 0.2, metalness: 0.4,
      clearcoat: 0.8, envMapIntensity: 1.5
    });
    this._payload = new THREE.Mesh(
      new THREE.BoxGeometry(0.025, 0.025, 0.025),
      payloadMat
    );
    this._payload.castShadow = true;
    this._payload.position.set(0.40, 0.013, -0.15); // Three.js coords: x, y(up), z
    s.add(this._payload);
    this._payloadAttached = false;

    // Payload glow ring
    this._payloadGlow = new THREE.Mesh(
      new THREE.RingGeometry(0.02, 0.035, 32),
      new THREE.MeshBasicMaterial({
        color: 0xff6600, transparent: true, opacity: 0.25,
        side: THREE.DoubleSide, depthWrite: false
      })
    );
    this._payloadGlow.rotation.x = -Math.PI / 2;
    this._payloadGlow.position.set(0.40, 0.002, -0.15);
    s.add(this._payloadGlow);

    // ── Drop Zone ──
    const dzRing = new THREE.Mesh(
      new THREE.RingGeometry(0.025, 0.04, 32),
      new THREE.MeshBasicMaterial({
        color: 0x6c5ce7, transparent: true, opacity: 0.3,
        side: THREE.DoubleSide, depthWrite: false
      })
    );
    dzRing.rotation.x = -Math.PI / 2;
    dzRing.position.set(0.30, 0.002, 0.20);
    s.add(dzRing);
    this._dropZoneRing = dzRing;

    // Drop zone inner dot
    const dzDot = new THREE.Mesh(
      new THREE.CircleGeometry(0.008, 20),
      new THREE.MeshBasicMaterial({
        color: 0x6c5ce7, transparent: true, opacity: 0.5,
        side: THREE.DoubleSide, depthWrite: false
      })
    );
    dzDot.rotation.x = -Math.PI / 2;
    dzDot.position.set(0.30, 0.003, 0.20);
    s.add(dzDot);
    this._dropZoneDot = dzDot;
  }

  /* ════════════════════════════════════════════════════════
     Payload Attach / Detach (for Pick & Place)
     ════════════════════════════════════════════════════════ */

  /** Reparent payload to EE — it will follow the end-effector */
  attachPayloadToEE() {
    if (this._payloadAttached) return;
    this._payloadAttached = true;
    // Hide glow ring while attached
    this._payloadGlow.visible = false;
  }

  /** Reparent payload back to world at its current EE position */
  detachPayloadToWorld() {
    if (!this._payloadAttached) return;
    this._payloadAttached = false;
    // Keep payload at current EE position (already set in updateScene)
    this._payloadGlow.position.set(
      this._payload.position.x, 0.002, this._payload.position.z
    );
    this._payloadGlow.visible = true;
  }

  /** Move payload to a specific world position (used for cycle reset) */
  setPayloadPosition(x, y, z) {
    this._payload.position.set(x, y, z);
    this._payloadGlow.position.set(x, 0.002, z);
  }

  /** Move drop zone visual marker (Three.js coords: x, z) */
  setDropZonePosition(x, z) {
    this._dropZoneRing.position.set(x, 0.002, z);
    this._dropZoneDot.position.set(x, 0.003, z);
  }

  /* ════════════════════════════════════════════════════════
     updateScene — positions all meshes from FK transforms

     COORDINATE MAPPING (critical):
       DH convention:    X,Y = horizontal plane, Z = vertical (up)
       Three.js:         X,Z = horizontal plane, Y = vertical (up)

       Three.x =  DH.x  (T[12])
       Three.y =  DH.z  (T[14])   ← vertical axis swap
       Three.z = -DH.y  (-T[13])  ← preserves right-handedness
     ════════════════════════════════════════════════════════ */

  /**
   * Update all 3D meshes for a given SCARA joint configuration.
   *
   * @param {number[]} q      — [q1, q2, q3, q4] joint variables
   * @param {THREE.Vector3|null} pTgt3 — target point (or null)
   * @returns {{ position: number[], jointTransforms: Float64Array[] }}
   */
  updateScene(q, pTgt3) {
    const result = fk(q, SCARA_DH_CONFIG);
    const { jointTransforms: JT } = result;

    // DH → Three.js coordinate transform
    const dh2three = (T) => V3(T[12], T[14], -T[13]);

    // FK joint origins in Three.js coordinates
    const P0 = dh2three(JT[0]); // base origin  → (0, 0, 0)
    const P1 = dh2three(JT[1]); // end of link 1 → (0.30, 0.35, 0) at home
    const P2 = dh2three(JT[2]); // end of link 2 → (0.55, 0.35, 0) at home
    const P3 = dh2three(JT[3]); // after prismatic → drops in Y with q[2]
    const P4 = dh2three(JT[4]); // end-effector

    // Arm height (all horizontal links sit at Y = BASE_H = DH d1)
    const armY = P1.y; // should equal BASE_H at home

    // ── Joint 1 motor — at top of base column ──
    this._j1Motor.position.set(0, armY + 0.017, 0);

    // ── Link 1 — horizontal arm from base top to P1 ──
    const l1Start = V3(0, armY + 0.012, 0);
    const l1End   = V3(P1.x, armY + 0.012, P1.z);
    this._link1.position.copy(l1Start.clone().lerp(l1End, 0.5));
    this._link1.rotation.set(0, Math.atan2(l1End.x - l1Start.x, l1End.z - l1Start.z), 0);

    // ── Joint 2 motor — at P1 ──
    this._j2Motor.position.set(P1.x, armY + 0.015, P1.z);

    // ── Link 2 — horizontal forearm from P1 to P2 ──
    const l2Start = V3(P1.x, armY + 0.010, P1.z);
    const l2End   = V3(P2.x, armY + 0.010, P2.z);
    this._link2.position.copy(l2Start.clone().lerp(l2End, 0.5));
    this._link2.rotation.set(0, Math.atan2(l2End.x - l2Start.x, l2End.z - l2Start.z), 0);

    // ── Prismatic housing — at P2, sits below arm plane ──
    this._prisHousing.position.set(P2.x, armY - 0.01, P2.z);

    // ── Prismatic shaft — extends downward from housing ──
    // P3.y < P2.y when q[2] > 0 (prismatic displaces downward)
    const shaftTopY   = armY - 0.03;
    const shaftExtent = Math.max(shaftTopY - P3.y, 0.02); // visual minimum
    this._shaft.scale.y = shaftExtent / STROKE;
    this._shaft.position.set(P2.x, shaftTopY - shaftExtent / 2, P2.z);

    // ── End-Effector — at P3 (prismatic output) ──
    const eeY = shaftTopY - shaftExtent;
    this._eeMesh.position.set(P2.x, eeY - 0.015, P2.z);
    this._eeMesh.rotation.set(Math.PI, q[3], 0); // cone points down, rotated by q4

    // ── Vertical drop-line (visual guide from housing to EE) ──
    if (this._dropLine) {
      this._scene.remove(this._dropLine);
      this._dropLine.geometry.dispose();
    }
    const dlGeo = new THREE.BufferGeometry().setFromPoints([
      V3(P2.x, shaftTopY, P2.z),
      V3(P2.x, eeY - 0.015, P2.z)
    ]);
    this._dropLine = new THREE.Line(dlGeo, this._dropLineMat);
    this._dropLine.computeLineDistances();
    this._scene.add(this._dropLine);

    // ── Target sphere + glow + beam ──
    if (pTgt3) {
      this._mTgtS.visible = true;
      this._mTgtS.position.copy(pTgt3);
      this._glowRings.forEach(r => {
        r.position.x = pTgt3.x;
        r.position.z = pTgt3.z;
        r.visible = true;
      });
      if (this._beamLine) {
        this._scene.remove(this._beamLine);
        this._beamLine.geometry.dispose();
      }
      const eeV3 = V3(P2.x, eeY - 0.015, P2.z);
      const bg = new THREE.BufferGeometry().setFromPoints([eeV3, pTgt3.clone()]);
      this._beamLine = new THREE.Line(bg, this._mBeam);
      this._beamLine.computeLineDistances();
      this._scene.add(this._beamLine);
    } else {
      this._mTgtS.visible = false;
      this._glowRings.forEach(r => (r.visible = false));
      if (this._beamLine) {
        this._scene.remove(this._beamLine);
        this._beamLine.geometry.dispose();
        this._beamLine = null;
      }
    }

    // Store computed EE world position for external use
    this._eeWorldPos = V3(P2.x, eeY - 0.015, P2.z);

    // ── Move payload with EE if attached ──
    if (this._payloadAttached) {
      this._payload.position.set(P2.x, eeY - 0.032, P2.z);
    }

    return result;
  }

  /* ════════════════════════════════════════════════════════
     Trail
     ════════════════════════════════════════════════════════ */
  addTrailPoint(pos3) {
    const mesh = this._trailMeshes[this._trailIdx % TRAIL_N];
    mesh.position.copy(pos3);
    mesh.visible = true;
    this._trailMeshes.forEach((m, i) => {
      if (m.visible) m.material.opacity = 0.12 + 0.5 * (i / TRAIL_N);
    });
    this._trailIdx++;
  }

  resetTrail() {
    this._trailMeshes.forEach(m => (m.visible = false));
    this._trailIdx = 0;
  }

  /* ════════════════════════════════════════════════════════
     POV Camera
     ════════════════════════════════════════════════════════ */
  updatePovCamera(tx, ty, tz) {
    this._povCamera.position.set(tx, ty, tz);
    this._povCamera.lookAt(0.30, 0.02, 0.0);
    this._povCamera.updateMatrixWorld();
  }

  /* ════════════════════════════════════════════════════════
     Render Loop
     ════════════════════════════════════════════════════════ */
  start() {
    const loop = () => {
      requestAnimationFrame(loop);
      this._controls.update();
      this._renderer.render(this._scene, this._camera);
      this._povRenderer.render(this._scene, this._povCamera);
    };
    loop();
  }

  /* ════════════════════════════════════════════════════════
     Getters
     ════════════════════════════════════════════════════════ */
  get camera()       { return this._camera; }
  get scene()        { return this._scene; }
  get renderer()     { return this._renderer; }
  get controls()     { return this._controls; }
  get targetSphere() { return this._mTgtS; }
  get dragPlane()    { return this._dragPlane; }
  get TRAIL_N()      { return TRAIL_N; }
  get eeWorldPos()   { return this._eeWorldPos; }
  get payload()      { return this._payload; }
  get payloadAttached() { return this._payloadAttached; }
}
