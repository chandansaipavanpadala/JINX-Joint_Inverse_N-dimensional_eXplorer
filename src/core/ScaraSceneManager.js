/**
 * ScaraSceneManager.js — Three.js 3D Scene Controller for the SCARA Arm
 *
 * Renders a 4-DOF SCARA (R-R-P-R) robot driven by the generalized
 * KinematicsNDOF engine. Zero DOM text manipulation.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { fk, SCARA_DH_CONFIG } from '../math/KinematicsNDOF.js';

const TRAIL_N = 80;
const V3 = (x, y, z) => new THREE.Vector3(x, y, z);

// DH config constants for mesh sizing
const BASE_H  = 0.35;  // d1 — base column height
const L1      = 0.30;  // a1 — upper arm length
const L2      = 0.25;  // a2 — forearm length
const STROKE  = 0.35;  // max prismatic stroke (matches DH limitMax)

export default class ScaraSceneManager {

  constructor(canvasId = 'cw') {
    const wrap = document.getElementById(canvasId);

    // ── Scene ──
    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x0a0c12);
    this._scene.fog = new THREE.Fog(0x0a0c12, 3.5, 7.0);

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
    this._renderer.toneMappingExposure = 1.1;
    wrap.insertBefore(this._renderer.domElement, wrap.firstChild);

    // ── Environment map ──
    const pmrem = new THREE.PMREMGenerator(this._renderer);
    pmrem.compileEquirectangularShader();
    const envScene = new THREE.Scene();
    envScene.add(new THREE.Mesh(
      new THREE.BoxGeometry(100, 100, 100),
      new THREE.MeshBasicMaterial({ color: 0x080a14, side: THREE.BackSide })
    ));
    // Factory ceiling panels — multiple white rects for even industrial fill
    [[-3,12,0],[3,12,0],[0,12,4],[0,12,-4]].forEach(([x,y,z]) => {
      const rl = new THREE.RectAreaLight(0xe8f0ff, 4, 8, 8);
      rl.position.set(x, y, z); rl.lookAt(0, 0, 0);
      envScene.add(rl);
    });
    this._scene.environment = pmrem.fromScene(envScene).texture;

    // ── OrbitControls ──
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.target.set(0.15, 0.12, 0);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.07;
    this._controls.minDistance = 0.2;
    this._controls.maxDistance = 4;
    this._camera.lookAt(this._controls.target);

    // ── TransformControls (for dragging scene objects) ──
    this._transformControls = new TransformControls(this._camera, this._renderer.domElement);
    this._transformControls.setMode('translate');
    this._transformControls.setSize(0.5);
    this._transformControls.showX = true;
    this._transformControls.showY = false;  // Lock to XZ plane (table surface)
    this._transformControls.showZ = true;
    this._scene.add(this._transformControls.getHelper());

    // Disable OrbitControls while dragging a scene object
    this._transformControls.addEventListener('dragging-changed', (event) => {
      this._controls.enabled = !event.value;
    });

    // Callback for external sync when an object is dragged
    this._onObjectDragged = null;
    this._transformControls.addEventListener('change', () => {
      if (this._transformControls.object && this._onObjectDragged) {
        const obj = this._transformControls.object;
        // Clamp Y to table surface during drag
        if (obj === this._payload) {
          obj.position.y = 0.013;
        } else if (obj === this._dropZoneMesh) {
          obj.position.y = 0.013;
        }
        this._onObjectDragged(obj);
      }
    });

    // ── Init sub-systems ──
    this._initLighting();
    this._initMaterials();
    this._initEnvironment();
    this._initScaraGeometry();
    this._initTargetAndTrail();
    this._initPerceptionCamera();

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
     Lighting — Cool industrial factory look
     ════════════════════════════════════════════════════════ */
  _initLighting() {
    const s = this._scene;

    // Ambient — cool blue-white factory ambience
    s.add(new THREE.AmbientLight(0x8090c8, 0.75));

    // Main overhead — bright clinical white, slight blue tint
    const sun = new THREE.DirectionalLight(0xe0ecff, 1.4);
    sun.position.set(1.5, 4.0, 1.0);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0003;
    sun.shadow.camera.near = 0.1;
    sun.shadow.camera.far = 12;
    sun.shadow.camera.top = sun.shadow.camera.right = 2.5;
    sun.shadow.camera.bottom = sun.shadow.camera.left = -2.5;
    s.add(sun);

    // Second overhead panel — opposite side for even fill
    const sun2 = new THREE.DirectionalLight(0xd8e4ff, 0.8);
    sun2.position.set(-1.5, 3.5, -1.0);
    s.add(sun2);

    // Cool fill — simulates reflected factory floor light
    const fill = new THREE.DirectionalLight(0x90b0d0, 0.4);
    fill.position.set(-2, 1.0, 0.5);
    s.add(fill);

    // Subtle warm accent (safety indicator vibe)
    const accent = new THREE.DirectionalLight(0xffcc44, 0.15);
    accent.position.set(0.5, 0.5, -2);
    s.add(accent);
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
     Environment — Industrial Factory: Floor, Pedestal, Conveyor

     CRITICAL: The old table top was at Y ≈ 0.0.
     The conveyor belt surface is also at Y ≈ 0.0 so the
     PickAndPlace state machine's graspZ / safeZ remain valid.
     ════════════════════════════════════════════════════════ */
  _initEnvironment() {
    const s = this._scene;

    // ── 1. Polished concrete factory floor ──
    const floorMat = new THREE.MeshPhysicalMaterial({
      color: 0x1a1d24, roughness: 0.75, metalness: 0.15,
      clearcoat: 0.25, clearcoatRoughness: 0.6,
    });
    const floorG = new THREE.PlaneGeometry(8, 8);
    floorG.rotateX(-Math.PI / 2);
    const floor = new THREE.Mesh(floorG, floorMat);
    floor.position.y = -0.76;
    floor.receiveShadow = true;
    s.add(floor);

    // Floor grid — subtle cyan-tinted industrial lines
    const floorGrid = new THREE.GridHelper(6, 48, 0x1a2838, 0x141c28);
    floorGrid.position.y = -0.756;
    s.add(floorGrid);

    // ── 2. Hazard floor markings (yellow safety stripes around robot base) ──
    const hazardMat = new THREE.MeshStandardMaterial({
      color: 0xccaa00, roughness: 0.85, metalness: 0.05,
      transparent: true, opacity: 0.25
    });
    // Rectangular safety zone on floor
    const hazardZone = new THREE.Mesh(
      new THREE.RingGeometry(0.65, 0.68, 64),
      hazardMat
    );
    hazardZone.rotation.x = -Math.PI / 2;
    hazardZone.position.y = -0.754;
    s.add(hazardZone);

    // ── 3. Industrial pedestal (replaces table under robot base) ──
    // Top cap — heavy steel plate, Y top face ≈ 0.0
    const pedestalMat = new THREE.MeshPhysicalMaterial({
      color: 0x2a2d38, roughness: 0.35, metalness: 0.90,
      clearcoat: 0.4, clearcoatRoughness: 0.3, envMapIntensity: 1.4
    });
    const pedTop = new THREE.Mesh(
      new THREE.CylinderGeometry(0.12, 0.12, 0.03, 32),
      pedestalMat
    );
    pedTop.position.set(0, -0.015, 0);
    pedTop.castShadow = pedTop.receiveShadow = true;
    s.add(pedTop);

    // Pedestal column
    const pedCol = new THREE.Mesh(
      new THREE.CylinderGeometry(0.07, 0.09, 0.72, 24),
      pedestalMat
    );
    pedCol.position.set(0, -0.39, 0);
    pedCol.castShadow = true;
    s.add(pedCol);

    // Pedestal base plate (floor mount)
    const pedBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.14, 0.15, 0.02, 32),
      pedestalMat
    );
    pedBase.position.set(0, -0.75, 0);
    pedBase.receiveShadow = true;
    s.add(pedBase);

    // Gold accent ring at pedestal top
    const pedAccent = new THREE.Mesh(
      new THREE.TorusGeometry(0.12, 0.004, 8, 48),
      new THREE.MeshPhysicalMaterial({
        color: 0xC8A200, roughness: 0.05, metalness: 0.95,
        clearcoat: 1.0, envMapIntensity: 2.0
      })
    );
    pedAccent.rotation.x = Math.PI / 2;
    pedAccent.position.set(0, 0.0, 0);
    s.add(pedAccent);

    // ── 4. Conveyor belt ──
    // Belt dimensions: extends along X-axis where pick/place targets are
    const BELT_W = 0.32;   // width (Z-axis)
    const BELT_L = 1.4;    // length (X-axis)
    const BELT_H = 0.74;   // total height from floor to belt surface
    const BELT_CX = 0.40;  // center X
    const BELT_CZ = 0.0;   // center Z
    const BELT_SURFACE_Y = -0.02; // top surface ~= old table top

    // Belt frame (main body)
    const frameMat = new THREE.MeshPhysicalMaterial({
      color: 0x3a3d48, roughness: 0.4, metalness: 0.85,
      clearcoat: 0.3, envMapIntensity: 1.2
    });
    const frame = new THREE.Mesh(
      new THREE.BoxGeometry(BELT_L, BELT_H - 0.04, BELT_W - 0.06),
      frameMat
    );
    frame.position.set(BELT_CX, BELT_SURFACE_Y - (BELT_H - 0.04) / 2, BELT_CZ);
    frame.castShadow = frame.receiveShadow = true;
    s.add(frame);

    // Belt surface — dark rubber
    const beltMat = new THREE.MeshPhysicalMaterial({
      color: 0x1c1c1c, roughness: 0.92, metalness: 0.05,
      clearcoat: 0.05,
    });
    const belt = new THREE.Mesh(
      new THREE.BoxGeometry(BELT_L - 0.06, 0.018, BELT_W - 0.04),
      beltMat
    );
    belt.position.set(BELT_CX, BELT_SURFACE_Y + 0.009, BELT_CZ);
    belt.receiveShadow = true;
    s.add(belt);

    // Belt texture lines (subtle ridges)
    const ridgeMat = new THREE.MeshStandardMaterial({
      color: 0x252525, roughness: 0.95, metalness: 0.0,
    });
    for (let i = 0; i < 28; i++) {
      const rx = BELT_CX - (BELT_L - 0.1) / 2 + i * ((BELT_L - 0.1) / 27);
      const ridge = new THREE.Mesh(
        new THREE.BoxGeometry(0.003, 0.001, BELT_W - 0.06),
        ridgeMat
      );
      ridge.position.set(rx, BELT_SURFACE_Y + 0.019, BELT_CZ);
      s.add(ridge);
    }

    // Side rails — metallic guard rails
    const railMat = new THREE.MeshPhysicalMaterial({
      color: 0x5a6070, roughness: 0.2, metalness: 0.92,
      clearcoat: 0.5, envMapIntensity: 1.6
    });
    [-1, 1].forEach(side => {
      const rail = new THREE.Mesh(
        new THREE.BoxGeometry(BELT_L, 0.04, 0.02),
        railMat
      );
      rail.position.set(BELT_CX, BELT_SURFACE_Y + 0.02, BELT_CZ + side * (BELT_W / 2));
      rail.castShadow = true;
      s.add(rail);
    });

    // End rollers — cylindrical drums at each end
    const rollerMat = new THREE.MeshPhysicalMaterial({
      color: 0x6a6d78, roughness: 0.25, metalness: 0.88,
      envMapIntensity: 1.0
    });
    [-1, 1].forEach(end => {
      const roller = new THREE.Mesh(
        new THREE.CylinderGeometry(0.035, 0.035, BELT_W + 0.02, 20),
        rollerMat
      );
      roller.rotation.x = Math.PI / 2;
      roller.position.set(
        BELT_CX + end * (BELT_L / 2 - 0.02),
        BELT_SURFACE_Y + 0.01,
        BELT_CZ
      );
      roller.castShadow = true;
      s.add(roller);

      // Roller end caps
      [-1, 1].forEach(capSide => {
        const cap = new THREE.Mesh(
          new THREE.CylinderGeometry(0.04, 0.04, 0.01, 16),
          railMat
        );
        cap.rotation.x = Math.PI / 2;
        cap.position.set(
          BELT_CX + end * (BELT_L / 2 - 0.02),
          BELT_SURFACE_Y + 0.01,
          BELT_CZ + capSide * (BELT_W / 2 + 0.01)
        );
        s.add(cap);
      });
    });

    // Conveyor support legs
    const legMat = new THREE.MeshPhysicalMaterial({
      color: 0x3a3d48, roughness: 0.45, metalness: 0.85,
      envMapIntensity: 1.0
    });
    [[-0.30, -0.12], [-0.30, 0.12], [1.10, -0.12], [1.10, 0.12]].forEach(([x, z]) => {
      const leg = new THREE.Mesh(
        new THREE.BoxGeometry(0.035, BELT_H - 0.04, 0.035),
        legMat
      );
      leg.position.set(x, BELT_SURFACE_Y - (BELT_H - 0.04) / 2, z);
      leg.castShadow = leg.receiveShadow = true;
      s.add(leg);

      // Foot pads
      const foot = new THREE.Mesh(
        new THREE.BoxGeometry(0.05, 0.01, 0.05),
        legMat
      );
      foot.position.set(x, -0.755, z);
      foot.receiveShadow = true;
      s.add(foot);
    });

    // ── 5. Conveyor surface grid (replaces old table grid) ──
    const convGrid = new THREE.GridHelper(1.2, 24, 0x303540, 0x252830);
    convGrid.position.set(BELT_CX, BELT_SURFACE_Y + 0.02, BELT_CZ);
    s.add(convGrid);

    // ── 6. Reachability ring (SCARA workspace annulus) ──
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

    // Drop zone selectable mesh (clickable target for TransformControls)
    this._dropZoneMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.04, 0.015, 0.04),
      new THREE.MeshPhysicalMaterial({
        color: 0x6c5ce7, roughness: 0.3, metalness: 0.4,
        clearcoat: 0.6, transparent: true, opacity: 0.55,
        envMapIntensity: 1.5
      })
    );
    this._dropZoneMesh.position.set(0.30, 0.013, 0.20);
    this._dropZoneMesh.castShadow = true;
    s.add(this._dropZoneMesh);
    this._dropZoneMesh.userData.draggable = true;
    this._dropZoneMesh.userData.role = 'dropZone';
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
    if (this._dropZoneMesh) this._dropZoneMesh.position.set(x, 0.013, z);
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
     Perception Camera — simulated pinhole camera + frustum
     ════════════════════════════════════════════════════════ */
  _initPerceptionCamera() {
    // Secondary camera for perception simulation
    // near=0.01 (not 0.05) so the table surface at y≈0 is visible from y=1.20
    this._percCamera = new THREE.PerspectiveCamera(60, 320 / 240, 0.01, 5.0);
    this._percCamera.position.set(0.30, 1.20, 0.00);
    this._percCamera.rotation.set(-Math.PI / 2, 0, 0); // Looking down
    this._scene.add(this._percCamera);

    // Camera helper draws the FOV frustum wireframe
    this._percHelper = new THREE.CameraHelper(this._percCamera);
    this._scene.add(this._percHelper);

    // Camera body mesh (small box to represent the physical camera)
    const camBodyMat = new THREE.MeshPhysicalMaterial({
      color: 0x2a2d38, roughness: 0.3, metalness: 0.85,
      clearcoat: 0.5, envMapIntensity: 1.4
    });
    this._camBodyMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.04, 0.03, 0.05),
      camBodyMat
    );
    this._camBodyMesh.castShadow = true;
    this._scene.add(this._camBodyMesh);

    // Camera lens (cylinder)
    const lensMat = new THREE.MeshPhysicalMaterial({
      color: 0x111122, roughness: 0.05, metalness: 0.4,
      clearcoat: 1.0, envMapIntensity: 2.0
    });
    this._camLensMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.014, 0.02, 16),
      lensMat
    );
    this._scene.add(this._camLensMesh);

    // LED indicator light
    this._camLED = new THREE.Mesh(
      new THREE.SphereGeometry(0.004, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0x00ff44 })
    );
    this._scene.add(this._camLED);

    // Start hidden — activated by CameraPanel
    this._percActive = false;
    this._percHelper.visible = false;
    this._camBodyMesh.visible = false;
    this._camLensMesh.visible = false;
    this._camLED.visible = false;

    // Off-screen render target — avoids resizing the main canvas (FIX 1)
    this._percRT = new THREE.WebGLRenderTarget(320, 240, {
      minFilter: THREE.LinearFilter,
      magFilter: THREE.LinearFilter,
      format: THREE.RGBAFormat,
    });
  }

  /**
   * Update the perception camera's position, rotation, and FOV.
   * @param {number} tx  - Position X (Three.js coords)
   * @param {number} ty  - Position Y (Three.js coords, up)
   * @param {number} tz  - Position Z (Three.js coords)
   * @param {number} rx  - Rotation X (rad)
   * @param {number} ry  - Rotation Y (rad)
   * @param {number} rz  - Rotation Z (rad)
   * @param {number} fov - Vertical FOV (degrees)
   */
  updatePerceptionCamera(tx, ty, tz, rx, ry, rz, fov) {
    this._percCamera.position.set(tx, ty, tz);
    this._percCamera.rotation.set(rx, ry, rz, 'XYZ');
    this._percCamera.fov = fov;
    this._percCamera.updateProjectionMatrix();
    this._percHelper.update();

    // Move camera body mesh to match
    this._camBodyMesh.position.set(tx, ty, tz);
    this._camBodyMesh.rotation.set(rx, ry, rz, 'XYZ');

    // Lens points downward from camera body
    const lensOffset = new THREE.Vector3(0, -0.025, 0);
    lensOffset.applyEuler(new THREE.Euler(rx, ry, rz, 'XYZ'));
    this._camLensMesh.position.set(tx + lensOffset.x, ty + lensOffset.y, tz + lensOffset.z);
    this._camLensMesh.rotation.set(rx, ry, rz, 'XYZ');

    // LED on top-right
    const ledOffset = new THREE.Vector3(0.015, 0.016, -0.02);
    ledOffset.applyEuler(new THREE.Euler(rx, ry, rz, 'XYZ'));
    this._camLED.position.set(tx + ledOffset.x, ty + ledOffset.y, tz + ledOffset.z);
  }

  /**
   * Render the perception camera view to an external canvas.
   * Uses a WebGLRenderTarget so the main viewport is never disturbed.
   * @param {HTMLCanvasElement} targetCanvas
   */
  renderPerceptionView(targetCanvas) {
    if (!targetCanvas || !this._percCamera || !this._percRT) return;

    // Temporarily hide camera body visuals from perception view
    const wasHelperVisible    = this._percHelper.visible;
    const wasBodyVisible      = this._camBodyMesh.visible;
    const wasLensVisible      = this._camLensMesh.visible;
    const wasLEDVisible       = this._camLED.visible;
    this._percHelper.visible    = false;
    this._camBodyMesh.visible   = false;
    this._camLensMesh.visible   = false;
    this._camLED.visible        = false;

    // Render into offscreen render target (no main canvas resize)
    this._renderer.setRenderTarget(this._percRT);
    this._renderer.render(this._scene, this._percCamera);
    this._renderer.setRenderTarget(null);

    // Restore visibility before restoring render target
    this._percHelper.visible    = wasHelperVisible;
    this._camBodyMesh.visible   = wasBodyVisible;
    this._camLensMesh.visible   = wasLensVisible;
    this._camLED.visible        = wasLEDVisible;

    // Read pixels from render target and draw onto the target canvas
    const w = this._percRT.width;
    const h = this._percRT.height;
    const buffer = new Uint8Array(w * h * 4);
    this._renderer.readRenderTargetPixels(this._percRT, 0, 0, w, h, buffer);

    // WebGL renders bottom-up; flip vertically when drawing to 2D canvas
    const ctx = targetCanvas.getContext('2d');
    const imgData = ctx.createImageData(w, h);
    for (let row = 0; row < h; row++) {
      const src = (h - 1 - row) * w * 4;
      const dst = row * w * 4;
      imgData.data.set(buffer.subarray(src, src + w * 4), dst);
    }
    ctx.putImageData(imgData, 0, 0);
  }

  /** Toggle perception camera visibility */
  setPerceptionActive(active) {
    this._percActive = active;
    this._percHelper.visible = active;
    this._camBodyMesh.visible = active;
    this._camLensMesh.visible = active;
    this._camLED.visible = active;
  }

  /* ════════════════════════════════════════════════════════
     Render Loop
     ════════════════════════════════════════════════════════ */
  start() {
    const loop = () => {
      requestAnimationFrame(loop);
      this._controls.update();
      this._renderer.render(this._scene, this._camera);

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
  get dropZoneMesh() { return this._dropZoneMesh; }
  get transformControls() { return this._transformControls; }
  get percCamera()   { return this._percCamera; }

  /** Register callback for object drag sync: (mesh) => void */
  set onObjectDragged(fn) { this._onObjectDragged = fn; }
}
