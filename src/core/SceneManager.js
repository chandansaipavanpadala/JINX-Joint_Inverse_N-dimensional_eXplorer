/**
 * SceneManager.js — Three.js 3D Scene Controller for the RRR Lamp
 *
 * Encapsulates ALL rendering concerns:
 *   Scene, Camera, Renderer, OrbitControls, POV camera,
 *   Lighting, Materials, Environment meshes, Robot meshes,
 *   Target/glow/trail visuals, and the animation loop.
 *
 * Zero DOM manipulation (no getElementById text updates).
 * The UI Controller (Part 4) reads getters and calls methods.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { fkMat, V3, L1, L2, L3, RAD, linkLengths } from '../math/Kinematics.js';

const TRAIL_N = 60;

export default class SceneManager {
  /* ════════════════════════════════════════════════════════
     Constructor
     ════════════════════════════════════════════════════════ */
  constructor(canvasContainerId = 'cw', povContainerId = 'camPov') {
    const wrap = document.getElementById(canvasContainerId);
    const povWrap = document.getElementById(povContainerId);

    /* ── Scene ── */
    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x070710);
    this._scene.fog = new THREE.Fog(0x070710, 2.5, 5.0);

    /* ── Main Camera ── */
    this._camera = new THREE.PerspectiveCamera(
      44, wrap.clientWidth / wrap.clientHeight, 0.01, 10
    );
    this._camera.position.set(1.0, 0.95, 1.0);

    /* ── Renderer ── */
    this._renderer = new THREE.WebGLRenderer({ antialias: true });
    this._renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this._renderer.shadowMap.enabled = true;
    this._renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this._renderer.setSize(wrap.clientWidth, wrap.clientHeight);
    this._renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this._renderer.toneMappingExposure = 1.3;
    wrap.insertBefore(this._renderer.domElement, wrap.firstChild);

    /* ── Environment map (studio look) ── */
    const pmrem = new THREE.PMREMGenerator(this._renderer);
    pmrem.compileEquirectangularShader();
    const envScene = new THREE.Scene();
    envScene.add(new THREE.Mesh(
      new THREE.BoxGeometry(100, 100, 100),
      new THREE.MeshBasicMaterial({ color: 0x050510, side: THREE.BackSide })
    ));
    const envLight = new THREE.RectAreaLight(0xffffff, 5, 10, 10);
    envLight.position.set(5, 10, 5);
    envLight.lookAt(0, 0, 0);
    envScene.add(envLight);
    this._scene.environment = pmrem.fromScene(envScene).texture;

    /* ── OrbitControls ── */
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.target.set(0.25, 0.38, 0);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.07;
    this._controls.minDistance = 0.2;
    this._controls.maxDistance = 4;
    this._camera.lookAt(this._controls.target);

    /* ── POV Camera ── */
    this._povRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this._povRenderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this._povRenderer.shadowMap.enabled = true;
    this._povRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    this._povRenderer.toneMappingExposure = 1.1;
    this._povRenderer.setSize(200, 150);
    povWrap.appendChild(this._povRenderer.domElement);

    this._povCamera = new THREE.PerspectiveCamera(55, 200 / 150, 0.01, 10);
    this._povCamera.position.set(0.0, 0.50, 0.30);
    this._povCamera.lookAt(0.35, 0.0, 0.0);

    /* ── Lighting ── */
    this._initLighting();

    /* ── Materials ── */
    this._initMaterials();

    /* ── Environment meshes (floor, table, reachability) ── */
    this._initEnvironment();

    /* ── Robot lamp meshes ── */
    this._initLampGeometry();

    /* ── Target, glow rings, beam, trail ── */
    this._initTargetAndTrail();

    /* ── Drag plane for raycaster (table surface y=0.025) ── */
    this._dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -0.025);

    /* ── Resize handler ── */
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
    // 1. Soft ambient
    s.add(new THREE.AmbientLight(0x6070c0, 0.90));
    // 2. Main directional (sun)
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
    // 3. Warm fill
    const fill = new THREE.DirectionalLight(0xffa060, 0.55);
    fill.position.set(-2, 1.5, -0.5);
    s.add(fill);
    // 4. Cool rim
    const rim = new THREE.DirectionalLight(0x4080ff, 0.35);
    rim.position.set(-0.5, 0.5, -2);
    s.add(rim);
    // 5. Lamp head spot
    this._spot = new THREE.SpotLight(0xFFF8D0, 3.5);
    this._spot.angle = 0.40;
    this._spot.penumbra = 0.40;
    this._spot.decay = 1.4;
    this._spot.distance = 2.0;
    this._spot.castShadow = true;
    this._spot.shadow.mapSize.set(1024, 1024);
    this._spot.shadow.bias = -0.001;
    s.add(this._spot);
    this._spotTgt = new THREE.Object3D();
    s.add(this._spotTgt);
    this._spot.target = this._spotTgt;
    // 6. Point light near head
    this._lPt = new THREE.PointLight(0xFFE890, 1.5, 1.2);
    s.add(this._lPt);
  }

  /* ════════════════════════════════════════════════════════
     Materials
     ════════════════════════════════════════════════════════ */
  _initMaterials() {
    this._mBody = new THREE.MeshPhysicalMaterial({
      color: 0x8C0A0A, roughness: 0.15, metalness: 0.4,
      clearcoat: 1.0, clearcoatRoughness: 0.05, envMapIntensity: 1.2
    });
    this._mJoint = new THREE.MeshPhysicalMaterial({
      color: 0xC8A200, roughness: 0.05, metalness: 0.95,
      clearcoat: 1.0, envMapIntensity: 2.0
    });
    this._mShOut = new THREE.MeshStandardMaterial({
      color: 0x202028, roughness: 0.5, metalness: 0.1, side: THREE.FrontSide
    });
    this._mShIn = new THREE.MeshStandardMaterial({
      color: 0xF8F0A0, roughness: 0.6, emissive: 0x604820,
      emissiveIntensity: 0.5, side: THREE.BackSide
    });
    this._mCone = new THREE.MeshBasicMaterial({
      color: 0xFFF870, transparent: true, opacity: 0.05,
      side: THREE.DoubleSide, depthWrite: false
    });
    this._mBeam = new THREE.LineDashedMaterial({
      color: 0xE8C020, dashSize: 0.02, gapSize: 0.01
    });
    this._mTgt = new THREE.MeshPhysicalMaterial({
      color: 0x00e5ff, emissive: 0x0088ff, emissiveIntensity: 1.0,
      roughness: 0.1, metalness: 0.8, clearcoat: 1.0
    });
    this._mTrail = new THREE.MeshBasicMaterial({
      color: 0x00e5ff, transparent: true, opacity: 0.6, depthWrite: false
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
    const floor = new THREE.Mesh(floorG, new THREE.MeshStandardMaterial({
      color: 0x0b0b18, roughness: 0.98
    }));
    floor.position.y = -0.76;
    floor.receiveShadow = true;
    s.add(floor);

    const floorGrid = new THREE.GridHelper(4, 32, 0x18182a, 0x10101e);
    floorGrid.position.y = -0.756;
    s.add(floorGrid);

    // Table top
    const tblTopMat = new THREE.MeshStandardMaterial({
      color: 0x5A3418, roughness: 0.68, metalness: 0.05
    });
    const tblTop = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.04, 0.75), tblTopMat);
    tblTop.position.set(0.35, -0.02, 0);
    tblTop.castShadow = true;
    tblTop.receiveShadow = true;
    s.add(tblTop);

    // Gold trim strip
    const trimM = new THREE.MeshStandardMaterial({
      color: 0xA07820, roughness: 0.25, metalness: 0.60
    });
    const trimMesh = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.005, 0.005), trimM);
    trimMesh.position.set(0.35, -0.0025, 0.375);
    s.add(trimMesh);

    // Table legs
    const legMat = new THREE.MeshStandardMaterial({
      color: 0x381E0C, roughness: 0.80, metalness: 0.04
    });
    [[-0.14, 0.32], [-0.14, -0.32], [0.84, 0.32], [0.84, -0.32]].forEach(([x, z]) => {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.72, 0.05), legMat);
      leg.position.set(x, -0.40, z);
      leg.castShadow = leg.receiveShadow = true;
      s.add(leg);
    });

    // Table surface grid
    const tGrid = new THREE.GridHelper(1.0, 20, 0x7A5030, 0x5A3818);
    tGrid.position.set(0.35, 0.001, 0);
    s.add(tGrid);

    // Table boundary line
    const tBoundMat = new THREE.LineBasicMaterial({
      color: 0xFF6020, transparent: true, opacity: 0.5
    });
    const tBoundGeo = new THREE.BufferGeometry().setFromPoints([
      V3(-0.15, 0.003, -0.30), V3(0.85, 0.003, -0.30),
      V3(0.85, 0.003, 0.30), V3(-0.15, 0.003, 0.30),
      V3(-0.15, 0.003, -0.30)
    ]);
    s.add(new THREE.Line(tBoundGeo, tBoundMat));

    // Reachability dome
    const reachG = new THREE.SphereGeometry(
      L2 + L3, 64, 32, 0, Math.PI * 2, 0, Math.PI / 2
    );
    const reachM = new THREE.MeshStandardMaterial({
      color: 0x00e5ff, transparent: true, opacity: 0.03,
      side: THREE.BackSide, depthWrite: false
    });
    const reachDome = new THREE.Mesh(reachG, reachM);
    reachDome.position.y = L1;
    s.add(reachDome);
    reachDome.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(reachG),
      new THREE.LineBasicMaterial({
        color: 0x00e5ff, transparent: true, opacity: 0.1
      })
    ));
  }

  /* ════════════════════════════════════════════════════════
     Robot Lamp Geometry
     ════════════════════════════════════════════════════════ */
  _initLampGeometry() {
    const lamp = new THREE.Group();
    this._scene.add(lamp);
    this._lamp = lamp;

    const mb = this._mBody, mj = this._mJoint;

    // Base
    const mBase = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.093, 0.012, 32), mb);
    mBase.position.y = 0.006;
    mBase.castShadow = mBase.receiveShadow = true;
    lamp.add(mBase);

    // Gold accent ring
    const mAccent = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.007, 32), mj);
    mAccent.position.set(0, 0.013, 0);
    lamp.add(mAccent);

    // Column (Link 1 — base height d₁)
    this._mCol = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.026, L1, 16), mb);
    this._mCol.position.y = L1 / 2;
    this._mCol.castShadow = true;
    lamp.add(this._mCol);

    // Shoulder joint motor
    this._mSh = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.036, 0.040, 24), mj);
    this._mSh.castShadow = true;
    lamp.add(this._mSh);

    // Lower arm (Link 2 — a₂)
    this._mLow = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, L2, 16), mb);
    this._mLow.castShadow = true;
    lamp.add(this._mLow);

    // Elbow motor
    this._mEl = new THREE.Mesh(new THREE.CylinderGeometry(0.030, 0.030, 0.035, 24), mj);
    this._mEl.castShadow = true;
    lamp.add(this._mEl);

    // Upper arm (Link 3 — a₃)
    this._mUp = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, L3, 16), mb);
    this._mUp.castShadow = true;
    lamp.add(this._mUp);

    // Wrist motor
    this._mWrist = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.032, 24), mj);
    this._mWrist.castShadow = true;
    lamp.add(this._mWrist);

    // Shade outer + inner
    this._mSO = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.048, 0.062, 24), this._mShOut);
    this._mSO.castShadow = true;
    lamp.add(this._mSO);
    this._mSI = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.048, 0.062, 24), this._mShIn);
    lamp.add(this._mSI);

    // Bulb
    this._mBulb = new THREE.Mesh(
      new THREE.SphereGeometry(0.010, 12, 12),
      new THREE.MeshStandardMaterial({
        color: 0xFFFAD0, emissive: 0xFFF8A0,
        emissiveIntensity: 2.0, roughness: 0.1
      })
    );
    lamp.add(this._mBulb);

    // Light cone (unit, scaled per frame)
    this._mLC = new THREE.Mesh(
      new THREE.ConeGeometry(1, 1, 32, 1, true), this._mCone
    );
    this._mLC.renderOrder = 1;
    lamp.add(this._mLC);

    // ── Clickable Link Mesh Registry ──
    // Each entry: { mesh, key, label, radius, defaultLen, min, max }
    this._linkMeshes = [
      { mesh: this._mCol, key: 'L1', label: 'Base Column (d₁)',  radius: [0.022, 0.026], defaultLen: L1, min: 0.05, max: 0.35 },
      { mesh: this._mLow, key: 'L2', label: 'Lower Arm (a₂)',    radius: [0.022, 0.022], defaultLen: L2, min: 0.10, max: 0.50 },
      { mesh: this._mUp,  key: 'L3', label: 'Upper Arm (a₃)',    radius: [0.018, 0.018], defaultLen: L3, min: 0.08, max: 0.45 },
    ];
    this._highlightedLink = -1;
    this._savedEmissive = null;
  }

  /* ════════════════════════════════════════════════════════
     Target sphere, glow rings, beam line, trail dots
     ════════════════════════════════════════════════════════ */
  _initTargetAndTrail() {
    const s = this._scene;

    // Target sphere
    this._mTgtS = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 20, 20), this._mTgt
    );
    this._mTgtS.castShadow = true;
    s.add(this._mTgtS);

    // Glow rings
    this._glowRings = [0.09, 0.055, 0.028].map((r, i) => {
      const m = new THREE.Mesh(
        new THREE.CircleGeometry(r, 36),
        new THREE.MeshBasicMaterial({
          color: 0xFFF870, transparent: true,
          opacity: [0.10, 0.16, 0.26][i],
          side: THREE.DoubleSide, depthWrite: false
        })
      );
      m.rotation.x = -Math.PI / 2;
      m.position.y = 0.001;
      m.renderOrder = 0;
      s.add(m);
      return m;
    });

    // Beam line (created/destroyed per frame)
    this._beamLine = null;

    // Trail dots
    this._trailMeshes = Array.from({ length: TRAIL_N }, () => {
      const m = new THREE.Mesh(
        new THREE.SphereGeometry(0.006, 6, 6),
        this._mTrail.clone()
      );
      m.visible = false;
      s.add(m);
      return m;
    });
    this._trailIdx = 0;
  }

  /* ════════════════════════════════════════════════════════
     Public Methods
     ════════════════════════════════════════════════════════ */

  /** Align a cylinder mesh between two 3D points */
  alignCyl(mesh, P1, P2) {
    const d = P2.clone().sub(P1), l = d.length();
    if (l < 1e-4) return;
    mesh.position.copy(P1).addScaledVector(d.normalize(), l / 2);
    mesh.quaternion.setFromUnitVectors(V3(0, 1, 0), d.normalize());
  }

  /**
   * Update all 3D meshes for a given joint configuration.
   * Returns the FK result for the UI layer to use.
   *
   * @param {number} t1 - Joint 1 angle (rad)
   * @param {number} t2 - Joint 2 angle (rad)
   * @param {number} t3 - Joint 3 angle (rad)
   * @param {THREE.Vector3|null} pTgt3 - Target point on table (or null)
   * @returns {{ x, y, z, r, P0, P1, P2, P3 }} FK result
   */
  updateScene(t1, t2, t3, pTgt3) {
    const f = fkMat(t1, t2, t3);
    const { P1, P2, P3 } = f;

    const sideAx = new THREE.Vector3(-Math.sin(t1), 0, -Math.cos(t1));
    const offV = sideAx.clone().multiplyScalar(0.06);

    // Column — reposition for dynamic L1
    this._mCol.position.y = P1.y / 2;

    // Shoulder
    this._mSh.position.copy(P1).add(offV);
    this._mSh.quaternion.setFromUnitVectors(V3(0, 1, 0), sideAx);

    // Lower arm
    this.alignCyl(this._mLow, P1.clone().add(offV), P2.clone().add(offV));

    // Elbow
    this._mEl.position.copy(P2).add(offV);
    this._mEl.quaternion.setFromUnitVectors(V3(0, 1, 0), sideAx);

    // Upper arm
    this.alignCyl(this._mUp, P2.clone().add(offV), P3.clone().add(offV));

    // Wrist
    this._mWrist.position.copy(P3).add(offV);
    this._mWrist.quaternion.setFromUnitVectors(V3(0, 1, 0), sideAx);

    // Shade direction
    const shDir = pTgt3 ? pTgt3.clone().sub(P3).normalize() : V3(0, -1, 0);
    this._mSO.position.copy(P3).addScaledVector(shDir, 0.062 / 2);
    this._mSI.position.copy(P3).addScaledVector(shDir, 0.062 / 2);
    this._mSO.quaternion.setFromUnitVectors(V3(0, 1, 0), shDir.clone().negate());
    this._mSI.quaternion.setFromUnitVectors(V3(0, 1, 0), shDir.clone().negate());

    // Bulb
    this._mBulb.position.copy(P3).addScaledVector(shDir, 0.008);

    // Light cone
    const cLen = pTgt3 ? Math.min(P3.distanceTo(pTgt3) * 1.1, 0.55) : 0.50;
    const cR = cLen * 0.27;
    this._mLC.position.copy(P3).addScaledVector(shDir, cLen / 2);
    this._mLC.quaternion.setFromUnitVectors(V3(0, 1, 0), shDir.clone().negate());
    this._mLC.scale.set(cR, cLen, cR);

    // Spot & point lights
    this._spot.position.copy(P3);
    this._spotTgt.position.copy(pTgt3 || V3(P3.x, 0, P3.z));
    this._lPt.position.copy(P3).addScaledVector(shDir, 0.06);

    // Target sphere + glow rings + beam
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
      const bg = new THREE.BufferGeometry().setFromPoints([P3.clone(), pTgt3.clone()]);
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

    return f;
  }

  /**
   * Add a trail dot at a given 3D position.
   * Call this each simulation frame.
   */
  addTrailPoint(pos3) {
    const mesh = this._trailMeshes[this._trailIdx % TRAIL_N];
    mesh.position.copy(pos3);
    mesh.visible = true;
    this._trailMeshes.forEach((m, i) => {
      if (m.visible) m.material.opacity = 0.12 + 0.5 * (i / TRAIL_N);
    });
    this._trailIdx++;
  }

  /** Reset all trail dots to invisible */
  resetTrail() {
    this._trailMeshes.forEach(m => (m.visible = false));
    this._trailIdx = 0;
  }

  /**
   * Update the POV camera position (extrinsic translation).
   * @param {number} tx
   * @param {number} ty
   * @param {number} tz
   */
  updatePovCamera(tx, ty, tz) {
    this._povCamera.position.set(tx, ty, tz);
    this._povCamera.lookAt(0.35, 0.02, 0.0);
    this._povCamera.updateMatrixWorld();
  }

  /** Start the render loop */
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
     Getters — for UI Controller (Part 4) to attach events
     ════════════════════════════════════════════════════════ */

  /** Main Three.js camera */
  get camera() { return this._camera; }

  /** The Three.js scene */
  get scene() { return this._scene; }

  /** The WebGLRenderer (for raycaster bounds) */
  get renderer() { return this._renderer; }

  /** OrbitControls instance (to disable during drag) */
  get controls() { return this._controls; }

  /** Target sphere mesh (for raycaster hit-testing) */
  get targetSphere() { return this._mTgtS; }

  /** Table-surface drag plane (for raycaster intersection) */
  get dragPlane() { return this._dragPlane; }

  /** Trail dot count constant */
  get TRAIL_N() { return TRAIL_N; }

  /** Clickable link mesh entries (array) */
  get linkMeshes() { return this._linkMeshes; }

  /** Just the Three.js mesh objects for raycasting */
  get linkMeshArray() { return this._linkMeshes.map(e => e.mesh); }

  /* ═══════════ Link Resizing ═══════════ */

  /**
   * Dynamically resize a link's cylinder geometry.
   * @param {number} index - Link index (0=L1, 1=L2, 2=L3)
   * @param {number} newLen - New length in meters
   */
  resizeLink(index, newLen) {
    const entry = this._linkMeshes[index];
    if (!entry) return;
    const [r1, r2] = entry.radius;
    entry.mesh.geometry.dispose();
    entry.mesh.geometry = new THREE.CylinderGeometry(r1, r2, newLen, 16);
    // Update the mutable kinematics config
    linkLengths[entry.key] = newLen;
  }

  /**
   * Highlight a link with emissive glow.
   * @param {number} index - Link index to highlight (-1 = clear)
   */
  highlightLink(index) {
    // Clear previous highlight
    this.clearHighlight();
    if (index < 0 || index >= this._linkMeshes.length) return;
    const mat = this._linkMeshes[index].mesh.material;
    this._savedEmissive = mat.emissive.getHex();
    this._savedEmissiveIntensity = mat.emissiveIntensity;
    mat.emissive.setHex(0x00e5ff);
    mat.emissiveIntensity = 0.6;
    this._highlightedLink = index;
  }

  /** Clear link highlight */
  clearHighlight() {
    if (this._highlightedLink >= 0 && this._highlightedLink < this._linkMeshes.length) {
      const mat = this._linkMeshes[this._highlightedLink].mesh.material;
      mat.emissive.setHex(this._savedEmissive || 0x000000);
      mat.emissiveIntensity = this._savedEmissiveIntensity || 0;
    }
    this._highlightedLink = -1;
  }
}
