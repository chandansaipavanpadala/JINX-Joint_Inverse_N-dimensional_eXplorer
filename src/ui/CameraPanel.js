/**
 * CameraPanel.js — Perception Camera Controller for JINX.
 * PDF §Camera & Perception: "Pinhole camera model, intrinsic/extrinsic
 * parameters, and FOV frustum visualization."
 *
 * Provides a FloatingPanel with:
 *   - Extrinsic sliders: tx, ty, tz, rx, ry, rz
 *   - Intrinsic sliders: FOV (maps to focal length)
 *   - Live camera viewport canvas (renders what the perception camera sees)
 *   - Intrinsic matrix K and projection readout
 *
 * Uses the existing glassmorphism design system.
 */

import FloatingPanel from './FloatingPanel.js';
import {
  buildIntrinsicMatrix,
  fovToFocalLength,
  project3DtoPixel,
  buildExtrinsicMatrix,
} from '../math/CameraModel.js';

const VIEWPORT_W = 320;
const VIEWPORT_H = 240;

export default class CameraPanel {
  /**
   * @param {Object} opts
   * @param {HTMLElement} opts.mountRoot  - Parent element for the FloatingPanel
   * @param {number}      [opts.startX=20]
   * @param {number}      [opts.startY=60]
   * @param {Function}    opts.onChange   - Called with camera params when any slider changes
   * @param {Function}    opts.getEEPos  - Returns current EE position [x,y,z] in DH frame
   */
  constructor(opts) {
    this._mountRoot = opts.mountRoot;
    this._startX = opts.startX ?? 20;
    this._startY = opts.startY ?? 60;
    this._onChange = opts.onChange || (() => {});
    this._getEEPos = opts.getEEPos || (() => [0, 0, 0]);
    this._panel = null;
    this._mounted = false;

    // Camera parameters (defaults: looking down at table from above)
    this._params = {
      tx: 0.30, ty: 1.20, tz: 0.00,
      rx: -90, ry: 0, rz: 0,
      fov: 60,
      cx: VIEWPORT_W / 2,
      cy: VIEWPORT_H / 2,
    };

    // DOM refs
    this._viewport = null;
    this._kMatrixEl = null;
    this._projReadout = null;
    this._sliders = {};
  }

  _ensureMounted() {
    if (this._mounted) return;
    this._mounted = true;

    const container = document.createElement('div');
    container.style.cssText = 'display:flex;flex-direction:column;gap:10px;width:340px;';

    // ── Camera viewport ──
    const vpWrap = document.createElement('div');
    vpWrap.style.cssText = `
      position:relative;border-radius:10px;overflow:hidden;
      border:1px solid rgba(0,229,255,0.2);
      background:rgba(0,0,0,0.4);
    `;
    const vpLabel = document.createElement('div');
    vpLabel.style.cssText = `
      position:absolute;top:6px;left:8px;font-size:9px;font-weight:600;
      color:rgba(0,229,255,0.7);letter-spacing:0.08em;
      text-transform:uppercase;z-index:2;
      font-family:'JetBrains Mono',monospace;
      text-shadow:0 1px 4px rgba(0,0,0,0.8);
    `;
    vpLabel.textContent = '📷 PERCEPTION FEED';
    vpWrap.appendChild(vpLabel);

    // Crosshair overlay
    const crosshair = document.createElement('div');
    crosshair.style.cssText = `
      position:absolute;inset:0;z-index:1;pointer-events:none;
      background:
        linear-gradient(rgba(0,229,255,0.1) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0,229,255,0.1) 1px, transparent 1px);
      background-size:40px 40px;
    `;
    vpWrap.appendChild(crosshair);

    // Center crosshair
    const centerX = document.createElement('div');
    centerX.style.cssText = `
      position:absolute;top:50%;left:calc(50% - 12px);width:24px;height:1px;
      background:rgba(255,204,0,0.5);z-index:2;pointer-events:none;
    `;
    const centerY = document.createElement('div');
    centerY.style.cssText = `
      position:absolute;top:calc(50% - 12px);left:50%;width:1px;height:24px;
      background:rgba(255,204,0,0.5);z-index:2;pointer-events:none;
    `;
    vpWrap.appendChild(centerX);
    vpWrap.appendChild(centerY);

    this._viewport = document.createElement('canvas');
    this._viewport.width = VIEWPORT_W;
    this._viewport.height = VIEWPORT_H;
    this._viewport.style.cssText = `width:100%;height:auto;display:block;`;
    vpWrap.appendChild(this._viewport);
    container.appendChild(vpWrap);

    // ── Extrinsic sliders ──
    this._addSection(container, 'Extrinsic Parameters [R|t]');

    const extSliders = [
      { id: 'cam-tx', label: 'tₓ (m)',  min: -1.5, max: 1.5, step: 0.01, key: 'tx' },
      { id: 'cam-ty', label: 'tᵧ (m)',  min: 0.2,  max: 2.5, step: 0.01, key: 'ty' },
      { id: 'cam-tz', label: 'tᵤ (m)',  min: -1.5, max: 1.5, step: 0.01, key: 'tz' },
      { id: 'cam-rx', label: 'rₓ (°)',  min: -180, max: 180, step: 1,    key: 'rx' },
      { id: 'cam-ry', label: 'rᵧ (°)',  min: -180, max: 180, step: 1,    key: 'ry' },
      { id: 'cam-rz', label: 'rᵤ (°)',  min: -180, max: 180, step: 1,    key: 'rz' },
    ];
    extSliders.forEach(s => this._addSlider(container, s));

    // ── Intrinsic sliders ──
    this._addSection(container, 'Intrinsic Parameters K');

    this._addSlider(container, {
      id: 'cam-fov', label: 'FOV (°)', min: 15, max: 120, step: 1, key: 'fov'
    });

    // ── K matrix readout ──
    this._kMatrixEl = document.createElement('div');
    this._kMatrixEl.style.cssText = `
      background:rgba(0,0,0,0.25);border-radius:8px;padding:8px 10px;
      font-family:'JetBrains Mono',monospace;font-size:10px;
      color:rgba(255,255,255,0.6);border:1px solid rgba(255,255,255,0.06);
      line-height:1.6;white-space:pre;
    `;
    container.appendChild(this._kMatrixEl);

    // ── EE Projection readout ──
    this._addSection(container, 'EE Projection (u, v)');
    this._projReadout = document.createElement('div');
    this._projReadout.style.cssText = `
      display:flex;gap:6px;
    `;
    const uCard = this._makeReadoutCard('u (px)', '#00e5ff');
    const vCard = this._makeReadoutCard('v (px)', '#ffcc00');
    const depthCard = this._makeReadoutCard('depth (m)', '#6c5ce7');
    this._projU = uCard.val;
    this._projV = vCard.val;
    this._projDepth = depthCard.val;
    this._projReadout.appendChild(uCard.el);
    this._projReadout.appendChild(vCard.el);
    this._projReadout.appendChild(depthCard.el);
    container.appendChild(this._projReadout);

    // ── Float panel ──
    this._panel = new FloatingPanel({
      id: 'fp-camera',
      title: 'Perception Camera',
      icon: '📷',
      contentEl: container,
      startX: this._startX,
      startY: this._startY,
      startHidden: false,
    });
    this._panel.mount(this._mountRoot);

    this._updateKMatrix();
    this._fireChange();
  }

  // ─── DOM Helpers ───

  _addSection(parent, text) {
    const sec = document.createElement('div');
    sec.className = 'sec';
    sec.textContent = text;
    parent.appendChild(sec);
  }

  _addSlider(parent, { id, label, min, max, step, key }) {
    const wrap = document.createElement('div');
    wrap.style.cssText = 'display:flex;align-items:center;gap:8px;';

    const lbl = document.createElement('label');
    lbl.style.cssText = `
      flex:0 0 60px;font-size:11px;color:rgba(255,255,255,0.5);
      font-family:'JetBrains Mono',monospace;
    `;
    lbl.textContent = label;

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.id = id;
    slider.min = min;
    slider.max = max;
    slider.step = step;
    slider.value = this._params[key];
    slider.style.cssText = 'flex:1;';
    slider.className = 'cam-slider';

    const val = document.createElement('span');
    val.style.cssText = `
      flex:0 0 48px;text-align:right;font-size:10px;
      color:rgba(255,255,255,0.7);font-family:'JetBrains Mono',monospace;
      font-variant-numeric:tabular-nums;
    `;
    val.textContent = Number(this._params[key]).toFixed(step < 1 ? 2 : 0);

    slider.addEventListener('input', () => {
      this._params[key] = +slider.value;
      val.textContent = Number(slider.value).toFixed(step < 1 ? 2 : 0);
      this._updateKMatrix();
      this._fireChange();
    });

    this._sliders[key] = { slider, val };

    wrap.appendChild(lbl);
    wrap.appendChild(slider);
    wrap.appendChild(val);
    parent.appendChild(wrap);
  }

  _makeReadoutCard(label, color) {
    const el = document.createElement('div');
    el.style.cssText = `
      flex:1;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 10px;
      border:1px solid ${color}30;
    `;
    const lbl = document.createElement('div');
    lbl.style.cssText = `font-size:9px;color:${color};opacity:0.7;margin-bottom:2px;
      font-family:'JetBrains Mono',monospace;`;
    lbl.textContent = label;
    const val = document.createElement('div');
    val.style.cssText = `font-size:13px;font-weight:700;color:${color};
      font-family:'JetBrains Mono',monospace;font-variant-numeric:tabular-nums;`;
    val.textContent = '—';
    el.appendChild(lbl);
    el.appendChild(val);
    return { el, val };
  }

  // ─── Logic ───

  _updateKMatrix() {
    const f = fovToFocalLength(this._params.fov, VIEWPORT_H);
    const cx = this._params.cx;
    const cy = this._params.cy;
    const K = buildIntrinsicMatrix(f, f, cx, cy);

    if (this._kMatrixEl) {
      this._kMatrixEl.textContent =
        `K = [ ${K[0].toFixed(1)}   ${K[1].toFixed(0)}   ${K[2].toFixed(1)} ]\n` +
        `    [  ${K[3].toFixed(0)}   ${K[4].toFixed(1)}   ${K[5].toFixed(1)} ]\n` +
        `    [  ${K[6].toFixed(0)}    ${K[7].toFixed(0)}    ${K[8].toFixed(0)}  ]`;
    }
  }

  _fireChange() {
    const p = this._params;
    this._onChange({
      tx: p.tx, ty: p.ty, tz: p.tz,
      rx: p.rx * Math.PI / 180,
      ry: p.ry * Math.PI / 180,
      rz: p.rz * Math.PI / 180,
      fov: p.fov,
    });
  }

  /**
   * Update the EE projection readout. Call every frame from the UI controller.
   * @param {number[]} eePos - [x,y,z] in DH frame
   */
  updateProjection(eePos) {
    if (!this._mounted || !this._panel.isVisible) return;

    const p = this._params;
    const f = fovToFocalLength(p.fov, VIEWPORT_H);
    const K = buildIntrinsicMatrix(f, f, p.cx, p.cy);
    const Ext = buildExtrinsicMatrix(
      p.tx, p.ty, p.tz,
      p.rx * Math.PI / 180,
      p.ry * Math.PI / 180,
      p.rz * Math.PI / 180
    );

    // Project EE position (DH coords → camera coords)
    // DH has X,Y horizontal, Z up. Three.js has X,Z horizontal, Y up.
    // The extrinsic is in Three.js convention (what the scene camera sees)
    // so we convert DH→Three for projection: threeX=dhX, threeY=dhZ, threeZ=-dhY
    const proj = project3DtoPixel(eePos[0], eePos[2], -eePos[1], K, Ext);

    if (this._projU) this._projU.textContent = proj.valid ? proj.u.toFixed(1) : '—';
    if (this._projV) this._projV.textContent = proj.valid ? proj.v.toFixed(1) : '—';
    if (this._projDepth) this._projDepth.textContent = proj.valid ? proj.depth.toFixed(3) : '—';
  }

  /** Get the viewport canvas for external rendering */
  get viewportCanvas() {
    this._ensureMounted();
    return this._viewport;
  }

  /** Get current camera params */
  get params() { return { ...this._params }; }

  // ─── Visibility ───

  show() {
    this._ensureMounted();
    this._panel.show();
  }

  hide() {
    if (this._panel) this._panel.hide();
  }

  toggle() {
    if (!this._mounted) { this.show(); return; }
    this._panel.toggle();
  }

  get isVisible() {
    return this._panel ? this._panel.isVisible : false;
  }
}
