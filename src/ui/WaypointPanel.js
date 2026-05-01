/**
 * WaypointPanel.js — Waypoint Planner UI (FloatingPanel-based)
 * PDF §Motion Planning: "Waypoint planner"
 *
 * Provides a draggable panel with:
 *  - Waypoint list with coordinates
 *  - Add (from current EE), Delete, Move Up/Down
 *  - Play / Stop / Loop toggle
 *  - Live progress readout
 *
 * Matches the JINX glassmorphism design language.
 */

import FloatingPanel from './FloatingPanel.js';

export default class WaypointPanel {
  /**
   * @param {Object} opts
   * @param {HTMLElement} opts.mountRoot  - Parent element (e.g. #cw)
   * @param {Function}    opts.getEEPos   - () => [x,y,z] current EE position
   * @param {Function}    opts.onPlay     - Called when Play is clicked
   * @param {Function}    opts.onStop     - Called when Stop is clicked
   * @param {Function}    opts.onAdd      - (wp) => void
   * @param {Function}    opts.onRemove   - (index) => void
   * @param {Function}    opts.onMove     - (fromIdx, toIdx) => void
   * @param {Function}    opts.onClear    - () => void
   * @param {Function}    opts.onModeChange - (mode) => void
   * @param {number}      [opts.startX=20]
   * @param {number}      [opts.startY=100]
   */
  constructor(opts) {
    this._opts = opts;
    this._mounted = false;
    this._panel = null;
    this._listEl = null;
    this._statusEl = null;
    this._progressEl = null;
    this._progressBar = null;
    this._segLabel = null;
    this._playBtn = null;
    this._loopBtn = null;
    this._isPlaying = false;
    this._isLoop = false;
  }

  // ═══════════════════════════════════════
  //  Lazy Mount
  // ═══════════════════════════════════════

  _ensureMounted() {
    if (this._mounted) return;
    this._mounted = true;

    const container = document.createElement('div');
    container.className = 'wp-panel-content';
    container.style.cssText = 'display:flex;flex-direction:column;gap:10px;width:310px;';

    // ── Section: Controls ──
    const ctrlRow = document.createElement('div');
    ctrlRow.style.cssText = 'display:flex;gap:6px;flex-wrap:wrap;';

    // Add Waypoint button
    const addBtn = this._makeBtn('＋ Add EE Pos', 'var(--cyan)', () => {
      const pos = this._opts.getEEPos();
      if (pos && this._opts.onAdd) this._opts.onAdd({ x: pos[0], y: pos[1], z: pos[2] });
    });
    addBtn.style.flex = '1';

    // Clear All button
    const clearBtn = this._makeBtn('🗑 Clear', 'var(--red2)', () => {
      if (this._opts.onClear) this._opts.onClear();
    });
    clearBtn.style.flex = '0 0 auto';

    ctrlRow.appendChild(addBtn);
    ctrlRow.appendChild(clearBtn);
    container.appendChild(ctrlRow);

    // ── Section: Waypoint List ──
    const listHeader = document.createElement('div');
    listHeader.className = 'sec';
    listHeader.textContent = 'Waypoints';
    container.appendChild(listHeader);

    this._listEl = document.createElement('div');
    this._listEl.className = 'wp-list';
    this._listEl.style.cssText = `
      max-height: 200px; overflow-y: auto; display: flex; flex-direction: column; gap: 4px;
      scrollbar-width: thin; scrollbar-color: var(--border) transparent;
    `;
    container.appendChild(this._listEl);

    // Empty state
    this._emptyMsg = document.createElement('div');
    this._emptyMsg.style.cssText = 'text-align:center;color:var(--muted);font-size:11px;padding:16px 0;opacity:0.6;';
    this._emptyMsg.textContent = 'No waypoints. Add the current EE position to begin.';
    this._listEl.appendChild(this._emptyMsg);

    // ── Section: Playback ──
    const playHeader = document.createElement('div');
    playHeader.className = 'sec';
    playHeader.textContent = 'Execution';
    container.appendChild(playHeader);

    const playRow = document.createElement('div');
    playRow.style.cssText = 'display:flex;gap:6px;';

    this._playBtn = document.createElement('button');
    this._playBtn.className = 'sbtn';
    this._playBtn.textContent = '▶ Play Path';
    this._playBtn.style.cssText = 'flex:1;padding:10px;font-size:12px;';
    this._playBtn.addEventListener('click', () => {
      if (this._isPlaying) {
        if (this._opts.onStop) this._opts.onStop();
      } else {
        if (this._opts.onPlay) this._opts.onPlay();
      }
    });

    this._loopBtn = document.createElement('button');
    this._loopBtn.style.cssText = `
      width:38px;height:38px;border-radius:10px;border:1px solid var(--border);
      background:rgba(255,255,255,0.04);color:var(--muted);font-size:16px;
      cursor:pointer;transition:all 0.2s;display:flex;align-items:center;justify-content:center;
    `;
    this._loopBtn.title = 'Toggle loop mode';
    this._loopBtn.textContent = '🔁';
    this._loopBtn.addEventListener('click', () => {
      this._isLoop = !this._isLoop;
      this._loopBtn.style.background = this._isLoop
        ? 'rgba(108,92,231,0.25)' : 'rgba(255,255,255,0.04)';
      this._loopBtn.style.borderColor = this._isLoop
        ? 'rgba(108,92,231,0.5)' : 'var(--border)';
      this._loopBtn.style.color = this._isLoop ? '#a855f7' : 'var(--muted)';
      if (this._opts.onModeChange) this._opts.onModeChange(this._isLoop ? 'loop' : 'once');
    });

    playRow.appendChild(this._playBtn);
    playRow.appendChild(this._loopBtn);
    container.appendChild(playRow);

    // ── Status readout ──
    const statusRow = document.createElement('div');
    statusRow.style.cssText = 'display:flex;gap:8px;';

    const stateCard = this._makeCard('State', 'IDLE');
    this._statusEl = stateCard.querySelector('.wp-cv');

    const progCard = this._makeCard('Progress', '0%');
    this._progressEl = progCard.querySelector('.wp-cv');

    const segCard = this._makeCard('Segment', '—');
    this._segLabel = segCard.querySelector('.wp-cv');

    statusRow.appendChild(stateCard);
    statusRow.appendChild(progCard);
    statusRow.appendChild(segCard);
    container.appendChild(statusRow);

    // Progress bar
    const barWrap = document.createElement('div');
    barWrap.className = 'mu-bar-wrap';
    this._progressBar = document.createElement('div');
    this._progressBar.className = 'mu-bar';
    this._progressBar.style.cssText = 'width:0%;background:var(--cyan);';
    barWrap.appendChild(this._progressBar);
    container.appendChild(barWrap);

    // ── Mount FloatingPanel ──
    this._panel = new FloatingPanel({
      id: 'fp-waypoints',
      title: 'Waypoint Planner',
      icon: '📍',
      contentEl: container,
      startX: this._opts.startX ?? 20,
      startY: this._opts.startY ?? 100,
      startHidden: false,
    });
    this._panel.mount(this._opts.mountRoot);
  }

  // ═══════════════════════════════════════
  //  Public API
  // ═══════════════════════════════════════

  show() { this._ensureMounted(); this._panel.show(); }
  hide() { if (this._panel) this._panel.hide(); }
  toggle() { if (!this._mounted) { this.show(); return; } this._panel.toggle(); }
  get isVisible() { return this._panel ? this._panel.isVisible : false; }

  /**
   * Refresh the waypoint list display.
   * @param {{ x: number, y: number, z: number }[]} waypoints
   */
  refreshList(waypoints) {
    if (!this._mounted) return;
    this._listEl.innerHTML = '';

    if (waypoints.length === 0) {
      this._listEl.appendChild(this._emptyMsg);
      return;
    }

    waypoints.forEach((wp, i) => {
      const row = document.createElement('div');
      row.style.cssText = `
        display:flex;align-items:center;gap:6px;padding:6px 8px;
        background:rgba(0,0,0,0.2);border:1px solid var(--border);border-radius:8px;
        font-family:'JetBrains Mono',monospace;font-size:10px;color:var(--text);
        transition:border-color 0.2s;
      `;
      row.addEventListener('mouseenter', () => row.style.borderColor = 'rgba(255,255,255,0.18)');
      row.addEventListener('mouseleave', () => row.style.borderColor = 'var(--border)');

      // Index badge
      const idx = document.createElement('span');
      idx.style.cssText = `
        min-width:20px;height:20px;border-radius:5px;display:flex;align-items:center;
        justify-content:center;font-size:9px;font-weight:700;
        background:rgba(255,204,0,0.12);color:var(--gold);flex-shrink:0;
      `;
      idx.textContent = i + 1;

      // Coords
      const coords = document.createElement('span');
      coords.style.cssText = 'flex:1;font-variant-numeric:tabular-nums;letter-spacing:-0.02em;';
      coords.textContent = `${wp.x.toFixed(3)}, ${wp.y.toFixed(3)}, ${wp.z.toFixed(3)}`;

      // Buttons
      const btnWrap = document.createElement('span');
      btnWrap.style.cssText = 'display:flex;gap:2px;flex-shrink:0;';

      if (i > 0) btnWrap.appendChild(this._makeSmallBtn('▲', () => {
        if (this._opts.onMove) this._opts.onMove(i, i - 1);
      }));
      if (i < waypoints.length - 1) btnWrap.appendChild(this._makeSmallBtn('▼', () => {
        if (this._opts.onMove) this._opts.onMove(i, i + 1);
      }));
      btnWrap.appendChild(this._makeSmallBtn('✕', () => {
        if (this._opts.onRemove) this._opts.onRemove(i);
      }, true));

      row.appendChild(idx);
      row.appendChild(coords);
      row.appendChild(btnWrap);
      this._listEl.appendChild(row);
    });
  }

  /**
   * Update the execution status display.
   * @param {string} state
   * @param {number} progress - 0..1
   * @param {number} segment
   * @param {number} totalSegments
   */
  updateStatus(state, progress, segment, totalSegments) {
    if (!this._mounted) return;
    if (this._statusEl) this._statusEl.textContent = state;
    if (this._progressEl) this._progressEl.textContent = `${Math.round(progress * 100)}%`;
    if (this._segLabel) {
      this._segLabel.textContent = totalSegments > 0 ? `${segment + 1}/${totalSegments}` : '—';
    }
    if (this._progressBar) this._progressBar.style.width = `${Math.round(progress * 100)}%`;
  }

  /**
   * Update play/stop button state.
   * @param {boolean} isPlaying
   */
  setPlaying(isPlaying) {
    this._isPlaying = isPlaying;
    if (!this._playBtn) return;
    if (isPlaying) {
      this._playBtn.textContent = '⏹ Stop';
      this._playBtn.classList.add('stop');
    } else {
      this._playBtn.textContent = '▶ Play Path';
      this._playBtn.classList.remove('stop');
    }
  }

  // ═══════════════════════════════════════
  //  DOM Helpers
  // ═══════════════════════════════════════

  _makeBtn(text, color, onClick) {
    const btn = document.createElement('button');
    btn.textContent = text;
    btn.style.cssText = `
      padding:8px 12px;border-radius:8px;border:1px solid ${color};
      background:transparent;color:${color};font-size:11px;font-weight:600;
      cursor:pointer;transition:all 0.2s;font-family:'Inter',sans-serif;
    `;
    btn.addEventListener('mouseenter', () => { btn.style.background = `${color}15`; });
    btn.addEventListener('mouseleave', () => { btn.style.background = 'transparent'; });
    btn.addEventListener('click', onClick);
    return btn;
  }

  _makeSmallBtn(text, onClick, isDanger = false) {
    const btn = document.createElement('button');
    btn.textContent = text;
    const c = isDanger ? 'var(--red2)' : 'var(--muted)';
    btn.style.cssText = `
      width:22px;height:22px;border-radius:5px;border:1px solid var(--border);
      background:rgba(255,255,255,0.03);color:${c};font-size:10px;
      cursor:pointer;display:flex;align-items:center;justify-content:center;
      transition:all 0.15s;padding:0;line-height:1;
    `;
    btn.addEventListener('mouseenter', () => {
      btn.style.background = isDanger ? 'rgba(255,77,77,0.2)' : 'rgba(255,255,255,0.1)';
    });
    btn.addEventListener('mouseleave', () => {
      btn.style.background = 'rgba(255,255,255,0.03)';
    });
    btn.addEventListener('click', (e) => { e.stopPropagation(); onClick(); });
    return btn;
  }

  _makeCard(label, value) {
    const card = document.createElement('div');
    card.style.cssText = `
      flex:1;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 10px;
      border:1px solid var(--border);
    `;
    const cl = document.createElement('div');
    cl.className = 'cl';
    cl.textContent = label;

    const cv = document.createElement('div');
    cv.className = 'wp-cv';
    cv.style.cssText = `
      font-size:12px;font-weight:600;color:#fff;
      font-family:'JetBrains Mono',monospace;font-variant-numeric:tabular-nums;
    `;
    cv.textContent = value;

    card.appendChild(cl);
    card.appendChild(cv);
    return card;
  }
}
