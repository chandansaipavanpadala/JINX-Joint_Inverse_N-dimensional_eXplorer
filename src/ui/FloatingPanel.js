/**
 * FloatingPanel.js — Draggable, collapsible floating controller panels.
 * PDF §4.5 F-5: "Each controller operates in its own independently
 * resizable floating panel. All panels may be open simultaneously."
 *
 * Usage:
 *   const panel = new FloatingPanel({
 *     id: 'fp-fk',
 *     title: 'Forward Kinematics',
 *     icon: '🎛️',
 *     contentEl: document.getElementById('pane-fk'),
 *     startX: 20, startY: 80,
 *     startCollapsed: false
 *   });
 *   panel.mount(document.getElementById('cw'));
 */

let _topZ = 60;

export default class FloatingPanel {
  /**
   * @param {Object} opts
   * @param {string} opts.id        - Unique DOM id
   * @param {string} opts.title     - Header title text
   * @param {string} opts.icon      - Emoji or SVG icon
   * @param {HTMLElement} opts.contentEl - Existing DOM element to move into the panel body
   * @param {number} [opts.startX=20]
   * @param {number} [opts.startY=80]
   * @param {boolean} [opts.startCollapsed=false]
   * @param {boolean} [opts.startHidden=false]
   */
  constructor(opts) {
    this.id = opts.id;
    this.title = opts.title;
    this.icon = opts.icon || '⚙';
    this.contentEl = opts.contentEl;
    this.startX = opts.startX ?? 20;
    this.startY = opts.startY ?? 80;
    this.collapsed = opts.startCollapsed ?? false;
    this.hidden = opts.startHidden ?? false;

    this._el = null;
    this._header = null;
    this._body = null;
    this._dragging = false;
    this._dragOff = { x: 0, y: 0 };

    this._onMouseDown = this._onMouseDown.bind(this);
    this._onMouseMove = this._onMouseMove.bind(this);
    this._onMouseUp = this._onMouseUp.bind(this);
  }

  /** Build DOM and append to parent */
  mount(parent) {
    const el = document.createElement('div');
    el.className = 'floating-panel' +
      (this.collapsed ? ' fp-collapsed' : '') +
      (this.hidden ? ' fp-hidden' : '');
    el.id = this.id;
    el.style.left = this.startX + 'px';
    el.style.top = this.startY + 'px';
    el.style.zIndex = _topZ;

    // ── Header (drag handle) ──
    const header = document.createElement('div');
    header.className = 'fp-header';
    header.innerHTML = `
      <span class="fp-icon">${this.icon}</span>
      <span class="fp-title">${this.title}</span>
    `;

    // Collapse button
    const colBtn = document.createElement('button');
    colBtn.className = 'fp-btn fp-collapse';
    colBtn.innerHTML = this.collapsed ? '▼' : '▲';
    colBtn.title = 'Collapse / Expand';
    colBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      this.toggleCollapse();
      colBtn.innerHTML = this.collapsed ? '▼' : '▲';
    });
    header.appendChild(colBtn);

    // Close button
    const closeBtn = document.createElement('button');
    closeBtn.className = 'fp-btn fp-close';
    closeBtn.innerHTML = '✕';
    closeBtn.title = 'Close panel';
    closeBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      this.hide();
    });
    header.appendChild(closeBtn);

    this._header = header;
    el.appendChild(header);

    // ── Body ──
    const body = document.createElement('div');
    body.className = 'fp-body';

    // Move content element into the body
    if (this.contentEl) {
      // Force display (remove pane hiding if needed)
      this.contentEl.style.display = 'flex';
      this.contentEl.classList.remove('pane');
      this.contentEl.classList.add('on');
      body.appendChild(this.contentEl);
    }
    this._body = body;
    el.appendChild(body);

    this._el = el;
    parent.appendChild(el);

    // ── Drag events ──
    header.addEventListener('mousedown', this._onMouseDown);
    header.addEventListener('touchstart', this._onTouchStart.bind(this), { passive: false });

    // Bring to front on click
    el.addEventListener('mousedown', () => this._bringToFront());
  }

  _bringToFront() {
    _topZ++;
    this._el.style.zIndex = _topZ;
    // Visual feedback
    document.querySelectorAll('.floating-panel').forEach(p => p.classList.remove('fp-active'));
    this._el.classList.add('fp-active');
  }

  _onMouseDown(e) {
    if (e.target.closest('.fp-btn')) return; // Don't drag when clicking buttons
    this._dragging = true;
    this._bringToFront();
    const rect = this._el.getBoundingClientRect();
    this._dragOff.x = e.clientX - rect.left;
    this._dragOff.y = e.clientY - rect.top;
    window.addEventListener('mousemove', this._onMouseMove);
    window.addEventListener('mouseup', this._onMouseUp);
    e.preventDefault();
  }

  _onMouseMove(e) {
    if (!this._dragging) return;
    const parent = this._el.parentElement;
    const pr = parent.getBoundingClientRect();
    let x = e.clientX - pr.left - this._dragOff.x;
    let y = e.clientY - pr.top - this._dragOff.y;
    // Clamp to parent bounds (keep at least 60px visible)
    x = Math.max(-this._el.offsetWidth + 60, Math.min(pr.width - 60, x));
    y = Math.max(0, Math.min(pr.height - 30, y));
    this._el.style.left = x + 'px';
    this._el.style.top = y + 'px';
  }

  _onMouseUp() {
    this._dragging = false;
    window.removeEventListener('mousemove', this._onMouseMove);
    window.removeEventListener('mouseup', this._onMouseUp);
  }

  // Touch support
  _onTouchStart(e) {
    if (e.target.closest('.fp-btn')) return;
    const touch = e.touches[0];
    this._dragging = true;
    this._bringToFront();
    const rect = this._el.getBoundingClientRect();
    this._dragOff.x = touch.clientX - rect.left;
    this._dragOff.y = touch.clientY - rect.top;
    const onMove = (ev) => {
      const t = ev.touches[0];
      const parent = this._el.parentElement;
      const pr = parent.getBoundingClientRect();
      let x = t.clientX - pr.left - this._dragOff.x;
      let y = t.clientY - pr.top - this._dragOff.y;
      x = Math.max(-this._el.offsetWidth + 60, Math.min(pr.width - 60, x));
      y = Math.max(0, Math.min(pr.height - 30, y));
      this._el.style.left = x + 'px';
      this._el.style.top = y + 'px';
      ev.preventDefault();
    };
    const onEnd = () => {
      this._dragging = false;
      window.removeEventListener('touchmove', onMove);
      window.removeEventListener('touchend', onEnd);
    };
    window.addEventListener('touchmove', onMove, { passive: false });
    window.addEventListener('touchend', onEnd);
    e.preventDefault();
  }

  toggleCollapse() {
    this.collapsed = !this.collapsed;
    this._el.classList.toggle('fp-collapsed', this.collapsed);
  }

  show() {
    this.hidden = false;
    this._el.classList.remove('fp-hidden');
    this._bringToFront();
  }

  hide() {
    this.hidden = true;
    this._el.classList.add('fp-hidden');
  }

  toggle() {
    if (this.hidden) this.show(); else this.hide();
  }

  get element() { return this._el; }
  get isVisible() { return !this.hidden; }
}

/**
 * Utility: Create a panel toggle toolbar
 * @param {FloatingPanel[]} panels
 * @param {HTMLElement} container - Where to append the toolbar buttons
 */
export function createPanelToolbar(panels, container) {
  const toolbar = document.createElement('div');
  toolbar.className = 'fp-toolbar';

  panels.forEach(panel => {
    const btn = document.createElement('button');
    btn.className = 'fp-toolbar-btn' + (panel.isVisible ? ' active' : '');
    btn.textContent = panel.title;
    btn.title = `Toggle ${panel.title} panel`;
    btn.addEventListener('click', () => {
      panel.toggle();
      btn.classList.toggle('active', panel.isVisible);
    });
    toolbar.appendChild(btn);
  });

  container.appendChild(toolbar);
  return toolbar;
}
