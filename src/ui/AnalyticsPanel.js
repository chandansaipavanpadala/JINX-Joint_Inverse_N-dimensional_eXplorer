/**
 * AnalyticsPanel.js — Real-time MATLAB-style plotting for JINX.
 * PDF §4.6 F-6: "All values update at 60 fps."
 *
 * Houses three live-updating charts inside a FloatingPanel:
 *   1. EE Position — Target vs Actual (XYZ or Z-only)
 *   2. Cartesian Error — ‖target − actual‖ over time
 *   3. Manipulability μ — kinematic health over time
 *
 * Uses Chart.js with dark-theme configuration and capped data arrays
 * to prevent memory leaks during long-running simulations.
 */

import { Chart, registerables } from 'chart.js';
import FloatingPanel from './FloatingPanel.js';

Chart.register(...registerables);

const MAX_POINTS = 120;

/* ── Chart.js dark theme defaults ── */
const GRID_COLOR = 'rgba(255, 255, 255, 0.06)';
const TICK_COLOR = 'rgba(255, 255, 255, 0.35)';
const FONT_FAMILY = "'JetBrains Mono', monospace";
const COLORS = {
  cyan:    '#00e5ff',
  gold:    '#ffcc00',
  accent:  '#6c5ce7',
  green:   '#00f07f',
  red:     '#ff5c5c',
  axisX:   '#ff5c5c',
  axisY:   '#4ddf6f',
  axisZ:   '#4da6ff',
};

function makeDarkScaleOpts(yLabel, suggestedMin, suggestedMax) {
  return {
    x: {
      display: true,
      title: { display: true, text: 'Time (s)', color: TICK_COLOR, font: { family: FONT_FAMILY, size: 9 } },
      ticks: { color: TICK_COLOR, font: { family: FONT_FAMILY, size: 8 }, maxTicksLimit: 6 },
      grid: { color: GRID_COLOR },
    },
    y: {
      display: true,
      title: { display: true, text: yLabel, color: TICK_COLOR, font: { family: FONT_FAMILY, size: 9 } },
      ticks: { color: TICK_COLOR, font: { family: FONT_FAMILY, size: 8 }, maxTicksLimit: 5 },
      grid: { color: GRID_COLOR },
      suggestedMin,
      suggestedMax,
    },
  };
}

function makeChartConfig(datasets, yLabel, suggestedMin, suggestedMax) {
  return {
    type: 'line',
    data: { labels: [], datasets },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { intersect: false, mode: 'index' },
      plugins: {
        legend: {
          display: datasets.length > 1,
          position: 'top',
          labels: {
            color: TICK_COLOR,
            font: { family: FONT_FAMILY, size: 9 },
            boxWidth: 10, boxHeight: 2, padding: 8,
          },
        },
        tooltip: {
          enabled: true,
          backgroundColor: 'rgba(10, 10, 22, 0.92)',
          titleColor: '#fff',
          bodyColor: TICK_COLOR,
          borderColor: 'rgba(255,255,255,0.1)',
          borderWidth: 1,
          titleFont: { family: FONT_FAMILY, size: 10 },
          bodyFont: { family: FONT_FAMILY, size: 9 },
        },
      },
      scales: makeDarkScaleOpts(yLabel, suggestedMin, suggestedMax),
      elements: {
        point: { radius: 0, hoverRadius: 3 },
        line: { tension: 0.25, borderWidth: 1.8 },
      },
      layout: { padding: { left: 2, right: 8, top: 4, bottom: 2 } },
    },
  };
}

function makeDataset(label, color, dashed = false) {
  return {
    label,
    data: [],
    borderColor: color,
    backgroundColor: color + '18',
    fill: false,
    borderDash: dashed ? [4, 3] : [],
  };
}

export default class AnalyticsPanel {
  /**
   * @param {string}      title     - Panel title (e.g. "SCARA Analytics")
   * @param {HTMLElement}  mountRoot - Parent element to mount the floating panel into
   * @param {number}       [startX=20]
   * @param {number}       [startY=60]
   */
  constructor(title, mountRoot, startX = 20, startY = 60) {
    this._mounted = false;
    this._title = title;
    this._mountRoot = mountRoot;
    this._startX = startX;
    this._startY = startY;
    this._time = 0;
    this._frameSkip = 0;

    // Data arrays (capped at MAX_POINTS)
    this._labels = [];
    this._posXTarget = []; this._posYTarget = []; this._posZTarget = [];
    this._posXActual = []; this._posYActual = []; this._posZActual = [];
    this._errorData = [];
    this._muData = [];

    // Chart instances
    this._posChart = null;
    this._errChart = null;
    this._muChart = null;

    // FloatingPanel instance
    this._panel = null;
  }

  /** Lazily build DOM + charts on first show */
  _ensureMounted() {
    if (this._mounted) return;
    this._mounted = true;

    // ── Build the content container ──
    const container = document.createElement('div');
    container.style.cssText = 'display:flex;flex-direction:column;gap:12px;width:380px;';

    // Section helper
    const makeSection = (label, height) => {
      const wrap = document.createElement('div');
      wrap.style.cssText = 'display:flex;flex-direction:column;gap:4px;';
      const lbl = document.createElement('div');
      lbl.className = 'sec';
      lbl.textContent = label;
      const cvs = document.createElement('canvas');
      cvs.style.cssText = `height:${height}px;width:100%;background:rgba(0,0,0,0.15);border-radius:8px;border:1px solid rgba(255,255,255,0.06);`;
      wrap.appendChild(lbl);
      wrap.appendChild(cvs);
      container.appendChild(wrap);
      return cvs;
    };

    const posCvs = makeSection('EE Position — Target vs Actual', 140);
    const errCvs = makeSection('Cartesian Error ‖e‖', 100);
    const muCvs  = makeSection('Manipulability μ', 100);

    // ── Create FloatingPanel ──
    this._panel = new FloatingPanel({
      id: 'fp-analytics',
      title: this._title,
      icon: '📈',
      contentEl: container,
      startX: this._startX,
      startY: this._startY,
      startHidden: false,
    });
    this._panel.mount(this._mountRoot);

    // ── Create Chart instances ──
    this._posChart = new Chart(posCvs.getContext('2d'), makeChartConfig([
      makeDataset('Target Z', COLORS.gold, true),
      makeDataset('Actual Z', COLORS.cyan),
      makeDataset('Actual X', COLORS.axisX),
      makeDataset('Actual Y', COLORS.axisY),
    ], 'Position (m)', -0.2, 0.8));

    this._errChart = new Chart(errCvs.getContext('2d'), makeChartConfig([
      makeDataset('‖e‖', COLORS.red),
    ], 'Error (m)', 0, 0.05));

    this._muChart = new Chart(muCvs.getContext('2d'), makeChartConfig([
      makeDataset('μ', COLORS.accent),
    ], 'Manipulability', 0, 0.05));
  }

  /**
   * Push one data frame. Call from the task loop every frame.
   * @param {number}   dt         - Delta time since last frame (seconds)
   * @param {number[]} targetPos  - [x, y, z] target in DH frame
   * @param {number[]} actualPos  - [x, y, z] actual EE in DH frame
   * @param {number}   mu         - Manipulability index
   */
  updateData(dt, targetPos, actualPos, mu) {
    if (!this._mounted || !this._panel.isVisible) return;

    // Throttle to every 2nd frame for performance
    this._frameSkip++;
    if (this._frameSkip % 2 !== 0) return;

    this._time += dt * 2; // Compensate for frame skip

    const error = Math.sqrt(
      (targetPos[0] - actualPos[0]) ** 2 +
      (targetPos[1] - actualPos[1]) ** 2 +
      (targetPos[2] - actualPos[2]) ** 2
    );

    // Push
    const label = this._time.toFixed(1);
    this._labels.push(label);
    this._posChart.data.datasets[0].data.push(targetPos[2]);
    this._posChart.data.datasets[1].data.push(actualPos[2]);
    this._posChart.data.datasets[2].data.push(actualPos[0]);
    this._posChart.data.datasets[3].data.push(actualPos[1]);
    this._errChart.data.datasets[0].data.push(error);
    this._muChart.data.datasets[0].data.push(mu);

    // Cap at MAX_POINTS
    if (this._labels.length > MAX_POINTS) {
      this._labels.shift();
      this._posChart.data.datasets.forEach(ds => ds.data.shift());
      this._errChart.data.datasets[0].data.shift();
      this._muChart.data.datasets[0].data.shift();
    }

    // Sync labels
    this._posChart.data.labels = this._labels;
    this._errChart.data.labels = this._labels;
    this._muChart.data.labels = this._labels;

    // Update charts
    this._posChart.update('none');
    this._errChart.update('none');
    this._muChart.update('none');
  }

  /** Reset all chart data (call when starting a new task) */
  reset() {
    this._time = 0;
    this._frameSkip = 0;
    this._labels.length = 0;
    if (this._posChart) {
      this._posChart.data.labels = [];
      this._posChart.data.datasets.forEach(ds => { ds.data.length = 0; });
      this._posChart.update('none');
    }
    if (this._errChart) {
      this._errChart.data.labels = [];
      this._errChart.data.datasets[0].data.length = 0;
      this._errChart.update('none');
    }
    if (this._muChart) {
      this._muChart.data.labels = [];
      this._muChart.data.datasets[0].data.length = 0;
      this._muChart.update('none');
    }
  }

  /** Show the panel (lazy-mounts on first call) */
  show() {
    this._ensureMounted();
    this._panel.show();
  }

  /** Hide the panel */
  hide() {
    if (this._panel) this._panel.hide();
  }

  /** Toggle visibility */
  toggle() {
    if (!this._mounted) { this.show(); return; }
    this._panel.toggle();
  }

  get isVisible() {
    return this._panel ? this._panel.isVisible : false;
  }
}
