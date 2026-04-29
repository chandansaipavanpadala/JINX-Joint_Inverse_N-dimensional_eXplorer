/**
 * main.js — JINX Application Entry Point
 *
 * Bootstraps the 3D scene and wires the UI controller.
 * Loaded as an ES module from rrr-lamp.html.
 */

import SceneManager from './core/SceneManager.js';
import UIController from './ui/UIController.js';

// Instantiate the 3D scene (renders into #cw, POV into #camPov)
const sceneManager = new SceneManager('cw', 'camPov');

// Instantiate the UI controller (wires sliders, buttons, raycaster)
const uiController = new UIController(sceneManager);

// Start the render loop
sceneManager.start();

// Initial calculation and layout
uiController.update();
uiController.drawTrapProfile();
uiController.computeCameraIK();
