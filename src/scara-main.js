/**
 * scara-main.js — JINX SCARA Arm Entry Point
 *
 * Bootstraps the 3D scene and wires the UI controller.
 * Loaded as an ES module from src/pages/scara.html.
 */

import ScaraSceneManager from './core/ScaraSceneManager.js';
import ScaraUIController from './ui/ScaraUIController.js';

// Instantiate the 3D scene (renders into #cw, POV into #camPov)
const sceneManager = new ScaraSceneManager('cw', 'camPov');

// Instantiate the UI controller (wires sliders, raycaster, DLS IK)
const uiController = new ScaraUIController(sceneManager);

// Start the render loop
sceneManager.start();

// Initial calculation and layout
uiController.update();
