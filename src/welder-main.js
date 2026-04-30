import { WelderSceneManager } from './core/WelderSceneManager.js';
import { WelderUIController } from './ui/WelderUIController.js';

// Initialize the 3D Scene
const sceneManager = new WelderSceneManager('cw');

// Initialize the UI Controller and map it to the Scene
const uiController = new WelderUIController(sceneManager);

// Start the animation render loop
sceneManager.start();

// Give the UI an initial sync
uiController.update();
