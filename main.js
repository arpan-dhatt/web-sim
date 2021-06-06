import * as THREE from './three.module.js';
import { OrbitControls } from './orbit-controls.js';

let KEYS = {};
document.addEventListener("keydown", event => {
	KEYS[event.key] = true
	console.log(event.key, true);
});

document.addEventListener("keyup", event => {
	KEYS[event.key] = false
	console.log(event.key, false);
});


init().then(() => {
}).catch(console.error);

