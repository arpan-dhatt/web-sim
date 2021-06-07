import * as THREE from './three.module.js';
import init from './sim-engine/pkg/sim_engine.js';
import { Drone } from './drone.js';
import * as UTILS from './utils.js';

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

	let { scene, camera, renderer, controls} = UTILS.setup();

	let drone = new Drone(scene, 1, 1, 1, 1);
	let ground = new UTILS.Ground(scene, -1.1);

	const light = new THREE.PointLight(0xffffff, 1);
	light.castShadow = true;
	light.translateZ(-10);
	light.translateY(3);
	scene.add(light);
	
	camera.translateZ(-5);

	controls.update();
	function animate() {
		requestAnimationFrame(animate);
		controls.update();
		renderer.render(scene, camera);
	}
	animate();
}).catch(console.error);

