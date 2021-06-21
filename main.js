import * as THREE from './three.module.js';
import init, {World} from './sim-engine/pkg/sim_engine.js';
import {Drone} from './drone.js';
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

function control_data() {
	let data = new Float32Array(4);
	if (KEYS["w"] == true) {
		data[0] = 1.0;
	}
	if (KEYS["q"] == true) {
		data[1] = 1.0;
	}
	if (KEYS["a"] == true) {
		data[2] = 1.0;
	}
	if (KEYS["s"] == true) {
		data[3] = 1.0;
	}
	return data;
}

init().then(() => {

	let {scene, camera, renderer, controls} = UTILS.setup();
	let world = World.new();

	let drone = new Drone(scene, world, 1, 1, 1, 1);
	let ground = new UTILS.Ground(scene, world, -1.1);

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
		let data = control_data();
		//let data = maybe_hover(drone);
		world.update_controls(data);
		let new_transforms = world.step();
		drone.update_transform(new_transforms);
		renderer.render(scene, camera);
	}
	animate();
}).catch(console.error);

