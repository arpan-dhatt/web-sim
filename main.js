import * as THREE from './three.module.js';
import {OrbitControls} from './orbit-controls.js';
import init from './sim-engine/pkg/sim_engine.js';
import { Drone } from './drone.js';

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
	console.log("Started");
	const scene = new THREE.Scene();
	const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
	const renderer = new THREE.WebGLRenderer();
	renderer.shadowMap.enabled = true;
	renderer.setSize(window.innerWidth, window.innerHeight);
	const controls = new OrbitControls(camera, renderer.domElement);
	document.body.appendChild(renderer.domElement);

	const geometry = new THREE.BoxGeometry();
	const material = new THREE.MeshPhongMaterial({color: 0x00ff00});
	const cube = new THREE.Mesh(geometry, material);
	cube.castShadow = true;
	scene.add(cube);
	cube.translateZ(20);

	let drone = new Drone(scene, 1, 1, 1, 1);

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

