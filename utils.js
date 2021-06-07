import * as THREE from './three.module.js';
import {OrbitControls} from './orbit-controls.js';

export class Ground {
	constructor(scene, y) {
		const geo = new THREE.BoxGeometry(100, 2, 100);
		const mat = new THREE.MeshPhongMaterial({color: 0x00ff0f});
		this.mesh = new THREE.Mesh(geo, mat);
		this.mesh.translateY(y-1);

		scene.add(this.mesh);
	}
}

export function setup() {
	console.log("Started");
	const scene = new THREE.Scene();
	const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
	const renderer = new THREE.WebGLRenderer();
	renderer.shadowMap.enabled = true;
	renderer.setSize(window.innerWidth, window.innerHeight);
	const controls = new OrbitControls(camera, renderer.domElement);
	document.body.appendChild(renderer.domElement);

	return {
		scene: scene,
		camera: camera,
		renderer: renderer,
		controls: controls
	}
}
