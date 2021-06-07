import * as THREE from './three.module.js';

export class Drone {
	constructor(scene, body_mass, per_arm_mass, max_static_thrust, max_inflow_vel) {
		this.body_mass = body_mass;
		this.per_arm_mass = per_arm_mass;
		this.max_static_thrust = max_static_thrust;
		this.max_inflow_vel = max_inflow_vel;

		const body_radius = 0.08;
		const body_height = 0.04;
		const arm_radius = 0.02;
		const arm_length = 0.1;

		const body_geo = new THREE.CylinderGeometry(body_radius, body_radius, body_height, 32, 1);
		const body_mat = new THREE.MeshPhongMaterial({color: 0xb3b3b3});
		const body = new THREE.Mesh(body_geo, body_mat);
		body.receiveShadow = true;
		body.castShadow = true;

		const arm_geo = new THREE.CylinderGeometry(arm_radius, arm_radius, arm_length, 16);
		const arm_mat = new THREE.MeshPhongMaterial({color: 0x6e6e6e});
		const arm_a = new THREE.Mesh(arm_geo, arm_mat);
		arm_a.position.x = (body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_a.position.z = (body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_a.rotateZ(Math.PI / 2);
		arm_a.rotateX(-Math.PI / 4);
		arm_a.receiveShadow = true;
		arm_a.castShadow = true;
		const arm_b = new THREE.Mesh(arm_geo, arm_mat);
		arm_b.position.x = -(body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_b.position.z = (body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_b.rotateZ(Math.PI / 2);
		arm_b.rotateX(Math.PI / 4);
		arm_b.receiveShadow = true;
		arm_b.castShadow = true;

		const arm_c = new THREE.Mesh(arm_geo, arm_mat);
		arm_c.position.x = -(body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_c.position.z = -(body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_c.rotateZ(Math.PI / 2);
		arm_c.rotateX(-Math.PI / 4);
		arm_c.receiveShadow = true;
		arm_c.castShadow = true;

		const arm_d = new THREE.Mesh(arm_geo, arm_mat);
		arm_d.position.x = (body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_d.position.z = -(body_radius + arm_length / 2) * Math.sqrt(2) / 2;
		arm_d.rotateZ(Math.PI / 2);
		arm_d.rotateX(Math.PI / 4);
		arm_d.receiveShadow = true;
		arm_d.castShadow = true;


		const group = new THREE.Group();
		group.add(body);
		group.add(arm_a);
		group.add(arm_b);
		group.add(arm_c);
		group.add(arm_d);

		this.mesh = group;

		scene.add(this.mesh);
	}
}
