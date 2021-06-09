use std::f32::consts::PI;
use std::f32::consts::SQRT_2;

use rapier3d::na::ArrayStorage;
use rapier3d::na::Const;
use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

enum VehicleType {
    Drone,
}

#[wasm_bindgen]
pub struct World {
    gravity: rapier3d::na::base::Matrix<f32, Const<3>, Const<1>, ArrayStorage<f32, 3, 1>>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joints: JointSet,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
    vehicle_handle: Option<RigidBodyHandle>,
    vehicle_type: Option<VehicleType>,
    pub controls_active: i32,
}

#[wasm_bindgen]
impl World {
    pub fn new() -> World {
        let rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        /* Create the ground. */
        let mut collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
        collider.set_translation(vector![0.0, -2.5, 0.0]);
        collider_set.insert(collider);

        World {
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            joints: JointSet::new(),
            bodies: rigid_body_set,
            colliders: collider_set,
            ccd_solver: CCDSolver::new(),
            physics_hooks: (),
            event_handler: (),
            vehicle_handle: None,
            vehicle_type: None,
            controls_active: 0,
        }
    }

    pub fn build_ground(&mut self, y: f32) {
        let collider = ColliderBuilder::cuboid(50.0, 1.0, 50.0)
            .translation(vector![0.0, y - 1.0, 0.0])
            .build();
        self.colliders.insert(collider);
    }

    pub fn build_vehicle(&mut self, vehicle: &str) -> Box<[f32]> {
        let handle = match vehicle {
            "drone" => self.build_drone(),
            _ => self.build_drone(),
        };
        self.vehicle_handle = Some(handle);
        return Box::new(self.bodies[handle].raw_isometry());
    }

    fn build_drone(&mut self) -> RigidBodyHandle {
        self.vehicle_type = Some(VehicleType::Drone);

        let body_radius = 0.08_f32;
        let body_height = 0.04_f32;
        let arm_radius = 0.02_f32;
        let arm_length = 0.1_f32;

        let rigid_body = RigidBodyBuilder::new_dynamic().build();
        let body_collider = ColliderBuilder::cylinder(body_height / 2f32, body_radius).build();

        let arm_builder = || ColliderBuilder::cylinder(arm_length / 2f32, arm_radius).build();
        let mut arm_a = arm_builder();
        arm_a.set_translation(vector![
            (body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            (body_radius + arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_a.set_rotation(vector![PI / 2f32, 0.0, -PI / 4.0]);

        let mut arm_b = arm_builder();
        arm_b.set_translation(vector![
            -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            (body_radius + arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_b.set_rotation(vector![PI / 2.0, 0.0, PI / 4.0]);

        let mut arm_c = arm_builder();
        arm_c.set_translation(vector![
            -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_c.set_rotation(vector![PI / 2.0, 0.0, -PI / 4.0]);

        let mut arm_d = arm_builder();
        arm_d.set_translation(vector![
            (body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_d.set_rotation(vector![PI / 2.0, 0.0, PI / 4.0]);

        let handle = self.bodies.insert(rigid_body);
        self.colliders
            .insert_with_parent(body_collider, handle, &mut self.bodies);
        self.colliders
            .insert_with_parent(arm_a, handle, &mut self.bodies);
        self.colliders
            .insert_with_parent(arm_b, handle, &mut self.bodies);
        self.colliders
            .insert_with_parent(arm_c, handle, &mut self.bodies);
        self.colliders
            .insert_with_parent(arm_d, handle, &mut self.bodies);

        handle
    }

    pub fn update_controls(&mut self, data: i32) {
        let body_radius = 0.08_f32;
        let body_height = 0.04_f32;
        let arm_radius = 0.02_f32;
        let arm_length = 0.1_f32;
        let mag = 0.01;

        if let Some(vehicle_type) = &self.vehicle_type {
            let body = self.bodies.get_mut(self.vehicle_handle.unwrap()).unwrap();
            if data & 0b1 == 1 << 0 {
                // A
                let point = body.position() * point![
                    (body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
                    0.0,
                    (body_radius + arm_length / 2f32) * SQRT_2 / 2f32
                ];
                let force = body.position() * vector![0.0, mag, 0.0];
                body.apply_force_at_point(force, point, true);
            }
            if data & 0b10 == 1 << 1 {
                // B
                let point = body.position() * point![
                    -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
                    0.0,
                    (body_radius + arm_length / 2f32) * SQRT_2 / 2f32
                ];
                let force = body.position() * vector![0.0, mag, 0.0];
                body.apply_force_at_point(force, point, true);

            }
            if data & 0b100 == 1 << 2 {
                // C
                let point = body.position() * point![
                    -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
                    0.0,
                    -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32
                ];
                let force = body.position() * vector![0.0, mag, 0.0];
                body.apply_force_at_point(force, point, true);

            }
            if data & 0b1000 == 1 << 3 {
                // D
                let point = body.position() * point![
                    (body_radius + arm_length / 2f32) * SQRT_2 / 2f32,
                    0.0,
                    -(body_radius + arm_length / 2f32) * SQRT_2 / 2f32
                ];
                let force = body.position() * vector![0.0, mag, 0.0];
                body.apply_force_at_point(force, point, true);

            }
        }
    }

    pub fn step(&mut self) -> Box<[f32]> {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &mut self.ccd_solver,
            &self.physics_hooks,
            &self.event_handler,
        );

        Box::new(self.bodies[self.vehicle_handle.unwrap()].raw_isometry())
    }
}

pub trait RawIsometry {
    fn raw_isometry(&self) -> [f32; 7];
}

impl RawIsometry for RigidBody {
    fn raw_isometry(&self) -> [f32; 7] {
        let translation = self.translation();
        let quaternion = self.rotation();
        [
            translation.x,
            translation.y,
            translation.z,
            quaternion.i,
            quaternion.j,
            quaternion.k,
            quaternion.w,
        ]
    }
}
