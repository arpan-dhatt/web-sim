use std::f32::consts::{PI, SQRT_2};

use rapier3d::prelude::*;

use crate::RawIsometry;

pub enum VehicleType {
    Drone,
}

pub fn create_vehicle(vehicle_type: VehicleType, use_data: bool, data: &[f32]) -> Box<dyn Vehicle> {
    Box::new(match vehicle_type {
        VehicleType::Drone => Drone::parameterize(use_data, data),
    })
}

pub trait Vehicle {
    fn build(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> RigidBodyHandle;
    fn controls(&mut self, data: &[f32]);
    fn execute_forces(&mut self, bodies: &mut RigidBodySet);
    fn transform(&self, bodies: &RigidBodySet) -> [f32; 7];
    fn sensor_data(&self, bodies: &RigidBodySet, integration_parameters: &IntegrationParameters, gravity: &Vector<Real>) -> [f32; 6];
}

pub struct Drone {
    body_radius: f32,
    body_height: f32,
    arm_radius: f32,
    arm_length: f32,
    max_prop_thrust: f32,
    max_prop_torque: f32,
    max_inflow_vel: f32,
    handle: Option<RigidBodyHandle>,
    escs: [f32; 4],
    linvel: Vector<Real>
}

enum Propeller {
    A,
    B,
    C,
    D,
}

impl Drone {
    fn parameterize(use_data: bool, data: &[f32]) -> Self {
        if use_data && data.len() >= 7 {
            return Drone {
                body_radius: data[0],
                body_height: data[1],
                arm_radius: data[2],
                arm_length: data[3],
                max_prop_thrust: data[4],
                max_prop_torque: data[5],
                max_inflow_vel: data[6],
                handle: None,
                escs: [0.0; 4],
                linvel: vector![0.0, 0.0, 0.0]
            };
        } else {
            return Drone::default();
        }
    }

    fn propeller_force(&self, body: &mut RigidBody, prop: Propeller, esc: f32) {
        let offset = (self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32;
        let point = point![
            match &prop {
                Propeller::A => offset,
                Propeller::B => -offset,
                Propeller::C => -offset,
                Propeller::D => offset,
            },
            0.0,
            match &prop {
                Propeller::A => offset,
                Propeller::B => offset,
                Propeller::C => -offset,
                Propeller::D => -offset,
            }
        ];
        let force = body.position() * vector![0.0, esc * self.max_prop_thrust, 0.0];
        body.apply_force_at_point(force, body.position() * point, true);
        let torque = vector![
            0.0,
            match &prop {
                Propeller::A | Propeller::C => self.max_prop_torque * esc,
                Propeller::B | Propeller::D => -self.max_prop_torque * esc,
            },
            0.0
        ];
        body.apply_torque(body.position() * torque, true);
    }
}

impl Default for Drone {
    fn default() -> Self {
        Drone {
            body_radius: 0.08,
            body_height: 0.04,
            arm_radius: 0.02,
            arm_length: 0.1,
            max_prop_thrust: 0.005,
            max_prop_torque: 0.0005,
            max_inflow_vel: 40.0,
            handle: None,
            escs: [0.0; 4],
            linvel: vector![0.0, 0.0, 0.0]
        }
    }
}

impl Vehicle for Drone {
    fn build(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> RigidBodyHandle {
        let mut rigid_body = RigidBodyBuilder::new_dynamic().build();
        rigid_body.set_linear_damping(0.95);
        rigid_body.set_angular_damping(0.95);
        let body_collider =
            ColliderBuilder::cylinder(self.body_height / 2f32, self.body_radius).build();

        let arm_builder =
            || ColliderBuilder::cylinder(self.arm_length / 2f32, self.arm_radius).build();
        let mut arm_a = arm_builder();
        arm_a.set_translation(vector![
            (self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            (self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_a.set_rotation(vector![PI / 2f32, 0.0, -PI / 4.0]);

        let mut arm_b = arm_builder();
        arm_b.set_translation(vector![
            -(self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            (self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_b.set_rotation(vector![PI / 2.0, 0.0, PI / 4.0]);

        let mut arm_c = arm_builder();
        arm_c.set_translation(vector![
            -(self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            -(self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_c.set_rotation(vector![PI / 2.0, 0.0, -PI / 4.0]);

        let mut arm_d = arm_builder();
        arm_d.set_translation(vector![
            (self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32,
            0.0,
            -(self.body_radius + self.arm_length / 2f32) * SQRT_2 / 2f32
        ]);
        arm_d.set_rotation(vector![PI / 2.0, 0.0, PI / 4.0]);

        let handle = bodies.insert(rigid_body);
        colliders.insert_with_parent(body_collider, handle, bodies);
        colliders.insert_with_parent(arm_a, handle, bodies);
        colliders.insert_with_parent(arm_b, handle, bodies);
        colliders.insert_with_parent(arm_c, handle, bodies);
        colliders.insert_with_parent(arm_d, handle, bodies);

        self.handle = Some(handle);

        handle
    }

    fn controls(&mut self, data: &[f32]) {
        if data.len() >= 4 {
            self.escs[0] = data[0];
            self.escs[1] = data[1];
            self.escs[2] = data[2];
            self.escs[3] = data[3];
        }
    }

    fn execute_forces(&mut self, bodies: &mut RigidBodySet) {
        if let Some(handle) = self.handle {
            if let Some(body) = bodies.get_mut(handle) {
                self.propeller_force(body, Propeller::A, self.escs[0]);
                self.propeller_force(body, Propeller::B, self.escs[1]);
                self.propeller_force(body, Propeller::C, self.escs[2]);
                self.propeller_force(body, Propeller::D, self.escs[3]);
            }
        }
    }

    fn transform(&self, bodies: &RigidBodySet) -> [f32; 7] {
        if let Some(handle) = self.handle {
            if let Some(body) = bodies.get(handle) {
                return body.raw_isometry();
            }
        }
        return [0.0; 7];
    }

    fn sensor_data(&self, bodies: &RigidBodySet, integration_parameters: &IntegrationParameters, gravity: &Vector<Real>) -> [f32; 6] {
        if let Some(handle) = self.handle {
            if let Some(body) = bodies.get(handle) {
                let world_acc = (body.linvel() - self.linvel) / integration_parameters.dt - gravity;
                let local_acc = body.position() * world_acc;
                let mut out = [0.0; 6];
                out[0] = local_acc.x;
                out[1] = local_acc.y;
                out[2] = local_acc.z;
                return out;
            }
        }
        return [0.0; 6];
    }
}
