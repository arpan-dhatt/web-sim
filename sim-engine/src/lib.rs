mod vehicle;

use rapier3d::na::ArrayStorage;
use rapier3d::na::Const;
use rapier3d::prelude::*;
use vehicle::Vehicle;
use wasm_bindgen::prelude::*;

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
    vehicle_handle: Option<Box<dyn Vehicle>>,
    vehicle_type: Option<vehicle::VehicleType>,
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
        }
    }

    pub fn build_ground(&mut self, y: f32) {
        let collider = ColliderBuilder::cuboid(50.0, 1.0, 50.0)
            .translation(vector![0.0, y - 1.0, 0.0])
            .build();
        self.colliders.insert(collider);
    }

    pub fn build_vehicle(&mut self, vehicle_name: &str) -> Box<[f32]> {
        let mut vehicle = match vehicle_name {
            "Drone" => vehicle::create_vehicle(vehicle::VehicleType::Drone, false, &[0.0]),
            _ => vehicle::create_vehicle(vehicle::VehicleType::Drone, false, &[0.0]),
        };
        vehicle.build(&mut self.bodies, &mut self.colliders);
        self.vehicle_handle = Some(vehicle);
        return Box::new(self.vehicle_handle.as_ref().unwrap().transform(&self.bodies));
    }

    pub fn update_controls(&mut self, data: &[f32]) {
        self.vehicle_handle.as_mut().unwrap().controls(data);
    }

    pub fn step(&mut self) -> Box<[f32]> {
        self.vehicle_handle.as_mut().unwrap().execute_forces(&mut self.bodies);
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
        return Box::new(self.vehicle_handle.as_ref().unwrap().transform(&self.bodies));
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
