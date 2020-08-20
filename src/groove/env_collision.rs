use crate::utils_rust::yaml_utils::{RobotCollisionSpecFileParser};
use nalgebra::{Vector3, Isometry3, Point3};
use nalgebra::geometry::{Translation3, UnitQuaternion, Quaternion};
use ncollide3d::pipeline::{*};
use ncollide3d::shape::{*};
use ncollide3d::query::{*};

#[derive(Clone, Debug)]
pub struct CollisionObjectData {
    pub name: String,
    pub is_link: bool,
    pub arm_idx: i32,
}

impl CollisionObjectData {
    pub fn new(name: String, is_link: bool, arm_idx: i32) -> CollisionObjectData {
        Self {
            name: name,
            is_link: is_link,
            arm_idx: arm_idx,
        }
    }
}

pub struct RelaxedIKEnvCollision {
    pub world: CollisionWorld<f64, CollisionObjectData>,
    pub link_handles: Vec<Vec<CollisionObjectSlabHandle>>,
    pub link_radius: f64,
    pub active_pairs: Vec<Vec<(CollisionObjectSlabHandle, CollisionObjectSlabHandle)>>,
    pub nearest_obstacle: Vec<Option<CollisionObjectSlabHandle>>,
}

impl RelaxedIKEnvCollision {
    pub fn init_collision_world (
        env_collision_file: RobotCollisionSpecFileParser,
        frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) -> Self {
        let link_radius = env_collision_file.robot_link_radius;
        let plane_obstacles = env_collision_file.cuboids;
        let sphere_obstacles = env_collision_file.spheres;

        // let mut robot_groups = CollisionGroups::new();
        // robot_groups.set_membership(&[1]);
        // robot_groups.set_whitelist(&[2]);

        // The links are part of group 1 and can only interact with obstacles
        let mut link_groups = CollisionGroups::new();
        link_groups.set_membership(&[1]);
        link_groups.set_blacklist(&[1]);
        link_groups.set_whitelist(&[2]);

        // All the other objects are part of the group 2 and interact only with the links
        let mut others_groups = CollisionGroups::new();
        others_groups.set_membership(&[2]);
        others_groups.set_blacklist(&[2]);
        others_groups.set_whitelist(&[1]);

        let proximity_query = GeometricQueryType::Proximity(1.0 * link_radius);

        let mut world = CollisionWorld::new(0.0);
        let mut link_handles: Vec<Vec<CollisionObjectSlabHandle>> = Vec::new();
        let mut active_pairs: Vec<Vec<(CollisionObjectSlabHandle, CollisionObjectSlabHandle)>> = Vec::new();
        let mut nearest_obstacle: Vec<Option<CollisionObjectSlabHandle>> = Vec::new();
        for arm_idx in 0..frames.len() {
            let mut handles: Vec<CollisionObjectSlabHandle> = Vec::new();
            let pair: Vec<(CollisionObjectSlabHandle, CollisionObjectSlabHandle)> = Vec::new();
            let last_elem = frames[arm_idx].0.len() - 1;
            for i in 0..last_elem {
                let start_pt = Point3::from(frames[arm_idx].0[i]);
                let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
                let segment = ShapeHandle::new(Segment::new(start_pt, end_pt));
                let segment_pos = nalgebra::one();
                let link_data = CollisionObjectData::new(format!("Link {}", i), true, arm_idx as i32);
                let handle = world.add(segment_pos, segment, link_groups, proximity_query, link_data);
                handles.push(handle.0);
            }
            link_handles.push(handles);
            active_pairs.push(pair);
            nearest_obstacle.push(None);
        }

        // let mut planes = Vec::new();
        // let mut planes_pos = Vec::new();
        for i in 0..plane_obstacles.len() {
            let plane_obs = &plane_obstacles[i];
            let half_extents = Vector3::new(plane_obs.x_halflength, plane_obs.y_halflength, plane_obs.z_halflength);
            let plane = ShapeHandle::new(Cuboid::new(half_extents));
            let plane_ts = Translation3::new(plane_obs.tx, plane_obs.ty, plane_obs.tz);
            let plane_rot = UnitQuaternion::from_euler_angles(plane_obs.rx, plane_obs.ry, plane_obs.rz);
            let plane_pos = Isometry3::from_parts(plane_ts, plane_rot);
            // planes.push(plane);
            // planes_pos.push(plane_pos);
            let plane_data = CollisionObjectData::new(format!("Plane {}", i + 1), false, -1);
            world.add(plane_pos, plane, others_groups, proximity_query, plane_data);
        }

        // let mut spheres = Vec::new();
        // let mut spheres_pos = Vec::new();
        for i in 0..sphere_obstacles.len() {
            let sphere_obs = &sphere_obstacles[i];
            let sphere = ShapeHandle::new(Ball::new(sphere_obs.radius));
            let sphere_ts = Translation3::new(sphere_obs.tx, sphere_obs.ty, sphere_obs.tz);
            let sphere_rot = UnitQuaternion::identity();
            let sphere_pos = Isometry3::from_parts(sphere_ts, sphere_rot);
            // spheres.push(sphere);
            // spheres_pos.push(sphere_pos);
            let sphere_data = CollisionObjectData::new(format!("Sphere {}", i + 1), false, -1);
            world.add(sphere_pos, sphere, others_groups, proximity_query, sphere_data);
        }

        // Register our handlers.
        // world.register_proximity_handler("ProximityMessage", ProximityMessage);
        
        return Self{world, link_handles, link_radius, active_pairs, nearest_obstacle};
    }

    pub fn update_collision_world(
        &mut self,
        frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) {
        for arm_idx in 0..frames.len() {
            let last_elem = frames[arm_idx].0.len() - 1;
            for i in 0..last_elem {
                let start_pt = Point3::from(frames[arm_idx].0[i]);
                let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
                let segment = ShapeHandle::new(Segment::new(start_pt, end_pt));
                let co = self.world.objects.get_mut(self.link_handles[arm_idx][i]).unwrap();
                co.set_shape(segment);
            }
        }
    }
}
