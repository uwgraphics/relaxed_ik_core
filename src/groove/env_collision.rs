use crate::utils_rust::yaml_utils::{RobotCollisionSpecFileParser};
use nalgebra::{Vector3, Isometry3, Point3};
use nalgebra::geometry::{Translation3, UnitQuaternion, Quaternion};
use ncollide3d::pipeline::{*};
use ncollide3d::shape::{*};
use ncollide3d::query::{*};

#[derive(Clone)]
pub struct CollisionObjectData {
    pub is_link: bool,
    pub arm_idx: i32,
}

impl CollisionObjectData {
    pub fn new(is_link: bool, arm_idx: i32) -> CollisionObjectData {
        Self {
            is_link: is_link,
            arm_idx: arm_idx,
        }
    }
}

pub struct RelaxedIKEnvCollision {
    pub world: CollisionWorld<f64, CollisionObjectData>,
    pub link_handles: Vec<CollisionObjectSlabHandle>,
    pub link_radius: f64,
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

        
        let plane_data = CollisionObjectData::new(false, -1);
        let sphere_data = CollisionObjectData::new(false, -1);

        let proximity_query = GeometricQueryType::Proximity(link_radius);

        let mut world = CollisionWorld::new(0.0);
        let mut link_handles: Vec<CollisionObjectSlabHandle> = Vec::new();
        for arm_idx in 0..frames.len() {
            let last_elem = frames[arm_idx].0.len() - 1;
            for i in 0..last_elem {
                let start_pt = Point3::from(frames[arm_idx].0[i]);
                let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
                let segment = ShapeHandle::new(Segment::new(start_pt, end_pt));
                let segment_pos = nalgebra::one();
                let link_data = CollisionObjectData::new(true, arm_idx as i32);
                let handle = world.add(segment_pos, segment, link_groups, proximity_query, link_data);
                link_handles.push(handle.0);
            }
        }
        // let mut planes = Vec::new();
        // let mut planes_pos = Vec::new();
        for plane_obs in plane_obstacles {
            let half_extents = Vector3::new(plane_obs.x_halflength, plane_obs.y_halflength, plane_obs.z_halflength);
            let plane = ShapeHandle::new(Cuboid::new(half_extents));
            let plane_ts = Translation3::new(plane_obs.tx, plane_obs.ty, plane_obs.tz);
            let plane_rot = UnitQuaternion::from_euler_angles(plane_obs.rx, plane_obs.ry, plane_obs.rz);
            let plane_pos = Isometry3::from_parts(plane_ts, plane_rot);
            // planes.push(plane);
            // planes_pos.push(plane_pos);
            world.add(plane_pos, plane, others_groups, proximity_query, plane_data.clone());
        }

        // let mut spheres = Vec::new();
        // let mut spheres_pos = Vec::new();
        for sphere_obs in sphere_obstacles {
            let sphere = ShapeHandle::new(Ball::new(sphere_obs.radius));
            let sphere_ts = Translation3::new(sphere_obs.tx, sphere_obs.ty, sphere_obs.tz);
            let sphere_rot = UnitQuaternion::identity();
            let sphere_pos = Isometry3::from_parts(sphere_ts, sphere_rot);
            // spheres.push(sphere);
            // spheres_pos.push(sphere_pos);
            // let sphere_handle = 
            world.add(sphere_pos, sphere, others_groups, proximity_query, sphere_data.clone());
        }

        // Register our handlers.
        // world.register_proximity_handler("ProximityMessage", ProximityMessage);

        return Self{world, link_handles, link_radius};
    }

    pub fn update_collision_world (
        &mut self,
        frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) {
        for arm_idx in 0..frames.len() {
            let length = frames[arm_idx].0.len();
            for i in 0..length - 1 {
                let start_pt = Point3::from(frames[arm_idx].0[i]);
                let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
                let segment = ShapeHandle::new(Segment::new(start_pt, end_pt));
                let co = self.world.objects.get_mut(self.link_handles[arm_idx * length + i]).unwrap();
                co.set_shape(segment);
            }
        }   
    }

    pub fn handle_proximity_event<'a> (
        &'a self,
        event: &ProximityEvent<CollisionObjectSlabHandle>,
    ) -> Option<(usize, &'a CollisionObject<f64, CollisionObjectData>)> {
        if event.new_status == Proximity::WithinMargin {
            println!("WithinMargin");
            let c1 = self.world.objects.get(event.collider1).unwrap();
            let c2 = self.world.objects.get(event.collider2).unwrap();
            if c1.data().is_link {
                return Some((c1.data().arm_idx as usize, c2));
            } else if c2.data().is_link {
                return Some((c2.data().arm_idx as usize, c1));
            } else {
                return None;
            }
        } else if event.new_status == Proximity::Intersecting {
            println!("Intersecting");
            return None;
        } else {
            println!("Disjoint");
            return None;
        }
    }
}

pub fn calculate_dis_sum (
    obstacle_pos: &Isometry3<f64>, 
    obstacle_shape: &dyn Shape<f64>,
    frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    arm_idx: usize,
    link_radius: f64, 
) -> f64 {
    println!("Obstacle pos: {:?}", obstacle_pos);
    let penalty_cutoff: f64 = link_radius / 2.0;
    let a = 0.005 * (penalty_cutoff.powi(10));
    let mut sum: f64 = 0.0;
    let last_elem = frames[arm_idx].0.len() - 1;
    for i in 0..last_elem {
        let start_pt = Point3::from(frames[arm_idx].0[i]);
        let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
        let segment = Segment::new(start_pt, end_pt);
        let segment_pos = nalgebra::one();
        let dis = distance(obstacle_pos, obstacle_shape, &segment_pos, &segment) - link_radius;
        println!("Link: {}, Distance: {:?}", i, dis);
        sum += a / dis.powi(10); 
    }
    println!("Sum: {}", sum);
    sum
}