use nalgebra::{UnitQuaternion, Vector3, Quaternion, Point3};
use crate::utils_rust::yaml_utils::{*};
use crate::spacetime::robot::Robot;
use crate::groove::collision_nn::CollisionNN;
use crate::utils_rust::sampler::ThreadRobotSampler;
use crate::utils_rust::file_utils::{*};
use crate::groove::env_collision::{*};
use ncollide3d::pipeline::{*};
use ncollide3d::query::{*};
use ncollide3d::shape::{*};
use time::PreciseTime;
use std::ops::Deref;

#[derive(Clone, Debug)]
pub struct Vars {
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>
}
impl Vars {
    pub fn new(init_state: Vec<f64>) -> Self {
        Vars{init_state: init_state.clone(), xopt: init_state.clone(), prev_state: init_state.clone(),
            prev_state2: init_state.clone(), prev_state3: init_state.clone()}
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }
}


pub struct RelaxedIKVars {
    pub robot: Robot,
    pub sampler: ThreadRobotSampler,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
    pub position_mode_relative: bool, // if false, will be absolute
    pub rotation_mode_relative: bool, // if false, will be absolute
    pub collision_nn: CollisionNN,
    pub env_collision: RelaxedIKEnvCollision,
}
impl RelaxedIKVars {
    pub fn from_yaml_path(fp: String, position_mode_relative: bool, rotation_mode_relative: bool) -> Self {
        let ifp = InfoFileParser::from_yaml_path(fp.clone());
        let mut robot = Robot::from_yaml_path(fp.clone());
        let num_chains = ifp.joint_names.len();
        let sampler = ThreadRobotSampler::new(robot.clone());

        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();

        let init_ee_positions = robot.get_ee_positions(ifp.starting_config.as_slice());
        let init_ee_quats = robot.get_ee_quats(ifp.starting_config.as_slice());

        for i in 0..num_chains {
            goal_positions.push(init_ee_positions[i]);
            goal_quats.push(init_ee_quats[i]);
        }

        let collision_nn_path = get_path_to_src()+ "relaxed_ik_core/config/collision_nn_rust/" + ifp.collision_nn_file.as_str() + ".yaml";
        let collision_nn = CollisionNN::from_yaml_path(collision_nn_path);

        let env_collision_path = get_path_to_src() + "env_collision_files/env_collision.yaml";
        let env_collision_file = EnvCollisionFileParser::from_yaml_path(env_collision_path);
        let frames = robot.get_frames_immutable(&ifp.starting_config.clone());
        let env_collision = RelaxedIKEnvCollision::init_collision_world(env_collision_file, &frames);

        RelaxedIKVars{robot, sampler, init_state: ifp.starting_config.clone(), xopt: ifp.starting_config.clone(),
            prev_state: ifp.starting_config.clone(), prev_state2: ifp.starting_config.clone(), prev_state3: ifp.starting_config.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative, collision_nn, 
            env_collision}
    }

    pub fn from_yaml_path_with_init(fp: String, init_state: Vec<f64>, position_mode_relative: bool, rotation_mode_relative: bool) -> Self {
        let ifp = InfoFileParser::from_yaml_path(fp.clone());
        let mut robot = Robot::from_yaml_path(fp.clone());
        let num_chains = robot.num_chains;
        let sampler = ThreadRobotSampler::new(robot.clone());


        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();

        let init_ee_positions = robot.get_ee_positions(init_state.as_slice());
        let init_ee_quats = robot.get_ee_quats(init_state.as_slice());

        for i in 0..num_chains {
            if position_mode_relative {
                goal_positions.push(nalgebra::Vector3::identity());
            } else {
                goal_positions.push(init_ee_positions[i]);
            }

            if rotation_mode_relative {
                goal_quats.push(nalgebra::UnitQuaternion::identity());
            } else {
                goal_quats.push(init_ee_quats[i]);
            }
        }

        let collision_nn_path = get_path_to_src()+ "relaxed_ik_core/config/collision_nn_rust/" + ifp.collision_nn_file.as_str() + ".yaml";
        let collision_nn = CollisionNN::from_yaml_path(collision_nn_path);

        let env_collision_path = get_path_to_src() + "env_collision_files/env_collision.yaml";
        let env_collision_file = EnvCollisionFileParser::from_yaml_path(env_collision_path);
        let frames = robot.get_frames_immutable(&ifp.starting_config.clone());
        let env_collision = RelaxedIKEnvCollision::init_collision_world(env_collision_file, &frames);

        RelaxedIKVars{robot, sampler, init_state: init_state.clone(), xopt: init_state.clone(),
            prev_state: init_state.clone(), prev_state2: init_state.clone(), prev_state3: init_state.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative, 
            collision_nn, env_collision}
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn update_collision_world(&mut self) -> bool {
        // let start = PreciseTime::now();
        let frames = self.robot.get_frames_immutable(&self.xopt);
        self.env_collision.update_links(&frames);
        for event in self.env_collision.world.proximity_events() {
            let c1 = self.env_collision.world.objects.get(event.collider1).unwrap();
            let c2 = self.env_collision.world.objects.get(event.collider2).unwrap();
            if event.new_status == Proximity::Intersecting {
                // println!("===== {:?} Intersecting of {:?} =====", c1.data().name, c2.data().name);
            } else if event.new_status == Proximity::WithinMargin {
                // println!("===== {:?} WithinMargin of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().is_link {
                    let arm_idx = c1.data().arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if !links.contains(&event.collider1) {
                            links.push(event.collider1);
                        }
                    } else {
                        let links: Vec<CollisionObjectSlabHandle> = vec![event.collider1];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider2, links);
                    }
                } else if c2.data().is_link {
                    let arm_idx = c2.data().arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if !links.contains(&event.collider2) {
                            links.push(event.collider2);
                        }
                    } else {
                        let links: Vec<CollisionObjectSlabHandle> = vec![event.collider2];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider1, links);
                    }
                }
            } else {
                // println!("===== {:?} Disjoint of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().is_link {
                    let arm_idx = c1.data().arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if links.contains(&event.collider1) {
                            let index = links.iter().position(|x| *x == event.collider1).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider2);
                        }
                    }
                } else if c2.data().is_link {
                    let arm_idx = c2.data().arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if links.contains(&event.collider2) {
                            let index = links.iter().position(|x| *x == event.collider2).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider1);
                        }
                    }
                } 
            }
            // self.print_active_pairs();
        }

        self.env_collision.world.update();

        for arm_idx in 0..frames.len() {
            let link_radius = self.env_collision.link_radius;
            let penalty_cutoff: f64 = link_radius * 1.5;
            let a = 0.01 * (penalty_cutoff.powi(20));
            // let mut sum_max: f64 = 0.0;
            let mut active_obstacles: Vec<Option<CollisionObjectSlabHandle>> = Vec::new();
            for key in self.env_collision.active_pairs[arm_idx].keys() {
                let obstacle = self.env_collision.world.objects.get(*key).unwrap();
                // println!("Obstacle: {:?}", obstacle.data());
                let mut sum: f64 = 0.0;
                let last_elem = frames[arm_idx].0.len() - 1;
                for j in 0..last_elem {
                    let start_pt = Point3::from(frames[arm_idx].0[j]);
                    let end_pt = Point3::from(frames[arm_idx].0[j + 1]);
                    let segment = Segment::new(start_pt, end_pt);
                    let segment_pos = nalgebra::one();
                    let dis = distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius;
                    // println!("VARS -> {:?}, Link{}, Distance: {:?}", obstacle.data(), j, dis);
                    if dis > 0.0 {
                        sum += a / (dis + link_radius).powi(20);
                    } else {
                        return true;
                    }
                }
                // println!("VARS -> {:?}, Sum: {:?}", obstacle.data().name, sum);
                if sum > 0.02 {
                    active_obstacles.push(Some(*key));
                }
            }
            // println!("Number of active obstacles: {}", active_obstacles.len());
            self.env_collision.active_obstacles[arm_idx] = active_obstacles;
        }
        
        // let end = PreciseTime::now();
        // println!("Update collision world takes {}", start.to(end));
        return false;
    }

    pub fn print_active_pairs(&self) {
        let frames = self.robot.get_frames_immutable(&self.xopt);
        for i in 0..frames.len() {
            for (key, values) in self.env_collision.active_pairs[i].iter() {
                let collider = self.env_collision.world.objects.get(*key).unwrap();
                for v in values {
                    let link = self.env_collision.world.objects.get(*v).unwrap();
                    println!("Arm {}, Active pair {:?} and {:?}", i, collider.data().name, link.data().name);
                }
            }
        }
    }
}
