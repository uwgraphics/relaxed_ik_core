use nalgebra::{UnitQuaternion, Vector3, Quaternion};
use crate::utils_rust::yaml_utils::{get_yaml_obj, InfoFileParser, RobotCollisionSpecFileParser};
use crate::spacetime::robot::Robot;
use crate::groove::collision_nn::CollisionNN;
use crate::utils_rust::sampler::ThreadRobotSampler;
use crate::utils_rust::file_utils::{*};
use ncollide3d::pipeline::{*};
use crate::groove::env_collision;

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
    pub env_collision: CollisionWorld<f64, env_collision::CollisionObjectData>,
    pub link_handles: Vec<CollisionObjectSlabHandle>,
    pub link_radius: f64,
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

        let env_collision_path = get_path_to_src() + "relaxed_ik_core/config/env_collision_files/" + ifp.env_collision_file_name.as_str();
        let env_collision_file = RobotCollisionSpecFileParser::from_yaml_path(env_collision_path);
        let link_radius = env_collision_file.robot_link_radius;
        let frames = robot.get_frames_immutable(&ifp.starting_config.clone());
        let (env_collision, link_handles) = env_collision::init_collision_world(env_collision_file, &frames);
        
        RelaxedIKVars{robot, sampler, init_state: ifp.starting_config.clone(), xopt: ifp.starting_config.clone(),
            prev_state: ifp.starting_config.clone(), prev_state2: ifp.starting_config.clone(), prev_state3: ifp.starting_config.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative, collision_nn, 
            env_collision, link_handles, link_radius}
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
        
        let env_collision_path = get_path_to_src() + "relaxed_ik_core/config/env_collision_files/" + "env_collision_ur5" + ".yaml";
        let env_collision_file = RobotCollisionSpecFileParser::from_yaml_path(env_collision_path);
        let frames = robot.get_frames_immutable(&ifp.starting_config.clone());
        let (env_collision, link_handles) = env_collision::init_collision_world(env_collision_file, &frames);
        let link_radius = env_collision_file.robot_link_radius;

        RelaxedIKVars{robot, sampler, init_state: init_state.clone(), xopt: init_state.clone(),
            prev_state: init_state.clone(), prev_state2: init_state.clone(), prev_state3: init_state.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative, 
            collision_nn, env_collision, link_handles, link_radius}
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }
}
