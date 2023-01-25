use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};
use crate::spacetime::robot::Robot;
use crate::utils_rust::file_utils::{*};
use time::PreciseTime;
use std::ops::Deref;
use yaml_rust::{YamlLoader, Yaml};
use std::fs::File;
use std::io::prelude::*;

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
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub tolerances: Vec<Vector6<f64>>
}
impl RelaxedIKVars {
    pub fn from_settings(path_to_setting: &str) -> Self {
        let path_to_src = get_path_to_src();
        let mut file = File::open(path_to_setting).unwrap();
        let mut contents = String::new();
        let res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];

        let path_to_urdf = path_to_src + "configs/urdfs/" + settings["urdf"].as_str().unwrap();
        println!("RelaxedIK is using below URDF file: {}", path_to_urdf);
        let chain = k::Chain::<f64>::from_urdf_file(path_to_urdf.clone()).unwrap();

        let num_chains = settings["base_links"].as_vec().unwrap().len();

        let base_links_arr = settings["base_links"].as_vec().unwrap();
        let ee_links_arr = settings["ee_links"].as_vec().unwrap();
        let mut base_links = Vec::new();
        let mut ee_links = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..base_links_arr.len() {
            base_links.push(base_links_arr[i].as_str().unwrap().to_string());
            ee_links.push(ee_links_arr[i].as_str().unwrap().to_string());
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let robot = Robot::from_urdf(path_to_urdf.clone(), &base_links, &ee_links);

        let mut starting_config = Vec::new();
        if settings["starting_config"].is_badvalue() {
            println!("No starting config provided, using all zeros");
            for i in 0..robot.num_dofs {
                starting_config.push(0.0);
            }
        } else {
            let starting_config_arr = settings["starting_config"].as_vec().unwrap();
            for i in 0..starting_config_arr.len() {
                starting_config.push(starting_config_arr[i].as_f64().unwrap());
            }
        }

        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&starting_config);
        for i in 0..pose.len() {
            goal_positions.push(pose[i].0);
            goal_quats.push(pose[i].1);
        }

        RelaxedIKVars{robot, init_state: starting_config.clone(), xopt: starting_config.clone(),
            prev_state: starting_config.clone(), prev_state2: starting_config.clone(), prev_state3: starting_config.clone(),
            goal_positions, goal_quats, tolerances}
    }
    
    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn reset(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = xopt.clone();
        self.prev_state2 = xopt.clone();
        self.prev_state = xopt.clone();
        self.xopt = xopt.clone();
    }

}
