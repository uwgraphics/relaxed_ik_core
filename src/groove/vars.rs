use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};
use crate::spacetime::robot::Robot;
use crate::utils_rust::file_utils::{*};
use time::PreciseTime;
use std::ops::Deref;
use yaml_rust::{YamlLoader, Yaml};
use std::fs::File;
use std::io::prelude::*;

use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
pub struct VarsConstructorData {
    // pub urdf: String,
    pub link_radius:f64,
    pub base_links: Vec<String>,
    pub ee_links: Vec<String>,
    starting_config: Vec<f64>
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
    pub tolerances: Vec<Vector6<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>
}
impl RelaxedIKVars {
    pub fn from_local_settings(path_to_setting: &str) -> Self {
        let path_to_src = get_path_to_src();
        let mut file = File::open(path_to_setting).unwrap();
        let mut contents = String::new();
        let res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];

        let path_to_urdf = path_to_src + "configs/urdfs/" + settings["urdf"].as_str().unwrap();
        println!("RelaxedIK is using below URDF file: {}", path_to_urdf);
        let chain = k::Chain::<f64>::from_urdf_file(path_to_urdf.clone()).unwrap();

        let base_links_arr = settings["base_links"].as_vec().unwrap();
        let ee_links_arr = settings["ee_links"].as_vec().unwrap();
        let num_chains = base_links_arr.len();

        let mut base_links = Vec::new();
        let mut ee_links = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..num_chains {
            base_links.push(base_links_arr[i].as_str().unwrap().to_string());
            ee_links.push(ee_links_arr[i].as_str().unwrap().to_string());
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let urdf = &std::fs::read_to_string(path_to_urdf).unwrap();
        let robot = Robot::from_urdf(urdf, &base_links, &ee_links);

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

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        RelaxedIKVars{robot, init_state: starting_config.clone(), xopt: starting_config.clone(),
            prev_state: starting_config.clone(), prev_state2: starting_config.clone(), prev_state3: starting_config.clone(),
            goal_positions: init_ee_positions.clone(), goal_quats: init_ee_quats.clone(), tolerances, init_ee_positions, init_ee_quats}
    }
    
    // for webassembly
    pub fn from_jsvalue( configs: VarsConstructorData, urdf: &str) -> Self  {

        let num_chains = configs.base_links.len();

        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..num_chains {
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let robot = Robot::from_urdf(urdf, &configs.base_links, &configs.ee_links);

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&configs.starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        RelaxedIKVars{robot, init_state: configs.starting_config.clone(), xopt: configs.starting_config.clone(),
            prev_state: configs.starting_config.clone(), prev_state2: configs.starting_config.clone(), prev_state3: configs.starting_config.clone(),
            goal_positions: init_ee_positions.clone(), goal_quats: init_ee_quats.clone(), tolerances, init_ee_positions, init_ee_quats}

    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn reset(&mut self, init_state: Vec<f64>) {
        self.prev_state3 = init_state.clone();
        self.prev_state2 = init_state.clone();
        self.prev_state = init_state.clone();
        self.xopt = init_state.clone();
        self.init_state = init_state.clone();

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = self.robot.get_ee_pos_and_quat_immutable(&init_state);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        self.init_ee_positions = init_ee_positions.clone();
        self.init_ee_quats = init_ee_quats.clone();
    }

}
