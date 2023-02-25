
use crate::groove::vars::{RelaxedIKVars, VarsConstructorData};
use crate::groove::groove::{OptimizationEngineOpen};
use crate::groove::objective_master::ObjectiveMaster;
use crate::utils_rust::transformations::{*};
use wasm_bindgen::prelude::*;
use js_sys::Array;
extern crate serde_json;
extern crate console_error_panic_hook;
use serde::{Serialize, Deserialize};
use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};

#[wasm_bindgen]
pub struct RelaxedIK {
    pub(crate) vars: RelaxedIKVars,
    pub(crate) om: ObjectiveMaster,
    pub(crate) groove: OptimizationEngineOpen
    // pub groove_nlopt: OptimizationEngineNLopt
}

#[wasm_bindgen]
impl RelaxedIK {
    #[wasm_bindgen(constructor)]
    pub fn new( configs:  &JsValue, urdf: String) -> Self {
        console_error_panic_hook::set_once();

        let cfg: VarsConstructorData = configs.into_serde().unwrap();

        let vars = RelaxedIKVars::from_jsvalue(cfg, &urdf);

        let mut om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);
        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());
        Self{vars, om, groove}
    }

    // pub fn recover_vars(&mut self, init_state:  &JsValue) {
    //     let mut starting_config: Vec<f64> = init_state.into_serde().unwrap();
    //     // if init_state is empty, move robot to previous init state
    //     if (starting_config.len() == 0) {
    //         starting_config = self.vars.init_state.clone();
    //     }
    //     self.vars.goal_positions.clear();
    //     self.vars.goal_quats.clear();
    //     self.vars.init_ee_positions = self.vars.robot.get_ee_positions(starting_config.as_slice());
    //     self.vars.init_ee_quats = self.vars.robot.get_ee_quats(starting_config.as_slice());

    //     for i in 0..self.vars.robot.joint_names.len() {
    //         self.vars.goal_positions.push(self.vars.init_ee_positions[i]);
    //         self.vars.goal_quats.push(self.vars.init_ee_quats[i]);
    //     }
    //     self.vars.xopt = starting_config.clone();
    //     self.vars.prev_state = starting_config.clone();
    //     self.vars.prev_state2 = starting_config.clone();
    //     self.vars.prev_state3 = starting_config.clone();
    // }

    pub fn solve(&mut self, pos_goal:  &JsValue,  quat_goal:  &JsValue) -> Array{
        let pos_vec: Vec<f64> = pos_goal.into_serde().unwrap();
        let quat_vec: Vec<f64> = quat_goal.into_serde().unwrap();

        let mut pos_goals: Vec<Vector3<f64>> = Vec::new();
        let mut quat_goals: Vec<UnitQuaternion<f64>> = Vec::new();

        for i in 0..self.vars.robot.num_chains {
            let pos = Vector3::new(pos_vec[i*3], pos_vec[i*3+1], pos_vec[i*3+2]);
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(quat_vec[i*4], quat_vec[i*4+1], quat_vec[i*4+2], quat_vec[i*4+3]));
            pos_goals.push(pos);
            quat_goals.push(quat);
        }
        let res = self.solve_helper(pos_goals, quat_goals);
        res.into_iter().map(JsValue::from).collect()
    }

}

impl RelaxedIK {
    pub fn solve_helper(&mut self, pos_goals: Vec<Vector3<f64>>, quat_goals: Vec<UnitQuaternion<f64>>) -> Vec<f64> {
        let mut out_x = self.vars.xopt.clone();

        let position_mode_relative = true;
        let rotation_mode_relative = true;
        for i in 0..self.vars.robot.num_chains {
            if position_mode_relative {
                    self.vars.goal_positions[i] = self.vars.init_ee_positions[i] + pos_goals[i];
            } else {
                    self.vars.goal_positions[i] = pos_goals[i].clone();
            }
            if rotation_mode_relative {
                self.vars.goal_quats[i] = quat_goals[i] * self.vars.init_ee_quats[i];
            } else {
                self.vars.goal_quats[i] = quat_goals[i].clone();
            }
        }

        self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
        self.vars.update(out_x.clone());

        out_x
    }
}