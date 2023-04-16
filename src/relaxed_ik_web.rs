
use crate::groove::vars::{RelaxedIKVars, VarsConstructorData};
use crate::groove::groove::{OptimizationEngineOpen};
use crate::groove::objective_master::ObjectiveMaster;
use crate::utils_rust::transformations::{*};
use wasm_bindgen::prelude::*;
use js_sys::Array;
extern crate serde_json;
use web_sys;
extern crate console_error_panic_hook;
use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};

#[wasm_bindgen]
pub struct RelaxedIK {
    pub(crate) vars: RelaxedIKVars,
    pub(crate) om: ObjectiveMaster,
    pub(crate) groove: OptimizationEngineOpen
}

#[wasm_bindgen]
impl RelaxedIK {
    #[wasm_bindgen(constructor)]
    pub fn new( configs:  JsValue, urdf: String) -> Self {
        console_error_panic_hook::set_once();

        let cfg: VarsConstructorData = serde_wasm_bindgen::from_value(configs).unwrap();

        let vars = RelaxedIKVars::from_jsvalue(cfg, &urdf);

        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths);
        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());
        Self{vars, om, groove}
    }

    pub fn reset(&mut self, init_state:  JsValue) {
        let starting_config = if init_state.is_null() || init_state.is_undefined() {
            self.vars.init_state.clone()
        } else {
            let tmp: Vec<f64> = serde_wasm_bindgen::from_value(init_state).unwrap();
            if tmp.len() != self.vars.robot.num_dofs {
                self.vars.init_state.clone()
            } else {
                tmp
            }
        };

        self.vars.reset( starting_config.clone());
    }

    pub fn solve_position(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue) -> Array{
        self.solve_position_helper(pos_goal, quat_goal, tolerance, false)
    }

    pub fn solve_position_relative(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue) -> Array{
        self.solve_position_helper(pos_goal, quat_goal, tolerance, true)
    }

    pub fn solve(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue) -> Array{
        self.solve_position_relative(pos_goal, quat_goal, JsValue::undefined())
    }
}

impl RelaxedIK {

    pub fn solve_position_helper(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue, relative: bool) -> Array{

        let pos_vec: Vec<f64> = serde_wasm_bindgen::from_value(pos_goal).unwrap();
        let quat_vec: Vec<f64> = serde_wasm_bindgen::from_value(quat_goal).unwrap();

        let mut tole_vec = if tolerance.is_null() || tolerance.is_undefined() {
            vec![0.0; self.vars.robot.num_chains * 6]
        } else {
            serde_wasm_bindgen::from_value(tolerance).unwrap()
        };

        let mut pos_goals: Vec<Vector3<f64>> = Vec::new();
        let mut quat_goals: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();

        for i in 0..self.vars.robot.num_chains {
            let pos = Vector3::new(pos_vec[i*3], pos_vec[i*3+1], pos_vec[i*3+2]);
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(quat_vec[i*4], quat_vec[i*4+1], quat_vec[i*4+2], quat_vec[i*4+3]));
            let tole = Vector6::new(tole_vec[i*6], tole_vec[i*6+1], tole_vec[i*6+2], tole_vec[i*6+3], tole_vec[i*6+4], tole_vec[i*6+5]);
            pos_goals.push(pos);
            quat_goals.push(quat);
            tolerances.push(tole);
        }

        let mut out_x = self.vars.xopt.clone();

        for i in 0..self.vars.robot.num_chains {
            if relative {
                self.vars.goal_positions[i] = self.vars.init_ee_positions[i] + pos_goals[i];
                self.vars.goal_quats[i] = quat_goals[i] * self.vars.init_ee_quats[i];
            } else {
                self.vars.goal_positions[i] = pos_goals[i].clone();
                self.vars.goal_quats[i] = quat_goals[i].clone();
            }
            self.vars.tolerances[i] = tolerances[i].clone();
        }

        self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
        self.vars.update(out_x.clone());

        out_x.into_iter().map(JsValue::from).collect()
    }

}