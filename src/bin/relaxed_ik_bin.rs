extern crate relaxed_ik_lib;
use relaxed_ik_lib::relaxed_ik;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use std::{io, thread, time};
use crate::relaxed_ik_lib::utils_rust::file_utils::{*};

fn main() {
    // initilize relaxed ik
    let path_to_src = get_path_to_src();
    let default_path_to_setting = path_to_src +  "configs/settings.yaml";
    let mut relaxed_ik = relaxed_ik::RelaxedIK::load_settings(default_path_to_setting.as_str());

    for i in 0..10{
        for j in 0..relaxed_ik.vars.robot.num_chains {
            // gradually move along the y axis
            relaxed_ik.vars.goal_positions[j] += Vector3::new(0.0, 0.01, 0.0);
        }
        let x = relaxed_ik.solve();
        println!("Joint solutions: {:?}", x);
    }
}
